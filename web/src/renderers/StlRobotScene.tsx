import { useMemo, useRef, useCallback } from 'react';
import { useFrame, useLoader } from '@react-three/fiber';
import { TransformControls } from '@react-three/drei';
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js';
import * as THREE from 'three';
import type { RobotRendererProps, VisualLink } from './types';
import { ALL_STL_FILES, STL_META } from './stlMapping';
import DebugAxes from './debugAxes';

// ─── STL paths ──────────────────────────────────────────────────────────────

const STL_BASE = '/stl/';
const STL_URLS = ALL_STL_FILES.map((f) => `${STL_BASE}${f}`);

// ─── Helpers ────────────────────────────────────────────────────────────────

interface MeshEntry extends VisualLink {
  isGripper: boolean;
  jawDirection: number;
}

function buildEntries(geometries: THREE.BufferGeometry[]): MeshEntry[] {
  const material = new THREE.MeshStandardMaterial({
    color: '#bbbbcc',
    roughness: 0.5,
    metalness: 0.3,
  });
  return geometries.map((geo, i) => {
    const meta = STL_META[i];
    console.log(`[StlRobotScene] Loaded ${meta.file}`);
    const mesh = new THREE.Mesh(geo, material);
    mesh.matrixAutoUpdate = false;
    mesh.matrix.identity();
    return {
      mesh,
      parentJoint: meta.parentJoint,
      calibrationTransform: new THREE.Matrix4().identity(),
      isGripper: meta.jawDirection !== 0,
      jawDirection: meta.jawDirection,
    };
  });
}

// ─── Renderer ───────────────────────────────────────────────────────────────

export default function StlRobotScene({
  frames,
  gripper,
  debugToggles,
  calibrationConfigRef,
  calibrationOverridesRef,
  calibrationTarget,
  calibrationMode,
  calibrationVersion,
  onCalibrationChange,
}: RobotRendererProps) {
  const geometries = useLoader(STLLoader, STL_URLS);

  // Build meshes once after geometries load
  const entries = useMemo(() => buildEntries(geometries), [geometries]);

  // Refs so useFrame always reads the latest props (avoids stale closure)
  const entriesRef = useRef(entries);
  entriesRef.current = entries;
  const framesRef = useRef(frames);
  framesRef.current = frames;
  const gripperRef = useRef(gripper);
  gripperRef.current = gripper;
  const firstFrameRef = useRef(true);
  const configRef = useRef(calibrationConfigRef);
  configRef.current = calibrationConfigRef;
  const overridesRef = useRef(calibrationOverridesRef);
  overridesRef.current = calibrationOverridesRef;
  const targetRef = useRef(calibrationTarget);
  targetRef.current = calibrationTarget;
  const modeRef = useRef(calibrationMode);
  modeRef.current = calibrationMode;

  // Find target entry index for TransformControls
  const targetIndex = useMemo(() => {
    if (!calibrationMode || !calibrationTarget) return -1;
    return entries.findIndex((_, i) => STL_META[i].file === calibrationTarget);
  }, [calibrationMode, calibrationTarget, entries]);

  // Callback: TransformControls moved the mesh → save new offset
  const handleObjectChange = useCallback(() => {
    if (targetIndex < 0) return;
    const entry = entriesRef.current[targetIndex];
    if (!entry || !calibrationTarget) return;

    // Read mesh world position
    const meshPos = new THREE.Vector3();
    entry.mesh.getWorldPosition(meshPos);

    // Get FK joint position for this mesh
    const curFrames = framesRef.current;
    const jointIdx = entry.parentJoint >= 0 ? entry.parentJoint : curFrames.length - 1;
    const fkPos = new THREE.Vector3(...curFrames[jointIdx].pos);

    // Calibration offset = worldPos - fkWorldPos
    const offset = meshPos.clone().sub(fkPos);
    const cal = new THREE.Matrix4().makeTranslation(offset.x, offset.y, offset.z);
    overridesRef.current?.current.set(calibrationTarget, cal);
    onCalibrationChange?.();
  }, [targetIndex, calibrationTarget, onCalibrationChange]);

  // ─── Per-frame mesh positioning ─────────────────────────────────────────
  useFrame(() => {
    const curFrames = framesRef.current;
    const curGripper = gripperRef.current;
    const isFirst = firstFrameRef.current;
    if (isFirst) firstFrameRef.current = false;

    // Reusable temp objects (no allocation per mesh)
    const tempPos = new THREE.Vector3();
    const tempQuat = new THREE.Quaternion();
    const tempScale = new THREE.Vector3(1, 1, 1);
    const world = new THREE.Matrix4();
    const jawM = new THREE.Matrix4();

    entriesRef.current.forEach((entry, i) => {
      const file = STL_META[i].file;
      const isTarget = calibrationMode && calibrationTarget === file;

      // Determine parent frame index (-1 → tool-tip = last frame)
      const jointIdx = entry.parentJoint >= 0
        ? entry.parentJoint
        : curFrames.length - 1;
      const pose = curFrames[jointIdx];
      if (!pose) return;

      // Diagnostic: log first-frame transform application
      if (isFirst) {
        const label = entry.parentJoint >= 0 ? `Joint ${entry.parentJoint}` : 'Tool-tip';
        console.log(`[StlRobotScene] Applying transform: ${label} -> ${file}`);
      }

      // Build FK world position
      tempPos.set(...pose.pos);
      tempQuat.set(...pose.quat);

      // Get calibration transform: override > config > identity
      const calConfig = configRef.current?.current.get(file);
      const calOverride = overridesRef.current?.current.get(file);
      const cal = calOverride ?? calConfig ?? new THREE.Matrix4().identity();
      const calPos = new THREE.Vector3();
      cal.decompose(calPos, new THREE.Quaternion(), new THREE.Vector3());

      if (isTarget) {
        // Target mesh: position+quaternion for TransformControls compatibility
        // TransformControls modifies position/quaternion on drag;
        // useFrame reapplies FK+calibration each frame so the gizmo stays in sync
        entry.mesh.position.copy(tempPos).add(calPos);
        entry.mesh.quaternion.copy(tempQuat);
        entry.mesh.matrixAutoUpdate = true;
        entry.mesh.visible = true;
        return;
      }

      // Non-target mesh: existing matrix-based pipeline
      world.compose(tempPos, tempQuat, tempScale);
      world.multiply(cal);
      entry.calibrationTransform.copy(cal);

      // Animate gripper jaws
      if (entry.isGripper) {
        const jawOpen = (1 - curGripper / 100) * 10;
        jawM.makeTranslation(0, entry.jawDirection * jawOpen, 0);
        world.multiply(jawM);
      }

      entry.mesh.matrix.copy(world);
      entry.mesh.matrixAutoUpdate = false;
      entry.mesh.matrixWorldNeedsUpdate = true;
      entry.mesh.visible = true;
    });
  });

  // Default toggles (all off) when none provided
  const toggles = debugToggles ?? {
    showJointFrames: false,
    showStlOrigins: false,
    showCalibrationAxes: false,
  };

  return (
    <group>
      {entries.map((entry, i) => {
        const isTarget = calibrationMode && calibrationTarget === STL_META[i].file;
        if (isTarget && targetIndex >= 0) {
          return (
            <TransformControls
              key={i}
              object={entry.mesh}
              mode="translate"
              size={0.7}
              onObjectChange={handleObjectChange}
            >
              <primitive object={entry.mesh} />
            </TransformControls>
          );
        }
        return <primitive key={i} object={entry.mesh} />;
      })}
      <DebugAxes
        framesRef={framesRef}
        stlMeta={STL_META}
        calibrationRef={(calibrationConfigRef ?? { current: new Map() }) as React.MutableRefObject<Map<string, THREE.Matrix4>>}
        toggles={toggles}
      />
    </group>
  );
}
