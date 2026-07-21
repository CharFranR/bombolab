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
  gizmoMode,
  stlScaleRef,
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
  const scaleRef = useRef(stlScaleRef);
  scaleRef.current = stlScaleRef;

  // Find target entry for TransformControls
  const targetEntry = useMemo(() => {
    if (!calibrationMode || !calibrationTarget) return null;
    const idx = entries.findIndex((_, i) => STL_META[i].file === calibrationTarget);
    return idx >= 0 ? { entry: entries[idx], index: idx } : null;
  }, [calibrationMode, calibrationTarget, entries]);

  // TransformControls moved/rotated the mesh → save calibration offset
  const handleObjectChange = useCallback(() => {
    if (!targetEntry || !calibrationTarget) return;
    const mesh = targetEntry.entry.mesh;
    const curFrames = framesRef.current;
    const jointIdx = targetEntry.entry.parentJoint >= 0
      ? targetEntry.entry.parentJoint
      : curFrames.length - 1;
    const pose = curFrames[jointIdx];
    if (!pose) return;

    // Build FK world matrix
    const fkWorld = new THREE.Matrix4().compose(
      new THREE.Vector3(...pose.pos),
      new THREE.Quaternion(...pose.quat),
      new THREE.Vector3(1, 1, 1),
    );

    // Actual mesh world matrix after TransformControls manipulation
    mesh.updateMatrixWorld();
    const meshWorld = mesh.matrixWorld.clone();

    // Calibration = FK⁻¹ × meshWorld (offset from FK in FK local space)
    const cal = fkWorld.clone().invert().multiply(meshWorld);
    overridesRef.current?.current.set(calibrationTarget, cal);
  }, [targetEntry, calibrationTarget]);

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
      const isTarget = targetEntry && targetEntry.index === i;

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

      // Get calibration: override > config > identity
      const calConfig = configRef.current?.current.get(file);
      const calOverride = overridesRef.current?.current.get(file);
      const cal = calOverride ?? calConfig ?? new THREE.Matrix4().identity();
      entry.calibrationTransform.copy(cal);

      const calPos = new THREE.Vector3();
      const calQuat = new THREE.Quaternion();
      cal.decompose(calPos, calQuat, new THREE.Vector3());

      if (isTarget) {
        // Target: FK × scale × calibration as position+quaternion for TransformControls
        const fkMatrix = new THREE.Matrix4().compose(tempPos.clone(), tempQuat.clone(), new THREE.Vector3(1, 1, 1));
        const s = scaleRef.current?.current ?? 1;
        fkMatrix.multiply(new THREE.Matrix4().makeScale(s, s, s));
        fkMatrix.multiply(cal);
        const finalPos = new THREE.Vector3();
        const finalQuat = new THREE.Quaternion();
        const finalScale = new THREE.Vector3();
        fkMatrix.decompose(finalPos, finalQuat, finalScale);
        entry.mesh.position.copy(finalPos);
        entry.mesh.quaternion.copy(finalQuat);
        entry.mesh.scale.copy(finalScale);
        entry.mesh.matrixAutoUpdate = true;
      } else {
        // Non-target: matrix pipeline
        world.compose(tempPos, tempQuat, tempScale);
        const s = scaleRef.current?.current ?? 1;
        world.multiply(new THREE.Matrix4().makeScale(s, s, s));
        world.multiply(cal);
        // Animate gripper jaws
        if (entry.isGripper) {
          const jawOpen = (1 - curGripper / 100) * 10;
          jawM.makeTranslation(0, entry.jawDirection * jawOpen, 0);
          world.multiply(jawM);
        }
        entry.mesh.matrix.copy(world);
        entry.mesh.matrixAutoUpdate = false;
        entry.mesh.matrixWorldNeedsUpdate = true;
      }
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
        const isTarget = targetEntry && targetEntry.index === i;
        const el = <primitive key={i} object={entry.mesh} />;
        if (isTarget) {
          return (
            <TransformControls
              key={i}
              object={entry.mesh}
              mode={gizmoMode ?? 'translate'}
              onObjectChange={handleObjectChange}
            >
              {el}
            </TransformControls>
          );
        }
        return el;
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
