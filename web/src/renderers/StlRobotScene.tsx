import { useMemo, useRef, useCallback } from 'react';
import { useFrame, useLoader } from '@react-three/fiber';
import { TransformControls } from '@react-three/drei';
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js';
import * as THREE from 'three';
import type { RobotRendererProps, VisualLink } from './types';
import { ALL_STL_FILES, STL_META } from './stlMapping';
import DebugAxes from './debugAxes';
import IkTarget from '../components/IkTarget';
import CandidatesOverlay from '../calibration/CandidatesOverlay';

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
  workspacePoints,
  tracePath,
  debugToggles,
  calibrationConfigRef,
  calibrationOverridesRef,
  calibrationTarget,
  calibrationMode,
  calibrationVersion,
  onCalibrationChange,
  gizmoMode,
  stlScaleRef,
  ikTarget,
  onIkTargetChange,
  onDragStart,
  onDragEnd,
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

  // P3 (Stage 3C): Float32Array del workspace memoizado — se reconstruye
  // solo cuando cambian los puntos, no en cada render de React.
  const workspaceArray = useMemo(
    () => (workspacePoints && workspacePoints.length > 0
      ? new Float32Array(workspacePoints.flat())
      : undefined),
    [workspacePoints],
  );

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
    // Gripper jaws animate in FK local space BEFORE calibration; the
    // calibration must NOT absorb the jaw offset. Replicate the exact
    // pipeline the non-target branch uses (FK × jaw × S × cal) so
    // meshWorld and fkWorld agree on the jaw term.
    const jawOpen = (1 - gripperRef.current / 100) * 10;
    if (targetEntry.entry.isGripper && jawOpen !== 0) {
      fkWorld.multiply(
        new THREE.Matrix4().makeTranslation(0, targetEntry.entry.jawDirection * jawOpen, 0),
      );
    }

    // Actual mesh world matrix after TransformControls manipulation
    mesh.updateMatrixWorld();
    const meshWorld = mesh.matrixWorld.clone();

    // Calibration = (FK × scale)⁻¹ × meshWorld  (exclude scale from cal)
    const s = scaleRef.current?.current ?? 1;
    const fkScaled = fkWorld.clone().multiply(new THREE.Matrix4().makeScale(s, s, s));
    const cal = fkScaled.clone().invert().multiply(meshWorld);
    // Remove scale component from calibration (scale is applied separately)
    const calPos = new THREE.Vector3();
    const calQuat = new THREE.Quaternion();
    cal.decompose(calPos, calQuat, new THREE.Vector3());
    const calClean = new THREE.Matrix4().compose(calPos, calQuat, new THREE.Vector3(1, 1, 1));
    overridesRef.current?.current.set(calibrationTarget, calClean);
    // Notify the panel so its numeric inputs re-read the new override.
    // Without this, gizmo drags write overridesRef but `version` never
    // bumps → the panel shows stale values and the next step-button click
    // silently discards the gizmo calibration (lost updates).
    onCalibrationChange?.();
  }, [targetEntry, calibrationTarget, onCalibrationChange]);

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
    const scaleM = new THREE.Matrix4();
    // Hoisted temporaries for the calibration decompose + target branch —
    // created once per frame instead of once per mesh (43 allocs/frame → 0
    // in the common non-target path). P1 (Stage 3C perf audit).
    const calPos = new THREE.Vector3();
    const calQuat = new THREE.Quaternion();
    const calScale = new THREE.Vector3();
    const calDecomp = new THREE.Vector3();
    const fkMatrix = new THREE.Matrix4();
    const jawM2 = new THREE.Matrix4();
    const scaleM2 = new THREE.Matrix4();
    const finalPos = new THREE.Vector3();
    const finalQuat = new THREE.Quaternion();
    const finalScale = new THREE.Vector3(1, 1, 1);

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

      // Reuse hoisted temporaries (no allocation per mesh)
      cal.decompose(calPos, calQuat, calDecomp);

      if (isTarget) {
        // Target: FK × scale × calibration as position+quaternion for TransformControls
        fkMatrix.compose(tempPos, tempQuat, tempScale);
        // Jaw animation in FK local space (before scale+calibration)
        if (entry.isGripper) {
          const jawOpen = (1 - curGripper / 100) * 10;
          jawM2.makeTranslation(0, entry.jawDirection * jawOpen, 0);
          fkMatrix.multiply(jawM2);
        }
        const s = scaleRef.current?.current ?? 1;
        scaleM2.makeScale(s, s, s);
        fkMatrix.multiply(scaleM2);
        fkMatrix.multiply(cal);
        fkMatrix.decompose(finalPos, finalQuat, finalScale);
        entry.mesh.position.copy(finalPos);
        entry.mesh.quaternion.copy(finalQuat);
        entry.mesh.scale.copy(finalScale);
        entry.mesh.matrixAutoUpdate = true;
      } else {
        // Non-target: matrix pipeline
        world.compose(tempPos, tempQuat, tempScale);
        // Animate gripper jaws BEFORE calibration (FK local space = same as SimpleRobotScene)
        if (entry.isGripper) {
          const jawOpen = (1 - curGripper / 100) * 10;
          jawM.makeTranslation(0, entry.jawDirection * jawOpen, 0);
          world.multiply(jawM);
        }
        const s = scaleRef.current?.current ?? 1;
        scaleM.makeScale(s, s, s);
        world.multiply(scaleM);
        world.multiply(cal);
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
        scaleRef={stlScaleRef}
      />
      {/* Calibrator candidates overlay (debug) */}
      {toggles.showCandidates && (
        <CandidatesOverlay
          frames={frames}
          gripper={gripper}
          calibrationConfigRef={calibrationConfigRef}
          calibrationOverridesRef={calibrationOverridesRef}
          stlScaleRef={stlScaleRef}
        />
      )}
      {/* Workspace point cloud — Float32Array memoizado (P3, Stage 3C):
          antes se reconstruía por render (24KB + flat() por interacción) */}
      {workspacePoints && workspacePoints.length > 0 && (
        <points>
          <bufferGeometry>
            <bufferAttribute
              attach="attributes-position"
              count={workspacePoints.length}
              array={workspaceArray}
              itemSize={3}
            />
          </bufferGeometry>
          <pointsMaterial size={5} color="#66aaff" transparent opacity={0.35} depthWrite={false} />
        </points>
      )}
      {/* Preview of the selected trajectory shape (drawing plane z) */}
      {tracePath && tracePath.length > 1 && (
        <line>
          <bufferGeometry>
            <bufferAttribute
              attach="attributes-position"
              count={tracePath.length}
              array={new Float32Array(tracePath.flat())}
              itemSize={3}
            />
          </bufferGeometry>
          <lineBasicMaterial color="#ff8866" linewidth={2} />
        </line>
      )}
      {/* IK target */}
      {ikTarget && onIkTargetChange && (
        <IkTarget position={ikTarget} onChange={onIkTargetChange} onDragStart={onDragStart} onDragEnd={onDragEnd} />
      )}
    </group>
  );
}
