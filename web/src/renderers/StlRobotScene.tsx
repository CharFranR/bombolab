import { useEffect, useMemo, useRef } from 'react';
import { useFrame, useLoader } from '@react-three/fiber';
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js';
import * as THREE from 'three';
import type { RobotRendererProps, VisualLink } from './types';
import { ALL_STL_FILES, STL_META } from './stlMapping';

// ─── Calibration config shape ────────────────────────────────────────────────

interface CalibrationEntry {
  filename: string;
  translation: [number, number, number];
  rotation: [number, number, number, number];
}

interface CalibrationConfig {
  version: number;
  entries: CalibrationEntry[];
}

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

export default function StlRobotScene({ frames, gripper }: RobotRendererProps) {
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

  // ─── Calibration config loader ───────────────────────────────────────────
  const calibrationRef = useRef<Map<string, THREE.Matrix4>>(new Map());

  useEffect(() => {
    let cancelled = false;
    fetch('/calibration.json')
      .then((res) => {
        if (!res.ok) {
          throw new Error(`HTTP ${res.status}: ${res.statusText}`);
        }
        return res.json();
      })
      .then((config: CalibrationConfig) => {
        if (cancelled) return;

        if (!config || config.version !== 1) {
          console.warn('[StlRobotScene] calibration.json: invalid or missing version field — using identity');
          return;
        }

        const map = new Map<string, THREE.Matrix4>();
        for (const entry of config.entries) {
          const [tx, ty, tz] = entry.translation;
          const [rx, ry, rz, rw] = entry.rotation;
          const m = new THREE.Matrix4().compose(
            new THREE.Vector3(tx, ty, tz),
            new THREE.Quaternion(rx, ry, rz, rw),
            new THREE.Vector3(1, 1, 1),
          );
          map.set(entry.filename, m);
        }

        calibrationRef.current = map;
        console.log(`[StlRobotScene] Loaded calibration.json — ${map.size} entries`);
      })
      .catch((err) => {
        if (cancelled) return;
        console.warn('[StlRobotScene] Failed to load calibration.json:', err.message);
        // calibrationRef stays as empty map → identity fallback
      });
    return () => { cancelled = true; };
  }, []);

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
      // Determine parent frame index (-1 → tool-tip = last frame)
      const jointIdx = entry.parentJoint >= 0
        ? entry.parentJoint
        : curFrames.length - 1;
      const pose = curFrames[jointIdx];
      if (!pose) return;

      // Diagnostic: log first-frame transform application
      if (isFirst) {
        const label = entry.parentJoint >= 0 ? `Joint ${entry.parentJoint}` : 'Tool-tip';
        console.log(`[StlRobotScene] Applying transform: ${label} -> ${STL_META[i].file}`);
      }

      // Build world transform from FK frame pose
      tempPos.set(...pose.pos);
      tempQuat.set(...pose.quat);
      world.compose(tempPos, tempQuat, tempScale);

      // Apply calibration transform from loaded config (identity if missing)
      const cal = calibrationRef.current.get(STL_META[i].file) ?? new THREE.Matrix4().identity();
      entry.calibrationTransform.copy(cal);
      world.multiply(entry.calibrationTransform);

      // Animate gripper jaws: translate along local Y
      if (entry.isGripper) {
        const jawOpen = (1 - curGripper / 100) * 10;
        jawM.makeTranslation(0, entry.jawDirection * jawOpen, 0);
        world.multiply(jawM);
      }

      // Push to mesh (matrixAutoUpdate = false → Three.js uses matrix directly)
      entry.mesh.matrix.copy(world);
      entry.mesh.matrixAutoUpdate = false;
      entry.mesh.matrixWorldNeedsUpdate = true;
    });
  });

  return (
    <group>
      {entries.map((entry, i) => (
        <primitive key={i} object={entry.mesh} />
      ))}
    </group>
  );
}
