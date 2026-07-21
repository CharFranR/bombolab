import { useMemo, useRef } from 'react';
import { useFrame, useLoader } from '@react-three/fiber';
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js';
import * as THREE from 'three';
import type { RobotRendererProps, VisualLink } from './types';
import { ALL_STL_FILES, STL_META } from './stlMapping';

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
    const mesh = new THREE.Mesh(geo, material);
    mesh.matrixAutoUpdate = false;
    mesh.matrix.identity();
    return {
      mesh,
      parentJoint: STL_META[i].parentJoint,
      calibrationTransform: new THREE.Matrix4().identity(),
      isGripper: STL_META[i].jawDirection !== 0,
      jawDirection: STL_META[i].jawDirection,
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

  // ─── Per-frame mesh positioning ─────────────────────────────────────────
  useFrame(() => {
    const curFrames = framesRef.current;
    const curGripper = gripperRef.current;

    // Reusable temp objects (no allocation per mesh)
    const tempPos = new THREE.Vector3();
    const tempQuat = new THREE.Quaternion();
    const tempScale = new THREE.Vector3(1, 1, 1);
    const world = new THREE.Matrix4();
    const jawM = new THREE.Matrix4();

    for (const entry of entriesRef.current) {
      // Determine parent frame index (-1 → tool-tip = last frame)
      const jointIdx = entry.parentJoint >= 0
        ? entry.parentJoint
        : curFrames.length - 1;
      const pose = curFrames[jointIdx];
      if (!pose) continue;

      // Build world transform from FK frame pose
      tempPos.set(...pose.pos);
      tempQuat.set(...pose.quat);
      world.compose(tempPos, tempQuat, tempScale);

      // Apply calibration transform (identity for now)
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
    }
  });

  return (
    <group>
      {entries.map((entry, i) => (
        <primitive key={i} object={entry.mesh} />
      ))}
    </group>
  );
}
