import { useMemo, useRef } from 'react';
import { useFrame } from '@react-three/fiber';
import * as THREE from 'three';
import type { DebugToggles, FramePose } from './types';
import type { StlMeta } from './stlMapping';

// ─── Props ───────────────────────────────────────────────────────────────────

interface DebugAxesProps {
  framesRef: React.MutableRefObject<FramePose[]>;
  stlMeta: StlMeta[];
  calibrationRef: React.MutableRefObject<Map<string, THREE.Matrix4>>;
  toggles: DebugToggles;
  scaleRef?: React.MutableRefObject<number>;
}

// ─── Constants ───────────────────────────────────────────────────────────────

const JOINT_AXIS_SIZE = 30;
const STL_AXIS_SIZE = 15;
const NUM_FK_FRAMES = 6;    // world + 5 joints (tool-tip is appended separately)
const NUM_STL_MESHES = 11;  // one per STL file

// ─── Component ───────────────────────────────────────────────────────────────

export default function DebugAxes({
  framesRef,
  stlMeta,
  calibrationRef,
  toggles,
  scaleRef,
}: DebugAxesProps) {
  // Create 28 AxesHelper objects once — never re-created after mount
  const helpers = useMemo(() => {
    const items: {
      helper: THREE.AxesHelper;
      type: 'joint' | 'stlOrigin' | 'calibration';
      idx: number; // index into either FK frames (joint) or STL_META (origin/cal)
    }[] = [];

    // 6 joint frame axes (size 30) — one per FK frame
    for (let i = 0; i < NUM_FK_FRAMES; i++) {
      items.push({ helper: new THREE.AxesHelper(JOINT_AXIS_SIZE), type: 'joint', idx: i });
    }

    // 11 STL origin axes (size 15) — at FK frame position before calibration
    for (let i = 0; i < NUM_STL_MESHES; i++) {
      items.push({ helper: new THREE.AxesHelper(STL_AXIS_SIZE), type: 'stlOrigin', idx: i });
    }

    // 11 calibration axes (size 15) — at FK frame position + calibration offset
    for (let i = 0; i < NUM_STL_MESHES; i++) {
      items.push({ helper: new THREE.AxesHelper(STL_AXIS_SIZE), type: 'calibration', idx: i });
    }

    return items;
  }, []);

  // Refs so useFrame reads latest values without re-creating helpers
  const helpersRef = useRef(helpers);
  helpersRef.current = helpers;

  // Temporary objects (no allocation per frame per mesh)
  const tempPos = useMemo(() => new THREE.Vector3(), []);
  const tempQuat = useMemo(() => new THREE.Quaternion(), []);
  const tempScale = useMemo(() => new THREE.Vector3(1, 1, 1), []);
  const tempWorld = useMemo(() => new THREE.Matrix4(), []);

  useFrame(() => {
    const curFrames = framesRef.current;
    const calMap = calibrationRef.current;
    const { showJointFrames, showStlOrigins, showCalibrationAxes } = toggles;
    const anyOn = showJointFrames || showStlOrigins || showCalibrationAxes;

    // Fast path: all toggles off → zero draw calls
    if (!anyOn) {
      for (const item of helpersRef.current) {
        item.helper.visible = false;
      }
      return;
    }

    for (const item of helpersRef.current) {
      const { helper, type, idx } = item;

      switch (type) {
        case 'joint': {
          const pose = curFrames[idx];
          if (!pose) { helper.visible = false; break; }
          tempPos.set(...pose.pos);
          tempQuat.set(...pose.quat);
          helper.position.copy(tempPos);
          helper.quaternion.copy(tempQuat);
          helper.visible = showJointFrames;
          break;
        }

        case 'stlOrigin': {
          const meta = stlMeta[idx];
          const jointIdx = meta.parentJoint >= 0
            ? meta.parentJoint
            : curFrames.length - 1;
          const pose = curFrames[jointIdx];
          if (!pose) { helper.visible = false; break; }
          tempPos.set(...pose.pos);
          tempQuat.set(...pose.quat);
          helper.position.copy(tempPos);
          helper.quaternion.copy(tempQuat);
          helper.visible = showStlOrigins;
          break;
        }

        case 'calibration': {
          const meta = stlMeta[idx];
          const jointIdx = meta.parentJoint >= 0
            ? meta.parentJoint
            : curFrames.length - 1;
          const pose = curFrames[jointIdx];
          if (!pose) { helper.visible = false; break; }

          // Compose world = FK pose × scale × calibration offset — must
          // match the mesh pipeline (StlRobotScene: FK × jaw × S × cal),
          // otherwise axes drift off the meshes whenever stlScale ≠ 1.
          tempPos.set(...pose.pos);
          tempQuat.set(...pose.quat);
          tempWorld.compose(tempPos, tempQuat, tempScale);

          const s = scaleRef?.current ?? 1;
          tempWorld.multiply(new THREE.Matrix4().makeScale(s, s, s));

          const cal = calMap.get(meta.file);
          if (cal) {
            tempWorld.multiply(cal);
          }

          tempWorld.decompose(helper.position, helper.quaternion, tempScale);
          helper.visible = showCalibrationAxes;
          break;
        }
      }
    }
  });

  return (
    <group>
      {helpers.map((item, i) => (
        <primitive key={i} object={item.helper} />
      ))}
    </group>
  );
}
