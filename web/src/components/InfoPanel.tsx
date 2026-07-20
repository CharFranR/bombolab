import type { Mat4, Pose, RobotDef } from '../kinematics/types';
import { forwardKinematics } from '../wasm';
import { useMemo } from 'react';

export default function InfoPanel({ robot }: { robot: RobotDef }) {
  const result = useMemo(
    () => {
      const fk = forwardKinematics(robot.segments, robot.baseTransform);
      const frames = fk.frames;
      // Aplicar tool transform
      const tool = fk.frames[fk.frames.length - 1];
      const toolMat: Mat4 = [
        1, 0, 0, robot.toolTransform[0],
        0, 1, 0, robot.toolTransform[1],
        0, 0, 1, robot.toolTransform[2],
        0, 0, 0, 1,
      ];
      const m = mulMat4(tool, toolMat);
      return { ee: poseFromMat4(frames[frames.length - 1]), tool: poseFromMat4(m) };
    },
    [robot.segments, robot.baseTransform, robot.toolTransform],
  );

  return (
    <div style={{ padding: '12px 16px', borderTop: '1px solid #333' }}>
      <h3 style={{ margin: '0 0 8px', fontSize: 14, fontWeight: 600, color: '#ccc', textTransform: 'uppercase', letterSpacing: 1 }}>
        End-Effector
      </h3>

      <div style={{ fontSize: 13, fontFamily: 'monospace', color: '#aaa', lineHeight: 1.8 }}>
        <div>Pos: ({result.tool.x.toFixed(1)}, {result.tool.y.toFixed(1)}, {result.tool.z.toFixed(1)})</div>
        <div style={{ marginTop: 4, fontSize: 11, color: '#666' }}>
          Rot:<br />
          [{result.tool.rot.slice(0, 3).map(v => v.toFixed(3)).join(', ')}]<br />
          [{result.tool.rot.slice(3, 6).map(v => v.toFixed(3)).join(', ')}]<br />
          [{result.tool.rot.slice(6, 9).map(v => v.toFixed(3)).join(', ')}]
        </div>
      </div>
    </div>
  );
}

function poseFromMat4(m: Mat4): Pose {
  return {
    x: m[3], y: m[7], z: m[11],
    rot: [m[0], m[1], m[2], m[4], m[5], m[6], m[8], m[9], m[10]],
  };
}

function mulMat4(a: Mat4, b: Mat4): Mat4 {
  const m = (r: number, c: number) =>
    a[r * 4 + 0] * b[0 * 4 + c] +
    a[r * 4 + 1] * b[1 * 4 + c] +
    a[r * 4 + 2] * b[2 * 4 + c] +
    a[r * 4 + 3] * b[3 * 4 + c];
  return [
    m(0,0), m(0,1), m(0,2), m(0,3),
    m(1,0), m(1,1), m(1,2), m(1,3),
    m(2,0), m(2,1), m(2,2), m(2,3),
    m(3,0), m(3,1), m(3,2), m(3,3),
  ];
}
