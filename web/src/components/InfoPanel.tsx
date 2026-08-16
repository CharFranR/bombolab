import type { Mat4, Pose, RobotDef } from '../kinematics/types';
import { useMemo } from 'react';

// P2 (Stage 3C): recibe rawFrames YA calculados por App — no ejecuta su
// propia forwardKinematics (elimina la segunda serialización WASM por
// render). Solo interpreta el resultado compartido.
export default function InfoPanel({ robot, rawFrames }: { robot: RobotDef; rawFrames?: Mat4[] }) {
  const result = useMemo(
    () => {
      if (!rawFrames || rawFrames.length === 0) {
        return { ee: null, tool: null };
      }
      // Aplicar tool transform
      const tool = rawFrames[rawFrames.length - 1];
      const m = mulMat4(tool, robot.toolTransform);
      return { ee: poseFromMat4(rawFrames[rawFrames.length - 1]), tool: poseFromMat4(m) };
    },
    [rawFrames, robot.toolTransform],
  );

  if (!result.tool) return null;

  // EE-1: floating glass card (theme.css `.glass-card`). Data source is the
  // same shared rawFrames FK result — presentation-only restyle: values get
  // the cyan accent, structure/logic unchanged.
  return (
    <div className="glass-card" style={{ padding: '12px 16px' }}>
      <h3 style={{ margin: '0 0 8px', fontSize: 14, fontWeight: 600, color: 'var(--c-text)', textTransform: 'uppercase', letterSpacing: 1, display: 'flex', alignItems: 'center', gap: 8 }}>
        <svg width={16} height={16} viewBox="0 0 24 24" fill="currentColor" aria-hidden="true">
          <path fillRule="evenodd" d="M12 6.8a5.2 5.2 0 1 0 0 10.4 5.2 5.2 0 0 0 0-10.4ZM12 8.4a3.6 3.6 0 1 1 0 7.2 3.6 3.6 0 0 1 0-7.2Z" />
          <rect x="11.15" y="1.8" width="1.7" height="4" rx="0.85" />
          <rect x="11.15" y="18.2" width="1.7" height="4" rx="0.85" />
          <rect x="1.8" y="11.15" width="4" height="1.7" rx="0.85" />
          <rect x="18.2" y="11.15" width="4" height="1.7" rx="0.85" />
        </svg>
        End-Effector
      </h3>

      <div style={{ fontSize: 13, fontFamily: 'var(--font-mono)', color: 'var(--c-gray)', lineHeight: 1.8 }}>
        <div>
          Pos:{' '}
          <span style={{ color: 'var(--c-cyan)' }}>
            ({result.tool.x.toFixed(1)}, {result.tool.y.toFixed(1)}, {result.tool.z.toFixed(1)})
          </span>
        </div>
        <div style={{ marginTop: 4, fontSize: 11, color: 'var(--c-gray)' }}>
          Rot:<br />
          <span style={{ color: 'var(--c-cyan)' }}>
            [{result.tool.rot.slice(0, 3).map(v => v.toFixed(3)).join(', ')}]
          </span>
          <br />
          <span style={{ color: 'var(--c-cyan)' }}>
            [{result.tool.rot.slice(3, 6).map(v => v.toFixed(3)).join(', ')}]
          </span>
          <br />
          <span style={{ color: 'var(--c-cyan)' }}>
            [{result.tool.rot.slice(6, 9).map(v => v.toFixed(3)).join(', ')}]
          </span>
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
