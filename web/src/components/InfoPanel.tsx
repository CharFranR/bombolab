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
        <svg width={15} height={15} viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth={1.8} strokeLinecap="round" strokeLinejoin="round">
          <circle cx="12" cy="12" r="6" />
          <path d="M12 2v4M12 18v4M2 12h4M18 12h4" />
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
