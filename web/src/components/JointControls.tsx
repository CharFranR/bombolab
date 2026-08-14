import type { Segment } from '../kinematics/types';

const DEG = 180 / Math.PI;

const JOINT_NAMES = ['Base (Yaw)', 'Shoulder', 'Elbow', 'Wrist Roll', 'Wrist Pitch'];

// Filled fraction of a slider range, as a CSS % for the --fill custom prop.
function fillPct(value: number, min: number, max: number): string {
  if (!(max > min)) return '0%';
  const pct = ((value - min) / (max - min)) * 100;
  return `${Math.min(100, Math.max(0, pct))}%`;
}

export default function JointControls({
  segments,
  gripper,
  onGripperChange,
  onChange,
  disabled = false,
}: {
  segments: Segment[];
  gripper: number;
  onGripperChange: (v: number) => void;
  onChange: (index: number, qRad: number) => void;
  disabled?: boolean;
}) {
  return (
    <div className="glass-card" style={{ padding: '12px 16px' }}>
      <h3 style={{ margin: '0 0 12px', fontSize: 14, fontWeight: 600, color: 'var(--c-text)', textTransform: 'uppercase', letterSpacing: 1 }}>
        Joint Control
      </h3>

      {segments.map((seg, i) => {
        const min = seg.q_min != null ? Math.round(seg.q_min * DEG) : -80;
        const max = seg.q_max != null ? Math.round(seg.q_max * DEG) : 80;
        return (
          <div key={i} style={{ marginBottom: 12 }}>
            <div style={{ display: 'flex', justifyContent: 'space-between', marginBottom: 4 }}>
              <label style={{ fontSize: 13, color: 'var(--c-text-dim)' }}>
                {JOINT_NAMES[i] ?? `J${i + 1}`}
              </label>
              <span style={{ fontSize: 12, color: 'var(--c-gray)', fontFamily: 'monospace' }}>
                {(seg.q * DEG).toFixed(1)}°
              </span>
            </div>
            <input
              type="range"
              min={min}
              max={max}
              value={seg.q * DEG}
              disabled={disabled}
              onChange={e => onChange(i, Number(e.target.value) / DEG)}
              className="jnt-slider"
              style={{ ['--fill' as string]: fillPct(seg.q * DEG, min, max) } as React.CSSProperties}
            />
          </div>
        );
      })}

      {/* Gripper */}
      <div style={{ marginTop: 16, paddingTop: 12, borderTop: '1px solid var(--border)' }}>
        <div style={{ display: 'flex', justifyContent: 'space-between', marginBottom: 4 }}>
          <label style={{ fontSize: 13, color: 'var(--c-text-dim)' }}>Gripper</label>
          <span style={{ fontSize: 12, color: 'var(--c-gray)', fontFamily: 'monospace' }}>
            {gripper}%
          </span>
        </div>
        <input
          type="range"
          min={0}
          max={100}
          value={gripper}
          disabled={disabled}
          onChange={e => onGripperChange(Number(e.target.value))}
          className="jnt-slider"
          style={{ ['--fill' as string]: fillPct(gripper, 0, 100) } as React.CSSProperties}
        />
      </div>
    </div>
  );
}
