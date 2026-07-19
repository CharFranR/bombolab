import type { Segment } from '../kinematics/types';

const DEG = 180 / Math.PI;

const JOINT_NAMES = ['Base (Yaw)', 'Shoulder', 'Elbow', 'Wrist Roll', 'Wrist Pitch'];

export default function JointControls({
  segments,
  onChange,
}: {
  segments: Segment[];
  onChange: (index: number, qRad: number) => void;
}) {
  return (
    <div style={{ padding: '12px 16px' }}>
      <h3 style={{ margin: '0 0 12px', fontSize: 14, fontWeight: 600, color: '#ccc', textTransform: 'uppercase', letterSpacing: 1 }}>
        Joint Control
      </h3>

      {segments.map((seg, i) => (
        <div key={i} style={{ marginBottom: 12 }}>
          <div style={{ display: 'flex', justifyContent: 'space-between', marginBottom: 4 }}>
            <label style={{ fontSize: 13, color: '#aaa' }}>
              {JOINT_NAMES[i] ?? `J${i + 1}`}
            </label>
            <span style={{ fontSize: 12, color: '#888', fontFamily: 'monospace' }}>
              {(seg.q * DEG).toFixed(1)}°
            </span>
          </div>
          <input
            type="range"
            min={-80}
            max={80}
            value={seg.q * DEG}
            onChange={e => onChange(i, Number(e.target.value) / DEG)}
            style={{
              width: '100%',
              height: 4,
              appearance: 'none',
              background: '#444',
              borderRadius: 2,
              outline: 'none',
              cursor: 'pointer',
            }}
          />
        </div>
      ))}
    </div>
  );
}
