import { DEFAULT_GAMEPAD_MAPPING, type GamepadCommand } from '../gamepad';

const DEG = 180 / Math.PI;

const JOINT_LABELS = ['Base (Yaw)', 'Shoulder', 'Elbow', 'Wrist Roll', 'Wrist Pitch'];

const AXIS_LABELS: Record<number, string> = {
  0: 'L-Stick X',
  1: 'L-Stick Y',
  2: 'R-Stick X',
  3: 'R-Stick Y',
  6: 'LT',
  7: 'RT',
};

const BUTTON_LABELS: Record<number, string> = {
  0: 'A (open)',
  1: 'B (close)',
};

export default function GamepadControls({
  enabled,
  onEnabledChange,
  apiAvailable,
  gamepadId,
  commands,
  active,
}: {
  enabled: boolean;
  onEnabledChange: (v: boolean) => void;
  apiAvailable: boolean;
  gamepadId: string | null;
  commands: GamepadCommand | null;
  active: boolean;
}) {
  const cfg = DEFAULT_GAMEPAD_MAPPING;
  return (
    <div style={{ padding: '12px 16px', borderTop: '1px solid #333' }}>
      <h3 style={{ margin: '0 0 12px', fontSize: 14, fontWeight: 600, color: '#ccc', textTransform: 'uppercase', letterSpacing: 1 }}>
        Gamepad
      </h3>

      {!apiAvailable && (
        <div style={{ fontSize: 12, color: '#e88', marginBottom: 8 }}>
          Gamepad API not supported by this browser (try Chrome or Edge).
        </div>
      )}

      <label style={{ display: 'flex', alignItems: 'center', gap: 6, fontSize: 13, color: '#aaa', marginBottom: 8, cursor: 'pointer' }}>
        <input
          type="checkbox"
          checked={enabled}
          disabled={!apiAvailable}
          onChange={(e) => onEnabledChange(e.target.checked)}
        />
        Enable gamepad control
      </label>

      <div style={{ fontSize: 12, marginBottom: 4 }}>
        {gamepadId
          ? <span style={{ color: '#8d8' }}>{gamepadId}</span>
          : <span style={{ color: '#888' }}>No gamepad detected</span>}
      </div>
      {enabled && !gamepadId && apiAvailable && (
        <div style={{ fontSize: 11, color: '#888', marginBottom: 4 }}>
          Press any button on your controller to connect.
        </div>
      )}
      {enabled && gamepadId && (
        <div style={{ fontSize: 11, marginBottom: 8, color: active ? '#8d8' : '#cc8' }}>
          {active ? 'ACTIVE — sticks drive the arm' : 'PAUSED — drawing/IK/playback in progress'}
        </div>
      )}

      {/* Live readout */}
      <div style={{ marginTop: 8, paddingTop: 8, borderTop: '1px solid #333' }}>
        <div style={{ fontSize: 11, color: '#888', textTransform: 'uppercase', letterSpacing: 1, marginBottom: 6 }}>
          Output
        </div>
        {commands ? (
          <>
            {commands.jointVelocities.map((v, i) => (
              <div key={i} style={{ display: 'flex', justifyContent: 'space-between', fontSize: 12 }}>
                <span style={{ color: '#aaa' }}>{JOINT_LABELS[i]}</span>
                <span style={{ color: '#888', fontFamily: 'monospace' }}>{(v * DEG).toFixed(1)}°/s</span>
              </div>
            ))}
            <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: 12 }}>
              <span style={{ color: '#aaa' }}>Gripper</span>
              <span style={{ color: '#888', fontFamily: 'monospace' }}>{commands.gripperDeltaPct.toFixed(1)}%/s</span>
            </div>
          </>
        ) : (
          <div style={{ fontSize: 12, color: '#555' }}>—</div>
        )}
      </div>

      {/* Mapping legend */}
      <div style={{ marginTop: 8, paddingTop: 8, borderTop: '1px solid #333' }}>
        <div style={{ fontSize: 11, color: '#888', textTransform: 'uppercase', letterSpacing: 1, marginBottom: 6 }}>
          Mapping
        </div>
        {cfg.analog.map((src, i) => {
          const label = src.kind === 'axis'
            ? (AXIS_LABELS[src.axis] ?? `Axis ${src.axis}`)
            : `${AXIS_LABELS[src.axisA] ?? `Axis ${src.axisA}`} - ${AXIS_LABELS[src.axisB] ?? `Axis ${src.axisB}`}`;
          return (
            <div key={i} style={{ display: 'flex', justifyContent: 'space-between', fontSize: 12 }}>
              <span style={{ color: '#aaa' }}>{label} → {JOINT_LABELS[src.joint]}</span>
              <span style={{ color: '#888', fontFamily: 'monospace' }}>{(cfg.jointSpeedRad[src.joint] * DEG).toFixed(0)}°/s</span>
            </div>
          );
        })}
        {cfg.gripperButtons.map((b, i) => (
          <div key={i} style={{ display: 'flex', justifyContent: 'space-between', fontSize: 12 }}>
            <span style={{ color: '#aaa' }}>{BUTTON_LABELS[b.button] ?? `Btn ${b.button}`} → Gripper</span>
            <span style={{ color: '#888', fontFamily: 'monospace' }}>{cfg.gripperSpeedPct.toFixed(0)}%/s</span>
          </div>
        ))}
      </div>
    </div>
  );
}
