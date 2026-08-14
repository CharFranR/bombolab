import { useState, useEffect, useCallback } from 'react';
import * as THREE from 'three';
import { ALL_STL_FILES } from './stlMapping';

// ─── Props ───────────────────────────────────────────────────────────────────

interface CalibrationPanelProps {
  target: string | null;
  onTargetChange: (file: string | null) => void;
  overridesRef: React.MutableRefObject<Map<string, THREE.Matrix4>>;
  configRef: React.MutableRefObject<Map<string, THREE.Matrix4>>;
  onSave: () => void;
  onReload: () => void;
  onUpload: () => void;
  gizmoMode: 'translate' | 'rotate';
  onGizmoModeChange: (mode: 'translate' | 'rotate') => void;
  stlScaleRef: React.MutableRefObject<number>;
  version: number;
}

// ─── Helpers ────────────────────────────────────────────────────────────────

function getTranslation(m: THREE.Matrix4): [number, number, number] {
  const pos = new THREE.Vector3();
  m.decompose(pos, new THREE.Quaternion(), new THREE.Vector3());
  return [pos.x, pos.y, pos.z];
}

const stepBtnStyle: React.CSSProperties = {
  padding: '1px 3px',
  fontSize: 9,
  background: 'rgba(255, 255, 255, 0.04)',
  border: '1px solid var(--border)',
  borderRadius: 6,
  color: 'var(--c-text-dim)',
  cursor: 'pointer',
  fontFamily: 'monospace',
};

const STEPS = [-50, -10, -1, 1, 10, 50] as const;

const inputStyle: React.CSSProperties = {
  width: '100%',
  padding: '4px 6px',
  fontSize: 12,
  background: 'rgba(255, 255, 255, 0.04)',
  border: '1px solid var(--border)',
  borderRadius: 6,
  color: '#ddd',
  boxSizing: 'border-box',
};

// ─── Component ───────────────────────────────────────────────────────────────

export default function CalibrationPanel({
  target,
  onTargetChange,
  overridesRef,
  configRef,
  onSave,
  onReload,
  onUpload,
  gizmoMode,
  onGizmoModeChange,
  stlScaleRef,
  version,
}: CalibrationPanelProps) {
  const [x, setX] = useState(0);
  const [y, setY] = useState(0);
  const [z, setZ] = useState(0);
  const [globalScale, setGlobalScale] = useState(stlScaleRef.current);

  // When target changes, read current translation from refs
  useEffect(() => {
    if (!target) { setX(0); setY(0); setZ(0); return; }
    const m = overridesRef.current.get(target)
          ?? configRef.current.get(target);
    if (m) {
      const [tx, ty, tz] = getTranslation(m);
      setX(tx); setY(ty); setZ(tz);
    } else {
      setX(0); setY(0); setZ(0);
    }
  }, [target, version, overridesRef, configRef]);

  // Update matrix translation in overridesRef for the current target
  const updateTranslation = useCallback((tx: number, ty: number, tz: number) => {
    if (!target) return;
    const current = overridesRef.current.get(target)
                ?? configRef.current.get(target)
                ?? new THREE.Matrix4().identity();
    const pos = new THREE.Vector3();
    const quat = new THREE.Quaternion();
    const scale = new THREE.Vector3();
    current.decompose(pos, quat, scale);
    const m = new THREE.Matrix4().compose(
      new THREE.Vector3(tx, ty, tz),
      quat,
      new THREE.Vector3(1, 1, 1),
    );
    overridesRef.current.set(target, m);
  }, [target, overridesRef, configRef]);

  const handleXChange = useCallback((e: React.ChangeEvent<HTMLInputElement>) => {
    const v = parseFloat(e.target.value) || 0;
    setX(v);
    updateTranslation(v, y, z);
  }, [y, z, updateTranslation]);

  const handleYChange = useCallback((e: React.ChangeEvent<HTMLInputElement>) => {
    const v = parseFloat(e.target.value) || 0;
    setY(v);
    updateTranslation(x, v, z);
  }, [x, z, updateTranslation]);

  const handleZChange = useCallback((e: React.ChangeEvent<HTMLInputElement>) => {
    const v = parseFloat(e.target.value) || 0;
    setZ(v);
    updateTranslation(x, y, v);
  }, [x, y, updateTranslation]);

  const handleTargetChange = useCallback((e: React.ChangeEvent<HTMLSelectElement>) => {
    const val = e.target.value;
    onTargetChange(val || null);
  }, [onTargetChange]);

  if (!target) return (
    <div
      className="glass-card"
      style={{
        position: 'absolute',
        top: 16,
        right: 16,
        zIndex: 10,
        padding: 16,
        display: 'flex',
        flexDirection: 'column',
        gap: 8,
        minWidth: 200,
      }}
    >
      <div style={{ fontSize: 13, fontWeight: 600, color: '#ddd', marginBottom: 4 }}>
        Calibration
      </div>
      <label style={{ fontSize: 11, color: 'var(--c-gray)' }}>STL File</label>
        <select
          value=""
          onChange={handleTargetChange}
          style={{
            width: '100%',
            padding: '6px 8px',
            fontSize: 12,
            background: 'rgba(255, 255, 255, 0.04)',
            border: '1px solid var(--border)',
            borderRadius: 6,
            color: '#ddd',
          }}
        >
        <option value="">-- Select a piece --</option>
        {ALL_STL_FILES.map((file) => (
          <option key={file} value={file}>{file}</option>
        ))}
      </select>
      <span style={{ fontSize: 10, color: 'var(--c-text-faint)' }}>
        Pick a piece above to start calibrating
      </span>
    </div>
  );

  return (
    <div
      className="glass-card"
      style={{
        position: 'absolute',
        top: 16,
        right: 16,
        zIndex: 10,
        padding: 16,
        display: 'flex',
        flexDirection: 'column',
        gap: 8,
        minWidth: 200,
      }}
    >
      <div style={{ fontSize: 13, fontWeight: 600, color: '#ddd', marginBottom: 4 }}>
        Calibration
      </div>

      {/* STL file selector */}
      <label style={{ fontSize: 11, color: 'var(--c-gray)' }}>STL File</label>
        <select
          value={target}
          onChange={handleTargetChange}
          style={{
            width: '100%',
            padding: '6px 8px',
            fontSize: 12,
            background: 'rgba(255, 255, 255, 0.04)',
            border: '1px solid var(--border)',
            borderRadius: 6,
            color: '#ddd',
          }}
        >
        <option value="">-- Select --</option>
        {ALL_STL_FILES.map((file) => (
          <option key={file} value={file}>{file}</option>
        ))}
      </select>

      {/* Translation inputs */}
      <label style={{ fontSize: 11, color: 'var(--c-gray)' }}>Translation (mm) — drag gizmo or type/step</label>
      <div style={{ display: 'flex', gap: 4 }}>
        {/* X */}
        <div style={{ flex: 1, display: 'flex', flexDirection: 'column', gap: 2 }}>
          <span style={{ fontSize: 10, color: '#ff6666', textAlign: 'center' }}>X</span>
          <input type="number" step={0.1} value={x} onChange={handleXChange} style={inputStyle} />
          <div style={{ display: 'flex', gap: 1, justifyContent: 'center' }}>
            {STEPS.map((s) => (
              <button key={s} style={stepBtnStyle}
                onMouseDown={(e) => e.preventDefault()}
                onClick={() => { const nx = x + s; setX(nx); updateTranslation(nx, y, z); }}>
                {s > 0 ? `+${s}` : s}
              </button>
            ))}
          </div>
        </div>
        {/* Y */}
        <div style={{ flex: 1, display: 'flex', flexDirection: 'column', gap: 2 }}>
          <span style={{ fontSize: 10, color: '#66ff66', textAlign: 'center' }}>Y</span>
          <input type="number" step={0.1} value={y} onChange={handleYChange} style={inputStyle} />
          <div style={{ display: 'flex', gap: 1, justifyContent: 'center' }}>
            {STEPS.map((s) => (
              <button key={s} style={stepBtnStyle}
                onMouseDown={(e) => e.preventDefault()}
                onClick={() => { const ny = y + s; setY(ny); updateTranslation(x, ny, z); }}>
                {s > 0 ? `+${s}` : s}
              </button>
            ))}
          </div>
        </div>
        {/* Z */}
        <div style={{ flex: 1, display: 'flex', flexDirection: 'column', gap: 2 }}>
          <span style={{ fontSize: 10, color: '#4488ff', textAlign: 'center' }}>Z</span>
          <input type="number" step={0.1} value={z} onChange={handleZChange} style={inputStyle} />
          <div style={{ display: 'flex', gap: 1, justifyContent: 'center' }}>
            {STEPS.map((s) => (
              <button key={s} style={stepBtnStyle}
                onMouseDown={(e) => e.preventDefault()}
                onClick={() => { const nz = z + s; setZ(nz); updateTranslation(x, y, nz); }}>
                {s > 0 ? `+${s}` : s}
              </button>
            ))}
          </div>
        </div>
      </div>

      {/* Global scale */}
      <label style={{ fontSize: 11, color: 'var(--c-gray)' }}>STL Scale (global)</label>
      <div style={{ display: 'flex', gap: 4, alignItems: 'center' }}>
        <input type="range" min={0.1} max={5} step={0.01} value={globalScale}
          onChange={(e) => { const v = parseFloat(e.target.value); setGlobalScale(v); stlScaleRef.current = v; }}
          style={{ flex: 1 }} />
        <input type="number" step={0.01} value={globalScale}
          onChange={(e) => { const v = parseFloat(e.target.value) || 1; setGlobalScale(v); stlScaleRef.current = v; }}
          style={{ ...inputStyle, width: 60 }} />
      </div>

      {/* Gizmo mode toggle */}
      <label style={{ fontSize: 11, color: 'var(--c-gray)' }}>Gizmo</label>
      <div style={{ display: 'flex', gap: 4 }}>
        <button onClick={() => onGizmoModeChange('translate')}
          style={{
            flex: 1, padding: '4px 0', fontSize: 11, cursor: 'pointer',
            background: gizmoMode === 'translate' ? 'rgba(0, 242, 254, 0.16)' : 'rgba(255, 255, 255, 0.04)',
            border: `1px solid ${gizmoMode === 'translate' ? 'rgba(0, 242, 254, 0.45)' : 'var(--border)'}`,
            borderRadius: 6, color: 'var(--c-text)',
          }}>↕ Translate</button>
        <button onClick={() => onGizmoModeChange('rotate')}
          style={{
            flex: 1, padding: '4px 0', fontSize: 11, cursor: 'pointer',
            background: gizmoMode === 'rotate' ? 'rgba(0, 102, 255, 0.18)' : 'rgba(255, 255, 255, 0.04)',
            border: `1px solid ${gizmoMode === 'rotate' ? 'rgba(0, 102, 255, 0.5)' : 'var(--border)'}`,
            borderRadius: 6, color: 'var(--c-text)',
          }}>↻ Rotate</button>
      </div>

      {/* Buttons */}
      <div style={{ display: 'flex', gap: 4, marginTop: 4 }}>
        <button onClick={onSave}
          style={{ flex: 1, padding: '6px 0', fontSize: 11, background: 'rgba(0, 242, 254, 0.16)', border: '1px solid rgba(0, 242, 254, 0.45)', borderRadius: 6, color: 'var(--c-cyan)', cursor: 'pointer' }}>
           Save
        </button>
        <button onClick={onUpload}
          style={{ flex: 1, padding: '6px 0', fontSize: 11, background: 'rgba(0, 102, 255, 0.18)', border: '1px solid rgba(0, 102, 255, 0.5)', borderRadius: 6, color: 'var(--c-cobalt)', cursor: 'pointer' }}>
           Upload
        </button>
      </div>
      <div style={{ display: 'flex', gap: 4 }}>
        <button onClick={onReload}
          style={{ flex: 1, padding: '4px 0', fontSize: 10, background: 'rgba(190, 60, 60, 0.18)', border: '1px solid rgba(210, 80, 80, 0.45)', borderRadius: 6, color: '#e88', cursor: 'pointer' }}>
           Reload defaults
        </button>
      </div>
    </div>
  );
}
