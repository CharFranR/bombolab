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
}

// ─── Helper: read translation from matrix ────────────────────────────────────

function getTranslation(m: THREE.Matrix4): [number, number, number] {
  const pos = new THREE.Vector3();
  m.decompose(pos, new THREE.Quaternion(), new THREE.Vector3());
  return [pos.x, pos.y, pos.z];
}

// ─── Component ───────────────────────────────────────────────────────────────

export default function CalibrationPanel({
  target,
  onTargetChange,
  overridesRef,
  configRef,
  onSave,
  onReload,
}: CalibrationPanelProps) {
  const [x, setX] = useState(0);
  const [y, setY] = useState(0);
  const [z, setZ] = useState(0);

  // When target changes, read current translation from refs
  useEffect(() => {
    if (!target) { setX(0); setY(0); setZ(0); return; }
    const m = overridesRef.current.get(target)
          ?? configRef.current.get(target);
    if (m) {
      const [tx, ty, tz] = getTranslation(m);
      setX(tx);
      setY(ty);
      setZ(tz);
    } else {
      setX(0); setY(0); setZ(0);
    }
  }, [target, overridesRef, configRef]);

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
    <div style={{
      position: 'absolute',
      top: 16,
      right: 16,
      zIndex: 10,
      background: 'rgba(30, 30, 35, 0.92)',
      padding: 16,
      borderRadius: 8,
      border: '1px solid #444',
      display: 'flex',
      flexDirection: 'column',
      gap: 8,
      minWidth: 200,
    }}>
      <div style={{ fontSize: 13, fontWeight: 600, color: '#ddd', marginBottom: 4 }}>
        Calibration
      </div>
      <label style={{ fontSize: 11, color: '#888' }}>STL File</label>
      <select
        value=""
        onChange={handleTargetChange}
        style={{
          width: '100%',
          padding: '6px 8px',
          fontSize: 12,
          background: '#3a3a3a',
          border: '1px solid #555',
          borderRadius: 4,
          color: '#ddd',
        }}
      >
        <option value="">-- Select a piece --</option>
        {ALL_STL_FILES.map((file) => (
          <option key={file} value={file}>{file}</option>
        ))}
      </select>
      <span style={{ fontSize: 10, color: '#666' }}>
        Pick a piece above to start calibrating
      </span>
    </div>
  );

  return (
    <div style={{
      position: 'absolute',
      top: 16,
      right: 16,
      zIndex: 10,
      background: 'rgba(30, 30, 35, 0.92)',
      padding: 16,
      borderRadius: 8,
      border: '1px solid #444',
      display: 'flex',
      flexDirection: 'column',
      gap: 8,
      minWidth: 200,
    }}>
      <div style={{ fontSize: 13, fontWeight: 600, color: '#ddd', marginBottom: 4 }}>
        Calibration
      </div>

      {/* STL file selector */}
      <label style={{ fontSize: 11, color: '#888' }}>STL File</label>
      <select
        value={target}
        onChange={handleTargetChange}
        style={{
          width: '100%',
          padding: '6px 8px',
          fontSize: 12,
          background: '#3a3a3a',
          border: '1px solid #555',
          borderRadius: 4,
          color: '#ddd',
        }}
      >
        <option value="">-- Select --</option>
        {ALL_STL_FILES.map((file) => (
          <option key={file} value={file}>{file}</option>
        ))}
      </select>

      {/* Translation inputs */}
      <label style={{ fontSize: 11, color: '#888' }}>Translation (mm)</label>
      <div style={{ display: 'flex', gap: 4 }}>
        <div style={{ flex: 1, display: 'flex', flexDirection: 'column', gap: 2 }}>
          <span style={{ fontSize: 10, color: '#ff6666', textAlign: 'center' }}>X</span>
          <input
            type="number"
            step={0.1}
            value={x}
            onChange={handleXChange}
            style={{
              width: '100%',
              padding: '4px 6px',
              fontSize: 12,
              background: '#3a3a3a',
              border: '1px solid #555',
              borderRadius: 4,
              color: '#ddd',
              boxSizing: 'border-box',
            }}
          />
        </div>
        <div style={{ flex: 1, display: 'flex', flexDirection: 'column', gap: 2 }}>
          <span style={{ fontSize: 10, color: '#66ff66', textAlign: 'center' }}>Y</span>
          <input
            type="number"
            step={0.1}
            value={y}
            onChange={handleYChange}
            style={{
              width: '100%',
              padding: '4px 6px',
              fontSize: 12,
              background: '#3a3a3a',
              border: '1px solid #555',
              borderRadius: 4,
              color: '#ddd',
              boxSizing: 'border-box',
            }}
          />
        </div>
        <div style={{ flex: 1, display: 'flex', flexDirection: 'column', gap: 2 }}>
          <span style={{ fontSize: 10, color: '#4488ff', textAlign: 'center' }}>Z</span>
          <input
            type="number"
            step={0.1}
            value={z}
            onChange={handleZChange}
            style={{
              width: '100%',
              padding: '4px 6px',
              fontSize: 12,
              background: '#3a3a3a',
              border: '1px solid #555',
              borderRadius: 4,
              color: '#ddd',
              boxSizing: 'border-box',
            }}
          />
        </div>
      </div>

      {/* Buttons */}
      <div style={{ display: 'flex', gap: 4, marginTop: 4 }}>
        <button
          onClick={onSave}
          style={{
            flex: 1,
            padding: '6px 0',
            fontSize: 11,
            background: '#364',
            border: 'none',
            borderRadius: 4,
            color: '#ccc',
            cursor: 'pointer',
          }}
        >
          Save Calibration
        </button>
        <button
          onClick={onReload}
          style={{
            flex: 1,
            padding: '6px 0',
            fontSize: 11,
            background: '#633',
            border: 'none',
            borderRadius: 4,
            color: '#ccc',
            cursor: 'pointer',
          }}
        >
          Reload
        </button>
      </div>
    </div>
  );
}
