import { useState, useCallback, useRef, useEffect, useMemo } from 'react';
import * as THREE from 'three';
import type { RobotDef, Segment } from './kinematics/types';
import { parseGcode, mapPoint, drawingBoundingBox, defaultMapping, fitScale } from './lib/gcodeCipra';
import { initWasm, fabriCreator, forwardKinematics, solveIk, solveDrawingIk, solveDrawingIkV2, solveDrawingPlaneIk } from './wasm';
import { qToServoDeg, gripperToServo, requestSerialPort, openPort, sendSerial } from './serial';
import { ServoInterpolator } from './interpolation';
import type { DebugToggles, FidelityMode, CalibrationConfig } from './renderers/types';
import { ALL_STL_FILES } from './renderers/stlMapping';
import RobotViewer from './components/RobotViewer';
import JointControls from './components/JointControls';
import InfoPanel from './components/InfoPanel';
import CalibrationPanel from './renderers/CalibrationPanel';

function LoadingScreen({ error }: { error?: string }) {
  return (
    <div style={{ display: 'flex', width: '100%', height: '100%', background: '#1c1c20', color: '#ccc', alignItems: 'center', justifyContent: 'center', flexDirection: 'column', gap: 12 }}>
      {error ? (
        <p style={{ fontSize: 14, color: '#e55' }}>Error: {error}</p>
      ) : (
        <p style={{ fontSize: 16, color: '#888' }}>Cargando WASM...</p>
      )}
    </div>
  );
}

export default function App() {
  const [ready, setReady] = useState(false);
  const [loadError, setLoadError] = useState<string | null>(null);
  const [robot, setRobot] = useState<RobotDef | null>(null);
  const [gripper, setGripper] = useState(50);
  const [connected, setConnected] = useState(false);
  const [serialError, setSerialError] = useState<string | null>(null);
  const [showWorkspace, setShowWorkspace] = useState(false);
  const [trajectoryPoints, setTrajectoryPoints] = useState<[number, number, number][]>([]);
  const [ikMode, setIkMode] = useState(false);
  const [drawingMode, setDrawingMode] = useState(0); // 0=off, 1=modo1, 2=modo2
  const [drawingActive, setDrawingActive] = useState(false);
  const [ikTarget, setIkTarget] = useState<[number, number, number] | null>(null);
  const [ikError, setIkError] = useState<number | null>(null);
  const [demoRunning, setDemoRunning] = useState(false);
  const demoTimerRef = useRef<ReturnType<typeof setInterval> | null>(null);
  const portRef = useRef<SerialPort | null>(null);
  const servoInterpolatorRef = useRef<ServoInterpolator | null>(null);
  const [fidelityMode, setFidelityMode] = useState<FidelityMode>('low');
  const [debugToggles, setDebugToggles] = useState<DebugToggles>({
    showJointFrames: false,
    showStlOrigins: false,
    showCalibrationAxes: false,
  });

  // ─── Calibration state ──────────────────────────────────────────────────
  const [calibrationMode, setCalibrationMode] = useState(false);
  const [calibrationTarget, setCalibrationTarget] = useState<string | null>(null);
  const [calibrationVersion, setCalibrationVersion] = useState(0);
  const [gizmoMode, setGizmoMode] = useState<'translate' | 'rotate'>('translate');
  const calibrationConfigRef = useRef<Map<string, THREE.Matrix4>>(new Map());
  const calibrationOverridesRef = useRef<Map<string, THREE.Matrix4>>(new Map());
  const stlScaleRef = useRef(1.0);
  const handleCalibrationChange = useCallback(() => {
    setCalibrationVersion((v) => v + 1);
  }, []);

  // Upload: user selects a JSON file, loads into overridesRef
  const handleUploadCalibration = useCallback(() => {
    const input = document.createElement('input');
    input.type = 'file';
    input.accept = '.json';
    input.onchange = (e: Event) => {
      const file = (e.target as HTMLInputElement).files?.[0];
      if (!file) return;
      const reader = new FileReader();
      reader.onload = () => {
        try {
          const data = JSON.parse(reader.result as string);
          if (data.version !== 1) { console.warn('Unknown calibration version'); return; }
          const map = new Map<string, THREE.Matrix4>();
          for (const entry of data.entries) {
            const m = new THREE.Matrix4().compose(
              new THREE.Vector3(...entry.translation),
              new THREE.Quaternion(...entry.rotation),
              new THREE.Vector3(1, 1, 1),
            );
            map.set(entry.filename, m);
          }
          calibrationOverridesRef.current = map;
          setCalibrationVersion((v) => v + 1);
        } catch (err) { console.error('Failed to parse calibration file', err); }
      };
      reader.readAsText(file);
    };
    input.click();
  }, []);

  useEffect(() => {
    initWasm()
      .then(() => {
        setRobot(fabriCreator());
        setReady(true);
      })
      .catch((e) => setLoadError(e.message ?? String(e)));
  return () => {
    if (demoTimerRef.current) clearInterval(demoTimerRef.current);
  };
}, []);

  // ─── Fetch calibration config on mount ──────────────────────────────────
  useEffect(() => {
    let cancelled = false;
    fetch('/calibration.json')
      .then((res) => {
        if (!res.ok) throw new Error(`HTTP ${res.status}`);
        return res.json();
      })
      .then((config: CalibrationConfig) => {
        if (cancelled) return;
        if (!config || config.version !== 1) {
          console.warn('[App] calibration.json: invalid or missing version — using identity');
          return;
        }
        const map = new Map<string, THREE.Matrix4>();
        for (const entry of config.entries) {
          const [tx, ty, tz] = entry.translation;
          const [rx, ry, rz, rw] = entry.rotation;
          const m = new THREE.Matrix4().compose(
            new THREE.Vector3(tx, ty, tz),
            new THREE.Quaternion(rx, ry, rz, rw),
            new THREE.Vector3(1, 1, 1),
          );
          map.set(entry.filename, m);
        }
        calibrationConfigRef.current = map;
        stlScaleRef.current = config.stlScale ?? 1.0;
        console.log(`[App] Loaded calibration.json — ${map.size} entries, scale ${stlScaleRef.current}`);
      })
      .catch((err) => {
        if (cancelled) return;
        console.warn('[App] Failed to load calibration.json:', err.message);
      });
    return () => { cancelled = true; };
  }, []);

  const sendQ = useCallback((segments: Segment[], g: number) => {
    const interp = servoInterpolatorRef.current;
    if (!interp) return;
    const servoDeg = qToServoDeg(segments.map(s => s.q));
    const target = [...servoDeg, gripperToServo(g)];
    console.log('[serial] target:', target.join(','));
    interp.moveTo(target);
  }, []);

  const sendQRef = useRef(sendQ);
  sendQRef.current = sendQ;

  const handleConnect = useCallback(async () => {
    if (!robot) return;
    try {
      setSerialError(null);
      const port = await requestSerialPort();
      await openPort(port);
      portRef.current = port;
      // Start the interpolation scheduler from the current pose and push
      // one frame so the firmware leaves its boot/home state.
      const initial = [...qToServoDeg(robot.segments.map(s => s.q)), gripperToServo(gripper)];
      servoInterpolatorRef.current = new ServoInterpolator((wire) => sendSerial(port, wire), initial);
      servoInterpolatorRef.current.keepAlive();
      setConnected(true);
    } catch (e: any) {
      setSerialError(e.message ?? 'Error al conectar');
    }
  }, [robot, gripper]);

  const handleDisconnect = useCallback(async () => {
    try {
      await portRef.current?.close();
    } catch {}
    servoInterpolatorRef.current?.stop();
    servoInterpolatorRef.current = null;
    portRef.current = null;
    setConnected(false);
  }, []);

  const handleJointChange = useCallback((index: number, qRad: number) => {
    setRobot(prev => {
      if (!prev) return prev;
      return { ...prev, segments: prev.segments.map((seg, i) => ({ ...seg, q: i === index ? qRad : seg.q })) };
    });
  }, []);

  useEffect(() => {
    if (!ikMode || !ikTarget || !robot) return;
    const solver = drawingMode === 2 ? solveDrawingPlaneIk
                 : drawingMode === 1 ? solveDrawingIk
                 : solveIk;
    const qInit = robot.segments.map(s => s.q);
    const result = solver(robot, ikTarget, qInit);
    setIkError(result.error);
    setDrawingActive(result.converged && result.error < 10);
    setRobot(prev => {
      if (!prev) return prev;
      return { ...prev, segments: prev.segments.map((seg, i) => ({ ...seg, q: result.q[i] ?? 0 })) };
    });
  }, [ikTarget, ikMode, drawingMode]);

  // Scroll wheel → ajustar Z del target IK
  useEffect(() => {
    if (!ikMode || !ikTarget) return;
    const onWheel = (e: WheelEvent) => {
      const step = e.deltaY > 0 ? -5 : 5;
      setIkTarget(prev => prev ? [prev[0], prev[1], prev[2] + step] : null);
    };
    window.addEventListener('wheel', onWheel, { passive: true });
    return () => window.removeEventListener('wheel', onWheel);
  }, [ikMode, ikTarget]);

  // Deactivate calibration mode when switching to low fidelity
  useEffect(() => {
    if (fidelityMode === 'low') setCalibrationMode(false);
  }, [fidelityMode]);

  useEffect(() => {
    if (!robot) return;
    sendQ(robot.segments, gripper);
  }, [robot, gripper, sendQ]);

  // Heartbeat — the firmware failsafe parks the arm at home after 5s
  // without a valid frame. Re-send the last-sent pose every second while
  // connected so an idle robot holds its commanded position instead of
  // returning to home (keeps working during in-flight interpolation).
  useEffect(() => {
    if (!connected) return;
    const id = setInterval(() => {
      servoInterpolatorRef.current?.keepAlive();
    }, 1000);
    return () => clearInterval(id);
  }, [connected]);

  const handleReset = useCallback(() => {
    const home = fabriCreator();
    setRobot(home);
    setGripper(50);
    sendQRef.current(home.segments, 50);
  }, []);

  // ─── Calibration save handler ──────────────────────────────────────────
  // Merges overrides into config, serializes as calibration.json, triggers
  // browser download. The user must manually replace web/public/calibration.json.
  const handleSaveCalibration = useCallback(() => {
    const entries = ALL_STL_FILES.map((file) => {
      const m = calibrationOverridesRef.current.get(file)
             ?? calibrationConfigRef.current.get(file)
             ?? new THREE.Matrix4().identity();
      const pos = new THREE.Vector3();
      const quat = new THREE.Quaternion();
      m.decompose(pos, quat, new THREE.Vector3());
      return {
        filename: file,
        translation: [pos.x, pos.y, pos.z] as [number, number, number],
        rotation: [quat.x, quat.y, quat.z, quat.w] as [number, number, number, number],
      };
    });
    const blob = new Blob(
      [JSON.stringify({ version: 1, stlScale: stlScaleRef.current, entries }, null, 2)],
      { type: 'application/json' },
    );
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = 'calibration.json';
    a.click();
    URL.revokeObjectURL(url);
  }, []);

  // ─── Calibration reload handler ─────────────────────────────────────────
  // Clears overrides and re-fetches the server config.
  const handleReloadCalibration = useCallback(() => {
    calibrationOverridesRef.current.clear();
    setCalibrationTarget(null);
    fetch('/calibration.json')
      .then((res) => {
        if (!res.ok) throw new Error(`HTTP ${res.status}`);
        return res.json();
      })
      .then((config: CalibrationConfig) => {
        if (!config || config.version !== 1) {
          console.warn('[App] Reload: calibration.json invalid — using identity');
          return;
        }
        const map = new Map<string, THREE.Matrix4>();
        for (const entry of config.entries) {
          const [tx, ty, tz] = entry.translation;
          const [rx, ry, rz, rw] = entry.rotation;
          const m = new THREE.Matrix4().compose(
            new THREE.Vector3(tx, ty, tz),
            new THREE.Quaternion(rx, ry, rz, rw),
            new THREE.Vector3(1, 1, 1),
          );
          map.set(entry.filename, m);
        }
        calibrationConfigRef.current = map;
        console.log(`[App] Reloaded calibration.json — ${map.size} entries`);
      })
      .catch((err) => {
        console.warn('[App] Reload: failed to fetch calibration.json:', err.message);
      });
  }, []);

  // Upload: user selects a CIPRA `.gcode` file and the browser resolves the
  // whole pipeline (parse → map/auto-scale → drawing-mode IK → FK tool-tip),
  // mirroring the gcode-bridge crate's algorithm for identical results.
  const handleLoadGcode = useCallback(() => {
    if (!robot) return;
    const input = document.createElement('input');
    input.type = 'file';
    input.accept = '.gcode,.nc,.txt';
    input.onchange = (e: Event) => {
      const file = (e.target as HTMLInputElement).files?.[0];
      if (!file) return;
      const reader = new FileReader();
      reader.onload = () => {
        const { strokes, error } = parseGcode(reader.result as string);
        if (error || strokes.length === 0) {
          console.error('[App] Failed to parse gcode:', error ?? 'no strokes');
          setTrajectoryPoints([]);
          return;
        }
        const bbox = drawingBoundingBox(strokes);
        if (!bbox) {
          setTrajectoryPoints([]);
          return;
        }
        const config = defaultMapping();
        const drawingW = bbox.maxX - bbox.minX;
        const drawingH = bbox.maxY - bbox.minY;
        const scale = config.scale ?? fitScale(config.target, drawingW, drawingH);

        const points: [number, number, number][] = [];
        let convergedCount = 0;
        let totalTargets = 0;
        for (const stroke of strokes) {
          if (stroke.length === 0) continue;
          const first = stroke[0];
          const travel = mapPoint(first[0], first[1], drawingW, drawingH, config, 'travel');
          for (const [mx, my, mz] of [travel, ...stroke.map((p) => mapPoint(p[0], p[1], drawingW, drawingH, config, 'draw'))]) {
            totalTargets += 1;
            const target: [number, number, number] = [mx, my, mz];
            const res = solveDrawingPlaneIk(robot, target, [0, 0, 0, 0, 0]);
            if (!res.converged) {
              console.warn(`[App] IK no converge en target (${mx},${my},${mz}): error ${res.error.toFixed(2)}mm`);
              continue;
            }
            convergedCount += 1;
            // Resolve FK for this step and extract the tool-tip position with
            // the same DH→THREE [x,z,y] swap the JSON loader uses.
            const segs = robot.segments.map((s, i) => ({ ...s, q: res.q[i] }));
            const fk = forwardKinematics(segs, robot.baseTransform);
            points.push([fk.ee[3], fk.ee[11], fk.ee[7]]);
          }
        }
        setTrajectoryPoints(points);
        console.log(
          `[App] Loaded gcode — ${strokes.length} strokes, ${points.length} tool-tip points (IK ${convergedCount}/${totalTargets} converged), scale ${scale.toFixed(4)}`,
        );
      };
      reader.readAsText(file);
    };
    input.click();
  }, [robot]);

  const workspacePoints = useMemo(
    () => showWorkspace ? generateWorkspace(2000) : [],
    [showWorkspace],
  );

  // P2 (Stage 3C): FK calculado UNA vez en App y distribuido a los
  // consumidores (RobotViewer + InfoPanel). App no interpreta ni modifica
  // la cinemática — solo comparte el resultado crudo.
  const rawFrames = useMemo(
    () => robot ? forwardKinematics(robot.segments, robot.baseTransform).frames : [],
    [robot],
  );

  if (!ready || !robot) return <LoadingScreen error={loadError ?? undefined} />;

  return (
    <div style={{ display: 'flex', width: '100%', height: '100%', background: '#1c1c20', color: '#ccc' }}>
      {/* Sidebar */}
      <div style={{
        width: 280,
        minWidth: 280,
        height: '100%',
        display: 'flex',
        flexDirection: 'column',
        background: '#24242a',
        borderRight: '1px solid #333',
      }}>
        {/* Header */}
        <div style={{ padding: '16px', borderBottom: '1px solid #333' }}>
          <h1 style={{ fontSize: 18, fontWeight: 700, margin: 0, color: '#eee' }}>
            Bombolab
          </h1>
          <p style={{ fontSize: 12, color: '#666', margin: '4px 0 0' }}>
            FABRI Creator · 5-DOF
          </p>
        </div>

        {/* Joint sliders */}
        <div style={{ flex: 1, overflow: 'auto' }}>
          <JointControls
            segments={robot.segments}
            gripper={gripper}
            onGripperChange={setGripper}
            onChange={handleJointChange}
            disabled={ikMode}
          />
        </div>

        {/* Info panel */}
        <InfoPanel robot={robot} rawFrames={rawFrames} />

        {/* Fidelity toggle */}
        <div style={{ padding: '8px 16px', borderTop: '1px solid #333' }}>
          <div style={{ display: 'flex', alignItems: 'center', gap: 4, marginBottom: 4 }}>
            <span style={{ fontSize: 11, color: '#888' }}>Fidelidad:</span>
          </div>
          <div style={{ display: 'flex', gap: 4 }}>
            <button
              onClick={() => setFidelityMode('low')}
              style={{
                flex: 1,
                padding: '6px 0',
                fontSize: 12,
                background: fidelityMode === 'low' ? '#553' : '#3a3a3a',
                border: '1px solid ' + (fidelityMode === 'low' ? '#885' : '#444'),
                borderRadius: 3,
                color: fidelityMode === 'low' ? '#ddc' : '#888',
                cursor: 'pointer',
              }}
            >
              Low
            </button>
            <button
              onClick={() => setFidelityMode('high')}
              style={{
                flex: 1,
                padding: '6px 0',
                fontSize: 12,
                background: fidelityMode === 'high' ? '#553' : '#3a3a3a',
                border: '1px solid ' + (fidelityMode === 'high' ? '#885' : '#444'),
                borderRadius: 3,
                color: fidelityMode === 'high' ? '#ddc' : '#888',
                cursor: 'pointer',
              }}
            >
              High
            </button>
          </div>
        </div>

        {/* Calibration mode — visible only in high fidelity */}
        {fidelityMode === 'high' && (
          <div style={{ padding: '8px 16px', borderTop: '1px solid #333' }}>
            <label style={{ display: 'flex', alignItems: 'center', gap: 6, fontSize: 12, color: '#aaa', cursor: 'pointer' }}>
              <input
                type="checkbox"
                checked={calibrationMode}
                onChange={(e) => setCalibrationMode(e.target.checked)}
              />
              Calibration Mode
            </label>
          </div>
        )}

        {/* Debug visualization toggles — visible only in high fidelity */}
        {fidelityMode === 'high' && (
          <div style={{ padding: '8px 16px', borderTop: '1px solid #333' }}>
            <div style={{ fontSize: 11, color: '#888', marginBottom: 6 }}>Debug:</div>
            <label style={{ display: 'flex', alignItems: 'center', gap: 6, fontSize: 12, color: '#aaa', cursor: 'pointer', marginBottom: 4 }}>
              <input
                type="checkbox"
                checked={debugToggles.showJointFrames}
                onChange={(e) => setDebugToggles(prev => ({ ...prev, showJointFrames: e.target.checked }))}
              />
              Show Joint Frames
            </label>
            <label style={{ display: 'flex', alignItems: 'center', gap: 6, fontSize: 12, color: '#aaa', cursor: 'pointer', marginBottom: 4 }}>
              <input
                type="checkbox"
                checked={debugToggles.showStlOrigins}
                onChange={(e) => setDebugToggles(prev => ({ ...prev, showStlOrigins: e.target.checked }))}
              />
              Show STL Origins
            </label>
            <label style={{ display: 'flex', alignItems: 'center', gap: 6, fontSize: 12, color: '#aaa', cursor: 'pointer', marginBottom: 4 }}>
              <input
                type="checkbox"
                checked={debugToggles.showCalibrationAxes}
                onChange={(e) => setDebugToggles(prev => ({ ...prev, showCalibrationAxes: e.target.checked }))}
              />
              Show Calibration Axes
            </label>
            <label style={{ display: 'flex', alignItems: 'center', gap: 6, fontSize: 12, color: '#aaa', cursor: 'pointer', marginBottom: 4 }}>
              <input
                type="checkbox"
                checked={debugToggles.showCandidates ?? false}
                onChange={(e) => setDebugToggles(prev => ({ ...prev, showCandidates: e.target.checked }))}
              />
              Show Calibrator Candidates
            </label>
          </div>
        )}

        {/* Conexión robot físico (WebSerial) */}
        <div style={{ padding: '8px 16px', borderTop: '1px solid #333' }}>
          <div style={{ display: 'flex', alignItems: 'center', gap: 8, marginBottom: 8 }}>
            <span style={{
              width: 8, height: 8, borderRadius: '50%',
              background: connected ? '#4cd964' : '#666',
            }} />
            <span style={{ fontSize: 12, color: '#888' }}>
              {connected ? 'Conectado' : 'Desconectado'}
            </span>
          </div>
          {serialError && (
            <div style={{ fontSize: 11, color: '#e55', marginBottom: 6 }}>{serialError}</div>
          )}
          {connected ? (
            <button onClick={handleDisconnect} style={{
              width: '100%', padding: 8, background: '#633',
              border: 'none', borderRadius: 4, color: '#ccc', fontSize: 13, cursor: 'pointer',
            }}>
              Desconectar
            </button>
          ) : (
            <button onClick={handleConnect} style={{
              width: '100%', padding: 8, background: '#364',
              border: 'none', borderRadius: 4, color: '#ccc', fontSize: 13, cursor: 'pointer',
            }}>
              Conectar robot físico
            </button>
          )}
        </div>

        {/* Drawing mode selector */}
        {ikMode && (
          <>
            <div style={{ padding: '4px 16px', borderTop: '1px solid #333', display: 'flex', alignItems: 'center', gap: 4 }}>
              <span style={{ fontSize: 11, color: '#888', marginRight: 4 }}>Dibujo:</span>
              {[0, 1, 2].map(mode => (
                <button
                  key={mode}
                  onClick={() => setDrawingMode(mode)}
                  style={{
                    flex: 1,
                    padding: '3px 0',
                    fontSize: 11,
                    background: drawingMode === mode ? '#553' : '#3a3a3a',
                    border: '1px solid ' + (drawingMode === mode ? '#885' : '#444'),
                    borderRadius: 3,
                    color: drawingMode === mode ? '#ddc' : '#888',
                    cursor: 'pointer',
                  }}
                >
                  {mode === 0 ? 'Off' : `Modo ${mode}`}
                </button>
              ))}
              {drawingMode > 0 && (
                <span style={{
                  fontSize: 10,
                  color: drawingActive ? '#4c4' : '#a84',
                  marginLeft: 6,
                }}>
                  {drawingActive ? '✓' : '⏎'}
                </span>
              )}
            </div>
            <div style={{ padding: '0 16px 4px', fontSize: 10, color: '#555' }}>
              Rueda mouse: sube/baja Z
            </div>
          </>
        )}

        {/* IK mode */}
        <div style={{ padding: '8px 16px', borderTop: ikMode ? 'none' : '1px solid #333' }}>
          <button
            onClick={() => {
              if (!ikMode) {
                const fk = forwardKinematics(robot.segments, robot.baseTransform);
                const toolM = [
                  1, 0, 0, robot.toolTransform[0],
                  0, 1, 0, robot.toolTransform[1],
                  0, 0, 1, robot.toolTransform[2],
                  0, 0, 0, 1,
                ];
                const ee = fk.frames[fk.frames.length - 1];
                const toolPose = (() => {
                  const m = (r: number, c: number) =>
                    ee[r*4+0]*toolM[0*4+c] + ee[r*4+1]*toolM[1*4+c] +
                    ee[r*4+2]*toolM[2*4+c] + ee[r*4+3]*toolM[3*4+c];
                  return [m(0,3), m(1,3), m(2,3)] as [number, number, number];
                })();
                setIkTarget(toolPose);
                setIkMode(true);
              } else {
                setIkMode(false);
                setIkTarget(null);
              }
            }}
            style={{
              width: '100%',
              padding: 8,
              background: ikMode ? '#553' : '#444',
              border: 'none',
              borderRadius: 4,
              color: '#ccc',
              fontSize: 13,
              cursor: 'pointer',
            }}
          >
            {ikMode ? 'Desactivar IK' : 'IK Mode'}
          </button>
          {ikMode && ikTarget && (
            <div style={{ fontSize: 11, color: '#888', marginTop: 4 }}>
              Target: ({ikTarget[0].toFixed(0)}, {ikTarget[1].toFixed(0)}, {ikTarget[2].toFixed(0)})
              {ikError !== null && (
                <span style={{ color: ikError < 10 ? '#4c4' : '#e84', marginLeft: 8 }}>
                  err: {ikError.toFixed(1)}mm
                </span>
              )}
            </div>
          )}
        </div>

        {/* Workspace toggle */}
        <div style={{ padding: '8px 16px', borderTop: '1px solid #333' }}>
          <button
            onClick={() => setShowWorkspace(!showWorkspace)}
            style={{
              width: '100%',
              padding: 8,
              background: showWorkspace ? '#553' : '#444',
              border: 'none',
              borderRadius: 4,
              color: '#ccc',
              fontSize: 13,
              cursor: 'pointer',
            }}
          >
            {showWorkspace ? 'Ocultar workspace' : 'Mostrar workspace'}
          </button>
        </div>

        {/* Cargar G-code directamente (dialecto CIPRA) */}
        <div style={{ padding: '8px 16px', borderTop: '1px solid #333' }}>
          <button
            onClick={handleLoadGcode}
            style={{
              width: '100%',
              padding: 8,
              background: trajectoryPoints.length > 0 ? '#355' : '#444',
              border: 'none',
              borderRadius: 4,
              color: '#ccc',
              fontSize: 13,
              cursor: 'pointer',
            }}
          >
            Cargar trayectoria .gcode
          </button>
        </div>

        {/* Demo cuadrado */}
        <div style={{ padding: '8px 16px', borderTop: '1px solid #333' }}>
          <button
            onClick={() => {
              if (demoRunning) {
                setDemoRunning(false);
                if (demoTimerRef.current) clearInterval(demoTimerRef.current);
                return;
              }
              // Cerrar gripper completamente
              setGripper(100);

              // Preguntar antes de empezar
              if (!window.confirm('¿El marcador ya está en el gripper? Apretá OK para empezar a dibujar.')) {
                return;
              }

              setIkMode(true);
              setDrawingMode(2); // modo 2: marcador vertical

              // Cuadrado 50x50mm centrado en (200, 0), z=80mm
              const cx = 200, cy = 0, z = 80, half = 25;
              const pts: [number, number, number][] = [
                [cx - half, cy - half, z],
                [cx + half, cy - half, z],
                [cx + half, cy + half, z],
                [cx - half, cy + half, z],
              ];
              let idx = 0;
              setIkTarget(pts[0]);
              setDemoRunning(true);
              demoTimerRef.current = setInterval(() => {
                idx = (idx + 1) % pts.length;
                setIkTarget(pts[idx]);
              }, 2000);
            }}
            style={{
              width: '100%',
              padding: 8,
              background: demoRunning ? '#533' : '#444',
              border: 'none',
              borderRadius: 4,
              color: '#ccc',
              fontSize: 13,
              cursor: 'pointer',
            }}
          >
            {demoRunning ? 'Detener demo' : 'Demo: cuadrado 5×5cm'}
          </button>
        </div>

        {/* Reset */}
        <div style={{ padding: '8px 16px', borderTop: '1px solid #333' }}>
          <button
            onClick={handleReset}
            style={{
              width: '100%',
              padding: '8px',
              background: '#444',
              border: 'none',
              borderRadius: 4,
              color: '#ccc',
              fontSize: 13,
              cursor: 'pointer',
            }}
          >
            Reset Home
          </button>
        </div>
      </div>

      {/* 3D Viewport */}
      <div style={{ flex: 1, position: 'relative' }}>
        <RobotViewer
          robot={robot}
          rawFrames={rawFrames}
          gripper={gripper}
          workspacePoints={workspacePoints}
          trajectoryPoints={trajectoryPoints}
          ikTarget={ikTarget}
          onIkTargetChange={setIkTarget}
          fidelityMode={fidelityMode}
          debugToggles={debugToggles}
          calibrationConfigRef={calibrationConfigRef}
          calibrationOverridesRef={calibrationOverridesRef}
          calibrationTarget={calibrationTarget}
          calibrationMode={calibrationMode}
          calibrationVersion={calibrationVersion}
          onCalibrationChange={handleCalibrationChange}
          gizmoMode={gizmoMode}
          stlScaleRef={stlScaleRef}
        />
        {calibrationMode && fidelityMode === 'high' && (
          <CalibrationPanel
            target={calibrationTarget}
            onTargetChange={setCalibrationTarget}
            overridesRef={calibrationOverridesRef}
            configRef={calibrationConfigRef}
            onSave={handleSaveCalibration}
            onReload={handleReloadCalibration}
            onUpload={handleUploadCalibration}
            gizmoMode={gizmoMode}
            onGizmoModeChange={setGizmoMode}
            stlScaleRef={stlScaleRef}
            version={calibrationVersion}
          />
        )}
      </div>
    </div>
  );
}

function generateWorkspace(samples: number): [number, number, number][] {
  const points: [number, number, number][] = [];
  const robot = fabriCreator();
  const DEG = Math.PI / 180;
  for (let i = 0; i < samples; i++) {
    const q = robot.segments.map((s) => {
      const lo = s.q_min ?? -80 * DEG;
      const hi = s.q_max ?? 80 * DEG;
      return Math.random() * (hi - lo) + lo;
    });
    const segs = robot.segments.map((s, j) => ({ ...s, q: q[j] }));
    const fk = forwardKinematics(segs, robot.baseTransform);
    points.push([fk.ee[3], fk.ee[11], fk.ee[7]]);
  }
  return points;
}
