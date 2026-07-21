import { useState, useCallback, useRef, useEffect, useMemo } from 'react';
import type { RobotDef, Segment } from './kinematics/types';
import { initWasm, fabriCreator, forwardKinematics, solveIk, solveDrawingIk, solveDrawingIkV2 } from './wasm';
import { qToServoDeg, buildWire, requestSerialPort, openPort, sendSerial } from './serial';
import type { FidelityMode } from './renderers/types';
import RobotViewer from './components/RobotViewer';
import JointControls from './components/JointControls';
import InfoPanel from './components/InfoPanel';

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
  const [ikMode, setIkMode] = useState(false);
  const [drawingMode, setDrawingMode] = useState(0); // 0=off, 1=modo1, 2=modo2
  const [drawingActive, setDrawingActive] = useState(false);
  const [ikTarget, setIkTarget] = useState<[number, number, number] | null>(null);
  const [ikError, setIkError] = useState<number | null>(null);
  const [demoRunning, setDemoRunning] = useState(false);
  const demoTimerRef = useRef<ReturnType<typeof setInterval> | null>(null);
  const portRef = useRef<SerialPort | null>(null);
  const [fidelityMode, setFidelityMode] = useState<FidelityMode>('low');

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

  const sendQ = useCallback((segments: Segment[], g: number) => {
    const port = portRef.current;
    if (!port) return;
    const q = segments.map(s => s.q);
    const servoDeg = qToServoDeg(q);
    const wire = buildWire(servoDeg, g);
    console.log('[serial] enviando:', new TextDecoder().decode(wire).trim());
    sendSerial(port, wire);
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
      setConnected(true);
      sendQ(robot.segments, gripper);
    } catch (e: any) {
      setSerialError(e.message ?? 'Error al conectar');
    }
  }, [robot, gripper, sendQ]);

  const handleDisconnect = useCallback(async () => {
    try {
      await portRef.current?.close();
    } catch {}
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
    const solver = drawingMode === 2 ? solveDrawingIkV2
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

  useEffect(() => {
    if (!robot) return;
    sendQ(robot.segments, gripper);
  }, [robot, gripper, sendQ]);

  const handleReset = useCallback(() => {
    const home = fabriCreator();
    setRobot(home);
    setGripper(50);
    sendQRef.current(home.segments, 50);
  }, []);

  const workspacePoints = useMemo(
    () => showWorkspace ? generateWorkspace(2000) : [],
    [showWorkspace],
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
        <InfoPanel robot={robot} />

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
      <RobotViewer robot={robot} gripper={gripper} workspacePoints={workspacePoints} ikTarget={ikTarget} onIkTargetChange={setIkTarget} fidelityMode={fidelityMode} />
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
