import { useState, useCallback, useRef, useEffect, useMemo } from 'react';
import type { RobotDef, Segment } from './kinematics/types';
import { fabriCreator, fabriCreatorSegments } from './robot/fabri_creator';
import { forwardKinematics } from './kinematics/forward';
import { IkSolver } from './kinematics/ik';
import { qToServoDeg, buildWire, requestSerialPort, openPort, sendSerial } from './serial';
import RobotViewer from './components/RobotViewer';
import JointControls from './components/JointControls';
import InfoPanel from './components/InfoPanel';

const ikSolver = new IkSolver();

function generateWorkspace(samples: number): [number, number, number][] {
  const points: [number, number, number][] = [];
  const robot = fabriCreator();
  const DEG = Math.PI / 180;
  for (let i = 0; i < samples; i++) {
    const q = robot.segments.map(() => (Math.random() * 160 - 80) * DEG);
    const segs = robot.segments.map((s, j) => ({ ...s, q: q[j] }));
    const fk = forwardKinematics(segs, robot.baseTransform);
    // tool transform
    const tool = [1, 0, 0, robot.toolTransform[0],
                  0, 1, 0, robot.toolTransform[1],
                  0, 0, 1, robot.toolTransform[2],
                  0, 0, 0, 1] as const;
    const m = mulMat4(fk.ee, tool as any);
    points.push([m[3], m[11], m[7]]); // Three.js Y-up swap
  }
  return points;
}

function mulMat4(a: any, b: any): number[] {
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

export default function App() {
  const [robot, setRobot] = useState<RobotDef>(() => fabriCreator());
  const [gripper, setGripper] = useState(50);
  const [connected, setConnected] = useState(false);
  const [serialError, setSerialError] = useState<string | null>(null);
  const [showWorkspace, setShowWorkspace] = useState(false);
  const [ikMode, setIkMode] = useState(false);
  const [ikTarget, setIkTarget] = useState<[number, number, number] | null>(null);
  const [ikError, setIkError] = useState<number | null>(null);
  const portRef = useRef<SerialPort | null>(null);

  const workspacePoints = useMemo(
    () => showWorkspace ? generateWorkspace(2000) : [],
    [showWorkspace],
  );

  // Enviar q al Arduino via serial — guardado en ref para acceso desde handlers
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
    try {
      setSerialError(null);
      const port = await requestSerialPort();
      await openPort(port);
      portRef.current = port;
      setConnected(true);
      // Enviar estado actual al conectar
      sendQ(robot.segments, gripper);
    } catch (e: any) {
      setSerialError(e.message ?? 'Error al conectar');
    }
  }, [robot.segments, gripper, sendQ]);

  const handleDisconnect = useCallback(async () => {
    try {
      await portRef.current?.close();
    } catch {}
    portRef.current = null;
    setConnected(false);
  }, []);

  const handleJointChange = useCallback((index: number, qRad: number) => {
    setRobot(prev => {
      const segments = prev.segments.map((seg, i) => ({
        ...seg,
        q: i === index ? qRad : seg.q,
      }));
      return { ...prev, segments };
    });
  }, []);

  // IK: cuando el target cambia, resolver y actualizar q
  useEffect(() => {
    if (!ikMode || !ikTarget) return;
    const result = ikSolver.solvePosition(ikTarget, robot.segments.map(s => s.q), robot);
    setIkError(result.error);
    setRobot(prev => {
      const segments = prev.segments.map((seg, i) => ({ ...seg, q: result.q[i] ?? 0 }));
      return { ...prev, segments };
    });
  }, [ikTarget, ikMode]);

  // Enviar q cada vez que cambia
  useEffect(() => {
    sendQ(robot.segments, gripper);
  }, [robot.segments, gripper, sendQ]);

  const handleReset = useCallback(() => {
    const home = fabriCreator();
    setRobot(home);
    setGripper(50);
    // Forzar envío a home usando la ref (evita stale closure)
    sendQRef.current(home.segments, 50);
  }, []);

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

        {/* IK mode */}
        <div style={{ padding: '8px 16px', borderTop: '1px solid #333' }}>
          <button
            onClick={() => {
              if (!ikMode) {
                // Entrar en IK: target en posición actual del tool
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
      <RobotViewer robot={robot} gripper={gripper} workspacePoints={workspacePoints} ikTarget={ikTarget} onIkTargetChange={setIkTarget} />
    </div>
  );
}
