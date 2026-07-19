import { useState, useCallback, useRef, useEffect } from 'react';
import type { RobotDef, Segment } from './kinematics/types';
import { fabriCreator, fabriCreatorSegments } from './robot/fabri_creator';
import { qToServoDeg, buildWire, requestSerialPort, openPort, sendSerial } from './serial';
import RobotViewer from './components/RobotViewer';
import JointControls from './components/JointControls';
import InfoPanel from './components/InfoPanel';

export default function App() {
  const [robot, setRobot] = useState<RobotDef>(() => fabriCreator());
  const [gripper, setGripper] = useState(0);
  const [connected, setConnected] = useState(false);
  const [serialError, setSerialError] = useState<string | null>(null);
  const portRef = useRef<SerialPort | null>(null);

  // Enviar q al Arduino via serial
  const sendQ = useCallback((segments: Segment[], g: number) => {
    const port = portRef.current;
    if (!port) return;
    const q = segments.map(s => s.q);
    const servoDeg = qToServoDeg(q);
    const wire = buildWire(servoDeg, g);
    sendSerial(port, wire);
  }, []);

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

  // Enviar q cada vez que cambia
  useEffect(() => {
    sendQ(robot.segments, gripper);
  }, [robot.segments, gripper, sendQ]);

  const handleReset = useCallback(() => {
    const home = fabriCreator();
    setRobot(home);
    setGripper(0);
    // Forzar envío a home aunque ya estuviera en q=0
    sendQ(home.segments, 0);
  }, [sendQ]);

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
      <RobotViewer robot={robot} gripper={gripper} />
    </div>
  );
}
