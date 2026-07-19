import { useState, useCallback, useRef, useEffect } from 'react';
import type { RobotDef, Segment } from './kinematics/types';
import { fabriCreator, fabriCreatorSegments } from './robot/fabri_creator';
import RobotViewer from './components/RobotViewer';
import JointControls from './components/JointControls';
import InfoPanel from './components/InfoPanel';

const WS_URL = 'ws://127.0.0.1:8080';

export default function App() {
  const [robot, setRobot] = useState<RobotDef>(() => fabriCreator());
  const [gripper, setGripper] = useState(0);
  const [wsConnected, setWsConnected] = useState(false);
  const wsRef = useRef<WebSocket | null>(null);

  // WebSocket: enviar q al bridge
  const sendQ = useCallback((segments: Segment[], g: number) => {
    const ws = wsRef.current;
    if (!ws || ws.readyState !== WebSocket.OPEN) return;
    const q = segments.map(s => s.q);
    ws.send(JSON.stringify({ type: 'q', joints: q, gripper: Math.round(g * 1.8) }));
  }, []);

  const handleConnect = useCallback(() => {
    if (wsRef.current) return;
    const ws = new WebSocket(WS_URL);
    ws.onopen = () => {
      setWsConnected(true);
      // Enviar estado actual al conectar
      sendQ(robot.segments, gripper);
    };
    ws.onclose = () => {
      setWsConnected(false);
      wsRef.current = null;
    };
    ws.onerror = () => {
      setWsConnected(false);
      wsRef.current = null;
    };
    wsRef.current = ws;
  }, [robot.segments, gripper, sendQ]);

  const handleDisconnect = useCallback(() => {
    wsRef.current?.close();
    wsRef.current = null;
    setWsConnected(false);
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
    setRobot(fabriCreator());
    setGripper(0);
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
          />
        </div>

        {/* Info panel */}
        <InfoPanel robot={robot} />

        {/* Conexión robot físico */}
        <div style={{ padding: '8px 16px', borderTop: '1px solid #333' }}>
          <div style={{ display: 'flex', alignItems: 'center', gap: 8, marginBottom: 8 }}>
            <span style={{
              width: 8, height: 8, borderRadius: '50%',
              background: wsConnected ? '#4cd964' : '#666',
            }} />
            <span style={{ fontSize: 12, color: '#888' }}>
              {wsConnected ? 'Conectado' : 'Desconectado'}
            </span>
          </div>
          {wsConnected ? (
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
