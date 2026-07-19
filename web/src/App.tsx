import { useState, useCallback } from 'react';
import type { RobotDef, Segment } from './kinematics/types';
import { fabriCreator, fabriCreatorSegments } from './robot/fabri_creator';
import RobotViewer from './components/RobotViewer';
import JointControls from './components/JointControls';
import InfoPanel from './components/InfoPanel';

const DEG = 180 / Math.PI;

export default function App() {
  const [robot, setRobot] = useState<RobotDef>(() => fabriCreator());

  const handleJointChange = useCallback((index: number, qRad: number) => {
    setRobot(prev => {
      const segments = prev.segments.map((seg, i) => ({
        ...seg,
        q: i === index ? qRad : seg.q,
      }));
      return { ...prev, segments };
    });
  }, []);

  const handleReset = useCallback(() => {
    setRobot(fabriCreator());
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
            onChange={handleJointChange}
          />
        </div>

        {/* Info panel */}
        <InfoPanel robot={robot} />

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
      <RobotViewer robot={robot} />
    </div>
  );
}
