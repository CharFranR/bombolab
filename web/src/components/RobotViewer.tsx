import { useMemo, useState, Suspense, Component } from 'react';
import { Canvas } from '@react-three/fiber';
import { OrbitControls } from '@react-three/drei';
import * as THREE from 'three';
import type { RobotDef, Mat4 } from '../kinematics/types';
import { forwardKinematics } from '../wasm';
import type { DebugToggles, FidelityMode, RobotRendererProps } from '../renderers/types';
import { framePose, mulMat4 } from '../renderers/types';
import SimpleRobotScene from '../renderers/SimpleRobotScene';
import StlRobotScene from '../renderers/StlRobotScene';

// ─── Error boundary for STL load failures ────────────────────────────────────

class StlErrorBoundary extends Component<{ children: React.ReactNode }, { hasError: boolean }> {
  constructor(props: { children: React.ReactNode }) {
    super(props);
    this.state = { hasError: false };
  }

  static getDerivedStateFromError(): { hasError: boolean } {
    return { hasError: true };
  }

  componentDidCatch(error: Error, info: React.ErrorInfo): void {
    console.error('[StlRobotScene] STL load failed:', error.message, info.componentStack);
  }

  render(): React.ReactNode {
    if (this.state.hasError) {
      return null; // graceful degradation — fall back to empty scene
    }
    return this.props.children;
  }
}

// ─── Dispatcher: FK precomputation + renderer branch ────────────────────────

function RobotSceneDispatcher({ robot, gripper = 0, workspacePoints = [], ikTarget, onIkTargetChange, onDragStart, onDragEnd, fidelityMode, debugToggles, calibrationConfigRef, calibrationOverridesRef, calibrationTarget, calibrationMode, calibrationVersion, onCalibrationChange, gizmoMode }: {
  robot: RobotDef;
  gripper?: number;
  workspacePoints?: [number, number, number][];
  ikTarget?: [number, number, number] | null;
  onIkTargetChange?: (pos: [number, number, number]) => void;
  onDragStart?: () => void;
  onDragEnd?: () => void;
  fidelityMode: FidelityMode;
  debugToggles?: DebugToggles;
  calibrationConfigRef?: React.MutableRefObject<Map<string, THREE.Matrix4>>;
  calibrationOverridesRef?: React.MutableRefObject<Map<string, THREE.Matrix4>>;
  calibrationTarget?: string | null;
  calibrationMode?: boolean;
  calibrationVersion?: number;
  onCalibrationChange?: () => void;
  gizmoMode?: 'translate' | 'rotate';
}) {
  // 1. Forward kinematics → raw Mat4 frames
  const { frames: rawFrames } = useMemo(
    () => forwardKinematics(robot.segments, robot.baseTransform),
    [robot.segments, robot.baseTransform],
  );

  // 2. Tool-transform matrix (translation along X only)
  const toolTransform: Mat4 = useMemo(() => [
    1, 0, 0, robot.toolTransform[0],
    0, 1, 0, 0,
    0, 0, 1, 0,
    0, 0, 0, 1,
  ], [robot.toolTransform]);

  // 3. Convert all FK frames + tool tip → FramePose[]
  const poses = useMemo(() => {
    // Tool-tip frame: last FK frame composed with tool transform
    const last = rawFrames[rawFrames.length - 1];
    const m = mulMat4(last, toolTransform);
    const toolTip = framePose(m);
    // All FK frames + tool tip appended at end
    const allPoses = rawFrames.map(framePose);
    allPoses.push(toolTip);
    return allPoses;
  }, [rawFrames, toolTransform]);

  const commonProps: RobotRendererProps = {
    frames: poses,
    gripper,
    workspacePoints,
    ikTarget,
    onIkTargetChange,
    onDragStart,
    onDragEnd,
    debugToggles,
    calibrationConfigRef,
    calibrationOverridesRef,
    calibrationTarget,
    calibrationMode,
    calibrationVersion,
    onCalibrationChange,
    gizmoMode,
  };

  // 4. Branch on fidelity mode (React-conditional → unmount/remount)
  if (fidelityMode === 'high') {
    return (
      <Suspense fallback={null}>
        <StlErrorBoundary>
          {calibrationMode && <SimpleRobotScene {...commonProps} />}
          <StlRobotScene {...commonProps} />
        </StlErrorBoundary>
      </Suspense>
    );
  }
  return <SimpleRobotScene {...commonProps} />;
}

// ─── Viewer principal ──────────────────────────────────────────────────────

export default function RobotViewer({ robot, gripper = 0, workspacePoints = [], ikTarget, onIkTargetChange, fidelityMode = 'low', debugToggles, calibrationConfigRef, calibrationOverridesRef, calibrationTarget, calibrationMode, calibrationVersion, onCalibrationChange, gizmoMode }: {
  robot: RobotDef;
  gripper?: number;
  workspacePoints?: [number, number, number][];
  ikTarget?: [number, number, number] | null;
  onIkTargetChange?: (pos: [number, number, number]) => void;
  fidelityMode: FidelityMode;
  debugToggles?: DebugToggles;
  calibrationConfigRef?: React.MutableRefObject<Map<string, THREE.Matrix4>>;
  calibrationOverridesRef?: React.MutableRefObject<Map<string, THREE.Matrix4>>;
  calibrationTarget?: string | null;
  calibrationMode?: boolean;
  calibrationVersion?: number;
  onCalibrationChange?: () => void;
  gizmoMode?: 'translate' | 'rotate';
}) {
  const [ikDragging, setIkDragging] = useState(false);
  return (
    <div style={{ flex: 1, height: '100%' }}>
      <Canvas
        shadows
        camera={{ position: [500, 400, 500], fov: 35, near: 1, far: 2000 }}
        gl={{ antialias: true }}
        onCreated={({ gl }) => {
          gl.setClearColor(new THREE.Color('#1c1c20'));
        }}
      >
        <ambientLight intensity={0.4} />
        <directionalLight position={[200, 400, 300]} intensity={1.2} castShadow />
        <directionalLight position={[-200, 100, -200]} intensity={0.3} />
        <hemisphereLight args={['#8888ff', '#444422', 0.3]} />

        <RobotSceneDispatcher
          robot={robot}
          gripper={gripper}
          workspacePoints={workspacePoints}
          ikTarget={ikTarget}
          onIkTargetChange={onIkTargetChange}
          onDragStart={() => setIkDragging(true)}
          onDragEnd={() => setIkDragging(false)}
          fidelityMode={fidelityMode}
          debugToggles={debugToggles}
          calibrationConfigRef={calibrationConfigRef}
          calibrationOverridesRef={calibrationOverridesRef}
          calibrationTarget={calibrationTarget}
          calibrationMode={calibrationMode}
          calibrationVersion={calibrationVersion}
          onCalibrationChange={onCalibrationChange}
          gizmoMode={gizmoMode}
        />

        <OrbitControls
          makeDefault
          enableDamping
          dampingFactor={0.1}
          minDistance={100}
          maxDistance={1200}
          target={[0, 200, 0]}
          enabled={!ikDragging}
        />
      </Canvas>
    </div>
  );
}
