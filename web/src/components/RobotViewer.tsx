import { useMemo, useState, Suspense, Component } from 'react';
import { Canvas } from '@react-three/fiber';
import { OrbitControls } from '@react-three/drei';
import * as THREE from 'three';
import type { RobotDef, Mat4 } from '../kinematics/types';
import { forwardKinematics } from '../wasm';
import type { DebugToggles, FidelityMode, RobotRendererProps } from '../renderers/types';
import type { WorkspacePoints } from '../workspace/colors';
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

function RobotSceneDispatcher({ robot, rawFrames, gripper = 0, workspacePoints, tracePath, traceProgressRef, ikTarget, onIkTargetChange, onDragStart, onDragEnd, fidelityMode, debugToggles, calibrationConfigRef, calibrationOverridesRef, calibrationTarget, calibrationMode, calibrationVersion, onCalibrationChange, gizmoMode, stlScaleRef }: {
  robot: RobotDef;
  rawFrames?: Mat4[];
  gripper?: number;
  workspacePoints?: WorkspacePoints;
  tracePath?: [number, number, number][];
  traceProgressRef?: React.MutableRefObject<number>;
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
  stlScaleRef?: React.MutableRefObject<number>;
}) {
  // 1. Forward kinematics → raw Mat4 frames.
  //    P2 (Stage 3C): App computa el FK una sola vez y lo distribuye;
  //    este dispatcher solo lo interpreta (convierte a poses) y lo
  //    transforma para el renderer. Fallback al cálculo interno si el
  //    prop rawFrames no viene (uso standalone del componente).
  const computedFrames = useMemo(
    () => forwardKinematics(robot.segments, robot.baseTransform).frames,
    [robot.segments, robot.baseTransform],
  );
  const frames: Mat4[] = rawFrames ?? computedFrames;

  const toolTransform: Mat4 = robot.toolTransform;

  // 3. Convert all FK frames + tool tip → FramePose[]
  const poses = useMemo(() => {
    // Tool-tip frame: last FK frame composed with tool transform
    const last = frames[frames.length - 1];
    const m = mulMat4(last, toolTransform);
    const toolTip = framePose(m);
    // All FK frames + tool tip appended at end
    const allPoses = frames.map(framePose);
    allPoses.push(toolTip);
    return allPoses;
  }, [frames, toolTransform]);

  const commonProps: RobotRendererProps = {
    frames: poses,
    gripper,
    workspacePoints,
    tracePath,
    traceProgressRef,
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
    stlScaleRef,
  };

  // 4. Branch on fidelity mode (React-conditional → unmount/remount)
  if (fidelityMode === 'high') {
    return (
      <Suspense fallback={null}>
        <StlErrorBoundary>
          <StlRobotScene {...commonProps} />
        </StlErrorBoundary>
      </Suspense>
    );
  }
  return <SimpleRobotScene {...commonProps} />;
}

// ─── Viewer principal ──────────────────────────────────────────────────────

export default function RobotViewer({ robot, rawFrames, gripper = 0, workspacePoints, tracePath, traceProgressRef, ikTarget, onIkTargetChange, fidelityMode = 'low', debugToggles, calibrationConfigRef, calibrationOverridesRef, calibrationTarget, calibrationMode, calibrationVersion, onCalibrationChange, gizmoMode, stlScaleRef }: {
  robot: RobotDef;
  rawFrames?: Mat4[];
  gripper?: number;
  workspacePoints?: WorkspacePoints;
  tracePath?: [number, number, number][];
  traceProgressRef?: React.MutableRefObject<number>;
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
  stlScaleRef?: React.MutableRefObject<number>;
}) {
  const [ikDragging, setIkDragging] = useState(false);
  return (
    <div style={{ flex: 1, height: '100%' }}>
      <Canvas
        shadows
        camera={{ position: [500, 400, 500], fov: 35, near: 1, far: 2000 }}
        gl={{ antialias: true, alpha: true }}
        onCreated={({ gl }) => {
          // Alpha 0: CSS gradient + vignette (slice 1) show through the canvas
          gl.setClearColor(0x000000, 0);
        }}
      >
        {/* Studio lighting: key (top-left, castShadow), cyan fill (right), rim (back) */}
        <ambientLight intensity={0.15} />
        <directionalLight position={[400, 600, 300]} intensity={2.2} castShadow />
        <directionalLight position={[-350, 200, 250]} intensity={0.9} color="#00f2fe" />
        <directionalLight position={[0, 100, -500]} intensity={1.4} color="#6688ff" />
        <hemisphereLight args={['#8888ff', '#444422', 0.15]} />

        <RobotSceneDispatcher
          robot={robot}
          rawFrames={rawFrames}
          gripper={gripper}
          workspacePoints={workspacePoints}
          tracePath={tracePath}
          traceProgressRef={traceProgressRef}
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
          stlScaleRef={stlScaleRef}
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
