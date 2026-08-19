import { useMemo, useState, Suspense, Component } from 'react';
import { Canvas } from '@react-three/fiber';
import { OrbitControls } from '@react-three/drei';
import * as THREE from 'three';
import type { RobotDef, Mat4 } from '../kinematics/types';
import { forwardKinematics } from '../wasm';
import type { DebugToggles, FidelityMode, RobotRendererProps } from '../renderers/types';
import type { WorkspacePoints } from '../workspace/colors';
import { framePose, mulMat4 } from '../renderers/types';
import type { IkPathSplit } from '../motion/fkPath';
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

function RobotSceneDispatcher({ robot, rawFrames, gripper = 0, workspacePoints, tracePath, traceProgressRef, ikTracePath, ikTarget, onIkTargetChange, onDragStart, onDragEnd, fidelityMode, debugToggles, calibrationConfigRef, calibrationOverridesRef, calibrationTarget, calibrationMode, calibrationVersion, onCalibrationChange, gizmoMode, stlScaleRef }: {
  robot: RobotDef;
  rawFrames?: Mat4[];
  gripper?: number;
  workspacePoints?: WorkspacePoints;
  tracePath?: [number, number, number][];
  traceProgressRef?: React.MutableRefObject<number>;
  ikTracePath?: IkPathSplit;
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
    ikTracePath,
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

export default function RobotViewer({ robot, rawFrames, gripper = 0, workspacePoints, tracePath, traceProgressRef, ikTracePath, ikTarget, onIkTargetChange, fidelityMode = 'low', debugToggles, calibrationConfigRef, calibrationOverridesRef, calibrationTarget, calibrationMode, calibrationVersion, onCalibrationChange, gizmoMode, stlScaleRef }: {
  robot: RobotDef;
  rawFrames?: Mat4[];
  gripper?: number;
  workspacePoints?: WorkspacePoints;
  tracePath?: [number, number, number][];
  traceProgressRef?: React.MutableRefObject<number>;
  ikTracePath?: IkPathSplit;
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

  // Character-spotlight floor pool: an unlit radial-gradient decal so the lit
  // floor under the robot reads clearly from ANY camera angle (not only
  // top-down). Scene units are millimeters; radius ~300mm matches the robot's
  // footprint while staying inside the grid's 450mm fade. MeshBasicMaterial
  // (unlit) + depthWrite:false → it never occludes the robot or receives
  // shadows; renderOrder -1 keeps grid lines/trace crisp on top.
  const poolTexture = useMemo(() => {
    const size = 256;
    const canvas = document.createElement('canvas');
    canvas.width = size;
    canvas.height = size;
    const ctx = canvas.getContext('2d')!;
    const grad = ctx.createRadialGradient(size / 2, size / 2, 0, size / 2, size / 2, size / 2);
    grad.addColorStop(0.0, 'rgba(240, 252, 255, 0.92)'); // bright white-cyan core
    grad.addColorStop(0.35, 'rgba(0, 242, 254, 0.55)'); // design cyan #00F2FE
    grad.addColorStop(0.7, 'rgba(0, 242, 254, 0.20)');
    grad.addColorStop(1.0, 'rgba(0, 242, 254, 0.0)'); // soft transparent edge
    ctx.fillStyle = grad;
    ctx.fillRect(0, 0, size, size);
    const tex = new THREE.CanvasTexture(canvas);
    tex.colorSpace = THREE.SRGBColorSpace;
    return tex;
  }, []);

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
        {/* Studio lighting, HOMOGENEOUS by design (user feedback: one robot face
            always read dark). The original asymmetric key (1.5 + castShadow) cast a
            body shadow that blacked one face — ambient can't fix a blocked face.
            Now: the CHARACTER SPOTLIGHT is the main light, an OVERHEAD symmetric
            cone ([0,900,0], angle 0.85, penumbra 1.0, decay 0) that lights every
            side face equally; the three directionals are balanced to the same
            intensity (0.8, no castShadow) so they only add gentle form without any
            light/dark axis; ambient 0.55 + hemisphere 0.5 guarantee no face ever
            drops dark. WARM TONE on the spot + key (#fff1dc/#fff4e6) against the
            cool cyan/blue fills — a studio warm/cool split that makes the lighting
            read clearly on the robot. Shared Canvas level → benefits BOTH fidelity
            views. */}
        <ambientLight intensity={0.65} />
        <directionalLight position={[400, 600, 300]} intensity={1.1} color="#fff4e6" />
        <directionalLight position={[-350, 200, 250]} intensity={1.1} color="#00f2fe" />
        <directionalLight position={[0, 100, -500]} intensity={1.1} color="#6688ff" />
        <spotLight
          position={[0, 900, 0]}
          angle={0.85}
          penumbra={1.0}
          intensity={2.4}
          decay={0}
          distance={1100}
          color="#fff1dc"
        />
        <hemisphereLight args={['#8888ff', '#444422', 0.6]} />

        {/* Visible floor light pool under the robot (y just above the grid plane,
            rotated flat, centered on the robot's origin). Unlit decal → always reads
            as a lit patch of floor from any angle. */}
        <mesh position={[0, -0.45, 0]} rotation={[-Math.PI / 2, 0, 0]} renderOrder={-1}>
          <circleGeometry args={[300, 64]} />
          <meshBasicMaterial map={poolTexture} transparent depthWrite={false} />
        </mesh>

        <RobotSceneDispatcher
          robot={robot}
          rawFrames={rawFrames}
          gripper={gripper}
          workspacePoints={workspacePoints}
          tracePath={tracePath}
          traceProgressRef={traceProgressRef}
          ikTracePath={ikTracePath}
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
          // Navigation across the workspace: pan with right-drag (or two
          // fingers on touch) along the FLOOR plane, not the screen — the
          // camera glides over the drawing area. screenSpacePanning=false
          // keeps the pan on the ground plane so drawings stay in view.
          enablePan
          screenSpacePanning={false}
          panSpeed={1}
        />
      </Canvas>
    </div>
  );
}
