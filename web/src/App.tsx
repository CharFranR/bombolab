import { useState, useCallback, useRef, useEffect, useMemo, useReducer } from 'react';
import * as THREE from 'three';
import type { RobotDef, Segment } from './kinematics/types';
import { initWasm, fabriCreator, forwardKinematics, solveIk, solveDrawingIk, solveDrawingIkV2, solveDrawingPlaneIk, motionPlayerNew, motionPlayerPlay, motionPlayerPause, motionPlayerResume, motionPlayerStop, motionPlayerUpdate, motionPlayerState, motionPlayerTarget, motionPlayerProgress, motionPlayerDrop, samplerNew, sampleBatch, samplerStats, samplerDrop, type PlayerStateJs, type WorkspaceMode, type WorkspaceStats } from './wasm';
import { parseWorkspaceBatch, concatWorkspaceChunks, validateSampleCount, type WorkspacePoints } from './workspace/colors';
import { squareCommands, diagnosticLinesCommands, arcCommands, drawingPath, type MotionCommandJS } from './motion/commands';
import { parseGcode } from './motion/gcode';
import { validateDrawingCommands, safeDrawingArea, isReachablePoint, DRAW_PLANE_Z, TRAVEL_PLANE_Z, type ReachResult } from './motion/reachability';
import { runSingularityGate } from './motion/singularityGate';
import { qToServoUs, gripperToServoUs, servoDegToUs, encodeWire, requestSerialPort, openPort, sendSerial, handshakeV2, uploadManifest, continueManifestUpload } from './serial';
import { buildManifest, sliceLines, V2_CHUNK_MAX } from './motion/manifest';
import { firmwareTraceCsv, firmwareTraceStats, type FirmwareSample } from './motion/traceFirmware';
import { ServoInterpolator, type InterpolationConfig } from './interpolation';
import { TraceRecorder, type TraceResult } from './motion/trace';
import { planTimeline, type PlanSample } from './motion/planTimeline';
import { downloadTraceCsv } from './motion/csv';
import type { DebugToggles, FidelityMode, CalibrationConfig } from './renderers/types';
import { ALL_STL_FILES } from './renderers/stlMapping';
import RobotViewer from './components/RobotViewer';
import JointControls from './components/JointControls';
import InfoPanel from './components/InfoPanel';
import CalibrationPanel from './renderers/CalibrationPanel';
import ServoCalibAnalyzer from './components/ServoCalibAnalyzer';
import PlanExecPanel from './components/PlanExecPanel';
import { loadGcodeText, mapDrawFailureToErrorCode, type LoadGcodeTextResult } from './cipra/loadGcodeText';
import { jobReducer, initialJobState, queueFull, shouldCompleteCipraDraw, type CipraJob } from './cipra/jobStore';
import { GcodeClient, buildGcodeWsUrl, readEnvWsUrl, getConnectionStatusLabel, type CipraConnectionStatus } from './cipra';

function LoadingScreen({ error }: { error?: string }) {
  return (
    <div style={{ display: 'flex', width: '100%', height: '100%', background: 'linear-gradient(160deg, var(--bg0), var(--bg1))', color: 'var(--c-gray)', alignItems: 'center', justifyContent: 'center', flexDirection: 'column', gap: 12 }}>
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
  const [workspaceCount, setWorkspaceCount] = useState(10000);
  const [workspaceMode, setWorkspaceMode] = useState<WorkspaceMode>('drawing-plane');
  const [workspaceRunning, setWorkspaceRunning] = useState(false);
  const [workspaceProgress, setWorkspaceProgress] = useState(0);
  const [workspaceError, setWorkspaceError] = useState<string | null>(null);
  const [workspacePoints, setWorkspacePoints] = useState<WorkspacePoints | null>(null);
  const [workspaceStats, setWorkspaceStats] = useState<WorkspaceStats | null>(null);
  const workspaceCancelRef = useRef(false);
  const [ikMode, setIkMode] = useState(false);
  const [drawingMode, setDrawingMode] = useState(0); // 0=off, 1=modo1, 2=modo2
  const [drawingActive, setDrawingActive] = useState(false);
  const [ikTarget, setIkTarget] = useState<[number, number, number] | null>(null);
  const [ikError, setIkError] = useState<number | null>(null);
  // Robot operating mode: the enum has more variants (Teaching, Calibration,
  // EmergencyStop) but only Normal and Drawing are implemented (slice 1).
  const [robotMode, setRobotMode] = useState<'normal' | 'drawing'>('normal');
  const [transitioning, setTransitioning] = useState(false);
  const [playerId, setPlayerId] = useState<number | null>(null);
  const [playerState, setPlayerState] = useState<PlayerStateJs>('idle');
  const [demoSizeCm, setDemoSizeCm] = useState<number>(8);
  const [tracePath, setTracePath] = useState<[number, number, number][]>([]);
  const [traceResult, setTraceResult] = useState<TraceResult | null>(null);
  const [firmwareTrace, setFirmwareTrace] = useState<FirmwareSample[]>([]);
  const [tracePlan, setTracePlan] = useState<PlanSample[] | null>(null);
  const traceProgressRef = useRef(0);
  const [activeDemo, setActiveDemo] = useState<string | null>(null);
  const [gcodeName, setGcodeName] = useState<string | null>(null);
  const [gcodeWarnings, setGcodeWarnings] = useState<string[]>([]);
  const [gcodeError, setGcodeError] = useState<string | null>(null);
  const gcodeInputRef = useRef<HTMLInputElement | null>(null);
  // Last parsed gcode text + name, kept so the "Reajustar" button can re-autofit
  // into a smaller safe area if the current drawing is rejected by the workspace.
  const lastGcodeRef = useRef<{ name: string; text: string } | null>(null);
  // Workspace block: trajectory had points outside the robot reach. Playback
  // is blocked until the user dismisses / re-fits it. No forbidden movement is
  // ever commanded.
  const [drawingBlock, setDrawingBlock] = useState<{
    reason: string;
    points: [number, number, number][];
    canRefit: boolean;
  } | null>(null);
  const [validating, setValidating] = useState(false);
  const gripperBeforeModeRef = useRef(50);
  const lastFrameRef = useRef(0);
  const lastTrajectoryTargetRef = useRef<[number, number, number] | null>(null);
  // Calibración de servos (deadband/backlash) — manual, a paso del usuario.
  // Cada pulso envía un frame crudo (bypasa el interpolador); el usuario
  // marca "se movió / no se movió" y la app arma el log CSV.
  const [calibRunning, setCalibRunning] = useState(false);
  const [calibStatus, setCalibStatus] = useState('');
  const calibRunningRef = useRef(false);
  const [calibJoint, setCalibJoint] = useState(0);
  const [calibPose, setCalibPose] = useState<number[]>([90, 90, 81, 95, 60, 110]);
  const calibPoseRef = useRef<number[]>([90, 90, 81, 95, 60, 110]);
  const [calibAnalyzerOpen, setCalibAnalyzerOpen] = useState(false);
  const [calibLastMove, setCalibLastMove] = useState<{ joint: number; from: number; to: number } | null>(null);
  const [calibLog, setCalibLog] = useState<{ joint: number; from: number; to: number; moved: boolean }[]>([]);
  const portRef = useRef<SerialPort | null>(null);
  const manifestModeRef = useRef(false);
  const manifestAbortRef = useRef(false);
  const firmwareTraceRef = useRef<FirmwareSample[]>([]);
  const servoInterpolatorRef = useRef<ServoInterpolator | null>(null);
  const traceRecorderRef = useRef<TraceRecorder | null>(null);

  // Backlash take-up per channel — EXPERIMENTAL and DISABLED by default:
  // the A/B test showed a fixed 2°/1° compensation made the drawing WORSE
  // (over-compensation: the play is not constant). Values are in µs
  // (10.31 µs ≈ 1°): [2,2,2,1,1,0]° → [20.6, 20.6, 20.6, 10.3, 10.3, 0] µs.
  const BACKLASH_US: InterpolationConfig['backlash'] = [20.6, 20.6, 20.6, 10.3, 10.3, 0];
  const [backlashEnabled, setBacklashEnabled] = useState(false);
  const [fidelityMode, setFidelityMode] = useState<FidelityMode>('low');
  const [debugToggles, setDebugToggles] = useState<DebugToggles>({
    showJointFrames: false,
    showStlOrigins: false,
    showCalibrationAxes: false,
  });
  // DEBUG accordion — presentation-only UI state (precedent: calibAnalyzerOpen).
  const [debugOpen, setDebugOpen] = useState(false);

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
    // nothing to clean up on unmount (players are dropped with the UI)
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
    const servoUs = qToServoUs(segments.map(s => s.q));
    const target = [...servoUs, gripperToServoUs(g)];
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
      const initial = [...qToServoUs(robot.segments.map(s => s.q)), gripperToServoUs(gripper)];
      if (traceRecorderRef.current === null) traceRecorderRef.current = new TraceRecorder();
      servoInterpolatorRef.current = new ServoInterpolator(
        (wire) => {
          traceRecorderRef.current?.record(wire);
          sendSerial(port, wire);
        },
        initial,
        { stepSize: 5, delayMs: 50, backlash: backlashEnabled ? BACKLASH_US : undefined },
      );
      servoInterpolatorRef.current.keepAlive();
      setConnected(true);
    } catch (e: any) {
      setSerialError(e.message ?? 'Error al conectar');
    }
  }, [robot, gripper, backlashEnabled]);

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
    const solver = robotMode === 'drawing'
      ? (drawingMode === 1 ? solveDrawingIk : solveDrawingPlaneIk)
      : solveIk;
    const qInit = robot.segments.map(s => s.q);
    const result = solver(robot, ikTarget, qInit);
    setIkError(result.error);
    // Only apply the solution when it truly converges. A best-effort q from a
    // failed solve can land outside the workspace even when the table-backed
    // guard approved the target (edge/concavity); applying it silently bends
    // the arm to an illegal pose. When the solve fails we hold the last valid
    // pose instead — qInit stays on the last good q, so the solver resumes
    // cleanly once the target is reachable again.
    const ok = result.converged && result.error < 10;
    setDrawingActive(ok);
    if (ok) {
      setRobot(prev => {
        if (!prev) return prev;
        return { ...prev, segments: prev.segments.map((seg, i) => ({ ...seg, q: result.q[i] ?? 0 })) };
      });
    }
  }, [ikTarget, ikMode, drawingMode, robotMode]);

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
  // During calibration the heartbeat re-sends the last RAW calibration
  // pose instead — otherwise the failsafe would park mid-calibration.
  useEffect(() => {
    if (!connected) return;
    const id = setInterval(() => {
      if (manifestModeRef.current) return;
      if (calibRunningRef.current) {
        const port = portRef.current;
        const pose = calibPoseRef.current;
        if (port && pose) sendSerial(port, encodeWire(pose.map(servoDegToUs)));
      } else {
        servoInterpolatorRef.current?.keepAlive();
      }
    }, 1000);
    return () => clearInterval(id);
  }, [connected]);

  const handleReset = useCallback(() => {
    const home = fabriCreator();
    setRobot(home);
    setGripper(50);
    sendQRef.current(home.segments, 50);
  }, []);

  // ─── Robot modes: Normal / Drawing ─────────────────────────────────────
  // Entering Drawing commands the gripper to hold the marker (logical intent
  // interpreted by the controller) and waits for the interpolator queue to
  // drain — deterministic (frames × delay), not servo feedback.
  const enterDrawingMode = useCallback(async () => {
    if (robotMode === 'drawing' || transitioning) return;
    if (!window.confirm('¿El marcador ya está en el gripper? Apretá OK para cerrar la pinza y entrar en modo dibujo.')) return;
    gripperBeforeModeRef.current = gripper;
    setTransitioning(true);
    setRobotMode('drawing');
    setIkMode(true);
    setDrawingMode(2); // marcador vertical (solver restringido)
    setGripper(90);    // HoldMarker: pinza cerrada al 90%
    await servoInterpolatorRef.current?.whenIdle();
    setTransitioning(false);
  }, [robotMode, transitioning, gripper]);

  const exitDrawingMode = useCallback(() => {
    if (manifestModeRef.current) {
      sendSerial(portRef.current!, new TextEncoder().encode('STOP\n'));
      manifestModeRef.current = false;
      manifestAbortRef.current = true;
    }
    if (playerId !== null) {
      try { motionPlayerDrop(playerId); } catch {}
      setPlayerId(null);
    }
    setPlayerState('idle');
    lastTrajectoryTargetRef.current = null;
    setTracePath([]);
    setActiveDemo(null);
    setGcodeName(null);
    setGcodeWarnings([]);
    setGcodeError(null);
    lastGcodeRef.current = null;
    setDrawingBlock(null);
    traceRecorderRef.current?.discard();
    setTraceResult(null);
    setTracePlan(null);
    setValidating(false);
    setRobotMode('normal');
    setIkMode(false);
    setIkTarget(null);
    setGripper(gripperBeforeModeRef.current); // restaurar pinza
  }, [playerId]);

  // ─── Trajectory playback ───────────────────────────────────────────────
  // Drives the wasm motion player with frame deltas (rAF). Only pushes IK
  // targets when the commanded TCP moved enough to avoid solver/React churn.
  useEffect(() => {
    if (playerId === null) return;
    lastFrameRef.current = 0;
    let raf = 0;
    let stop = false;
    const loop = (now: number) => {
      if (stop) return;
      const dt = lastFrameRef.current > 0 ? Math.min((now - lastFrameRef.current) / 1000, 0.1) : 0.016;
      lastFrameRef.current = now;
      try {
        motionPlayerUpdate(playerId, dt);
        const st = motionPlayerState(playerId);
        setPlayerState(st);
        // Ref, not state: the renderer reads it in useFrame, so the heavy R3F
        // scene is NOT re-rendered on every frame delta.
        traceProgressRef.current = st === 'completed' ? 1 : motionPlayerProgress(playerId);
        if (st === 'completed' && manifestModeRef.current) {
          manifestModeRef.current = false;
          setFirmwareTrace([...firmwareTraceRef.current]);
        }
        if ((st === 'running' || st === 'paused') && !manifestModeRef.current) {
          const target = motionPlayerTarget(playerId);
          const last = lastTrajectoryTargetRef.current;
          if (!last || Math.hypot(target[0] - last[0], target[1] - last[1], target[2] - last[2]) > 0.5) {
            // Runtime safeguard: never let the IK solver chase a target outside
            // the reachable workspace. isReachablePoint is O(1) table-backed.
            if (!isReachablePoint(target)) {
              setDrawingBlock({
                reason:
                  'La trayectoria intentó un punto fuera del rango de trabajo ' +
                  '(x=' + target[0].toFixed(1) + ', y=' + target[1].toFixed(1) + '). ' +
                  'Reproducción detenida para evitar movimientos prohibidos.',
                points: [target],
                canRefit: false,
              });
              try { motionPlayerDrop(playerId); } catch {}
              setPlayerId(null);
              setPlayerState('idle');
              lastTrajectoryTargetRef.current = null;
              setIkTarget(null);
              stop = true;
              cancelAnimationFrame(raf);
              return;
            }
            lastTrajectoryTargetRef.current = target;
            setIkTarget(target);
          }
        }
      } catch (e) {
        console.error('[motion]', e);
      }
      if (!stop) raf = requestAnimationFrame(loop);
    };
    raf = requestAnimationFrame(loop);
    return () => {
      stop = true;
      cancelAnimationFrame(raf);
    };
  }, [playerId]);

  const finalizeTrace = useCallback(() => {
    const result = traceRecorderRef.current?.stop();
    if (result) setTraceResult(result);
  }, []);

  // Starts a drawing trajectory ONLY after a pre-flight reachability check
  // confirms every waypoint (and sampled mid-segment points) fits inside the
  // robot's drawing workspace. If any point is out of reach, playback is blocked
  // and `drawingBlock` is set — the robot never receives a forbidden target.
  // The movement logic itself (IK / motion player) is unchanged; this only
  // gates what it is fed.
  const startTrajectory = useCallback(async (cmds: MotionCommandJS[], key: string) => {
    if (transitioning || !robot || robotMode !== 'drawing') return false;
    // Replace any running/completed trajectory — the demo buttons must
    // always work; starting a new demo drops the previous player.
    if (playerId !== null) {
      try { motionPlayerDrop(playerId); } catch {}
    }
    setDrawingBlock(null);
    setValidating(true);
    let reach: ReachResult | null = null;
    try {
      reach = await validateDrawingCommands(cmds, {
        onProgress: (done, total) => {
          if (done === total) setValidating(false);
        },
      });
    } catch (e: any) {
      setValidating(false);
      setDrawingBlock({
        reason: 'No se pudo validar la trayectoria: ' + (e?.message ?? String(e)),
        points: [],
        canRefit: false,
      });
      return false;
    }
    setValidating(false);
    const canRefit = key === 'gcode' && (lastGcodeRef.current?.name ?? gcodeName) != null;

    if (!reach.ok) {
      const isDemo = key !== 'gcode';
      setDrawingBlock({
        reason:
          'La trayectoria contiene ' +
          reach.failures.length +
          ' punto(s) fuera del rango de trabajo del robot. No se dibuja para evitar movimientos prohibidos.' +
          (isDemo ? ' Prueba con otro tamaño o posición del demo.' : ''),
        points: reach.failures,
        canRefit,
      });
      return false;
    }

    const gateProceed = await runSingularityGate(robot, cmds, {
      canRefit,
      confirmFn: (message) => window.confirm(message + '\n\n¿Dibujar de todos modos?'),
      setDrawingBlock,
      setGcodeWarnings: (updater) => setGcodeWarnings(updater),
    });
    if (!gateProceed) return false;

    // Start the trajectory from the robot's current tool-tip pose (the TCP),
    // NOT the base. fk.ee is the tool pose (frame_last * tool_transform); in the
    // row-major Mat4 returned by forwardKinematics the translation lives in
    // [3],[7],[11]. Reading [12..14] instead yields the affine last row [0,0,1],
    // i.e. base origin, which made demos climb diagonally from the base.
    const fk = forwardKinematics(robot.segments, robot.baseTransform);
    const tip = fk.ee;
    const start: [number, number, number] = [tip[3], tip[7], tip[11]];
    setTracePath(drawingPath(cmds).map(robotToThree));
    traceProgressRef.current = 0;
    setActiveDemo(key);
    const samples = planTimeline(cmds, {
      ik: drawingMode === 1 ? solveDrawingIk : solveDrawingPlaneIk,
      robot,
      startQ: robot.segments.map((s) => s.q),
      gripperPct: gripper,
      startTcp: start,
    });
    setTracePlan(samples);
    const id = motionPlayerNew(cmds, start);
    // Review fix #2: remember the playback id THIS trajectory created; the
    // CIPRA draw path copies it into cipraDrawPlayerIdRef after a successful
    // start so a later `completed` state can be attributed to the right job.
    lastStartedPlayerIdRef.current = id;
    setPlayerId(id);
    motionPlayerPlay(id);
    traceRecorderRef.current?.start();
    setTraceResult(null);
    setPlayerState('running');
    const port = portRef.current;
    if (port && connected) {
      manifestAbortRef.current = false;
      const built = buildManifest(samples);
      if (built instanceof Error) {
        console.warn('[manifest] build fallback legacy:', built.message);
      } else {
        try {
          const chunkMax = await handshakeV2(port);
          if (chunkMax instanceof Error) {
            console.warn('[manifest] handshake fallback legacy:', chunkMax.message);
          } else {
            const chunks = sliceLines(built.lines, chunkMax);
            const upload = await uploadManifest(port, chunks, chunkMax);
            if (upload.error) {
              setDrawingBlock({
                reason: 'El firmware rechazó el manifest: ' + upload.error,
                points: [],
                canRefit: false,
              });
              try { motionPlayerDrop(id); } catch {}
              setPlayerId(null);
              setPlayerState('idle');
              return false;
            }
            sendSerial(port, new TextEncoder().encode('EXECUTE\n'));
            manifestModeRef.current = true;
            firmwareTraceRef.current = [];
            const remaining = built.lines.slice(upload.sent);
            void (async () => {
              const res = await continueManifestUpload(
                port,
                remaining,
                undefined,
                (tUs, joints) => {
                  firmwareTraceRef.current.push({ tUs, joints });
                  if (firmwareTraceRef.current.length > 100000) {
                    firmwareTraceRef.current.shift();
                  }
                },
              );
              if (res.error && !manifestAbortRef.current) {
                setDrawingBlock({
                  reason: 'El firmware abortó el manifest: ' + res.error,
                  points: [],
                  canRefit: false,
                });
                try { motionPlayerDrop(id); } catch {}
                setPlayerId(null);
                setPlayerState('idle');
              }
            })();
          }
        } catch (e) {
          console.warn('[manifest] fallback legacy:', e);
        }
      }
    }
    setIkTarget(start); // mantener la pose actual hasta el primer waypoint
    return true;
  }, [playerId, transitioning, robot, robotMode, drawingMode, gripper, gcodeName, connected]);

  // Shared "gcode text → validate → draw" pipeline (R12): the .gcode file
  // picker and the CIPRA arrival "Dibujar" action both go through this so
  // reachability/robot-mode gating is never duplicated.
  const runLoadGcodeText = useCallback(
    (text: string, name: string) =>
      loadGcodeText(text, name, {
        safeDrawingArea: () => safeDrawingArea(DRAW_PLANE_Z),
        parseGcode,
        startTrajectory,
        setValidating,
        setGcodeError,
        setGcodeWarnings,
        setGcodeName,
      }),
    [startTrajectory],
  );

  const handleStartDemo = useCallback(() => {
    void (async () => {
      const half = (demoSizeCm * 10) / 2; // 5×5 → half 25; 8×8 → half 40
      await startTrajectory(squareCommands(200, 0, 80, half), 'square');
    })();
  }, [startTrajectory, demoSizeCm]);

  const handleStartDiagnostic = useCallback(() => {
    void startTrajectory(diagnosticLinesCommands(), 'lines');
  }, [startTrajectory]);

  const handleStartArc = useCallback(() => {
    void startTrajectory(arcCommands(), 'arc');
  }, [startTrajectory]);

  const handleGcodeFile = useCallback((file: File) => {
    const reader = new FileReader();
    reader.onload = () => {
      void (async () => {
        try {
          const text = String(reader.result);
          lastGcodeRef.current = { name: file.name, text };
          await runLoadGcodeText(text, file.name);
        } catch (err: any) {
          setValidating(false);
          setGcodeError(err?.message ?? 'Error al leer el archivo .gcode');
        }
      })();
    };
    reader.onerror = () => setGcodeError('Error al leer el archivo');
    reader.readAsText(file);
  }, [runLoadGcodeText]);

  // Re-parse a rejected gcode against a progressively smaller safe drawing area
  // (up to N attempts) so the user can recover without leaving drawing mode.
  const handleRefitGcode = useCallback(async () => {
    const raw = lastGcodeRef.current;
    if (!raw) return;
    setValidating(true);
    const baseArea = await safeDrawingArea(DRAW_PLANE_Z);
    try {
      let area = baseArea;
      for (let attempt = 0; attempt < 10; attempt++) {
        const margin = 0.88;
        const cx = (area.xMin + area.xMax) / 2;
        const cy = (area.yMin + area.yMax) / 2;
        const hx = ((area.xMax - area.xMin) / 2) * margin;
        const hy = ((area.yMax - area.yMin) / 2) * margin;
        area = { xMin: cx - hx, xMax: cx + hx, yMin: cy - hy, yMax: cy + hy };
        const result = parseGcode(raw.text, { area, planeZ: DRAW_PLANE_Z, travelZ: TRAVEL_PLANE_Z });
        if (result.commands.length === 0) continue;
        setGcodeWarnings(result.warnings);
        const ok = await startTrajectory(result.commands, 'gcode');
        if (ok) return;
      }
      setGcodeError('No se pudo ajustar el dibujo al área de trabajo alcanzable.');
    } finally {
      setValidating(false);
    }
  }, [startTrajectory]);

  // ─── CIPRA WebSocket subscriber (R10/R13/R15) ──────────────────────────
  // Jobs live in a pure reducer. The client ACKs validated arrivals itself;
  // App only decides ACCEPT/DRAW/DISCARD through the Dibujar/Descartar panel.
  // The ALERT banner is purely informational — it never auto-starts a job.
  const [cipraJobs, cipraDispatch] = useReducer(jobReducer, initialJobState);
  const [cipraConn, setCipraConn] = useState<CipraConnectionStatus>('disconnected');
  const [cipraNoticeDismissed, setCipraNoticeDismissed] = useState(false);
  const cipraClientRef = useRef<GcodeClient | null>(null);
  // Playback id created by the most recent motionPlayerNew (set inside
  // startTrajectory). handleDrawCipraJob copies it into cipraDrawPlayerIdRef
  // after a successful draw, so the binding is synchronous with the start.
  const lastStartedPlayerIdRef = useRef<number | null>(null);
  // Motion-player id bound to the CURRENTLY DRAWING CIPRA job (review fix #2):
  // COMPLETE only fires when the completed playback IS the one this job
  // started. Cleared on new draw start, FAIL and discard.
  const cipraDrawPlayerIdRef = useRef<number | null>(null);
  // Latest queue state readable from the WS client callbacks (they are mounted
  // once with an empty closure); the queue-full gate needs current state.
  const cipraJobsRef = useRef(cipraJobs);
  useEffect(() => {
    cipraJobsRef.current = cipraJobs;
  }, [cipraJobs]);

  useEffect(() => {
    const client = new GcodeClient(
      buildGcodeWsUrl(
        { protocol: window.location.protocol, hostname: window.location.hostname },
        readEnvWsUrl(),
      ),
    );
    client.onStatus = setCipraConn;
    // Review fix #1: when the queue is at MAX_PENDING_JOBS the client replies
    // E_QUEUE_FULL to the publisher and does NOT surface the arrival here; the
    // reducer's own queueFull guard stays as the hard invariant.
    client.canAcceptJob = () => !queueFull(cipraJobsRef.current);
    client.onReady = (env) => {
      cipraDispatch({
        type: 'ARRIVE',
        job: { id: env.id, name: env.name || env.id, payload: env.payload },
      });
    };
    client.connect();
    cipraClientRef.current = client;
    return () => {
      client.disconnect();
      cipraClientRef.current = null;
    };
  }, []);

  // A fresh arrival re-arms the banner so a new job is always announced.
  useEffect(() => {
    if (cipraJobs.lastNotice) setCipraNoticeDismissed(false);
  }, [cipraJobs.lastNotice?.jobId]);

  // Trajectory finished → mark the single active cipra job completed (R8).
  // Review fix #2: only when the finished playback IS the one the job started
  // (playerId === cipraDrawPlayerIdRef). A demo/file/refit playback finishing
  // must not complete a CIPRA job that is not actually playing it.
  useEffect(() => {
    if (playerState === 'completed') finalizeTrace();
    if (shouldCompleteCipraDraw(cipraJobs, playerState, playerId, cipraDrawPlayerIdRef.current)) {
      cipraDispatch({ type: 'COMPLETE', id: cipraJobs.drawingId as string });
      cipraDrawPlayerIdRef.current = null;
    }
  }, [cipraJobs, playerState, playerId, finalizeTrace]);

  const handleClearDrawingBlock = useCallback(() => {
    if (manifestModeRef.current) {
      sendSerial(portRef.current!, new TextEncoder().encode('STOP\n'));
      manifestModeRef.current = false;
      manifestAbortRef.current = true;
      setFirmwareTrace([...firmwareTraceRef.current]);
    }
    setDrawingBlock(null);
    setTracePath([]);
    traceRecorderRef.current?.discard();
    setTraceResult(null);
    setTracePlan(null);
    if (playerId !== null) {
      try { motionPlayerDrop(playerId); } catch {}
      setPlayerId(null);
    }
    setPlayerState('idle');
    setIkTarget(null);
  }, [playerId]);

  const failCipraDraw = useCallback(
    (jobId: string, reason: LoadGcodeTextResult['reason'] | 'exception') => {
      // Review fix #3: a failed draw must NOT strand the job in `drawing`.
      // FAIL moves it back to pending (selectable again) and frees the
      // single-active guard; the drawing UI is cleaned exactly like
      // handleClearDrawingBlock so no partial state survives the failure.
      cipraDispatch({ type: 'FAIL', id: jobId });
      cipraDrawPlayerIdRef.current = null; // no playback binding survives a FAIL
      handleClearDrawingBlock();
      const code = mapDrawFailureToErrorCode(reason);
      setGcodeError(
        reason === 'blocked'
          ? 'No se pudo dibujar: la trayectoria queda fuera del área alcanzable. El trabajo volvió a la cola.'
          : 'No se pudo dibujar: el G-Code no se pudo procesar. El trabajo volvió a la cola.',
      );
      // Review fix #5: the ACK already confirmed DELIVERY — this error tells
      // the publisher WHY the job could not be drawn.
      cipraClientRef.current?.sendError(code, jobId);
    },
    [handleClearDrawingBlock],
  );

  const handleDrawCipraJob = useCallback(
    async (job: CipraJob) => {
      if (cipraJobs.drawingId !== null || transitioning) return; // single-active (R9)
      cipraDrawPlayerIdRef.current = null; // fresh draw attempt — no stale binding
      cipraDispatch({ type: 'ACCEPT', id: job.id });
      cipraDispatch({ type: 'DRAW', id: job.id });
      lastGcodeRef.current = { name: job.name, text: job.payload };
      let result: LoadGcodeTextResult | { ok: false; reason: 'exception' };
      try {
        result = await runLoadGcodeText(job.payload, job.name);
      } catch (err) {
        // Review fix #3: a thrown parse/validation error is a draw failure too.
        result = { ok: false, reason: 'exception' };
      }
      if (result.ok) {
        // Review fix #2: bind COMPLETE to THIS playback — the motion player id
        // created by this draw is what a later `completed` state must match.
        cipraDrawPlayerIdRef.current = lastStartedPlayerIdRef.current;
      } else {
        failCipraDraw(job.id, result.reason);
      }
    },
    [cipraJobs.drawingId, transitioning, runLoadGcodeText],
  );

  const handleDiscardCipraJob = useCallback(
    (id: string) => {
      if (cipraJobs.drawingId === id) {
        // Review fix #4 (user chose STOP): discarding a job that is DRAWING
        // stops its playback via motionPlayerDrop and clears the drawing
        // block/trace/player state exactly like handleClearDrawingBlock; the
        // COMPLETE capture is unbound so no stale completion can fire after
        // the discard. Not drawing → plain DISCARD below.
        handleClearDrawingBlock();
        cipraDrawPlayerIdRef.current = null;
      }
      cipraDispatch({ type: 'DISCARD', id });
    },
    [cipraJobs.drawingId, handleClearDrawingBlock],
  );

  const cipraPanelJobs = useMemo(
    () => cipraJobs.jobs.filter((j) => j.status !== 'completed' && j.status !== 'discarded'),
    [cipraJobs.jobs],
  );

  const handleBacklashToggle = useCallback((enabled: boolean) => {
    setBacklashEnabled(enabled);
    servoInterpolatorRef.current?.setBacklash(enabled ? BACKLASH_US : undefined);
  }, []);

  const handlePlaybackControl = useCallback(() => {
    if (playerId === null) return;
    try {
      if (manifestModeRef.current) {
        sendSerial(portRef.current!, new TextEncoder().encode('STOP\n'));
        manifestModeRef.current = false;
        manifestAbortRef.current = true;
      }
      if (playerState === 'running') {
        motionPlayerPause(playerId);
        setPlayerState('paused');
        finalizeTrace();
      } else if (playerState === 'paused') {
        motionPlayerResume(playerId);
        setPlayerState('running');
      } else {
        motionPlayerPlay(playerId);
        traceRecorderRef.current?.start();
        setTraceResult(null);
        setPlayerState('running');
      }
    } catch (e) {
      console.error('[motion]', e);
    }
  }, [playerId, playerState, finalizeTrace]);

  const handleStopDemo = useCallback(() => {
    if (playerId === null) return;
    try {
      if (manifestModeRef.current) {
        sendSerial(portRef.current!, new TextEncoder().encode('STOP\n'));
        manifestModeRef.current = false;
        manifestAbortRef.current = true;
        setFirmwareTrace([...firmwareTraceRef.current]);
      }
      motionPlayerStop(playerId);
      setPlayerState('stopped');
      finalizeTrace();
    } catch (e) {
      console.error('[motion]', e);
    }
  }, [playerId, finalizeTrace]);

  const handleExportTrace = useCallback(() => {
    if (traceResult === null || traceResult.samples.length === 0) return;
    downloadTraceCsv(traceResult);
  }, [traceResult]);

  const handleExportFirmwareTrace = useCallback(() => {
    if (firmwareTrace.length === 0) return;
    const csv = firmwareTraceCsv(firmwareTrace);
    const blob = new Blob([csv], { type: 'text/csv' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = 'firmware-trace.csv';
    a.click();
    URL.revokeObjectURL(url);
  }, [firmwareTrace]);

  // ─── Servo calibration (deadband / backlash) — manual mode ─────────────
  // User-paced: each button press sends ONE raw 1° step (bypassing the
  // interpolator, heartbeat paused); the user watches and marks whether the
  // servo moved. The app records the verdicts into a CSV log.
  const SERVO_NAMES = ['J1 yaw', 'J2 shoulder', 'J3 elbow', 'J4 roll', 'J5 pitch', 'Gripper'];
  const stepBtn: React.CSSProperties = {
    padding: '6px 10px',
    background: '#3a3a3a',
    border: 'none',
    borderRadius: 4,
    color: '#ccc',
    fontSize: 12,
    cursor: 'pointer',
  };

  const enterCalibration = useCallback(async () => {
    const port = portRef.current;
    if (!port || calibRunning) return;
    if (manifestModeRef.current) {
      sendSerial(port, new TextEncoder().encode('STOP\n'));
      manifestModeRef.current = false;
      manifestAbortRef.current = true;
    }
    // Stop any running trajectory.
    if (playerId !== null) {
      try { motionPlayerDrop(playerId); } catch {}
      setPlayerId(null);
      setPlayerState('idle');
    }
    // Smooth return to home first (the calibration pose holds the others there).
    const home = fabriCreator();
    setRobot(home);
    setGripper(50);
    sendQRef.current(home.segments, 50);
    await servoInterpolatorRef.current?.whenIdle();

    const homeServo = [90, 90, 81, 95, 60, 110]; // degrees (calibration UI)
    calibRunningRef.current = true;
    setCalibRunning(true);
    setCalibPose(homeServo);
    calibPoseRef.current = homeServo;
    setCalibLog([]);
    setCalibLastMove(null);
    setCalibStatus('Modo calibración: elegí joint, pulsá ±1° y marcá si se movió.');
    sendSerial(port, encodeWire(homeServo.map(servoDegToUs)));
    servoInterpolatorRef.current?.sync(homeServo.map(servoDegToUs));
  }, [calibRunning, playerId]);

  const exitCalibration = useCallback(() => {
    const port = portRef.current;
    const homeServo = [90, 90, 81, 95, 60, 110]; // degrees
    if (port) sendSerial(port, encodeWire(homeServo.map(servoDegToUs)));
    servoInterpolatorRef.current?.sync(homeServo.map(servoDegToUs));
    calibRunningRef.current = false;
    setCalibRunning(false);
    setCalibLastMove(null);
    setCalibStatus('');
  }, []);

  const calibStep = useCallback((delta: number) => {
    const port = portRef.current;
    if (!port || !calibRunning) return;
    const pose = [...calibPose];
    const from = pose[calibJoint];
    const to = Math.max(5, Math.min(175, from + delta));
    if (to === from) return;
    pose[calibJoint] = to;
    setCalibPose(pose);
    calibPoseRef.current = pose;
    setCalibLastMove({ joint: calibJoint, from, to });
    setCalibStatus(`${SERVO_NAMES[calibJoint]}: ${from}° → ${to}° — ¿se movió?`);
    sendSerial(port, encodeWire(pose.map(servoDegToUs)));
    servoInterpolatorRef.current?.sync(pose.map(servoDegToUs));
  }, [calibRunning, calibPose, calibJoint]);

  const calibRecord = useCallback((moved: boolean) => {
    if (!calibLastMove) return;
    setCalibLog((prev) => [...prev, { ...calibLastMove, moved }]);
    setCalibLastMove(null);
    setCalibStatus('Anotado. Mandá el siguiente paso.');
  }, [calibLastMove]);

  const downloadCalibLog = useCallback(() => {
    const lines = ['joint,from,to,moved'];
    for (const e of calibLog) {
      lines.push(`${e.joint + 1},${e.from},${e.to},${e.moved ? 'si' : 'no'}`);
    }
    const blob = new Blob([lines.join('\n')], { type: 'text/csv' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = 'servo-calibration.csv';
    a.click();
    URL.revokeObjectURL(url);
  }, [calibLog]);

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

  const cancelWorkspace = useCallback(() => {
    workspaceCancelRef.current = true;
  }, []);

  const runWorkspace = useCallback(async () => {
    const validationError = validateSampleCount(workspaceCount);
    if (validationError) {
      setWorkspaceError(validationError);
      return;
    }
    workspaceCancelRef.current = false;
    setWorkspaceRunning(true);
    setWorkspaceError(null);
    setWorkspaceStats(null);
    setWorkspacePoints(null);
    setWorkspaceProgress(0);
    let samplerId: number | null = null;
    try {
      samplerId = samplerNew(Date.now(), workspaceMode);
      const chunks: WorkspacePoints[] = [];
      const chunkSize = 1000;
      for (let done = 0; done < workspaceCount; done += chunkSize) {
        if (workspaceCancelRef.current) break;
        const k = Math.min(chunkSize, workspaceCount - done);
        chunks.push(parseWorkspaceBatch(sampleBatch(samplerId, k)));
        setWorkspaceProgress(Math.round(((done + k) / workspaceCount) * 100));
        await new Promise((resolve) => setTimeout(resolve, 0));
      }
      if (workspaceCancelRef.current) {
        setWorkspaceProgress(0);
        return;
      }
      setWorkspacePoints(chunks.length > 0 ? concatWorkspaceChunks(chunks) : null);
      setWorkspaceStats(samplerStats(samplerId));
    } catch (e) {
      setWorkspaceError(e instanceof Error ? e.message : String(e));
    } finally {
      if (samplerId !== null) {
        samplerDrop(samplerId);
      }
      setWorkspaceRunning(false);
    }
  }, [workspaceCount, workspaceMode]);

  // P2 (Stage 3C): FK calculado UNA vez en App y distribuido a los
  // consumidores (RobotViewer + InfoPanel). App no interpreta ni modifica
  // la cinemática — solo comparte el resultado crudo.
  const rawFrames = useMemo(
    () => robot ? forwardKinematics(robot.segments, robot.baseTransform).frames : [],
    [robot],
  );

  // EE-1: base style for the Fidelity segmented options (`.segmented` container
  // + `.segmented__opt--active` class in theme.css for the active option).
  const segmentOptStyle: React.CSSProperties = {
    padding: '4px 14px',
    border: '1px solid transparent',
    borderRadius: 999,
    background: 'transparent',
    color: 'var(--c-gray)',
    fontSize: 12,
    fontWeight: 600,
    fontFamily: 'var(--font-sans)',
    cursor: 'pointer',
    transition: 'color 0.2s ease, background 0.2s ease, border-color 0.2s ease',
  };

  if (!ready || !robot) return <LoadingScreen error={loadError ?? undefined} />;

  return (
    <div style={{ display: 'flex', width: '100%', height: '100%', background: 'linear-gradient(160deg, var(--bg0), var(--bg1))', color: '#ccc' }}>
      {/* Sidebar — floating glass column (D9): position clears the 44px fixed
          top bar (top 56) and the centered pill bar (bottom 92). All child
          config UIs below keep their exact order, content and wiring. */}
      <div
        className="glass-card"
        style={{
          position: 'fixed',
          top: 56,
          left: 16,
          bottom: 92,
          width: 280,
          minWidth: 280,
          display: 'flex',
          flexDirection: 'column',
          overflow: 'hidden',
          zIndex: 10,
        }}
      >
        {/* Header — panel label only: the brand already lives in the top bar,
            so the redundant sidebar h1 was removed (dedup). */}
        <div style={{ padding: '16px', borderBottom: '1px solid #333' }}>
          <p style={{ fontSize: 13, color: '#aaa', margin: 0 }}>
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
            <div
              className="debug-acc"
              onClick={() => setDebugOpen(!debugOpen)}
              style={{
                display: 'flex',
                alignItems: 'center',
                gap: 6,
                fontSize: 11,
                color: '#888',
                marginBottom: debugOpen ? 6 : 0,
              }}
            >
              <span style={{
                display: 'inline-block',
                transform: debugOpen ? 'rotate(90deg)' : 'rotate(0deg)',
                transition: 'transform 0.15s ease',
              }}>&gt;</span>
              DEBUG
            </div>
            {debugOpen && (
              <>
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
              </>
            )}
          </div>
        )}

        {/* Conexión robot físico (WebSerial) — the dot + Conectado/Desconectado
            row was removed (top bar badge shows it); this block now renders the
            serialError feedback ONLY, and only when an error exists. */}
        {serialError && (
          <div style={{ padding: '8px 16px', borderTop: '1px solid #333' }}>
            <div style={{ fontSize: 11, color: '#e55', marginBottom: 6 }}>{serialError}</div>
          </div>
        )}

        {/* Calibración de servos (deadband/backlash) — manual */}
        {calibRunning && (
          <div style={{ padding: '8px 16px', borderTop: '1px solid #333' }}>
            <div style={{ fontSize: 11, color: '#aa8', marginBottom: 6 }}>{calibStatus}</div>
              <div style={{ display: 'flex', gap: 4, marginBottom: 6, flexWrap: 'wrap' }}>
                {SERVO_NAMES.map((n, i) => (
                  <button
                    key={i}
                    onClick={() => setCalibJoint(i)}
                    style={{
                      flex: 1,
                      minWidth: 60,
                      padding: '4px 2px',
                      fontSize: 10,
                      background: calibJoint === i ? '#553' : '#3a3a3a',
                      border: '1px solid ' + (calibJoint === i ? '#885' : '#444'),
                      borderRadius: 3,
                      color: calibJoint === i ? '#ddc' : '#888',
                      cursor: 'pointer',
                    }}
                  >
                    {n}
                  </button>
                ))}
              </div>
              <div style={{ display: 'flex', gap: 6, alignItems: 'center', marginBottom: 6 }}>
                <button
                  onClick={() => calibStep(-5)}
                  style={stepBtn}
                >
                  −5°
                </button>
                <button onClick={() => calibStep(-1)} style={stepBtn}>−1°</button>
                <span style={{ fontSize: 13, fontFamily: 'monospace', color: '#ccc', minWidth: 40, textAlign: 'center' }}>
                  {calibPose[calibJoint]}°
                </span>
                <button onClick={() => calibStep(1)} style={stepBtn}>+1°</button>
                <button onClick={() => calibStep(5)} style={stepBtn}>+5°</button>
              </div>
              {calibLastMove && (
                <div style={{ display: 'flex', gap: 6, marginBottom: 6 }}>
                  <button
                    onClick={() => calibRecord(true)}
                    style={{ ...stepBtn, background: '#464', flex: 1, padding: 8 }}
                  >
                     Se movió
                  </button>
                  <button
                    onClick={() => calibRecord(false)}
                    style={{ ...stepBtn, background: '#633', flex: 1, padding: 8 }}
                  >
                     No se movió
                  </button>
                </div>
              )}
              <div style={{ display: 'flex', gap: 6 }}>
                <button
                  onClick={downloadCalibLog}
                  disabled={calibLog.length === 0}
                  style={{ ...stepBtn, flex: 1 }}
                >
                  Descargar CSV ({calibLog.length})
                </button>
                <button
                  onClick={() => setCalibAnalyzerOpen(!calibAnalyzerOpen)}
                  style={{ ...stepBtn, flex: 1, background: calibAnalyzerOpen ? '#553' : '#3a3a3a' }}
                >
                  {calibAnalyzerOpen ? 'Ocultar análisis' : 'Analizar'}
                </button>
                <button onClick={() => setCalibLog([])} style={stepBtn}>Limpiar</button>
                <button onClick={exitCalibration} style={{ ...stepBtn, background: '#633' }}>Salir</button>
              </div>
              {calibAnalyzerOpen && <ServoCalibAnalyzer log={calibLog} />}
            </div>
          )}
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

            </div>
            <div style={{ padding: '0 16px 4px', fontSize: 10, color: '#555' }}>
              Rueda mouse: sube/baja Z
            </div>
          </>
        )}

        {/* IK target readout — the IK Mode toggle itself now lives in the
            bottom pill bar (same inline toggle logic, moved) */}
        {ikMode && (
          <div style={{ padding: '8px 16px' }}>
            {ikTarget && (
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
        )}

        {/* Run Analysis */}
        <div style={{ padding: '8px 16px', borderTop: '1px solid #333' }}>
          <div style={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between', marginBottom: 6 }}>
            <span style={{ fontSize: 11, color: '#888' }}>Análisis de workspace</span>
            {workspaceRunning && (
              <button
                onClick={cancelWorkspace}
                style={{
                  padding: '2px 8px',
                  fontSize: 11,
                  background: '#553',
                  border: 'none',
                  borderRadius: 3,
                  color: '#dc8',
                  cursor: 'pointer',
                }}
              >
                Cancelar
              </button>
            )}
          </div>
          <div style={{ fontSize: 10, color: '#777', marginBottom: 4 }}>N muestras</div>
          <div style={{ display: 'flex', gap: 4, marginBottom: 8 }}>
            {[1000, 5000, 10000, 50000].map((n) => (
              <button
                key={n}
                onClick={() => setWorkspaceCount(n)}
                style={{
                  flex: 1,
                  padding: '3px 0',
                  fontSize: 11,
                  background: workspaceCount === n ? '#553' : '#3a3a3a',
                  border: '1px solid ' + (workspaceCount === n ? '#885' : '#444'),
                  borderRadius: 3,
                  color: '#ccc',
                  cursor: 'pointer',
                }}
              >
                {n / 1000}k
              </button>
            ))}
          </div>
          <div style={{ fontSize: 10, color: '#777', marginBottom: 4 }}>Modo</div>
          <div style={{ display: 'flex', gap: 4, marginBottom: 8 }}>
            <button
              onClick={() => setWorkspaceMode('drawing-plane')}
              style={{
                flex: 1,
                padding: '3px 0',
                fontSize: 11,
                background: workspaceMode === 'drawing-plane' ? '#553' : '#3a3a3a',
                border: '1px solid ' + (workspaceMode === 'drawing-plane' ? '#885' : '#444'),
                borderRadius: 3,
                color: '#ccc',
                cursor: 'pointer',
              }}
            >
              Plano de dibujo
            </button>
            <button
              onClick={() => setWorkspaceMode('full-5dof')}
              style={{
                flex: 1,
                padding: '3px 0',
                fontSize: 11,
                background: workspaceMode === 'full-5dof' ? '#553' : '#3a3a3a',
                border: '1px solid ' + (workspaceMode === 'full-5dof' ? '#885' : '#444'),
                borderRadius: 3,
                color: '#ccc',
                cursor: 'pointer',
              }}
            >
              5 DOF
            </button>
          </div>
          {workspaceError && (
            <div role="alert" style={{ fontSize: 11, color: '#e55', marginTop: 4 }}>
              {workspaceError}
            </div>
          )}
          {workspaceStats && (
            <div style={{ fontSize: 10, color: '#888', marginTop: 6, fontFamily: 'monospace' }}>
              válidos {workspaceStats.n_valid} · rechazados {workspaceStats.n_rejected} · reach{' '}
              {workspaceStats.reach !== null ? `${workspaceStats.reach.toFixed(0)} mm` : '—'}
            </div>
          )}
        </div>

        {/* Modo dibujo — the entry/exit toggle now lives in the bottom pill
            bar (same enterDrawingMode/exitDrawingMode wiring); this block
            keeps the full drawing config UI */}
        {robotMode === 'drawing' && (
          <div style={{ padding: '8px 16px', borderTop: '1px solid #333' }}>
            <label style={{ display: 'flex', alignItems: 'center', gap: 6, fontSize: 11, color: '#aa8', marginBottom: 6, cursor: 'pointer' }}>
                <input
                  type="checkbox"
                  checked={backlashEnabled}
                  onChange={(e) => handleBacklashToggle(e.target.checked)}
                />
                Compensación de backlash (experimental, 2°/1°)
              </label>
              <div style={{ display: 'flex', alignItems: 'center', gap: 4, marginBottom: 6 }}>
                <span style={{ fontSize: 10, color: '#777' }}>Tamaño:</span>
                {[5, 7, 8].map((cm) => (
                  <button
                    key={cm}
                    onClick={() => setDemoSizeCm(cm)}
                    style={{
                      padding: '2px 8px',
                      fontSize: 11,
                      background: demoSizeCm === cm ? '#553' : '#3a3a3a',
                      border: '1px solid ' + (demoSizeCm === cm ? '#885' : '#444'),
                      borderRadius: 3,
                      color: demoSizeCm === cm ? '#ddc' : '#888',
                      cursor: 'pointer',
                    }}
                  >
                    {cm}×{cm}
                  </button>
                ))}
              </div>
              <div style={{ display: 'flex', gap: 6, marginBottom: 6 }}>
                <button
                  onClick={handleStartDemo}
                  disabled={transitioning}
                  style={{
                    flex: 1,
                    padding: 8,
                    background: activeDemo === 'square' ? '#553' : '#3a3a3a',
                    border: '1px solid ' + (activeDemo === 'square' ? '#885' : '#444'),
                    borderRadius: 4,
                    color: activeDemo === 'square' ? '#ddc' : '#888',
                    fontSize: 13,
                    cursor: 'pointer',
                  }}
                >
                  Demo: cuadrado {demoSizeCm}×{demoSizeCm}cm
                </button>
                <button
                  onClick={handleStartDiagnostic}
                  disabled={transitioning}
                  style={{
                    flex: 1,
                    padding: 8,
                    background: activeDemo === 'lines' ? '#553' : '#3a3a3a',
                    border: '1px solid ' + (activeDemo === 'lines' ? '#885' : '#444'),
                    borderRadius: 4,
                    color: activeDemo === 'lines' ? '#ddc' : '#888',
                    fontSize: 12,
                    cursor: 'pointer',
                  }}
                >
                  Diagnóstico: líneas
                </button>
                <button
                  onClick={handleStartArc}
                  disabled={transitioning}
                  style={{
                    flex: 1,
                    padding: 8,
                    background: activeDemo === 'arc' ? '#553' : '#3a3a3a',
                    border: '1px solid ' + (activeDemo === 'arc' ? '#885' : '#444'),
                    borderRadius: 4,
                    color: activeDemo === 'arc' ? '#ddc' : '#888',
                    fontSize: 12,
                    cursor: 'pointer',
                  }}
                >
                  Arco (sin reversiones)
                </button>
              </div>
              <div style={{ display: 'flex', gap: 6, marginBottom: 6 }}>
                <button
                  onClick={() => gcodeInputRef.current?.click()}
                  disabled={transitioning}
                  style={{
                    flex: 1,
                    padding: 8,
                    background: activeDemo === 'gcode' ? '#553' : '#3a3a3a',
                    border: '1px solid ' + (activeDemo === 'gcode' ? '#885' : '#444'),
                    borderRadius: 4,
                    color: activeDemo === 'gcode' ? '#ddc' : '#888',
                    fontSize: 13,
                    cursor: 'pointer',
                  }}
                >
                  {gcodeName ? `G-code: ${gcodeName}` : 'Cargar .gcode'}
                </button>
                <input
                  ref={gcodeInputRef}
                  type="file"
                  accept=".gcode,.gco,.nc,.txt"
                  style={{ display: 'none' }}
                  onChange={(e) => {
                    const file = e.target.files?.[0];
                    if (file) handleGcodeFile(file);
                    e.target.value = '';
                  }}
                />
              </div>
              {gcodeError && (
                <div style={{ fontSize: 11, color: '#e55', marginBottom: 6 }}>
                  {gcodeError}
                </div>
              )}
              {gcodeWarnings.length > 0 && (
                <div style={{ fontSize: 10, color: '#aa8', marginBottom: 6 }}>
                  {gcodeWarnings.slice(0, 5).map((w, i) => (
                    <div key={i}>{w}</div>
                  ))}
                  {gcodeWarnings.length > 5 && (
                    <div>… y {gcodeWarnings.length - 5} más</div>
                  )}
                </div>
              )}
              {validating && (
                <div style={{ fontSize: 11, color: '#88f', marginBottom: 6 }}>
                  Validando que la trayectoria entre en el rango de trabajo…
                </div>
              )}
              {drawingBlock && (
                <div
                  style={{
                    padding: 8,
                    marginBottom: 6,
                    borderRadius: 4,
                    background: '#300',
                    border: '1px solid #833',
                    color: '#f88',
                    fontSize: 11,
                  }}
                >
                  <div style={{ fontWeight: 600, marginBottom: 4 }}>
                    {drawingBlock.reason}
                  </div>
                  {drawingBlock.points.length > 0 && (
                    <div style={{ color: '#c99', marginBottom: 4 }}>
                      Puntos fuera de rango (máx. 8):{' '}
                      {drawingBlock.points.slice(0, 8).map((p, i) => (
                        <span key={i}>
                          ({p[0].toFixed(1)}, {p[1].toFixed(1)}, {p[2].toFixed(0)})
                          {i < Math.min(drawingBlock.points.length, 8) - 1 ? '; ' : ''}
                        </span>
                      ))}
                      {drawingBlock.points.length > 8
                        ? `… (+${drawingBlock.points.length - 8} más)`
                        : ''}
                    </div>
                  )}
                  <div style={{ display: 'flex', gap: 6, marginTop: 4 }}>
                    <button
                      onClick={handleClearDrawingBlock}
                      style={{
                        flex: 1,
                        padding: '2px 6px',
                        fontSize: 11,
                        background: '#333',
                        border: '1px solid #444',
                        borderRadius: 3,
                        color: '#aaa',
                        cursor: 'pointer',
                      }}
                    >
                      Cerrar
                    </button>
                    {drawingBlock.canRefit && (
                      <button
                        onClick={handleRefitGcode}
                        style={{
                          flex: 1,
                          padding: '2px 6px',
                          fontSize: 11,
                          background: '#533',
                          border: '1px solid #885',
                          borderRadius: 3,
                          color: '#ddc',
                          cursor: 'pointer',
                        }}
                      >
                        Reajustar y dibujar
                      </button>
                    )}
                  </div>
                </div>
              )}
              {cipraPanelJobs.length > 0 && (
                <div
                  style={{
                    padding: 8,
                    marginBottom: 6,
                    borderRadius: 4,
                    background: '#232',
                    border: '1px solid #364',
                  }}
                >
                  <div style={{ fontWeight: 600, marginBottom: 4, fontSize: 11, color: '#9d9' }}>
                    Trabajos de CIPRA
                  </div>
                  {cipraPanelJobs.map((job) => (
                    <div
                      key={job.id}
                      style={{
                        padding: 6,
                        marginBottom: 4,
                        borderRadius: 4,
                        background: '#1d1d20',
                        border: '1px solid #333',
                      }}
                    >
                      <div style={{ fontSize: 11, color: '#ccc', fontWeight: 600 }}>{job.name}</div>
                      <div style={{ fontSize: 10, color: '#777', marginBottom: 4 }}>
                        id {job.id.slice(0, 8)} · {job.status}
                      </div>
                      <div style={{ display: 'flex', gap: 6 }}>
                        <button
                          onClick={() => { void handleDrawCipraJob(job); }}
                          disabled={cipraJobs.drawingId !== null || transitioning}
                          style={{
                            flex: 1,
                            padding: '4px 6px',
                            fontSize: 11,
                            background: '#364',
                            border: '1px solid #487',
                            borderRadius: 3,
                            color: '#cfc',
                            cursor: cipraJobs.drawingId !== null || transitioning ? 'not-allowed' : 'pointer',
                          }}
                        >
                          Dibujar
                        </button>
                        <button
                          onClick={() => handleDiscardCipraJob(job.id)}
                          style={{
                            flex: 1,
                            padding: '4px 6px',
                            fontSize: 11,
                            background: '#533',
                            border: '1px solid #833',
                            borderRadius: 3,
                            color: '#fcc',
                            cursor: 'pointer',
                          }}
                        >
                          Descartar
                        </button>
                      </div>
                    </div>
                  ))}
                </div>
              )}
              <div style={{ display: 'flex', gap: 6, marginBottom: 6 }}>
                <button
                  onClick={handlePlaybackControl}
                  disabled={playerId === null}
                  style={{
                    padding: '8px 12px',
                    background: '#3a3a3a',
                    border: 'none',
                    borderRadius: 4,
                    color: '#ccc',
                    fontSize: 13,
                    cursor: 'pointer',
                  }}
                >
                  {playerState === 'running' ? 'Pausa' : playerState === 'paused' ? 'Reanudar' : 'Replay'}
                </button>
                <button
                  onClick={handleStopDemo}
                  disabled={playerId === null}
                  style={{
                    padding: '8px 12px',
                    background: '#633',
                    border: 'none',
                    borderRadius: 4,
                    color: '#ccc',
                    fontSize: 13,
                    cursor: 'pointer',
                  }}
                >
                  Stop
                </button>
              </div>
              <div style={{ display: 'flex', gap: 6, marginBottom: 6 }}>
                <button
                  onClick={handleExportTrace}
                  disabled={traceResult === null || traceResult.samples.length === 0}
                  style={{
                    padding: '8px 12px',
                    background: '#3a3a3a',
                    border: 'none',
                    borderRadius: 4,
                    color: '#ccc',
                    fontSize: 13,
                    cursor: 'pointer',
                  }}
                >
                  Exportar traza CSV
                </button>
                {traceResult !== null && traceResult.samples.length > 0 && (
                  <div style={{ fontSize: 11, color: '#888', alignSelf: 'center' }}>
                    {traceResult.samples.length} muestras
                    {traceResult.truncated ? ' · traza truncada' : ''}
                  </div>
                )}
                <button
                  onClick={handleExportFirmwareTrace}
                  disabled={firmwareTrace.length === 0}
                  style={{
                    padding: '8px 12px',
                    background: '#3a3a3a',
                    border: 'none',
                    borderRadius: 4,
                    color: '#ccc',
                    fontSize: 13,
                    cursor: 'pointer',
                  }}
                >
                  Exportar traza firmware CSV
                </button>
                {firmwareTrace.length > 0 && (
                  <div style={{ fontSize: 11, color: '#888', alignSelf: 'center' }}>
                    firmware: {firmwareTraceStats(firmwareTrace).count} muestras ·{' '}
                    {(firmwareTraceStats(firmwareTrace).durationUs / 1e6).toFixed(2)} s
                  </div>
                )}
              </div>
              <PlanExecPanel trace={traceResult} plan={tracePlan} />
              <div style={{ fontSize: 11, color: '#888', marginBottom: 6 }}>
                Trayectoria: <b style={{ color: '#ccc' }}>{playerState}</b>
                {playerId !== null && playerState !== 'idle' && (
                  <> · {Math.round(motionPlayerProgress(playerId) * 100)}%</>
                )}
              </div>
          </div>
        )}

        {/* Reset — moved to the bottom pill bar (handleReset) */}
      </div>

      {/* 3D Viewport */}
      <div className="app-viewport" style={{ flex: 1, position: 'relative' }}>
        <RobotViewer
          robot={robot}
          rawFrames={rawFrames}
          gripper={gripper}
          workspacePoints={workspacePoints ?? undefined}
          tracePath={tracePath}
          traceProgressRef={traceProgressRef}
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

        {/* CIPRA arrival ALERT — floating top-right overlay of the viewport,
            ANY mode (R13). Informational only: it never decides, it only
            announces and offers "Ir al modo dibujo". The decision panel lives
            inside drawing mode. Container restyled as a glass card (CA-1);
            role, wiring and strings byte-identical. */}
        {cipraJobs.lastNotice && !cipraNoticeDismissed && (
          <div
            role="alert"
            className="glass-card"
            style={{
              position: 'absolute',
              top: 16,
              right: 16,
              zIndex: 20,
              width: 280,
              padding: '8px 16px',
            }}
          >
            <div style={{ fontSize: 12, color: '#dc8', fontWeight: 600 }}>
               Trabajo nuevo desde CIPRA
            </div>
            <div style={{ fontSize: 11, color: '#aa8', margin: '4px 0' }}>
              {cipraJobs.lastNotice.whileDrawing
                ? 'Llegó un trabajo mientras se dibuja — quedó en cola para decidir.'
                : 'Se recibió un trabajo nuevo de CIPRA.'}
            </div>
            <div style={{ display: 'flex', gap: 6, marginTop: 4 }}>
              {robotMode !== 'drawing' && (
                <button
                  onClick={() => {
                    void enterDrawingMode();
                    setCipraNoticeDismissed(true);
                  }}
                  disabled={transitioning}
                  style={{
                    flex: 1,
                    padding: '4px 6px',
                    fontSize: 11,
                    background: '#464',
                    border: 'none',
                    borderRadius: 3,
                    color: '#ccc',
                    cursor: 'pointer',
                  }}
                >
                  Ir al modo dibujo
                </button>
              )}
              <button
                onClick={() => setCipraNoticeDismissed(true)}
                style={{
                  flex: 1,
                  padding: '4px 6px',
                  fontSize: 11,
                  background: '#333',
                  border: '1px solid #444',
                  borderRadius: 3,
                  color: '#aaa',
                  cursor: 'pointer',
                }}
              >
                Cerrar
              </button>
            </div>
          </div>
        )}

        {/* Right column (EE-1): floating END-EFFECTOR glass card + Fidelity
            segmented control, moved here from the sidebar (bottom-right
            overlay, above the pill bar). Wiring identical: same state setter. */}
        <div style={{
          position: 'absolute',
          bottom: 92,
          right: 16,
          zIndex: 15,
          display: 'flex',
          flexDirection: 'column',
          alignItems: 'stretch',
          gap: 10,
          width: 260,
        }}>
          <InfoPanel robot={robot} rawFrames={rawFrames} />
          <div
            className="glass-card"
            style={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between', gap: 8, padding: '8px 12px' }}
          >
            <span style={{ fontSize: 11, color: 'var(--c-gray)' }}>Fidelidad:</span>
            <div className="segmented">
              <button
                onClick={() => setFidelityMode('low')}
                className={fidelityMode === 'low' ? 'segmented__opt--active' : undefined}
                style={segmentOptStyle}
              >
                Low
              </button>
              <button
                onClick={() => setFidelityMode('high')}
                className={fidelityMode === 'high' ? 'segmented__opt--active' : undefined}
                style={segmentOptStyle}
              >
                High
              </button>
            </div>
          </div>
        </div>

        {/* Bottom pill bar (PB-1): dark pills hosting the pre-existing action
            handlers, moved from the sidebar buttons — wiring unchanged.
            Config UIs (Run Analysis selectors, servo calib, demo/gcode
            blocks) stay in the left column. */}
        <div style={{
          position: 'absolute',
          bottom: 16,
          left: '50%',
          transform: 'translateX(-50%)',
          zIndex: 15,
          display: 'flex',
          gap: 8,
          padding: '8px 10px',
          borderRadius: 999,
          background: 'rgba(13, 17, 23, 0.65)',
          border: '1px solid var(--border)',
          boxShadow: 'var(--glass-shadow)',
          backdropFilter: 'blur(var(--glass-blur))',
        }}>
          {connected ? (
            <button className="pill-btn" onClick={handleDisconnect}>
              Desconectar
            </button>
          ) : (
            <button className="pill-btn" onClick={handleConnect}>
              Conectar
            </button>
          )}
          <button
            className="pill-btn"
            onClick={() => {
              if (!ikMode) {
                const fk = forwardKinematics(robot.segments, robot.baseTransform);
                const toolM = robot.toolTransform;
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
          >
            {ikMode ? 'Desactivar IK' : 'IK Mode'}
          </button>
          <button
            className="pill-btn"
            onClick={() => { void enterCalibration(); }}
            disabled={!connected}
          >
            Calibrar Servos
          </button>
          <button className="pill-btn" onClick={handleReset}>
            Reset Home
          </button>
          <button
            className="pill-btn"
            onClick={() => { void runWorkspace(); }}
            disabled={workspaceRunning}
          >
            {workspaceRunning ? `Muestreando… ${workspaceProgress}%` : 'Run Analysis'}
          </button>
          {robotMode === 'normal' ? (
            <button
              className="pill-btn"
              onClick={() => { void enterDrawingMode(); }}
              disabled={transitioning}
            >
              {transitioning ? 'Cerrando pinza…' : 'Modo dibujo'}
            </button>
          ) : (
            <button className="pill-btn" onClick={exitDrawingMode}>
              Salir de modo dibujo
            </button>
          )}
        </div>
      </div>

      {/* Top bar chrome (TB-1): full-width glass strip overlaying the viewport.
          zIndex 5 stays above the canvas (z-auto) and vignette (z1), below
          CalibrationPanel (z10) and the CIPRA alert overlay (z20). */}
      <div
        className="top-bar"
        style={{ position: 'fixed', top: 0, left: 0, right: 0, zIndex: 5 }}
      >
        <span style={{ fontSize: 13, fontWeight: 700, letterSpacing: 1.5, color: '#e6edf3' }}>
          BOMBOLAB — FABRI Creator
        </span>
        <div style={{ display: 'flex', alignItems: 'center', gap: 8 }}>
          <span className={'badge ' + (connected ? 'badge--online' : 'badge--offline')}>
            {connected ? 'Conectado' : 'Desconectado'}
          </span>
          <span role="status" className={'badge ' + (cipraConn === 'connected' ? 'badge--online' : 'badge--offline')}>
            {getConnectionStatusLabel(cipraConn)}
          </span>
          <button
            onClick={() => setCipraNoticeDismissed(v => !v)}
            aria-label="Notificaciones CIPRA"
            title="Notificaciones CIPRA"
            style={{
              display: 'flex',
              alignItems: 'center',
              justifyContent: 'center',
              width: 32,
              height: 32,
              padding: 0,
              background: 'transparent',
              border: '1px solid var(--border)',
              borderRadius: '50%',
              cursor: 'pointer',
              color: cipraJobs.lastNotice && !cipraNoticeDismissed
                ? 'var(--c-cyan)'
                : 'var(--c-gray)',
            }}
          >
            <svg
              width={24}
              height={24}
              viewBox="0 0 24 24"
              fill="none"
              stroke="currentColor"
              strokeWidth={1.8}
              strokeLinecap="round"
              strokeLinejoin="round"
            >
              <path d="M18 8A6 6 0 0 0 6 8c0 7-3 9-3 9h18s-3-2-3-9" />
              <path d="M13.73 21a2 2 0 0 1-3.46 0" />
            </svg>
          </button>
        </div>
      </div>
    </div>
  );
}

/** Robot DH coordinates (x, y, z) → three.js scene coordinates (x, z, y).
 *  Same mapping used by framePose() in renderers/types.ts and IkTarget. */
function robotToThree(p: [number, number, number]): [number, number, number] {
  return [p[0], p[2], p[1]];
}
