import { useState, useCallback, useRef, useEffect, useMemo, useReducer } from 'react';
import * as THREE from 'three';
import type { RobotDef, Segment } from './kinematics/types';
import { initWasm, fabriCreator, forwardKinematics, solveIk, solveDrawingIk, solveDrawingIkV2, solveDrawingPlaneIk, motionPlayerNew, motionPlayerPlay, motionPlayerPause, motionPlayerResume, motionPlayerStop, motionPlayerUpdate, motionPlayerState, motionPlayerTarget, motionPlayerProgress, motionPlayerDrop, samplerNew, sampleBatch, samplerStats, samplerDrop, type PlayerStateJs, type WorkspaceMode, type WorkspaceStats } from './wasm';
import { parseWorkspaceBatch, concatWorkspaceChunks, validateSampleCount, type WorkspacePoints } from './workspace/colors';
import { squareCommands, diagnosticLinesCommands, arcCommands, drawingPath, type MotionCommandJS } from './motion/commands';
import { parseGcode } from './motion/gcode';
import { validateDrawingCommands, safeDrawingArea, isReachablePoint, DRAW_PLANE_Z, TRAVEL_PLANE_Z, type ReachResult } from './motion/reachability';
import { runSingularityGate } from './motion/singularityGate';
import { qToServoUs, gripperToServoUs, servoDegToUs, encodeWire, requestSerialPort, openPort, sendSerial, awaitSendSerial, releaseSerial, drainSerial, handshakeV2, uploadManifest, continueManifestUpload, sendManifestHeader } from './serial';
import { buildManifest, sliceLines, V2_CHUNK_MAX } from './motion/manifest';
import { firmwareTraceCsv, firmwareTraceStats, type FirmwareSample } from './motion/traceFirmware';
import { DEFAULT_INTERPOLATION, ServoInterpolator, type InterpolationConfig } from './interpolation';
import { TraceRecorder, type TraceResult } from './motion/trace';
import { planTimeline, type PlanSample } from './motion/planTimeline';
import { downloadTraceCsv, downloadBlob, copyText, exportTraceCsv } from './motion/csv';
import type { DebugToggles, FidelityMode, CalibrationConfig } from './renderers/types';
import { ALL_STL_FILES } from './renderers/stlMapping';
import RobotViewer from './components/RobotViewer';
import JointControls from './components/JointControls';
import GamepadControls from './components/GamepadControls';
import { mapGamepadToCommands, clampAngle, type GamepadCommand } from './gamepad';
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
        <p style={{ fontSize: 16, color: 'var(--c-gray)' }}>Cargando WASM...</p>
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
  // ─── Gamepad (rate) control ──────────────────────────────────────────────
  // Velocity-style sticks: active only while enabled AND the same gates that
  // disable the manual sliders (normal mode, no IK target) AND no trajectory
  // playback. The readout is throttled in the loop (see below) so this panel
  // does not re-render at 60 Hz.
  const [gamepadEnabled, setGamepadEnabled] = useState(false);
  const [gamepadId, setGamepadId] = useState<string | null>(null);
  const [gamepadReadout, setGamepadReadout] = useState<GamepadCommand | null>(null);
  const [gamepadApiAvailable] = useState(
    () => typeof navigator !== 'undefined' && typeof navigator.getGamepads === 'function',
  );
  const [demoSizeCm, setDemoSizeCm] = useState<number>(8);
  const [tracePath, setTracePath] = useState<[number, number, number][]>([]);
  const [traceResult, setTraceResult] = useState<TraceResult | null>(null);
  const [firmwareTrace, setFirmwareTrace] = useState<FirmwareSample[]>([]);
  const [manifestStatus, setManifestStatus] = useState('');
  const [exportMsg, setExportMsg] = useState('');
  const diagRef = useRef<string[]>([]);
  const pushDiag = useCallback((msg: string) => {
    diagRef.current.push(`[${new Date().toISOString().slice(11, 23)}] ${msg}`);
  }, []);
  const [serialLost, setSerialLost] = useState(false);
  const [csvModal, setCsvModal] = useState<{ title: string; content: string } | null>(null);
  const [tracePlan, setTracePlan] = useState<PlanSample[] | null>(null);
  const traceProgressRef = useRef(0);
  const [activeDemo, setActiveDemo] = useState<string | null>(null);
  const [gcodeName, setGcodeName] = useState<string | null>(null);
  const [gcodeWarnings, setGcodeWarnings] = useState<string[]>([]);
  const [gcodeError, setGcodeError] = useState<string | null>(null);
  // Real-scale 1:1 mode: parse the gcode WITHOUT autofit so coordinates are
  // used verbatim (the pattern 150×100 is already authored in robot space).
  // The reachability gate still blocks trajectories that leave the workspace.
  const [gcodeAutofit, setGcodeAutofit] = useState(true);
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
  const manifestAbortControllerRef = useRef<AbortController | null>(null);

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
  // D5: presentational visibility ONLY — the "Análisis" pill button toggles
  // the Analysis card inside the right contextual dock. No behavior change
  // to runWorkspace/sample counts/etc.
  const [analysisOpen, setAnalysisOpen] = useState(false);

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

  // Cablea un puerto ya abierto: listeners, interpolator, estado. Reutilizable
  // por la conexión manual y por la reconexión automática tras pérdida USB.
  const setupPort = useCallback((port: SerialPort) => {
    if (!robot) return;
    portRef.current = port;
    port.addEventListener('disconnect', () => {
      setSerialLost(true);
      setConnected(false);
      setManifestStatus('⚠ dispositivo USB perdido — reintentando reconexión…');
      void (async () => {
        // El Arduino se resetea y re-enumera: buscar el dispositivo (incluido el
        // MISMO objeto de puerto, que Chrome puede reutilizar tras la re-enumeración)
        // y reconectar automáticamente (getPorts no requiere gesto del usuario).
        const info = port.getInfo();
        for (let i = 0; i < 60; i++) {
          await new Promise((r) => setTimeout(r, 500));
          try {
            const ports = await navigator.serial.getPorts();
            const fresh = ports.find((p) => {
              const pi = p.getInfo();
              return pi.usbVendorId === info.usbVendorId && pi.usbProductId === info.usbProductId;
            });
            if (fresh) {
              // si es el mismo objeto muerto, probar igual: puede haber revivido
              await fresh.open({ baudRate: 115200 });
              setupPort(fresh);
              setSerialLost(false);
              setManifestStatus('reconectado automáticamente tras la pérdida USB');
              return;
            }
          } catch {
            /* el dispositivo aún no volvió o el open falló: reintentar */
          }
        }
        setManifestStatus('⚠ no se pudo reconectar solo: conectá el dispositivo de nuevo.');
      })();
    });
    // Start the interpolation scheduler from the current pose and push
    // one frame so the firmware leaves its boot/home state.
    const initial = [...qToServoUs(robot.segments.map((s) => s.q)), gripperToServoUs(gripper)];
    if (traceRecorderRef.current === null) traceRecorderRef.current = new TraceRecorder();
    servoInterpolatorRef.current = new ServoInterpolator(
      (wire) => {
        traceRecorderRef.current?.record(wire);
        sendSerial(port, wire);
      },
      initial,
      // Pacing unificado en 40 ms con el CLI Rust (DEFAULT_INTERPOLATION).
      { ...DEFAULT_INTERPOLATION, backlash: backlashEnabled ? BACKLASH_US : undefined },
    );
    servoInterpolatorRef.current.keepAlive();
    setConnected(true);
  }, [robot, gripper, backlashEnabled]);

  const handleConnect = useCallback(async () => {
    if (!robot) return;
    try {
      setSerialError(null);
      const port = await requestSerialPort();
      await openPort(port);
      setupPort(port);
    } catch (e: any) {
      setSerialError(e.message ?? 'Error al conectar');
    }
  }, [robot, gripper, backlashEnabled]);

  const handleDisconnect = useCallback(async () => {
    try {
      releaseSerial(portRef.current!);
      await portRef.current?.close();
    } catch {}
    // Any in-flight manifest flow is over — abort it so no stale reader
    // keeps consuming the port and the STOP paths below never hit a null port.
    manifestAbortControllerRef.current?.abort();
    manifestModeRef.current = false;
    manifestAbortRef.current = true;
    servoInterpolatorRef.current?.stop();
    servoInterpolatorRef.current = null;
    portRef.current = null;
    setConnected(false);
  }, []);

  useEffect(() => {
    if (connected) setSerialLost(false);
  }, [connected]);

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

  // NOTE: scroll-wheel Z adjustment of the IK target was REMOVED — the global
  // window wheel listener hijacked OrbitControls zoom: every zoom gesture
  // changed ikTarget.z by ±5 and, via the IK-solve effect, moved the robot
  // joints. Zoom must only zoom the camera. (If the Z adjust is wanted again,
  // it belongs on the IK card as an explicit control, not on the wheel.)

  // Deactivate calibration mode when switching to low fidelity
  useEffect(() => {
    if (fidelityMode === 'low') setCalibrationMode(false);
  }, [fidelityMode]);

  useEffect(() => {
    if (!robot) return;
    // Durante un manifest el robot NO debe recibir frames legacy: contaminarían
    // el handshake v2 y el flujo de ACKs (el heartbeat ya respeta este flag).
    if (manifestModeRef.current) return;
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

  // ─── Gamepad rAF loop ───────────────────────────────────────────────────
  // Reads the gamepad every animation frame and accumulates q += v·dt into
  // the SAME `robot` state the sliders drive, so the 3D view follows without
  // hardware; the existing `sendQ` effect (robot/gripper change → interpolator
  // → sendSerial) then pushes the wire frames through the exact same path as
  // manual control. Gates are re-checked per frame via refs and the loop
  // self-pauses (no accumulation, no sends, readout zeroed) whenever any of
  // them closes — it never commands during drawing/IK/playback, and a
  // gamepad disconnect simply leaves the state untouched (no surprise frame).
  const robotModeRef = useRef(robotMode);
  robotModeRef.current = robotMode;
  const playerStateRef = useRef(playerState);
  playerStateRef.current = playerState;
  const ikModeRef = useRef(ikMode);
  ikModeRef.current = ikMode;
  const robotRef = useRef(robot);
  robotRef.current = robot;
  useEffect(() => {
    if (!gamepadEnabled) return;
    let raf = 0;
    let lastT = 0;
    let lastReadoutT = 0;
    let lastSeenId: string | null = null;
    const loop = (now: number) => {
      raf = requestAnimationFrame(loop);
      const gatesOpen =
        robotRef.current !== null &&
        robotModeRef.current === 'normal' &&
        playerStateRef.current === 'idle' &&
        !ikModeRef.current &&
        !manifestModeRef.current;
      // Status is polled even while gated so the panel shows connect/disconnect.
      let gp: Gamepad | null = null;
      try {
        const gps = typeof navigator.getGamepads === 'function' ? navigator.getGamepads() : [];
        gp = gps.find((g): g is Gamepad => g !== null) ?? null;
      } catch { /* gamepad API unavailable or threw */ }
      const id = gp?.id ?? null;
      if (id !== lastSeenId) {
        lastSeenId = id;
        setGamepadId(id);
      }
      if (!gatesOpen) {
        if (now - lastReadoutT > 200) { setGamepadReadout(null); lastReadoutT = now; }
        return;
      }
      if (!gp) {
        if (now - lastReadoutT > 200) { setGamepadReadout(null); lastReadoutT = now; }
        return;
      }
      const dt = lastT > 0 ? Math.min((now - lastT) / 1000, 0.1) : 0;
      lastT = now;
      const cmd = mapGamepadToCommands(gp);
      if (now - lastReadoutT > 100) { setGamepadReadout(cmd); lastReadoutT = now; }
      setRobot(prev => {
        if (!prev) return prev;
        let changed = false;
        const segments = prev.segments.map((seg, i) => {
          const v = cmd.jointVelocities[i] ?? 0;
          if (v === 0) return seg;
          const q = clampAngle(seg.q + v * dt, seg.q_min, seg.q_max);
          if (q === seg.q) return seg;
          changed = true;
          return { ...seg, q };
        });
        return changed ? { ...prev, segments } : prev;
      });
      if (cmd.gripperDeltaPct !== 0) {
        setGripper(prev => Math.min(100, Math.max(0, prev + cmd.gripperDeltaPct * dt)));
      }
    };
    raf = requestAnimationFrame(loop);
    return () => cancelAnimationFrame(raf);
  }, [gamepadEnabled]);

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
      manifestAbortControllerRef.current?.abort();
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
          // The JS player completing is NOT the end of the manifest phase: the
          // firmware may still be executing its trajectory. The phase ends on
          // DONE (continueManifestUpload onDone) or an error/abort path — only
          // surface the firmware trace collected so far.
          setFirmwareTrace([...firmwareTraceRef.current]);
          // El interpolator quedó frenado al iniciar el manifest: resincronizar
          // con la última pose aplicada por el firmware para el próximo control manual.
          const lastFw = firmwareTraceRef.current[firmwareTraceRef.current.length - 1];
          if (lastFw) servoInterpolatorRef.current?.sync(lastFw.joints);
          // Descarga automática: el dato queda accesible aunque el botón falle.
          if (firmwareTraceRef.current.length > 0) {
            try {
              downloadBlob('firmware-trace.csv', new Blob([firmwareTraceCsv(firmwareTraceRef.current)], { type: 'text/csv' }));
              setExportMsg(`firmware-trace.csv descargado automáticamente (${firmwareTraceRef.current.length} muestras). Si no lo ves, usá "Ver CSV".`);
            } catch {
              /* el modal "Ver CSV" queda como respaldo */
            }
          }
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
    if (!result) return;
    // Merge with any earlier (pause-finalized) segment; the seam sample is never duplicated.
    setTraceResult((prev) => {
      if (prev === null) return result;
      const shift = result.t0 - prev.t0;
      const samples = [...prev.samples];
      let lastTs = samples.length > 0 ? samples[samples.length - 1].ts_ms : -Infinity;
      for (const s of result.samples) {
        const ts = s.ts_ms + shift;
        if (ts <= lastTs) continue;
        const last = samples[samples.length - 1];
        if (last !== undefined && last.q_us.every((v, i) => v === s.q_us[i])) {
          samples[samples.length - 1] = { ...last, count: last.count + s.count };
        } else {
          samples.push({ ...s, ts_ms: ts });
        }
        lastTs = ts;
      }
      return { samples, t0: prev.t0, truncated: prev.truncated || result.truncated, framesWritten: prev.framesWritten + result.framesWritten, dedupe: prev.dedupe + result.dedupe };
    });
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
    // El recorder se crea perezosamente: antes solo existía tras conectar el puerto,
    // así que sin hardware la traza web nunca se grababa y el export quedaba mudo.
    if (traceRecorderRef.current === null) traceRecorderRef.current = new TraceRecorder();
    traceRecorderRef.current?.start();
    setTraceResult(null);
    setPlayerState('running');
    const port = portRef.current;
    if (port && connected) {
      // R3-1: the firmware may still be executing a previous manifest — a new
      // flow must STOP it first (otherwise ERR BAD_STATE discards the
      // executor and the handshake fails into a silent local-only fallback).
      if (manifestModeRef.current) {
        sendSerial(port, new TextEncoder().encode('STOP\n'));
        manifestModeRef.current = false;
        manifestAbortRef.current = true;
        setFirmwareTrace([...firmwareTraceRef.current]);
      }
      manifestAbortControllerRef.current?.abort();
      const manifestController = new AbortController();
      manifestAbortControllerRef.current = manifestController;
      manifestAbortRef.current = false;
      const built = buildManifest(samples);
      if (built instanceof Error) {
        console.warn('[manifest] build fallback legacy:', built.message);
      } else {
        try {
          // F4: gate the legacy output (heartbeat/sendQ) from BEFORE the
          // handshake until the firmware phase truly ends (DONE/error/abort).
          // Detener heartbeat/sendQ legacy ANTES del handshake: el tráfico legacy
          // contamina el puerto y el handshake v2 falla (residuos OK/ERR).
          manifestModeRef.current = true;
          setManifestStatus('iniciando: handshake v2…');
          diagRef.current = [`[${new Date().toISOString().slice(11, 23)}] browser: ${navigator.userAgent}`];
          pushDiag(`puerto: readable=${!!port.readable} writable=${!!port.writable}`);
          // Frenar el interpolator legacy: si está en movimiento (p. ej. la pinza
          // al entrar en modo dibujo), sus frames contaminan el handshake v2
          // (el firmware responde OK a cada uno y el HELLO OK se pierde).
          servoInterpolatorRef.current?.stop();
          const hsT0 = Date.now();
          const chunkMax = await handshakeV2(port);
          pushDiag(`handshake: ${Date.now() - hsT0}ms → ${chunkMax instanceof Error ? 'FALLÓ: ' + chunkMax.message : 'OK (chunkMax=' + chunkMax + ')'}`);
          if (!(chunkMax instanceof Error)) {
            // Drenar ERRs stale que hayan quedado en el TX del firmware por
            // intentos previos fallidos (p. ej. el CLI o un draw abortado).
            const stale = await drainSerial(port);
            if (stale > 0) pushDiag(`drenado post-handshake: ${stale} líneas stale`);
          }
          if (chunkMax instanceof Error) {
            manifestModeRef.current = false;
            setManifestStatus('handshake v2 FALLÓ: ' + chunkMax.message);
            console.warn('[manifest] handshake fallback legacy:', chunkMax.message);
          } else {
            setManifestStatus('handshake v2 OK');
            // Protocolo v2: el firmware exige MANIFEST (count/durationUs) antes del
            // primer SAMPLE (gate manifest_ready). Se envía como línea individual;
            // el firmware no responde a esta línea. El upload no se cierra con
            // END_UPLOAD: tras el EXECUTE el resto viaja vía continueManifestUpload.
            await sendManifestHeader(port, built.count, built.durationUs);
            pushDiag(`manifest enviado: ${built.count} samples, ${built.durationUs}us`);
            pushDiag('línea MANIFEST: `MANIFEST ' + built.count + ' ' + built.durationUs + '`');
            pushDiag('primer SAMPLE: `' + built.lines[0] + '`');
            pushDiag('último SAMPLE: `' + built.lines[built.lines.length - 1] + '`');
            setManifestStatus(`manifest enviado (${built.count} samples)`);
            const chunks = sliceLines(built.lines, chunkMax);
            // Registrar cada SAMPLE enviado en el trace (mismo formato wire que el
            // interpolator) para que "Exportar traza CSV" capture el manifest.
            const recordManifestSample = (q: number[]) => {
              traceRecorderRef.current?.record(new TextEncoder().encode(q.join(',') + '\n'));
            };
            const upload = await uploadManifest(port, chunks, chunkMax, undefined, recordManifestSample);
            pushDiag(`upload: sent=${upload.sent}/${upload.total}${upload.error ? ' error=' + upload.error : ''}`);
            setManifestStatus(`upload ${upload.sent}/${upload.total}`);
            if (upload.error) {
              manifestModeRef.current = false;
              setManifestStatus('upload FALLÓ: ' + upload.error);
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
// Defensa en profundidad: NUNCA mandar EXECUTE con el ring incompleto.
            // Defensa en profundidad: NUNCA mandar EXECUTE con el ring incompleto.
            // cargó una fracción sin confirmar, el EXECUTE deja al firmware mudo
            // (sin T-lines) esperando samples que nunca llegan.
            const ringLimit = 2 * chunkMax;
            const expectedPreExecute = Math.min(upload.total, ringLimit);
            if (upload.sent < expectedPreExecute) {
              manifestModeRef.current = false;
              setManifestStatus('upload INCOMPLETO: ' + upload.sent + '/' + upload.total + ' — no se envía EXECUTE');
              setDrawingBlock({
                reason: `El manifest no se cargó completo antes del EXECUTE (${upload.sent}/${upload.total}). Reintentá o verificá el firmware.`,
                points: [],
                canRefit: false,
              });
              try { motionPlayerDrop(id); } catch {}
              setPlayerId(null);
              setPlayerState('idle');
              return false;
            }
            await awaitSendSerial(port, new TextEncoder().encode('EXECUTE\n'));
            pushDiag(`EXECUTE enviado (ring: ${upload.sent})`);
            setManifestStatus(`EXECUTE enviado (ring: ${upload.sent})`);
            firmwareTraceRef.current = [];
            const remaining = built.lines.slice(upload.sent);
            void (async () => {
              let tCount = 0;
              const res = await continueManifestUpload(
                port,
                2 * chunkMax, // ring del firmware (2×chunkMax): tope de vuelo exacto
                remaining,
                undefined,
                (tUs, joints) => {
                  tCount += 1;
                  if (tCount <= 3 || tCount % 100 === 0) {
                    setManifestStatus(`T-lines: ${tCount} (ejecutando…)`);
                  }
                  firmwareTraceRef.current.push({ tUs, joints });
                  if (firmwareTraceRef.current.length > 100000) {
                    firmwareTraceRef.current.shift();
                  }
                },
                () => {
                  manifestModeRef.current = false;
                  setFirmwareTrace([...firmwareTraceRef.current]);
                },
                manifestController.signal,
                built.durationUs,
                recordManifestSample,
              );
              pushDiag(`continueManifestUpload: ${JSON.stringify(res)} T-lines=${tCount}`);
              setManifestStatus(`T-lines: ${tCount}${res.error ? ' · error: ' + res.error : ''}`);
              if (res.error && !manifestAbortRef.current) {
                manifestModeRef.current = false;
                setFirmwareTrace([...firmwareTraceRef.current]);
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
          manifestModeRef.current = false;
          const msg = e instanceof Error ? e.message : String(e);
          pushDiag('error: ' + msg);
          if (/lost|disconnect/i.test(msg)) {
            setSerialLost(true);
            setManifestStatus('⚠ dispositivo USB perdido durante el manifest: revisá la alimentación de los servos y el cable, y reconectá.');
          } else {
            setManifestStatus('error: ' + msg);
          }
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
  const runLoadGcodeText = useCallback(    (text: string, name: string) =>
      loadGcodeText(text, name, {
        safeDrawingArea: () => safeDrawingArea(DRAW_PLANE_Z),
        parseGcode,
        startTrajectory,
        setValidating,
        setGcodeError,
        setGcodeWarnings,
        setGcodeName,
        autofit: gcodeAutofit,
      }),
    [startTrajectory, gcodeAutofit],
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
      manifestAbortControllerRef.current?.abort();
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
    // 'drawing' excluded: once the user presses Dibujar the job leaves the
    // decision panel (same as Descartar) so the card stays uncluttered. A
    // failed draw (FAIL) moves it back to pending, so it reappears here.
    () => cipraJobs.jobs.filter((j) => j.status !== 'completed' && j.status !== 'discarded' && j.status !== 'drawing'),
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
        manifestAbortControllerRef.current?.abort();
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
        // R3-2: pause finalized the recorder (stop()); resume must restart
        // it or post-pause samples are lost from the exported CSV.
        let rec = traceRecorderRef.current;
        if (rec === null || !rec.isRecording()) {
          rec = new TraceRecorder();
          traceRecorderRef.current = rec;
        }
        rec.start();
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
        manifestAbortControllerRef.current?.abort();
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

  // Exportar = mostrar el CSV en pantalla (modal): funciona en CUALQUIER navegador,
  // sin depender del sistema de descargas (que algunos entornos bloquean en silencio).
  const handleExportTrace = useCallback(() => {
    if (traceResult === null || traceResult.samples.length === 0) {
      setExportMsg('Sin datos: ejecutá un dibujo para generar la traza.');
      return;
    }
    setCsvModal({ title: `traza web — ${traceResult.samples.length} muestras`, content: exportTraceCsv(traceResult) });
  }, [traceResult]);

  const handleExportFirmwareTrace = useCallback(() => {
    if (firmwareTrace.length === 0) {
      setExportMsg('Sin T-lines: el manifest debe ejecutarse en el robot. ' + (manifestStatus || ''));
      return;
    }
    setCsvModal({ title: `traza firmware — ${firmwareTrace.length} muestras`, content: firmwareTraceCsv(firmwareTrace) });
  }, [firmwareTrace, manifestStatus]);

  const handleModalDownload = useCallback(() => {
    if (csvModal === null) return;
    try {
      downloadBlob('trace.csv', new Blob([csvModal.content], { type: 'text/csv' }));
      setExportMsg('Descargando trace.csv… (si no aparece, usá Copiar)');
    } catch (e) {
      setExportMsg('Error de descarga: ' + (e instanceof Error ? e.message : String(e)));
    }
  }, [csvModal]);

  const handleModalCopy = useCallback(async () => {
    if (csvModal === null) return;
    const ok = await copyText(csvModal.content);
    setExportMsg(ok ? 'Copiado al portapapeles' : 'No se pudo copiar');
  }, [csvModal]);

  const handleViewDiag = useCallback(() => {
    setCsvModal({
      title: 'diagnóstico del flujo manifest',
      content: diagRef.current.length > 0 ? diagRef.current.join('\n') : '(sin datos todavía: ejecutá un dibujo y si falla, volvé a este botón)',
    });
  }, []);

  const handleViewTrace = useCallback(() => {
    if (traceResult === null || traceResult.samples.length === 0) return;
    setCsvModal({ title: `traza web (${traceResult.samples.length} muestras)`, content: exportTraceCsv(traceResult) });
  }, [traceResult]);

  const handleViewFirmwareTrace = useCallback(() => {
    if (firmwareTrace.length === 0) return;
    setCsvModal({ title: `traza firmware (${firmwareTrace.length} muestras)`, content: firmwareTraceCsv(firmwareTrace) });
  }, [firmwareTrace]);

  const handleCopyTrace = useCallback(async () => {
    if (traceResult === null || traceResult.samples.length === 0) return;
    const ok = await copyText(exportTraceCsv(traceResult));
    setExportMsg(ok ? `Copiado al portapapeles (${traceResult.samples.length} muestras)` : 'No se pudo copiar');
  }, [traceResult]);

  const handleCopyFirmwareTrace = useCallback(async () => {
    if (firmwareTrace.length === 0) return;
    const ok = await copyText(firmwareTraceCsv(firmwareTrace));
    setExportMsg(ok ? `Copiado al portapapeles (${firmwareTrace.length} muestras)` : 'No se pudo copiar');
  }, [firmwareTrace]);

  // ─── Servo calibration (deadband / backlash) — manual mode ─────────────
  // User-paced: each button press sends ONE raw 1° step (bypassing the
  // interpolator, heartbeat paused); the user watches and marks whether the
  // servo moved. The app records the verdicts into a CSV log.
  const SERVO_NAMES = ['J1 yaw', 'J2 shoulder', 'J3 elbow', 'J4 roll', 'J5 pitch', 'Gripper'];
  const stepBtn: React.CSSProperties = {
    padding: '6px 10px',
    background: 'rgba(255, 255, 255, 0.04)',
    border: '1px solid var(--border)',
    borderRadius: 6,
    color: 'var(--c-gray)',
    fontSize: 12,
    cursor: 'pointer',
  };

  const enterCalibration = useCallback(async () => {
    const port = portRef.current;
    if (!port || calibRunning) return;
    if (manifestModeRef.current) {
      sendSerial(port, new TextEncoder().encode('STOP\n'));
      manifestAbortControllerRef.current?.abort();
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
    downloadBlob('servo-calibration.csv', blob);
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
    downloadBlob('calibration.json', blob);
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

// EE-1: base style for the Fidelity segmented options — geometry and
  // typography only; the look lives in theme.css (`.segmented__opt` base +
  // `.segmented__opt--active` violet capsule). No background key here so the
  // active class can own the surface.
  const segmentOptStyle: React.CSSProperties = {
    padding: '4px 14px',
    border: '1px solid transparent',
    borderRadius: 999,
    fontSize: 12,
    fontWeight: 600,
    fontFamily: 'var(--font-sans)',
    cursor: 'pointer',
  };

  // Gamepad loop gates, mirrored for the panel badge (same gates as the
  // rAF loop above: normal mode + no IK target + no trajectory playback).
  const gamepadActive =
    gamepadEnabled && robotMode === 'normal' && playerState === 'idle' && !ikMode && !manifestModeRef.current;

  if (!ready || !robot) return <LoadingScreen error={loadError ?? undefined} />;

  return (
    <div style={{ display: 'flex', width: '100%', height: '100%', background: 'linear-gradient(160deg, var(--bg0), var(--bg1))', color: '#ccc' }}>
      {/* Sidebar — floating glass column (D9): position clears the floating
          top bar (top 72 = bar bottom 64 + 8px gap) and the serial-errors
          card (bottom 16 + height 120 + 8px gap), whose bottom edge aligns
          with the pill bar's bottom edge. CAD contextual-inspector
          layout: JointControls, End-Effector (moved from the right column),
          Fidelity, then the persistent toggles. The whole panel scrolls
          (overflowY auto) — blocks flow in order, JointControls keeps its
          own styling. */}
      <div
        className="glass-card"
        style={{
          position: 'fixed',
          top: 72,
          left: 16,
          bottom: 144,
          width: 280,
          minWidth: 280,
          display: 'flex',
          flexDirection: 'column',
          overflowY: 'auto',
          zIndex: 10,
        }}
      >
        {serialLost && (
          <div style={{
            padding: '10px 16px', background: '#4a2222', borderBottom: '1px solid #733',
            fontSize: 12, color: '#f88',
          }}>
            ⚠ Conexión USB perdida: el Arduino se desconectó (posible brownout por la
            alimentación de los servos o cable USB). Revisá la alimentación y reconectá.
          </div>
        )}
        {/* Joint sliders */}
          <JointControls
            segments={robot.segments}
            gripper={gripper}
            onGripperChange={setGripper}
            onChange={handleJointChange}
            disabled={ikMode}
          />

          <GamepadControls
            enabled={gamepadEnabled}
            onEnabledChange={setGamepadEnabled}
            apiAvailable={gamepadApiAvailable}
            gamepadId={gamepadId}
            commands={gamepadReadout}
            active={gamepadActive}
          />

        {/* End-Effector (EE-1) — moved from the right column into the left
            panel (CAD inspector layout). Component and wiring byte-identical. */}
        <div style={{ padding: '10px 16px', borderTop: '1px solid var(--border)' }}>
          <InfoPanel robot={robot} rawFrames={rawFrames} />
        </div>

        {/* Fidelity segmented control — moved from the right column. Wiring
            byte-identical (same state setter). */}
        <div style={{ padding: '10px 16px', borderTop: '1px solid var(--border)', display: 'flex', alignItems: 'center', justifyContent: 'space-between', gap: 8 }}>
          <span style={{ fontSize: 11, color: 'var(--c-gray)' }}>Fidelidad:</span>
          <div className="segmented">
            <button
              onClick={() => setFidelityMode('low')}
              className={'segmented__opt' + (fidelityMode === 'low' ? ' segmented__opt--active' : '')}
              style={segmentOptStyle}
            >
              Low
            </button>
            <button
              onClick={() => setFidelityMode('high')}
              className={'segmented__opt' + (fidelityMode === 'high' ? ' segmented__opt--active' : '')}
              style={segmentOptStyle}
            >
              High
            </button>
          </div>
        </div>

        {/* Calibration mode — visible only in high fidelity */}
        {fidelityMode === 'high' && (
          <div style={{ padding: '10px 16px', borderTop: '1px solid var(--border)' }}>
            <label className="toggle">
              <input
                type="checkbox"
                checked={calibrationMode}
                onChange={(e) => setCalibrationMode(e.target.checked)}
              />
              <span className="toggle__track"><span className="toggle__knob" /></span>
              Calibration Mode
            </label>
          </div>
        )}

        {/* Debug visualization toggles — visible only in high fidelity */}
        {fidelityMode === 'high' && (
          <div style={{ padding: '10px 16px', borderTop: '1px solid var(--border)' }}>
            <div
              className="debug-acc"
              onClick={() => setDebugOpen(!debugOpen)}
              style={{
                display: 'flex',
                alignItems: 'center',
                gap: 6,
                fontSize: 11,
                color: 'var(--c-gray)',
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
                <label className="toggle" style={{ marginBottom: 4 }}>
                  <input
                    type="checkbox"
                    checked={debugToggles.showJointFrames}
                    onChange={(e) => setDebugToggles(prev => ({ ...prev, showJointFrames: e.target.checked }))}
                  />
                  <span className="toggle__track"><span className="toggle__knob" /></span>
                  Show Joint Frames
                </label>
                <label className="toggle" style={{ marginBottom: 4 }}>
                  <input
                    type="checkbox"
                    checked={debugToggles.showStlOrigins}
                    onChange={(e) => setDebugToggles(prev => ({ ...prev, showStlOrigins: e.target.checked }))}
                  />
                  <span className="toggle__track"><span className="toggle__knob" /></span>
                  Show STL Origins
                </label>
                <label className="toggle" style={{ marginBottom: 4 }}>
                  <input
                    type="checkbox"
                    checked={debugToggles.showCalibrationAxes}
                    onChange={(e) => setDebugToggles(prev => ({ ...prev, showCalibrationAxes: e.target.checked }))}
                  />
                  <span className="toggle__track"><span className="toggle__knob" /></span>
                  Show Calibration Axes
                </label>
                <label className="toggle" style={{ marginBottom: 4 }}>
                  <input
                    type="checkbox"
                    checked={debugToggles.showCandidates ?? false}
                    onChange={(e) => setDebugToggles(prev => ({ ...prev, showCandidates: e.target.checked }))}
                  />
                  <span className="toggle__track"><span className="toggle__knob" /></span>
                  Show Calibrator Candidates
                </label>
              </>
            )}
          </div>
        )}

        {/* Reset — moved to the bottom pill bar (handleReset) */}
      </div>

      {/* Serial errors card — always visible, compact, own glass card below
          the sidebar (same left column, same width); its bottom edge
          coincides with the pill bar's bottom edge (bottom 16). Empty
          state shows a soft muted "Sin errores" placeholder; when an error
          exists the alert icon lights up (red glow + pulse) and the
          message scrolls if it grows. */}
      <div
        className="glass-card anim-in"
        style={{
          position: 'fixed',
          left: 16,
          bottom: 16,
          width: 280,
          height: 120,
          display: 'flex',
          flexDirection: 'column',
          padding: '12px 16px',
          zIndex: 10,
        }}
      >
        <div className="card-title" style={{ display: 'flex', alignItems: 'center', gap: 8 }}>
          <svg
            width={16}
            height={16}
            viewBox="0 0 24 24"
            fill="currentColor"
            aria-hidden="true"
            style={{
              color: serialError ? '#F87171' : 'var(--c-text-faint)',
              filter: serialError ? 'drop-shadow(0 0 6px rgba(248, 113, 113, 0.55))' : 'none',
              animation: serialError ? 'badgePulse 2s ease-in-out infinite' : 'none',
            }}
          >
            <path fillRule="evenodd" d="M12 3.4 22.3 20.8H1.7L12 3.4ZM10.9 9.2h2.2v5.2h-2.2v-5.2Zm0 7.2h2.2v2h-2.2v-2Z" />
          </svg>
          Errores seriales
        </div>
        {serialError ? (
          <div style={{ overflowY: 'auto', flex: 1, fontSize: 11, color: '#F87171', lineHeight: 1.6 }}>
            {serialError}
          </div>
        ) : (
          <div style={{ flex: 1, display: 'flex', alignItems: 'center', justifyContent: 'center', fontSize: 11, color: 'var(--c-text-faint)', opacity: 0.6 }}>
            Sin errores
          </div>
        )}
      </div>

        {/* Right contextual dock (D5): fixed container that stacks ONLY the
            active mode cards (drawing / calibration / IK / analysis /
            playback). Empty when no mode is active — clean viewport. The
            dock positions the cards; each card keeps its own glass chrome.
            Card order below is the fixed stacking order. */}
        <div style={{
          position: 'fixed',
          right: 16,
          top: 72,
          width: 320,
          maxHeight: 'calc(100vh - 160px)',
          overflowY: 'auto',
          zIndex: 15,
          display: 'flex',
          flexDirection: 'column',
          gap: 12,
        }}>
          {/* Drawing card — relocated from the D2 floating card into the dock
              (same gate, content byte-identical; its own fixed positioning
              dropped, the dock positions it) */}
          {robotMode === 'drawing' && ikMode && (
            <div
              className="glass-card anim-in"
              style={{ padding: '12px 16px' }}
            >
              <div className="card-title" style={{ display: 'flex', alignItems: 'center', gap: 8 }}>
                <svg width={16} height={16} viewBox="0 0 24 24" fill="currentColor" aria-hidden="true">
                  <path d="M17 3.4a2.85 2.83 0 1 1 4 4L7.5 20.5 2 22l1.5-5.5L17 3.4Z" />
                </svg>
                Drawing Mode
              </div>
            <div style={{ padding: '4px 16px', display: 'flex', alignItems: 'center', gap: 4 }}>
              <span className="section-label" style={{ marginRight: 4 }}>Dibujo:</span>
              {[0, 1, 2].map(mode => (
                <button
                  key={mode}
                  onClick={() => setDrawingMode(mode)}
                  style={{
                    flex: 1,
                    padding: '3px 0',
                    fontSize: 11,
                    background: drawingMode === mode ? 'rgba(0, 242, 254, 0.16)' : 'rgba(255, 255, 255, 0.04)',
                    border: '1px solid ' + (drawingMode === mode ? 'rgba(0, 242, 254, 0.45)' : 'var(--border)'),
                    borderRadius: 6,
                    color: drawingMode === mode ? 'var(--c-cyan)' : 'var(--c-gray)',
                    cursor: 'pointer',
                  }}
                >
                  {mode === 0 ? 'Off' : `Modo ${mode}`}
                </button>
              ))}

            </div>
            <div style={{ padding: '0 16px 4px', fontSize: 10, color: 'var(--c-text-faint)' }}>
              Rueda mouse: sube/baja Z
            </div>
            <div style={{ padding: '8px 16px' }}>
            <label className="toggle" style={{ marginBottom: 6 }}>
              <input
                type="checkbox"
                checked={backlashEnabled}
                onChange={(e) => handleBacklashToggle(e.target.checked)}
              />
              <span className="toggle__track"><span className="toggle__knob" /></span>
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
                      background: demoSizeCm === cm ? 'rgba(0, 242, 254, 0.16)' : 'rgba(255, 255, 255, 0.04)',
                      border: '1px solid ' + (demoSizeCm === cm ? 'rgba(0, 242, 254, 0.45)' : 'var(--border)'),
                      borderRadius: 6,
                      color: demoSizeCm === cm ? 'var(--c-cyan)' : 'var(--c-gray)',
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
                    background: activeDemo === 'square' ? 'rgba(0, 242, 254, 0.16)' : 'rgba(255, 255, 255, 0.04)',
                    border: '1px solid ' + (activeDemo === 'square' ? 'rgba(0, 242, 254, 0.45)' : 'var(--border)'),
                    borderRadius: 6,
                    color: activeDemo === 'square' ? 'var(--c-cyan)' : 'var(--c-gray)',
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
                    background: activeDemo === 'lines' ? 'rgba(0, 242, 254, 0.16)' : 'rgba(255, 255, 255, 0.04)',
                    border: '1px solid ' + (activeDemo === 'lines' ? 'rgba(0, 242, 254, 0.45)' : 'var(--border)'),
                    borderRadius: 6,
                    color: activeDemo === 'lines' ? 'var(--c-cyan)' : 'var(--c-gray)',
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
                    background: activeDemo === 'arc' ? 'rgba(0, 242, 254, 0.16)' : 'rgba(255, 255, 255, 0.04)',
                    border: '1px solid ' + (activeDemo === 'arc' ? 'rgba(0, 242, 254, 0.45)' : 'var(--border)'),
                    borderRadius: 6,
                    color: activeDemo === 'arc' ? 'var(--c-cyan)' : 'var(--c-gray)',
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
                  className={activeDemo === 'gcode' ? 'ctl-btn ctl-btn--active' : 'ctl-btn'}
                  style={{ flex: 1 }}
                >
                  {gcodeName ? `G-code: ${gcodeName}` : 'Cargar .gcode'}
                </button>
                <label
                  style={{
                    display: 'flex',
                    alignItems: 'center',
                    gap: 4,
                    padding: '0 6px',
                    fontSize: 11,
                    color: '#999',
                    cursor: 'pointer',
                    userSelect: 'none',
                  }}
                >
                  <input
                    type="checkbox"
                    checked={gcodeAutofit}
                    onChange={(e) => setGcodeAutofit(e.target.checked)}
                    title="Activado: escala y centra el gcode al área de trabajo. Desactivado: usa las coordenadas tal cual (escala real 1:1)."
                  />
                  Autofit
                </label>
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
                  className="anim-pop"
                  style={{
                    padding: 12,
                    marginBottom: 6,
                    borderRadius: 'var(--radius-ctl)',
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
                        background: 'rgba(255, 255, 255, 0.04)',
                        border: '1px solid #444',
                        borderRadius: 6,
                        color: 'var(--c-text-dim)',
                        cursor: 'pointer',
                      }}
                    >
                      Cerrar
                    </button>
                    {drawingBlock.canRefit && gcodeAutofit && (
                      <button
                        onClick={handleRefitGcode}
                        style={{
                          flex: 1,
                          padding: '2px 6px',
                          fontSize: 11,
                          background: 'transparent',
                          border: '1px solid var(--border)',
                          borderRadius: 6,
                          color: 'var(--c-cyan)',
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
                  className="anim-pop"
                  style={{
                    padding: 12,
                    marginBottom: 6,
                    borderRadius: 'var(--radius-ctl)',
                    background: '#232',
                    border: '1px solid rgba(0, 242, 254, 0.45)',
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
                        borderRadius: 'var(--radius-ctl)',
                        background: '#1d1d20',
                        border: '1px solid var(--border)',
                      }}
                    >
                      <div style={{ fontSize: 11, color: 'var(--c-text)', fontWeight: 600 }}>{job.name}</div>
                      <div style={{ fontSize: 10, color: '#777', marginBottom: 4 }}>
                        id {job.id.slice(0, 8)} · {job.status}
                      </div>
                  <div style={{ display: 'flex', gap: 6 }}>
                    <button
                      onClick={() => { void handleDrawCipraJob(job); }}
                      disabled={cipraJobs.drawingId !== null || transitioning}
                      className="ctl-btn ctl-btn--active"
                      style={{ flex: 1 }}
                    >
                      Dibujar
                    </button>
                    <button
                      onClick={() => handleDiscardCipraJob(job.id)}
                      className="ctl-btn"
                      style={{
                        flex: 1,
                        background: 'rgba(190, 60, 60, 0.18)',
                        border: '1px solid rgba(210, 80, 80, 0.45)',
                        color: '#e88',
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
                  onClick={handleExportTrace}
                  disabled={traceResult === null || traceResult.samples.length === 0}
                  style={{
                    padding: '8px 12px',
                    background: 'rgba(255, 255, 255, 0.04)',
                    border: '1px solid var(--border)',
                    borderRadius: 6,
                    color: 'var(--c-gray)',
                    fontSize: 13,
                    cursor: 'pointer',
                  }}
                >
                  Exportar traza CSV
                </button>
                {traceResult !== null && traceResult.samples.length > 0 && (
                  <div style={{ fontSize: 11, color: 'var(--c-gray)', alignSelf: 'center' }}>
                    {traceResult.samples.length} muestras
                    {traceResult.truncated ? ' · traza truncada' : ''}
                  </div>
                )}
                <button
                  onClick={handleExportFirmwareTrace}
                  disabled={firmwareTrace.length === 0}
                  style={{
                    padding: '8px 12px',
                    background: 'rgba(255, 255, 255, 0.04)',
                    border: '1px solid var(--border)',
                    borderRadius: 6,
                    color: 'var(--c-gray)',
                    fontSize: 13,
                    cursor: 'pointer',
                  }}
                >
                  Exportar traza firmware CSV
                </button>
                {firmwareTrace.length > 0 && (
                  <div style={{ fontSize: 11, color: 'var(--c-gray)', alignSelf: 'center' }}>
                    firmware: {firmwareTraceStats(firmwareTrace).count} muestras ·{' '}
                    {(firmwareTraceStats(firmwareTrace).durationUs / 1e6).toFixed(2)} s
                  </div>
                )}
              </div>
              <PlanExecPanel trace={traceResult} plan={tracePlan} />
            </div>
          </div>
          )}

          {/* Calibration card — servo calibration UI moved from the sidebar
              into the dock (same gate calibRunning; content byte-identical) */}
          {calibRunning && (
            <div className="glass-card anim-in" style={{ padding: '12px 16px' }}>
              <div className="card-title" style={{ display: 'flex', alignItems: 'center', gap: 8 }}>
                <svg width={16} height={16} viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth={2.2} strokeLinecap="round" aria-hidden="true">
                  <path d="M7.3 3.9a9.4 9.4 0 1 1 9.4 0" />
                  <path d="m12 12 3.4-5" />
                </svg>
                Calibración
              </div>
              <div style={{ fontSize: 11, color: 'var(--c-cyan)', marginBottom: 6 }}>{calibStatus}</div>
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
                      background: calibJoint === i ? 'rgba(0, 242, 254, 0.16)' : 'rgba(255, 255, 255, 0.04)',
                      border: '1px solid ' + (calibJoint === i ? 'rgba(0, 242, 254, 0.45)' : 'var(--border)'),
                      borderRadius: 6,
                      color: calibJoint === i ? 'var(--c-cyan)' : 'var(--c-gray)',
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
                <span style={{ fontSize: 13, fontFamily: 'monospace', color: 'var(--c-text)', minWidth: 40, textAlign: 'center' }}>
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
                    style={{ ...stepBtn, background: 'rgba(190, 60, 60, 0.18)', border: '1px solid rgba(210, 80, 80, 0.45)', color: '#e88', flex: 1, padding: 8 }}
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
                  style={{
                    ...stepBtn,
                    flex: 1,
                    background: calibAnalyzerOpen ? 'rgba(0, 242, 254, 0.16)' : 'rgba(255, 255, 255, 0.04)',
                    border: calibAnalyzerOpen ? '1px solid rgba(0, 242, 254, 0.45)' : '1px solid var(--border)',
                    color: calibAnalyzerOpen ? 'var(--c-cyan)' : 'var(--c-gray)',
                  }}
                >
                  {calibAnalyzerOpen ? 'Ocultar análisis' : 'Analizar'}
                </button>
                <button onClick={() => setCalibLog([])} style={stepBtn}>Limpiar</button>
                <button onClick={exitCalibration} style={{ ...stepBtn, background: 'rgba(190, 60, 60, 0.18)', border: '1px solid rgba(210, 80, 80, 0.45)', color: '#e88' }}>Salir</button>
              </div>
              {calibAnalyzerOpen && <div className="anim-in"><ServoCalibAnalyzer log={calibLog} /></div>}
            </div>
          )}

          {/* IK card — standalone IK mode (never while the drawing card is
              up, so it never duplicates the drawing card's selector): Dibujo
              selector + hint + IK target readout moved from the sidebar */}
          {ikMode && robotMode !== 'drawing' && (
            <div className="glass-card anim-in" style={{ padding: '12px 16px' }}>
              <div className="card-title" style={{ display: 'flex', alignItems: 'center', gap: 8 }}>
                <svg width={16} height={16} viewBox="0 0 24 24" fill="currentColor" aria-hidden="true">
                  <path fillRule="evenodd" d="M12 6.8a5.2 5.2 0 1 0 0 10.4 5.2 5.2 0 0 0 0-10.4ZM12 8.4a3.6 3.6 0 1 1 0 7.2 3.6 3.6 0 0 1 0-7.2Z" />
                  <rect x="11.15" y="1.8" width="1.7" height="4" rx="0.85" />
                  <rect x="11.15" y="18.2" width="1.7" height="4" rx="0.85" />
                  <rect x="1.8" y="11.15" width="4" height="1.7" rx="0.85" />
                  <rect x="18.2" y="11.15" width="4" height="1.7" rx="0.85" />
                </svg>
                IK Mode
              </div>
              <div style={{ padding: '4px 16px', display: 'flex', alignItems: 'center', gap: 4 }}>
                <span className="section-label" style={{ marginRight: 4 }}>Dibujo:</span>
                {[0, 1, 2].map(mode => (
                  <button
                    key={mode}
                    onClick={() => setDrawingMode(mode)}
                    style={{
                      flex: 1,
                      padding: '3px 0',
                      fontSize: 11,
                      background: drawingMode === mode ? 'rgba(0, 242, 254, 0.16)' : 'rgba(255, 255, 255, 0.04)',
                      border: '1px solid ' + (drawingMode === mode ? 'rgba(0, 242, 254, 0.45)' : 'var(--border)'),
                      borderRadius: 6,
                      color: drawingMode === mode ? 'var(--c-cyan)' : 'var(--c-gray)',
                      cursor: 'pointer',
                    }}
                  >
                    {mode === 0 ? 'Off' : `Modo ${mode}`}
                  </button>
                ))}
              </div>
              <div style={{ padding: '0 16px 4px', fontSize: 10, color: 'var(--c-text-faint)' }}>
                Rueda mouse: sube/baja Z
              </div>
              {ikTarget && (
                <div style={{ fontSize: 11, color: 'var(--c-gray)', marginTop: 4 }}>
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

          {/* Analysis card — the whole "Análisis de workspace" block moved
              from the sidebar into the dock, gated by the "Análisis" pill
              button (analysisOpen). Owns the run flow: the primary "Run
              Analysis" pill (with progress) + "Cancelar" while running. */}
          {analysisOpen && (
            <div className="glass-card anim-in" style={{ padding: '12px 16px' }}>
              <div style={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between', marginBottom: 6 }}>
                <span className="section-label" style={{ display: 'flex', alignItems: 'center', gap: 8 }}>
                  <svg width={16} height={16} viewBox="0 0 24 24" fill="currentColor" aria-hidden="true">
                    <rect x="4.5" y="13.5" width="3.6" height="8" rx="1.2" />
                    <rect x="10.2" y="9.5" width="3.6" height="12" rx="1.2" />
                    <rect x="15.9" y="5" width="3.6" height="16.5" rx="1.2" />
                  </svg>
                  Análisis de workspace
                </span>
                {workspaceRunning && (
                  <button
                    onClick={cancelWorkspace}
                    style={{
                      padding: '2px 10px',
                      fontSize: 11,
                      background: 'transparent',
                      border: '1px solid var(--border)',
                      borderRadius: 999,
                      color: 'var(--c-cyan)',
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
                      background: workspaceCount === n ? 'rgba(0, 242, 254, 0.16)' : 'rgba(255, 255, 255, 0.04)',
                      border: '1px solid ' + (workspaceCount === n ? 'rgba(0, 242, 254, 0.45)' : 'var(--border)'),
                      borderRadius: 6,
                      color: workspaceCount === n ? 'var(--c-cyan)' : 'var(--c-gray)',
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
                    background: workspaceMode === 'drawing-plane' ? 'rgba(0, 242, 254, 0.16)' : 'rgba(255, 255, 255, 0.04)',
                    border: '1px solid ' + (workspaceMode === 'drawing-plane' ? 'rgba(0, 242, 254, 0.45)' : 'var(--border)'),
                    borderRadius: 6,
                    color: workspaceMode === 'drawing-plane' ? 'var(--c-cyan)' : 'var(--c-gray)',
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
                    background: workspaceMode === 'full-5dof' ? 'rgba(0, 242, 254, 0.16)' : 'rgba(255, 255, 255, 0.04)',
                    border: '1px solid ' + (workspaceMode === 'full-5dof' ? 'rgba(0, 242, 254, 0.45)' : 'var(--border)'),
                    borderRadius: 6,
                    color: workspaceMode === 'full-5dof' ? 'var(--c-cyan)' : 'var(--c-gray)',
                    cursor: 'pointer',
                  }}
                >
                  5 DOF
                </button>
              </div>
              <button
                className="pill-btn"
                onClick={() => { void runWorkspace(); }}
                disabled={workspaceRunning}
                style={{
                  width: '100%',
                  marginTop: 4,
                  background: 'rgba(0, 242, 254, 0.16)',
                  border: '1px solid rgba(0, 242, 254, 0.45)',
                  color: 'var(--c-cyan)',
                }}
              >
                {workspaceRunning ? `Muestreando… ${workspaceProgress}%` : 'Run Analysis'}
              </button>
              {workspaceError && (
                <div role="alert" style={{ fontSize: 11, color: '#e55', marginTop: 4 }}>
                  {workspaceError}
                </div>
              )}
              {workspaceStats && (
                <div style={{ fontSize: 10, color: 'var(--c-gray)', marginTop: 6, fontFamily: 'monospace' }}>
                  válidos {workspaceStats.n_valid} · rechazados {workspaceStats.n_rejected} · reach{' '}
                  {workspaceStats.reach !== null ? `${workspaceStats.reach.toFixed(0)} mm` : '—'}
                </div>
              )}
            </div>
          )}

          {/* Playback card — play/pause/stop + trajectory status moved from
              the drawing card into the dock; only rendered while a player
              exists (playerId !== null) */}
          {playerId !== null && (
            <div className="glass-card anim-in" style={{ padding: '12px 16px' }}>
              <div className="card-title" style={{ display: 'flex', alignItems: 'center', gap: 8 }}>
                <svg width={16} height={16} viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth={2.2} strokeLinejoin="round" aria-hidden="true">
                  <path d="M7.5 4.8 19 12 7.5 19.2Z" />
                </svg>
                Playback
              </div>
              <div style={{ display: 'flex', gap: 6, marginBottom: 6 }}>
                <button
                  onClick={handlePlaybackControl}
                  disabled={playerId === null}
                  style={{
                    padding: '8px 12px',
                    background: playerState === 'running' ? 'rgba(0, 242, 254, 0.16)' : 'rgba(255, 255, 255, 0.04)',
                    border: playerState === 'running' ? '1px solid rgba(0, 242, 254, 0.45)' : '1px solid var(--border)',
                    borderRadius: 6,
                    color: playerState === 'running' ? 'var(--c-cyan)' : 'var(--c-gray)',
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
                    background: 'rgba(190, 60, 60, 0.18)',
                    border: '1px solid rgba(210, 80, 80, 0.45)',
                    borderRadius: 6,
                    color: '#e88',
                    fontSize: 13,
                    cursor: 'pointer',
                  }}
                >
                  Stop
                </button>
              </div>
              <div style={{ fontSize: 11, color: 'var(--c-gray)', marginBottom: 6 }}>
                Trayectoria: <b style={{ color: 'var(--c-text)' }}>{playerState}</b>
                {manifestStatus && (
                  <div style={{ fontSize: 11, color: '#8a8', marginBottom: 4 }}>manifest: {manifestStatus}</div>
                )}
                <button
                  onClick={handleViewDiag}
                  style={{
                    padding: '6px 10px', background: '#333', border: 'none', borderRadius: 4,
                    color: '#aac', fontSize: 12, cursor: 'pointer', width: '100%', marginBottom: 4,
                  }}
                >
                  📋 Diagnóstico del último intento
                </button>
                {exportMsg && (
                  <div style={{ fontSize: 11, color: '#aa8', marginBottom: 4 }}>{exportMsg}</div>
                )}
                {playerId !== null && playerState !== 'idle' && (
                  <> · {Math.round(motionPlayerProgress(playerId) * 100)}%</>
                )}
              </div>
              {playerId !== null && playerState !== 'idle' && (
                <div
                  style={{
                    height: 6,
                    borderRadius: 999,
                    background: 'rgba(255, 255, 255, 0.06)',
                    border: '1px solid var(--border)',
                    overflow: 'hidden',
                  }}
                >
                  <div
                    style={{
                      height: '100%',
                      width: `${motionPlayerProgress(playerId) * 100}%`,
                      borderRadius: 999,
                      background: 'linear-gradient(90deg, var(--c-cyan), var(--c-cobalt))',
                      transition: 'width 0.15s linear',
                    }}
                  />
                </div>
              )}
            </div>
          )}
        </div>

      {/* Modal CSV — acceso garantizado al dato, sin depender del sistema de descargas */}
      {csvModal !== null && (
        <div style={{
          position: 'fixed', top: 0, left: 0, right: 0, bottom: 0,
          background: 'rgba(0,0,0,0.7)', zIndex: 1000,
          display: 'flex', alignItems: 'center', justifyContent: 'center',
        }}>
          <div style={{
            width: '80%', maxWidth: 900, maxHeight: '85%',
            background: '#24242a', border: '1px solid #444', borderRadius: 8,
            display: 'flex', flexDirection: 'column', padding: 12, gap: 8,
          }}>
            <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
              <span style={{ fontSize: 13, color: '#ccc' }}>{csvModal.title}</span>
              <button
                onClick={() => setCsvModal(null)}
                style={{ padding: '6px 12px', background: '#444', border: 'none', borderRadius: 4, color: '#ccc', cursor: 'pointer', fontSize: 12 }}
              >
                Cerrar
              </button>
            </div>
            <textarea
              readOnly
              value={csvModal.content}
              onFocus={(e) => e.currentTarget.select()}
              style={{
                flex: 1, minHeight: 300, background: '#1c1c20', color: '#9c9',
                border: '1px solid #333', borderRadius: 4, fontSize: 11,
                fontFamily: 'monospace', padding: 8, whiteSpace: 'pre', overflow: 'auto',
              }}
            />
            <div style={{ fontSize: 11, color: '#888' }}>
              Seleccioná todo (Ctrl+A dentro del cuadro) y copiá (Ctrl+C) — o guardá con "Copiar" abajo si preferís.
            </div>
          </div>
        </div>
      )}

      {/* 3D Viewport */}
      <div className="app-viewport" style={{ flex: 1, position: 'relative' }}>
        <RobotViewer
          robot={robot}
          rawFrames={rawFrames}
          gripper={gripper}
          workspacePoints={analysisOpen ? (workspacePoints ?? undefined) : undefined}
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
            className="glass-card anim-in"
            style={{
              position: 'absolute',
              top: 72,
              right: 16,
              zIndex: 20,
              width: 280,
              padding: 12,
            }}
          >
            <div style={{ fontSize: 12, color: 'var(--c-cyan)', fontWeight: 600 }}>
               Trabajo nuevo desde CIPRA
            </div>
            <div style={{ fontSize: 11, color: 'var(--c-text-dim)', margin: '4px 0' }}>
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
                  className="ctl-btn"
                  style={{
                    flex: 1,
                    background: '#464',
                    border: 'none',
                    color: 'var(--c-text)',
                  }}
                >
                  Ir al modo dibujo
                </button>
              )}
              <button
                onClick={() => setCipraNoticeDismissed(true)}
                className="ctl-btn"
                style={{ flex: 1 }}
              >
                Cerrar
              </button>
            </div>
          </div>
        )}

        {/* Bottom pill bar (PB-1): dark pills hosting the pre-existing action
            handlers, moved from the sidebar buttons — wiring unchanged.
            Config UIs (Run Analysis selectors, servo calib, demo/gcode
            blocks) stay in the left column. Surface = `.pill-bar` liquid
            glass capsule (theme.css); positioning stays inline. */}
        <div
          className="pill-bar"
          style={{
            position: 'absolute',
            bottom: 16,
            left: '50%',
            transform: 'translateX(-50%)',
            zIndex: 15,
            display: 'flex',
            gap: 8,
            padding: '8px 10px',
          }}
        >
          {connected ? (
            <button className="pill-btn" onClick={handleDisconnect}>
              <svg width={16} height={16} viewBox="0 0 24 24" fill="currentColor" aria-hidden="true">
                <path fillRule="evenodd" d="M12 2.6a9.4 9.4 0 1 1 0 18.8 9.4 9.4 0 0 1 0-18.8ZM12 4.8a7.2 7.2 0 1 0 0 14.4 7.2 7.2 0 0 0 0-14.4Z" />
                <rect x="11.15" y="1.8" width="1.7" height="6.6" rx="0.85" />
              </svg>
              Desconectar
            </button>
          ) : (
            <button className="pill-btn" onClick={handleConnect}>
              <svg width={16} height={16} viewBox="0 0 24 24" fill="currentColor" aria-hidden="true">
                <path fillRule="evenodd" d="M12 2.6a9.4 9.4 0 1 1 0 18.8 9.4 9.4 0 0 1 0-18.8ZM12 4.8a7.2 7.2 0 1 0 0 14.4 7.2 7.2 0 0 0 0-14.4Z" />
                <rect x="11.15" y="1.8" width="1.7" height="6.6" rx="0.85" />
              </svg>
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
            <svg width={16} height={16} viewBox="0 0 24 24" fill="currentColor" aria-hidden="true">
              <path fillRule="evenodd" d="M12 6.8a5.2 5.2 0 1 0 0 10.4 5.2 5.2 0 0 0 0-10.4ZM12 8.4a3.6 3.6 0 1 1 0 7.2 3.6 3.6 0 0 1 0-7.2Z" />
              <rect x="11.15" y="1.8" width="1.7" height="4" rx="0.85" />
              <rect x="11.15" y="18.2" width="1.7" height="4" rx="0.85" />
              <rect x="1.8" y="11.15" width="4" height="1.7" rx="0.85" />
              <rect x="18.2" y="11.15" width="4" height="1.7" rx="0.85" />
            </svg>
            {ikMode ? 'Desactivar IK' : 'IK Mode'}
          </button>
          <button
            className="pill-btn"
            onClick={() => { void enterCalibration(); }}
            disabled={!connected}
          >
            <svg width={16} height={16} viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth={2.2} strokeLinecap="round" aria-hidden="true">
              <path d="M7.3 3.9a9.4 9.4 0 1 1 9.4 0" />
              <path d="m12 12 3.4-5" />
            </svg>
            Calibrar Servos
          </button>
          <button className="pill-btn" onClick={handleReset}>
            <svg width={16} height={16} viewBox="0 0 24 24" fill="currentColor" aria-hidden="true">
              <path fillRule="evenodd" d="M12 3.4 2.9 10.6h2.1v8.6c0 1.1.9 2 2 2h10c1.1 0 2-.9 2-2v-8.6h2.1L12 3.4ZM10 14.6h4v6.6h-4v-6.6Z" />
            </svg>
            Reset Home
          </button>
          <button
            className={'pill-btn' + (analysisOpen ? ' pill-btn--active' : '')}
            onClick={() => setAnalysisOpen(v => !v)}
          >
            {/* Toggle label: "Análisis" → "Salir de Análisis" while the card
                is open; the workspace points render only while analysisOpen */}
            <svg width={16} height={16} viewBox="0 0 24 24" fill="currentColor" aria-hidden="true">
              <rect x="4.5" y="13.5" width="3.6" height="8" rx="1.2" />
              <rect x="10.2" y="9.5" width="3.6" height="12" rx="1.2" />
              <rect x="15.9" y="5" width="3.6" height="16.5" rx="1.2" />
            </svg>
            {analysisOpen ? 'Salir de Análisis' : 'Análisis'}
          </button>
          {robotMode === 'normal' ? (
            <button
              className="pill-btn"
              onClick={() => { void enterDrawingMode(); }}
              disabled={transitioning}
            >
              <svg width={16} height={16} viewBox="0 0 24 24" fill="currentColor" aria-hidden="true">
                <path d="M17 3.4a2.85 2.83 0 1 1 4 4L7.5 20.5 2 22l1.5-5.5L17 3.4Z" />
              </svg>
              {transitioning ? 'Cerrando pinza…' : 'Modo dibujo'}
            </button>
          ) : (
            <button
              className="pill-btn pill-btn--active"
              onClick={exitDrawingMode}
            >
              <svg width={16} height={16} viewBox="0 0 24 24" fill="currentColor" aria-hidden="true">
                <path d="M17 3.4a2.85 2.83 0 1 1 4 4L7.5 20.5 2 22l1.5-5.5L17 3.4Z" />
              </svg>
              Salir de modo dibujo
            </button>
          )}
        </div>
      </div>

      {/* Top bar chrome (TB-1): floating glass capsule overlaying the
          viewport. zIndex 5 stays above the canvas (z-auto) and vignette
          (z1), below CalibrationPanel (z10) and the CIPRA alert (z20).
          Inset 12px with a 10px top gap reads as a floating bar. */}
      <div
        className="top-bar"
        style={{ position: 'fixed', top: 10, left: 12, right: 12, zIndex: 5 }}
      >
        <span style={{ display: 'flex', alignItems: 'center', gap: 12, fontSize: 14, fontWeight: 700, letterSpacing: 1.5, color: '#E2E8F0' }}>
          <img
            src="/logo.png"
            alt="Bombolab"
            width={32}
            height={32}
            style={{ borderRadius: 8, objectFit: 'contain' }}
          />
          BOMBOLAB — FABRI Creator · 5-DOF
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
              position: 'relative',
              display: 'flex',
              alignItems: 'center',
              justifyContent: 'center',
              width: 32,
              height: 32,
              padding: 0,
              background:
                'linear-gradient(180deg, rgba(255,255,255,0.09), rgba(255,255,255,0.02))',
              border: '1px solid rgba(255, 255, 255, 0.14)',
              borderTopColor: 'rgba(255, 255, 255, 0.22)',
              borderRadius: '50%',
              boxShadow: 'inset 0 1px 0 rgba(255, 255, 255, 0.1)',
              cursor: 'pointer',
              transition: 'border-color 0.2s ease, box-shadow 0.2s ease',
              color: cipraJobs.lastNotice && !cipraNoticeDismissed
                ? 'var(--c-cyan)'
                : 'var(--c-gray)',
            }}
          >
            <svg width={22} height={22} viewBox="0 0 24 24" fill="currentColor" aria-hidden="true">
              <path d="M12 2.6a3.4 3.4 0 0 0-3.4 3.4v.8c-2.7 1.6-4.4 4.4-4.4 7.4 0 3.5-1.6 5.6-1.6 5.6h18.8s-1.6-2.1-1.6-5.6c0-3-1.7-5.8-4.4-7.4V6a3.4 3.4 0 0 0-3.4-3.4Z" />
              <rect x="10.9" y="20.2" width="2.2" height="2.4" rx="1.1" />
            </svg>
            {cipraJobs.lastNotice && !cipraNoticeDismissed && (
              <span
                aria-hidden="true"
                style={{
                  position: 'absolute',
                  top: 5,
                  right: 5,
                  width: 8,
                  height: 8,
                  borderRadius: '50%',
                  background: '#F87171',
                  boxShadow: '0 0 6px rgba(248, 113, 113, 0.8)',
                  animation: 'badgePulse 2s ease-in-out infinite',
                }}
              />
            )}
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
