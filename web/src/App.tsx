import { useState, useCallback, useRef, useEffect, useMemo } from 'react';
import * as THREE from 'three';
import type { RobotDef, Segment } from './kinematics/types';
import { initWasm, fabriCreator, forwardKinematics, solveIk, solveDrawingIk, solveDrawingIkV2, solveDrawingPlaneIk, motionPlayerNew, motionPlayerPlay, motionPlayerPause, motionPlayerResume, motionPlayerStop, motionPlayerUpdate, motionPlayerState, motionPlayerTarget, motionPlayerProgress, motionPlayerDrop, type PlayerStateJs } from './wasm';
import { squareCommands, diagnosticLinesCommands, arcCommands, drawingPath, type MotionCommandJS } from './motion/commands';
import { qToServoUs, gripperToServoUs, servoDegToUs, encodeWire, requestSerialPort, openPort, sendSerial } from './serial';
import { ServoInterpolator, type InterpolationConfig } from './interpolation';
import type { DebugToggles, FidelityMode, CalibrationConfig } from './renderers/types';
import { ALL_STL_FILES } from './renderers/stlMapping';
import RobotViewer from './components/RobotViewer';
import JointControls from './components/JointControls';
import InfoPanel from './components/InfoPanel';
import CalibrationPanel from './renderers/CalibrationPanel';
import ServoCalibAnalyzer from './components/ServoCalibAnalyzer';

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
  // Robot operating mode: the enum has more variants (Teaching, Calibration,
  // EmergencyStop) but only Normal and Drawing are implemented (slice 1).
  const [robotMode, setRobotMode] = useState<'normal' | 'drawing'>('normal');
  const [transitioning, setTransitioning] = useState(false);
  const [playerId, setPlayerId] = useState<number | null>(null);
  const [playerState, setPlayerState] = useState<PlayerStateJs>('idle');
  const [demoSizeCm, setDemoSizeCm] = useState<number>(8);
  const [tracePath, setTracePath] = useState<[number, number, number][]>([]);
  const [activeDemo, setActiveDemo] = useState<string | null>(null);
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
  const servoInterpolatorRef = useRef<ServoInterpolator | null>(null);

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
      servoInterpolatorRef.current = new ServoInterpolator(
        (wire) => sendSerial(port, wire),
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
    setDrawingActive(result.converged && result.error < 10);
    setRobot(prev => {
      if (!prev) return prev;
      return { ...prev, segments: prev.segments.map((seg, i) => ({ ...seg, q: result.q[i] ?? 0 })) };
    });
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
    if (playerId !== null) {
      try { motionPlayerDrop(playerId); } catch {}
      setPlayerId(null);
    }
    setPlayerState('idle');
    lastTrajectoryTargetRef.current = null;
    setTracePath([]);
    setActiveDemo(null);
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
    const loop = (now: number) => {
      const dt = lastFrameRef.current > 0 ? Math.min((now - lastFrameRef.current) / 1000, 0.1) : 0.016;
      lastFrameRef.current = now;
      try {
        motionPlayerUpdate(playerId, dt);
        const st = motionPlayerState(playerId);
        setPlayerState(st);
        if (st === 'running' || st === 'paused') {
          const target = motionPlayerTarget(playerId);
          const last = lastTrajectoryTargetRef.current;
          if (!last || Math.hypot(target[0] - last[0], target[1] - last[1], target[2] - last[2]) > 0.5) {
            lastTrajectoryTargetRef.current = target;
            setIkTarget(target);
          }
        }
      } catch (e) {
        console.error('[motion]', e);
      }
      raf = requestAnimationFrame(loop);
    };
    raf = requestAnimationFrame(loop);
    return () => cancelAnimationFrame(raf);
  }, [playerId]);

  const startTrajectory = useCallback((cmds: MotionCommandJS[], key: string) => {
    if (transitioning || !robot || robotMode !== 'drawing') return;
    // Replace any running/completed trajectory — the demo buttons must
    // always work; starting a new demo drops the previous player.
    if (playerId !== null) {
      try { motionPlayerDrop(playerId); } catch {}
    }
    const fk = forwardKinematics(robot.segments, robot.baseTransform);
    const ee = fk.frames[fk.frames.length - 1];
    const start: [number, number, number] = [ee[12], ee[13], ee[14]];
    setTracePath(drawingPath(cmds));
    setActiveDemo(key);
    const id = motionPlayerNew(cmds, start);
    setPlayerId(id);
    motionPlayerPlay(id);
    setPlayerState('running');
    setIkTarget(start); // mantener la pose actual hasta el primer waypoint
  }, [playerId, transitioning, robot, robotMode]);

  const handleStartDemo = useCallback(() => {
    const half = (demoSizeCm * 10) / 2; // 5×5 → half 25; 8×8 → half 40
    startTrajectory(squareCommands(200, 0, 80, half), 'square');
  }, [startTrajectory, demoSizeCm]);

  const handleStartDiagnostic = useCallback(() => {
    startTrajectory(diagnosticLinesCommands(), 'lines');
  }, [startTrajectory]);

  const handleStartArc = useCallback(() => {
    startTrajectory(arcCommands(), 'arc');
  }, [startTrajectory]);

  const handleBacklashToggle = useCallback((enabled: boolean) => {
    setBacklashEnabled(enabled);
    servoInterpolatorRef.current?.setBacklash(enabled ? BACKLASH_US : undefined);
  }, []);

  const handlePlaybackControl = useCallback(() => {
    if (playerId === null) return;
    try {
      if (playerState === 'running') {
        motionPlayerPause(playerId);
        setPlayerState('paused');
      } else if (playerState === 'paused') {
        motionPlayerResume(playerId);
        setPlayerState('running');
      } else {
        motionPlayerPlay(playerId);
        setPlayerState('running');
      }
    } catch (e) {
      console.error('[motion]', e);
    }
  }, [playerId, playerState]);

  const handleStopDemo = useCallback(() => {
    if (playerId === null) return;
    try {
      motionPlayerStop(playerId);
      setPlayerState('stopped');
    } catch (e) {
      console.error('[motion]', e);
    }
  }, [playerId]);

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

  const workspacePoints = useMemo(
    () => showWorkspace ? generateWorkspace(2000) : [],
    [showWorkspace],
  );

  // P2 (Stage 3C): FK calculado UNA vez en App y distribuido a los
  // consumidores (RobotViewer + InfoPanel). App no interpreta ni modifica
  // la cinemática — solo comparte el resultado crudo.
  const rawFrames = useMemo(
    () => robot ? forwardKinematics(robot.segments, robot.baseTransform).frames : [],
    [robot],
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
        <InfoPanel robot={robot} rawFrames={rawFrames} />

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
            <div style={{ fontSize: 11, color: '#888', marginBottom: 6 }}>Debug:</div>
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
          </div>
        )}

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

        {/* Calibración de servos (deadband/backlash) — manual */}
        <div style={{ padding: '8px 16px', borderTop: '1px solid #333' }}>
          {!calibRunning ? (
            <button
              onClick={() => { void enterCalibration(); }}
              disabled={!connected}
              style={{
                width: '100%',
                padding: 8,
                background: '#444',
                border: 'none',
                borderRadius: 4,
                color: '#ccc',
                fontSize: 12,
                cursor: 'pointer',
              }}
            >
              Calibrar servos (manual)
            </button>
          ) : (
            <>
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
                    ✓ Se movió
                  </button>
                  <button
                    onClick={() => calibRecord(false)}
                    style={{ ...stepBtn, background: '#633', flex: 1, padding: 8 }}
                  >
                    ✗ No se movió
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
            </>
          )}
          {calibAnalyzerOpen && <ServoCalibAnalyzer log={calibLog} />}
        </div>
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

        {/* Modo dibujo */}
        <div style={{ padding: '8px 16px', borderTop: '1px solid #333' }}>
          {robotMode === 'normal' ? (
            <button
              onClick={() => { void enterDrawingMode(); }}
              disabled={transitioning}
              style={{
                width: '100%',
                padding: 8,
                background: '#464',
                border: 'none',
                borderRadius: 4,
                color: '#ccc',
                fontSize: 13,
                cursor: 'pointer',
              }}
            >
              {transitioning ? 'Cerrando pinza…' : 'Modo dibujo'}
            </button>
          ) : (
            <>
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
              <div style={{ fontSize: 11, color: '#888', marginBottom: 6 }}>
                Trayectoria: <b style={{ color: '#ccc' }}>{playerState}</b>
                {playerId !== null && playerState !== 'idle' && (
                  <> · {Math.round(motionPlayerProgress(playerId) * 100)}%</>
                )}
              </div>
              <button
                onClick={exitDrawingMode}
                style={{
                  width: '100%',
                  padding: 6,
                  background: '#333',
                  border: 'none',
                  borderRadius: 4,
                  color: '#a99',
                  fontSize: 12,
                  cursor: 'pointer',
                }}
              >
                Salir de modo dibujo (restaura pinza)
              </button>
            </>
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
      <div style={{ flex: 1, position: 'relative' }}>
        <RobotViewer
          robot={robot}
          rawFrames={rawFrames}
          gripper={gripper}
          workspacePoints={workspacePoints}
          tracePath={tracePath}
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
      </div>
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
