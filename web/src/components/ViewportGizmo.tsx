import { useCallback, useEffect, useMemo, useRef, useState, type RefObject } from 'react';
import { Canvas, useFrame, useThree, type ThreeEvent } from '@react-three/fiber';
import { Edges, Html, useCursor } from '@react-three/drei';
import * as THREE from 'three';

// ─── Shared bridge: main canvas → gizmo canvas ────────────────────────────
// Module-level mutable singleton so the two canvases never re-render each
// other. GizmoSync (mounted in the MAIN canvas) publishes the camera
// quaternion every frame; the gizmo scene mirrors it so the cube/axes always
// show the current view orientation (Blender-style navigation gizmo).

// Drag manipulation request: the gizmo canvas accumulates pointer deltas from
// window-level pointermove/pointerup listeners and publishes them here;
// GizmoSync consumes them per-frame so motion stays smooth. All drag modes
// manipulate the VIEW (camera / orbit target) — never the robot model.
export type GizmoDragMode = 'pan' | 'panAxis' | 'orbit';

export type GizmoDrag = {
  mode: GizmoDragMode;
  // World-space unit axis for axis-constrained pan (null for free pan).
  axis: THREE.Vector3 | null;
  dx: number; // accumulated horizontal delta (px, DOM sign: right = +)
  dy: number; // accumulated vertical delta (px, DOM sign: down = +)
};

export const gizmoLink = {
  quaternion: new THREE.Quaternion(),
  // Distance from the main camera to the orbit target, published each frame so
  // click-to-fly can preserve the user's current zoom (clamped to sane bounds).
  cameraDistance: 900,
  // Click-to-fly request: set by the gizmo canvas, consumed by GizmoSync.
  flight: null as { dir: THREE.Vector3; dist: number } | null,
  // Active drag request: set by the gizmo canvas on pointerdown (empty deltas),
  // accumulated on pointermove, cleared on pointerup. GizmoSync applies the
  // deltas per-frame and zeroes them after consuming.
  drag: null as GizmoDrag | null,
};

// ─── Click-to-fly constants ────────────────────────────────────────────────
// Fly the main camera to a canonical axis view. ROBOT_TARGET matches the
// OrbitControls target in RobotViewer so the orbit center never jumps. The
// distance clamp keeps the robot fully framed (min 400mm) without extreme
// zooms (max 900mm); outside that band we preserve the current zoom.

const FLY_DURATION = 0.7; // seconds — snappy but not jarring
const ROBOT_TARGET = new THREE.Vector3(0, 200, 0);
const UP = new THREE.Vector3(0, 1, 0);
const scratchMatrix = new THREE.Matrix4();
// Scratch vectors/quaternion for per-frame drag math (no allocations).
const rightVec = new THREE.Vector3();
const upVec = new THREE.Vector3();
const panVec = new THREE.Vector3();
const axisCam = new THREE.Vector3();
const camInvQ = new THREE.Quaternion();
const pivot = new THREE.Vector3(); // orbit rotation center (the orbit target)

// ─── Drag manipulation constants ────────────────────────────────────────────
// Press+release with less than CLICK_SLOP_PX of movement is a CLICK (keeps the
// existing click-to-fly). Anything at/over the threshold is a DRAG (pan /
// axis-constrained pan / ring orbit).
const CLICK_SLOP_PX = 4;
// Feel factor: apply 80% of the geometrically exact world-per-pixel scale so
// pans track the cursor slightly relaxed (AutoCAD-style, never 1:1 twitchy).
const PAN_FEEL = 0.8;
// Ring-drag orbit sensitivity: 0.006 rad/px ≈ 360° per ~500px of horizontal
// drag — a comfortable wrist rotation for a corner HUD.
const ORBIT_RAD_PER_PX = 0.006;

type Flight = {
  elapsed: number;
  startPos: THREE.Vector3;
  startQuat: THREE.Quaternion;
  startTarget: THREE.Vector3;
  endPos: THREE.Vector3;
  endQuat: THREE.Quaternion;
};

// Clicked a gizmo face/axis → request a flight. GizmoSync consumes the request
// on its next frame; the distance is decided here (clamped current distance).
function requestFlight(dir: THREE.Vector3) {
  const dist = THREE.MathUtils.clamp(gizmoLink.cameraDistance, 400, 900);
  gizmoLink.flight = { dir: dir.clone(), dist };
}

// ─── GizmoSync: publish orientation + run click-to-fly flights ─────────────
// Mounted INSIDE the main canvas. useFrame runs after the scene renders, so
// the gizmo always mirrors the camera's final (post-orbit, post-flight)
// orientation. The two canvases never re-render each other — quaternion,
// distance and flight requests cross the boundary by value through the
// module-level singleton.

export function GizmoSync() {
  const flightRef = useRef<Flight | null>(null);
  // Owns the controls.enabled=false window during gizmo drags so we only
  // restore the flag when a drag we started actually ends (a concurrent IK
  // drag on the main canvas disables controls through its own React prop).
  const dragActiveRef = useRef(false);

  useFrame((state, delta) => {
    const camera = state.camera;
    gizmoLink.quaternion.copy(camera.quaternion);
    gizmoLink.cameraDistance = camera.position.distanceTo(ROBOT_TARGET);

    // OrbitControls (makeDefault in RobotViewer) exposes .enabled + .target,
    // but R3F types state.controls as EventDispatcher | null → downcast to
    // the narrow surface we actually touch.
    const controls = state.controls as unknown as
      | { enabled: boolean; target: THREE.Vector3; update: () => void }
      | null;
    if (!controls) return;

    const flight = flightRef.current;
    if (flight) {
      // Drop any request that arrived mid-flight (ignore double-clicks) and
      // disable orbit input so the user can't fight the camera animation.
      gizmoLink.flight = null;
      controls.enabled = false;
      flight.elapsed += delta;
      const t = Math.min(flight.elapsed / FLY_DURATION, 1);
      const s = t * t * (3 - 2 * t); // smoothstep: ease in + ease out
      camera.position.lerpVectors(flight.startPos, flight.endPos, s);
      camera.quaternion.slerpQuaternions(flight.startQuat, flight.endQuat, s);
      controls.target.lerpVectors(flight.startTarget, ROBOT_TARGET, s);
      if (t >= 1) {
        flightRef.current = null;
        controls.enabled = true;
      }
      return; // the flight owns the camera this frame — no drag consumption
    }
    if (gizmoLink.flight) {
      // Consume the request and build the end pose. The gizmo cube's local
      // axes ARE the world axes, so the clicked face/axis direction is used
      // directly as the world direction. three's Matrix4.lookAt is pole-safe
      // (nudges z by 0.0001), so zenith views can't flip the camera.
      const { dir, dist } = gizmoLink.flight;
      gizmoLink.flight = null;
      const endPos = dir.clone().multiplyScalar(dist).add(ROBOT_TARGET);
      scratchMatrix.lookAt(endPos, ROBOT_TARGET, UP);
      flightRef.current = {
        elapsed: 0,
        startPos: camera.position.clone(),
        startQuat: camera.quaternion.clone(),
        startTarget: controls.target.clone(),
        endPos,
        endQuat: new THREE.Quaternion().setFromRotationMatrix(scratchMatrix),
      };
    }

    // ─── View-space drag manipulation (pan / axis-constrained pan) ─────────
    // Deltas published by the gizmo canvas are consumed per frame, so the
    // camera + orbit target move continuously while the pointer moves. Only
    // the VIEW moves — the robot model, IK, gcode and drawing-plane semantics
    // are untouched (drag never reaches the scene graph).
    const drag = gizmoLink.drag;
    if (drag) {
      const { mode, axis, dx, dy } = drag;
      drag.dx = 0;
      drag.dy = 0; // consumed
      dragActiveRef.current = true;
      controls.enabled = false;

      // World-per-pixel at the target depth: distance * 2*tan(fov/2) mapped
      // onto the canvas height. state.size is the MAIN canvas size in px.
      const dist = camera.position.distanceTo(controls.target);
      const fov = (camera as THREE.PerspectiveCamera).fov ?? 35;
      const worldPerPx =
        (dist * 2 * Math.tan(THREE.MathUtils.degToRad(fov) / 2)) /
        state.size.height;
      const scale = worldPerPx * PAN_FEEL;

      if (mode === 'pan') {
        // Face drag: translate camera + target in the view plane. The robot
        // follows the cursor: a rightward drag (dx>0) moves the camera left.
        rightVec.set(1, 0, 0).applyQuaternion(camera.quaternion);
        upVec.set(0, 1, 0).applyQuaternion(camera.quaternion);
        panVec.set(0, 0, 0).addScaledVector(rightVec, -dx * scale);
        panVec.addScaledVector(upVec, dy * scale);
        camera.position.add(panVec);
        controls.target.add(panVec);
      } else if (mode === 'panAxis' && axis) {
        // Axis drag: project the screen delta onto the world axis (in camera
        // space) and translate camera + target ONLY along that axis.
        axisCam.copy(axis).applyQuaternion(camInvQ.copy(camera.quaternion).invert());
        const dot = axisCam.x * dx + axisCam.y * -dy; // screen delta in camera space
        const delta = dot * scale;
        camera.position.addScaledVector(axis, -delta);
        controls.target.addScaledVector(axis, -delta);
      } else if (mode === 'orbit' && axis) {
        // Ring drag: rotate the CAMERA around the ring's axis through the
        // orbit target (the scene appears to spin around the colored axis);
        // the target itself stays put. controls.update() re-derives the
        // spherical state from the new position (r170) so nothing snaps back.
        const angle = dx * ORBIT_RAD_PER_PX;
        pivot.copy(controls.target);
        camera.position.sub(pivot).applyAxisAngle(axis, angle).add(pivot);
        camera.lookAt(pivot);
        controls.update();
      }
    } else if (dragActiveRef.current) {
      dragActiveRef.current = false;
      controls.enabled = true;
    }
  });

  return null;
}

// ─── Gizmo visuals ─────────────────────────────────────────────────────────
// Fixed orthographic projection (no perspective distortion) with a dedicated
// camera at [0,0,7] looking at the origin. Only the group rotates — the
// projection itself never changes.

const AXIS_DEFS = [
  { key: 'x', dir: [1, 0, 0], color: '#ff4444', hover: '#ff8080', label: 'X' },
  { key: 'y', dir: [0, 1, 0], color: '#4cd964', hover: '#7dfa94', label: 'Y' },
  { key: 'z', dir: [0, 0, 1], color: '#4488ff', hover: '#77aaff', label: 'Z' },
] as const;

const FACE_REST = '#eceff3'; // premium opaque white (replaces dark glass)
const FACE_HOVER = '#00f2fe'; // design cyan — matches the app accent
// Subtle dark edge strokes so the white cube still reads its silhouette.
const CUBE_EDGES = 'rgba(23, 32, 46, 0.4)';

// ─── Surrounding geometry (premium CAD look) ────────────────────────────────
// Negative semi-axes start at the cube center and reach past the rings;
// the portion inside the opaque cube is hidden by depth. Dashed lines need
// computeLineDistances() before they render (done when building the Line).

const NEG_AXIS_LENGTH = 1.4;
const RING_RADIUS = 1.55; // rotation rings in the plane perpendicular to each axis
const RING_TUBE = 0.022;
const RING_ARC = 2.4; // radians ≈ 137° — arc rings, not full circles
const RING_OPACITY = 0.7;
const GUIDE_RADIUS = 1.95; // dashed guide circle around the whole assembly

// ─── Drag session (gizmo canvas) ─────────────────────────────────────────────
// Started by R3F pointerdown on a face/axis/ring, driven by WINDOW-level
// pointermove/pointerup listeners (the pointer regularly leaves the small
// mini canvas mid-drag). Deltas are accumulated in DOM pixel space (dx right,
// dy down) and published to gizmoLink.drag for GizmoSync to consume per-frame.
// On release, movement below CLICK_SLOP_PX is treated as a CLICK → the stored
// flyDir triggers the existing click-to-fly; anything above is a DRAG.

type DragSession = {
  mode: GizmoDragMode;
  axis: THREE.Vector3 | null;
  flyDir: THREE.Vector3 | null; // click-to-fly direction (faces/axes only)
  pointerId: number;
  startX: number;
  startY: number;
  lastX: number;
  lastY: number;
  dx: number;
  dy: number;
};

function GizmoScene() {
  const group = useRef<THREE.Group>(null!);
  const cubeRef = useRef<THREE.Mesh>(null!);
  const [faceHovered, setFaceHovered] = useState<number | null>(null);
  const canvas = useThree((s) => s.gl.domElement);
  useCursor(faceHovered !== null, 'pointer', 'auto', canvas);

  const sessionRef = useRef<DragSession | null>(null);

  // Fixed materials for the six cube faces (BoxGeometry group order:
  // +X, -X, +Y, -Y, +Z, -Z). MeshBasicMaterial = unlit flat CAD look.
  // Opaque white — the premium Blender-style view cube; hover re-tints the
  // hovered face cyan (behavior preserved from the dark-glass iteration).
  const faceMaterials = useMemo(
    () =>
      Array.from(
        { length: 6 },
        () => new THREE.MeshBasicMaterial({ color: FACE_REST }),
      ),
    [],
  );

  useFrame(() => {
    // Mirror the main camera's view orientation: the gizmo camera is fixed
    // (identity orientation), so the cube must rotate by the INVERSE of the
    // main camera's world rotation to reproduce exactly what the main camera
    // sees (same as drei's GizmoHelper: invert(mainCamera.matrix)).
    group.current.quaternion.copy(gizmoLink.quaternion).invert();
  });

  // Window-level pointer listeners — mounted once. They only act while a
  // session is active, so plain hovers on the main canvas are unaffected.
  useEffect(() => {
    const handleMove = (e: PointerEvent) => {
      const s = sessionRef.current;
      if (!s || e.pointerId !== s.pointerId) return;
      s.dx += e.clientX - s.lastX;
      s.dy += e.clientY - s.lastY;
      s.lastX = e.clientX;
      s.lastY = e.clientY;
      if (gizmoLink.drag) {
        gizmoLink.drag.dx = s.dx;
        gizmoLink.drag.dy = s.dy;
      }
    };
    const handleEnd = (e: PointerEvent, cancelled: boolean) => {
      const s = sessionRef.current;
      if (!s || e.pointerId !== s.pointerId) return;
      sessionRef.current = null;
      gizmoLink.drag = null; // → GizmoSync restores controls.enabled
      const moved =
        Math.hypot(e.clientX - s.startX, e.clientY - s.startY) >= CLICK_SLOP_PX;
      if (!cancelled && !moved && s.flyDir) requestFlight(s.flyDir);
    };
    const handleUp = (e: PointerEvent) => handleEnd(e, false);
    const handleCancel = (e: PointerEvent) => handleEnd(e, true);
    window.addEventListener('pointermove', handleMove);
    window.addEventListener('pointerup', handleUp);
    window.addEventListener('pointercancel', handleCancel);
    return () => {
      window.removeEventListener('pointermove', handleMove);
      window.removeEventListener('pointerup', handleUp);
      window.removeEventListener('pointercancel', handleCancel);
    };
  }, []);

  // Started by R3F onPointerDown on a cube face / axis shaft / ring. The
  // empty drag request published here makes GizmoSync lock the orbit controls
  // immediately (even a plain click disables them until release — no jitter).
  const startDrag = useCallback(
    (
      e: ThreeEvent<PointerEvent>,
      mode: GizmoDragMode,
      axis: THREE.Vector3 | null,
      flyDir: THREE.Vector3 | null,
    ) => {
      if (e.button !== 0) return; // left button / primary touch only
      e.stopPropagation();
      sessionRef.current = {
        mode,
        axis,
        flyDir,
        pointerId: e.pointerId,
        startX: e.clientX,
        startY: e.clientY,
        lastX: e.clientX,
        lastY: e.clientY,
        dx: 0,
        dy: 0,
      };
      gizmoLink.drag = {
        mode,
        axis: axis ? axis.clone() : null,
        dx: 0,
        dy: 0,
      };
    },
    [],
  );

  const handleFaceOver = useCallback(
    (e: ThreeEvent<PointerEvent>) => {
      e.stopPropagation();
      if (e.faceIndex == null) return;
      const face = Math.floor(e.faceIndex / 2); // BoxGeometry: 2 tris per face
      faceMaterials.forEach((m) => m.color.set(FACE_REST));
      faceMaterials[face].color.set(FACE_HOVER);
      setFaceHovered(face);
    },
    [faceMaterials],
  );

  const handleFaceOut = useCallback(() => {
    faceMaterials.forEach((m) => m.color.set(FACE_REST));
    setFaceHovered(null);
  }, [faceMaterials]);

  return (
    <group ref={group}>
      {/* Orientation cube: opaque white faces + subtle dark edges. The local
          face normal is axis-aligned and the group rotation never touches it,
          so the clicked normal IS the world direction to fly toward. */}
      <mesh
        ref={cubeRef}
        material={faceMaterials}
        onPointerOver={handleFaceOver}
        onPointerOut={handleFaceOut}
        onPointerDown={(e) =>
          startDrag(
            e,
            'pan',
            null,
            e.face?.normal ? e.face.normal.clone() : null,
          )
        }
      >
        <boxGeometry args={[1, 1, 1]} />
        <Edges color={CUBE_EDGES} lineWidth={1} />
      </mesh>

      {/* Negative semi-axes: dashed lines from the cube center out past the
          rings, tinted like their positive counterparts (premium CAD look). */}
      {AXIS_DEFS.map((axis) => (
        <NegativeAxisLine key={`neg-${axis.key}`} axis={axis} />
      ))}

      {/* Rotation rings: one arc ring per axis, each in the plane perpendicular
          to its axis (default torus lies in XY = the Z-ring; X- and Y-rings are
          rotated into the YZ / XZ planes). Dragging a ring orbits the view. */}
      {AXIS_DEFS.map((axis) => (
        <RotationRing key={`ring-${axis.key}`} axis={axis} onDragStart={startDrag} />
      ))}

      {/* Guide circle: light dashed ring surrounding the whole assembly. */}
      <GuideCircle />

      {/* Cartesian axes with arrowheads + X/Y/Z labels */}
      {AXIS_DEFS.map((axis) => (
        <Axis key={axis.key} axis={axis} cubeRef={cubeRef} onDragStart={startDrag} />
      ))}
    </group>
  );
}

// ─── Negative semi-axis (dashed) ────────────────────────────────────────────
// Modeled along local -Y like the positive axes, rotated onto the target axis
// with the same mapping (X: +Y→+X is -90° about Z; Z: +Y→+Z is +90° about X).

function NegativeAxisLine({ axis }: { axis: (typeof AXIS_DEFS)[number] }) {
  const line = useMemo(() => {
    const geometry = new THREE.BufferGeometry().setFromPoints([
      new THREE.Vector3(0, 0, 0),
      new THREE.Vector3(0, -NEG_AXIS_LENGTH, 0),
    ]);
    const material = new THREE.LineDashedMaterial({
      color: axis.color,
      dashSize: 0.06,
      gapSize: 0.05,
      transparent: true,
      opacity: 0.75,
    });
    const l = new THREE.Line(geometry, material);
    l.computeLineDistances(); // REQUIRED for dashes to render
    return l;
  }, [axis]);

  const rotation: [number, number, number] =
    axis.key === 'x'
      ? [0, 0, -Math.PI / 2]
      : axis.key === 'z'
        ? [Math.PI / 2, 0, 0]
        : [0, 0, 0];

  return (
    <group rotation={rotation}>
      <primitive object={line} />
    </group>
  );
}

// ─── Rotation ring (arc torus in the plane perpendicular to its axis) ────────

function RotationRing({ axis, onDragStart }: {
  axis: (typeof AXIS_DEFS)[number];
  onDragStart: DragStarter;
}) {
  const [hovered, setHovered] = useState(false);
  const canvas = useThree((s) => s.gl.domElement);
  useCursor(hovered, 'grab', 'auto', canvas);

  // Default torus lies in the XY plane (ring around +Z); rotate the X-ring
  // into the YZ plane and the Y-ring into the XZ plane.
  const rotation: [number, number, number] =
    axis.key === 'x'
      ? [0, Math.PI / 2, 0]
      : axis.key === 'y'
        ? [Math.PI / 2, 0, 0]
        : [0, 0, 0];

  // The ring rotates the view around its WORLD axis (the cube's local axes are
  // the world axes). Drag-only: releasing without movement does NOT fly.
  const axisVec = new THREE.Vector3(...axis.dir);

  return (
    <mesh
      rotation={rotation}
      onPointerOver={(e) => {
        e.stopPropagation();
        setHovered(true);
      }}
      onPointerOut={(e) => {
        e.stopPropagation();
        setHovered(false);
      }}
      onPointerDown={(e) => onDragStart(e, 'orbit', axisVec, null)}
    >
      <torusGeometry args={[RING_RADIUS, RING_TUBE, 12, 64, RING_ARC]} />
      <meshBasicMaterial color={axis.color} transparent opacity={RING_OPACITY} />
    </mesh>
  );
}

// ─── Guide circle: light dashed ring around the whole assembly ───────────────

function GuideCircle() {
  const line = useMemo(() => {
    const points: THREE.Vector3[] = [];
    const segments = 128;
    for (let i = 0; i <= segments; i++) {
      const t = (i / segments) * Math.PI * 2;
      points.push(
        new THREE.Vector3(
          Math.cos(t) * GUIDE_RADIUS,
          Math.sin(t) * GUIDE_RADIUS,
          0,
        ),
      );
    }
    const geometry = new THREE.BufferGeometry().setFromPoints(points);
    const material = new THREE.LineDashedMaterial({
      color: '#ffffff',
      dashSize: 0.12,
      gapSize: 0.09,
      transparent: true,
      opacity: 0.35,
    });
    const l = new THREE.Line(geometry, material);
    l.computeLineDistances(); // REQUIRED for dashes to render
    return l;
  }, []);

  return <primitive object={line} />;
}

type DragStarter = (
  e: ThreeEvent<PointerEvent>,
  mode: GizmoDragMode,
  axis: THREE.Vector3 | null,
  flyDir: THREE.Vector3 | null,
) => void;

function Axis({ axis, cubeRef, onDragStart }: {
  axis: (typeof AXIS_DEFS)[number];
  cubeRef: RefObject<THREE.Mesh>;
  onDragStart: DragStarter;
}) {
  const [hovered, setHovered] = useState(false);
  const canvas = useThree((s) => s.gl.domElement);
  useCursor(hovered, 'pointer', 'auto', canvas);

  // Shafts are modeled along +Y; rotate them onto the target axis.
  // X: +Y → +X is a -90° rotation about Z; Z: +Y → +Z is +90° about X.
  const rotation: [number, number, number] =
    axis.key === 'x'
      ? [0, 0, -Math.PI / 2]
      : axis.key === 'z'
        ? [Math.PI / 2, 0, 0]
        : [0, 0, 0];

  // Like the cube, the axis shafts are world-aligned (the parent group only
  // rotates the whole gizmo): pressing an axis starts an axis-constrained pan,
  // and releasing without movement flies straight to that axis view.
  const color = hovered ? axis.hover : axis.color;
  const axisVec = new THREE.Vector3(...axis.dir);

  return (
    <group rotation={rotation}>
      <group
        onPointerOver={(e) => {
          e.stopPropagation();
          setHovered(true);
        }}
        onPointerOut={(e) => {
          e.stopPropagation();
          setHovered(false);
        }}
        onPointerDown={(e) => onDragStart(e, 'panAxis', axisVec, axisVec)}
      >
        {/* Shaft from the cube face (0.5) out to ~1.1 */}
        <mesh position={[0, 0.8, 0]}>
          <cylinderGeometry args={[0.035, 0.035, 0.6, 12]} />
          <meshBasicMaterial color={color} />
        </mesh>
        {/* Arrowhead cone, tip at ~1.28 */}
        <mesh position={[0, 1.16, 0]}>
          <coneGeometry args={[0.11, 0.24, 16]} />
          <meshBasicMaterial color={color} />
        </mesh>
        {/* Invisible fat hit target: the shaft is thin, so a transparent
            cylinder keeps the whole axis easy to click (opacity 0 still
            raycasts; depthWrite off keeps it out of the depth buffer). */}
        <mesh position={[0, 0.8, 0]}>
          <cylinderGeometry args={[0.16, 0.16, 1.2, 12]} />
          <meshBasicMaterial transparent opacity={0} depthWrite={false} />
        </mesh>
      </group>
      {/* Label at the axis tip; occluded when the axis points away */}
      <Html
        position={[0, 1.45, 0]}
        center
        pointerEvents="none"
        occlude={[cubeRef]}
        zIndexRange={[10, 0]}
      >
        <span
          style={{
            fontFamily: 'var(--font-mono)',
            fontSize: 11,
            fontWeight: 700,
            color: axis.color,
            textShadow: '0 1px 3px rgba(0,0,0,0.9)',
            userSelect: 'none',
            whiteSpace: 'nowrap',
          }}
        >
          {axis.label}
        </span>
      </Html>
    </group>
  );
}

// ─── ViewportGizmo: HUD overlay (bottom-right of the 3D viewport) ──────────

export default function ViewportGizmo() {
  return (
    <div
      style={{
        position: 'absolute',
        bottom: 24,
        right: 24,
        zIndex: 16,
        width: 140,
        height: 140,
        borderRadius: 12,
        overflow: 'hidden',
        background: 'rgba(13, 17, 23, 0.65)',
        border: '1px solid var(--border)',
        backdropFilter: 'blur(var(--glass-blur))',
        WebkitBackdropFilter: 'blur(var(--glass-blur))',
        boxShadow: 'var(--glass-shadow)',
      }}
    >
      <Canvas
        orthographic
        // zoom 30 @ 140px → visible half-height = 70/30 ≈ 2.33 units, which
        // frames the guide circle (r 1.95) + rings (r 1.55) with margin.
        camera={{ position: [0, 0, 7], zoom: 30, near: 0.1, far: 100 }}
        gl={{ antialias: true, alpha: true }}
        dpr={[1, 2]}
        onCreated={({ gl }) => {
          // Alpha 0: the glass container shows through the canvas
          gl.setClearColor(0x000000, 0);
        }}
      >
        <GizmoScene />
      </Canvas>
    </div>
  );
}
