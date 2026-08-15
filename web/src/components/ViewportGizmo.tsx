import { useCallback, useMemo, useRef, useState, type RefObject } from 'react';
import { Canvas, useFrame, useThree, type ThreeEvent } from '@react-three/fiber';
import { Edges, Html, useCursor } from '@react-three/drei';
import * as THREE from 'three';

// ─── Shared bridge: main canvas → gizmo canvas ────────────────────────────
// Module-level mutable singleton so the two canvases never re-render each
// other. GizmoSync (mounted in the MAIN canvas) publishes the camera
// quaternion every frame; the gizmo scene mirrors it so the cube/axes always
// show the current view orientation (Blender-style navigation gizmo).

export const gizmoLink = {
  quaternion: new THREE.Quaternion(),
  // Distance from the main camera to the orbit target, published each frame so
  // click-to-fly can preserve the user's current zoom (clamped to sane bounds).
  cameraDistance: 900,
  // Click-to-fly request: set by the gizmo canvas, consumed by GizmoSync.
  flight: null as { dir: THREE.Vector3; dist: number } | null,
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

  useFrame((state, delta) => {
    const camera = state.camera;
    gizmoLink.quaternion.copy(camera.quaternion);
    gizmoLink.cameraDistance = camera.position.distanceTo(ROBOT_TARGET);

    // OrbitControls (makeDefault in RobotViewer) exposes .enabled + .target,
    // but R3F types state.controls as EventDispatcher | null → downcast to
    // the narrow surface we actually touch.
    const controls = state.controls as unknown as
      | { enabled: boolean; target: THREE.Vector3 }
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
    } else if (gizmoLink.flight) {
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

const FACE_REST = '#18202b';
const FACE_HOVER = '#00f2fe'; // design cyan — matches the app accent

function GizmoScene() {
  const group = useRef<THREE.Group>(null!);
  const cubeRef = useRef<THREE.Mesh>(null!);
  const [faceHovered, setFaceHovered] = useState<number | null>(null);
  const canvas = useThree((s) => s.gl.domElement);
  useCursor(faceHovered !== null, 'pointer', 'auto', canvas);

  // Fixed materials for the six cube faces (BoxGeometry group order:
  // +X, -X, +Y, -Y, +Z, -Z). MeshBasicMaterial = unlit flat CAD look.
  const faceMaterials = useMemo(
    () =>
      Array.from(
        { length: 6 },
        () =>
          new THREE.MeshBasicMaterial({
            color: FACE_REST,
            transparent: true,
            opacity: 0.9,
            depthWrite: false,
          }),
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
      {/* Orientation cube: dark glass faces + crisp border edges. The local
          face normal is axis-aligned and the group rotation never touches it,
          so the clicked normal IS the world direction to fly toward. */}
      <mesh
        ref={cubeRef}
        material={faceMaterials}
        onPointerOver={handleFaceOver}
        onPointerOut={handleFaceOut}
        onClick={(e) => {
          e.stopPropagation();
          if (e.face?.normal) requestFlight(e.face.normal);
        }}
      >
        <boxGeometry args={[1, 1, 1]} />
        <Edges color="rgba(255,255,255,0.12)" lineWidth={1} />
      </mesh>

      {/* Cartesian axes with arrowheads + X/Y/Z labels */}
      {AXIS_DEFS.map((axis) => (
        <Axis key={axis.key} axis={axis} cubeRef={cubeRef} />
      ))}
    </group>
  );
}

function Axis({ axis, cubeRef }: {
  axis: (typeof AXIS_DEFS)[number];
  cubeRef: RefObject<THREE.Mesh>;
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
  // rotates the whole gizmo) → clicking an axis flies straight to that view.
  const color = hovered ? axis.hover : axis.color;

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
        onClick={(e) => {
          e.stopPropagation();
          requestFlight(new THREE.Vector3(...axis.dir));
        }}
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
        width: 130,
        height: 130,
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
        camera={{ position: [0, 0, 7], zoom: 40, near: 0.1, far: 100 }}
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
