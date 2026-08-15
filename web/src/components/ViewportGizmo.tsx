import { useMemo, useRef, type RefObject } from 'react';
import { Canvas, useFrame } from '@react-three/fiber';
import { Edges, Html } from '@react-three/drei';
import * as THREE from 'three';

// ─── Shared bridge: main canvas → gizmo canvas ────────────────────────────
// Module-level mutable singleton so the two canvases never re-render each
// other. GizmoSync (mounted in the MAIN canvas) publishes the camera
// quaternion every frame; the gizmo scene mirrors it so the cube/axes always
// show the current view orientation (Blender-style navigation gizmo).

export const gizmoLink = {
  quaternion: new THREE.Quaternion(),
};

// ─── GizmoSync: publish main-camera orientation every frame ────────────────
// Mounted INSIDE the main canvas. useFrame runs after the scene renders, so
// the gizmo always mirrors the camera's final (post-orbit, post-flight)
// orientation. The two canvases never re-render each other — the quaternion
// crosses the boundary by value through the module-level singleton.

export function GizmoSync() {
  useFrame(({ camera }) => {
    gizmoLink.quaternion.copy(camera.quaternion);
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

function GizmoScene() {
  const group = useRef<THREE.Group>(null!);
  const cubeRef = useRef<THREE.Mesh>(null!);

  // Fixed materials for the six cube faces (BoxGeometry group order:
  // +X, -X, +Y, -Y, +Z, -Z). MeshBasicMaterial = unlit flat CAD look.
  const faceMaterials = useMemo(
    () =>
      Array.from(
        { length: 6 },
        () =>
          new THREE.MeshBasicMaterial({
            color: '#18202b',
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

  return (
    <group ref={group}>
      {/* Orientation cube: dark glass faces + crisp border edges */}
      <mesh ref={cubeRef} material={faceMaterials}>
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
  // Shafts are modeled along +Y; rotate them onto the target axis.
  // X: +Y → +X is a -90° rotation about Z; Z: +Y → +Z is +90° about X.
  const rotation: [number, number, number] =
    axis.key === 'x'
      ? [0, 0, -Math.PI / 2]
      : axis.key === 'z'
        ? [Math.PI / 2, 0, 0]
        : [0, 0, 0];

  return (
    <group rotation={rotation}>
      {/* Shaft from the cube face (0.5) out to ~1.1 */}
      <mesh position={[0, 0.8, 0]}>
        <cylinderGeometry args={[0.035, 0.035, 0.6, 12]} />
        <meshBasicMaterial color={axis.color} />
      </mesh>
      {/* Arrowhead cone, tip at ~1.28 */}
      <mesh position={[0, 1.16, 0]}>
        <coneGeometry args={[0.11, 0.24, 16]} />
        <meshBasicMaterial color={axis.color} />
      </mesh>
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
