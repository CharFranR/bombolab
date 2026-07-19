import { useMemo } from 'react';
import { Canvas } from '@react-three/fiber';
import { OrbitControls, Grid, Line } from '@react-three/drei';
import * as THREE from 'three';
import type { RobotDef, Mat4 } from '../kinematics/types';
import { forwardKinematics, dhMatrix } from '../kinematics/forward';

// ─── Colores ───────────────────────────────────────────────────────────────

const COLORS = {
  floor: '#2a2a30',
  grid: '#444450',
  link: '#e8a040',
  joint: '#ffb432',
  base: '#787888',
  effector: '#4cd964',
  tool: '#4cd964',
  axisX: '#ff4444',
  axisY: '#44ff44',
  axisZ: '#4488ff',
};

// ─── Servo geometry ────────────────────────────────────────────────────────

/** Crea un servo como caja con detalles. */
function Servo({ position, rotation, color }: {
  position: [number, number, number];
  rotation: [number, number, number];
  color: string;
}) {
  return (
    <group position={position} rotation={rotation}>
      {/* Cuerpo principal del servo */}
      <mesh castShadow>
        <boxGeometry args={[20, 16, 20]} />
        <meshStandardMaterial color={color} roughness={0.6} metalness={0.2} />
      </mesh>
      {/* Eje del servo (disco superior) */}
      <mesh position={[0, 10, 0]} castShadow>
        <cylinderGeometry args={[6, 6, 4, 16]} />
        <meshStandardMaterial color="#666677" roughness={0.3} metalness={0.4} />
      </mesh>
    </group>
  );
}

// ─── Link (eslabón) ────────────────────────────────────────────────────────

function Link({ from, to, width = 14 }: {
  from: [number, number, number];
  to: [number, number, number];
  width?: number;
}) {
  const fromVec = useMemo(() => new THREE.Vector3(...from), [from]);
  const toVec = useMemo(() => new THREE.Vector3(...to), [to]);

  const { position, quaternion, scale } = useMemo(() => {
    const mid = new THREE.Vector3().addVectors(fromVec, toVec).multiplyScalar(0.5);
    const dir = new THREE.Vector3().subVectors(toVec, fromVec);
    const len = dir.length();
    const up = new THREE.Vector3(0, 1, 0);
    const q = new THREE.Quaternion().setFromUnitVectors(up, dir.clone().normalize());
    return {
      position: [mid.x, mid.y, mid.z] as [number, number, number],
      quaternion: [q.x, q.y, q.z, q.w] as [number, number, number, number],
      scale: [width, len, width] as [number, number, number],
    };
  }, [fromVec, toVec, width]);

  return (
    <mesh position={position} quaternion={quaternion}>
      <boxGeometry args={[1, 1, 1]} />
      <meshStandardMaterial
        color={COLORS.link}
        roughness={0.5}
        metalness={0.3}
      />
      <mesh scale={scale}>
        <boxGeometry args={[1, 1, 1]} />
        <meshStandardMaterial
          color={COLORS.link}
          roughness={0.5}
          metalness={0.3}
          transparent
          opacity={0.15}
        />
      </mesh>
    </mesh>
  );
}

// ─── Base ──────────────────────────────────────────────────────────────────

function Base({ position }: { position: [number, number, number] }) {
  return (
    <group position={position}>
      {/* Plataforma base */}
      <mesh receiveShadow position={[0, -7, 0]}>
        <cylinderGeometry args={[30, 35, 14, 24]} />
        <meshStandardMaterial color="#555566" roughness={0.7} metalness={0.3} />
      </mesh>
      {/* Soporte vertical */}
      <mesh position={[0, 20, 0]} castShadow>
        <cylinderGeometry args={[10, 12, 30, 16]} />
        <meshStandardMaterial color="#606070" roughness={0.6} metalness={0.2} />
      </mesh>
    </group>
  );
}

// ─── Ejes de coordenadas ───────────────────────────────────────────────────

function Axes() {
  const len = 80;
  return (
    <group>
      <Line
        points={[[0, 0, 0], [len, 0, 0]]}
        color={COLORS.axisX}
        lineWidth={2}
      />
      <Line
        points={[[0, 0, 0], [0, len, 0]]}
        color={COLORS.axisY}
        lineWidth={2}
      />
      <Line
        points={[[0, 0, 0], [0, 0, len]]}
        color={COLORS.axisZ}
        lineWidth={2}
      />
    </group>
  );
}

// ─── Escena del robot ──────────────────────────────────────────────────────

function RobotScene({ robot }: { robot: RobotDef }) {
  const { frames } = useMemo(
    () => forwardKinematics(robot.segments, robot.baseTransform),
    [robot.segments, robot.baseTransform],
  );

  // Posiciones: robot X→Three X, robot Z→Three Y (up!), robot Y→Three Z
  // DH tiene Z=up, Three.js tiene Y=up — swap Y↔Z
  const positions = useMemo(
    () => frames.map(f => [f[3], f[11], f[7]] as [number, number, number]),
    [frames],
  );

  return (
    <group>
      {/* Piso */}
      <Grid
        position={[0, -0.5, 0]}
        args={[300, 300]}
        cellSize={10}
        cellThickness={0.5}
        cellColor={COLORS.grid}
        sectionSize={50}
        sectionThickness={1}
        sectionColor="#555560"
        fadeDistance={400}
        infiniteGrid
      />

      {/* Ejes */}
      <Axes />

      {/* Base */}
      <Base position={positions[0]} />

      {/* Links (eslabones entre frames consecutivos) */}
      {positions.slice(0, -1).map((from, i) => (
        <Link key={`link-${i}`} from={from} to={positions[i + 1]} />
      ))}

      {/* Servos (articulaciones) */}
      {positions.slice(1).map((pos, i) => (
        <Servo
          key={`joint-${i}`}
          position={pos}
          rotation={[0, 0, 0]}
          color={i === positions.length - 2 ? COLORS.effector : COLORS.joint}
        />
      ))}

      {/* Tool (punta del marcador) — aplicar tool transform al último frame */}
      {frames.length > 0 && (() => {
        const last = frames[frames.length - 1];
        const tool = dhMatrix({ theta: 0, d: 0, a: robot.toolTransform[0], alpha: 0 }, 0);
        // Aplicar tool: last · tool
        const m = mulMat4(last, tool);
        const tip: [number, number, number] = [m[3], m[11], m[7]];
        return (
          <>
            <Link from={positions[positions.length - 1]} to={tip} width={6} />
            <mesh position={tip}>
              <coneGeometry args={[4, 16, 8]} />
              <meshStandardMaterial color={COLORS.effector} roughness={0.3} />
            </mesh>
          </>
        );
      })()}
    </group>
  );
}

function mulMat4(a: Mat4, b: Mat4): Mat4 {
  const m = (r: number, c: number) =>
    a[r * 4 + 0] * b[0 * 4 + c] +
    a[r * 4 + 1] * b[1 * 4 + c] +
    a[r * 4 + 2] * b[2 * 4 + c] +
    a[r * 4 + 3] * b[3 * 4 + c];
  return [
    m(0,0), m(0,1), m(0,2), m(0,3),
    m(1,0), m(1,1), m(1,2), m(1,3),
    m(2,0), m(2,1), m(2,2), m(2,3),
    m(3,0), m(3,1), m(3,2), m(3,3),
  ];
}

// ─── Viewer principal ──────────────────────────────────────────────────────

export default function RobotViewer({ robot }: { robot: RobotDef }) {
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

        <RobotScene robot={robot} />

        <OrbitControls
          enableDamping
          dampingFactor={0.1}
          minDistance={100}
          maxDistance={1200}
          target={[0, 200, 0]}
        />
      </Canvas>
    </div>
  );
}
