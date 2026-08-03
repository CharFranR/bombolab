import { useMemo } from 'react';
import { Grid, Line } from '@react-three/drei';
import * as THREE from 'three';
import type { RobotRendererProps } from './types';
import IkTarget from '../components/IkTarget';

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
    // Guard: two consecutive FK frames can coincide (e.g. elbow at full
    // extension), making dir a zero vector. normalize() of a zero vector
    // yields NaN, which poisons the quaternion and the whole mesh.
    // Fall back to the identity quaternion when the link has no length.
    const q = len < 1e-6
      ? new THREE.Quaternion()
      : new THREE.Quaternion().setFromUnitVectors(up, dir.clone().normalize());
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

// ─── Escena del robot (renderizado simple) ─────────────────────────────────

export default function SimpleRobotScene({
  frames: poses,
  gripper = 0,
  workspacePoints = [],
  trajectoryPoints = [],
  trajectoryReveal,
  ikTarget,
  onIkTargetChange,
  onDragStart,
  onDragEnd,
}: RobotRendererProps) {
  // poses includes base, joint frames, and tool tip as last element
  // geoFrames = everything except tool tip (base + all joint frames)
  const geoFrames = poses.slice(0, -1);

  // P3 (Stage 3C): Float32Array del workspace memoizado — se reconstruye
  // solo cuando cambian los puntos, no en cada render de React.
  const workspaceArray = useMemo(
    () => (workspacePoints.length > 0 ? new Float32Array(workspacePoints.flat()) : undefined),
    [workspacePoints],
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

      {/* Base — fija en el ground */}
      <Base position={poses[0]?.pos ?? [0, 0, 0]} />

      {/* Links (eslabones entre frames consecutivos, sin incluir tool tip) */}
      {geoFrames.slice(0, -1).map((p, i) => (
        <Link key={`link-${i}`} from={p.pos} to={geoFrames[i + 1].pos} />
      ))}

      {/* Servos — cada uno en su frame i */}
      {geoFrames.slice(0, -1).map((p, i) => (
        <group key={`joint-${i}`} position={new THREE.Vector3(...p.pos)} quaternion={new THREE.Quaternion(...p.quat)}>
          <Servo
            position={[0, 0, 0]}
            rotation={[0, 0, 0]}
            color={i === 0 ? COLORS.base : i >= geoFrames.length - 2 ? COLORS.effector : COLORS.joint}
          />
        </group>
      ))}

      {/* Gripper paralelo 75mm — acostado en XY, mordazas abren en Y */}
      {poses.length > 1 && (() => {
        const tp = poses[poses.length - 1];
        const tq = new THREE.Quaternion(...tp.quat);
        const jawOpen = (1 - gripper / 100) * 10; // 0%=abierto(10mm), 100%=cerrado(0mm)
        return (
          <>
            {/* Link J5 → punta = cuerpo del gripper */}
            <Link from={geoFrames[geoFrames.length - 1].pos} to={tp.pos} width={8} />
            <group position={new THREE.Vector3(...tp.pos)} quaternion={tq}>
              {/* Cuerpo (riel) */}
              <mesh position={[-30, 0, 0]}>
                <boxGeometry args={[60, 6, 12]} />
                <meshStandardMaterial color="#7777aa" roughness={0.4} metalness={0.4} />
              </mesh>
              {/* Mordaza izquierda (abre en -Y) */}
              <mesh position={[8, -7 - jawOpen, 0]}>
                <boxGeometry args={[24, 4, 10]} />
                <meshStandardMaterial color="#ccccdd" roughness={0.3} metalness={0.5} />
              </mesh>
              {/* Mordaza derecha (abre en +Y) */}
              <mesh position={[8, 7 + jawOpen, 0]}>
                <boxGeometry args={[24, 4, 10]} />
                <meshStandardMaterial color="#ccccdd" roughness={0.3} metalness={0.5} />
              </mesh>
              {/* Diente de sujeción izq */}
              <mesh position={[20, -7 - jawOpen, 0]}>
                <boxGeometry args={[4, 4, 14]} />
                <meshStandardMaterial color="#9999bb" roughness={0.6} metalness={0.2} />
              </mesh>
              {/* Diente de sujeción der */}
              <mesh position={[20, 7 + jawOpen, 0]}>
                <boxGeometry args={[4, 4, 14]} />
                <meshStandardMaterial color="#9999bb" roughness={0.6} metalness={0.2} />
              </mesh>
            </group>
          </>
        );
      })()}

      {/* IK target */}
      {ikTarget && onIkTargetChange && (
        <IkTarget position={ikTarget} onChange={onIkTargetChange} onDragStart={onDragStart} onDragEnd={onDragEnd} />
      )}

      {/* Workspace point cloud — Float32Array memoizado (P3, Stage 3C) */}
      {workspacePoints.length > 0 && (
        <points>
          <bufferGeometry>
            <bufferAttribute
              attach="attributes-position"
              count={workspacePoints.length}
              array={workspaceArray}
              itemSize={3}
            />
          </bufferGeometry>
          <pointsMaterial size={5} color="#66aaff" transparent opacity={0.35} depthWrite={false} />
        </points>
      )}

      {/* Drawing trajectory — blue polyline, revealed progressively */}
      {trajectoryPoints.length > 1 && (trajectoryReveal == null || trajectoryReveal > 1) && (
        <Line
          points={trajectoryReveal != null ? trajectoryPoints.slice(0, trajectoryReveal) : trajectoryPoints}
          color="#4488ff"
          lineWidth={2}
          transparent
          opacity={0.9}
        />
      )}
    </group>
  );
}
