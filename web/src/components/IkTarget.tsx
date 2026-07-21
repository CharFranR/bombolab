import { useEffect, useRef } from 'react';
import { useFrame, useThree } from '@react-three/fiber';
import * as THREE from 'three';

/**
 * Esfera target IK — click + drag sobre ella para moverla.
 */
export default function IkTarget({
  position,
  onChange,
  onDragStart,
  onDragEnd,
}: {
  position: [number, number, number];
  onChange: (pos: [number, number, number]) => void;
  onDragStart?: () => void;
  onDragEnd?: () => void;
}) {
  const meshRef = useRef<THREE.Mesh>(null);
  const isDown = useRef(false);
  const plane = useRef(new THREE.Plane(new THREE.Vector3(0, 1, 0), 0));
  const intersect = useRef(new THREE.Vector3());
  const { camera, raycaster, pointer } = useThree();

  // Listener global para soltar el drag
  useEffect(() => {
    const up = () => {
      if (isDown.current) {
        isDown.current = false;
        onDragEnd?.();
      }
    };
    window.addEventListener('pointerup', up);
    return () => window.removeEventListener('pointerup', up);
  }, [onDragEnd]);

  useFrame(() => {
    if (!isDown.current || !meshRef.current) return;
    raycaster.setFromCamera(pointer, camera);
    plane.current.set(new THREE.Vector3(0, 1, 0), -position[2]);
    const hit = raycaster.ray.intersectPlane(plane.current, intersect.current);
    if (hit) {
      onChange([intersect.current.x, intersect.current.z, intersect.current.y]);
    }
  });

  return (
    <mesh
      ref={meshRef}
      position={[position[0], position[2], position[1]]}
      onPointerDown={(e) => {
        e.stopPropagation();
        isDown.current = true;
        onDragStart?.();
      }}
      onWheel={(e) => {
        e.stopPropagation();
        const step = (e as any).deltaY > 0 ? -5 : 5;
        onChange([position[0], position[1], position[2] + step]);
      }}
    >
      <sphereGeometry args={[20, 24, 24]} />
      <meshStandardMaterial
        color="#ff6644"
        emissive="#ff4422"
        emissiveIntensity={0.3}
        roughness={0.3}
        metalness={0.1}
      />
      <mesh rotation={[Math.PI / 2, 0, 0]}>
        <torusGeometry args={[26, 2, 12, 24]} />
        <meshStandardMaterial color="#ff8866" emissive="#ff4422" emissiveIntensity={0.2} />
      </mesh>
    </mesh>
  );
}
