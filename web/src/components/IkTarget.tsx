import { useRef } from 'react';
import { useFrame, useThree } from '@react-three/fiber';
import * as THREE from 'three';

/**
 * Esfera target IK — se arrastra haciendo click + drag sobre ella.
 * Proyecta el movimiento al plano horizontal en Z=target.z (DH Z = Three.js Y swap).
 */
export default function IkTarget({
  position,
  onChange,
}: {
  position: [number, number, number];
  onChange: (pos: [number, number, number]) => void;
}) {
  const meshRef = useRef<THREE.Mesh>(null);
  const dragging = useRef(false);
  const plane = useRef(new THREE.Plane(new THREE.Vector3(0, 1, 0), 0));
  const intersect = useRef(new THREE.Vector3());
  const { camera, raycaster, pointer } = useThree();

  useFrame(() => {
    if (!dragging.current || !meshRef.current) return;

    raycaster.setFromCamera(pointer, camera);
    // DH Z → Three.js Y: plano horizontal a la altura del target
    plane.current.set(new THREE.Vector3(0, 1, 0), -position[2]);
    const hit = raycaster.ray.intersectPlane(plane.current, intersect.current);
    if (hit) {
      onChange([
        intersect.current.x,
        intersect.current.z,
        intersect.current.y,
      ]);
    }
  });

  return (
    <mesh
      ref={meshRef}
      position={[position[0], position[2], position[1]]}
      onPointerDown={(e) => {
        e.stopPropagation();
        dragging.current = true;
      }}
      onPointerUp={() => { dragging.current = false; }}
      onPointerLeave={() => { dragging.current = false; }}
    >
      <sphereGeometry args={[8, 16, 16]} />
      <meshStandardMaterial
        color="#ff6644"
        emissive="#ff4422"
        emissiveIntensity={0.3}
        roughness={0.3}
        metalness={0.1}
      />
    </mesh>
  );
}
