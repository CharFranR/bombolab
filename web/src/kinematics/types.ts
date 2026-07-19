/** Parámetros DH (estándar) para un eslabón. */
export interface DHParams {
  theta: number;  // rad — ángulo fijo (se suma q para revolute)
  d: number;      // mm — offset en Z
  a: number;      // mm — offset en X
  alpha: number;  // rad — twist en X
}

/** Parámetros DH + valor articular q. */
export interface Segment extends DHParams {
  q: number; // rad — valor actual de la articulación
}

/** Matriz 4×4 como array plano [row0col0, row0col1, ...] */
export type Mat4 = [
  number, number, number, number,
  number, number, number, number,
  number, number, number, number,
  number, number, number, number,
];

/** Punto 3D + orientación (resultado de FK) */
export interface Pose {
  x: number; y: number; z: number;
  /** Matriz de rotación 3×3 como array plano */
  rot: [number, number, number, number, number, number, number, number, number];
}

export interface RobotDef {
  name: string;
  segments: Segment[];
  baseTransform: [number, number, number];   // [x, y, z] mm
  toolTransform: [number, number, number];  // [x, y, z] mm
}
