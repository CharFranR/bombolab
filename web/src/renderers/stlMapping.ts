/**
 * STL load order and per-mesh metadata.
 *
 * Joint mapping (from user):
 *   0 = base rotation    → Base.stl, Tapa Base.stl
 *   1 = shoulder         → Eje Central.stl    (base → shoulder, 95mm)
 *   2 = elbow            → Antebrazo.stl      (shoulder → elbow, 162mm)
 *   3 = wrist roll       → Brazo.stl          (elbow → wrist, 111mm)
 *   4 = wrist tilt       → Muñeca.stl         (wrist, 41mm)
 *  -1 = tool-tip frame   → Base de la garra, Engranajes, Pinzas
 *
 * jawDirection: 0 = rigid part, -1/+1 = gripper jaw opening axis (±Y).
 */

export interface StlMeta {
  file: string;
  parentJoint: number; // index into FK frames[] (-1 = tool-tip / last frame)
  jawDirection: number; // 0, -1, or +1
}

/** All STLs in load order — MUST match STL_META index-for-index. */
export const ALL_STL_FILES: string[] = [
  'Base.stl',
  'Tapa Base.stl',
  'Eje Central.stl',
  'Antebrazo.stl',
  'Brazo.stl',
  'Muñeca.stl',
  'Base de la garra.stl',
  'Engranaje1.stl',
  'Engranaje2.stl',
  'Pinza1.stl',
  'Pinza2.stl',
];

export const STL_META: StlMeta[] = [
  { file: 'Base.stl',              parentJoint:  0, jawDirection:  0 },
  { file: 'Tapa Base.stl',         parentJoint:  0, jawDirection:  0 },
  { file: 'Eje Central.stl',       parentJoint:  1, jawDirection:  0 },
  { file: 'Antebrazo.stl',         parentJoint:  2, jawDirection:  0 },
  { file: 'Brazo.stl',             parentJoint:  3, jawDirection:  0 },
  { file: 'Muñeca.stl',            parentJoint:  4, jawDirection:  0 },
  { file: 'Base de la garra.stl',  parentJoint: -1, jawDirection:  0 },
  { file: 'Engranaje1.stl',        parentJoint: -1, jawDirection:  0 },
  { file: 'Engranaje2.stl',        parentJoint: -1, jawDirection:  0 },
  { file: 'Pinza1.stl',            parentJoint: -1, jawDirection:  1 },
  { file: 'Pinza2.stl',            parentJoint: -1, jawDirection: -1 },
];
