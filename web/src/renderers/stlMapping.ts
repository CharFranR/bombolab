/** Maps joint index → STL file name (joint 3 has no STL — skipped). */
export const JOINT_TO_STL: Record<number, { file: string }> = {
  0: { file: 'Base.stl' },
  1: { file: 'Brazo.stl' },
  2: { file: 'Antebrazo.stl' },
  4: { file: 'Muñeca.stl' },
};

/** Gripper jaw STLs attached to the tool-tip frame. */
export const GRIPPER_STLS: { file: string; jawDirection: number }[] = [
  { file: 'Pinza1.stl', jawDirection: -1 }, // opens in -Y
  { file: 'Pinza2.stl', jawDirection:  1 }, // opens in +Y
];

/** All STL filenames in load order (joint STLs first, then gripper STLs). */
export const ALL_STL_FILES: string[] = [
  JOINT_TO_STL[0].file,
  JOINT_TO_STL[1].file,
  JOINT_TO_STL[2].file,
  JOINT_TO_STL[4].file,
  ...GRIPPER_STLS.map((g) => g.file),
];
