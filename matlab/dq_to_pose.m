function [R, p] = dq_to_pose(qd)
%DQ_TO_POSE Pose (R, p) desde un cuaternion dual unitario.

    R = quat_to_rot(qd.real);

    v = 2.0 * quat_mul(qd.dual, quat_conj(qd.real));
    p = v(2:4);
end