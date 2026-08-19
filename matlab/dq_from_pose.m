function qd = dq_from_pose(R, p)
%DQ_FROM_POSE Cuaternion dual unitario desde pose (R, p).

    real_q = quat_from_rot(R);
    t = [0; p(1); p(2); p(3)];
    dual_q = 0.5 * quat_mul(t, real_q);

    qd = struct('real', real_q, 'dual', dual_q);
end