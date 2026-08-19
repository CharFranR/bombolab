function qd = dq_power(qd, t)
%DQ_POWER Potencia fraccional de un cuaternion dual unitario (para ScLERP).

    r = qd.real;
    d = qd.dual;

    v = r(2:4);
    nv = norm(v);
    teta = 2.0 * atan2(nv, r(1));

    if nv > 1e-12
        u = v / nv;
    else
        u = [1; 0; 0];
    end

    rt = [cos(t * teta / 2.0); sin(t * teta / 2.0) * u];
    dt = t * quat_mul(quat_mul(d, quat_conj(r)), rt);

    qd = struct('real', rt, 'dual', dt);
end