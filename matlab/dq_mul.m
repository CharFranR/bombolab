function r = dq_mul(qd1, qd2)
%DQ_MUL Producto de cuaterniones duales.

    r = struct('real', quat_mul(qd1.real, qd2.real), ...
               'dual', quat_mul(qd1.real, qd2.dual) + quat_mul(qd1.dual, qd2.real));
end