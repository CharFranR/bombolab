function qc = dq_conj(qd)
%DQ_CONJ Conjugado de un cuaternion dual (inverso si es unitario).

    qc = struct('real', quat_conj(qd.real), 'dual', quat_conj(qd.dual));
end