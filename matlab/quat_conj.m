function qc = quat_conj(q)
%QUAT_CONJ Conjugado de un cuaternion: [a; b; c; d] -> [a; -b; -c; -d].

    qc = [q(1); -q(2); -q(3); -q(4)];
end