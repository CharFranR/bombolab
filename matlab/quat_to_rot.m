function R = quat_to_rot(q)
%QUAT_TO_ROT Matriz de rotacion 3x3 desde cuaternion unitario.

    a = q(1); b = q(2); c = q(3); d = q(4);

    R = [1 - 2*(c^2 + d^2),     2*(b*c - a*d),     2*(b*d + a*c);
             2*(b*c + a*d), 1 - 2*(b^2 + d^2),     2*(c*d - a*b);
             2*(b*d - a*c),     2*(c*d + a*b), 1 - 2*(b^2 + c^2)];
end