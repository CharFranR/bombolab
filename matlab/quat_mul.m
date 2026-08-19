function r = quat_mul(p, q)
%QUAT_MUL Producto de cuaterniones (Hamilton), p y q como vectores [a; b; c; d].

    a1 = p(1); b1 = p(2); c1 = p(3); d1 = p(4);
    a2 = q(1); b2 = q(2); c2 = q(3); d2 = q(4);

    r = [a1*a2 - b1*b2 - c1*c2 - d1*d2;
         a1*b2 + b1*a2 + c1*d2 - d1*c2;
         a1*c2 - b1*d2 + c1*a2 + d1*b2;
         a1*d2 + b1*c2 - c1*b2 + d1*a2];
end