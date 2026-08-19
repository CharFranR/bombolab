function A = dh_matrix(seg, q)
%DH_MATRIX Matriz de transformacion homogenea A_i de un eslabon (DH).

    switch seg.tipo
        case 'twist'
            alpha = seg.alpha + q;
            c = cos(alpha);
            s = sin(alpha);
            A = [1, 0, 0, seg.a; ...
                 0, c, -s, seg.d; ...
                 0, s,  c, 0.0; ...
                 0, 0,  0, 1.0];
        otherwise
            theta = q + seg.theta0;
            cz = cos(theta);
            sz = sin(theta);
            cx = cos(seg.alpha);
            sx = sin(seg.alpha);
            A = [cz, -sz*cx,  sz*sx, seg.a*cz; ...
                 sz,  cz*cx, -cz*sx, seg.a*sz; ...
                 0,      sx,     cx,     seg.d; ...
                 0,       0,      0,       1.0];
    end
end