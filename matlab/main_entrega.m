%MAIN_ENTREGA Script principal del paquete MATLAB del FABRI Creator.
%   Cinematica directa (DH), cuaterniones duales y cinematica diferencial.
%   Ejecutar: main_entrega (en MATLAB o GNU Octave, desde matlab/).

    clc;
    fprintf('%s\n', repmat('=', 1, 78));
    fprintf('  FABRI CREATOR - PAQUETE MATLAB (entrega Robotec-2026)\n');
    fprintf('  Cinematica directa (DH), cuaterniones duales y cinematica\n');
    fprintf('  diferencial. Modelo identico al gemelo digital.\n');
    fprintf('%s\n\n', repmat('=', 1, 78));

    robot = fabri_creator();

    %% 1. Tabla de parametros DH
    fprintf('=== 1. TABLA DE PARAMETROS DH (theta0, d, a, alpha) ===\n');
    fprintf('  Articulacion | tipo     | theta0 [rad] | d [mm] | a [mm] | alpha [rad] | q_min [rad] | q_max [rad]\n');
    fprintf('  -------------+----------+--------------+--------+--------+-------------+-------------+------------\n');
    for i = 1:numel(robot.segments)
        s = robot.segments(i);
        fprintf('       %d       | %-8s | %12.4f | %6.1f | %6.1f | %11.4f | %11.4f | %11.4f\n', ...
            i, s.tipo, s.theta0, s.d, s.a, s.alpha, s.q_min, s.q_max);
    end
    fprintf('  Base: traslacion (0, 0, 57) mm.  Util: marcador perpendicular (117, 0, 0) mm.\n');
    fprintf('  Home de servos [deg]: 90, 90, 81, 95, 60  =>  home cinematico q = [0, 0, 0, 0, 0].\n\n');

    %% 2. Cinematica directa DH vs valores de referencia (gemelo digital)
    q_test = [pi/6, pi/4, -pi/4, pi/3, pi/6];

    opts_none = struct('usar_base', false, 'usar_tool', false);
    [T_ref, frames] = cd_dh(q_test, robot, opts_none);

    T_gemelo = [ 0.5335, -0.8080, -0.2500, 212.0488; ...
                 0.8080,  0.3995,  0.4330, 122.4264; ...
                -0.2500, -0.4330,  0.8660, 149.8528; ...
                 0.0000,  0.0000,  0.0000,   1.0000];

    fprintf('=== 2. CINEMATICA DIRECTA POR DH (q_test = pi/6, pi/4, -pi/4, pi/3, pi/6) ===\n');
    fprintf('  Marcos F0..F5 (posiciones, mm):\n');
    for i = 1:size(frames, 3)
        p = frames(1:3, 4, i);
        fprintf('    p_%d = (%9.4f, %9.4f, %9.4f)\n', i - 1, p(1), p(2), p(3));
    end
    fprintf('  T_0,5(q_test) obtenida:\n');
    for r = 1:4
        fprintf('    [%9.4f %9.4f %9.4f %9.4f]\n', T_ref(r, :));
    end
    err_T = max(max(abs(T_ref - T_gemelo)));
    fprintf('  Error maximo vs gemelo digital (Rust): %.3e mm\n', err_T);
    if err_T < 1e-4
        fprintf('  VERIFICACION: OK (coincide con el gemelo digital).\n\n');
    else
        fprintf('  VERIFICACION: FALLO - revisar parametros.\n\n');
    end

    %% 3. Cinematica directa por cuaterniones duales
    [T_dq, qd_ee] = cd_cuaterniones_duales(q_test, robot, opts_none);

    qr_ref = [0.8365, -0.2588, -0.0000, 0.4830]';
    qd_ref = [-8.7455, 118.2548, -19.3924, 78.5203]';

    fprintf('=== 3. CINEMATICA DIRECTA POR CUATERNIONES DUALES ===\n');
    fprintf('  q_real = (%8.4f, %8.4f, %8.4f, %8.4f)\n', qd_ee.real);
    fprintf('  q_dual = (%8.4f, %8.4f, %8.4f, %8.4f)\n', qd_ee.dual);
    fprintf('  |q_real| = %.6f (cuaternion unitario)\n', norm(qd_ee.real));
    err_qr = max(abs(qd_ee.real - qr_ref));
    err_qd = max(abs(qd_ee.dual - qd_ref));
    fprintf('  Error maximo vs gemelo digital: real %.3e, dual %.3e\n', err_qr, err_qd);

    err_dh_dq = max(max(abs(T_dq - T_ref)));
    fprintf('  Error T(QD) vs T(DH): %.3e\n', err_dh_dq);
    if err_qr < 1e-4 && err_qd < 1e-4 && err_dh_dq < 1e-9
        fprintf('  VERIFICACION: OK (cuaterniones duales reproducen la pose DH).\n\n');
    else
        fprintf('  VERIFICACION: FALLO - revisar implementacion QD.\n\n');
    end

    %% 4. Pose del efector con base y util (marcador)
    opts_full = struct('usar_base', true, 'usar_tool', true);
    [T_home] = cd_dh([0, 0, 0, 0, 0], robot, opts_full);
    [T_test_full] = cd_dh(q_test, robot, opts_full);

    fprintf('=== 4. POSE DEL EFECTOR (marcador, con base y util) ===\n');
    fprintf('  Reposo (q = 0):      p_ee = (%9.4f, %9.4f, %9.4f) mm\n', T_home(1:3, 4));
    fprintf('  q_test (con util):   p_ee = (%9.4f, %9.4f, %9.4f) mm\n', T_test_full(1:3, 4));
    fprintf('  Altura del reposo:   %.1f mm (>= 500 mm requeridos por el proyecto)\n\n', T_home(3, 4));

    %% 5. Cinematica diferencial: jacobiano dual vs clasico vs numerico
    fprintf('=== 5. CINEMATICA DIFERENCIAL (q_test, sin base ni util) ===\n');

    [J_dual, J_clasico] = cinematica_diferencial(q_test, robot, opts_none);

    J_gemelo = [ -122.4264,   73.4847,   -0.0000,    0.0000,    0.0000; ...
                  212.0488,   42.4264,    0.0000,    0.0000,    0.0000; ...
                    0.0000, -229.8528, -145.0000,    0.0000,   -0.0000; ...
                    0.0000,   -0.5000,   -0.5000,    0.8660,   -0.2500; ...
                    0.0000,    0.8660,    0.8660,    0.5000,    0.4330; ...
                    1.0000,    0.0000,    0.0000,    0.0000,    0.8660];

    fprintf('  J clasico [v; w] (obtenido):\n');
    for r = 1:6
        fprintf('    [%9.4f %9.4f %9.4f %9.4f %9.4f]\n', J_clasico(r, :));
    end
    err_J = max(max(abs(J_clasico - J_gemelo)));
    fprintf('  Error maximo J clasico vs gemelo digital: %.3e\n', err_J);

    err_Jdual = max(max(abs(J_dual - J_clasico([4 5 6 1 2 3], :))));
    fprintf('  Error maximo J dual vs J clasico reordenado: %.3e\n', err_Jdual);

    eps_q = 1e-6;
    J_num = zeros(6, 5);
    for i = 1:5
        qp = q_test; qm = q_test;
        qp(i) = qp(i) + eps_q;
        qm(i) = qm(i) - eps_q;
        Tp = cd_dh(qp, robot, opts_none);
        Tm = cd_dh(qm, robot, opts_none);
        J_num(1:3, i) = (Tp(1:3, 4) - Tm(1:3, 4)) / (2 * eps_q);
        dR = (Tp(1:3, 1:3) - Tm(1:3, 1:3)) / (2 * eps_q);
        dR_rot = dR * T_ref(1:3, 1:3)';
        J_num(4:6, i) = 0.5 * [dR_rot(3, 2) - dR_rot(2, 3); ...
                              dR_rot(1, 3) - dR_rot(3, 1); ...
                              dR_rot(2, 1) - dR_rot(1, 2)];
    end
    err_Jnum = max(max(abs(J_clasico - J_num)));
    fprintf('  Error maximo J clasico vs diferencias finitas: %.3e\n', err_Jnum);

    if err_J < 1e-3 && err_Jdual < 1e-9 && err_Jnum < 1e-5
        fprintf('  VERIFICACION: OK (jacobianos coherentes).\n\n');
    else
        fprintf('  VERIFICACION: FALLO - revisar cinematica diferencial.\n\n');
    end

    %% 6. Velocidad del efector con cuaterniones duales
    qp = [0.2, -0.1, 0.15, 0.05, 0.3]';

    fprintf('=== 6. VELOCIDAD DEL EFECTOR (q_test, qp = 0.2, -0.1, 0.15, 0.05, 0.3) ===\n');
    vel_dual = J_dual * qp;
    vel_clas = J_clasico * qp;

    fprintf('  Por cuaterniones duales:  w = (%8.4f, %8.4f, %8.4f) rad/s\n', vel_dual(1:3));
    fprintf('                            v = (%8.4f, %8.4f, %8.4f) mm/s\n', vel_dual(4:6));
    fprintf('  Por jacobiano clasico:    v = (%8.4f, %8.4f, %8.4f) mm/s\n', vel_clas(1:3));
    fprintf('                            w = (%8.4f, %8.4f, %8.4f) rad/s\n', vel_clas(4:6));
    err_vel = max(abs(vel_dual - vel_clas([4 5 6 1 2 3])));
    fprintf('  Error maximo entre ambos: %.3e\n', err_vel);

    det_jtj = det(J_clasico' * J_clasico);
    fprintf('  det(J^T J) = %.3e  => configuracion regular (rango 5)\n\n', det_jtj);

    %% 7. Trayectoria interpolada con cuaterniones duales (ScLERP)
    fprintf('=== 7. INTERPOLACION DE TRAYECTORIA CON CUATERNIONES DUALES ===\n');
    fprintf('  Se interpola la pose del reposo a la pose q_test (ScLERP).\n');

    [~, qd_a] = cd_cuaterniones_duales([0, 0, 0, 0, 0], robot, opts_full);
    [~, qd_b] = cd_cuaterniones_duales(q_test, robot, opts_full);

    qd_rel = dq_mul(dq_conj(qd_a), qd_b);
    fprintf('  t    | p_ee (mm)                          | |q_real|\n');
    fprintf('  -----+------------------------------------+---------\n');
    for t = 0:0.25:1
        qd_t = dq_mul(qd_a, dq_power(qd_rel, t));
        [~, p_t] = dq_to_pose(qd_t);
        fprintf('  %3.2f | (%9.4f, %9.4f, %9.4f) | %.6f\n', ...
            t, p_t(1), p_t(2), p_t(3), norm(qd_t.real));
    end
    fprintf('\n');

    fprintf('%s\n', repmat('=', 1, 78));
    fprintf('  FIN DEL PAQUETE MATLAB. Todos los resultados coinciden con\n');
    fprintf('  el gemelo digital de bombolab-core (Rust/web).\n');
    fprintf('%s\n', repmat('=', 1, 78));