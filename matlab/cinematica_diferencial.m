function [J_dual, J_clasico] = cinematica_diferencial(q, robot, opts)
%CINEMATICA_DIFERENCIAL Jacobiano del efector por cuaterniones duales y clasico.
%   J_dual filas [w; v]; J_clasico filas [v; w] (convencion bombolab-core).

    if nargin < 3 || isempty(opts)
        opts = struct();
    end
    if ~isfield(opts, 'usar_base'), opts.usar_base = true; end
    if ~isfield(opts, 'usar_tool'), opts.usar_tool = true; end

    n = numel(robot.segments);
    if numel(q) ~= n
        error('cinematica_diferencial:q', 'q debe tener %d articulaciones.', n);
    end

    if opts.usar_base
        current = robot.base;
    else
        current = eye(4);
    end
    frames = zeros(4, 4, n + 1);
    frames(:, :, 1) = current;
    for i = 1:n
        current = current * dh_matrix(robot.segments(i), q(i));
        frames(:, :, i + 1) = current;
    end
    T_ee = current;
    if opts.usar_tool
        T_ee = T_ee * robot.tool;
    end
    p_ee = T_ee(1:3, 4);

    J_clasico = zeros(6, n);
    for i = 1:n
        if i == 1
            prev = eye(4);
        else
            prev = frames(:, :, i);
        end
        if strcmp(robot.segments(i).tipo, 'twist')
            axis = prev(1:3, 1);
            p_i  = frames(1:3, 4, i + 1);
        else
            axis = prev(1:3, 3);
            p_i  = prev(1:3, 4);
        end
        J_clasico(1:3, i) = cross(axis, p_ee - p_i);
        J_clasico(4:6, i) = axis;
    end

    if opts.usar_base
        qd_base = dq_from_pose(robot.base(1:3, 1:3), robot.base(1:3, 4));
    else
        qd_base = dq_from_pose(eye(3), [0; 0; 0]);
    end
    if opts.usar_tool
        qd_tool = dq_from_pose(robot.tool(1:3, 1:3), robot.tool(1:3, 4));
    else
        qd_tool = dq_from_pose(eye(3), [0; 0; 0]);
    end

    qd_list = cell(1, n);
    for i = 1:n
        A = dh_matrix(robot.segments(i), q(i));
        qd_list{i} = dq_from_pose(A(1:3, 1:3), A(1:3, 4));
    end

    qd_total = qd_base;
    for i = 1:n
        qd_total = dq_mul(qd_total, qd_list{i});
    end
    qd_total = dq_mul(qd_total, qd_tool);

    prefix = cell(1, n + 1);
    suffix = cell(1, n + 2);
    prefix{1} = qd_base;
    for i = 1:n
        prefix{i + 1} = dq_mul(prefix{i}, qd_list{i});
    end
    suffix{n + 1} = qd_tool;
    for i = n:-1:1
        suffix{i} = dq_mul(qd_list{i}, suffix{i + 1});
    end

    J_dual = zeros(6, n);
    for i = 1:n
        seg = robot.segments(i);
        theta = q(i) + seg.theta0;

        if strcmp(seg.tipo, 'twist')
            e = [1; 0; 0];
            t_dot = [0; 0; 0; 0];
            t_i = [0; seg.a; seg.d; 0];
        else
            e = [0; 0; 1];
            t_dot = [0; -seg.a*sin(theta); seg.a*cos(theta); 0];
            t_i = [0; seg.a*cos(theta); seg.a*sin(theta); seg.d];
        end
        r_i = qd_list{i}.real;

        r_dot_i = 0.5 * quat_mul([0; e], r_i);
        qd_dot_i = struct('real', r_dot_i, ...
            'dual', 0.5 * (quat_mul(t_dot, r_i) + quat_mul(t_i, r_dot_i)));

        qd_dot_total = dq_mul(dq_mul(prefix{i}, qd_dot_i), suffix{i + 1});

        v_dual = dq_mul(qd_dot_total, dq_conj(qd_total));
        v_dual.real = 2.0 * v_dual.real;
        v_dual.dual = 2.0 * v_dual.dual;

        w     = v_dual.real(2:4);
        v_org = v_dual.dual(2:4);
        v_ee  = v_org + cross(w, p_ee);

        J_dual(1:3, i) = w;
        J_dual(4:6, i) = v_ee;
    end
end