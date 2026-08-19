function [T_ee, qd_ee, qd_frames] = cd_cuaterniones_duales(q, robot, opts)
%CD_CUATERNIONES_DUALES Cinematica directa mediante cuaterniones duales.

    if nargin < 3 || isempty(opts)
        opts = struct();
    end
    if ~isfield(opts, 'usar_base'), opts.usar_base = true; end
    if ~isfield(opts, 'usar_tool'), opts.usar_tool = true; end

    n = numel(robot.segments);
    if numel(q) ~= n
        error('cd_cuaterniones_duales:q', 'q debe tener %d articulaciones.', n);
    end

    if opts.usar_base
        qd = dq_from_pose(robot.base(1:3, 1:3), robot.base(1:3, 4));
    else
        qd = dq_from_pose(eye(3), [0; 0; 0]);
    end

    qd_frames = cell(1, n + 1);
    qd_frames{1} = qd;

    for i = 1:n
        A = dh_matrix(robot.segments(i), q(i));
        qd_i = dq_from_pose(A(1:3, 1:3), A(1:3, 4));
        qd = dq_mul(qd, qd_i);
        qd_frames{i + 1} = qd;
    end

    if opts.usar_tool
        qd_tool = dq_from_pose(robot.tool(1:3, 1:3), robot.tool(1:3, 4));
        qd = dq_mul(qd, qd_tool);
    end

    qd_ee = qd;
    [R, p] = dq_to_pose(qd);
    T_ee = [R, p; 0, 0, 0, 1];
end