function [q, err_final, convergio] = cinematica_inversa(target, q_init, robot, opts)
%CINEMATICA_INVERSA IK de posicion por Levenberg-Marquardt (bombolab-core).
%   [q, err_final, convergio] = cinematica_inversa(target, q_init, robot, opts)
%   opts: max_iter (200), tol (1.0), damping (0.05), step (0.5).

    if nargin < 3 || isempty(robot)
        robot = fabri_creator();
    end
    if nargin < 2 || isempty(q_init)
        q_init = zeros(1, 5);
    end
    if nargin < 4 || isempty(opts)
        opts = struct();
    end
    if ~isfield(opts, 'max_iter'), opts.max_iter = 200; end
    if ~isfield(opts, 'tol'),      opts.tol = 1.0;      end
    if ~isfield(opts, 'damping'),  opts.damping = 0.05; end
    if ~isfield(opts, 'step'),     opts.step = 0.5;     end

    target = target(:);
    q = q_init(:);
    n = numel(robot.segments);
    if numel(q) ~= n
        error('cinematica_inversa:q', 'q_init debe tener %d articulaciones.', n);
    end

    damping_sq = opts.damping^2;
    err_final = inf;
    convergio = false;

    for iter = 1:opts.max_iter
        [T_ee, frames] = cd_dh(q, robot);
        p_ee = T_ee(1:3, 4);
        err = target - p_ee;
        err_final = norm(err);
        if err_final < opts.tol
            convergio = true;
            return;
        end

        Jp = zeros(3, n);
        for i = 1:n
            if i == 1
                prev = robot.base;
                prev_p = robot.base(1:3, 4);
            else
                prev = frames(:, :, i);
                prev_p = prev(1:3, 4);
            end
            if strcmp(robot.segments(i).tipo, 'twist')
                axis = prev(1:3, 1);
                p_i  = frames(1:3, 4, i + 1);
            else
                axis = prev(1:3, 3);
                p_i  = prev_p;
            end
            Jp(:, i) = cross(axis, p_ee - p_i);
        end

        reg = Jp * Jp' + damping_sq * eye(3);
        delta_q = Jp' * (reg \ err);

        dq_norm = norm(delta_q);
        if dq_norm > opts.step
            delta_q = delta_q * (opts.step / dq_norm);
        end

        for i = 1:n
            q(i) = q(i) + delta_q(i);
            q(i) = max(robot.segments(i).q_min, min(robot.segments(i).q_max, q(i)));
        end
    end

    [T_ee] = cd_dh(q, robot);
    err_final = norm(target - T_ee(1:3, 4));
    convergio = false;
end