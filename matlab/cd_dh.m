function [T_ee, frames] = cd_dh(q, robot, opts)
%CD_DH Cinematica directa del FABRI Creator por parametros DH.
%   T_ee = base * A1 * ... * A5 * tool; frames(:,:,k) = marco F_{k-1}.

    if nargin < 3 || isempty(opts)
        opts = struct();
    end
    if ~isfield(opts, 'usar_base'), opts.usar_base = true; end
    if ~isfield(opts, 'usar_tool'), opts.usar_tool = true; end

    n = numel(robot.segments);
    if numel(q) ~= n
        error('cd_dh:q', 'q debe tener %d articulaciones.', n);
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
end