function h = dibujar_robot(ax, robot, q, target)
%DIBUJAR_ROBOT Dibuja el FABRI Creator en 3D con limites fijos.
%   La primera llamada crea la escena; las siguientes solo actualizan datos.

    if nargin < 4 || isempty(target)
        target = [];
    end

    scene = getappdata(ax, 'fabri_scene');

    if isempty(scene)
        R = 0;
        for i = 1:numel(robot.segments)
            R = R + abs(robot.segments(i).a);
        end
        R = R + abs(robot.tool(1, 4));

        cla(ax);
        hold(ax, 'on');

        plot3(ax, [0, 0], [0, 0], [0, robot.base(3, 4)], 'k-', 'LineWidth', 5);

        skel = plot3(ax, NaN, NaN, NaN, ...
            'b-o', 'LineWidth', 4, 'MarkerSize', 9, 'MarkerFaceColor', 'b');
        tool = plot3(ax, NaN, NaN, NaN, 'r-', 'LineWidth', 4);

        ee = scatter3(ax, 0, 0, 0, 60, 'g', 'filled');
        tg = scatter3(ax, 0, 0, 0, 120, 'r', 'filled', 'Visible', 'off');

        quiver3(ax, 0, 0, 0, 50, 0, 0, 'r', 'LineWidth', 1.5);
        quiver3(ax, 0, 0, 0, 0, 50, 0, 'g', 'LineWidth', 1.5);
        quiver3(ax, 0, 0, 0, 0, 0, 50, 'b', 'LineWidth', 1.5);

        hold(ax, 'off');

        axis(ax, 'equal');
        xlim(ax, [-R, R]);
        ylim(ax, [-R, R]);
        zlim(ax, [-40, R + 60]);
        grid(ax, 'on');
        view(ax, 3);
        xlabel(ax, 'X [mm]');
        ylabel(ax, 'Y [mm]');
        zlabel(ax, 'Z [mm]');

        scene = struct('skel', skel, 'tool', tool, 'ee', ee, 'tg', tg);
        setappdata(ax, 'fabri_scene', scene);
    end

    [T_ee, frames] = cd_dh(q, robot);

    n = size(frames, 3);
    pts = zeros(n, 3);
    for i = 1:n
        pts(i, :) = frames(1:3, 4, i)';
    end
    p_ee = T_ee(1:3, 4)';

    set(scene.skel, 'XData', pts(:, 1), 'YData', pts(:, 2), 'ZData', pts(:, 3));
    set(scene.tool, 'XData', [pts(end, 1), p_ee(1)], ...
        'YData', [pts(end, 2), p_ee(2)], 'ZData', [pts(end, 3), p_ee(3)]);
    set(scene.ee, 'XData', p_ee(1), 'YData', p_ee(2), 'ZData', p_ee(3));

    if ~isempty(target)
        set(scene.tg, 'XData', target(1), 'YData', target(2), 'ZData', target(3), ...
            'Visible', 'on');
    else
        set(scene.tg, 'Visible', 'off');
    end
end