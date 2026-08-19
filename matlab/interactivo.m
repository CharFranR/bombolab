function interactivo(auto_cerrar)
%INTERACTIVO Ventana interactiva del FABRI Creator (3D, sliders e IK).
%   Uso: interactivo  |  En Octave: octave --gui, luego interactivo.

    if nargin < 1 || isempty(auto_cerrar)
        auto_cerrar = false;
    end

    try
        graphics_toolkit('qt');
    catch
    end

    robot = fabri_creator();
    q = zeros(1, 5);
    deg = 180 / pi;
    is_octave = exist('OCTAVE_VERSION', 'builtin') ~= 0;

    f = figure('Name', 'FABRI Creator - Cinematica interactiva', ...
        'NumberTitle', 'off', 'MenuBar', 'none', ...
        'Position', [80, 80, 1000, 640], 'Color', [0.94, 0.94, 0.94]);

    ax = axes('Parent', f, 'Units', 'normalized', ...
        'Position', [0.05, 0.08, 0.52, 0.86]);

    uicontrol(f, 'Style', 'text', 'Units', 'normalized', ...
        'Position', [0.62, 0.93, 0.34, 0.05], 'String', 'FABRI Creator', ...
        'FontSize', 13, 'FontWeight', 'bold', 'HorizontalAlignment', 'left');

    txt_pose = uicontrol(f, 'Style', 'text', 'Units', 'normalized', ...
        'Position', [0.62, 0.87, 0.34, 0.05], 'String', '', ...
        'FontSize', 10, 'HorizontalAlignment', 'left', 'BackgroundColor', [0.94, 0.94, 0.94]);

    sliders = cell(1, 5);
    txt_q = cell(1, 5);
    nombres = {'q1 (base)', 'q2', 'q3', 'q4 (twist)', 'q5 (muneca)'};
    for i = 1:5
        y = 0.78 - (i - 1) * 0.08;
        uicontrol(f, 'Style', 'text', 'Units', 'normalized', ...
            'Position', [0.62, y, 0.10, 0.04], 'String', nombres{i}, ...
            'FontSize', 9, 'HorizontalAlignment', 'left', 'BackgroundColor', [0.94, 0.94, 0.94]);
        txt_q{i} = uicontrol(f, 'Style', 'text', 'Units', 'normalized', ...
            'Position', [0.72, y, 0.10, 0.04], 'String', '0.0 deg', ...
            'FontSize', 9, 'HorizontalAlignment', 'left', 'BackgroundColor', [0.94, 0.94, 0.94]);
        cb = @(h, ~) on_slider(i, h);
        if is_octave
            sliders{i} = uicontrol(f, 'Style', 'slider', 'Units', 'normalized', ...
                'Position', [0.82, y, 0.14, 0.04], ...
                'Min', robot.segments(i).q_min, 'Max', robot.segments(i).q_max, ...
                'Value', 0.0, 'Callback', cb);
        else
            sliders{i} = uicontrol(f, 'Style', 'slider', 'Units', 'normalized', ...
                'Position', [0.82, y, 0.14, 0.04], ...
                'Min', robot.segments(i).q_min, 'Max', robot.segments(i).q_max, ...
                'Value', 0.0, 'Callback', cb, 'ContinuousValueCallback', cb);
        end
    end

    uicontrol(f, 'Style', 'pushbutton', 'Units', 'normalized', ...
        'Position', [0.62, 0.40, 0.16, 0.05], 'String', 'Home (reposo)', ...
        'FontSize', 10, 'Callback', @(h, ~) on_home());

    uicontrol(f, 'Style', 'text', 'Units', 'normalized', ...
        'Position', [0.62, 0.34, 0.34, 0.04], 'String', 'Objetivo del marcador (mm)', ...
        'FontSize', 10, 'FontWeight', 'bold', 'HorizontalAlignment', 'left', ...
        'BackgroundColor', [0.94, 0.94, 0.94]);

    campos = struct();
    etiquetas = {'X', 'Y', 'Z'};
    for i = 1:3
        y = 0.30 - (i - 1) * 0.055;
        uicontrol(f, 'Style', 'text', 'Units', 'normalized', ...
            'Position', [0.62, y, 0.04, 0.04], 'String', etiquetas{i}, ...
            'FontSize', 10, 'HorizontalAlignment', 'center', 'BackgroundColor', [0.94, 0.94, 0.94]);
        campos.(etiquetas{i}) = uicontrol(f, 'Style', 'edit', 'Units', 'normalized', ...
            'Position', [0.67, y, 0.12, 0.04], 'String', '277', ...
            'FontSize', 10, 'HorizontalAlignment', 'center');
    end

    uicontrol(f, 'Style', 'pushbutton', 'Units', 'normalized', ...
        'Position', [0.81, 0.30, 0.15, 0.05], 'String', 'Resolver IK', ...
        'FontSize', 10, 'FontWeight', 'bold', 'Callback', @(h, ~) on_ik());

    txt_ik = uicontrol(f, 'Style', 'text', 'Units', 'normalized', ...
        'Position', [0.62, 0.10, 0.34, 0.10], 'String', '', ...
        'FontSize', 9, 'HorizontalAlignment', 'left', 'BackgroundColor', [0.94, 0.94, 0.94]);

    actualizar_vista();

    if auto_cerrar
        pause(2);
        delete(f);
    end

    function on_slider(i, h)
        q(i) = get(h, 'Value');
        set(txt_q{i}, 'String', sprintf('%.1f deg', q(i) * deg));
        actualizar_vista();
    end

    function on_home()
        q(:) = 0;
        for i = 1:5
            set(sliders{i}, 'Value', 0.0);
            set(txt_q{i}, 'String', '0.0 deg');
        end
        actualizar_vista();
    end

    function on_ik()
        try
            target = [str2double(get(campos.X, 'String')), ...
                      str2double(get(campos.Y, 'String')), ...
                      str2double(get(campos.Z, 'String'))];
        catch
            set(txt_ik, 'String', 'Ingrese valores numericos en X, Y, Z.');
            return;
        end
        if any(isnan(target))
            set(txt_ik, 'String', 'Ingrese valores numericos en X, Y, Z.');
            return;
        end

        [q_sol, err, ok] = cinematica_inversa(target, q, robot);
        q = q_sol(:)';

        for i = 1:5
            set(sliders{i}, 'Value', q(i));
            set(txt_q{i}, 'String', sprintf('%.1f deg', q(i) * deg));
        end

        if ok
            msg = sprintf('IK OK - error %.2f mm\nq = [%s]', err, ...
                sprintf('%.1f ', q * deg));
        else
            msg = sprintf('Sin convergencia (error %.2f mm)\nEl punto puede estar fuera del alcance.\nq = [%s]', ...
                err, sprintf('%.1f ', q * deg));
        end
        set(txt_ik, 'String', msg);

        actualizar_vista(target);
    end

    function actualizar_vista(target)
        if nargin < 1 || isempty(target)
            target = [];
        end
        dibujar_robot(ax, robot, q, target);
        [T_ee] = cd_dh(q, robot);
        p = T_ee(1:3, 4);
        set(txt_pose, 'String', sprintf('Marcador: (%.1f, %.1f, %.1f) mm', p(1), p(2), p(3)));
        try
            drawnow('limitrate');
        catch
            drawnow;
        end
    end
end