function robot = fabri_creator()
%FABRI_CREATOR Modelo cinematico del manipulador FABRI Creator (DH, mm y rad).

    deg = pi / 180;
    s(1) = struct('tipo', 'revolute', 'theta0', 0.0,   'd', 65.0, 'a', 15.0,  'alpha', -pi/2, ...
                  'q_min', -85*deg, 'q_max', 85*deg);
    s(2) = struct('tipo', 'revolute', 'theta0', -pi/2, 'd', 0.0,   'a', 120.0, 'alpha', 0.0, ...
                  'q_min', -85*deg, 'q_max', 85*deg);
    s(3) = struct('tipo', 'revolute', 'theta0', pi/2,  'd', 0.0,   'a', 100.0, 'alpha', -pi/2, ...
                  'q_min', -76*deg, 'q_max', 85*deg);
    s(4) = struct('tipo', 'twist',    'theta0', 0.0,   'd', 0.0,   'a', 45.0,  'alpha', pi/2, ...
                  'q_min', -80*deg, 'q_max', 85*deg);
    s(5) = struct('tipo', 'revolute', 'theta0', 0.0,   'd', 0.0,   'a', 0.0,   'alpha', 0.0, ...
                  'q_min', -115*deg, 'q_max', 55*deg);

    base = eye(4);
    base(3, 4) = 57.0;

    tool = eye(4);
    tool(1, 4) = 117.0;

    home_servo       = [90.0, 90.0, 81.0, 95.0, 60.0] * deg;
    servo_offsets    = home_servo;
    servo_directions = [-1, -1, 1, -1, -1];

    robot = struct('segments', s, 'base', base, 'tool', tool, ...
                   'home_servo', home_servo, 'servo_offsets', servo_offsets, ...
                   'servo_directions', servo_directions);
end