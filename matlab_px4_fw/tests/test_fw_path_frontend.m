function test_fw_path_frontend()
%TEST_FW_PATH_FRONTEND Basic line and circle geometry checks.
p = fw_path_frontend_defaults();
vehicle = struct('position_ne', [50; 20], ...
    'ground_velocity_ne', [20; 0]);

line_command = emptyCommand();
line_command.type = uint8(1);
line_command.line_start_ne = [0; 0];
line_command.line_end_ne = [100; 0];
line_command.line_start_alt = 100;
line_command.line_end_alt = 200;
line_command.airspeed = 20;

[lat_sp, long_sp, path] = fw_path_frontend_step(vehicle, line_command, p);
assert(lat_sp.course < 0, 'Aircraft east of a northbound path must steer west.');
assert(abs(lat_sp.lateral_acceleration) < 1e-9);
assert(abs(long_sp.altitude - 150) < 1e-9);
assert(norm(path.closest_point_ne - [50; 0]) < 1e-9);

loiter_command = emptyCommand();
loiter_command.type = uint8(2);
loiter_command.loiter_center_ne = [0; 0];
loiter_command.loiter_radius = 100;
loiter_command.loiter_ccw = false;
loiter_command.loiter_altitude = 300;
loiter_command.airspeed = 20;
vehicle.position_ne = [100; 0];

[lat_sp, long_sp, path] = fw_path_frontend_step(vehicle, loiter_command, p);
assert(abs(lat_sp.course - pi / 2) < 1e-9);
assert(abs(lat_sp.lateral_acceleration - 4) < 1e-9);
assert(abs(long_sp.altitude - 300) < 1e-9);
assert(abs(path.curvature - 0.01) < 1e-9);

route = struct( ...
    'waypoints_ne', [0, 100, 100; 0, 0, 100], ...
    'altitudes', [100, 150, 200], ...
    'airspeeds', 20, ...
    'acceptance_radius', 10);
route_state = fw_waypoint_route_init();

vehicle.position_ne = [50; 10];
[~, long_sp, ~, route_state, status] = ...
    fw_waypoint_route_step(vehicle, route, route_state, p);
assert(status.target_waypoint_index == 2);
assert(abs(long_sp.altitude - 125) < 1e-9);

vehicle.position_ne = [95; 0];
[lat_sp, ~, ~, route_state, status] = ...
    fw_waypoint_route_step(vehicle, route, route_state, p);
assert(status.target_waypoint_index == 3);
assert(lat_sp.course > 0);

vehicle.position_ne = [100; 95];
[~, ~, ~, route_state, status] = ...
    fw_waypoint_route_step(vehicle, route, route_state, p);
assert(route_state.finished);
assert(status.finished);

disp('test_fw_path_frontend: PASS');
end

function command = emptyCommand()
command = struct( ...
    'type', uint8(0), ...
    'line_start_ne', zeros(2, 1), ...
    'line_end_ne', zeros(2, 1), ...
    'line_start_alt', 0.0, ...
    'line_end_alt', 0.0, ...
    'loiter_center_ne', zeros(2, 1), ...
    'loiter_radius', 1.0, ...
    'loiter_ccw', false, ...
    'loiter_altitude', 0.0, ...
    'airspeed', 0.0);
end
