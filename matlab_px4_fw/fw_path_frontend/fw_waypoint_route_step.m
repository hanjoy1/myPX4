function [lat_sp, long_sp, path, state, status] = ...
    fw_waypoint_route_step(vehicle, route, state, params)
%FW_WAYPOINT_ROUTE_STEP Track a sequence of fixed-wing waypoints.
%
% This is the route-management layer above fw_path_frontend_step. At each
% call it selects one line segment:
%
%   waypoint(:, i) -> waypoint(:, i + 1)
%
% and advances to the following segment when the aircraft either enters the
% waypoint acceptance radius or passes the switching plane through the
% waypoint. The switching plane is perpendicular to the current segment.
%
% route fields:
%   waypoints_ne      [2xN] local NED horizontal waypoints [m]
%   altitudes         [1xN] waypoint altitudes AMSL [m]
%   airspeeds         scalar or [1xN] equivalent airspeed setpoints [m/s]
%   acceptance_radius positive waypoint switching radius [m]
%
% state is initialized with fw_waypoint_route_init().

waypoint_count = size(route.waypoints_ne, 2);

if waypoint_count < 2
    error('fw_waypoint_route_step:TooFewWaypoints', ...
        'At least two waypoints are required.');
end

if numel(route.altitudes) ~= waypoint_count
    error('fw_waypoint_route_step:InvalidAltitudes', ...
        'route.altitudes must contain one altitude per waypoint.');
end

if ~(isscalar(route.airspeeds) || numel(route.airspeeds) == waypoint_count)
    error('fw_waypoint_route_step:InvalidAirspeeds', ...
        'route.airspeeds must be scalar or contain one value per waypoint.');
end

last_segment = waypoint_count - 1;
segment_index = min(max(double(state.segment_index), 1), last_segment);
position = vehicle.position_ne(:);

% More than one waypoint may need to be skipped if the vehicle or route is
% updated discontinuously. The bounded loop is also suitable for simulation
% with a relatively large sample time.
while segment_index < last_segment
    segment_start = route.waypoints_ne(:, segment_index);
    segment_end = route.waypoints_ne(:, segment_index + 1);
    segment = segment_end - segment_start;
    segment_length = norm(segment);

    if segment_length < 1.0e-6
        % Skip duplicate consecutive waypoints.
        segment_index = segment_index + 1;
        continue;
    end

    tangent = segment / segment_length;
    distance_to_end = norm(position - segment_end);
    passed_switching_plane = dot(position - segment_end, tangent) >= 0.0;
    inside_acceptance_radius = ...
        distance_to_end <= route.acceptance_radius;

    if inside_acceptance_radius || passed_switching_plane
        segment_index = segment_index + 1;
    else
        break;
    end
end

state.segment_index = uint16(segment_index);

segment_start = route.waypoints_ne(:, segment_index);
segment_end = route.waypoints_ne(:, segment_index + 1);

command = emptyLineCommand();
command.line_start_ne = segment_start;
command.line_end_ne = segment_end;
command.line_start_alt = route.altitudes(segment_index);
command.line_end_alt = route.altitudes(segment_index + 1);

if isscalar(route.airspeeds)
    command.airspeed = route.airspeeds;
else
    command.airspeed = route.airspeeds(segment_index + 1);
end

[lat_sp, long_sp, path] = ...
    fw_path_frontend_step(vehicle, command, params);

distance_to_final = norm(position - route.waypoints_ne(:, end));
state.finished = segment_index == last_segment && ...
    distance_to_final <= route.acceptance_radius;

status = struct( ...
    'segment_index', uint16(segment_index), ...
    'start_waypoint_index', uint16(segment_index), ...
    'target_waypoint_index', uint16(segment_index + 1), ...
    'distance_to_target', norm(position - segment_end), ...
    'distance_to_final', distance_to_final, ...
    'finished', state.finished);
end

function command = emptyLineCommand()
% Keep a fixed command bus layout for MATLAB Function/Simulink use.
command = struct( ...
    'type', uint8(1), ...
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
