function [lat_sp, long_sp, path] = fw_path_frontend_step(vehicle, command, params)
%FW_PATH_FRONTEND_STEP Fixed-wing cruise path-control front end.
%
% This function represents the normal-cruise part of PX4 fw_mode_manager:
%
%   path command -> fixed_wing_lateral_setpoint
%                -> fixed_wing_longitudinal_setpoint
%
% It deliberately excludes takeoff, landing, manual modes, TECS and the
% attitude/rate loops.
%
% Coordinate convention:
%   Local NED horizontal plane: position_ne = [north; east], metres.
%   Course is atan2(east, north), radians.
%
% vehicle fields:
%   position_ne       [2x1] current local position [m]
%   ground_velocity_ne[2x1] current ground velocity [m/s]
%
% command fields:
%   type              uint8: 1 = line, 2 = loiter circle
%   line_start_ne     [2x1] previous waypoint [m]
%   line_end_ne       [2x1] current waypoint [m]
%   line_start_alt    altitude at previous waypoint AMSL [m]
%   line_end_alt      altitude at current waypoint AMSL [m]
%   loiter_center_ne  [2x1] circle centre [m]
%   loiter_radius     positive radius [m]
%   loiter_ccw        true for counter-clockwise, false for clockwise
%   loiter_altitude   target altitude AMSL [m]
%   airspeed          equivalent airspeed setpoint [m/s]
%
% params fields:
%   lookahead_time    path convergence look-ahead time [s]
%   lookahead_min     minimum look-ahead distance [m]
%   lateral_accel_max maximum absolute feed-forward acceleration [m/s^2]
%
% Outputs correspond to the messages consumed by the lateral/longitudinal
% controller:
%   lat_sp.course
%   lat_sp.lateral_acceleration
%   long_sp.altitude
%   long_sp.height_rate
%   long_sp.equivalent_airspeed
%
% This is a compact path front end, not a line-by-line NPFG port. It uses
% closest-path geometry, look-ahead convergence and curvature feed-forward.

MODE_LINE = uint8(1);
MODE_LOITER = uint8(2);

position = vehicle.position_ne(:);
ground_velocity = vehicle.ground_velocity_ne(:);
ground_speed = norm(ground_velocity);
lookahead = max(params.lookahead_min, params.lookahead_time * ground_speed);

path = struct( ...
    'closest_point_ne', zeros(2, 1), ...
    'unit_tangent_ne', zeros(2, 1), ...
    'cross_track_error', 0.0, ...
    'curvature', 0.0);

if command.type == MODE_LINE
    segment = command.line_end_ne(:) - command.line_start_ne(:);
    segment_length = norm(segment);

    if segment_length < 1.0e-6
        error('fw_path_frontend_step:InvalidLine', ...
            'Line start and end points must be different.');
    end

    tangent = segment / segment_length;
    relative_position = position - command.line_start_ne(:);
    along_track = dot(relative_position, tangent);
    closest_point = command.line_start_ne(:) + along_track * tangent;
    curvature = 0.0;

    % Positive error means that the aircraft is to the right of the path
    % in the NED plane.
    path_error = position - closest_point;
    cross_track_error = cross2(tangent, path_error);

    % Interpolate altitude along the finite waypoint segment.
    segment_fraction = min(max(along_track / segment_length, 0.0), 1.0);
    altitude_sp = command.line_start_alt + segment_fraction * ...
        (command.line_end_alt - command.line_start_alt);

elseif command.type == MODE_LOITER
    radius = abs(command.loiter_radius);

    if radius < 1.0e-3
        error('fw_path_frontend_step:InvalidRadius', ...
            'Loiter radius must be greater than zero.');
    end

    center_to_vehicle = position - command.loiter_center_ne(:);
    distance_to_center = norm(center_to_vehicle);

    if distance_to_center > 0.1
        radial = center_to_vehicle / distance_to_center;
    elseif ground_speed > 0.1
        radial = ground_velocity / ground_speed;
    else
        radial = [1.0; 0.0];
    end

    % PX4 local NED convention: +1 is clockwise and -1 is CCW.
    direction = 1.0;
    if command.loiter_ccw
        direction = -1.0;
    end

    tangent = direction * [-radial(2); radial(1)];
    closest_point = command.loiter_center_ne(:) + radius * radial;
    curvature = direction / radius;
    path_error = position - closest_point;
    cross_track_error = distance_to_center - radius;
    altitude_sp = command.loiter_altitude;

else
    error('fw_path_frontend_step:InvalidMode', ...
        'command.type must be uint8(1) for line or uint8(2) for loiter.');
end

% Aim along the path tangent while adding a vector toward the closest path
% point. Far from the path this commands stronger convergence; on the path
% it becomes exactly tangent to the path.
convergence_vector = tangent + (closest_point - position) / lookahead;

if norm(convergence_vector) < 1.0e-6
    convergence_vector = tangent;
end

convergence_direction = convergence_vector / norm(convergence_vector);
course_sp = atan2(convergence_direction(2), convergence_direction(1));

% Curvature feed-forward. For a circle this reduces to a = Vg^2/R with
% direction encoded by the sign. The downstream lateral controller adds
% course/heading-error feedback before mapping acceleration to roll.
lateral_acceleration_ff = ground_speed^2 * curvature;
lateral_acceleration_ff = min(max(lateral_acceleration_ff, ...
    -params.lateral_accel_max), params.lateral_accel_max);

lat_sp = struct( ...
    'course', wrapToPiLocal(course_sp), ...
    'airspeed_direction', NaN, ...
    'lateral_acceleration', lateral_acceleration_ff);

long_sp = struct( ...
    'altitude', altitude_sp, ...
    'height_rate', NaN, ...
    'equivalent_airspeed', command.airspeed, ...
    'pitch_direct', NaN, ...
    'throttle_direct', NaN);

path.closest_point_ne = closest_point;
path.unit_tangent_ne = tangent;
path.cross_track_error = cross_track_error;
path.curvature = curvature;
end

function value = cross2(a, b)
value = a(1) * b(2) - a(2) * b(1);
end

function angle = wrapToPiLocal(angle)
angle = mod(angle + pi, 2.0 * pi) - pi;
end
