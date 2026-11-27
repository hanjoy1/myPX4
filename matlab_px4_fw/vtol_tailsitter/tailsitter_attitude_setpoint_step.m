function [vtol_sp, state, status] = tailsitter_attitude_setpoint_step( ...
    mc_sp, fw_sp, control, state, params)
%TAILSITTER_ATTITUDE_SETPOINT_STEP Select/generate tailsitter VTOL setpoint.
%
% This is a compact MATLAB front end for the attitude/thrust-selection part
% of PX4 Tailsitter. It does not include the state switching conditions,
% attitude controllers, rate controllers or actuator allocation.
%
% Inputs
% ------
% mc_sp.q_d          [4x1] MC desired quaternion [w x y z]
% mc_sp.thrust_body  [3x1] MC desired thrust; normally z < 0
% fw_sp.q_d          [4x1] FW desired quaternion [w x y z]
% fw_sp.thrust_body  [3x1] FW desired thrust; normally x > 0
%
% control.mode:
%   uint8(0) = MC
%   uint8(1) = front transition, MC -> FW
%   uint8(2) = FW
%   uint8(3) = back transition, FW -> MC
% control.dt          control period [s]
%
% state must be retained between calls and initialized with
% tailsitter_attitude_setpoint_init().
%
% Output
% ------
% vtol_sp.q_d         unified desired quaternion
% vtol_sp.thrust_body unified desired thrust vector
%
% Coordinate convention
% ---------------------
% During MC and transitions, q_d uses the MC/tailsitter hover frame.
% During FW mode, q_d uses the normal fixed-wing frame, matching PX4.
% The two neutral frames differ by approximately -90 deg around body Y.

MODE_MC = uint8(0);
MODE_FRONT = uint8(1);
MODE_FW = uint8(2);
MODE_BACK = uint8(3);

mode = uint8(control.mode);
dt = min(max(control.dt, 0.0), 0.1);

q_mc = normalizeQuat(mc_sp.q_d(:));
q_fw = normalizeQuat(fw_sp.q_d(:));
mc_thrust = mc_sp.thrust_body(:);
fw_thrust = fw_sp.thrust_body(:);

% A normal FW attitude expressed in the tailsitter/MC hover frame.
% This is the same -90 deg body-Y relationship used in PX4 backtransition.
q_fw_in_mc_frame = quatMultiply(q_fw, euler_to_quat([0; -pi / 2; 0]));

entering_new_mode = mode ~= state.previous_mode;

if entering_new_mode
    state.transition_time = 0.0;

    if mode == MODE_FRONT
        % Begin from the latest MC attitude and rotate toward the FW
        % attitude represented in the MC frame.
        state.transition_start_q = q_mc;
        state.transition_target_q = q_fw_in_mc_frame;

    elseif mode == MODE_BACK
        % Begin from the FW attitude represented in the MC frame and rotate
        % back toward the current MC hover target.
        state.transition_start_q = q_fw_in_mc_frame;
        state.transition_target_q = q_mc;
    end
end

state.transition_time = state.transition_time + dt;

if mode == MODE_MC
    q_out = q_mc;
    thrust_out = mc_thrust;

elseif mode == MODE_FRONT
    progress = clamp01(state.transition_time / ...
        max(params.front_transition_duration, 0.1));

    % Update the endpoint so navigation/yaw changes can still be followed
    % while the front transition is progressing.
    state.transition_target_q = q_fw_in_mc_frame;
    q_out = quatSlerp(state.transition_start_q, ...
        state.transition_target_q, progress);

    % PX4 tailsitter transition attitude is executed by the MC controller,
    % therefore the vertical MC thrust remains the active thrust channel.
    thrust_out = [0; 0; mc_thrust(3)];

elseif mode == MODE_FW
    q_out = q_fw;
    thrust_out = fw_thrust;

    % Immediately after front transition, blend from the equivalent last
    % MC thrust magnitude to the new FW +X thrust.
    if state.previous_mode == MODE_FRONT
        state.transition_time = 0.0;
    end

    blend = clamp01(state.transition_time / ...
        max(params.front_thrust_blend_duration, 0.01));
    thrust_out(1) = blend * fw_thrust(1) + ...
        (1.0 - blend) * (-state.last_mc_thrust_z);

elseif mode == MODE_BACK
    progress = clamp01(state.transition_time / ...
        max(params.back_transition_duration, 0.1));

    % The hover endpoint may move slowly with the MC position/yaw target.
    state.transition_target_q = q_mc;
    q_out = quatSlerp(state.transition_start_q, ...
        state.transition_target_q, progress);

    thrust_blend = clamp01(state.transition_time / ...
        max(params.back_thrust_blend_duration, 0.01));

    % Convert the last positive FW +X thrust to the negative MC -Z
    % convention, then blend toward the current MC vertical thrust.
    thrust_z = thrust_blend * mc_thrust(3) + ...
        (1.0 - thrust_blend) * (-state.last_fw_thrust_x);
    thrust_out = [0; 0; thrust_z];

else
    error('tailsitter_attitude_setpoint_step:InvalidMode', ...
        'control.mode must be uint8 0, 1, 2 or 3.');
end

% Save the latest valid inputs for thrust continuity at the next switch.
state.last_mc_thrust_z = mc_thrust(3);
state.last_fw_thrust_x = fw_thrust(1);
state.previous_mode = mode;

vtol_sp = struct( ...
    'q_d', normalizeQuat(q_out), ...
    'thrust_body', thrust_out);

status = struct( ...
    'mode', mode, ...
    'transition_progress', transitionProgress(mode, state.transition_time, params), ...
    'in_transition', mode == MODE_FRONT || mode == MODE_BACK);
end

function progress = transitionProgress(mode, transition_time, params)
if mode == uint8(1)
    progress = clamp01(transition_time / max(params.front_transition_duration, 0.1));
elseif mode == uint8(3)
    progress = clamp01(transition_time / max(params.back_transition_duration, 0.1));
else
    progress = 0.0;
end
end

function value = clamp01(value)
value = min(max(value, 0.0), 1.0);
end

function q = quatMultiply(a, b)
% Hamilton product for PX4 quaternions [w x y z].
q = [
    a(1) * b(1) - a(2) * b(2) - a(3) * b(3) - a(4) * b(4);
    a(1) * b(2) + a(2) * b(1) + a(3) * b(4) - a(4) * b(3);
    a(1) * b(3) - a(2) * b(4) + a(3) * b(1) + a(4) * b(2);
    a(1) * b(4) + a(2) * b(3) - a(3) * b(2) + a(4) * b(1)
];
q = normalizeQuat(q);
end

function q = quatSlerp(q0, q1, fraction)
% Shortest-path spherical linear interpolation.
q0 = normalizeQuat(q0);
q1 = normalizeQuat(q1);
dot_product = dot(q0, q1);

if dot_product < 0.0
    q1 = -q1;
    dot_product = -dot_product;
end

dot_product = min(max(dot_product, -1.0), 1.0);

if dot_product > 0.9995
    q = normalizeQuat(q0 + fraction * (q1 - q0));
    return;
end

angle = acos(dot_product);
q = (sin((1.0 - fraction) * angle) * q0 + ...
    sin(fraction * angle) * q1) / sin(angle);
q = normalizeQuat(q);
end

function q = normalizeQuat(q)
n = norm(q);
if n < 1.0e-9
    q = [1; 0; 0; 0];
else
    q = q / n;
end
end
