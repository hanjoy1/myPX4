function [q_d, thrust_body, transition_progress, in_transition] = ...
    tailsitter_setpoint_simulink(mc_q_d, mc_thrust_z, ...
    fw_q_d, fw_thrust_x, mode, dt, reset, transition_params)
%TAILSITTER_SETPOINT_SIMULINK Numeric-only interface for Simulink.
%
% This function is intended to be copied into or called from a MATLAB
% Function block. All external ports are fixed-size numeric signals; no
% structs or buses are required.
%
% Inputs
% ------
% mc_q_d              [4x1] MC quaternion [w x y z]
% mc_thrust_z         scalar MC thrust, normally negative
% fw_q_d              [4x1] FW quaternion [w x y z]
% fw_thrust_x         scalar FW thrust, normally positive
% mode                uint8 scalar:
%                       0 = MC
%                       1 = front transition
%                       2 = FW
%                       3 = back transition
% dt                  scalar sample time [s]
% reset               boolean; true resets internal transition memory
% transition_params   [4x1]:
%                       (1) front transition duration [s]
%                       (2) back transition duration [s]
%                       (3) MC-to-FW thrust blend duration [s]
%                       (4) FW-to-MC thrust blend duration [s]
%
% Outputs
% -------
% q_d                 [4x1] selected/generated quaternion [w x y z]
% thrust_body         [3x1] PX4 body thrust vector
% transition_progress scalar in [0,1]
% in_transition       boolean
%
% Persistent state means the block must be called at a fixed sample time.
% Pulse reset=true during model initialization or whenever the flight
% controller state is reset.

MODE_MC = uint8(0);
MODE_FRONT = uint8(1);
MODE_FW = uint8(2);
MODE_BACK = uint8(3);

persistent initialized
persistent previous_mode
persistent transition_time
persistent transition_start_q
persistent transition_target_q
persistent last_mc_thrust_z
persistent last_fw_thrust_x

mc_q = normalizeQuat(mc_q_d(:));
fw_q = normalizeQuat(fw_q_d(:));
mode_u8 = uint8(mode);
dt_limited = min(max(dt, 0.0), 0.1);

if isempty(initialized)
    initialized = false;
end

if ~initialized || reset
    previous_mode = MODE_MC;
    transition_time = 0.0;
    transition_start_q = mc_q;
    transition_target_q = mc_q;
    last_mc_thrust_z = mc_thrust_z;
    last_fw_thrust_x = fw_thrust_x;
    initialized = true;
end

front_duration = max(transition_params(1), 0.1);
back_duration = max(transition_params(2), 0.1);
front_blend_duration = max(transition_params(3), 0.01);
back_blend_duration = max(transition_params(4), 0.01);

% Fixed-wing zero attitude represented in the tailsitter MC/hover frame.
q_minus_90_pitch = [cos(pi / 4); 0; -sin(pi / 4); 0];
fw_q_in_mc_frame = quatMultiply(fw_q, q_minus_90_pitch);

entering_new_mode = mode_u8 ~= previous_mode;

if entering_new_mode
    transition_time = 0.0;

    if mode_u8 == MODE_FRONT
        transition_start_q = mc_q;
        transition_target_q = fw_q_in_mc_frame;

    elseif mode_u8 == MODE_BACK
        transition_start_q = fw_q_in_mc_frame;
        transition_target_q = mc_q;
    end
end

transition_time = transition_time + dt_limited;
transition_progress = 0.0;
in_transition = false;

if mode_u8 == MODE_MC
    q_d = mc_q;
    thrust_body = [0; 0; mc_thrust_z];

elseif mode_u8 == MODE_FRONT
    in_transition = true;
    transition_progress = clamp01(transition_time / front_duration);
    transition_target_q = fw_q_in_mc_frame;
    q_d = quatSlerp(transition_start_q, transition_target_q, ...
        transition_progress);

    % The tailsitter transition attitude is executed by the MC controller.
    thrust_body = [0; 0; mc_thrust_z];

elseif mode_u8 == MODE_FW
    q_d = fw_q;

    % Start the takeover timer at exactly zero on the first FW sample.
    if previous_mode == MODE_FRONT
        transition_time = 0.0;
    end

    thrust_blend = clamp01(transition_time / front_blend_duration);
    blended_fw_thrust = thrust_blend * fw_thrust_x + ...
        (1.0 - thrust_blend) * (-last_mc_thrust_z);
    thrust_body = [blended_fw_thrust; 0; 0];

elseif mode_u8 == MODE_BACK
    in_transition = true;
    transition_progress = clamp01(transition_time / back_duration);
    transition_target_q = mc_q;
    q_d = quatSlerp(transition_start_q, transition_target_q, ...
        transition_progress);

    thrust_blend = clamp01(transition_time / back_blend_duration);
    blended_mc_thrust = thrust_blend * mc_thrust_z + ...
        (1.0 - thrust_blend) * (-last_fw_thrust_x);
    thrust_body = [0; 0; blended_mc_thrust];

else
    % Deterministic safe fallback for code generation: use the MC demand.
    q_d = mc_q;
    thrust_body = [0; 0; mc_thrust_z];
end

q_d = normalizeQuat(q_d);
last_mc_thrust_z = mc_thrust_z;
last_fw_thrust_x = fw_thrust_x;
previous_mode = mode_u8;
end

function value = clamp01(value)
value = min(max(value, 0.0), 1.0);
end

function q = quatMultiply(a, b)
q = [
    a(1) * b(1) - a(2) * b(2) - a(3) * b(3) - a(4) * b(4);
    a(1) * b(2) + a(2) * b(1) + a(3) * b(4) - a(4) * b(3);
    a(1) * b(3) - a(2) * b(4) + a(3) * b(1) + a(4) * b(2);
    a(1) * b(4) + a(2) * b(3) - a(3) * b(2) + a(4) * b(1)
];
q = normalizeQuat(q);
end

function q = quatSlerp(q0, q1, fraction)
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
