function state = tailsitter_attitude_setpoint_init(mc_sp)
%TAILSITTER_ATTITUDE_SETPOINT_INIT Initialize tailsitter setpoint state.
%
% mc_sp fields:
%   q_d         [4x1] PX4 quaternion [w x y z]
%   thrust_body [3x1] MC thrust, normally thrust_body(3) < 0

state = struct( ...
    'previous_mode', uint8(0), ...
    'transition_time', 0.0, ...
    'transition_start_q', normalizeQuat(mc_sp.q_d(:)), ...
    'transition_target_q', normalizeQuat(mc_sp.q_d(:)), ...
    'last_mc_thrust_z', mc_sp.thrust_body(3), ...
    'last_fw_thrust_x', 0.0);
end

function q = normalizeQuat(q)
n = norm(q);
if n < 1.0e-9
    q = [1; 0; 0; 0];
else
    q = q / n;
end
end
