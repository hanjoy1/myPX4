function [body_rate_sp, state] = fw_roll_controller(roll_sp, euler_yaw_rate_sp, roll, pitch, state, p)
%FW_ROLL_CONTROLLER MATLAB port of PX4 RollController::control_roll.
if ~(isfinite(roll_sp) && isfinite(euler_yaw_rate_sp) && isfinite(pitch) && isfinite(roll))
    body_rate_sp = state.body_rate_sp;
    return;
end

roll_error = roll_sp - roll;
state.euler_rate_sp = roll_error / p.FW_R_TC;

raw = state.euler_rate_sp - sin(pitch) * euler_yaw_rate_sp;
body_rate_sp = constrain(raw, -radians(p.FW_R_RMAX), radians(p.FW_R_RMAX));
state.body_rate_sp = body_rate_sp;
end
