function [body_rate_sp, state] = fw_pitch_controller(pitch_sp, euler_yaw_rate_sp, roll, pitch, state, p)
%FW_PITCH_CONTROLLER MATLAB port of PX4 PitchController::control_pitch.
if ~(isfinite(pitch_sp) && isfinite(euler_yaw_rate_sp) && isfinite(pitch) && isfinite(roll))
    body_rate_sp = state.body_rate_sp;
    return;
end

pitch_error = pitch_sp - pitch;
state.euler_rate_sp = pitch_error / p.FW_P_TC;

raw = cos(roll) * state.euler_rate_sp + cos(pitch) * sin(roll) * euler_yaw_rate_sp;
body_rate_sp = constrain(raw, -radians(p.FW_P_RMAX_NEG), radians(p.FW_P_RMAX_POS));
state.body_rate_sp = body_rate_sp;
end
