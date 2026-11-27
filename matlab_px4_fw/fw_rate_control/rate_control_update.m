function [torque, state] = rate_control_update(rate, rate_sp, angular_accel, dt, landed, state, gains, sat_pos, sat_neg)
%RATE_CONTROL_UPDATE MATLAB port of PX4 RateControl::update.
rate_error = rate_sp - rate;
torque = gains.P(:) .* rate_error + state.rate_int - gains.D(:) .* angular_accel + gains.FF(:) .* rate_sp;

if ~landed
    [state.rate_int, ~] = rate_control_update_integral(state.rate_int, rate_error, dt, gains, sat_pos, sat_neg);
end
end
