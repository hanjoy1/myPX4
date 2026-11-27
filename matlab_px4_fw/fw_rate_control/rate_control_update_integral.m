function [rate_int, rate_error] = rate_control_update_integral(rate_int, rate_error, dt, gains, sat_pos, sat_neg)
%RATE_CONTROL_UPDATE_INTEGRAL MATLAB port of PX4 RateControl::updateIntegral.
for i = 1:3
    if sat_pos(i)
        rate_error(i) = min(rate_error(i), 0.0);
    end

    if sat_neg(i)
        rate_error(i) = max(rate_error(i), 0.0);
    end

    i_factor = rate_error(i) / radians(400.0);
    i_factor = max(0.0, 1.0 - i_factor * i_factor);

    rate_i = rate_int(i) + i_factor * gains.I(i) * rate_error(i) * dt;

    if isfinite(rate_i)
        rate_int(i) = constrain(rate_i, -gains.Imax(i), gains.Imax(i));
    end
end
end
