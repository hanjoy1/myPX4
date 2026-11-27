function [body_rate_sp, state] = fw_yaw_controller(roll_sp, euler_pitch_rate_sp, roll, pitch, airspeed, state, p)
%FW_YAW_CONTROLLER MATLAB port of PX4 YawController::control_yaw.
if ~(isfinite(roll_sp) && isfinite(roll) && isfinite(pitch) && isfinite(euler_pitch_rate_sp) && isfinite(airspeed))
    body_rate_sp = state.body_rate_sp;
    return;
end

inverted = false;

if abs(roll) < radians(90.0)
    constrained_roll = constrain(roll, radians(-80.0), radians(80.0));
else
    inverted = true;

    if roll > 0.0
        constrained_roll = constrain(roll, radians(100.0), radians(180.0));
    else
        constrained_roll = constrain(roll, radians(-180.0), radians(-100.0));
    end
end

constrained_roll = constrain(constrained_roll, -abs(roll_sp), abs(roll_sp));

if ~inverted
    state.euler_rate_sp = tan(constrained_roll) * cos(pitch) * p.CONSTANTS_ONE_G / airspeed;
    raw = -sin(roll) * euler_pitch_rate_sp + cos(roll) * cos(pitch) * state.euler_rate_sp;
    state.body_rate_sp = constrain(raw, -radians(p.FW_Y_RMAX), radians(p.FW_Y_RMAX));
end

if ~isfinite(state.body_rate_sp)
    state.body_rate_sp = 0.0;
end

body_rate_sp = state.body_rate_sp;
end
