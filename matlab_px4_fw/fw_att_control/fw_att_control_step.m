function [rates_sp, state] = fw_att_control_step(in, state, p)
%FW_ATT_CONTROL_STEP PX4 fixed-wing attitude control step.
%
% Inputs are uORB-like structs:
%   in.vehicle_attitude.q                  [w x y z]
%   in.vehicle_attitude_setpoint.q_d       [w x y z]
%   in.vehicle_attitude_setpoint.thrust_body(3)
%   in.vehicle_control_mode flags:
%       flag_control_rates_enabled
%       flag_control_attitude_enabled
%       flag_control_manual_enabled
%       flag_control_climb_rate_enabled
%   in.vehicle_status.vehicle_type         2 fixed-wing, 1 rotary-wing
%   in.vehicle_status.in_transition_mode
%   in.vehicle_status.is_vtol_tailsitter
%   in.manual_control_setpoint.yaw         optional stick, [-1, 1]

if nargin < 2 || isempty(state)
    state = fw_att_control_init();
end

rates_sp = state.last_rates_sp;

att_eul = quat_to_euler(in.vehicle_attitude.q(:));
roll = att_eul(1);
pitch = att_eul(2);
yaw = att_eul(3);

vs = in.vehicle_status;
is_in_transition_except_tailsitter = vs.in_transition_mode && ~vs.is_vtol_tailsitter;
is_fixed_wing = vs.vehicle_type == 2;
in_fw_or_transition = is_fixed_wing || is_in_transition_except_tailsitter;

if vs.vehicle_type == 1 && ~vs.is_vtol
    state.last_rates_sp = rates_sp;
    return;
end

rates_sp.thrust_body = in.vehicle_attitude_setpoint.thrust_body(:);

attitude_setpoint = in.vehicle_attitude_setpoint;

if in.vehicle_control_mode.flag_control_manual_enabled && in_fw_or_transition && isfield(in, 'manual_control_setpoint')
    climb_rate_enabled = false;

    if isfield(in.vehicle_control_mode, 'flag_control_climb_rate_enabled')
        climb_rate_enabled = in.vehicle_control_mode.flag_control_climb_rate_enabled;
    end

    if ~climb_rate_enabled && in.vehicle_control_mode.flag_control_attitude_enabled
        roll_body = in.manual_control_setpoint.roll * radians(p.FW_MAN_R_MAX);
        pitch_body = -in.manual_control_setpoint.pitch * radians(p.FW_MAN_P_MAX) + radians(p.FW_PSP_OFF);
        pitch_body = constrain(pitch_body, -radians(p.FW_MAN_P_MAX), radians(p.FW_MAN_P_MAX));

        attitude_setpoint.thrust_body = [(in.manual_control_setpoint.throttle + 1.0) * 0.5; 0.0; 0.0];
        attitude_setpoint.q_d = euler_to_quat([roll_body; pitch_body; yaw]);
        rates_sp.thrust_body = attitude_setpoint.thrust_body(:);
    end
end

if in.vehicle_control_mode.flag_control_rates_enabled
    if in.vehicle_land_detected.landed || ~in_fw_or_transition
        rates_sp.reset_integral = true;
    else
        rates_sp.reset_integral = false;
    end

    if in.vehicle_control_mode.flag_control_attitude_enabled && in_fw_or_transition
        sp_eul = quat_to_euler(attitude_setpoint.q_d(:));
        roll_sp = sp_eul(1);
        pitch_sp = sp_eul(2);

        if all(isfinite(sp_eul))
            [~, state.roll] = fw_roll_controller(roll_sp, state.yaw.euler_rate_sp, roll, pitch, state.roll, p);
            [~, state.pitch] = fw_pitch_controller(pitch_sp, state.yaw.euler_rate_sp, roll, pitch, state.pitch, p);
            airspeed = get_airspeed_constrained_att(in, p);
            [~, state.yaw] = fw_yaw_controller(roll_sp, state.pitch.euler_rate_sp, roll, pitch, airspeed, state.yaw, p);

            body_rates = [
                state.roll.body_rate_sp;
                state.pitch.body_rate_sp;
                state.yaw.body_rate_sp
            ];

            if in.vehicle_control_mode.flag_control_manual_enabled && isfield(in, 'manual_control_setpoint')
                yaw_stick = in.manual_control_setpoint.yaw;
                body_rates(3) = body_rates(3) + constrain(yaw_stick * radians(p.FW_MAN_YR_MAX), ...
                    -radians(p.FW_Y_RMAX), radians(p.FW_Y_RMAX));
            end

            if vs.is_vtol_tailsitter
                body_rates = [body_rates(3); body_rates(2); -body_rates(1)];
            end

            rates_sp.roll = body_rates(1);
            rates_sp.pitch = body_rates(2);
            rates_sp.yaw = body_rates(3);
        end
    end
end

state.last_rates_sp = rates_sp;
end
