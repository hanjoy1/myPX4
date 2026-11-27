function [torque_sp, thrust_sp, state, status] = fw_rate_control_step(in, state, p)
%FW_RATE_CONTROL_STEP PX4 fixed-wing rate control step.
%
% Inputs are uORB-like structs:
%   in.vehicle_angular_velocity.xyz        [p q r] rad/s
%   in.vehicle_angular_velocity.xyz_derivative optional angular accel rad/s^2
%   in.vehicle_rates_setpoint.roll/pitch/yaw
%   in.vehicle_rates_setpoint.thrust_body(3)
%   in.vehicle_control_mode flags:
%       flag_control_rates_enabled
%       flag_control_attitude_enabled
%       flag_control_manual_enabled
%   in.vehicle_land_detected.landed
%   in.airspeed_validated.calibrated_airspeed_m_s
% Optional:
%   in.saturation_positive(3), in.saturation_negative(3)
%   in.battery_status.connected, scale

if nargin < 2 || isempty(state)
    state = fw_rate_control_init();
end

torque_sp = state.last_torque_sp;
thrust_sp = state.last_thrust_sp;

rate = in.vehicle_angular_velocity.xyz(:);

if isfield(in.vehicle_angular_velocity, 'xyz_derivative')
    angular_accel = in.vehicle_angular_velocity.xyz_derivative(:);
else
    angular_accel = [0.0; 0.0; 0.0];
end

[airspeed, state] = get_airspeed_and_update_scaling(in, state, p);
trim = fw_trim_schedule(airspeed, state.airspeed_scaling, p);
rates_sp = in.vehicle_rates_setpoint;

vs = in.vehicle_status;
is_in_transition_except_tailsitter = vs.in_transition_mode && ~vs.is_vtol_tailsitter;
is_fixed_wing = vs.vehicle_type == 2;
in_fw_or_transition = is_fixed_wing || is_in_transition_except_tailsitter;

if in.vehicle_control_mode.flag_control_manual_enabled && in_fw_or_transition && isfield(in, 'manual_control_setpoint')
    if in.vehicle_control_mode.flag_control_rates_enabled && ~in.vehicle_control_mode.flag_control_attitude_enabled
        if vs.is_vtol_tailsitter && vs.vehicle_type == 2
            rates_sp.roll = in.manual_control_setpoint.yaw * radians(p.FW_ACRO_Z_MAX);
            rates_sp.yaw = -in.manual_control_setpoint.roll * radians(p.FW_ACRO_X_MAX);
        else
            rates_sp.roll = in.manual_control_setpoint.roll * radians(p.FW_ACRO_X_MAX);
            rates_sp.yaw = in.manual_control_setpoint.yaw * radians(p.FW_ACRO_Z_MAX);
        end

        rates_sp.pitch = -in.manual_control_setpoint.pitch * radians(p.FW_ACRO_Y_MAX);
        rates_sp.thrust_body = [(in.manual_control_setpoint.throttle + 1.0) * 0.5; 0.0; 0.0];
    elseif ~in.vehicle_control_mode.flag_control_rates_enabled
        airspeed_scaling_sq = state.airspeed_scaling * state.airspeed_scaling;
        torque_sp.xyz(1) = constrain(in.manual_control_setpoint.roll * p.FW_MAN_R_SC + p.TRIM_ROLL * airspeed_scaling_sq, -1.0, 1.0);
        torque_sp.xyz(2) = constrain(-in.manual_control_setpoint.pitch * p.FW_MAN_P_SC + p.TRIM_PITCH * airspeed_scaling_sq, -1.0, 1.0);
        torque_sp.xyz(3) = constrain(in.manual_control_setpoint.yaw * p.FW_MAN_Y_SC + p.TRIM_YAW * airspeed_scaling_sq, -1.0, 1.0);
        thrust_sp.xyz(1) = constrain((in.manual_control_setpoint.throttle + 1.0) * 0.5, 0.0, 1.0);
    end
end

sat_pos = false(3, 1);
sat_neg = false(3, 1);

if isfield(in, 'saturation_positive')
    sat_pos = logical(in.saturation_positive(:));
end

if isfield(in, 'saturation_negative')
    sat_neg = logical(in.saturation_negative(:));
end

if rates_sp.reset_integral
    state.rate_int(:) = 0.0;
end

if in.vehicle_control_mode.flag_control_rates_enabled
    rate_sp = [
        rates_sp.roll;
        rates_sp.pitch;
        rates_sp.yaw
    ];

    if in.vehicle_status.is_vtol_tailsitter
        rate_sp = [-rates_sp.yaw; rates_sp.pitch; rates_sp.roll];
    end

    gains.P = [p.FW_RR_P; p.FW_PR_P; p.FW_YR_P];
    gains.I = [p.FW_RR_I; p.FW_PR_I; p.FW_YR_I];
    gains.D = [p.FW_RR_D; p.FW_PR_D; p.FW_YR_D];
    gains.Imax = [p.FW_RR_IMAX; p.FW_PR_IMAX; p.FW_YR_IMAX];
    gains.FF = [p.FW_RR_FF; p.FW_PR_FF; p.FW_YR_FF] ./ state.airspeed_scaling;

    [control_u, state] = rate_control_update(rate, rate_sp, angular_accel, in.dt, ...
        in.vehicle_land_detected.landed, state, gains, sat_pos, sat_neg);

    control_u = control_u .* state.airspeed_scaling .* state.airspeed_scaling;

    if ~in.vehicle_control_mode.flag_control_attitude_enabled && ...
            in.vehicle_control_mode.flag_control_manual_enabled && ~p.FW_ACRO_YAW_EN
        control_u(3) = in.manual_control_setpoint.yaw * p.FW_MAN_Y_SC;
        state.rate_int(3) = 0.0;
    end

    if all(isfinite(control_u))
        torque_sp.xyz = constrain(control_u + trim, -1.0, 1.0);
    else
        state.rate_int(:) = 0.0;
        torque_sp.xyz = trim;
    end

    if isfinite(rates_sp.thrust_body(1))
        thrust_sp.xyz(1) = rates_sp.thrust_body(1);
    else
        thrust_sp.xyz(1) = 0.0;
    end

    if p.FW_BAT_SCALE_EN && thrust_sp.xyz(1) > 0.1 && isfield(in, 'battery_status')
        bs = in.battery_status;

        if bs.connected && bs.scale > 0.0
            state.battery_scale = bs.scale;
        end

        thrust_sp.xyz(1) = thrust_sp.xyz(1) * state.battery_scale;
    end
else
    state.rate_int(:) = 0.0;
end

torque_sp.xyz(3) = constrain(torque_sp.xyz(3) + p.FW_RLL_TO_YAW_FF * torque_sp.xyz(1), -1.0, 1.0);

if in.vehicle_status.is_vtol_tailsitter
    helper = torque_sp.xyz(1);
    torque_sp.xyz(1) = torque_sp.xyz(3);
    torque_sp.xyz(3) = -helper;
end

if ~(in.vehicle_control_mode.flag_control_rates_enabled || ...
        in.vehicle_control_mode.flag_control_attitude_enabled || ...
        in.vehicle_control_mode.flag_control_manual_enabled)
    torque_sp = state.last_torque_sp;
    thrust_sp = state.last_thrust_sp;
end

state.last_torque_sp = torque_sp;
state.last_thrust_sp = thrust_sp;

status.rollspeed_integ = state.rate_int(1);
status.pitchspeed_integ = state.rate_int(2);
status.yawspeed_integ = state.rate_int(3);
status.airspeed_scaling = state.airspeed_scaling;
status.airspeed_used = airspeed;
end
