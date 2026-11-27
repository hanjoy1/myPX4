function state = fw_rate_control_init()
%FW_RATE_CONTROL_INIT Internal states for PX4 fixed-wing rate controller.
state.rate_int = [0.0; 0.0; 0.0];
state.airspeed_scaling = 1.0;
state.battery_scale = 1.0;
state.last_torque_sp = vehicle_torque_setpoint_zero();
state.last_thrust_sp = vehicle_thrust_setpoint_zero();
end
