function state = fw_att_control_init()
%FW_ATT_CONTROL_INIT Internal states for PX4 fixed-wing attitude controller.
empty.euler_rate_sp = 0.0;
empty.body_rate_sp = 0.0;

state.roll = empty;
state.pitch = empty;
state.yaw = empty;
state.last_rates_sp = vehicle_rates_setpoint_zero();
end
