function test_fw_inner_loop()
%TEST_FW_INNER_LOOP Smoke test for the MATLAB PX4 fixed-wing inner loop.
p = init_px4_fw_control();

att_state = fw_att_control_init();
rate_state = fw_rate_control_init();

att_in.vehicle_attitude.q = euler_to_quat([radians(5); radians(1); radians(0)]);
att_in.vehicle_attitude_setpoint.q_d = euler_to_quat([radians(20); radians(4); radians(0)]);
att_in.vehicle_attitude_setpoint.thrust_body = [0.45; 0; 0];
att_in.vehicle_control_mode.flag_control_rates_enabled = true;
att_in.vehicle_control_mode.flag_control_attitude_enabled = true;
att_in.vehicle_control_mode.flag_control_manual_enabled = false;
att_in.vehicle_status.vehicle_type = 2;
att_in.vehicle_status.in_transition_mode = false;
att_in.vehicle_status.is_vtol = false;
att_in.vehicle_status.is_vtol_tailsitter = false;
att_in.vehicle_land_detected.landed = false;
att_in.airspeed_validated.calibrated_airspeed_m_s = 15.0;

[rates_sp, att_state] = fw_att_control_step(att_in, att_state, p);

rate_in.dt = 0.01;
rate_in.vehicle_angular_velocity.xyz = [0.05; 0.01; 0.0];
rate_in.vehicle_angular_velocity.xyz_derivative = [0; 0; 0];
rate_in.vehicle_rates_setpoint = rates_sp;
rate_in.vehicle_control_mode = att_in.vehicle_control_mode;
rate_in.vehicle_status = att_in.vehicle_status;
rate_in.vehicle_land_detected = att_in.vehicle_land_detected;
rate_in.airspeed_validated = att_in.airspeed_validated;
rate_in.manual_control_setpoint.yaw = 0.0;

[torque_sp, thrust_sp, rate_state, status] = fw_rate_control_step(rate_in, rate_state, p);

disp(rates_sp);
disp(torque_sp);
disp(thrust_sp);
disp(status);

assert(all(isfinite([rates_sp.roll; rates_sp.pitch; rates_sp.yaw])));
assert(all(isfinite(torque_sp.xyz)));
assert(isfinite(thrust_sp.xyz(1)));
end
