function msg = vehicle_rates_setpoint_zero()
%VEHICLE_RATES_SETPOINT_ZERO Zeroed vehicle_rates_setpoint-like struct.
msg.roll = 0.0;
msg.pitch = 0.0;
msg.yaw = 0.0;
msg.thrust_body = [0.0; 0.0; 0.0];
msg.reset_integral = false;
end
