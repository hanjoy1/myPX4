function trim = fw_trim_schedule(airspeed, airspeed_scaling, p)
%FW_TRIM_SCHEDULE Match PX4 airspeed-dependent actuator trim scheduling.
trim = [p.TRIM_ROLL; p.TRIM_PITCH; p.TRIM_YAW] .* airspeed_scaling .* airspeed_scaling;

if airspeed < p.FW_AIRSPD_TRIM
    trim(1) = trim(1) + interpolate_px4(airspeed, p.FW_AIRSPD_MIN, p.FW_AIRSPD_TRIM, p.FW_DTRIM_R_VMIN, 0.0);
    trim(2) = trim(2) + interpolate_px4(airspeed, p.FW_AIRSPD_MIN, p.FW_AIRSPD_TRIM, p.FW_DTRIM_P_VMIN, 0.0);
    trim(3) = trim(3) + interpolate_px4(airspeed, p.FW_AIRSPD_MIN, p.FW_AIRSPD_TRIM, p.FW_DTRIM_Y_VMIN, 0.0);
else
    trim(1) = trim(1) + interpolate_px4(airspeed, p.FW_AIRSPD_TRIM, p.FW_AIRSPD_MAX, 0.0, p.FW_DTRIM_R_VMAX);
    trim(2) = trim(2) + interpolate_px4(airspeed, p.FW_AIRSPD_TRIM, p.FW_AIRSPD_MAX, 0.0, p.FW_DTRIM_P_VMAX);
    trim(3) = trim(3) + interpolate_px4(airspeed, p.FW_AIRSPD_TRIM, p.FW_AIRSPD_MAX, 0.0, p.FW_DTRIM_Y_VMAX);
end
end
