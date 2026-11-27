function airspeed = get_airspeed_constrained_att(in, p)
%GET_AIRSPEED_CONSTRAINED_ATT Match PX4 attitude-controller airspeed choice.
airspeed = p.FW_AIRSPD_TRIM;

if isfield(in, 'airspeed_validated') && p.FW_USE_AIRSPD
    av = in.airspeed_validated;

    valid = isfield(av, 'calibrated_airspeed_m_s') && isfinite(av.calibrated_airspeed_m_s);

    if valid
        airspeed = max(0.5, av.calibrated_airspeed_m_s);
    end
end

if isfield(in, 'vehicle_status')
    vs = in.vehicle_status;

    if isfield(vs, 'is_vtol') && isfield(vs, 'vehicle_type') && isfield(vs, 'in_transition_mode')
        if vs.is_vtol && vs.vehicle_type == 1 && ~vs.in_transition_mode
            airspeed = p.FW_AIRSPD_STALL;
        end
    end
end

airspeed = constrain(airspeed, p.FW_AIRSPD_STALL, p.FW_AIRSPD_MAX);
end
