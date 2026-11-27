function [airspeed, state] = get_airspeed_and_update_scaling(in, state, p)
%GET_AIRSPEED_AND_UPDATE_SCALING Match PX4 fixed-wing rate-controller scaling.
airspeed = p.FW_AIRSPD_TRIM;

if isfield(in, 'airspeed_validated') && p.FW_USE_AIRSPD
    av = in.airspeed_validated;

    if isfield(av, 'calibrated_airspeed_m_s') && isfinite(av.calibrated_airspeed_m_s)
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

if p.FW_ARSP_SCALE_EN
    min_airspeed = max(p.FW_AIRSPD_STALL, 0.1);
    airspeed_constrained = max(airspeed, min_airspeed);
    state.airspeed_scaling = p.FW_AIRSPD_TRIM / airspeed_constrained;
else
    state.airspeed_scaling = 1.0;
end
end
