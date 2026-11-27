function p = fw_default_params()
%FW_DEFAULT_PARAMS PX4 fixed-wing defaults needed by fw_att_control/fw_rate_control.
% Values mirror the parameter defaults in the PX4 source files.

p.CONSTANTS_ONE_G = 9.80665;

% Airspeed
p.FW_USE_AIRSPD = true;
p.FW_AIRSPD_MIN = 10.0;
p.FW_AIRSPD_MAX = 20.0;
p.FW_AIRSPD_STALL = 7.0;
p.FW_AIRSPD_TRIM = 15.0;
p.FW_ARSP_SCALE_EN = true;

% Fixed-wing attitude controller
p.FW_R_TC = 0.4;
p.FW_P_TC = 0.4;
p.FW_P_RMAX_POS = 60.0;
p.FW_P_RMAX_NEG = 60.0;
p.FW_R_RMAX = 70.0;
p.FW_Y_RMAX = 50.0;
p.FW_MAN_YR_MAX = 30.0;
p.FW_MAN_R_MAX = 45.0;
p.FW_MAN_P_MAX = 30.0;
p.FW_PSP_OFF = 0.0;

% Wheel steering controller
p.FW_W_EN = false;
p.FW_W_RMAX = 30.0;
p.FW_WR_P = 0.5;
p.FW_WR_I = 0.1;
p.FW_WR_IMAX = 0.4;
p.FW_WR_FF = 0.2;

% Fixed-wing rate controller
p.FW_PR_P = 0.08;
p.FW_PR_I = 0.1;
p.FW_PR_D = 0.0;
p.FW_PR_IMAX = 0.4;
p.FW_RR_P = 0.05;
p.FW_RR_I = 0.1;
p.FW_RR_D = 0.0;
p.FW_RR_IMAX = 0.2;
p.FW_YR_P = 0.05;
p.FW_YR_I = 0.1;
p.FW_YR_D = 0.0;
p.FW_YR_IMAX = 0.2;
p.FW_RR_FF = 0.5;
p.FW_PR_FF = 0.5;
p.FW_YR_FF = 0.3;
p.FW_RLL_TO_YAW_FF = 0.0;

% Acro/manual direct control
p.FW_ACRO_X_MAX = 90.0;
p.FW_ACRO_Y_MAX = 90.0;
p.FW_ACRO_Z_MAX = 45.0;
p.FW_ACRO_YAW_EN = false;
p.FW_MAN_R_SC = 1.0;
p.FW_MAN_P_SC = 1.0;
p.FW_MAN_Y_SC = 1.0;

% Trim scheduling
p.TRIM_ROLL = 0.0;
p.TRIM_PITCH = 0.0;
p.TRIM_YAW = 0.0;
p.FW_DTRIM_R_VMIN = 0.0;
p.FW_DTRIM_P_VMIN = 0.0;
p.FW_DTRIM_Y_VMIN = 0.0;
p.FW_DTRIM_R_VMAX = 0.0;
p.FW_DTRIM_P_VMAX = 0.0;
p.FW_DTRIM_Y_VMAX = 0.0;

% Optional PX4 features surfaced as inputs in this MATLAB version.
p.FW_BAT_SCALE_EN = false;
p.VT_FW_DIFTHR_EN = int32(0);
end
