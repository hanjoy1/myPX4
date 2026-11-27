# PX4 Fixed-Wing Inner Loop MATLAB Port

This folder ports the PX4 fixed-wing inner-loop modules for MATLAB/Simulink use:

- `fw_path_frontend`: cruise line/loiter command to lateral and longitudinal setpoints
- `vtol_tailsitter`: MC/FW attitude and thrust selection with front/back transition generation
- `fw_att_control`: `vehicle_attitude_setpoint` to `vehicle_rates_setpoint`
- `fw_rate_control`: `vehicle_rates_setpoint` to `vehicle_torque_setpoint` and `vehicle_thrust_setpoint`

The code is written as explicit step functions:

```matlab
p = init_px4_fw_control();
att_state = fw_att_control_init();
rate_state = fw_rate_control_init();

[rates_sp, att_state] = fw_att_control_step(att_in, att_state, p);
[torque_sp, thrust_sp, rate_state, status] = fw_rate_control_step(rate_in, rate_state, p);
```

## PX4 Source Mapping

| MATLAB file | PX4 source |
| --- | --- |
| `fw_path_frontend/fw_path_frontend_step.m` | cruise subset of `src/modules/fw_mode_manager/FixedWingModeManager.cpp` |
| `fw_path_frontend/fw_waypoint_route_step.m` | simplified `Navigator` route/waypoint switching layer |
| `vtol_tailsitter/tailsitter_setpoint_simulink.m` | numeric-only Simulink interface for tailsitter attitude/thrust selection |
| `vtol_tailsitter/tailsitter_actuator_setpoint_simulink.m` | numeric PX4 tailsitter MC/FW torque/thrust output selection |
| `vtol_tailsitter/tailsitter_mode_simulink.m` | numeric tailsitter MC/front-transition/FW/back-transition state machine |
| `fw_att_control/fw_att_control_step.m` | `src/modules/fw_att_control/FixedwingAttitudeControl.cpp` |
| `fw_att_control/fw_roll_controller.m` | `src/modules/fw_att_control/fw_roll_controller.cpp` |
| `fw_att_control/fw_pitch_controller.m` | `src/modules/fw_att_control/fw_pitch_controller.cpp` |
| `fw_att_control/fw_yaw_controller.m` | `src/modules/fw_att_control/fw_yaw_controller.cpp` |
| `fw_rate_control/fw_rate_control_step.m` | `src/modules/fw_rate_control/FixedwingRateControl.cpp` |
| `fw_rate_control/rate_control_update.m` | `src/lib/rate_control/rate_control.cpp` |

## Simulink Module Boundary

Use these as two MATLAB Function blocks:

```text
VehicleAttitude
VehicleAttitudeSetpoint
VehicleControlMode
VehicleStatus
VehicleLandDetected
AirspeedValidated
ManualControlSetpoint
        |
        v
fw_att_control_step
        |
        v
VehicleRatesSetpoint
        |
        v
fw_rate_control_step
        |
        v
VehicleTorqueSetpoint + VehicleThrustSetpoint
```

The state structs (`att_state`, `rate_state`) should be stored in Unit Delay/Memory blocks, or inside persistent variables if you wrap these functions in MATLAB Function blocks.

## Notes

- Quaternion convention is PX4-style `[w x y z]`.
- Angles are radians internally; parameter defaults that are degrees are converted at use sites, matching PX4.
- Vehicle type convention used here: `1 = rotary wing`, `2 = fixed wing`.
- This layer intentionally stops at torque/thrust setpoints. The next module for actuator outputs is PX4 `control_allocator`.
