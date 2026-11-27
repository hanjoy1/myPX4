function test_tailsitter_attitude_setpoint()
%TEST_TAILSITTER_ATTITUDE_SETPOINT Exercise all four tailsitter modes.
params = tailsitter_attitude_setpoint_defaults();
params.front_transition_duration = 1.0;
params.back_transition_duration = 1.0;

mc_sp = struct( ...
    'q_d', euler_to_quat([0; 0; 0]), ...
    'thrust_body', [0; 0; -0.5]);
fw_sp = struct( ...
    'q_d', euler_to_quat([0; 0; 0]), ...
    'thrust_body', [0.4; 0; 0]);

state = tailsitter_attitude_setpoint_init(mc_sp);
control = struct('mode', uint8(0), 'dt', 0.01);

[sp, state] = tailsitter_attitude_setpoint_step( ...
    mc_sp, fw_sp, control, state, params);
assert(norm(sp.q_d - mc_sp.q_d) < 1e-9);
assert(abs(sp.thrust_body(3) + 0.5) < 1e-9);

control.mode = uint8(1);
control.dt = 0.1;
for k = 1:5
    [sp, state, status] = tailsitter_attitude_setpoint_step( ...
        mc_sp, fw_sp, control, state, params);
end
assert(status.in_transition);
assert(abs(status.transition_progress - 0.5) < 1e-9);
assert(abs(norm(sp.q_d) - 1.0) < 1e-9);
assert(abs(sp.thrust_body(3) + 0.5) < 1e-9);

control.mode = uint8(2);
control.dt = 0.01;
[sp, state] = tailsitter_attitude_setpoint_step( ...
    mc_sp, fw_sp, control, state, params);
assert(abs(norm(sp.q_d) - 1.0) < 1e-9);
assert(sp.thrust_body(1) > 0.4);

control.mode = uint8(3);
control.dt = 0.1;
for k = 1:5
    [sp, state, status] = tailsitter_attitude_setpoint_step( ...
        mc_sp, fw_sp, control, state, params);
end
assert(status.in_transition);
assert(abs(status.transition_progress - 0.5) < 1e-9);
assert(abs(norm(sp.q_d) - 1.0) < 1e-9);
assert(abs(sp.thrust_body(3) + 0.5) < 1e-9);

disp('test_tailsitter_attitude_setpoint: PASS');
end
