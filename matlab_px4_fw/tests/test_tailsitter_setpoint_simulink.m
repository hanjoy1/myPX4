function test_tailsitter_setpoint_simulink()
%TEST_TAILSITTER_SETPOINT_SIMULINK Numeric-port smoke test.
mc_q = [1; 0; 0; 0];
fw_q = [1; 0; 0; 0];
transition_params = [1.0; 1.0; 1.0; 0.5];

[q_d, thrust, progress, active] = tailsitter_setpoint_simulink( ...
    mc_q, -0.5, fw_q, 0.4, uint8(0), 0.1, true, ...
    transition_params);
assert(norm(q_d - mc_q) < 1e-9);
assert(abs(thrust(3) + 0.5) < 1e-9);
assert(progress == 0);
assert(~active);

for k = 1:5
    [q_d, thrust, progress, active] = tailsitter_setpoint_simulink( ...
        mc_q, -0.5, fw_q, 0.4, uint8(1), 0.1, false, ...
        transition_params);
end
assert(abs(norm(q_d) - 1.0) < 1e-9);
assert(abs(thrust(3) + 0.5) < 1e-9);
assert(abs(progress - 0.5) < 1e-9);
assert(active);

[q_d, thrust] = tailsitter_setpoint_simulink( ...
    mc_q, -0.5, fw_q, 0.4, uint8(2), 0.1, false, ...
    transition_params);
assert(norm(q_d - fw_q) < 1e-9);
assert(abs(thrust(1) - 0.5) < 1e-9);

for k = 1:5
    [q_d, thrust, progress, active] = tailsitter_setpoint_simulink( ...
        mc_q, -0.5, fw_q, 0.4, uint8(3), 0.1, false, ...
        transition_params);
end
assert(abs(norm(q_d) - 1.0) < 1e-9);
assert(abs(thrust(3) + 0.5) < 1e-9);
assert(abs(progress - 0.5) < 1e-9);
assert(active);

disp('test_tailsitter_setpoint_simulink: PASS');
end
