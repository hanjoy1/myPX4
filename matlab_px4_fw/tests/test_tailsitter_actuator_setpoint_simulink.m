function test_tailsitter_actuator_setpoint_simulink()
%TEST_TAILSITTER_ACTUATOR_SETPOINT_SIMULINK 检查PX4尾座式控制量映射。
fw_torque = [0.1; 0.2; 0.3];
fw_thrust = [0.4; 0; 0];
mc_torque = [0.5; 0.6; 0.7];
mc_thrust = [0; 0; -0.55];

% 多旋翼模式不锁舵面，固定翼三个差动推力使能位全部打开。
p = [0; 7; 2; 3; 4];

% 在多旋翼模式初始化：电机采用MC输出，舵面采用FW角速度控制器输出。
[torque, thrust] = tailsitter_actuator_setpoint_simulink( ...
    uint8(0), fw_torque, fw_thrust, mc_torque, mc_thrust, 0.01, p);
assert(norm(torque(:, 1) - mc_torque) < 1e-9);
assert(abs(thrust(3, 1) + 0.55) < 1e-9);
assert(norm(torque(:, 2) - fw_torque) < 1e-9);

% 前转换：电机采用MC输出，气动舵面采用FW输出。
[torque, thrust] = tailsitter_actuator_setpoint_simulink( ...
    uint8(1), fw_torque, fw_thrust, mc_torque, mc_thrust, 0.01, p);
assert(norm(torque(:, 1) - mc_torque) < 1e-9);
assert(abs(thrust(3, 1) + 0.55) < 1e-9);
assert(norm(torque(:, 2) - fw_torque) < 1e-9);

% 进入固定翼：最初50毫秒保持最后MC推力，并将电机差动力矩清零。
[torque, thrust] = tailsitter_actuator_setpoint_simulink( ...
    uint8(2), fw_torque, fw_thrust, mc_torque, mc_thrust, 0.01, p);
assert(norm(torque(:, 1)) < 1e-9);
assert(abs(thrust(3, 1) + 0.55) < 1e-9);

for k = 1:5
    [torque, thrust] = tailsitter_actuator_setpoint_simulink( ...
        uint8(2), fw_torque, fw_thrust, mc_torque, mc_thrust, 0.01, p);
end
assert(norm(torque(:, 1) - [0.2; 0.6; 1.2]) < 1e-9);
assert(abs(thrust(3, 1) + 0.4) < 1e-9);

% 进入后转换：短暂保持最后FW推力，但电机力矩已经使用MC输出。
[torque, thrust] = tailsitter_actuator_setpoint_simulink( ...
    uint8(3), fw_torque, fw_thrust, mc_torque, mc_thrust, 0.01, p);
assert(norm(torque(:, 1) - mc_torque) < 1e-9);
assert(abs(thrust(3, 1) + 0.4) < 1e-9);

% VT_ELEV_MC_LOCK=1时，只有纯多旋翼模式会将舵面力矩清零。
p(1) = 1;
for k = 1:5
    tailsitter_actuator_setpoint_simulink( ...
        uint8(3), fw_torque, fw_thrust, mc_torque, mc_thrust, 0.01, p);
end
[torque, ~] = tailsitter_actuator_setpoint_simulink( ...
    uint8(0), fw_torque, fw_thrust, mc_torque, mc_thrust, 0.01, p);
assert(norm(torque(:, 2)) < 1e-9);

disp('test_tailsitter_actuator_setpoint_simulink: PASS');
end
