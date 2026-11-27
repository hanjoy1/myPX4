function [torque_sp, thrust_sp] = tailsitter_actuator_setpoint_simulink( ...
    vtol_mode, fw_torque_sp, fw_thrust_sp, ...
    mc_torque_sp, mc_thrust_sp, dt, vtol_params)
%TAILSITTER_ACTUATOR_SETPOINT_SIMULINK 尾座式VTOL期望力矩和推力选择。
%#codegen
%
% 所有输入输出端口都是固定尺寸的数值信号，可以直接用于Simulink的
% MATLAB Function模块。本函数实现PX4中
% Tailsitter::fill_actuator_outputs()的控制量选择和坐标轴映射。
%
% 输入
% ----
% vtol_mode       uint8标量：
%                   0 = 多旋翼模式
%                   1 = 前转换模式
%                   2 = 固定翼模式
%                   3 = 后转换模式
%
% fw_torque_sp    [3x1] 固定翼角速度控制器输出的期望力矩
% fw_thrust_sp    [3x1] 固定翼纵向控制器输出的期望推力
%                           正常固定翼推进力满足fw_thrust_sp(1)>0
%
% mc_torque_sp    [3x1] 多旋翼角速度控制器输出的期望力矩
% mc_thrust_sp    [3x1] 多旋翼控制器输出的期望推力
%                           正常多旋翼升力满足mc_thrust_sp(3)<0
%
% dt              标量，控制周期，单位为秒
%
% vtol_params     [5x1] 参数向量：
%   (1) VT_ELEV_MC_LOCK：0=多旋翼模式允许舵面动作，1=纯多旋翼模式锁住舵面
%   (2) VT_FW_DIFTHR_EN位掩码：偏航=1，滚转=2，俯仰=4
%   (3) VT_FW_DIFTHR_S_Y
%   (4) VT_FW_DIFTHR_S_P
%   (5) VT_FW_DIFTHR_S_R
%
% 输出
% ----
% torque_sp       [3x2]
% thrust_sp       [3x2]
%
% 第1列是PX4控制分配实例0，主要用于电机。
% 第2列是PX4控制分配实例1，主要用于气动舵面。
%
% 典型的下游连接方式：
%   motor_torque_sp   = torque_sp(:,1)  电机期望力矩
%   motor_thrust_sp   = thrust_sp(:,1)  电机期望推力
%   surface_torque_sp = torque_sp(:,2)  舵面期望力矩
%   surface_thrust_sp = thrust_sp(:,2)  舵面期望推力，本函数中始终为零

MODE_MC = uint8(0);
MODE_FRONT = uint8(1);
MODE_FW = uint8(2);
MODE_BACK = uint8(3);

YAW_BIT = uint32(1);
ROLL_BIT = uint32(2);
PITCH_BIT = uint32(4);
HANDOVER_HOLD_TIME = 0.05;

persistent previous_mode
persistent time_since_mode_change
persistent last_mc_thrust_z
persistent last_fw_thrust_x
persistent hold_last_fw_thrust
persistent handover_mc_thrust_z
persistent handover_fw_thrust_x

mode = uint8(vtol_mode);
fw_torque = fw_torque_sp(:);
fw_thrust = fw_thrust_sp(:);
mc_torque = mc_torque_sp(:);
mc_thrust = mc_thrust_sp(:);
dt_limited = min(max(dt, 0.0), 0.1);

% MATLAB Function模块进行代码生成时，persistent变量在被读取之前必须
% 能够被静态证明已经完成初始化。直接检查previous_mode，并在同一个
% 首次调用分支内初始化全部持久变量，避免代码生成器报告“某些执行路径
% 中未定义”。
if isempty(previous_mode)
    previous_mode = mode;
    time_since_mode_change = HANDOVER_HOLD_TIME;
    last_mc_thrust_z = mc_thrust(3);
    last_fw_thrust_x = fw_thrust(1);
    hold_last_fw_thrust = false;
    handover_mc_thrust_z = mc_thrust(3);
    handover_fw_thrust_x = fw_thrust(1);
end

if mode ~= previous_mode
    time_since_mode_change = 0.0;
    hold_last_fw_thrust = previous_mode == MODE_FW ...
        && (mode == MODE_BACK || mode == MODE_MC);

    if mode == MODE_FW
        handover_mc_thrust_z = last_mc_thrust_z;
    end

    if hold_last_fw_thrust
        handover_fw_thrust_x = last_fw_thrust_x;
    end
else
    time_since_mode_change = time_since_mode_change + dt_limited;
end

elev_mc_lock = vtol_params(1) > 0.5;
diff_thrust_mask = uint32(max(vtol_params(2), 0.0));
diff_scale_yaw = vtol_params(3);
diff_scale_pitch = vtol_params(4);
diff_scale_roll = vtol_params(5);

% PX4发布两个控制分配消息实例。每个控制周期都先将两个实例清零，
% 防止上一个飞行状态的某个轴控制量残留到当前周期。
torque_sp = zeros(3, 2);
thrust_sp = zeros(3, 2);

% ------------------------ 实例0：电机 ------------------------
if mode == MODE_FW
    % PX4尾座式坐标映射：
    %   固定翼推进力沿机体正X轴；
    %   电机/多旋翼接口的升力沿机体负Z轴。
    % 因此需要将固定翼X轴推力映射到电机Z轴，并改变符号。
    thrust_sp(3, 1) = -fw_thrust(1);

    % 可选的固定翼差动推力。下面的轴位置和参数名称严格保持PX4
    % Tailsitter::fill_actuator_outputs()中的对应关系。
    if bitand(diff_thrust_mask, YAW_BIT) ~= 0
        torque_sp(1, 1) = fw_torque(1) * diff_scale_yaw;
    end

    if bitand(diff_thrust_mask, PITCH_BIT) ~= 0
        torque_sp(2, 1) = fw_torque(2) * diff_scale_pitch;
    end

    if bitand(diff_thrust_mask, ROLL_BIT) ~= 0
        torque_sp(3, 1) = fw_torque(3) * diff_scale_roll;
    end

    % 刚进入固定翼模式时，FW控制器或TECS的新输出可能尚未到达。
    % PX4在最初50毫秒保持切换前最后一次多旋翼推力，
    % 同时将电机差动力矩清零，避免控制器接管瞬间发生突变。
    if time_since_mode_change < HANDOVER_HOLD_TIME
        thrust_sp(3, 1) = handover_mc_thrust_z;
        torque_sp(:, 1) = zeros(3, 1);
    end

else
    % 多旋翼、前转换和后转换状态均使用多旋翼角速度控制器产生的
    % 电机期望力矩和多旋翼负Z轴推力。
    thrust_sp(3, 1) = mc_thrust(3);
    torque_sp(:, 1) = mc_torque;

    % 后转换刚开始，或者固定翼故障导致直接切入多旋翼时，
    % 多旋翼控制器的新推力可能还没有到达。此时把切换瞬间冻结的
    % 最后固定翼正X轴推力转换为负Z轴推力，并保持50毫秒。
    % 前转换明确排除在该逻辑之外，与PX4保持一致。
    if mode ~= MODE_FRONT && time_since_mode_change < HANDOVER_HOLD_TIME ...
            && hold_last_fw_thrust
        thrust_sp(3, 1) = -handover_fw_thrust_x;
    end
end

% ---------------------- 实例1：气动舵面 ----------------------
% VT_ELEV_MC_LOCK只在纯多旋翼模式下锁住舵面。
% 前转换、后转换和固定翼模式始终允许固定翼角速度控制器的力矩作用于舵面。
if ~elev_mc_lock || mode ~= MODE_MC
    torque_sp(:, 2) = fw_torque;
end

% PX4尾座式逻辑不会给舵面控制分配实例提供推力，因此第2列推力始终为零。
thrust_sp(:, 2) = zeros(3, 1);

% 在完成本周期输出计算之后保存当前两套推力。
% 这样下一次发生模式切换时，能够使用上一个模式最后一个周期的真实推力。
last_mc_thrust_z = mc_thrust(3);
last_fw_thrust_x = fw_thrust(1);
previous_mode = mode;

if time_since_mode_change >= HANDOVER_HOLD_TIME
    hold_last_fw_thrust = false;
end
end
