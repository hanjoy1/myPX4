function mode = tailsitter_mode_simulink( ...
    fixed_wing_requested, pitch, calibrated_airspeed, ...
    dt, mode_params)
%TAILSITTER_MODE_SIMULINK 尾座式VTOL四状态切换。
%#codegen
%
% 本函数保留PX4 Tailsitter::update_vtol_state()中的正常飞行状态逻辑，
% 根据固定翼请求、俯仰角和空速判断当前VTOL状态。
% 本简化版本不处理固定翼故障、空速有效性、解锁状态和着陆状态。
% 函数内部使用persistent变量保存上一周期状态和转换时间，
% 外部端口全部为固定尺寸数值信号，适合Simulink MATLAB Function模块。
%
% 输入
% ----
% fixed_wing_requested  Boolean：true表示请求进入或保持固定翼模式
% pitch                 标量：当前实际俯仰角，单位rad
% calibrated_airspeed   标量：当前校准空速，单位m/s
% dt                    标量：控制周期，单位s
%
% mode_params           [4x1]参数向量：
%   (1) 前转换完成俯仰角门限，PX4默认约为-1.05 rad，即-60度
%   (2) 后转换完成俯仰角门限，PX4默认约为-0.26 rad，即-15度
%   (3) 后转换最大持续时间VT_B_TRANS_DUR，单位s
%   (4) 前转换完成空速VT_ARSP_TRANS，单位m/s
%
% 输出
% ----
% mode                  uint8标量：
%   0 = 多旋翼模式
%   1 = 前转换模式
%   2 = 固定翼模式
%   3 = 后转换模式
%
% 推荐参数示例：
% mode_params = [-1.05; -0.26; 5.0; 10.0];

MODE_MC = uint8(0);
MODE_FRONT = uint8(1);
MODE_FW = uint8(2);
MODE_BACK = uint8(3);

persistent initialized
persistent current_mode
persistent transition_time

if isempty(initialized)
    initialized = false;
end

% 模型启动后的第一次调用始终从多旋翼模式开始，与PX4启动行为一致。
if ~initialized
    current_mode = MODE_MC;
    transition_time = 0.0;
    initialized = true;
end

dt_limited = min(max(dt, 0.0), 0.1);
pitch_to_fw = mode_params(1);
pitch_to_mc = mode_params(2);
back_transition_duration = max(mode_params(3), 0.1);
transition_airspeed = max(mode_params(4), 0.0);

% 只有处于前转换或后转换时才累计转换时间。
if current_mode == MODE_FRONT || current_mode == MODE_BACK
    transition_time = transition_time + dt_limited;
end

if ~fixed_wing_requested
    % 当前请求多旋翼模式。
    if current_mode == MODE_MC
        % 已经处于多旋翼模式，保持当前状态。

    elseif current_mode == MODE_FW
        % 从稳定固定翼状态收到多旋翼请求，开始后转换。
        current_mode = MODE_BACK;
        transition_time = 0.0;

    elseif current_mode == MODE_FRONT
        % 前转换尚未完成时取消固定翼请求，直接返回多旋翼模式。
        current_mode = MODE_MC;
        transition_time = 0.0;

    elseif current_mode == MODE_BACK
        % 后转换达到俯仰角门限，或者超过最大持续时间后进入多旋翼模式。
        back_attitude_reached = pitch >= pitch_to_mc;
        backtransition_timeout = transition_time > back_transition_duration;

        if back_attitude_reached || backtransition_timeout
            current_mode = MODE_MC;
            transition_time = 0.0;
        end
    end

else
    % 当前请求固定翼模式。
    if current_mode == MODE_MC
        % 从多旋翼悬停状态开始前转换。
        current_mode = MODE_FRONT;
        transition_time = 0.0;

    elseif current_mode == MODE_FW
        % 已经处于稳定固定翼状态，保持当前状态。

    elseif current_mode == MODE_FRONT
        % 前转换必须同时达到俯仰角门限和转换空速。
        front_attitude_reached = pitch <= pitch_to_fw;
        front_airspeed_reached = calibrated_airspeed >= transition_airspeed;
        front_transition_completed = front_attitude_reached ...
            && front_airspeed_reached;

        if front_transition_completed
            current_mode = MODE_FW;
            transition_time = 0.0;
        end

    elseif current_mode == MODE_BACK
        % 后转换期间重新请求固定翼，直接中止后转换并回到固定翼模式。
        current_mode = MODE_FW;
        transition_time = 0.0;
    end
end

mode = current_mode;
end
