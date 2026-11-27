function [q_sp_vtol, f_sp_vtol] = tailsitter_attitude_setpoint( ...
    q_att, q_sp_mc, f_sp_mc, q_sp_fw, f_sp_fw, mode, params_vtol)
%TAILSITTER_ATTITUDE_SETPOINT 按照PX4尾座式VTOL逻辑生成姿态和推力期望。
%#codegen
%
% 本函数重点复现：
%   Tailsitter::update_transition_state()
%
% 输入四元数全部采用PX4顺序：
%   q = [w; x; y; z]
%
% 输入：
%   q_att       [4x1] 当前实际姿态。PX4使用它计算前、后转换旋转轴。
%   q_sp_mc     [4x1] 多旋翼控制器期望姿态。
%   f_sp_mc     标量，多旋翼机体Z轴期望推力，正常情况下小于零。
%   q_sp_fw     [4x1] 固定翼控制器期望姿态。
%   f_sp_fw     标量，固定翼机体X轴期望推力，正常情况下大于零。
%   mode        标量：
%                 0 = 多旋翼模式
%                 1 = 前转换
%                 2 = 固定翼模式
%                 3 = 后转换
%   params_vtol 参数：
%                 .dt
%                 .mode.duration_arsp_trans  对应VT_F_TRANS_DUR
%                 .mode.duration_back_trans  对应VT_B_TRANS_DUR
%                 .fw_psp_off                对应FW_PSP_OFF，单位deg
%
% 输出：
%   q_sp_vtol   [4x1] 最终期望姿态四元数。
%   f_sp_vtol   [3x1] 最终机体系期望推力。
%
% 与PX4源码一致的关键点：
%   1. 前转换不向q_sp_fw插值；
%   2. 转换姿态等于“轴角旋转四元数左乘转换起始四元数”；
%   3. 前转换起点使用MC期望pitch/yaw，并强制roll=0；
%   4. 后转换起点使用FW期望pitch、当前机头方向yaw，并强制roll=0；
%   5. 前、后转换期间均保留MC控制器给出的Z轴推力；
%   6. 后转换开始0.5 s内把最后FW推力混合到MC推力。
%
% PX4还会检查MC/FW姿态期望的时间戳是否在1 s以内。本函数没有时间戳
% 输入，因此假定每个周期输入的姿态期望均有效、均为最新值。

MODE_MC = uint8(0);
MODE_FRONT = uint8(1);
MODE_FW = uint8(2);
MODE_BACK = uint8(3);

BACK_THRUST_BLEND_DURATION = 0.5;

q_actual = normalizeQuat(q_att(:));
q_mc = normalizeQuat(q_sp_mc(:));
q_fw = normalizeQuat(q_sp_fw(:));
mode_now = uint8(mode);

dt = min(max(params_vtol.dt, 0.0), 0.1);
front_duration = max(params_vtol.mode.duration_arsp_trans, 0.1);
back_duration = max(params_vtol.mode.duration_back_trans, 0.1);
fw_pitch_offset = params_vtol.fw_psp_off * pi / 180.0;

% 这些持久变量对应PX4中的：
%   _flag_was_in_trans_mode
%   _q_trans_start
%   _q_trans_sp
%   _trans_rot_axis
%   _time_since_trans_start
% 以及后转换推力混合所需的_last_thr_in_fw_mode。
persistent previous_mode
persistent transition_time
persistent q_trans_start
persistent q_trans_sp
persistent trans_rot_axis
persistent last_fw_thrust_x

% 代码生成要求所有持久变量在任何读取路径之前都完成确定初始化。
if isempty(previous_mode)
    previous_mode = mode_now;
    transition_time = 0.0;
    q_trans_start = q_mc;
    q_trans_sp = q_mc;
    trans_rot_axis = [0.0; 1.0; 0.0];
    last_fw_thrust_x = f_sp_fw;
end

in_transition = mode_now == MODE_FRONT || mode_now == MODE_BACK;
was_in_transition = previous_mode == MODE_FRONT || previous_mode == MODE_BACK;
entering_transition = in_transition && ...
    (~was_in_transition || mode_now ~= previous_mode);

% -------------------------------------------------------------------------
% 进入转换状态时，只初始化一次转换起点和旋转轴。
% 对应PX4中的：
%   if (!_flag_was_in_trans_mode) { ... }
% -------------------------------------------------------------------------
if entering_transition
    transition_time = 0.0;

    if mode_now == MODE_FRONT
        % 前转换起点：
        %   const Eulerf setpoint_euler(mc_q_d);
        %   _q_trans_start = Eulerf(0, pitch, yaw);
        %
        % 保留MC期望俯仰和航向，但强制机翼水平，即roll=0。
        euler_mc = quatToEulerZYX(q_mc);
        q_trans_start = eulerZYXToQuat( ...
            0.0, euler_mc(2), euler_mc(3));

        % 当前实际机体X轴在世界坐标系中的方向：
        %   x = Dcm(q_att) * [1;0;0]
        body_x_world = quatRotate(q_actual, [1.0; 0.0; 0.0]);

        % PX4：
        %   _trans_rot_axis = -x.cross([0;0;-1])
        trans_rot_axis = -cross(body_x_world, [0.0; 0.0; -1.0]);
        trans_rot_axis = normalizeAxis( ...
            trans_rot_axis, [0.0; -1.0; 0.0]);

    else
        % 后转换首先使用实际姿态计算当前推力轴/机头几何方向：
        %   z = -q_trans_start.dcm_z()
        body_z_world = quatRotate(q_actual, [0.0; 0.0; 1.0]);
        z_direction = -body_z_world;

        % PX4：
        %   _trans_rot_axis = z.cross([0;0;-1])
        trans_rot_axis = cross(z_direction, [0.0; 0.0; -1.0]);
        trans_rot_axis = normalizeAxis( ...
            trans_rot_axis, [0.0; 1.0; 0.0]);

        % 后转换期间保持进入转换时飞机当前指向的水平航向。
        yaw_sp = atan2(z_direction(2), z_direction(1));

        % 当前接口假定FW姿态期望有效，因此直接使用其pitch。
        % 若要复现PX4的“FW期望超过1 s则pitch=0”退化逻辑，需要再增加
        % fw_sp_valid输入。
        euler_fw = quatToEulerZYX(q_fw);
        pitch_body = euler_fw(2);

        % 先构造机翼水平的FW语义姿态。
        q_trans_start = eulerZYXToQuat(0.0, pitch_body, yaw_sp);

        % 转换由MC姿态控制器执行，因此右乘绕Y轴-90°的固定变换。
        q_minus_90_pitch = eulerZYXToQuat(0.0, -pi / 2.0, 0.0);
        q_trans_start = quatMultiply( ...
            q_trans_start, q_minus_90_pitch);
    end

    % PX4进入转换时令当前转换期望等于转换起点。
    q_trans_sp = q_trans_start;
end

% 只有处于转换状态时才累计转换时间。
if in_transition
    transition_time = transition_time + dt;
else
    transition_time = 0.0;
end

% 默认输出。下面各模式再覆盖需要的通道。
q_sp_vtol = q_mc;
f_sp_vtol = [0.0; 0.0; f_sp_mc];

if mode_now == MODE_MC
    % 纯多旋翼模式直接使用MC姿态和MC负Z轴推力。
    q_sp_vtol = q_mc;
    f_sp_vtol = [0.0; 0.0; f_sp_mc];

elseif mode_now == MODE_FRONT
    % 归一化后按PX4公式计算tilt：
    %   tilt=0   表示尾座式悬停；
    %   tilt≈pi/2表示接近固定翼平飞。
    q_trans_sp = normalizeQuat(q_trans_sp);
    tilt = tailsitterTilt(q_trans_sp);

    % PX4使用恒定角速度完成约90°前转换。
    transition_pitch_rate = (pi / 2.0) / front_duration;

    % 注意：条件使用本周期更新前的q_trans_sp计算，与PX4执行顺序一致。
    % 达到FW_PSP_OFF对应的目标倾角后保持上一周期姿态，不继续旋转。
    if tilt < pi / 2.0 - fw_pitch_offset
        transition_angle = transition_time * transition_pitch_rate;
        q_rotation = axisAngleToQuat( ...
            trans_rot_axis, transition_angle);

        % PX4：
        %   q_trans_sp = AxisAngle(axis, angle) * q_trans_start
        q_trans_sp = quatMultiply(q_rotation, q_trans_start);
    end

    q_sp_vtol = normalizeQuat(q_trans_sp);

    % 转换期间姿态由MC控制器执行，同时保留MC负Z轴推力。
    f_sp_vtol = [0.0; 0.0; f_sp_mc];

elseif mode_now == MODE_FW
    % 纯固定翼模式才开始使用固定翼控制器姿态和正X轴推力。
    q_sp_vtol = q_fw;
    f_sp_vtol = [f_sp_fw; 0.0; 0.0];

elseif mode_now == MODE_BACK
    q_trans_sp = normalizeQuat(q_trans_sp);
    tilt = tailsitterTilt(q_trans_sp);

    % PX4使用恒定角速度完成约90°后转换。
    transition_pitch_rate = (pi / 2.0) / back_duration;

    % 当姿态距离悬停方向不足约0.01 rad时保持，不再越过悬停方向。
    if tilt > 0.01
        transition_angle = transition_time * transition_pitch_rate;
        q_rotation = axisAngleToQuat( ...
            trans_rot_axis, transition_angle);
        q_trans_sp = quatMultiply(q_rotation, q_trans_start);
    end

    q_sp_vtol = normalizeQuat(q_trans_sp);

    % update_transition_state()先使用当前MC负Z推力。
    thrust_z = f_sp_mc;

    % 后转换最初0.5 s，将最后FW正X推力反号后混入MC负Z推力。
    blend = clamp01( ...
        transition_time / BACK_THRUST_BLEND_DURATION);
    thrust_z = blend * thrust_z + ...
        (1.0 - blend) * (-last_fw_thrust_x);

    f_sp_vtol = [0.0; 0.0; thrust_z];

else
    % 无效模式采用确定的MC安全输出，避免代码生成出现未定义输出。
    q_sp_vtol = q_mc;
    f_sp_vtol = [0.0; 0.0; f_sp_mc];
end

% PX4在FW状态持续保存最近一次FW推力，供后转换开始时使用。
if mode_now == MODE_FW
    last_fw_thrust_x = f_sp_fw;
end

previous_mode = mode_now;
end

function tilt = tailsitterTilt(q)
% 使用PX4 Tailsitter::update_transition_state()中的原始倾角公式。
q = normalizeQuat(q);
cos_tilt = q(1) * q(1) - q(2) * q(2) ...
    - q(3) * q(3) + q(4) * q(4);
cos_tilt = min(max(cos_tilt, -1.0), 1.0);
tilt = acos(cos_tilt);
end

function axis_out = normalizeAxis(axis_in, fallback_axis)
% AxisAngle要求有效单位旋转轴；退化时使用确定的默认俯仰轴。
axis_norm = norm(axis_in);
if axis_norm > 1.0e-6
    axis_out = axis_in / axis_norm;
else
    axis_out = fallback_axis;
end
end

function q = axisAngleToQuat(axis, angle)
% 轴角转换为PX4顺序四元数[w;x;y;z]。
axis = normalizeAxis(axis, [0.0; 1.0; 0.0]);
half_angle = 0.5 * angle;
sin_half_angle = sin(half_angle);
q = [
    cos(half_angle);
    axis(1) * sin_half_angle;
    axis(2) * sin_half_angle;
    axis(3) * sin_half_angle
];
q = normalizeQuat(q);
end

function vector_out = quatRotate(q, vector_in)
% 使用四元数表示的主动旋转，将机体系向量旋转到世界坐标系。
q = normalizeQuat(q);
q_vector = q(2:4);
temporary = 2.0 * cross(q_vector, vector_in);
vector_out = vector_in + q(1) * temporary ...
    + cross(q_vector, temporary);
end

function euler = quatToEulerZYX(q)
% PX4四元数[w;x;y;z]转换为ZYX欧拉角[roll;pitch;yaw]。
q = normalizeQuat(q);
w = q(1);
x = q(2);
y = q(3);
z = q(4);

roll = atan2(2.0 * (w * x + y * z), ...
    1.0 - 2.0 * (x * x + y * y));

sin_pitch = 2.0 * (w * y - z * x);
sin_pitch = min(max(sin_pitch, -1.0), 1.0);
pitch = asin(sin_pitch);

yaw = atan2(2.0 * (w * z + x * y), ...
    1.0 - 2.0 * (y * y + z * z));

euler = [roll; pitch; yaw];
end

function q = eulerZYXToQuat(roll, pitch, yaw)
% ZYX欧拉角转换为PX4顺序四元数[w;x;y;z]。
half_roll = 0.5 * roll;
half_pitch = 0.5 * pitch;
half_yaw = 0.5 * yaw;

cr = cos(half_roll);
sr = sin(half_roll);
cp = cos(half_pitch);
sp = sin(half_pitch);
cy = cos(half_yaw);
sy = sin(half_yaw);

q = [
    cy * cp * cr + sy * sp * sr;
    cy * cp * sr - sy * sp * cr;
    cy * sp * cr + sy * cp * sr;
    sy * cp * cr - cy * sp * sr
];
q = normalizeQuat(q);
end

function q = quatMultiply(a, b)
% Hamilton乘法，保持PX4四元数顺序[w;x;y;z]。
a = normalizeQuat(a);
b = normalizeQuat(b);
q = [
    a(1) * b(1) - a(2) * b(2) - a(3) * b(3) - a(4) * b(4);
    a(1) * b(2) + a(2) * b(1) + a(3) * b(4) - a(4) * b(3);
    a(1) * b(3) - a(2) * b(4) + a(3) * b(1) + a(4) * b(2);
    a(1) * b(4) + a(2) * b(3) - a(3) * b(2) + a(4) * b(1)
];
q = normalizeQuat(q);
end

function q = normalizeQuat(q)
% 对四元数归一化，并为非法零四元数提供确定退化值。
quaternion_norm = norm(q);
if quaternion_norm < 1.0e-9
    q = [1.0; 0.0; 0.0; 0.0];
else
    q = q / quaternion_norm;
end
end

function value = clamp01(value)
value = min(max(value, 0.0), 1.0);
end
