/****************************************************************************
 *
 *   Copyright (c) 2018 - 2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file PositionControl.cpp
 */

#include "PositionControl.hpp"
#include "ControlMath.hpp"
#include <float.h>
#include <mathlib/mathlib.h>
#include <px4_platform_common/defines.h>
#include <geo/geo.h>

using namespace matrix;

const trajectory_setpoint_s PositionControl::empty_trajectory_setpoint = {0, {NAN, NAN, NAN}, {NAN, NAN, NAN}, {NAN, NAN, NAN}, {NAN, NAN, NAN}, NAN, NAN};

// 设置速度 PID 三项增益，分别对应 x/y/z 轴。
void PositionControl::setVelocityGains(const Vector3f &P, const Vector3f &I, const Vector3f &D)
{
	_gain_vel_p = P;
	_gain_vel_i = I;
	_gain_vel_d = D;
}

// 设置速度限制：水平速度、上升速度和下降速度分别限幅。
void PositionControl::setVelocityLimits(const float vel_horizontal, const float vel_up, const float vel_down)
{
	_lim_vel_horizontal = vel_horizontal;
	_lim_vel_up = vel_up;
	_lim_vel_down = vel_down;
}

// 设置归一化推力上下限，并保证最小推力不为 0，便于后续由推力方向推断姿态。
void PositionControl::setThrustLimits(const float min, const float max)
{
	// make sure there's always enough thrust vector length to infer the attitude
	// 确保推力矢量长度不会太小，否则下游难以从推力方向推断姿态。
	_lim_thr_min = math::max(min, 10e-4f);
	_lim_thr_max = max;
}

// 设置垂向控制优先时保留给水平控制的推力余量。
void PositionControl::setHorizontalThrustMargin(const float margin)
{
	_lim_thr_xy_margin = margin;
}

// 更新悬停推力估计，并通过调整积分项保持输出推力连续。
void PositionControl::updateHoverThrust(const float hover_thrust_new)
{
	// Given that the equation for thrust is T = a_sp * Th / g - Th
	// with a_sp = desired acceleration, Th = hover thrust and g = gravity constant,
	// we want to find the acceleration that needs to be added to the integrator in order obtain
	// the same thrust after replacing the current hover thrust by the new one.
	// T' = T => a_sp' * Th' / g - Th' = a_sp * Th / g - Th
	// so a_sp' = (a_sp - g) * Th / Th' + g
	// we can then add a_sp' - a_sp to the current integrator to absorb the effect of changing Th by Th'
	// 上面这组公式是在求“悬停推力改变前后保持同一输出推力”所需的等效加速度补偿。
	// 中文说明：悬停油门估计值变化时，如果直接替换 hover_thrust，输出推力会突变。
	// 这里把等效差值补到速度积分项里，使控制器输出尽量连续。
	const float previous_hover_thrust = _hover_thrust;
	setHoverThrust(hover_thrust_new);

	_vel_int(2) += (_acc_sp(2) - CONSTANTS_ONE_G) * previous_hover_thrust / _hover_thrust
		       + CONSTANTS_ONE_G - _acc_sp(2);
}

// 写入当前飞机状态，供下一次 update() 使用。
void PositionControl::setState(const PositionControlStates &states)
{
	_pos = states.position;
	_vel = states.velocity;
	_yaw = states.yaw;
	_vel_dot = states.acceleration;
}

// 写入外部输入的轨迹设定值；NaN 元素会在控制中表示“不设置/不控制”。
void PositionControl::setInputSetpoint(const trajectory_setpoint_s &setpoint)
{
	_pos_sp = Vector3f(setpoint.position);
	_vel_sp = Vector3f(setpoint.velocity);
	_acc_sp = Vector3f(setpoint.acceleration);
	_yaw_sp = setpoint.yaw;
	_yawspeed_sp = setpoint.yawspeed;
}

// 执行一次完整串级位置控制，成功时内部会生成推力和 yaw/yawspeed 设定值。
bool PositionControl::update(const float dt)
{

	bool valid = _inputValid();

	if (valid) {
		// 串级控制主路径：位置 P 环先修正速度设定值，速度 PID 环再生成推力设定值。
		_positionControl();
		_velocityControl(dt);

		// 没有给 yaw/yawspeed 时保持当前航向、偏航角速度置零，避免向下游发布 NaN。
		_yawspeed_sp = PX4_ISFINITE(_yawspeed_sp) ? _yawspeed_sp : 0.f;
		_yaw_sp = PX4_ISFINITE(_yaw_sp) ? _yaw_sp : _yaw; // TODO: better way to disable yaw control
	}

	// There has to be a valid output acceleration and thrust setpoint otherwise something went wrong
	// 输出加速度和推力必须都是有限值，否则说明输入组合或控制计算不可执行。
	return valid && _acc_sp.isAllFinite() && _thr_sp.isAllFinite();
}

// 位置 P 环：位置误差生成速度设定值，并与速度前馈合成后限幅。
void PositionControl::_positionControl()
{
	// P-position controller
	// 位置误差乘 P 增益得到“由位置环要求的速度”。
	if(_lim_vel_up > 2.5f)
	{
		int a = 1;
		(void)a;

	}
	Vector3f vel_sp_position = (_pos_sp - _pos).emult(_gain_pos_p);
	// Position and feed-forward velocity setpoints or position states being NAN results in them not having an influence
	// 如果外部给了速度设定值，它作为前馈叠加；NaN 表示该项不参与。
	ControlMath::addIfNotNanVector3f(_vel_sp, vel_sp_position);
	// make sure there are no NAN elements for further reference while constraining
	// 后续限幅计算不能带 NaN，因此把位置环输出中未参与控制的轴清零。
	ControlMath::setZeroIfNanVector3f(vel_sp_position);

	// Constrain horizontal velocity by prioritizing the velocity component along the
	// the desired position setpoint over the feed-forward term.
	// 水平限速时优先保留位置误差闭环产生的速度，前馈速度在剩余裕量内缩放。
	_vel_sp.xy() = ControlMath::constrainXY(vel_sp_position.xy(), (_vel_sp - vel_sp_position).xy(), _lim_vel_horizontal);
	// Constrain velocity in z-direction.
	// NED 坐标：上升是负 z，所以速度下限为 -_lim_vel_up，下降上限为 _lim_vel_down。
	_vel_sp(2) = math::constrain(_vel_sp(2), -_lim_vel_up, _lim_vel_down);
}

// 速度 PID 环：速度误差生成加速度设定值，再转换为推力并处理积分抗饱和。
void PositionControl::_velocityControl(const float dt)
{
	// Constrain vertical velocity integral
	// 垂向积分项限制在 +/-1g，避免高度误差长期积累导致过大的推力请求。
	_vel_int(2) = math::constrain(_vel_int(2), -CONSTANTS_ONE_G, CONSTANTS_ONE_G);

	// PID velocity control
	// D 项使用速度导数 _vel_dot，相当于对速度误差中的测量项做微分。
	Vector3f vel_error = _vel_sp - _vel;
	Vector3f acc_sp_velocity = vel_error.emult(_gain_vel_p) + _vel_int - _vel_dot.emult(_gain_vel_d);

	// No control input from setpoints or corresponding states which are NAN
	// 外部加速度设定值可作为前馈；没有前馈时由速度 PID 的输出填补。
	ControlMath::addIfNotNanVector3f(_acc_sp, acc_sp_velocity);

	// 把期望加速度换算成归一化推力矢量，并施加倾角限制。
	_accelerationControl();

	// Integrator anti-windup in vertical direction
	// 垂向推力已经饱和且误差还在把积分推向更饱和方向时，停止该方向积分。
	if ((_thr_sp(2) >= -_lim_thr_min && vel_error(2) >= 0.f) ||
	    (_thr_sp(2) <= -_lim_thr_max && vel_error(2) <= 0.f)) {
		vel_error(2) = 0.f;
	}

	// Prioritize vertical control while keeping a horizontal margin
	// 推力总量有限：先满足垂向控制，但保留 _lim_thr_xy_margin 给水平控制。
	const Vector2f thrust_sp_xy(_thr_sp);
	const float thrust_sp_xy_norm = thrust_sp_xy.norm();
	const float thrust_max_squared = math::sq(_lim_thr_max);

	// Determine how much vertical thrust is left keeping horizontal margin
	// 计算在保留水平推力余量后，垂向最多还能使用多少推力。
	const float allocated_horizontal_thrust = math::min(thrust_sp_xy_norm, _lim_thr_xy_margin);
	const float thrust_z_max_squared = thrust_max_squared - math::sq(allocated_horizontal_thrust);

	// Saturate maximal vertical thrust
	// 限制最大垂向推力，避免总推力超过上限。
	_thr_sp(2) = math::max(_thr_sp(2), -sqrtf(thrust_z_max_squared));

	// Determine how much horizontal thrust is left after prioritizing vertical control
	// 在垂向控制优先后，计算剩余可分配给水平控制的推力。
	const float thrust_max_xy_squared = thrust_max_squared - math::sq(_thr_sp(2));
	float thrust_max_xy = 0.f;

	if (thrust_max_xy_squared > 0.f) {
		thrust_max_xy = sqrtf(thrust_max_xy_squared);
	}

	// Saturate thrust in horizontal direction
	// 如果水平推力超过剩余能力，则按方向等比例缩小。
	if (thrust_sp_xy_norm > thrust_max_xy) {
		_thr_sp.xy() = thrust_sp_xy / thrust_sp_xy_norm * thrust_max_xy;
	}

	// Use tracking Anti-Windup for horizontal direction: during saturation, the integrator is used to unsaturate the output
	// see Anti-Reset Windup for PID controllers, L.Rundqwist, 1990
	// 水平方向采用 tracking anti-windup：如果推力限幅导致实际可产生的水平加速度小于期望值，
	// 则反向修正速度误差，让积分项帮助输出从饱和中恢复。
	const Vector2f acc_sp_xy_produced = Vector2f(_thr_sp) * (CONSTANTS_ONE_G / _hover_thrust);

	// The produced acceleration can be greater or smaller than the desired acceleration due to the saturations and the actual vertical thrust (computed independently).
	// The ARW loop needs to run if the signal is saturated only.
	// 由于推力饱和和垂向推力独立计算，实际能产生的水平加速度可能小于期望；
	// 只有检测到饱和时才运行 ARW 修正。
	if (_acc_sp.xy().norm_squared() > acc_sp_xy_produced.norm_squared()) {
		const float arw_gain = 2.f / _gain_vel_p(0);
		const Vector2f acc_sp_xy = _acc_sp.xy();

		vel_error.xy() = Vector2f(vel_error) - arw_gain * (acc_sp_xy - acc_sp_xy_produced);
	}

	// Make sure integral doesn't get NAN
	// 防止 NaN 误差进入积分项。
	ControlMath::setZeroIfNanVector3f(vel_error);
	// Update integral part of velocity control
	// 更新速度环积分项。
	_vel_int += vel_error.emult(_gain_vel_i) * dt;
}

// 加速度控制：把期望加速度转换成受倾角/推力限制的归一化推力矢量。
void PositionControl::_accelerationControl()
{
	// Assume standard acceleration due to gravity in vertical direction for attitude generation
	// body_z 是机体 z 轴在世界坐标中的期望方向。PX4 中推力方向与机体 z 轴相反，
	// 因此用 -acc_x/-acc_y 和重力项构造期望姿态。
	float z_specific_force = -CONSTANTS_ONE_G;

	if (!_decouple_horizontal_and_vertical_acceleration) {
		// Include vertical acceleration setpoint for better horizontal acceleration tracking
		// 耦合模式下，垂向加速度也会影响推力方向，能更精确跟踪三维加速度。
		z_specific_force += _acc_sp(2);
	}

	Vector3f body_z = Vector3f(-_acc_sp(0), -_acc_sp(1), -z_specific_force).normalized();
	ControlMath::limitTilt(body_z, Vector3f(0, 0, 1), _lim_tilt);
	// Convert to thrust assuming hover thrust produces standard gravity
	// NED z 向下：悬停推力为负；期望向上加速时 acc_sp(2)<0，会要求更大的负推力。
	const float thrust_ned_z = _acc_sp(2) * (_hover_thrust / CONSTANTS_ONE_G) - _hover_thrust;
	// Project thrust to planned body attitude
	// 将 NED z 方向推力投影到受倾角限制后的 body_z 方向，得到总推力大小。
	const float cos_ned_body = (Vector3f(0, 0, 1).dot(body_z));
	const float collective_thrust = math::min(thrust_ned_z / cos_ned_body, -_lim_thr_min);
	_thr_sp = body_z * collective_thrust;
}

// 检查输入设定值和状态估计是否足够完整，避免用无效轴参与控制。
bool PositionControl::_inputValid()
{
	bool valid = true;

	// Every axis x, y, z needs to have some setpoint
	// 每个轴至少要有位置、速度或加速度中的一种设定值，否则控制器不知道这一轴该做什么。
	for (int i = 0; i <= 2; i++) {
		valid = valid && (PX4_ISFINITE(_pos_sp(i)) || PX4_ISFINITE(_vel_sp(i)) || PX4_ISFINITE(_acc_sp(i)));
	}

	// x and y input setpoints always have to come in pairs
	// 水平 x/y 必须成对控制，避免只控制一个水平轴导致方向和限速逻辑不一致。
	valid = valid && (PX4_ISFINITE(_pos_sp(0)) == PX4_ISFINITE(_pos_sp(1)));
	valid = valid && (PX4_ISFINITE(_vel_sp(0)) == PX4_ISFINITE(_vel_sp(1)));
	valid = valid && (PX4_ISFINITE(_acc_sp(0)) == PX4_ISFINITE(_acc_sp(1)));

	// For each controlled state the estimate has to be valid
	for (int i = 0; i <= 2; i++) {
		if (PX4_ISFINITE(_pos_sp(i))) {
			// 控制位置的轴必须有有效位置估计。
			valid = valid && PX4_ISFINITE(_pos(i));
		}

		if (PX4_ISFINITE(_vel_sp(i))) {
			// 控制速度的轴必须有有效速度和速度导数估计。
			valid = valid && PX4_ISFINITE(_vel(i)) && PX4_ISFINITE(_vel_dot(i));
		}
	}

	return valid;
}

// 输出内部实际执行的位置/速度/加速度/推力设定值，供日志和其他模块使用。
void PositionControl::getLocalPositionSetpoint(vehicle_local_position_setpoint_s &local_position_setpoint) const
{
	local_position_setpoint.x = _pos_sp(0);
	local_position_setpoint.y = _pos_sp(1);
	local_position_setpoint.z = _pos_sp(2);
	local_position_setpoint.yaw = _yaw_sp;
	local_position_setpoint.yawspeed = _yawspeed_sp;
	local_position_setpoint.vx = _vel_sp(0);
	local_position_setpoint.vy = _vel_sp(1);
	local_position_setpoint.vz = _vel_sp(2);
	_acc_sp.copyTo(local_position_setpoint.acceleration);
	_thr_sp.copyTo(local_position_setpoint.thrust);
}

// 将推力矢量和 yaw 设定值转换为姿态控制器可执行的 attitude_setpoint。
void PositionControl::getAttitudeSetpoint(vehicle_attitude_setpoint_s &attitude_setpoint) const
{
	ControlMath::thrustToAttitude(_thr_sp, _yaw_sp, attitude_setpoint);
	attitude_setpoint.yaw_sp_move_rate = _yawspeed_sp;
}
