/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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
 * @file GotoControl.cpp
 */

#include "GotoControl.hpp"

#include <drivers/drv_hrt.h>
#include <float.h>
#include <lib/mathlib/mathlib.h>

using namespace time_literals;

// 检查 goto_setpoint 是否启用、是否有时间戳、是否未超时。
bool GotoControl::checkForSetpoint(const hrt_abstime &now, const bool enabled)
{
	_goto_setpoint_sub.update(&_goto_setpoint);

	if (!enabled) {
		// Flag the setpoint as invalid if disabled, so if it is enabled in near future,
		// we don't use an old setpoint
		// 禁用时清掉时间戳，防止稍后重新启用时追踪过期 goto 目标。
		_goto_setpoint.timestamp = 0;
	}

	const bool timestamp_initialized = _goto_setpoint.timestamp != 0;
	const bool no_timeout = now < (_goto_setpoint.timestamp + 500_ms);
	const bool need_to_run = timestamp_initialized && no_timeout && enabled;

	if (!need_to_run) {
		// 下次重新收到有效 goto 时，要把平滑器 reset 到当前飞机状态。
		_is_initialized = false;
	}

	return need_to_run;
}

// 使用当前位置/航向和 goto 目标更新平滑器，并发布 trajectory_setpoint。
void GotoControl::update(const float dt, const matrix::Vector3f &position, const float heading)
{
	if (!_is_initialized) {
		// 第一次运行或中断后恢复时，从当前状态开始平滑，避免轨迹跳变。
		resetPositionSmoother(position);
		resetHeadingSmoother(heading);
		_is_initialized = true;
	}

	const Vector3f position_setpoint(_goto_setpoint.position);

	if (!position_setpoint.isAllFinite()) {
		// TODO: error messaging
		// TODO：这里后续应增加错误提示/事件上报。
		// goto 目标必须是完整三维位置；无效时等待下一次有效状态再重置平滑器。
		_need_smoother_reset = true;
		return;
	}

	if (!position.isAllFinite()) {
		// TODO: error messaging
		// TODO：这里后续应增加当前位置无效的错误提示/事件上报。
		_need_smoother_reset = true;
		return;
	}

	if (_need_smoother_reset) {
		resetPositionSmoother(position);
	}

	setPositionSmootherLimits(_goto_setpoint);

	const Vector3f feedforward_velocity{};
	const bool force_zero_velocity_setpoint = false;
	PositionSmoothing::PositionSmoothingSetpoints out_setpoints;
	// 根据当前位置、目标位置和速度/加速度/jerk 限制，生成可跟踪的平滑轨迹点。
	_position_smoothing.generateSetpoints(position, position_setpoint, feedforward_velocity, dt,
					      force_zero_velocity_setpoint, out_setpoints);

	trajectory_setpoint_s trajectory_setpoint{};
	out_setpoints.position.copyTo(trajectory_setpoint.position);
	out_setpoints.velocity.copyTo(trajectory_setpoint.velocity);
	out_setpoints.acceleration.copyTo(trajectory_setpoint.acceleration);
	out_setpoints.jerk.copyTo(trajectory_setpoint.jerk);

	if (_goto_setpoint.flag_control_heading && PX4_ISFINITE(_goto_setpoint.heading) && PX4_ISFINITE(heading)) {
		if (!_controlling_heading || _need_smoother_reset) {
			// 刚开始控制航向时，从当前航向开始，避免 yaw setpoint 突跳。
			resetHeadingSmoother(heading);
		}

		setHeadingSmootherLimits(_goto_setpoint);
		// 航向单独用 HeadingSmoothing 生成 yaw/yawspeed，和位置轨迹一起发布。
		_heading_smoothing.update(_goto_setpoint.heading, dt);

		trajectory_setpoint.yaw = _heading_smoothing.getSmoothedHeading();
		trajectory_setpoint.yawspeed = _heading_smoothing.getSmoothedHeadingRate();

		_controlling_heading = true;

	} else {
		trajectory_setpoint.yaw = NAN;
		trajectory_setpoint.yawspeed = NAN;

		_controlling_heading = false;
	}

	_need_smoother_reset = false;

	trajectory_setpoint.timestamp = hrt_absolute_time();
	// 发布后会被 MulticopterPositionControl 当作普通 trajectory_setpoint 订阅和跟踪。
	_trajectory_setpoint_pub.publish(trajectory_setpoint);

	vehicle_constraints_s vehicle_constraints{
		.timestamp = trajectory_setpoint.timestamp,
		.speed_up = NAN,
		.speed_down = NAN,
		.want_takeoff = false
	};
	_vehicle_constraints_pub.publish(vehicle_constraints);
}

// 将位置平滑器重置到当前位置，初始速度和加速度都为 0。
void GotoControl::resetPositionSmoother(const matrix::Vector3f &position)
{
	if (!position.isAllFinite()) {
		// TODO: error messaging
		// TODO：这里后续应增加当前位置无效的错误提示/事件上报。
		_need_smoother_reset = true;
		return;
	}

	const Vector3f initial_acceleration{};
	const Vector3f initial_velocity{};
	_position_smoothing.reset(initial_acceleration, initial_velocity, position);

	_need_smoother_reset = false;
}

// 将航向平滑器重置到当前航向，初始航向角速度为 0。
void GotoControl::resetHeadingSmoother(const float heading)
{
	if (!PX4_ISFINITE(heading)) {
		// TODO: error messaging
		// TODO：这里后续应增加航向无效的错误提示/事件上报。
		_controlling_heading = false;
		return;
	}

	const float initial_heading_rate{0.f};
	_heading_smoothing.reset(heading, initial_heading_rate);
}

// 根据 goto_setpoint 中可选速度限制，设置位置平滑器的水平/垂向速度和加速度约束。
void GotoControl::setPositionSmootherLimits(const goto_setpoint_s &goto_setpoint)
{
	// Horizontal constraints
	// 水平方向约束。
	// 默认采用 MPC 参数；goto_setpoint 可以请求更低的速度上限，但不能超过车辆全局限制。
	float max_horizontal_speed = _param_mpc_xy_cruise;
	float max_horizontal_accel = _param_mpc_acc_hor;

	if (goto_setpoint.flag_set_max_horizontal_speed
	    && PX4_ISFINITE(goto_setpoint.max_horizontal_speed)) {
		max_horizontal_speed = math::constrain(goto_setpoint.max_horizontal_speed, 0.f,
						       _param_mpc_xy_cruise);

		// linearly scale horizontal acceleration limit with horizontal speed limit to maintain smoothing dynamic
		// only limit acceleration once within velocity constraints
		// 按水平速度上限线性缩放水平加速度上限，以保持平滑器动态特性。
		// 只有当前速度已经进入速度限制范围内时才收紧加速度。
		// 速度上限降低时同步降低加速度上限，使平滑轨迹的动态比例保持接近。
		if (!_position_smoothing.getCurrentVelocityXY().longerThan(max_horizontal_speed)) {
			const float speed_scale = max_horizontal_speed / _param_mpc_xy_cruise;
			max_horizontal_accel = math::constrain(_param_mpc_acc_hor * speed_scale, 0.f, _param_mpc_acc_hor);
		}
	}

	_position_smoothing.setCruiseSpeed(max_horizontal_speed);
	_position_smoothing.setMaxAccelerationXY(max_horizontal_accel);

	// Vertical constraints
	// 垂向约束。
	// NED 坐标中目标 z 更小表示飞得更高，因此上升/下降使用不同速度和加速度限制。
	float vehicle_max_vertical_speed = _param_mpc_z_v_auto_dn;
	float vehicle_max_vertical_accel = _param_mpc_acc_down_max;

	if (goto_setpoint.position[2] < _position_smoothing.getCurrentPositionZ()) { // goto higher -> more negative
		vehicle_max_vertical_speed = _param_mpc_z_v_auto_up;
		vehicle_max_vertical_accel = _param_mpc_acc_up_max;
	}

	float max_vertical_speed = vehicle_max_vertical_speed;
	float max_vertical_accel = vehicle_max_vertical_accel;

	if (goto_setpoint.flag_set_max_vertical_speed && PX4_ISFINITE(goto_setpoint.max_vertical_speed)) {
		max_vertical_speed = math::constrain(goto_setpoint.max_vertical_speed, 0.f, vehicle_max_vertical_speed);

		// linearly scale vertical acceleration limit with vertical speed limit to maintain smoothing dynamic
		// only limit acceleration once within velocity constraints
		// 按垂向速度上限线性缩放垂向加速度上限，以保持平滑器动态特性。
		// 只有当前垂向速度已经进入速度限制范围内时才收紧加速度。
		// 垂向也按速度上限比例缩放加速度上限。
		if (fabsf(_position_smoothing.getCurrentVelocityZ()) <= max_vertical_speed) {
			const float speed_scale = max_vertical_speed / vehicle_max_vertical_speed;
			max_vertical_accel = math::constrain(vehicle_max_vertical_accel * speed_scale, 0.f, vehicle_max_vertical_accel);
		}
	}

	_position_smoothing.setMaxVelocityZ(max_vertical_speed);
	_position_smoothing.setMaxAccelerationZ(max_vertical_accel);
}

// 根据 goto_setpoint 中可选航向角速度限制，设置航向平滑器的角速度和角加速度约束。
void GotoControl::setHeadingSmootherLimits(const goto_setpoint_s &goto_setpoint)
{
	// 航向默认采用自动 yaw rate/accel 参数；goto_setpoint 可进一步收紧。
	float max_heading_rate = _param_mpc_yawrauto_max;
	float max_heading_accel = _param_mpc_yawrauto_acc;

	if (goto_setpoint.flag_set_max_heading_rate && PX4_ISFINITE(goto_setpoint.max_heading_rate)) {
		max_heading_rate = math::constrain(goto_setpoint.max_heading_rate, 0.f, _param_mpc_yawrauto_max);

		// linearly scale heading acceleration limit with heading rate limit to maintain smoothing dynamic
		// only limit acceleration once within velocity constraints
		// 按航向角速度上限线性缩放航向角加速度上限，以保持平滑器动态特性。
		// 只有当前 yaw rate 已经进入限制范围内时才收紧 yaw acceleration。
		// yaw rate 上限降低时同步降低 yaw acceleration，避免航向平滑器过于激进。
		if (fabsf(_heading_smoothing.getSmoothedHeadingRate()) <= max_heading_rate) {
			const float rate_scale = max_heading_rate / _param_mpc_yawrauto_max;
			max_heading_accel = math::constrain(_param_mpc_yawrauto_acc * rate_scale, 0.f, _param_mpc_yawrauto_acc);
		}
	}

	_heading_smoothing.setMaxHeadingRate(max_heading_rate);
	_heading_smoothing.setMaxHeadingAccel(max_heading_accel);
}
