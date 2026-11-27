/****************************************************************************
 *
 *   Copyright (c) 2020-2023 PX4 Development Team. All rights reserved.
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
 * 3. Neither the name ECL nor the names of its contributors may be
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
 * @file fw_wheel_controller.cpp
 * Implementation of a simple PID wheel controller for heading tracking.
 *
 * 控制含义：
 *   跑道航向/偏航误差 -> 机体系偏航角速度设定值 -> 前轮转向输出
 *
 * 该控制器只用于跑道起飞/降落的地面滑跑阶段，和空中方向舵/偏航控制器相互独立。
 */

#include "fw_wheel_controller.h"
#include <float.h>
#include <lib/geo/geo.h>
#include <mathlib/mathlib.h>
#include <matrix/math.hpp>

using matrix::wrap_pi;

float WheelController::control_bodyrate(float dt, float body_z_rate, float groundspeed, float groundspeed_scaler)
{
	/* Do not calculate control signal with bad inputs */
	// 输入非法时返回上一拍输出的限幅值，保证前轮指令不会突然变成 NaN。
	if (!(PX4_ISFINITE(body_z_rate) &&
	      PX4_ISFINITE(groundspeed) &&
	      PX4_ISFINITE(groundspeed_scaler))) {

		return math::constrain(_last_output, -1.f, 1.f);
	}

	const float rate_error = _body_rate_setpoint - body_z_rate;
	// rate_error 是期望偏航角速度和实际机体系 z 轴角速度之差。

	if (_k_i > 0.f && groundspeed > 1.f) { // only start integrating when above 1m/s

		/*
		 * 前轮转向效能随地速变化。积分项使用与比例通道一致的地速缩放；
		 * 当飞机几乎静止时不积分，避免原地积累无意义的转向积分。
		 */
		float id = rate_error * dt * groundspeed_scaler * groundspeed_scaler;

		if (_last_output < -1.f) {
			/* only allow motion to center: increase value */
			// 输出已低于下限时，只允许积分朝回中方向变化，防止继续 windup。
			id = math::max(id, 0.f);

		} else if (_last_output > 1.f) {
			/* only allow motion to center: decrease value */
			// 输出已高于上限时，只允许积分朝回中方向变化。
			id = math::min(id, 0.f);
		}

		// 积分项受 FW_WR_IMAX 限制。
		_integrator = math::constrain(_integrator + id * _k_i, -_integrator_max, _integrator_max);
	}

	/* Apply PI rate controller and store non-limited output */
	// 保存未限幅输出，用它判断下一拍积分是否已经把控制量推到饱和区。
	_last_output = _body_rate_setpoint * _k_ff * groundspeed_scaler +
		       groundspeed_scaler * groundspeed_scaler * (rate_error * _k_p) + _integrator;

	// 前轮最终输出是归一化舵机命令，限制在 [-1, 1]。
	return math::constrain(_last_output, -1.f, 1.f);
}

float WheelController::control_attitude(float yaw_setpoint, float yaw)
{
	/* Do not calculate control signal with bad inputs */
	// 航向输入非法时保持上一拍 body rate setpoint。
	if (!(PX4_ISFINITE(yaw_setpoint) &&
	      PX4_ISFINITE(yaw))) {

		return _body_rate_setpoint;
	}

	const float yaw_error = wrap_pi(yaw_setpoint - yaw);

	/* 航向误差通过时间常数转换为偏航角速度指令。 */
	_body_rate_setpoint = yaw_error / _tc; // assume 0 pitch and roll angle, thus jacobian is simply identity matrix

	if (_max_rate > 0.01f) {
		// 如果配置了最大前轮偏航角速度，就对航向外环输出做限幅。
		if (_body_rate_setpoint > 0.f) {
			_body_rate_setpoint = (_body_rate_setpoint > _max_rate) ? _max_rate : _body_rate_setpoint;

		} else {
			_body_rate_setpoint = (_body_rate_setpoint < -_max_rate) ? -_max_rate : _body_rate_setpoint;
		}

	}

	return _body_rate_setpoint;
}
