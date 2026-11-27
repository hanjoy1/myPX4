/****************************************************************************
 *
 *   Copyright (c) 2020-2022 PX4 Development Team. All rights reserved.
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
 * @file fw_yaw_controller.cpp
 * Implementation of a simple coordinated turn yaw controller.
 *
 * 控制含义：
 *   滚转/俯仰/空速 -> 协调转弯偏航角速度设定值 -> 机体系偏航角速度
 *
 * 固定翼通常不把偏航控制到某个绝对角度，而是计算零侧滑协调转弯需要的偏航角速度，
 * 再把欧拉偏航角速度映射到机体系 r 轴角速度。
 */

#include "fw_yaw_controller.h"
#include <float.h>
#include <lib/geo/geo.h>
#include <mathlib/mathlib.h>

float YawController::control_yaw(float roll_setpoint, float euler_pitch_rate_setpoint, float roll, float pitch,
				 float airspeed)
{
	/* Do not calculate control signal with bad inputs */
	// 任一输入非法时保持上一次输出，避免协调转弯公式产生 NaN。
	if (!(PX4_ISFINITE(roll_setpoint) &&
	      PX4_ISFINITE(roll) &&
	      PX4_ISFINITE(pitch) &&
	      PX4_ISFINITE(euler_pitch_rate_setpoint) &&
	      PX4_ISFINITE(airspeed))) {

		return _body_rate_setpoint;
	}

	float constrained_roll;
	bool inverted = false;

	/* roll is used as feedforward term and inverted flight needs to be considered */
	// 滚转角用于协调转弯前馈；接近 90 度时 tan(roll) 会发散，所以需要先限幅。
	if (fabsf(roll) < math::radians(90.f)) {
		/* not inverted, but numerically still potentially close to infinity */
		// 非倒飞时限制到 +/-80 度，保留大坡度能力但避开 tan() 奇异区。
		constrained_roll = math::constrain(roll, math::radians(-80.f), math::radians(80.f));

	} else {
		// 倒飞时不更新协调转弯 yaw rate，保持上一拍输出，避免使用不适用的正飞公式。
		inverted = true;

		// inverted flight, constrain on the two extremes of -pi..+pi to avoid infinity
		//note: the ranges are extended by 10 deg here to avoid numeric resolution effects
		if (roll > 0.f) {
			/* right hemisphere */
			constrained_roll = math::constrain(roll, math::radians(100.f), math::radians(180.f));

		} else {
			/* left hemisphere */
			constrained_roll = math::constrain(roll, math::radians(-180.f), math::radians(-100.f));
		}
	}

	constrained_roll = math::constrain(constrained_roll, -fabsf(roll_setpoint), fabsf(roll_setpoint));


	if (!inverted) {
		/*
		 * 协调转弯关系：
		 *   yaw_rate = tan(bank_angle) * g / airspeed
		 * 其中 cos(pitch) 用来修正水平分量。
		 */
		/* Calculate desired yaw rate from coordinated turn constraint / (no side forces) */
		_euler_rate_setpoint = tanf(constrained_roll) * cosf(pitch) * CONSTANTS_ONE_G / airspeed;

		/* Transform setpoint to body angular rates (jacobian) */
		// 将欧拉偏航角速度转换到机体系 r 轴，同时补偿滚转和俯仰角速度耦合。
		const float yaw_body_rate_setpoint_raw = -sinf(roll) * euler_pitch_rate_setpoint +
				cosf(roll) * cosf(pitch) * _euler_rate_setpoint;

		// 偏航角速度输出受 FW_Y_RMAX 限制。
		_body_rate_setpoint = math::constrain(yaw_body_rate_setpoint_raw, -_max_rate, _max_rate);
	}

	if (!PX4_ISFINITE(_body_rate_setpoint)) {
		// 最后一道保护：如果历史输出也已经非法，则强制归零。
		_body_rate_setpoint = 0.f;
	}

	return _body_rate_setpoint;
}
