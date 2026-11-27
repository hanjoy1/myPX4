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
 * @file fw_pitch_controller.cpp
 * Implementation of a simple pitch P controller.
 *
 * 控制含义：
 *   俯仰姿态误差 -> 俯仰欧拉角速度设定值 -> 机体系俯仰角速度设定值
 *
 * 欧拉角速度到机体系角速度的换算需要当前滚转角和俯仰角，因为飞机不在水平姿态时，
 * 期望欧拉俯仰角速度并不等于机体系 q 轴角速度。
 */

#include "fw_pitch_controller.h"
#include <float.h>
#include <lib/geo/geo.h>
#include <mathlib/mathlib.h>

float PitchController::control_pitch(float pitch_setpoint, float euler_yaw_rate_setpoint, float roll, float pitch)
{
	/* Do not calculate control signal with bad inputs */
	// 任一输入非法时保持上一次输出，避免 NaN 传播到下游角速度控制器。
	if (!(PX4_ISFINITE(pitch_setpoint) &&
	      PX4_ISFINITE(euler_yaw_rate_setpoint) &&
	      PX4_ISFINITE(pitch) &&
	      PX4_ISFINITE(roll))) {

		return _body_rate_setpoint;
	}

	const float pitch_error = pitch_setpoint - pitch;
	/* 一阶姿态响应：俯仰误差按 FW_P_TC 时间常数闭合。 */
	_euler_rate_setpoint = pitch_error / _tc;

	/* Transform setpoint to body angular rates (jacobian) */
	// 欧拉俯仰角速度通过当前滚转/俯仰姿态映射到机体系 q 轴角速度。
	const float pitch_body_rate_setpoint_raw = cosf(roll) * _euler_rate_setpoint +
			cosf(pitch) * sinf(roll) * euler_yaw_rate_setpoint;

	// 俯仰上拉和下压可以配置不同最大角速度，因此使用正/负两个限幅参数。
	_body_rate_setpoint = math::constrain(pitch_body_rate_setpoint_raw, -_max_rate_neg, _max_rate_pos);

	return _body_rate_setpoint;
}
