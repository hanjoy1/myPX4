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
 * @file fw_roll_controller.cpp
 * Implementation of a simple roll P controller.
 *
 * 控制含义：
 *   滚转姿态误差 -> 滚转欧拉角速度设定值 -> 机体系滚转角速度设定值
 *
 * 这里通过欧拉角速度到机体系角速度的雅可比关系扣除偏航欧拉角速度前馈，
 * 使飞机在协调转弯等存在俯仰/偏航运动时，机体系 p 轴角速度设定仍然一致。
 */

#include "fw_roll_controller.h"
#include <float.h>
#include <lib/geo/geo.h>
#include <mathlib/mathlib.h>

float RollController::control_roll(float roll_setpoint, float euler_yaw_rate_setpoint, float roll, float pitch)
{
	/* Do not calculate control signal with bad inputs */
	// 任一输入非法时保持上一次输出，避免 NaN 传播到下游角速度控制器。
	if (!(PX4_ISFINITE(roll_setpoint) &&
	      PX4_ISFINITE(euler_yaw_rate_setpoint) &&
	      PX4_ISFINITE(pitch) &&
	      PX4_ISFINITE(roll))) {

		return _body_rate_setpoint;
	}

	const float roll_error = roll_setpoint - roll;
	/* 一阶姿态响应：时间常数越小，同样姿态误差产生的角速度需求越大。 */
	_euler_rate_setpoint = roll_error / _tc;

	/* Transform setpoint to body angular rates (jacobian) */
	// 欧拉滚转角速度需扣除俯仰角与偏航角速度耦合项，得到机体系 p 轴角速度。
	const float roll_body_rate_setpoint_raw = _euler_rate_setpoint - sinf(pitch) *
			euler_yaw_rate_setpoint;

	// 最终输出受 FW_R_RMAX 限制，避免姿态外环给内环过大的角速度需求。
	_body_rate_setpoint = math::constrain(roll_body_rate_setpoint_raw, -_max_rate, _max_rate);

	return _body_rate_setpoint;
}
