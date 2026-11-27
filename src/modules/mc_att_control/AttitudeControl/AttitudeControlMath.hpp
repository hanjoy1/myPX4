/****************************************************************************
 *
 *   Copyright (C) 2023 PX4 Development Team. All rights reserved.
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
 * @file AttitudeControlMath.hpp
 *
 * 中文阅读提示：
 * 这里放姿态控制里一些几何辅助函数。目前主要用于 VTOL 场景：
 * 当 yaw setpoint 和当前 yaw 差别很大时，修正手动 roll/pitch 倾角 setpoint，
 * 让用户摇杆方向仍然符合“以当前机头方向为参考”的直觉。
 */

#pragma once

#include <matrix/matrix/math.hpp>

namespace AttitudeControlMath
{
/**
 * Rotate a tilt quaternion (without yaw rotation) such that when rotated by a yaw setpoint
 * the resulting tilt is the same as if it was rotated by the current yaw of the vehicle
 * 旋转一个不含 yaw 的倾角四元数，使它随后叠加 yaw setpoint 后的倾斜方向，
 * 等效于用当前飞机 yaw 叠加时的倾斜方向。
 *
 * 换句话说：当当前 yaw 和期望 yaw 差很大时，修正 roll/pitch 倾角，
 * 避免用户打前杆却因为 yaw setpoint 坐标系不同而表现成侧向倾斜。
 *
 * @param q_sp_tilt pure tilt quaternion (yaw = 0) that needs to be corrected
 * @param q_sp_tilt 需要修正的纯倾角四元数，yaw = 0，会被原地修改
 * @param q_att current attitude of the vehicle
 * @param q_att 当前飞机姿态
 * @param q_sp_yaw pure yaw quaternion of the desired yaw setpoint
 * @param q_sp_yaw 期望 yaw setpoint 对应的纯 yaw 四元数
 */
void inline correctTiltSetpointForYawError(matrix::Quatf &q_sp_tilt, const matrix::Quatf &q_att,
		const matrix::Quatf &q_sp_yaw)
{
	const matrix::Vector3f z_unit(0.f, 0.f, 1.f);

	// Extract yaw from the current attitude
	// 先从当前姿态中拆出“倾角部分”和“剩余 yaw 部分”。
	const matrix::Vector3f att_z = q_att.dcm_z();
	const matrix::Quatf q_tilt(z_unit, att_z);
	const matrix::Quatf q_yaw = q_tilt.inversed() * q_att; // This is not euler yaw
	// q_yaw 是当前姿态去掉倾角后的剩余旋转，不完全等同于欧拉 yaw，但可用于这里的几何补偿。

	// Find the quaternion that creates a tilt aligned with the body frame
	// when rotated by the yaw setpoint
	// To do so, solve q_yaw * q_tilt_ne = q_sp_yaw * q_sp_rp_compensated
	// 求一个补偿后的 q_sp_rp_compensated，使它叠加 q_sp_yaw 后，
	// 和“当前 yaw 下的用户倾角意图”对齐。
	const matrix::Quatf q_sp_rp_compensated = q_sp_yaw.inversed() * q_yaw * q_sp_tilt;

	// Extract the corrected tilt
	// 从补偿后的姿态中重新提取纯倾角部分，写回 q_sp_tilt。
	const matrix::Vector3f att_sp_z = q_sp_rp_compensated.dcm_z();
	q_sp_tilt = matrix::Quatf(z_unit, att_sp_z);
}
}
