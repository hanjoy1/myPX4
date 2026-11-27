/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
 * @file AttitudeControl.cpp
 */

#include <AttitudeControl.hpp>

#include <mathlib/math/Functions.hpp>

using namespace matrix;

// 设置姿态 P 增益和 yaw 权重。yaw 权重会改变姿态误差分解中的 yaw 优先级。
void AttitudeControl::setProportionalGain(const matrix::Vector3f &proportional_gain, const float yaw_weight)
{
	_proportional_gain = proportional_gain;
	_yaw_w = math::constrain(yaw_weight, 0.f, 1.f);

	// compensate for the effect of the yaw weight rescaling the output
	// yaw_weight 会缩放 yaw 误差角，为了不改变小误差下的 yaw 闭环增益，这里反向补偿 yaw P 增益。
	if (_yaw_w > 1e-4f) {
		_proportional_gain(2) /= _yaw_w;
	}
}

// 执行四元数姿态 P 控制：当前姿态 q 和期望姿态 qd 的误差 -> body rate setpoint。
matrix::Vector3f AttitudeControl::update(const Quatf &q) const
{
	Quatf qd = _attitude_setpoint_q;

	// calculate reduced desired attitude neglecting vehicle's yaw to prioritize roll and pitch
	// 先计算 reduced desired attitude：忽略 yaw，只让当前机体 z 轴对齐期望机体 z 轴。
	// 机体 z 轴方向决定推力方向，因此这一步优先保证 roll/pitch/推力方向正确。
	const Vector3f e_z = q.dcm_z();
	const Vector3f e_z_d = qd.dcm_z();
	Quatf qd_red(e_z, e_z_d);

	if (fabsf(qd_red(1)) > (1.f - 1e-5f) || fabsf(qd_red(2)) > (1.f - 1e-5f)) {
		// In the infinitesimal corner case where the vehicle and thrust have the completely opposite direction,
		// full attitude control anyways generates no yaw input and directly takes the combination of
		// roll and pitch leading to the correct desired yaw. Ignoring this case would still be totally safe and stable.
		// 极端情况：当前推力方向和期望推力方向几乎完全相反，reduced attitude 退化。
		// 这时直接使用完整期望姿态 qd，仍然是安全稳定的。
		qd_red = qd;

	} else {
		// Transform rotation from current to desired thrust vector into a world frame reduced desired attitude.
		// This is a right multiplication as the tilt error quaternion is obtained from two Z vectors expressed in the world frame.
		// qd_red 当前只是“从当前 z 轴转到期望 z 轴”的旋转；右乘当前姿态 q 后，
		// 得到世界坐标系下的 reduced desired attitude。
		qd_red *= q;
	}

	// With a full desired attitude given by: qd = qd_red * qd_dyaw, extract the delta yaw component.
	// By definition, the delta yaw quaternion has the form (cos(angle/2), 0, 0, sin(angle/2))
	// 完整期望姿态可分解为 reduced attitude 加上一个剩余 yaw 旋转：
	// qd = qd_red * qd_dyaw，因此 qd_dyaw = inverse(qd_red) * qd。
	Quatf qd_dyaw = qd_red.inversed() * qd;
	qd_dyaw.canonicalize();
	// catch numerical problems with the domain of acosf and asinf
	// 限制 acos/asinf 输入范围，避免浮点误差导致 NaN。
	qd_dyaw(0) = math::constrain(qd_dyaw(0), -1.f, 1.f);
	qd_dyaw(3) = math::constrain(qd_dyaw(3), -1.f, 1.f);

	// scale the delta yaw angle and re-combine the desired attitude
	// 按 yaw_weight 缩放剩余 yaw 角，再和 reduced attitude 重新组合。
	// yaw_weight 越小，越优先 roll/pitch 推力方向，越不急着纠正 yaw。
	qd = qd_red * Quatf(cosf(_yaw_w * acosf(qd_dyaw(0))), 0.f, 0.f, sinf(_yaw_w * asinf(qd_dyaw(3))));

	// quaternion attitude control law, qe is rotation from q to qd
	// 姿态误差四元数 qe 表示从当前姿态 q 旋转到期望姿态 qd。
	const Quatf qe = q.inversed() * qd;

	// using sin(alpha/2) scaled rotation axis as attitude error (see quaternion definition by axis angle)
	// also taking care of the antipodal unit quaternion ambiguity
	// 使用四元数虚部作为姿态误差向量：2 * sin(alpha/2) * axis。
	// canonical() 处理 q 和 -q 表示同一姿态的二义性，保证选取较短旋转方向。
	const Vector3f eq = 2.f * qe.canonical().imag();

	// calculate angular rates setpoint
	// 姿态误差乘 P 增益，得到机体系角速度设定值。
	Vector3f rate_setpoint = eq.emult(_proportional_gain);

	// Feed forward the yaw setpoint rate.
	// yawspeed_setpoint is the feed forward commanded rotation around the world z-axis,
	// but we need to apply it in the body frame (because _rates_sp is expressed in the body frame).
	// Therefore we infer the world z-axis (expressed in the body frame) by taking the last column of R.transposed (== q.inversed)
	// and multiply it by the yaw setpoint rate (yawspeed_setpoint).
	// This yields a vector representing the commanded rotatation around the world z-axis expressed in the body frame
	// such that it can be added to the rates setpoint.
	// yaw rate 前馈定义在世界 z 轴上，但输出 rates_sp 是机体系。
	// 因此要把世界 z 轴转换到机体系，再乘以 yaw rate 前馈并加到 body rates setpoint。
	if (std::isfinite(_yawspeed_setpoint)) {
		rate_setpoint += q.inversed().dcm_z() * _yawspeed_setpoint;
	}

	// limit rates
	// 对 roll/pitch/yaw 角速度设定值逐轴限幅，避免超出 rate controller 或机体能力。
	for (int i = 0; i < 3; i++) {
		rate_setpoint(i) = math::constrain(rate_setpoint(i), -_rate_limit(i), _rate_limit(i));
	}

	return rate_setpoint;
}
