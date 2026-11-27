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
 * @file AttitudeControl.hpp
 *
 * A quaternion based attitude controller.
 *
 * 中文阅读提示：
 * 这个类只做“姿态误差 -> 机体系角速度设定值”的计算。
 * 它不处理 uORB、不处理推力、不直接控制电机。
 *
 * 控制思想：
 * - 当前姿态 q 与期望姿态 qd 都用单位四元数表示。
 * - 先优先对齐机体 z 轴，也就是优先保证推力方向，对应 roll/pitch。
 * - yaw 误差可以用 yaw_weight 降权，因为多旋翼 yaw 控制能力通常弱于 roll/pitch。
 * - 最终四元数误差转换为 body rate setpoint，交给角速度控制器继续闭环。
 *
 * @author Matthias Grob	<maetugr@gmail.com>
 *
 * Publication documenting the implemented Quaternion Attitude Control:
 * Nonlinear Quadrocopter Attitude Control (2013)
 * by Dario Brescianini, Markus Hehn and Raffaello D'Andrea
 * Institute for Dynamic Systems and Control (IDSC), ETH Zurich
 *
 * https://www.research-collection.ethz.ch/bitstream/handle/20.500.11850/154099/eth-7387-01.pdf
 */

#pragma once

#include <matrix/matrix/math.hpp>
#include <mathlib/math/Limits.hpp>

class AttitudeControl
{
public:
	AttitudeControl() = default;
	~AttitudeControl() = default;

	/**
	 * Set proportional attitude control gain
	 * 设置姿态 P 环增益。
	 * @param proportional_gain 3D vector containing gains for roll, pitch, yaw
	 * @param proportional_gain roll/pitch/yaw 三轴比例增益，单位可理解为 rad 姿态误差对应 rad/s 角速度
	 * @param yaw_weight A fraction [0,1] deprioritizing yaw compared to roll and pitch
	 * @param yaw_weight yaw 权重 [0,1]，用于相对 roll/pitch 降低 yaw 优先级
	 */
	void setProportionalGain(const matrix::Vector3f &proportional_gain, const float yaw_weight);

	/**
	 * Set hard limit for output rate setpoints
	 * 设置输出角速度设定值的硬限幅。
	 * @param rate_limit [rad/s] 3D vector containing limits for roll, pitch, yaw
	 * @param rate_limit [rad/s] roll/pitch/yaw 三轴角速度上限
	 */
	void setRateLimit(const matrix::Vector3f &rate_limit) { _rate_limit = rate_limit; }

	/**
	 * Set a new attitude setpoint replacing the one tracked before
	 * 设置新的姿态设定值，替换之前跟踪的目标姿态。
	 * @param qd desired vehicle attitude setpoint
	 * @param qd 期望飞机姿态四元数
	 * @param yawspeed_setpoint [rad/s] yaw feed forward angular rate in world frame
	 * @param yawspeed_setpoint [rad/s] 世界坐标系 z 轴上的 yaw rate 前馈
	 */
	void setAttitudeSetpoint(const matrix::Quatf &qd, const float yawspeed_setpoint)
	{
		_attitude_setpoint_q = qd;
		_attitude_setpoint_q.normalize();
		_yawspeed_setpoint = yawspeed_setpoint;
	}

	/**
	 * Adjust last known attitude setpoint by a delta rotation
	 * Optional use to avoid glitches when attitude estimate reference e.g. heading changes.
	 * 用一个增量旋转修正最近一次姿态设定值。
	 * 典型用途：EKF 航向/姿态参考发生 reset 时，同步旋转旧 setpoint，避免控制输出突变。
	 * @param q_delta delta rotation to apply
	 * @param q_delta 要施加到旧姿态设定值上的增量旋转
	 */
	void adaptAttitudeSetpoint(const matrix::Quatf &q_delta)
	{
		_attitude_setpoint_q = q_delta * _attitude_setpoint_q;
		_attitude_setpoint_q.normalize();
	}

	/**
	 * Run one control loop cycle calculation
	 * 执行一次姿态 P 环计算。
	 * @param q estimation of the current vehicle attitude unit quaternion
	 * @param q 当前飞机姿态估计，单位四元数
	 * @return [rad/s] body frame 3D angular rate setpoint vector to be executed by the rate controller
	 * @return [rad/s] 机体系 roll/pitch/yaw 角速度设定值，交给 rate controller 执行
	 */
	matrix::Vector3f update(const matrix::Quatf &q) const;

private:
	matrix::Vector3f _proportional_gain; ///< 姿态 P 环增益，输出角速度设定值
	matrix::Vector3f _rate_limit; ///< 角速度设定值限幅
	float _yaw_w{0.f}; ///< yaw 权重 [0,1]，相对 roll/pitch 降低 yaw 优先级

	matrix::Quatf _attitude_setpoint_q; ///< 最近一次期望姿态，例如来自位置控制器
	float _yawspeed_setpoint{0.f}; ///< 最近一次 yaw rate 前馈设定值
};
