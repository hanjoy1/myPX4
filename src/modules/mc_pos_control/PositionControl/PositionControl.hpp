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
 * @file PositionControl.hpp
 *
 * A cascaded position controller for position/velocity control only.
 *
 * 中文阅读提示：
 * 这个类是 mc_pos_control 的核心控制器，只做“位置/速度 -> 推力矢量”的计算。
 * PX4 本地坐标系是 NED：x 北、y 东、z 向下，因此“上升速度/加速度”通常是负 z。
 */

#pragma once

#include <lib/mathlib/mathlib.h>
#include <matrix/matrix/math.hpp>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>

struct PositionControlStates {
	matrix::Vector3f position;     ///< 当前本地位置，NED 坐标 [m]
	matrix::Vector3f velocity;     ///< 当前本地速度，NED 坐标 [m/s]
	matrix::Vector3f acceleration; ///< 当前速度导数/加速度估计，NED 坐标 [m/s^2]
	float yaw;                     ///< 当前航向角 [rad]
};

/**
 * 	Core Position-Control for MC.
 * 	This class contains P-controller for position and
 * 	PID-controller for velocity.
 * 	Inputs:
 * 		vehicle position/velocity/yaw
 * 		desired set-point position/velocity/thrust/yaw/yaw-speed
 * 		constraints that are stricter than global limits
 * 	Output
 * 		thrust vector and a yaw-setpoint
 *
 * 	If there is a position and a velocity set-point present, then
 * 	the velocity set-point is used as feed-forward. If feed-forward is
 * 	active, then the velocity component of the P-controller output has
 * 	priority over the feed-forward component.
 *
 * 	A setpoint that is NAN is considered as not set.
 * 	If there is a position/velocity- and thrust-setpoint present, then
 *  the thrust-setpoint is ommitted and recomputed from position-velocity-PID-loop.
 *
 * 中文阅读提示：
 * - 控制结构是串级：位置 P 环先把位置误差变成速度设定值，然后速度 PID 环把速度误差变成加速度/推力。
 * - trajectory_setpoint 中某一轴为 NaN 表示该轴不由这一层控制；例如只有速度设定值时就是速度控制。
 * - 如果同时给了位置和速度，速度被当作前馈；如果超过速度限制，位置误差产生的速度优先。
 * - 输出的 _thr_sp 是归一化推力矢量，随后会被 ControlMath 转成 attitude_setpoint。
 */
class PositionControl
{
public:

	PositionControl() = default;
	~PositionControl() = default;

	/**
	 * Set the position control gains
	 * 设置位置环控制增益。
	 * @param P 3D vector of proportional gains for x,y,z axis
	 * @param P x/y/z 三个轴的位置比例增益
	 */
	void setPositionGains(const matrix::Vector3f &P) { _gain_pos_p = P; }

	/**
	 * Set the velocity control gains
	 * 设置速度环 PID 控制增益。
	 * @param P 3D vector of proportional gains for x,y,z axis
	 * @param P x/y/z 三个轴的速度比例增益
	 * @param I 3D vector of integral gains
	 * @param I x/y/z 三个轴的速度积分增益
	 * @param D 3D vector of derivative gains
	 * @param D x/y/z 三个轴的速度微分增益
	 */
	void setVelocityGains(const matrix::Vector3f &P, const matrix::Vector3f &I, const matrix::Vector3f &D);

	/**
	 * Set the maximum velocity to execute with feed forward and position control
	 * 设置位置控制和速度前馈共同执行时的最大速度限制。
	 * @param vel_horizontal horizontal velocity limit
	 * @param vel_horizontal 水平速度上限 [m/s]
	 * @param vel_up upwards velocity limit
	 * @param vel_up 上升速度上限 [m/s]，NED 中对应负 z 方向
	 * @param vel_down downwards velocity limit
	 * @param vel_down 下降速度上限 [m/s]，NED 中对应正 z 方向
	 */
	void setVelocityLimits(const float vel_horizontal, const float vel_up, float vel_down);

	/**
	 * Set the minimum and maximum collective normalized thrust [0,1] that can be output by the controller
	 * 设置控制器允许输出的最小/最大归一化总推力。
	 * @param min minimum thrust e.g. 0.1 or 0
	 * @param min 最小推力，例如 0 或 0.1
	 * @param max maximum thrust e.g. 0.9 or 1
	 * @param max 最大推力，例如 0.9 或 1
	 */
	void setThrustLimits(const float min, const float max);

	/**
	 * Set margin that is kept for horizontal control when prioritizing vertical thrust
	 * 设置垂向推力优先时仍为水平控制保留的推力余量。
	 * @param margin of normalized thrust that is kept for horizontal control e.g. 0.3
	 * @param margin 保留给水平控制的归一化推力余量，例如 0.3
	 */
	void setHorizontalThrustMargin(const float margin);

	/**
	 * Set the maximum tilt angle in radians the output attitude is allowed to have
	 * 设置输出姿态允许的最大倾角。
	 * @param tilt angle in radians from level orientation
	 * @param tilt 相对水平姿态的最大倾角 [rad]
	 */
	void setTiltLimit(const float tilt) { _lim_tilt = tilt; }

	/**
	 * Set the normalized hover thrust
	 * 设置归一化悬停推力。
	 * @param hover_thrust [HOVER_THRUST_MIN, HOVER_THRUST_MAX] with which the vehicle hovers not accelerating down or up with level orientation
	 * @param hover_thrust 飞机水平姿态下不升不降所需推力，会被限制在有效范围内
	 */
	void setHoverThrust(const float hover_thrust) { _hover_thrust = math::constrain(hover_thrust, HOVER_THRUST_MIN, HOVER_THRUST_MAX); }

	/**
	 * Update the hover thrust without immediately affecting the output
	 * by adjusting the integrator. This prevents propagating the dynamics
	 * of the hover thrust signal directly to the output of the controller.
	 * 更新悬停推力估计，同时通过调整积分项避免控制输出立刻跳变。
	 * 这样 hover thrust estimator 的动态变化不会直接传到推力输出。
	 * @param hover_thrust_new 新的归一化悬停推力估计值
	 */
	void updateHoverThrust(const float hover_thrust_new);

	/**
	 * Pass the current vehicle state to the controller
	 * 把当前飞机状态传入控制器。
	 * @param PositionControlStates structure
	 * @param states 包含位置、速度、加速度估计和航向的状态结构体
	 */
	void setState(const PositionControlStates &states);

	/**
	 * Pass the desired setpoints
	 * Note: NAN value means no feed forward/leave state uncontrolled if there's no higher order setpoint.
	 * 传入期望设定值。
	 * 注意：NaN 表示该项不作为前馈；如果没有更高阶设定值，对应状态将不被控制。
	 * @param setpoint setpoints including feed-forwards to execute in update()
	 * @param setpoint update() 中要执行的位置/速度/加速度/yaw/yawspeed 设定值
	 */
	void setInputSetpoint(const trajectory_setpoint_s &setpoint);

	/**
	 * Apply P-position and PID-velocity controller that updates the member
	 * thrust, yaw- and yawspeed-setpoints.
	 * 执行位置 P 环和速度 PID 环，更新内部推力、航向和偏航角速度设定值。
	 * @see _thr_sp
	 * @see _yaw_sp
	 * @see _yawspeed_sp
	 * @param dt time in seconds since last iteration
	 * @param dt 距离上一次控制循环的时间 [s]
	 * @return true if update succeeded and output setpoint is executable, false if not
	 * @return true 表示控制更新成功且输出设定值可执行；false 表示输入或输出无效
	 */
	bool update(const float dt);

	/**
	 * Set the integral term in xy to 0.
	 * 清空速度控制器积分项。
	 * @see _vel_int
	 */
	void resetIntegral() { _vel_int.setZero(); }
	void resetIntegralXY() { _vel_int.xy() = matrix::Vector2f(); } ///< 只清空水平 x/y 积分项，保留垂向积分

	/**
	 * If set, the tilt setpoint is computed by assuming no vertical acceleration
	 * 如果设置为 true，计算倾角时假设没有垂向加速度，减小垂向控制对水平倾角的影响。
	 * @param val true 表示解耦水平/垂向加速度；false 表示三维加速度共同决定推力方向
	 */
	void decoupleHorizontalAndVecticalAcceleration(bool val) { _decouple_horizontal_and_vertical_acceleration = val; }

	/**
	 * Get the controllers output local position setpoint
	 * These setpoints are the ones which were executed on including PID output and feed-forward.
	 * The acceleration or thrust setpoints can be used for attitude control.
	 * 获取控制器实际执行后的本地位置设定值。
	 * 这些设定值包含输入前馈和 PID 修正；其中加速度/推力可用于姿态控制。
	 * @param local_position_setpoint reference to struct to fill up
	 * @param local_position_setpoint 待填充的 vehicle_local_position_setpoint_s 结构体引用
	 */
	void getLocalPositionSetpoint(vehicle_local_position_setpoint_s &local_position_setpoint) const;

	/**
	 * Get the controllers output attitude setpoint
	 * This attitude setpoint was generated from the resulting acceleration setpoint after position and velocity control.
	 * It needs to be executed by the attitude controller to achieve velocity and position tracking.
	 * 获取控制器输出的姿态设定值。
	 * 该姿态由位置/速度控制后的加速度/推力设定值生成，需要由姿态控制器执行以实现位置/速度跟踪。
	 * @param attitude_setpoint reference to struct to fill up
	 * @param attitude_setpoint 待填充的 vehicle_attitude_setpoint_s 结构体引用
	 */
	void getAttitudeSetpoint(vehicle_attitude_setpoint_s &attitude_setpoint) const;

	/**
	 * All setpoints are set to NAN (uncontrolled). Timestampt zero.
	 * 空轨迹设定值：所有设定值为 NaN，表示不控制；时间戳为 0。
	 */
	static const trajectory_setpoint_s empty_trajectory_setpoint;

private:
	// The range limits of the hover thrust configuration/estimate
	// 悬停推力配置/估计允许的范围限制
	static constexpr float HOVER_THRUST_MIN = 0.05f;
	static constexpr float HOVER_THRUST_MAX = 0.9f;

	bool _inputValid(); ///< 检查每个受控轴是否同时具备有效设定值和对应状态估计

	void _positionControl(); ///< 位置 P 环：位置误差 -> 速度设定值，并施加速度限制
	void _velocityControl(const float dt); ///< 速度 PID 环：速度误差 -> 加速度设定值 -> 推力设定值
	void _accelerationControl(); ///< 加速度到推力矢量的转换，同时施加倾角限制

	// Gains
	matrix::Vector3f _gain_pos_p; ///< 位置环 P 增益，对应 x/y/z
	matrix::Vector3f _gain_vel_p; ///< 速度环 P 增益，对应 x/y/z
	matrix::Vector3f _gain_vel_i; ///< 速度环 I 增益，对应 x/y/z
	matrix::Vector3f _gain_vel_d; ///< 速度环 D 增益，对应 x/y/z

	// Limits
	float _lim_vel_horizontal{}; ///< 水平速度限制 [m/s]
	float _lim_vel_up{}; ///< 上升速度限制 [m/s]；NED 中上升对应负 z 速度
	float _lim_vel_down{}; ///< 下降速度限制 [m/s]；NED 中下降对应正 z 速度
	float _lim_thr_min{}; ///< 允许输出的最小归一化推力
	float _lim_thr_max{}; ///< 允许输出的最大归一化推力
	float _lim_thr_xy_margin{}; ///< 优先保证垂向推力时，为水平控制保留的推力余量
	float _lim_tilt{}; ///< 输出姿态允许的最大倾角 [rad]

	float _hover_thrust{}; ///< 悬停所需归一化推力；用于把期望加速度换算成推力
	bool _decouple_horizontal_and_vertical_acceleration{true}; ///< 计算倾角时是否忽略垂向加速度，减小垂向前馈对水平倾角的耦合

	// States
	matrix::Vector3f _pos; /**< 当前本地位置 */
	matrix::Vector3f _vel; /**< 当前本地速度 */
	matrix::Vector3f _vel_dot; /**< 速度导数，作为加速度估计使用 */
	matrix::Vector3f _vel_int; /**< 速度控制器积分项，单位等价于加速度 */
	float _yaw{}; /**< 当前航向角 */

	// Setpoints
	matrix::Vector3f _pos_sp; /**< 期望位置；元素可为 NaN 表示该轴不控制位置 */
	matrix::Vector3f _vel_sp; /**< 期望速度；可由外部给定，也可由位置环生成 */
	matrix::Vector3f _acc_sp; /**< 期望加速度；可作为前馈，也可由速度环生成 */
	matrix::Vector3f _thr_sp; /**< 期望归一化推力矢量，后续转换为姿态设定值 */
	float _yaw_sp{}; /**< 期望航向角 */
	float _yawspeed_sp{}; /** 期望偏航角速度 */
};
