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
 * @file GotoControl.hpp
 *
 * A class which smooths position and heading references from "go-to" setpoints
 * for planar multicopters.
 *
 * Be sure to set constraints with setGotoConstraints() before calling the update() method for the first time
 *
 * 中文阅读提示：
 * GotoControl 是 setpoint 生成/平滑层，不做 PID。它把 goto_setpoint 中的目标位置和航向
 * 变成平滑的 trajectory_setpoint，再由 MulticopterPositionControl/PositionControl 跟踪。
 */

#pragma once

#include <lib/motion_planning/HeadingSmoothing.hpp>
#include <lib/motion_planning/PositionSmoothing.hpp>
#include <mathlib/math/Limits.hpp>
#include <matrix/matrix/math.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/goto_setpoint.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_constraints.h>

class GotoControl
{
public:
	GotoControl() = default;
	~GotoControl() = default;

	/**
	 * 检查当前是否有可用且未超时的 goto_setpoint。
	 * @param now 当前时间戳
	 * @param enabled 是否允许使用 goto_setpoint
	 * @return true 表示本周期需要运行 goto 平滑器并发布 trajectory_setpoint
	 */
	bool checkForSetpoint(const hrt_abstime &now, const bool enabled);

	/**
	 * @brief resets the position smoother at the current position with zero velocity and acceleration.
	 * @brief 将位置平滑器重置到当前位置，初始速度和加速度都置零。
	 *
	 * @param position [m] (NED) local vehicle position
	 * @param position [m] NED 本地坐标系下的当前飞机位置
	 */
	void resetPositionSmoother(const matrix::Vector3f &position);

	/**
	 * @brief resets the heading smoother at the current heading with zero heading rate and acceleration.
	 * @brief 将航向平滑器重置到当前航向，初始航向角速度和角加速度都置零。
	 *
	 * @param heading [rad] (from North) vehicle heading
	 * @param heading [rad] 当前飞机航向角，从北向开始计
	 */
	void resetHeadingSmoother(const float heading);

	/**
	 * @brief updates the smoothers with the current setpoints and outputs the "trajectory setpoint" for lower level
	 * loops to track.
	 * @brief 使用当前 goto 目标更新位置/航向平滑器，并输出供下层控制环跟踪的 trajectory_setpoint。
	 *
	 * @param[in] dt [s] time since last control update
	 * @param[in] dt [s] 距离上一次控制更新的时间
	 * @param[in] position [m] (NED) local vehicle position
	 * @param[in] position [m] NED 本地坐标系下的当前飞机位置
	 * @param[in] heading [rad] (from North) vehicle heading
	 * @param[in] heading [rad] 当前飞机航向角，从北向开始计
	 * @note 当前实现直接使用内部订阅到的 _goto_setpoint，并通过 uORB 发布 trajectory_setpoint。
	 */
	void update(const float dt, const matrix::Vector3f &position, const float heading);

	// Setting all parameters from the outside saves 300bytes flash
	// 从外部设置参数可节省约 300 bytes flash；中文：避免 GotoControl 自己继承 ModuleParams。
	void setParamMpcAccHor(const float param_mpc_acc_hor) { _param_mpc_acc_hor = param_mpc_acc_hor; } ///< 设置水平加速度限制 MPC_ACC_HOR
	void setParamMpcAccDownMax(const float param_mpc_acc_down_max) { _param_mpc_acc_down_max = param_mpc_acc_down_max; } ///< 设置最大下降加速度
	void setParamMpcAccUpMax(const float param_mpc_acc_up_max) { _param_mpc_acc_up_max = param_mpc_acc_up_max; } ///< 设置最大上升加速度
	void setParamMpcJerkAuto(const float param_mpc_jerk_auto) { _position_smoothing.setMaxJerk(param_mpc_jerk_auto); } ///< 设置自动模式 jerk 限制
	void setParamMpcXyCruise(const float param_mpc_xy_cruise) { _param_mpc_xy_cruise = param_mpc_xy_cruise; } ///< 设置水平巡航速度
	void setParamMpcXyErrMax(const float param_mpc_xy_err_max) { _position_smoothing.setMaxAllowedHorizontalError(param_mpc_xy_err_max); } ///< 设置允许的最大水平跟踪误差
	void setParamMpcXyVelMax(const float param_mpc_xy_vel_max) { _position_smoothing.setMaxVelocityXY(param_mpc_xy_vel_max); } ///< 设置水平速度硬上限
	void setParamMpcYawrautoMax(const float param_mpc_yawrauto_max) { _param_mpc_yawrauto_max = param_mpc_yawrauto_max; } ///< 设置自动 yaw rate 上限
	void setParamMpcYawrautoAcc(const float param_mpc_yawrauto_acc) { _param_mpc_yawrauto_acc = param_mpc_yawrauto_acc; } ///< 设置自动 yaw 加速度上限
	void setParamMpcZVAutoDn(const float param_mpc_z_v_auto_dn) { _param_mpc_z_v_auto_dn = param_mpc_z_v_auto_dn; } ///< 设置自动下降速度上限
	void setParamMpcZVAutoUp(const float param_mpc_z_v_auto_up) { _param_mpc_z_v_auto_up = param_mpc_z_v_auto_up; } ///< 设置自动上升速度上限

private:
	/**
	 * @brief optionally sets dynamic translational speed limits with corresponding scale on acceleration
	 * @brief 根据 goto_setpoint 可选地设置动态平移速度限制，并按比例缩放加速度限制。
	 *
	 * @param goto_setpoint struct containing current go-to setpoints
	 * @param goto_setpoint 当前 go-to 目标及其可选速度/加速度约束
	 */
	void setPositionSmootherLimits(const goto_setpoint_s &goto_setpoint);

	/**
	 * @brief optionally sets a dynamic heading rate limit with corresponding scale on heading acceleration
	 * @brief 根据 goto_setpoint 可选地设置动态航向角速度限制，并按比例缩放航向角加速度限制。
	 *
	 * @param goto_setpoint struct containing current go-to setpoints
	 * @param goto_setpoint 当前 go-to 目标及其可选航向约束
	 */
	void setHeadingSmootherLimits(const goto_setpoint_s &goto_setpoint);

	uORB::Subscription _goto_setpoint_sub{ORB_ID(goto_setpoint)}; ///< 高层“去某点”命令输入
	uORB::Publication<trajectory_setpoint_s> _trajectory_setpoint_pub{ORB_ID(trajectory_setpoint)}; ///< 平滑后的轨迹设定值输出
	uORB::Publication<vehicle_constraints_s> _vehicle_constraints_pub{ORB_ID(vehicle_constraints)}; ///< 同步发布约束，供主控制器处理起飞/速度限制

	goto_setpoint_s _goto_setpoint{}; ///< 最近收到的 goto_setpoint
	PositionSmoothing _position_smoothing; ///< 位置/速度/加速度/jerk 平滑器
	HeadingSmoothing _heading_smoothing; ///< 航向角/角速度平滑器

	bool _is_initialized{false}; ///< true 表示平滑器已经重置到当前飞机状态

	// flags that the next update() requires a valid current vehicle position to reset the smoothers
	// 标记下一次 update() 需要有效当前位置来重置平滑器
	bool _need_smoother_reset{true};

	// flags if the last update() was controlling heading
	// 标记上一轮 update() 是否在控制航向
	bool _controlling_heading{false};

	float _param_mpc_acc_hor{0.f}; ///< 水平加速度限制
	float _param_mpc_acc_down_max{0.f}; ///< 下降加速度限制
	float _param_mpc_acc_up_max{0.f}; ///< 上升加速度限制
	float _param_mpc_xy_cruise{0.f}; ///< 水平巡航速度
	float _param_mpc_yawrauto_max{0.f}; ///< 自动 yaw rate 上限
	float _param_mpc_yawrauto_acc{0.f}; ///< 自动 yaw 加速度上限
	float _param_mpc_z_v_auto_dn{0.f}; ///< 自动下降速度上限
	float _param_mpc_z_v_auto_up{0.f}; ///< 自动上升速度上限
};
