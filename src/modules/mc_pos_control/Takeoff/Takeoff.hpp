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
 * @file Takeoff.hpp
 *
 * A class handling all takeoff states and a smooth ramp up of the motors.
 *
 * 中文阅读提示：
 * 起飞处理不直接输出推力，而是通过“上升速度限制”的斜坡让 PositionControl 慢慢增加推力。
 * 这样可以避免解锁后因为高度误差或速度积分导致电机/姿态设定值突然跳变。
 */

#pragma once

#include <lib/hysteresis/hysteresis.h>
#include <drivers/drv_hrt.h>
#include <uORB/topics/takeoff_status.h>

using namespace time_literals;

enum class TakeoffState {
	disarmed = takeoff_status_s::TAKEOFF_STATE_DISARMED, ///< 未解锁
	spoolup = takeoff_status_s::TAKEOFF_STATE_SPOOLUP, ///< 已解锁，等待电机转速建立
	ready_for_takeoff = takeoff_status_s::TAKEOFF_STATE_READY_FOR_TAKEOFF, ///< 电机就绪，等待起飞意图
	rampup = takeoff_status_s::TAKEOFF_STATE_RAMPUP, ///< 正在执行平滑起飞斜坡
	flight = takeoff_status_s::TAKEOFF_STATE_FLIGHT ///< 已进入正常飞行
};

class TakeoffHandling
{
public:
	TakeoffHandling() = default;
	~TakeoffHandling() = default;

	// initialize parameters
	// 初始化起飞相关参数
	/** 设置电机预转等待时间，对应 COM_SPOOLUP_TIME。 */
	void setSpoolupTime(const float seconds) { _spoolup_time_hysteresis.set_hysteresis_time_from(false, seconds * 1_s); }
	/** 设置平滑起飞斜坡时间，对应 MPC_TKO_RAMP_T。 */
	void setTakeoffRampTime(const float seconds) { _takeoff_ramp_time = seconds; }

	/**
	 * Calculate a vertical velocity to initialize the takeoff ramp
	 * that when passed to the velocity controller results in a zero throttle setpoint.
	 * 计算起飞斜坡的初始垂向速度限制。
	 * 该速度限制传给速度控制器后，会得到接近零推力的输出，用于平滑起步。
	 * Note: older versions also considered hover thrust here; the current implementation only needs the velocity P gain.
	 * 注意：旧版本这里还会考虑 hover thrust；当前实现只需要速度 P 增益。
	 * @param velocity_p_gain proportional gain of the velocity controller to calculate the thrust
	 * @param velocity_p_gain 速度控制器 P 增益，用来反推零推力等效速度
	 */
	void generateInitialRampValue(const float velocity_p_gain);

	/**
	 * Update the state for the takeoff.
	 * Has to be called also when not flying altitude controlled to skip the takeoff and not do it in flight when switching mode.
	 * 更新起飞状态机。
	 * 即使当前不是高度控制模式也要调用它，这样切换模式时能跳过已完成的起飞流程，避免空中重复起飞。
	 * @param armed 当前是否解锁
	 * @param landed 当前是否被落地检测认为已落地
	 * @param want_takeoff 是否有起飞意图
	 * @param takeoff_desired_vz 期望起飞垂向速度，当前实现未直接使用
	 * @param skip_takeoff 是否跳过起飞斜坡，直接进入 flight 状态
	 * @param now_us 当前时间戳 [us]
	 */
	void updateTakeoffState(const bool armed, const bool landed, const bool want_takeoff,
				const float takeoff_desired_vz, const bool skip_takeoff, const hrt_abstime &now_us);

	/**
	 * Update and return the velocity constraint ramp value during takeoff.
	 * By ramping up _takeoff_ramp_vz during the takeoff and using it to constain the maximum climb rate a smooth takeoff behavior is achieved.
	 * Returns zero on the ground and takeoff_desired_vz in flight.
	 * 更新并返回起飞过程中的上升速度限制斜坡值。
	 * 通过逐步放开最大爬升率，让离地过程更平滑。
	 * 起飞前返回接近零推力的初始值，进入正常飞行后返回 takeoff_desired_vz。
	 * @param dt time in seconds since the last call/loop iteration
	 * @param dt 距离上一次调用的时间 [s]
	 * @param takeoff_desired_vz end value for the velocity ramp
	 * @param takeoff_desired_vz 起飞速度斜坡的最终目标值 [m/s]
	 * @return current upward velocity limit
	 * @return 当前应使用的上升速度限制值
	 */
	float updateRamp(const float dt, const float takeoff_desired_vz);

	/** 获取当前起飞状态。 */
	TakeoffState getTakeoffState() { return _takeoff_state; }

private:
	TakeoffState _takeoff_state = TakeoffState::disarmed;

	systemlib::Hysteresis _spoolup_time_hysteresis{false}; ///< 解锁后经过 COM_SPOOLUP_TIME 秒才变为 true

	float _takeoff_ramp_time{0.f}; ///< 起飞速度限制从初始值过渡到目标值的时间 [s]
	float _takeoff_ramp_vz_init{0.f}; ///< 会产生零推力的初始垂向速度限制
	float _takeoff_ramp_progress{0.f}; ///< 起飞斜坡进度，0 到 1
};
