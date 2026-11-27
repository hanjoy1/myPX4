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
 * @file Takeoff.cpp
 */

#include "Takeoff.hpp"
#include <mathlib/mathlib.h>
#include <lib/geo/geo.h>

// 反推起飞斜坡初始速度限制，使速度控制器在斜坡起点输出约为零推力。
void TakeoffHandling::generateInitialRampValue(float velocity_p_gain)
{
	// 通过速度 P 增益反推一个初始垂向速度误差，使速度控制器输出约为零推力。
	// 这样 rampup 刚开始时不会突然给出悬停以上推力。
	velocity_p_gain = math::max(velocity_p_gain, 0.01f);
	_takeoff_ramp_vz_init = -CONSTANTS_ONE_G / velocity_p_gain;
}

// 更新起飞状态机，管理 disarmed/spoolup/ready/rampup/flight 之间的转换。
void TakeoffHandling::updateTakeoffState(const bool armed, const bool landed, const bool want_takeoff,
		const float takeoff_desired_vz, const bool skip_takeoff, const hrt_abstime &now_us)
{
	// Hysteresis 只有在 armed 持续 COM_SPOOLUP_TIME 后才变 true，
	// 用来给电机转速建立留出时间。
	_spoolup_time_hysteresis.set_state_and_update(armed, now_us);

	switch (_takeoff_state) {
	case TakeoffState::disarmed:
		if (armed) {
			_takeoff_state = TakeoffState::spoolup;

		} else {
			break;
		}

	// 有意 fallthrough：如果条件已经满足，可以在一次 update 内连续推进多个状态。
	// FALLTHROUGH
	case TakeoffState::spoolup:
		if (_spoolup_time_hysteresis.get_state()) {
			_takeoff_state = TakeoffState::ready_for_takeoff;

		} else {
			break;
		}

	// FALLTHROUGH
	case TakeoffState::ready_for_takeoff:
		if (want_takeoff) {
			// 收到起飞意图后从 0 重新开始速度限制斜坡。
			_takeoff_state = TakeoffState::rampup;
			_takeoff_ramp_progress = 0.f;

		} else {
			break;
		}

	// FALLTHROUGH
	case TakeoffState::rampup:
		if (_takeoff_ramp_progress >= 1.f) {
			_takeoff_state = TakeoffState::flight;

		} else {
			break;
		}

	// FALLTHROUGH
	case TakeoffState::flight:
		if (landed) {
			_takeoff_state = TakeoffState::ready_for_takeoff;
		}

		break;

	default:
		break;
	}

	if (armed && skip_takeoff) {
		// 抛飞或非高度控制模式切换时不需要地面起飞斜坡，直接视为 flight。
		_takeoff_state = TakeoffState::flight;
	}

	// TODO: need to consider free fall here
	// TODO：这里后续需要考虑自由落体状态下的起飞状态机处理。
	if (!armed) {
		// 一旦未解锁，状态机回到初始状态。
		_takeoff_state = TakeoffState::disarmed;
	}
}

// 根据起飞状态更新并返回当前上升速度限制，用速度限制斜坡实现平滑离地。
float TakeoffHandling::updateRamp(const float dt, const float takeoff_desired_vz)
{
	// 默认使用外部给定的上升速度限制；起飞前/起飞中会被改成斜坡值。
	float upwards_velocity_limit = takeoff_desired_vz;

	if (_takeoff_state < TakeoffState::rampup) {
		// 起飞前使用“零推力等效速度”，让位置控制器不会主动拉升。
		upwards_velocity_limit = _takeoff_ramp_vz_init;
	}

	if (_takeoff_state == TakeoffState::rampup) {
		// rampup 阶段把速度限制从初始值线性插值到目标上升速度。
		if (_takeoff_ramp_time > dt) {
			_takeoff_ramp_progress += dt / _takeoff_ramp_time;

		} else {
			_takeoff_ramp_progress = 1.f;
		}

		if (_takeoff_ramp_progress < 1.f) {
			upwards_velocity_limit = _takeoff_ramp_vz_init + _takeoff_ramp_progress * (takeoff_desired_vz - _takeoff_ramp_vz_init);
		}
	}

	return upwards_velocity_limit;
}
