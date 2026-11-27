/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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
 * @file RunwayTakeoff.cpp
 * Runway takeoff handling for fixed-wing UAVs with steerable wheels.
 *
 * 中文说明：固定翼跑道起飞辅助逻辑。
 * 该模块不直接输出舵面，而是为 FixedWingModeManager 提供地面滑跑、
 * 抬轮和爬升阶段所需的油门、俯仰、滚转和前轮转向约束。
 *
 * @author Roman Bapst <roman@px4.io>
 * @author Andreas Antener <andreas@uaventure.com>
 */

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "RunwayTakeoff.h"
#include <mathlib/mathlib.h>
#include <px4_platform_common/events.h>

using namespace time_literals;

namespace runwaytakeoff
{

void RunwayTakeoff::init(const hrt_abstime &time_now)
{
	// 初始化进入油门爬升阶段：从这里开始计算跑道起飞状态机的相对时间。
	takeoff_state_ = RunwayTakeoffState::THROTTLE_RAMP;
	initialized_ = true;
	time_initialized_ = time_now;
	takeoff_time_ = 0;
}

void RunwayTakeoff::update(const hrt_abstime &time_now, const float takeoff_airspeed, const float calibrated_airspeed,
			   const float vehicle_altitude, const float clearance_altitude)
{
	/*
	 * 跑道起飞状态机：
	 *   THROTTLE_RAMP      油门按时间逐渐爬升，避免突然推力造成姿态/方向突变
	 *   CLAMPED_TO_RUNWAY  仍贴在跑道上滑跑，等待达到抬轮空速
	 *   CLIMBOUT           已抬轮，保持起飞爬升约束直到达到安全高度
	 */
	switch (takeoff_state_) {
	case RunwayTakeoffState::THROTTLE_RAMP:
		// 油门爬升计时结束后，进入贴地滑跑阶段。
		if ((time_now - time_initialized_) > (param_rwto_ramp_time_.get() * 1_s)) {
			takeoff_state_ = RunwayTakeoffState::CLAMPED_TO_RUNWAY;
		}

		break;

	case RunwayTakeoffState::CLAMPED_TO_RUNWAY: {
			// 抬轮空速优先使用 RWTO_ROT_AIRSPD，否则取目标起飞空速的 90%。
			const float rotation_airspeed = (param_rwto_rot_airspd_.get() > FLT_EPSILON) ? math::min(param_rwto_rot_airspd_.get(),
							takeoff_airspeed) : 0.9f * takeoff_airspeed;

			if (calibrated_airspeed > rotation_airspeed) {
				// 达到抬轮空速后记录抬轮时间，用于俯仰限制渐变。
				takeoff_time_ = time_now;
				takeoff_state_ = RunwayTakeoffState::CLIMBOUT;
				events::send(events::ID("runway_takeoff_reached_airspeed"), events::Log::Info,
					     "Takeoff airspeed reached, climbout");
			}

			break;
		}

	case RunwayTakeoffState::CLIMBOUT:
		// 相对起飞高度超过越障高度后，交还给正常固定翼飞行控制。
		if (vehicle_altitude > clearance_altitude) {
			takeoff_state_ = RunwayTakeoffState::FLY;
			events::send(events::ID("runway_takeoff_reached_clearance_altitude"), events::Log::Info, "Reached clearance altitude");
		}

		break;

	default:
		break;
	}
}

float RunwayTakeoff::getPitch()
{
	// 滑跑和贴地阶段直接给跑道俯仰 setpoint；进入爬升后返回 NaN 让 TECS 接管。
	if (takeoff_state_ <= RunwayTakeoffState::CLAMPED_TO_RUNWAY) {
		return math::radians(param_rwto_psp_.get());
	}

	return NAN;
}

float RunwayTakeoff::getRoll()
{
	// until we have enough ground clearance, set roll to 0
	// 中文：离地前保持平翼；离地爬升后返回 NaN，横向制导可正常给滚转。
	if (takeoff_state_ < RunwayTakeoffState::CLIMBOUT) {
		return 0.0f;
	}

	return NAN;
}

float RunwayTakeoff::getThrottle(const float idle_throttle) const
{
	// 根据跑道起飞状态给直接油门；返回 NaN 表示正常 TECS 油门控制。
	float throttle = idle_throttle;

	switch (takeoff_state_) {
	case RunwayTakeoffState::THROTTLE_RAMP:

		// 从 idle 平滑插值到跑道最大油门，降低滑跑初段扰动。
		throttle = interpolateValuesOverAbsoluteTime(idle_throttle, param_rwto_max_thr_.get(), time_initialized_,
				param_rwto_ramp_time_.get());

		break;

	case RunwayTakeoffState::CLAMPED_TO_RUNWAY:
	case RunwayTakeoffState::CLIMBOUT:
		// 滑跑和爬升离地阶段使用跑道起飞最大油门。
		throttle = param_rwto_max_thr_.get();

		break;

	case RunwayTakeoffState::FLY:
		// 正常飞行后不再直接控制油门。
		throttle = NAN;
	}

	return throttle;
}

float RunwayTakeoff::getMinPitch(float min_pitch_in_climbout, float min_pitch) const
{
	// 生成当前阶段的最小俯仰限制，用于防止滑跑/抬轮阶段俯仰突变。
	if (takeoff_state_ < RunwayTakeoffState::CLIMBOUT) {
		// constrain to the taxi pitch setpoint
		// 中文：滑跑时把上下限夹在 taxi pitch 附近，形成近似直接俯仰。
		return math::radians(param_rwto_psp_.get() - 0.01f);

	} else if (takeoff_state_ < RunwayTakeoffState::FLY) {
		// ramp in the climbout pitch constraint over the rotation transition time
		// 中文：抬轮到爬升阶段，最小俯仰从 taxi pitch 平滑过渡到起飞爬升下限。
		const float taxi_pitch_min = math::radians(param_rwto_psp_.get() - 0.01f);
		return interpolateValuesOverAbsoluteTime(taxi_pitch_min, min_pitch_in_climbout, takeoff_time_,
				param_rwto_rot_time_.get());

	} else {
		// 正常飞行后使用全局最小俯仰限制。
		return min_pitch;
	}
}

float RunwayTakeoff::getMaxPitch(const float max_pitch) const
{
	// 生成当前阶段的最大俯仰限制，与 getMinPitch 一起形成阶段性俯仰窗口。
	if (takeoff_state_ < RunwayTakeoffState::CLIMBOUT) {
		// constrain to the taxi pitch setpoint
		// 中文：滑跑时最大俯仰也夹在 taxi pitch 附近。
		return math::radians(param_rwto_psp_.get() + 0.01f);

	} else if (takeoff_state_ < RunwayTakeoffState::FLY) {
		// ramp in the climbout pitch constraint over the rotation transition time
		// 中文：抬轮后最大俯仰逐渐放开到全局最大俯仰。
		const float taxi_pitch_max = math::radians(param_rwto_psp_.get() + 0.01f);
		return interpolateValuesOverAbsoluteTime(taxi_pitch_max, max_pitch, takeoff_time_, param_rwto_rot_time_.get());

	} else {
		// 正常飞行后使用全局最大俯仰限制。
		return max_pitch;
	}
}

void RunwayTakeoff::reset()
{
	// 清空初始化标志和抬轮时间，下次进入跑道起飞重新从油门爬升开始。
	initialized_ = false;
	takeoff_state_ = RunwayTakeoffState::THROTTLE_RAMP;
	takeoff_time_ = 0;
}

float RunwayTakeoff::interpolateValuesOverAbsoluteTime(const float start_value, const float end_value,
		const hrt_abstime &start_time, const float interpolation_time) const
{
	// 按绝对时间做线性插值；用于油门 ramp 和俯仰限制 ramp。
	const float seconds_since_start = hrt_elapsed_time(&start_time) * 1.e-6f;
	const float interpolator = math::constrain(seconds_since_start / interpolation_time, 0.0f, 1.0f);

	// interpolator=0 输出 start_value，=1 输出 end_value。
	return interpolator * end_value + (1.0f - interpolator) * start_value;
}

} // namespace runwaytakeoff
