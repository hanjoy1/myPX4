/****************************************************************************
 *
 *   Copyright (c) 2013-2022 PX4 Development Team. All rights reserved.
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
 * @file LaunchDetector.cpp
 * Auto launch detection for catapult/hand-launch vehicles
 *
 * 该检测器刻意保持简单且确定：统计机体前向加速度超过阈值的持续时间，
 * 达到配置时长后才认为飞机已发射。FixedWingModeManager 使用该结果延迟油门
 * 和爬升逻辑，直到真正检测到发射。
 *
 * @author Thomas Gubler <thomasgubler@gmail.com>
 */

#include "LaunchDetector.h"

#include <px4_platform_common/log.h>
#include <px4_platform_common/events.h>

namespace launchdetection
{

void LaunchDetector::update(const float dt, const float accel_x)
{
	/*
	 * 状态机含义：
	 *   NONE -> 等待加速度触发
	 *   ENABLEMOTORS -> 已检测到发射，但电机延迟可能仍在生效
	 *   DETECTED -> 发射完全确认
	 */
	switch (state_) {
	case launch_detection_status_s::STATE_WAITING_FOR_LAUNCH:

		// 等待发射阶段周期性提示用户，说明系统还在等待手抛/弹射加速度。
		info_delay_counter_s_ += dt;

		/* Inform user that launchdetection is running every kInfoDelay seconds */
		if (info_delay_counter_s_ >= kInfoDelay) {
			events::send(events::ID("launch_detection_running_info"), events::Log::Info, "Launch detection running");
			info_delay_counter_s_ = 0.f; // reset counter
		}

		/* Detect a acceleration that is longer and stronger as the minimum given by the params */
		// 中文：加速度必须“超过阈值且持续足够久”，单个尖峰不会触发发射。
		if (accel_x > param_fw_laun_ac_thld_.get()) {
			acceleration_detected_counter_ += dt;

			if (acceleration_detected_counter_ > param_fw_laun_ac_t_.get()) {
				if (param_fw_laun_mot_del_.get() > 0.f) {
					// 配置了电机延迟：先允许姿态控制，但油门仍延迟到计时结束。
					state_ = launch_detection_status_s::STATE_LAUNCH_DETECTED_DISABLED_MOTOR;
					events::send<float>(events::ID("launch_detection_wait_for_throttle"), {events::Log::Info, events::LogInternal::Info},
							    "Launch detected: enablecontrol, waiting {1:.1}s until full throttle", (double)param_fw_laun_mot_del_.get());

				} else {
					/* No motor delay set: go directly to enablemotors state */
					// 中文：无电机延迟时直接进入 FLYING，允许正常油门。
					state_ = launch_detection_status_s::STATE_FLYING;
					events::send(events::ID("launch_detection_no_motor_delay"), {events::Log::Info, events::LogInternal::Info},
						     "Launch detected: enable motors (no motor delay)");
				}
			}

		} else {
			// 加速度低于阈值则重新累计，要求连续满足触发条件。
			acceleration_detected_counter_ = 0.f;
		}

		break;

	case launch_detection_status_s::STATE_LAUNCH_DETECTED_DISABLED_MOTOR:
		/* Vehicle is currently controlling attitude but at idle throttle. Waiting until delay is
		 * over to allow full throttle */
		// 中文：发射已确认但电机仍保持 idle，等待 FW_LAUN_MOT_DEL 延迟完成。
		motor_delay_counter_ += dt;

		if (motor_delay_counter_ > param_fw_laun_mot_del_.get()) {
			events::send(events::ID("launch_detection_enable_motors"), {events::Log::Info, events::LogInternal::Info},
				     "Launch detected: enable motors");
			state_ = launch_detection_status_s::STATE_FLYING;
		}

		info_delay_counter_s_ = kInfoDelay; // reset counter
		// 中文：重置提示计时，避免此状态反复输出等待提示。

		break;

	default: // state flying
		// FLYING 或其他完成状态：保持计时器复位，不再执行检测。
		info_delay_counter_s_ = kInfoDelay; // reset counter
		break;

	}
}

uint LaunchDetector::getLaunchDetected() const
{
	// 返回 uORB launch_detection_status 使用的状态枚举值。
	return state_;
}

void LaunchDetector::reset()
{
	// 恢复到等待发射状态，并清空所有持续时间计数器。
	info_delay_counter_s_ = 0.f;
	acceleration_detected_counter_ = 0.f;
	motor_delay_counter_ = 0.f;
	state_ = launch_detection_status_s::STATE_WAITING_FOR_LAUNCH;
}


} // namespace launchdetection
