/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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
 * @file CombinedControllerConfigurationHandler.cpp
 *
 * 横纵向控制器配置发布辅助类。只有配置值发生变化时才发布，避免不必要的 uORB 流量；
 * 同时允许特殊飞行阶段覆盖控制器限制。
 */

#include "ControllerConfigurationHandler.hpp"
#include <drivers/drv_hrt.h>

using namespace time_literals;


void CombinedControllerConfigurationHandler::update(const hrt_abstime now)
{
	/*
	 * 比较本周期准备发布的配置和上一次已发布配置。只有发生变化的配置会更新时间戳并发布。
	 * NaN 在这里有明确含义：告诉下游控制器该字段使用默认参数值。
	 */
	_longitudinal_updated = floatValueChanged(_longitudinal_configuration_current_cycle.pitch_min,
				_longitudinal_publisher.get().pitch_min);
	_longitudinal_updated |= floatValueChanged(_longitudinal_configuration_current_cycle.pitch_max,
				 _longitudinal_publisher.get().pitch_max);
	_longitudinal_updated |= floatValueChanged(_longitudinal_configuration_current_cycle.throttle_min,
				 _longitudinal_publisher.get().throttle_min);
	_longitudinal_updated |= floatValueChanged(_longitudinal_configuration_current_cycle.throttle_max,
				 _longitudinal_publisher.get().throttle_max);
	_longitudinal_updated |= floatValueChanged(_longitudinal_configuration_current_cycle.speed_weight,
				 _longitudinal_publisher.get().speed_weight);
	_longitudinal_updated |= floatValueChanged(_longitudinal_configuration_current_cycle.climb_rate_target,
				 _longitudinal_publisher.get().climb_rate_target);
	_longitudinal_updated |= floatValueChanged(_longitudinal_configuration_current_cycle.sink_rate_target,
				 _longitudinal_publisher.get().sink_rate_target);
	_longitudinal_updated |= booleanValueChanged(_longitudinal_configuration_current_cycle.enforce_low_height_condition,
				 _longitudinal_publisher.get().enforce_low_height_condition);
	_longitudinal_updated |= booleanValueChanged(_longitudinal_configuration_current_cycle.disable_underspeed_protection,
				 _longitudinal_publisher.get().disable_underspeed_protection);

	_lateral_updated |= floatValueChanged(_lateral_configuration_current_cycle.lateral_accel_max,
					      _lateral_publisher.get().lateral_accel_max);

	if (_longitudinal_updated || now - _time_last_longitudinal_publish > 1_s) {
		// 即使没有变化，也至少 1 Hz 重发一次，防止下游订阅者错过最新配置。
		_longitudinal_configuration_current_cycle.timestamp = now;
		_longitudinal_publisher.update(_longitudinal_configuration_current_cycle);
		_time_last_longitudinal_publish = _longitudinal_configuration_current_cycle.timestamp;
	}

	if (_lateral_updated || now - _time_last_lateral_publish > 1_s) {
		// 横向配置同样按变化触发或 1 Hz 心跳发布。
		_lateral_configuration_current_cycle.timestamp = now;
		_lateral_publisher.update(_lateral_configuration_current_cycle);
		_time_last_lateral_publish = _lateral_configuration_current_cycle.timestamp;
	}

	_longitudinal_updated = _lateral_updated = false;
	// 本周期发布完后恢复为空配置；下一个模式函数只需要写自己要覆盖的字段。
	_longitudinal_configuration_current_cycle = empty_longitudinal_control_configuration;
	_lateral_configuration_current_cycle = empty_lateral_control_configuration;
}

void CombinedControllerConfigurationHandler::setThrottleMax(float throttle_max)
{
	// 设置纵向控制油门上限；NaN 表示下游使用参数默认值。
	_longitudinal_configuration_current_cycle.throttle_max = throttle_max;
}

void CombinedControllerConfigurationHandler::setThrottleMin(float throttle_min)
{
	// 设置纵向控制油门下限，常用于滑翔、降落 idle、地面关油门等阶段。
	_longitudinal_configuration_current_cycle.throttle_min = throttle_min;
}

void CombinedControllerConfigurationHandler::setSpeedWeight(float speed_weight)
{
	// 设置 TECS 速度/高度权重：0 偏保高，2 偏保速。
	_longitudinal_configuration_current_cycle.speed_weight = speed_weight;
}

void CombinedControllerConfigurationHandler::setPitchMin(const float pitch_min)
{
	// 设置 TECS 可用最小俯仰角，起飞/降落会用它限制机头下俯。
	_longitudinal_configuration_current_cycle.pitch_min = pitch_min;
}

void CombinedControllerConfigurationHandler::setPitchMax(const float pitch_max)
{
	// 设置 TECS 可用最大俯仰角，跑道起飞和 flare 阶段会动态收紧。
	_longitudinal_configuration_current_cycle.pitch_max = pitch_max;
}

void CombinedControllerConfigurationHandler::setClimbRateTarget(float climbrate_target)
{
	// 覆盖目标爬升率幅值，起飞阶段通常使用最大起飞爬升率。
	_longitudinal_configuration_current_cycle.climb_rate_target = climbrate_target;
}

void CombinedControllerConfigurationHandler::setDisableUnderspeedProtection(bool disable)
{
	// 禁用 TECS underspeed 保护，常用于起飞/flare，防止保护逻辑改变预期俯仰。
	_longitudinal_configuration_current_cycle.disable_underspeed_protection = disable;
}

void CombinedControllerConfigurationHandler::setSinkRateTarget(const float sinkrate_target)
{
	// 覆盖目标下降率幅值，降落下滑道会按几何需要放宽该限制。
	_longitudinal_configuration_current_cycle.sink_rate_target = sinkrate_target;
}

void CombinedControllerConfigurationHandler::setLateralAccelMax(float lateral_accel_max)
{
	// 设置横向最大加速度，用于近地滚转限制或降落中止直线爬升。
	_lateral_configuration_current_cycle.lateral_accel_max = lateral_accel_max;
}

void CombinedControllerConfigurationHandler::setEnforceLowHeightCondition(bool low_height_condition)
{
	// 强制下游 TECS 使用低高度条件，降落/降落前盘旋用于更保守近地控制。
	_longitudinal_configuration_current_cycle.enforce_low_height_condition = low_height_condition;
}

void CombinedControllerConfigurationHandler::resetLastPublishTime()
{
	// 清零发布时间，下次 update() 会立即发布完整配置。
	_time_last_longitudinal_publish = _time_last_lateral_publish = 0;
}
