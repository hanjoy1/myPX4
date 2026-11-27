/****************************************************************************
 *
 *   Copyright (c) 2017-2023 PX4 Development Team. All rights reserved.
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
 * @file TECS.cpp
 *
 * 中文说明：
 * 本文件实现固定翼 TECS 纵向能量控制。
 * 从控制结构上看，它把“高度/爬升率目标 + 空速目标”转换为“俯仰角目标 + 油门目标”。
 * 其中：
 *  - 高度/爬升率目标先进入 TECSAltitudeReferenceModel，得到平滑 altitude_reference；
 *  - 空速输入先进入 TECSAirspeedFilter，得到滤波后的 EAS 和 EAS rate；
 *  - TECSControl 把 hdot、Vdot 转为势能率/动能率，再分别控制 pitch 和 throttle。
 *
 * @author Paul Riseborough
 */

#include "TECS.hpp"

#include <lib/geo/geo.h>

#include <px4_platform_common/defines.h>

#include "matrix/Matrix.hpp"
#include "matrix/Vector2.hpp"
#include <mathlib/math/Functions.hpp>

using math::constrain;
using math::max;
using math::min;

static inline constexpr bool TIMESTAMP_VALID(float dt) { return (PX4_ISFINITE(dt) && dt > FLT_EPSILON);}

void TECSAirspeedFilter::initialize(const float equivalent_airspeed, const float equivalent_airspeed_trim,
				    const bool airspeed_sensor_available)
{
	// 中文：如果空速传感器可用且当前 EAS 有效，则滤波器从当前 EAS 开始，保证状态连续。
	if (airspeed_sensor_available && PX4_ISFINITE(equivalent_airspeed)) {
		_airspeed_state.speed = equivalent_airspeed;

	} else {
		// 中文：没有有效空速时，使用配平空速作为默认 EAS，避免 TECS 后续能量换算得到 NAN。
		_airspeed_state.speed = equivalent_airspeed_trim;
	}

	// 中文：初始化时没有可靠的空速变化率历史，所以变化率从 0 开始。
	_airspeed_state.speed_rate = 0.f;
}

void TECSAirspeedFilter::update(const float dt, const Input &input, const Param &param,
				const bool airspeed_sensor_available)
{
	// Input checking
	// 中文：dt 无效时不更新状态，保持上一次空速估计，避免滤波器数值发散。
	if (!TIMESTAMP_VALID(dt)) {
		// Do not update the states.
		PX4_WARN("Time intervall is not valid.");
		return;
	}

	float airspeed;

	// 中文：空速传感器可用时用测量值；不可用时回退到巡航空速。
	if (PX4_ISFINITE(input.equivalent_airspeed) && airspeed_sensor_available) {
		airspeed = input.equivalent_airspeed;

	} else {
		airspeed = param.equivalent_airspeed_trim;
	}

	float airspeed_derivative;

	// 中文：同理，空速变化率只有在测量有效且空速可用时才使用；否则设为 0。
	if (PX4_ISFINITE(input.equivalent_airspeed_rate) && airspeed_sensor_available) {
		airspeed_derivative = input.equivalent_airspeed_rate;

	} else {
		airspeed_derivative = 0.0f;
	}

	/* Filter airspeed and rate using a constant airspeed rate model in a steady state Kalman Filter.
	 We use the gains of the continuous Kalman filter Kc and approximate the discrete version Kalman gain Kd =dt*Kc,
	 since the continuous algebraic Riccatti equation is easier to solve.
	 中文：
	 这里用“速度 + 速度变化率”的二阶状态滤波：
	   x = [EAS, EAS_dot]
	   预测模型：EAS(k+1) = EAS(k) + dt * EAS_dot(k)，EAS_dot 近似常值。
	 后面的 kalman_gain 是稳态 Kalman 增益近似，用 dt*Kc 从连续形式离散化。
	*/

	matrix::Vector2f new_state_predicted;

	// 中文：模型预测步骤，不使用测量，只根据上一状态外推。
	new_state_predicted(0) = _airspeed_state.speed + dt * _airspeed_state.speed_rate;
	new_state_predicted(1) = _airspeed_state.speed_rate;

	// 中文：根据测量噪声和过程噪声计算稳态 Kalman 增益。
	const float airspeed_noise_inv{1.0f / param.airspeed_measurement_std_dev};
	const float airspeed_rate_noise_inv{1.0f / param.airspeed_rate_measurement_std_dev};
	const float airspeed_rate_noise_inv_squared_process_noise{airspeed_rate_noise_inv *airspeed_rate_noise_inv * param.airspeed_rate_noise_std_dev};
	const float denom{airspeed_noise_inv + airspeed_rate_noise_inv_squared_process_noise};
	const float common_nom{std::sqrt(param.airspeed_rate_noise_std_dev * (2.0f * airspeed_noise_inv + airspeed_rate_noise_inv_squared_process_noise))};

	matrix::Matrix<float, 2, 2> kalman_gain;
	kalman_gain(0, 0) = airspeed_noise_inv * common_nom / denom;
	kalman_gain(0, 1) = airspeed_rate_noise_inv_squared_process_noise / denom;
	kalman_gain(1, 0) = airspeed_noise_inv * airspeed_noise_inv * param.airspeed_rate_noise_std_dev / denom;
	kalman_gain(1, 1) = airspeed_rate_noise_inv_squared_process_noise * common_nom / denom;

	// 中文：innovation 是测量值和预测值的差，用来修正预测状态。
	const matrix::Vector2f innovation{(airspeed - new_state_predicted(0)), (airspeed_derivative - new_state_predicted(1))};
	matrix::Vector2f new_state;
	new_state = new_state_predicted + dt * (kalman_gain * (innovation));

	// Clip airspeed at zero
	// 中文：物理上空速不能为负；若滤波结果小于 0，则强制夹到 0，并重新计算相容的 speed_rate。
	if (new_state(0) < FLT_EPSILON) {
		new_state(0) = 0.0f;
		// calculate input that would result in zero speed.
		const float desired_airspeed_innovation = (-new_state_predicted(0) / dt - kalman_gain(0,
				1) * innovation(1)) / kalman_gain(0,
						0);
		new_state(1) = new_state_predicted(1) + dt * (kalman_gain(1, 0) * desired_airspeed_innovation + kalman_gain(1,
				1) * innovation(1));
	}

	// Update states
	// 中文：保存滤波后的 EAS 和 EAS_dot，TECS 主类之后会乘 eas2tas 得到 TAS 和 TAS_dot。
	_airspeed_state.speed = new_state(0);
	_airspeed_state.speed_rate = new_state(1);
}

TECSAirspeedFilter::AirspeedFilterState TECSAirspeedFilter::getState() const
{
	return _airspeed_state;
}

void TECSAltitudeReferenceModel::update(const float dt, const AltitudeReferenceState &setpoint, float altitude,
					float height_rate, const Param &param)
{
	// Input checks
	// 中文：参考轨迹发生器对 dt 很敏感，dt 无效时直接保持上次状态。
	if (!TIMESTAMP_VALID(dt)) {
		// Do not update the states.
		PX4_WARN("Time intervall is not valid.");
		return;
	}

	// 中文：如果高度估计无效，用 0 兜底，避免轨迹发生器进入 NAN。
	const float current_alt = PX4_ISFINITE(altitude) ? altitude : 0.f;

	// 中文：爬升率直接控制轨迹发生器配置。注意这里沿用了 MC 轨迹库，FW 上下方向约定不同。
	_velocity_control_traj_generator.setMaxJerk(param.jerk_max);
	_velocity_control_traj_generator.setMaxAccelUp(param.vert_accel_limit);
	_velocity_control_traj_generator.setMaxAccelDown(param.vert_accel_limit);
	_velocity_control_traj_generator.setMaxVelUp(param.max_sink_rate); // different convention for FW than for MC
	_velocity_control_traj_generator.setMaxVelDown(param.max_climb_rate); // different convention for FW than for MC

	// Altitude setpoint reference
	// 中文：高度控制轨迹发生器配置，用 jerk/加速度/速度限制把高度 setpoint 平滑化。
	_alt_control_traj_generator.setMaxJerk(param.jerk_max);
	_alt_control_traj_generator.setMaxAccel(param.vert_accel_limit);
	_alt_control_traj_generator.setMaxVel(fmax(param.max_climb_rate, param.max_sink_rate));

	// XXX: this is a bit risky.. .alt_rate here could be NAN (by interface design) - and is only ok to input to the
	// setVelSpFeedback() method because it calls the reset in the logic below when it is NAN.
	// TODO: stop it with the NAN interfaces, make sure to take care of this when refactoring and separating altitude
	// and height rate control loops.
	// 中文：PX4 用 NAN 表示“这个通道没有给 setpoint”。这里把 alt_rate 反馈给速度轨迹器，
	// 若 alt_rate 是 NAN，后面的分支会 reset，避免把 NAN 传播到控制输出。
	_velocity_control_traj_generator.setVelSpFeedback(setpoint.alt_rate);

	bool control_altitude = true;
	float altitude_setpoint = setpoint.alt;

	if (!PX4_ISFINITE(setpoint.alt) && !PX4_ISFINITE(setpoint.alt_rate)) {
		// neither altitude nor altitude rate is set - reset to current altitude
		// 中文：既没有高度目标也没有爬升率目标时，锁定当前位置，输出 0 爬升率。
		_velocity_control_traj_generator.reset(0.f, 0, current_alt);
		altitude_setpoint = current_alt;

	} else if (PX4_ISFINITE(setpoint.alt_rate)) {
		// input is height rate (not altitude)
		// 中文：直接爬升率模式。轨迹器根据 alt_rate 生成位置/速度轨迹；
		// 若轨迹器能锁定高度，则 control_altitude 为 true，否则 TECS 直接使用爬升率目标。
		_velocity_control_traj_generator.setCurrentPositionEstimate(current_alt);
		_velocity_control_traj_generator.update(dt, setpoint.alt_rate);
		altitude_setpoint = _velocity_control_traj_generator.getCurrentPosition();
		control_altitude = PX4_ISFINITE(altitude_setpoint); // returns true if altitude is locked

	} else {
		// input is altitude
		// 中文：高度目标模式。速度轨迹器只同步当前高度/爬升率，真正高度参考由 _alt_control_traj_generator 生成。
		_velocity_control_traj_generator.reset(0, height_rate, altitude_setpoint);
	}

	if (control_altitude) {
		// 中文：高度闭环模式。根据当前位置到目标高度的距离，计算一个受 jerk/加速度约束的爬升率目标。
		const float target_climbrate_m_s = math::min(param.target_climbrate, param.max_climb_rate);
		const float target_sinkrate_m_s = math::min(param.target_sinkrate, param.max_sink_rate);

		const float delta_trajectory_to_target_m = altitude_setpoint - _alt_control_traj_generator.getCurrentPosition();

		// 中文：computeMaxSpeedFromDistance 给出在剩余距离内可停车/收敛的最大速度，避免高度参考冲过目标。
		float height_rate_target = math::signNoZero<float>(delta_trajectory_to_target_m) *
					   math::trajectory::computeMaxSpeedFromDistance(
						   param.jerk_max, param.vert_accel_limit, fabsf(delta_trajectory_to_target_m), 0.f);

		height_rate_target = math::constrain(height_rate_target, -target_sinkrate_m_s, target_climbrate_m_s);

		// 中文：更新高度轨迹发生器，输出平滑的 altitude_reference 和 height_rate_reference。
		_alt_control_traj_generator.updateDurations(height_rate_target);
		_alt_control_traj_generator.updateTraj(dt);
		_height_rate_setpoint_direct = NAN;

	} else {
		// 中文：直接爬升率模式。此时高度参考发生器跟随当前位置，TECSControl 会绕过高度 P 环，
		// 直接使用 _height_rate_setpoint_direct 作为 hdot_sp。
		_alt_control_traj_generator.setCurrentVelocity(_velocity_control_traj_generator.getCurrentVelocity());
		_alt_control_traj_generator.setCurrentPosition(current_alt);
		_height_rate_setpoint_direct = _velocity_control_traj_generator.getCurrentVelocity();
	}
}

TECSAltitudeReferenceModel::AltitudeReferenceState TECSAltitudeReferenceModel::getAltitudeReference() const
{
	TECSAltitudeReferenceModel::AltitudeReferenceState ref{
		.alt = _alt_control_traj_generator.getCurrentPosition(),
		.alt_rate = _alt_control_traj_generator.getCurrentVelocity(),
	};

	return ref;
}

void TECSAltitudeReferenceModel::initialize(const AltitudeReferenceState &state)
{
	// 中文：初始化高度参考时把无效输入兜底为 0，防止轨迹发生器内部状态变成 NAN。
	const float init_state_alt = PX4_ISFINITE(state.alt) ? state.alt : 0.f;
	const float init_state_alt_rate = PX4_ISFINITE(state.alt_rate) ? state.alt_rate : 0.f;

	// 中文：两个轨迹发生器都从同一个高度/爬升率状态开始，保证高度模式和直接爬升率模式切换时连续。
	_alt_control_traj_generator.reset(0.0f, init_state_alt_rate, init_state_alt);
	_velocity_control_traj_generator.reset(0.f, init_state_alt_rate, init_state_alt);
}

void TECSControl::initialize(const Setpoint &setpoint, const Input &input, Param &param, const Flag &flag)
{
	// 中文：TECSControl 初始化时清掉两个积分器，避免上一个模式/上一次运行的积分残留。
	resetIntegrals();

	AltitudePitchControl control_setpoint;

	control_setpoint.tas_setpoint = setpoint.tas_setpoint;

	// 中文：先用当前空速误差计算真空速变化率目标 Vdot_sp。
	control_setpoint.tas_rate_setpoint = _calcAirspeedControlOutput(setpoint, input, param, flag);

	// 中文：再用当前高度误差计算爬升率目标 hdot_sp。
	control_setpoint.altitude_rate_setpoint = _calcAltitudeControlOutput(setpoint, input, param);

	// 中文：把 hdot_sp / Vdot_sp 转换成势能率/动能率。
	SpecificEnergyRates specific_energy_rate{_calcSpecificEnergyRates(control_setpoint, input)};

	// 中文：初始化时也计算低空速比例，保证 pitch/throttle 初值符合保护逻辑。
	_detectUnderspeed(input, param, flag);

	// 中文：计算俯仰通道的能量平衡率，并得到初始 pitch_sp。
	const SpecificEnergyWeighting weight{_updateSpeedAltitudeWeights(param, flag)};
	ControlValues seb_rate{_calcPitchControlSebRate(weight, specific_energy_rate)};

	_pitch_setpoint = _calcPitchControlOutput(input, seb_rate, param, flag);

	// 中文：计算油门通道的总能量率，并得到初始 throttle_sp。
	const STERateLimit limit{_calculateTotalEnergyRateLimit(param)};

	// 中文：总能量率滤波器从当前估计值初始化，避免油门阻尼项初始突跳。
	_ste_rate_estimate_filter.reset(specific_energy_rate.spe_rate.estimate + specific_energy_rate.ske_rate.estimate);

	ControlValues ste_rate{_calcThrottleControlSteRate(limit, specific_energy_rate, param)};

	_throttle_setpoint = _calcThrottleControlOutput(limit, ste_rate, param, flag);

	// Debug output
	_debug_output.total_energy_rate_estimate = ste_rate.estimate;
	_debug_output.total_energy_rate_sp = ste_rate.setpoint;
	_debug_output.throttle_integrator = _throttle_integ_state;
	_debug_output.energy_balance_rate_estimate = seb_rate.estimate;
	_debug_output.energy_balance_rate_sp = seb_rate.setpoint;
	_debug_output.pitch_integrator = _pitch_integ_state;
	_debug_output.altitude_rate_control = control_setpoint.altitude_rate_setpoint;
	_debug_output.true_airspeed_derivative_control = control_setpoint.tas_rate_setpoint;
}

void TECSControl::update(const float dt, const Setpoint &setpoint, const Input &input, Param &param, const Flag &flag)
{
	// Input checking
	// 中文：dt 无效时不更新 pitch/throttle，保持上周期输出。
	if (!TIMESTAMP_VALID(dt)) {
		// Do not update the states and output.
		PX4_WARN("Time intervall is not valid.");
		return;
	}

	AltitudePitchControl control_setpoint;

	control_setpoint.tas_setpoint = setpoint.tas_setpoint;
	// 中文：空速环输出 Vdot_sp；没有空速闭环时该值为 0。
	control_setpoint.tas_rate_setpoint = _calcAirspeedControlOutput(setpoint, input, param, flag);

	if (PX4_ISFINITE(setpoint.altitude_rate_setpoint_direct)) {
		// direct height rate control
		// 中文：直接爬升率控制模式，绕过“高度误差 -> 爬升率”的外环。
		control_setpoint.altitude_rate_setpoint = setpoint.altitude_rate_setpoint_direct;

	} else {
		// altitude is locked, go through altitude outer loop
		// 中文：高度控制模式，用高度误差和高度参考变化率前馈生成 hdot_sp。
		control_setpoint.altitude_rate_setpoint = _calcAltitudeControlOutput(setpoint, input, param);
	}

	// 中文：把 hdot_sp 和 Vdot_sp 转换到能量域。
	SpecificEnergyRates specific_energy_rate{_calcSpecificEnergyRates(control_setpoint, input)};

	// 中文：根据当前 TAS 判断是否进入 underspeed 保护。
	_detectUnderspeed(input, param, flag);

	// 中文：pitch 控制能量平衡，throttle 控制总能量。
	_calcPitchControl(dt, input, specific_energy_rate, param, flag);

	_calcThrottleControl(dt, specific_energy_rate, param, flag);

	_debug_output.altitude_rate_control = control_setpoint.altitude_rate_setpoint;
	_debug_output.true_airspeed_derivative_control = control_setpoint.tas_rate_setpoint;
	_debug_output.pitch_integrator = _pitch_integ_state;
	_debug_output.throttle_integrator = _throttle_integ_state;
}

TECSControl::STERateLimit TECSControl::_calculateTotalEnergyRateLimit(const Param &param) const
{
	TECSControl::STERateLimit limit;
	// Calculate the specific total energy rate limits from the max throttle limits
	// 中文：总能量率上限近似由最大爬升率决定：STEdot_max = climb_rate_max * g。
	limit.STE_rate_max = math::max(param.max_climb_rate, FLT_EPSILON) * CONSTANTS_ONE_G;
	// 中文：总能量率下限近似由最小下沉率决定，下降时为负。
	limit.STE_rate_min = - math::max(param.min_sink_rate, FLT_EPSILON) * CONSTANTS_ONE_G;

	return limit;
}

float TECSControl::_calcAirspeedControlOutput(const Setpoint &setpoint, const Input &input, const Param &param,
		const Flag &flag) const
{
	float airspeed_rate_output{0.0f};

	const STERateLimit limit{_calculateTotalEnergyRateLimit(param)};

	// calculate the demanded true airspeed rate of change based on first order response of true airspeed error
	// if airspeed measurement is not enabled then always set the rate setpoint to zero in order to avoid constant rate setpoints
	if (flag.airspeed_enabled) {
		// Calculate limits for the demanded rate of change of speed based on physical performance limits
		// with a 50% margin to allow the total energy controller to correct for errors. Increase it in case of fast descend
		// 中文：把总能量率能力除以 TAS，得到物理上允许的 TAS 加速度范围。
		// 这里乘 0.5 是给总能量控制留余量；fast descend 时余量放宽。
		const float max_tas_rate_sp = (param.fast_descend * 0.5f + 0.5f) * limit.STE_rate_max / math::max(input.tas,
					      FLT_EPSILON);
		const float min_tas_rate_sp = (param.fast_descend * 0.5f + 0.5f) * limit.STE_rate_min / math::max(input.tas,
					      FLT_EPSILON);
		// 中文：一阶空速误差闭环：Vdot_sp = (V_sp - V) / T_airspd，然后按能力限幅。
		airspeed_rate_output = constrain((setpoint.tas_setpoint - input.tas) * param.airspeed_error_gain, min_tas_rate_sp,
						 max_tas_rate_sp);
	}

	return airspeed_rate_output;
}

float TECSControl::_calcAltitudeControlOutput(const Setpoint &setpoint, const Input &input, const Param &param) const
{
	float altitude_rate_output;
	// 中文：高度外环：hdot_sp = 高度误差 / T_alt + 高度参考变化率前馈。
	altitude_rate_output = (setpoint.altitude_reference.alt - input.altitude) * param.altitude_error_gain
			       + param.altitude_setpoint_gain_ff * setpoint.altitude_reference.alt_rate;

	// 中文：爬升率目标限制在飞机最大爬升/最大下沉能力内。
	altitude_rate_output = math::constrain(altitude_rate_output, -param.max_sink_rate, param.max_climb_rate);

	return altitude_rate_output;
}

TECSControl::SpecificEnergyRates TECSControl::_calcSpecificEnergyRates(const AltitudePitchControl &control_setpoint,
		const Input &input) const
{
	SpecificEnergyRates specific_energy_rates;
	// Calculate specific energy rate demands in units of (m**2/sec**3)
	// 中文：势能率目标 SPEdot_sp = hdot_sp * g。
	specific_energy_rates.spe_rate.setpoint = control_setpoint.altitude_rate_setpoint *
			CONSTANTS_ONE_G; // potential energy rate of change
	// 中文：动能率目标 SKEdot_sp = V_sp * Vdot_sp；这里使用目标真空速乘目标加速度。
	specific_energy_rates.ske_rate.setpoint = control_setpoint.tas_setpoint *
			control_setpoint.tas_rate_setpoint; // kinetic energy rate of change

	// Calculate specific energy rates in units of (m**2/sec**3)
	// 中文：当前势能率估计 SPEdot = hdot * g。
	specific_energy_rates.spe_rate.estimate = input.altitude_rate * CONSTANTS_ONE_G; // potential energy rate of change
	// 中文：当前动能率估计 SKEdot = V * Vdot。
	specific_energy_rates.ske_rate.estimate = input.tas * input.tas_rate;// kinetic energy rate of change

	return specific_energy_rates;
}

void TECSControl::_detectUnderspeed(const Input &input, const Param &param, const Flag &flag)
{
	if (!flag.detect_underspeed_enabled || !flag.airspeed_enabled) {
		// 中文：没有启用保护或没有空速闭环时，不判断低空速。
		_ratio_undersped = 0.0f;
		return;
	}

	// this is the expected (something like standard) deviation from the airspeed setpoint that we allow the airspeed
	// to vary in before ramping in underspeed mitigation
	const float tas_error_bound = param.tas_error_percentage * param.equivalent_airspeed_trim;

	// this is the soft boundary where underspeed mitigation is ramped in
	// NOTE: it's currently the same as the error bound, but separated here to indicate these values do not in general
	// need to be the same
	const float tas_underspeed_soft_bound = param.tas_error_percentage * param.equivalent_airspeed_trim;

	// 中文：
	// tas_starting_to_underspeed：开始逐渐进入低速保护的 TAS；
	// tas_fully_undersped：完全进入低速保护的 TAS；
	// 当前 TAS 在两者之间时，_ratio_undersped 从 0 线性增加到 1。
	const float tas_fully_undersped = math::max(param.tas_min - tas_error_bound - tas_underspeed_soft_bound, 0.0f);
	const float tas_starting_to_underspeed = math::max(param.tas_min - tas_error_bound, tas_fully_undersped);

	_ratio_undersped = 1.0f - math::constrain((input.tas - tas_fully_undersped) /
			   math::max(tas_starting_to_underspeed - tas_fully_undersped, FLT_EPSILON), 0.0f, 1.0f);
}

TECSControl::SpecificEnergyWeighting TECSControl::_updateSpeedAltitudeWeights(const Param &param, const Flag &flag)
{

	SpecificEnergyWeighting weight;
	// Calculate the weight applied to control of specific kinetic energy error
	// 中文：pitch_speed_weight 范围 0..2：
	// 0 表示 pitch 完全偏向势能/高度，2 表示 pitch 完全偏向动能/空速。
	float pitch_speed_weight = constrain(param.pitch_speed_weight, 0.0f, 2.0f);

	if (_ratio_undersped > FLT_EPSILON && flag.airspeed_enabled) {
		// 中文：低空速时逐渐把 pitch 控制权重拉向 2，也就是优先低头/控空速。
		pitch_speed_weight = 2.0f * _ratio_undersped + (1.0f - _ratio_undersped) * pitch_speed_weight;

	} else if (!flag.airspeed_enabled) {
		// 中文：无空速反馈时，不能可靠控制动能，因此 pitch 强制只保高度。
		pitch_speed_weight = 0.0f;

	} else if (param.fast_descend > FLT_EPSILON) {
		// pitch loop controls the airspeed to max
		// 中文：快速下降时，pitch 逐渐转向保最大空速，油门则逐渐转向最小油门。
		pitch_speed_weight = 1.f + param.fast_descend;

	}

	// 中文：势能权重和动能权重互补，总范围为 0..2。
	weight.spe_weighting = constrain(2.0f - pitch_speed_weight, 0.f, 2.f);
	weight.ske_weighting = constrain(pitch_speed_weight, 0.f, 2.f);

	return weight;
}

void TECSControl::_calcPitchControl(float dt, const Input &input, const SpecificEnergyRates &specific_energy_rates,
				    const Param &param, const Flag &flag)
{
	// 中文：先根据空速/高度权重构造能量平衡率，然后用它更新 pitch。
	const SpecificEnergyWeighting weight{_updateSpeedAltitudeWeights(param, flag)};
	ControlValues seb_rate{_calcPitchControlSebRate(weight, specific_energy_rates)};

	_calcPitchControlUpdate(dt, input, seb_rate, param);
	const float pitch_setpoint{_calcPitchControlOutput(input, seb_rate, param, flag)};

	// Comply with the specified vertical acceleration limit by applying a pitch rate limit
	// NOTE: at zero airspeed, the pitch increment is unbounded
	// 中文：垂向加速度近似 a_z ≈ V * pitch_rate，因此 pitch 每周期最大变化量约为 dt*a_z_max/V。
	const float pitch_increment = dt * param.vert_accel_limit / math::max(input.tas, FLT_EPSILON);
	_pitch_setpoint = constrain(pitch_setpoint, _pitch_setpoint - pitch_increment,
				    _pitch_setpoint + pitch_increment);
	// 中文：最后再用飞行阶段给出的 pitch_min/pitch_max 限幅。
	_pitch_setpoint = constrain(_pitch_setpoint, param.pitch_min, param.pitch_max);

	//Debug Output
	_debug_output.energy_balance_rate_estimate = seb_rate.estimate;
	_debug_output.energy_balance_rate_sp = seb_rate.setpoint;
	_debug_output.pitch_integrator = _pitch_integ_state;
}

TECSControl::ControlValues TECSControl::_calcPitchControlSebRate(const SpecificEnergyWeighting &weight,
		const SpecificEnergyRates &specific_energy_rates) const
{
	ControlValues seb_rate;
	/*
	 * The SKE_weighting variable controls how speed and altitude control are prioritized by the pitch demand calculation.
	 * A weighting of 1 gives equal speed and altitude priority
	 * A weighting of 0 gives 100% priority to altitude control and must be used when no airspeed measurement is available.
	 * A weighting of 2 provides 100% priority to speed control and is used when:
	 * a) an underspeed condition is detected.
	 * b) during climbout where a minimum pitch angle has been set to ensure altitude is gained. If the airspeed
	 * rises above the demanded value, the pitch angle demand is increased by the TECS controller to prevent the vehicle overspeeding.
	 * The weighting can be adjusted between 0 and 2 depending on speed and altitude accuracy requirements.
	*/
	// 中文：
	// pitch 控制的目标不是总能量，而是能量平衡：
	//   SEBdot = SPEdot - SKEdot
	// 若高度需求更强，SPE 权重大；若空速需求更强，SKE 权重大。
	seb_rate.setpoint = specific_energy_rates.spe_rate.setpoint * weight.spe_weighting -
			    specific_energy_rates.ske_rate.setpoint *
			    weight.ske_weighting;

	seb_rate.estimate = (specific_energy_rates.spe_rate.estimate * weight.spe_weighting) -
			    (specific_energy_rates.ske_rate.estimate * weight.ske_weighting);

	return seb_rate;
}

void TECSControl::_calcPitchControlUpdate(float dt, const Input &input, const ControlValues &seb_rate,
		const Param &param)
{
	if (param.integrator_gain_pitch > FLT_EPSILON) {

		// Calculate derivative from change in climb angle to rate of change of specific energy balance
		// 中文：小角度下，改变俯仰/爬升角会改变势能和动能分配，比例近似为 TAS * g。
		const float climb_angle_to_SEB_rate = max(input.tas, param.tas_min) * CONSTANTS_ONE_G;

		// Calculate pitch integrator input term
		// 中文：把能量平衡率误差换算成 pitch 积分输入。
		float pitch_integ_input = _getControlError(seb_rate) * param.integrator_gain_pitch / climb_angle_to_SEB_rate;

		// Prevent the integrator changing in a direction that will increase pitch demand saturation
		// 中文：抗积分饱和。pitch 已到上限时，只允许积分往降低 pitch 的方向变化；下限同理。
		if (_pitch_setpoint >= param.pitch_max) {
			pitch_integ_input = min(pitch_integ_input, 0.f);

		} else if (_pitch_setpoint <= param.pitch_min) {
			pitch_integ_input = max(pitch_integ_input, 0.f);
		}

		// Update the pitch integrator state.
		_pitch_integ_state = _pitch_integ_state + pitch_integ_input * dt;

	} else {
		// 中文：积分增益为 0 时清零积分器，保证参数关闭积分后没有残留。
		_pitch_integ_state = 0.0f;
	}
}

float TECSControl::_calcPitchControlOutput(const Input &input, const ControlValues &seb_rate, const Param &param,
		const Flag &flag) const
{
	float airspeed_for_seb_rate = param.equivalent_airspeed_trim;

	// avoid division by zero by checking if airspeed is finite and greater than zero
	if (flag.airspeed_enabled && PX4_ISFINITE(input.tas) && input.tas > FLT_EPSILON) {
		airspeed_for_seb_rate = input.tas;
	}

	// Calculate derivative from change in climb angle to rate of change of specific energy balance
	const float climb_angle_to_SEB_rate = airspeed_for_seb_rate * CONSTANTS_ONE_G;

	// Calculate a specific energy correction that doesn't include the integrator contribution
	// 中文：能量平衡率修正由两部分组成：
	// 1. 能量平衡率误差阻尼；
	// 2. 能量平衡率目标前馈。
	float SEB_rate_correction = _getControlError(seb_rate) * param.pitch_damping_gain +
				    param.seb_rate_ff *
				    seb_rate.setpoint;

	// Convert the specific energy balance rate correction to a target pitch angle. This calculation assumes:
	// a) The climb angle follows pitch angle with a lag that is small enough not to destabilise the control loop.
	// b) The offset between climb angle and pitch angle (angle of attack) is constant, excluding the effect of
	// pitch transients due to control action or turbulence.
	// 中文：把能量平衡率修正量除以 TAS*g，得到近似需要的俯仰角修正。
	const float pitch_setpoint_unc = SEB_rate_correction / climb_angle_to_SEB_rate + _pitch_integ_state;

	// 中文：输出的是“相对 pitch trim 的俯仰角目标”，外部调用处已经扣除了 FW_PSP_OFF。
	return constrain(pitch_setpoint_unc, param.pitch_min, param.pitch_max);
}

void TECSControl::_calcThrottleControl(float dt, const SpecificEnergyRates &specific_energy_rates, const Param &param,
				       const Flag &flag)
{
	const STERateLimit limit{_calculateTotalEnergyRateLimit(param)};

	// Update STE rate estimate LP filter
	// 中文：当前总能量率估计 STEdot = SPEdot_est + SKEdot_est，经过低通后用于油门阻尼反馈。
	const float STE_rate_estimate_raw = specific_energy_rates.spe_rate.estimate + specific_energy_rates.ske_rate.estimate;
	_ste_rate_estimate_filter.setParameters(dt, param.ste_rate_time_const);
	_ste_rate_estimate_filter.update(STE_rate_estimate_raw);
	ControlValues ste_rate{_calcThrottleControlSteRate(limit, specific_energy_rates, param)};
	float throttle_setpoint{param.throttle_min};

	if (1.f - param.fast_descend < FLT_EPSILON) {
		// During fast descend, we control airspeed over the pitch control loop. Give minimal thrust as soon as we are descending
		// 中文：完全快速下降时，油门直接给最小，空速由 pitch 通道朝最大空速控制。
		throttle_setpoint = param.throttle_min;

	} else {
		// 中文：正常或部分 fast-descend 混合状态，先更新油门积分，再计算油门输出。
		_calcThrottleControlUpdate(dt, limit, ste_rate, param, flag);
		throttle_setpoint = (1.f - param.fast_descend) * _calcThrottleControlOutput(limit, ste_rate, param,
				    flag) + param.fast_descend * param.throttle_min;
	}

	// Rate limit the throttle demand
	// 中文：油门斜率限制，避免 throttle_sp 突变导致推进系统或飞机纵向响应过冲。
	if (fabsf(param.throttle_slewrate) > FLT_EPSILON) {
		const float throttle_increment_limit = dt * (param.throttle_max - param.throttle_min) * param.throttle_slewrate;
		throttle_setpoint = constrain(throttle_setpoint, _throttle_setpoint - throttle_increment_limit,
					      _throttle_setpoint + throttle_increment_limit);
	}

	// 中文：最终油门输出限制在当前飞行阶段允许的 min/max 内。
	_throttle_setpoint = constrain(throttle_setpoint, param.throttle_min, param.throttle_max);

	// Debug output
	_debug_output.total_energy_rate_estimate = ste_rate.estimate;
	_debug_output.total_energy_rate_sp = ste_rate.setpoint;
	_debug_output.throttle_integrator = _throttle_integ_state;
}

TECSControl::ControlValues TECSControl::_calcThrottleControlSteRate(const STERateLimit &limit,
		const SpecificEnergyRates &specific_energy_rates,
		const Param &param) const
{
	// Output ste rate values
	ControlValues ste_rate;
	// 中文：油门控制总能量率：STEdot_sp = SPEdot_sp + SKEdot_sp。
	ste_rate.setpoint = specific_energy_rates.spe_rate.setpoint + specific_energy_rates.ske_rate.setpoint;

	// Adjust the demanded total energy rate to compensate for induced drag rise in turns.
	// Assume induced drag scales linearly with normal load factor.
	// The additional normal load factor is given by (1/cos(bank angle) - 1)
	// 中文：坡度转弯时载荷因子增大，诱导阻力增大，因此给总能量率目标额外加一项油门补偿需求。
	ste_rate.setpoint += param.load_factor_correction * (param.load_factor - 1.f);

	// 中文：总能量率目标不能超过飞机爬升/下沉物理能力。
	ste_rate.setpoint = constrain(ste_rate.setpoint, limit.STE_rate_min, limit.STE_rate_max);
	// 中文：估计值取低通后的 STEdot_est，减少油门反馈对噪声的敏感性。
	ste_rate.estimate = _ste_rate_estimate_filter.getState();

	return ste_rate;
}

void TECSControl::_calcThrottleControlUpdate(float dt, const STERateLimit &limit, const ControlValues &ste_rate,
		const Param &param, const Flag &flag)
{
	// Calculate gain scaler from specific energy rate error to throttle
	// 中文：把总能量率误差归一化为油门量的比例系数。
	const float STE_rate_to_throttle = 1.0f / (limit.STE_rate_max - limit.STE_rate_min);

	// Integral handling
	if (flag.airspeed_enabled) {
		if (param.integrator_gain_throttle > FLT_EPSILON) {
			// underspeed conditions zero out integration
			// 中文：低空速保护时逐渐冻结油门积分，避免积分器在保护逻辑下继续累积。
			float throttle_integ_input = (_getControlError(ste_rate) * param.integrator_gain_throttle) * dt *
						     STE_rate_to_throttle * (1.0f - _ratio_undersped);

			// only allow integrator propagation into direction which unsaturates throttle
			// 中文：抗积分饱和。油门到上限时只允许积分往减小油门方向走；到下限时只允许往增大方向走。
			if (_throttle_setpoint >= param.throttle_max) {
				throttle_integ_input = math::min(0.f, throttle_integ_input);

			} else if (_throttle_setpoint <= param.throttle_min) {
				throttle_integ_input = math::max(0.f, throttle_integ_input);
			}

			// Calculate a throttle demand from the integrated total energy rate error
			// This will be added to the total throttle demand to compensate for steady state errors
			_throttle_integ_state = PX4_ISFINITE(throttle_integ_input) ? _throttle_integ_state + throttle_integ_input : 0.f;

		} else {
			// 中文：关闭油门积分时清零积分状态。
			_throttle_integ_state = 0.0f;
		}

	}
}

float TECSControl::_calcThrottleControlOutput(const STERateLimit &limit, const ControlValues &ste_rate,
		const Param &param,
		const Flag &flag) const
{
	// Calculate gain scaler from specific energy rate error to throttle
	// 中文：将总能量率误差转换成归一化油门修正。
	const float STE_rate_to_throttle = 1.0f / (limit.STE_rate_max - limit.STE_rate_min);

	// Calculate a predicted throttle from the demanded rate of change of energy, using the cruise throttle
	// as the starting point. Assume:
	// Specific total energy rate = _STE_rate_max is achieved when throttle is set to _throttle_setpoint_max
	// Specific total energy rate = 0 at cruise throttle
	// Specific total energy rate = _STE_rate_min is achieved when throttle is set to _throttle_setpoint_min

	// assume airspeed and density-independent delta_throttle to sink/climb rate mapping
	// TODO: include air density for thrust mappings
	// 中文：用分段线性关系建立“总能量率目标 -> 前馈油门”映射：
	// 爬升侧从 throttle_trim 插值到 throttle_max；下降侧从 throttle_trim 插值到 throttle_min。
	const float throttle_above_trim_per_ste_rate = (param.throttle_max - param.throttle_trim) / limit.STE_rate_max;
	const float throttle_below_trim_per_ste_rate = (param.throttle_trim - param.throttle_min) / limit.STE_rate_min;

	float throttle_predicted = 0.0f;

	if (ste_rate.setpoint >= FLT_EPSILON) {
		// throttle is between trim and maximum
		// 中文：需要增加总能量，油门高于配平油门。
		throttle_predicted = param.throttle_trim + ste_rate.setpoint * throttle_above_trim_per_ste_rate;

	} else {
		// throttle is between trim and minimum
		// 中文：需要降低总能量，油门低于配平油门。
		throttle_predicted = param.throttle_trim - ste_rate.setpoint * throttle_below_trim_per_ste_rate;

	}

	// Add proportional and derivative control feedback to the predicted throttle and constrain to throttle limits
	// 中文：在前馈油门基础上加入总能量率误差阻尼反馈。
	float throttle_setpoint = (_getControlError(ste_rate) * param.throttle_damping_gain) * STE_rate_to_throttle +
				  throttle_predicted;

	if (flag.airspeed_enabled) {
		// Add the integrator feedback during closed loop operation with an airspeed sensor
		// 中文：有空速闭环时正常加入油门积分，补偿稳态总能量误差。
		throttle_setpoint += _throttle_integ_state;

	} else {
		/* We want to avoid reducing the throttle output when switching from airspeed enabled mode into airspeedless mode.
		  Thus, if the throttle integrator has a positive value, add it still to the throttle setpoint. */
		// 中文：无空速模式下只保留正积分，避免从有空速切到无空速时油门突然变小。
		throttle_setpoint += math::max(0.0f, _throttle_integ_state);
	}


	// ramp in max throttle setting with underspeediness value
	// 中文：低空速保护会把油门连续混合到最大油门。
	throttle_setpoint = _ratio_undersped * param.throttle_max + (1.0f - _ratio_undersped) * throttle_setpoint;

	return constrain(throttle_setpoint, param.throttle_min, param.throttle_max);
}

void TECSControl::resetIntegrals()
{
	// 中文：两个积分器分别对应 pitch 能量平衡积分和 throttle 总能量积分。
	_pitch_integ_state = 0.0f;
	_throttle_integ_state = 0.0f;
}

void TECS::initControlParams(float target_climbrate, float target_sinkrate, float eas_to_tas, float pitch_limit_max,
			     float pitch_limit_min, float throttle_min, float throttle_setpoint_max, float throttle_trim)
{
	// Update parameters from input
	// Reference model
	// 中文：目标爬升/下沉率来自上层纵向控制配置，可能随飞行阶段变化。
	_reference_param.target_climbrate = target_climbrate;
	_reference_param.target_sinkrate = target_sinkrate;
	// Control
	// 中文：TECSControl 内部使用 TAS，因此把 EAS 上下限乘 eas_to_tas 转换为 TAS 上下限。
	_control_param.tas_min = eas_to_tas * _equivalent_airspeed_min;
	_control_param.tas_max = eas_to_tas * _equivalent_airspeed_max;
	// 中文：pitch/throttle 限制和配平油门由 fw_lateral_longitudinal_control 按当前模式传入。
	_control_param.pitch_max = pitch_limit_max;
	_control_param.pitch_min = pitch_limit_min;
	_control_param.throttle_trim = throttle_trim;
	_control_param.throttle_max = throttle_setpoint_max;
	_control_param.throttle_min = throttle_min;
}

float TECS::calcTrueAirspeedSetpoint(float eas_to_tas, float eas_setpoint)
{
	// 中文：正常情况下 TAS_sp = EAS_sp * eas_to_tas。
	// 快速下降时 _fast_descend 从 0 逐渐到 1，TAS_sp 也逐渐从普通目标混合到 tas_max。
	return lerp(eas_to_tas * eas_setpoint, _control_param.tas_max, _fast_descend);
}

void TECS::initialize(const float altitude, const float altitude_rate, const float equivalent_airspeed,
		      float eas_to_tas)
{
	// Init subclasses
	// 中文：TECS 总初始化由三个子模块同步完成，确保高度参考、空速滤波和控制输出从当前状态开始。
	TECSAltitudeReferenceModel::AltitudeReferenceState current_state{.alt = altitude,
			.alt_rate = altitude_rate};
	_altitude_reference_model.initialize(current_state);
	_airspeed_filter.initialize(equivalent_airspeed, _airspeed_filter_param.equivalent_airspeed_trim,
				    _control_flag.airspeed_enabled);

	TECSControl::Setpoint control_setpoint;
	// 中文：初始化时让控制 setpoint 等于当前参考，避免第一次输出过大。
	control_setpoint.altitude_reference = _altitude_reference_model.getAltitudeReference();
	control_setpoint.altitude_rate_setpoint_direct =
		_altitude_reference_model.getAltitudeReference().alt_rate; // init to reference altitude rate
	control_setpoint.tas_setpoint = equivalent_airspeed * eas_to_tas;

	const TECSControl::Input control_input{ .altitude = altitude,
						.altitude_rate = altitude_rate,
						.tas = eas_to_tas * equivalent_airspeed,
						.tas_rate = 0.0f};

	_control.initialize(control_setpoint, control_input, _control_param, _control_flag);

	// 中文：每次重新初始化 TECS，都退出快速下降状态。
	_fast_descend = 0.f;
	_enabled_fast_descend_timestamp = 0U;
}

void TECS::update(float pitch, float altitude, float hgt_setpoint, float EAS_setpoint, float equivalent_airspeed,
		  float eas_to_tas, float throttle_min, float throttle_setpoint_max,
		  float throttle_trim, float pitch_limit_min, float pitch_limit_max, float target_climbrate,
		  float target_sinkrate, const float speed_deriv_forward, float hgt_rate, float hgt_rate_sp)
{
	// Calculate the time since last update (seconds)
	// 中文：TECS 自己用 hrt 时间计算 dt；调用者不直接传 dt 进 TECS::update。
	const hrt_abstime now(hrt_absolute_time());
	const float dt = static_cast<float>((now - _update_timestamp)) / 1_s;

	// 中文：先刷新本周期限制和参数，使后续滤波/控制都使用当前飞行阶段配置。
	initControlParams(target_climbrate, target_sinkrate, eas_to_tas, pitch_limit_max, pitch_limit_min, throttle_min,
			  throttle_setpoint_max, throttle_trim);

	if (dt < DT_MIN) {
		// Update intervall too small, do not update. Assume constant states/output in this case.
		// 中文：dt 太小会造成微分/滤波数值不可靠，直接保持上一周期输出。
		return;
	}

	if (dt > DT_MAX || _update_timestamp == 0UL) {
		// Update time intervall too large, can't guarantee sanity of state updates anymore. reset the control loop.
		// 中文：首次运行或调度间隔过大时，重新初始化 TECS，避免旧状态跨很长时间继续积分。
		initialize(altitude, hgt_rate, equivalent_airspeed, eas_to_tas);

	} else {
		/* Check if we want to fast descend. On fast descend, we set the throttle to min, and use the altitude control
		loop to control the speed to the maximum airspeed. */
		// 中文：快速下降逻辑会修改 _fast_descend，并影响 TAS_sp、pitch 权重和 throttle 混合。
		_setFastDescend(hgt_setpoint, altitude);
		_control_param.fast_descend = _fast_descend;

		// Update airspeedfilter submodule
		// 中文：输入是 EAS 和 EAS_dot；TECSControl 使用前会再乘 eas_to_tas 转为 TAS。
		const TECSAirspeedFilter::Input airspeed_input{ .equivalent_airspeed = equivalent_airspeed,
				.equivalent_airspeed_rate = speed_deriv_forward / eas_to_tas};

		_airspeed_filter.update(dt, airspeed_input, _airspeed_filter_param, _control_flag.airspeed_enabled);

		// Update Reference model submodule
		if (1.f - _fast_descend < FLT_EPSILON) {
			// Reset the altitude reference model, while we are in fast descend.
			// 中文：完全快速下降时不再追平滑高度参考，而是把参考重置到当前高度/爬升率；
			// 这样 pitch 主要用于控制高速下降中的空速。
			const TECSAltitudeReferenceModel::AltitudeReferenceState init_state{
				.alt = altitude,
				.alt_rate = hgt_rate};
			_altitude_reference_model.initialize(init_state);

		} else {
			// 中文：正常纵向控制时，高度 setpoint 和直接爬升率 setpoint 进入高度参考模型。
			const TECSAltitudeReferenceModel::AltitudeReferenceState setpoint{ .alt = hgt_setpoint,
					.alt_rate = hgt_rate_sp};

			_altitude_reference_model.update(dt, setpoint, altitude, hgt_rate, _reference_param);
		}

		TECSControl::Setpoint control_setpoint;
		// 中文：组装能量控制核心所需 setpoint。
		control_setpoint.altitude_reference = _altitude_reference_model.getAltitudeReference();
		control_setpoint.altitude_rate_setpoint_direct = _altitude_reference_model.getHeightRateSetpointDirect();
		control_setpoint.tas_setpoint = calcTrueAirspeedSetpoint(eas_to_tas, EAS_setpoint);

		// 中文：组装能量控制核心所需状态输入，EAS 滤波状态在这里转换成 TAS。
		const TECSControl::Input control_input{ .altitude = altitude,
							.altitude_rate = hgt_rate,
							.tas = eas_to_tas * _airspeed_filter.getState().speed,
							.tas_rate = eas_to_tas * _airspeed_filter.getState().speed_rate};

		// 中文：真正更新 pitch_sp 和 throttle_sp。
		_control.update(dt, control_setpoint, control_input, _control_param, _control_flag);
	}

	// 中文：保存调试输出。fw_lateral_longitudinal_control 会把这些字段发布到 tecs_status。
	_debug_status.control = _control.getDebugOutput();
	_debug_status.true_airspeed_sp = calcTrueAirspeedSetpoint(eas_to_tas, EAS_setpoint);
	_debug_status.true_airspeed_filtered = eas_to_tas * _airspeed_filter.getState().speed;
	_debug_status.true_airspeed_derivative = eas_to_tas * _airspeed_filter.getState().speed_rate;
	_debug_status.altitude_reference = _altitude_reference_model.getAltitudeReference().alt;
	_debug_status.height_rate_reference = _altitude_reference_model.getAltitudeReference().alt_rate;
	_debug_status.height_rate_direct = _altitude_reference_model.getHeightRateSetpointDirect();
	_debug_status.fast_descend = _fast_descend;

	_update_timestamp = now;
}

void TECS::_setFastDescend(const float alt_setpoint, const float alt)
{
	// disable fast descend if we are close to the target altitude or the altitude setpoint is not finite

	// 中文：
	// 快速下降触发条件：
	//  - 有有效高度目标；
	//  - 空速闭环可用；
	//  - FW_T_F_ALT_ERR 等阈值为正；
	//  - 当前高度比目标高度高出超过阈值。
	// 触发后 TECS 会降低油门，同时把空速目标推向最大空速，使飞机更快下降。
	if (PX4_ISFINITE(alt_setpoint) && _control_flag.airspeed_enabled && (_fast_descend_alt_err > FLT_EPSILON)
	    && ((alt_setpoint + _fast_descend_alt_err) < alt)) {
		auto now = hrt_absolute_time();

		// 中文：第一次进入快速下降时记录时间，用于渐入。
		if (_enabled_fast_descend_timestamp == 0U) {
			_enabled_fast_descend_timestamp = now;
		}

		// 中文：_fast_descend 在 2 秒内从 0 渐入到 1，避免油门和空速目标突变。
		_fast_descend = constrain(max(_fast_descend, static_cast<float>(now - _enabled_fast_descend_timestamp) /
					      static_cast<float>(FAST_DESCEND_RAMP_UP_TIME)), 0.f, 1.f);

	} else if (PX4_ISFINITE(alt_setpoint) && (_fast_descend > FLT_EPSILON) && (_fast_descend_alt_err > FLT_EPSILON)) {
		// Were in fast descend, scale it down. up until 5m above target altitude
		// 中文：已经处于快速下降但接近目标高度时，按剩余高度误差逐渐退出快速下降。
		_fast_descend = constrain((alt - alt_setpoint - 5.f) / _fast_descend_alt_err, 0.f, 1.f);
		_enabled_fast_descend_timestamp = 0U;

	} else {
		// 中文：不满足快速下降条件时完全关闭。
		_fast_descend = 0.f;
		_enabled_fast_descend_timestamp = 0U;
	}
}
