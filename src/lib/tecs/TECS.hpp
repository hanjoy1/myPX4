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
 * @file TECS.hpp
 *
 * 中文说明：
 * TECS（Total Energy Control System，总能量控制系统）是固定翼纵向外环控制器。
 * 它不直接输出舵面，也不直接输出电机 PWM，而是把高度/爬升率目标和空速目标转换为：
 *  - pitch setpoint：俯仰角设定值，后续由 fw_att_control/fw_rate_control 跟踪，最终作用到升降舵；
 *  - throttle setpoint：油门/推力设定值，后续经 control_allocator 输出到电机。
 *
 * TECS 的基本思想：
 *  - 油门主要控制“总能量变化率”：势能变化率 + 动能变化率；
 *  - 俯仰主要控制“能量分配变化率”：势能变化率 - 动能变化率；
 *  - 高度 h 对应势能 g*h，空速 V 对应动能 0.5*V^2。
 *
 * 对 Simulink 复现建议按本文件中的三个子类拆分：
 *  1. TECSAirspeedFilter：空速与空速变化率滤波；
 *  2. TECSAltitudeReferenceModel：高度/爬升率参考轨迹生成；
 *  3. TECSControl：能量控制核心，计算 pitch_sp 和 throttle_sp。
 *
 * @author Paul Riseborough
 */

#pragma once

#include <drivers/drv_hrt.h>
#include <mathlib/mathlib.h>
#include <matrix/math.hpp>
#include <lib/mathlib/math/filter/AlphaFilter.hpp>

#include <uORB/Publication.hpp>
#include <uORB/topics/tecs_status.h>
#include <uORB/uORB.h>
#include <motion_planning/VelocitySmoothing.hpp>
#include <motion_planning/ManualVelocitySmoothingZ.hpp>

using namespace time_literals;

class TECSAirspeedFilter
{
public:
	/**
	 * @brief State of the equivalent airspeed filter.
	 * 中文：等效空速 EAS 滤波器状态。
	 *
	 */
	struct AirspeedFilterState {
		float speed;		///< speed of the air in EAS [m/s]；中文：滤波后的等效空速。
		float speed_rate;	///< rate of speed of the air [m/s²]；中文：滤波后的等效空速变化率。
	};

	/**
	 * @brief Parameters of the airspeed filter.
	 * 中文：空速滤波器参数，决定滤波器相信“空速测量”和“空速变化率测量”的程度。
	 *
	 */
	struct Param {
		float equivalent_airspeed_trim; 	///< the trim value of the equivalent airspeed [m/s]；中文：无有效空速时的配平/巡航空速。
		float airspeed_measurement_std_dev; 	///< airspeed measurement standard deviation in [m/s]；中文：空速测量噪声标准差。
		float airspeed_rate_measurement_std_dev;///< airspeed rate measurement standard deviation in [m/s²]；中文：空速变化率测量噪声标准差。
		float airspeed_rate_noise_std_dev; 	///< standard deviation on the airspeed rate deviation in the model in [m/s²]；中文：模型中空速加速度过程噪声。
	};

	/**
	 * @brief Input, which will be filtered.
	 * 中文：进入空速滤波器的原始输入。
	 *
	 */
	struct Input {
		float equivalent_airspeed; 	///< the measured equivalent airspeed in [m/s]；中文：当前测得/估计的 EAS。
		float equivalent_airspeed_rate;	///< the measured rate of equivalent airspeed in [m/s²]；中文：EAS 变化率，PX4 固定翼当前调用处通常给 0。
	};
public:
	TECSAirspeedFilter() = default;
	~TECSAirspeedFilter() = default;
	/**
	 * @brief Initialize filter
	 * 中文：初始化空速滤波器。若空速可用且有限，用实测空速初始化；否则用配平空速初始化。
	 *
	 * @param[in] equivalent_airspeed is the equivalent airspeed in [m/s].
	 * @param[in] equivalent_airspeed_trim is the equivalent airspeed trim (vehicle setting) in [m/s].
	 * @param[in] airspeed_sensor_available boolean if the airspeed sensor is available.
	 */
	void initialize(float equivalent_airspeed, const float equivalent_airspeed_trim,
			const bool airspeed_sensor_available);

	/**
	 * @brief Update filter
	 * 中文：每个 TECS 周期更新滤波器，输出连续的空速和空速变化率估计。
	 *
	 * @param[in] dt is the timestep in [s].
	 * @param[in] input are the raw measured values.
	 * @param[in] param are the filter parameters.
	 * @param[in] airspeed_sensor_available boolean if the airspeed sensor is available.
	 */
	void update(float dt, const Input &input, const Param &param, const bool airspeed_sensor_available);

	/**
	 * @brief Get the filtered airspeed states.
	 *
	 * @return Current state of the airspeed filter.
	 */
	AirspeedFilterState getState() const;

private:
	// States
	AirspeedFilterState _airspeed_state{.speed = 0.0f, .speed_rate = 0.0f};	///< Complimentary filter state；中文：空速滤波内部状态。
};

class TECSAltitudeReferenceModel
{
public:
	/**
	 * @brief Altitude reference state.
	 * 中文：高度参考模型输出状态。TECS 控制的是这个平滑后的参考，而不是直接跟踪突变 setpoint。
	 *
	 */
	struct AltitudeReferenceState {
		float alt; 	///< Reference altitude amsl in [m]；中文：参考高度，AMSL，高度向上为正。
		float alt_rate;	///< Reference altitude rate in [m/s]；中文：参考爬升率，向上为正。
	};

	/**
	 * @brief Parameters for the reference model.
	 * 中文：高度/爬升率轨迹发生器参数，用来限制参考轨迹的速度、加速度和 jerk。
	 *
	 */
	struct Param {
		float target_climbrate;	///< The target climbrate in [m/s]；中文：期望爬升率设定上限。
		float target_sinkrate;	///< The target sinkrate in [m/s]；中文：期望下沉率设定上限。
		float jerk_max;		///< Magnitude of the maximum jerk allowed [m/s³]；中文：高度参考轨迹的 jerk 限制。
		float vert_accel_limit;	///< Magnitude of the maximum vertical acceleration allowed [m/s²]；中文：垂向加速度限制。
		float max_climb_rate;	///< Climb rate produced by max allowed throttle [m/s]；中文：飞机物理最大爬升率。
		float max_sink_rate;	///< Maximum sink rate (with min throttle, max speed) [m/s]；中文：飞机允许最大下沉率。
	};

public:
	TECSAltitudeReferenceModel() = default;
	~TECSAltitudeReferenceModel() = default;

	/**
	 * @brief Initialize reference models.
	 * 中文：用当前高度和高度变化率初始化两个轨迹发生器，避免 TECS 启动瞬间产生高度阶跃。
	 *
	 * @param[in] state is the current altitude state of the vehicle.
	 */
	void initialize(const AltitudeReferenceState &state);

	/**
	 * @brief Update reference models.
	 * 中文：根据上游给的是“高度目标”还是“爬升率目标”，生成平滑的高度参考。
	 * 如果 setpoint.alt_rate 有限，表示直接爬升率控制；否则走高度闭环参考模型。
	 *
	 * @param[in] dt is the update interval in [s].
	 * @param[in] setpoint are the desired setpoints.
	 * @param[in] altitude is the altitude amsl in [m].
	 * @param[in] height_rate is the height rate setpoint in [m/s].
	 * @param[in] param are the reference model parameters.
	 */
	void update(float dt, const AltitudeReferenceState &setpoint, float altitude, float height_rate, const Param &param);

	/**
	 * @brief Get the current altitude reference of altitude reference model.
	 *
	 * @return Altitude reference state.
	 */
	AltitudeReferenceState getAltitudeReference() const;

	/**
	 * @brief Get the Height Rate Setpoint directly from the velocity trajector generator
	 * 中文：直接爬升率模式下返回生成器输出；若当前是高度控制模式则返回 NAN。
	 *
	 * @return float direct height rate setpoint [m/s]
	 */
	float getHeightRateSetpointDirect() const {return _height_rate_setpoint_direct; }


private:
	// State
	VelocitySmoothing
	_alt_control_traj_generator;		///< Generates altitude rate and altitude setpoint trajectory when altitude is commanded；中文：高度目标模式轨迹发生器。
	ManualVelocitySmoothingZ
	_velocity_control_traj_generator;	///< generates height rate trajectory when height rate is commanded；中文：直接爬升率模式轨迹发生器。
	float _height_rate_setpoint_direct{NAN}; ///< generated direct height rate setpoint；中文：直接爬升率目标，NAN 表示没有使用直接爬升率模式。
};

class TECSControl
{
public:
	/**
	 * @brief The control parameters.
	 * 中文：能量控制核心参数。它们由 fw_lateral_longitudinal_control 根据 PX4 参数、飞行阶段和限制写入。
	 *
	 */
	struct Param {
		// Vehicle specific params
		float max_sink_rate;			///< Maximum sink rate (with min throttle and max speed) [m/s]；中文：最大下沉率，用于总能量下降限幅。
		float min_sink_rate;			///< Minimum sink rate (with min throttle and trim speed) [m/s]；中文：最小下沉率能力，用于最小总能量率。
		float max_climb_rate;			///< Climb rate produced by max allowed throttle [m/s]；中文：最大爬升率，用于总能量上升限幅。
		float vert_accel_limit;			///< Magnitude of the maximum vertical acceleration allowed [m/s²]；中文：垂向加速度限制，最后转成 pitch_sp 变化限制。
		float equivalent_airspeed_trim;		///< Equivalent cruise airspeed for airspeed less mode [m/s]；中文：无空速反馈时用于能量换算的巡航空速。
		float tas_min;				///< True airspeed demand lower limit [m/s]；中文：真空速下限。
		float tas_max;				///< True airspeed demand upper limit [m/s]；中文：真空速上限。
		float pitch_max;			///< Maximum pitch angle above trim allowed in [rad]；中文：最大俯仰角，已扣除 FW_PSP_OFF。
		float pitch_min;			///< Minimal pitch angle below trim allowed in [rad]；中文：最小俯仰角，已扣除 FW_PSP_OFF。
		float throttle_trim;		///< Normalized throttle required to fly level at calibrated airspeed setpoint [0,1]；中文：当前空速/密度下的配平油门。
		float throttle_max;			///< Normalized throttle upper limit；中文：油门上限。
		float throttle_min;			///< Normalized throttle lower limit；中文：油门下限。

		// Altitude control param
		float altitude_error_gain;		///< Altitude error inverse time constant [1/s]；中文：高度误差到爬升率目标的比例增益，等于 1 / FW_T_ALT_TC。
		float altitude_setpoint_gain_ff;	///< Gain from altitude demand derivative to demanded climb rate；中文：高度参考变化率前馈，通常来自 FW_T_HRATE_FF。

		// Airspeed control param
		/// [0,1] percentage of true airspeed trim corresponding to expected (safe) true airspeed tracking errors
		float tas_error_percentage;				///< 中文：underspeed 保护的软边界比例，按巡航空速百分比计算。
		float airspeed_error_gain;				///< Airspeed error inverse time constant [1/s]；中文：空速误差到空速变化率目标的比例增益，等于 1 / FW_T_TAS_TC。

		// Energy control param
		float ste_rate_time_const;				///< Filter time constant for specific total energy rate (damping path) [s]；中文：总能量率估计低通滤波时间常数。
		float seb_rate_ff;					///< Specific energy balance rate feedforward gain；中文：能量平衡率前馈，用在 pitch 控制。

		// Pitch control param
		float pitch_speed_weight;				///< Speed control weighting used by pitch demand calculation；中文：俯仰控制中空速/高度权重，0 保高度，1 折中，2 保空速。
		float integrator_gain_pitch;				///< Integrator gain used by the pitch demand calculation；中文：pitch 能量平衡积分增益。
		float pitch_damping_gain;				///< Damping gain of the pitch demand calculation [s]；中文：pitch 能量平衡误差阻尼增益。

		// Throttle control param
		float integrator_gain_throttle;				///< Integrator gain used by the throttle demand calculation；中文：油门总能量积分增益。
		float throttle_damping_gain;				///< Damping gain of the throttle demand calculation [s]；中文：油门总能量率误差阻尼增益。
		float throttle_slewrate;				///< Throttle demand slew rate limit [1/s]；中文：油门指令斜率限制。

		float load_factor_correction;				///< Gain from normal load factor increase to total energy rate demand [m²/s³]；中文：坡度转弯附加阻力油门补偿。
		float load_factor;					///< Additional normal load factor；中文：载荷因子，通常由滚转角 1/cos(phi) 得到。

		float fast_descend;					///< 中文：快速下降混合比例，0 为正常 TECS，1 为最小油门且 pitch 控最大空速。
	};

	/**
	 * @brief The debug output
	 * 中文：TECS 调试输出，fw_lateral_longitudinal_control 会转发到 tecs_status 话题，方便日志对照。
	 *
	 */
	struct DebugOutput {
		float altitude_rate_control;			///< Altitude rate setpoint from altitude control loop [m/s]；中文：高度误差转换出的爬升率目标。
		float true_airspeed_derivative_control;		///< Airspeed rate setpoint from airspeed control loop [m/s²]；中文：空速误差转换出的真空速变化率目标。
		float total_energy_rate_estimate;		///< Total energy rate estimate [m²/s³]；中文：总能量变化率估计 STEdot = SPEdot + SKEdot。
		float total_energy_rate_sp;			///< Total energy rate setpoint [m²/s³]；中文：总能量变化率目标，主要驱动油门。
		float energy_balance_rate_estimate;		///< Energy balance rate estimate [m²/s³]；中文：能量平衡变化率估计 SEBdot = SPEdot - SKEdot。
		float energy_balance_rate_sp;			///< Energy balance rate setpoint [m²/s³]；中文：能量平衡变化率目标，主要驱动俯仰。
		float pitch_integrator;				///< Pitch control integrator state [-]；中文：俯仰积分状态。
		float throttle_integrator;			///< Throttle control integrator state [-]；中文：油门积分状态。
	};

	/**
	 * @brief Given setpoint to control.
	 * 中文：送入 TECSControl 的参考输入，已经经过高度参考模型和 EAS->TAS 转换。
	 *
	 */
	struct Setpoint {
		TECSAltitudeReferenceModel::AltitudeReferenceState altitude_reference;	///< Altitude/height rate reference；中文：平滑后的高度和爬升率参考。
		float altitude_rate_setpoint_direct;					///< Direct height rate setpoint；中文：直接爬升率控制输入，有限值时绕过高度 P 环。
		float tas_setpoint;						///< True airspeed setpoint；中文：真空速目标，EAS_setpoint * eas2tas 后得到。
	};

	/**
	 * @brief Givent current measurement from the UAS.
	 * 中文：当前飞机纵向状态测量/估计。
	 *
	 */
	struct Input {
		float altitude;		///< Current altitude amsl of the UAS [m]；中文：当前海拔高度。
		float altitude_rate;	///< Current altitude rate of the UAS [m/s]；中文：当前爬升率，向上为正。
		float tas;		///< Current true airspeed of the UAS [m/s]；中文：当前真空速。
		float tas_rate;		///< Current true airspeed rate of the UAS [m/s²]；中文：当前真空速变化率。
	};

	/**
	 * @brief Control flags.
	 * 中文：控制逻辑开关。
	 *
	 */
	struct Flag {
		bool airspeed_enabled;			///< Flag if the airspeed sensor is enabled；中文：是否使用空速闭环。
		bool detect_underspeed_enabled;		///< Flag if underspeed detection is enabled；中文：是否启用低空速保护。
	};
public:
	TECSControl() = default;
	~TECSControl() = default;
	/**
	 * @brief Initialization of the state.
	 * 中文：初始化 TECSControl 的积分器、总能量滤波器以及初始 pitch/throttle 输出。
	 *
	 */
	void initialize(const Setpoint &setpoint, const Input &input, Param &param, const Flag &flag);
	/**
	 * @brief Update state and output.
	 * 中文：能量控制核心更新入口。每次调用都会计算新的 pitch_sp 和 throttle_sp。
	 *
	 * @param[in] dt is the update time intervall in [s].
	 * @param[in] setpoint is the current setpoint struct.
	 * @param[in] input is the current input measurements.
	 * @param[in] param is the current parameter set.
	 * @param[in] flag is the current activated flags.
	 */
	void update(float dt, const Setpoint &setpoint, const Input &input, Param &param, const Flag &flag);
	/**
	 * @brief Reset the control loop integrals.
	 * 中文：清零 pitch 和 throttle 两个积分器，通常在 TECS 初始化/模式切换时使用。
	 *
	 */
	void resetIntegrals();
	/**
	 * @brief Get the percent of the undersped.
	 *
	 * @return Ratio of detected undersped [0,1].
	 */
	float getRatioUndersped() const {return _ratio_undersped;};
	/**
	 * @brief Get the throttle setpoint.
	 *
	 * @return throttle setpoint.
	 */
	float getThrottleSetpoint() const {return _throttle_setpoint;};
	/**
	 * @brief Get the pitch setpoint.
	 *
	 * @return The commanded pitch angle above trim in [rad].
	 */
	float getPitchSetpoint() const {return _pitch_setpoint;};
	/**
	 * @brief Get the Debug Output
	 *
	 * @return the debug outpus struct.
	 */
	const DebugOutput &getDebugOutput() const { return _debug_output; }

private:
	/**
	 * @brief Specific total energy rate limit.
	 * 中文：总能量率限幅。单位是 m²/s³，因为 specific energy 是每单位质量能量。
	 *
	 */
	struct STERateLimit {
		float STE_rate_max;	///< Maximum specific total energy rate limit [m²/s³]；中文：最大总能量增加率，约等于 max_climb_rate * g。
		float STE_rate_min;	///< Minimal specific total energy rate limit [m²/s³]；中文：最小总能量变化率，约等于 -min_sink_rate * g。
	};

	/**
	 * @brief Control values.
	 * setpoint and current state estimate value as input to a controller.
	 * 中文：统一保存“目标值”和“估计值”，方便用 setpoint - estimate 计算误差。
	 *
	 */
	struct ControlValues {
		float setpoint;		///< Control setpoint.
		float estimate;		///< Control estimate of current state value.
	};

	/**
	 * @brief Calculated specific energy rates.
	 * 中文：势能变化率和动能变化率。TECS 后续把它们组合成总能量和能量平衡。
	 *
	 */
	struct SpecificEnergyRates {
		ControlValues ske_rate;	///< Specific kinetic energy rate [m²/s³]；中文：动能变化率，约为 V * Vdot。
		ControlValues spe_rate;	///< Specific potential energy rate [m²/s³]；中文：势能变化率，约为 hdot * g。
	};

	/**
	 * @brief Controlled altitude and pitch setpoints.
	 * 中文：高度/空速外环输出的中间控制目标，尚未转换成能量。
	 *
	 */
	struct AltitudePitchControl {
		float altitude_rate_setpoint;	///< Controlled altitude rate setpoint [m/s]；中文：爬升率目标。
		float tas_rate_setpoint;	///< Controlled true airspeed rate setpoint [m/s²]；中文：真空速变化率目标。
		float tas_setpoint; 		///< Controller true airspeed setpoint [m/s]；中文：真空速目标。
	};

	/**
	 * @brief Weight factors for specific energy.
	 * 中文：势能/动能权重。pitch_speed_weight 越大，俯仰越偏向保空速。
	 *
	 */
	struct SpecificEnergyWeighting {
		float spe_weighting;	///< Specific potential energy weight；中文：高度/势能权重。
		float ske_weighting;	///< Specific kinetic energy weight；中文：空速/动能权重。
	};

private:
	/**
	 * @brief Get control error from etpoint and estimate
	 * 中文：统一误差定义，TECS 中所有能量误差都按 setpoint - estimate。
	 *
	 * @param val is the current control setpoint and estimate.
	 * @return error value
	 */
	static inline constexpr float _getControlError(TECSControl::ControlValues val) {return (val.setpoint - val.estimate);};
	/**
	 * @brief Calculate specific total energy rate limits.
	 * 中文：根据飞机最大爬升率/最小下沉率，把物理能力转换成总能量率上下限。
	 *
	 * @param[in] param are the control parametes.
	 * @return Specific total energy rate limits in [m²/s³].
	 */
	STERateLimit _calculateTotalEnergyRateLimit(const Param &param) const;
	/**
	 * @brief calculate airspeed control proportional output.
	 * 中文：空速外环。把 TAS 误差转换为 TAS 变化率目标，即 Vdot_sp。
	 *
	 * @param setpoint is the control setpoints.
	 * @param input	is the current input measurement of the UAS.
	 * @param param	is the control parameters.
	 * @param flag	is the control flags.
	 * @return controlled airspeed rate setpoint in [m/s²].
	 */
	float _calcAirspeedControlOutput(const Setpoint &setpoint, const Input &input, const Param &param,
					 const Flag &flag) const;
	/**
	 * @brief calculate altitude control proportional output.
	 * 中文：高度外环。把高度误差转换为爬升率目标，即 hdot_sp。
	 *
	 * @param setpoint is the control setpoints.
	 * @param input is the current input measurement of the UAS.
	 * @param param is the control parameters.
	 * @return controlled altitude rate setpoint in [m/s].
	 */
	float _calcAltitudeControlOutput(const Setpoint &setpoint, const Input &input, const Param &param) const;
	/**
	 * @brief Calculate specific energy rates.
	 * 中文：能量变换核心：
	 *  - 势能率 SPEdot = hdot * g；
	 *  - 动能率 SKEdot = TAS * TASdot。
	 *
	 * @param control_setpoint is the controlles altitude and airspeed rate setpoints.
	 * @param input is the current input measurement of the UAS.
	 * @return Specific energy rates in [m²/s³].
	 */
	SpecificEnergyRates _calcSpecificEnergyRates(const AltitudePitchControl &control_setpoint, const Input &input) const;
	/**
	 * @brief Detect underspeed.
	 * 中文：低空速保护检测。输出 _ratio_undersped，0 表示正常，1 表示完全低速保护。
	 *
	 * @param input is the current input measurement of the UAS.
	 * @param param is the control parameters.
	 * @param flag is the control flags.
	 */
	void _detectUnderspeed(const Input &input, const Param &param, const Flag &flag);
	/**
	 * @brief Update specific energy balance weights.
	 * 中文：更新 pitch 控制的高度/空速权重。
	 * 无空速时强制偏向高度；underspeed 时逐渐偏向保空速；fast descend 时也偏向保高速。
	 *
	 * @param param is the control parameters.
	 * @param flag is the control flags.
	 * @return Weights used for the specific energy balance.
	 */
	SpecificEnergyWeighting _updateSpeedAltitudeWeights(const Param &param, const Flag &flag);
	/**
	 * @brief Calculate pitch control.
	 * 中文：俯仰控制主函数。
	 * 俯仰控制“能量平衡率”SEBdot = SPEdot - SKEdot，决定能量在高度和速度之间怎么分配。
	 *
	 * @param dt is the update time intervall in [s].
	 * @param input is the current input measurement of the UAS.
	 * @param specific_energy_rate is the calculated specific energy.
	 * @param param is the control parameters.
	 * @param flag is the control flags.
	 */
	void _calcPitchControl(float dt, const Input &input, const SpecificEnergyRates &specific_energy_rate,
			       const Param &param,
			       const Flag &flag);

	/**
	 * @brief Calculate pitch control specific energy balance rates.
	 * 中文：生成 pitch 控制量的目标/估计能量平衡率：
	 * SEBdot_sp = SPEdot_sp * 高度权重 - SKEdot_sp * 空速权重。
	 *
	 * @param weight is the weighting use of the potential and kinetic energy.
	 * @param specific_energy_rate is the specific energy rates in [m²/s³].
	 * @return specific energy balance rate values in [m²/s³].
	 */
	ControlValues _calcPitchControlSebRate(const SpecificEnergyWeighting &weight,
					       const SpecificEnergyRates &specific_energy_rate) const;

	/**
	 * @brief Calculate the pitch control update function.
	 * Update the states of the pitch control (pitch integrator).
	 * 中文：更新 pitch 积分器，并在 pitch 上/下限饱和时做抗积分饱和。
	 *
	 * @param dt is the update time intervall in [s].
	 * @param input is the current input measurement of the UAS.
	 * @param seb_rate is the specific energy balance rate in [m²/s³].
	 * @param param is the control parameters.
	 */
	void _calcPitchControlUpdate(float dt, const Input &input, const ControlValues &seb_rate, const Param &param);

	/**
	 * @brief Calculate the pitch control output function.
	 * 中文：把能量平衡率修正量转换为俯仰角：
	 * pitch_sp ≈ SEBdot_correction / (TAS * g) + pitch_integrator。
	 *
	 * @param input is the current input measurement of the UAS.
	 * @param seb_rate is the specific energy balance rate in [m²/s³].
	 * @param param is the control parameters.
	 * @param flag is the control flags.
	 * @return pitch setpoint angle above trim [rad].
	 */
	float _calcPitchControlOutput(const Input &input, const ControlValues &seb_rate, const Param &param,
				      const Flag &flag) const;

	/**
	 * @brief Update controlled throttle setpoint.
	 * 中文：油门控制主函数。
	 * 油门控制“总能量率”STEdot = SPEdot + SKEdot，决定飞机总能量增加或减少。
	 *
	 * @param dt is the update time intervall in [s].
	 * @param specific_energy_rate is the calculated specific energy.
	 * @param flag is the control flags.
	 */
	void _calcThrottleControl(float dt, const SpecificEnergyRates &specific_energy_rate, const Param &param,
				  const Flag &flag);

	/**
	 * @brief Calculate throttle control specific total energy
	 * 中文：生成油门控制量的目标/估计总能量率，并叠加坡度转弯载荷补偿。
	 *
	 * @param limit is the specific total energy rate limits in [m²/s³].
	 * @param specific_energy_rate is the specific energy rates in [m²/s³].
	 * @param param is the control parameters.
	 * @return specific total energy rate values in [m²/s³]
	 */
	ControlValues _calcThrottleControlSteRate(const STERateLimit &limit, const SpecificEnergyRates &specific_energy_rate,
			const Param &param) const;

	/**
	 * @brief Calculate the throttle control update function.
	 * Update the throttle control states (throttle integrator).
	 * 中文：更新油门积分器，并在油门上/下限饱和或 underspeed 时抑制积分。
	 *
	 * @param dt is the update time intervall in [s].
	 * @param limit is the specific total energy rate limits in [m²/s³].
	 * @param ste_rate is the specific total energy rates in [m²/s³].
	 * @param param is the control parameters.
	 * @param flag is the control flags.
	 */
	void _calcThrottleControlUpdate(float dt, const STERateLimit &limit, const ControlValues &ste_rate, const Param &param,
					const Flag &flag);

	/**
	 * @brief Calculate the throttle control output function.
	 * 中文：把总能量率目标映射到油门：
	 * 先用爬升/下沉能力做前馈油门，再加总能量率误差阻尼和积分修正。
	 *
	 * @param limit is the specific total energy rate limits in [m²/s³].
	 * @param ste_rate is the specific total energy rates in [m²/s³].
	 * @param param is the control parameters.
	 * @param flag is the control flags.
	 * @return throttle setpoin in [0,1].
	 */
	float _calcThrottleControlOutput(const STERateLimit &limit, const ControlValues &ste_rate, const Param &param,
					 const Flag &flag) const;

private:
	// State
	AlphaFilter<float> _ste_rate_estimate_filter;		///< Low pass filter for the specific total energy rate；中文：总能量率估计低通滤波器。
	float _pitch_integ_state{0.0f};				///< Pitch integrator state [rad]；中文：俯仰积分器状态。
	float _throttle_integ_state{0.0f};			///< Throttle integrator state [-]；中文：油门积分器状态。

	// Output
	DebugOutput _debug_output;				///< Debug output；中文：给 tecs_status 日志使用的内部状态。
	float _pitch_setpoint{0.0f};				///< Controlled pitch setpoint above trim [rad]；中文：TECS 输出俯仰角目标，已扣除 pitch trim 偏置。
	float _throttle_setpoint{0.0f};				///< Controlled throttle setpoint [0,1]；中文：TECS 输出归一化油门目标。
	float _ratio_undersped{0.0f};				///< A continuous representation of how "undersped" the TAS is [0,1]；中文：低空速保护混合比例。
};

class TECS
{
public:
	struct DebugOutput {
		TECSControl::DebugOutput control;	///< 中文：能量控制核心调试量。
		float true_airspeed_sp;		///< 中文：真空速目标，EAS 目标乘 eas2tas 后得到。
		float true_airspeed_filtered;	///< 中文：滤波后的真空速。
		float true_airspeed_derivative;	///< 中文：滤波后的真空速变化率。
		float altitude_reference;	///< 中文：平滑后的高度参考。
		float height_rate_reference;	///< 中文：平滑后的爬升率参考。
		float height_rate_direct;	///< 中文：直接爬升率模式输出；NAN 表示高度模式。
		float fast_descend;		///< 中文：快速下降混合比例。
	};
public:
	TECS() = default;
	~TECS() = default;

	// no copy, assignment, move, move assignment
	TECS(const TECS &) = delete;
	TECS &operator=(const TECS &) = delete;
	TECS(TECS &&) = delete;
	TECS &operator=(TECS &&) = delete;

	const DebugOutput &getStatus() const { return _debug_status; }

	/**
	 * Get the current airspeed status
	 * 中文：返回 TECS 当前是否使用空速闭环。
	 *
	 * @return true if airspeed is enabled for control
	 */
	bool airspeed_sensor_enabled() { return _control_flag.airspeed_enabled; }

	/**
	 * Set the airspeed enable state
	 * 中文：由 fw_lateral_longitudinal_control 根据空速有效性调用。
	 */
	void enable_airspeed(bool enabled) { _control_flag.airspeed_enabled = enabled; }

	/**
	 * @brief Update the control loop calculations
	 * 中文：TECS 总更新入口。
	 * 输入当前 pitch/高度/空速、目标高度/空速、油门/俯仰限制和爬升/下沉限制；
	 * 输出可通过 get_pitch_setpoint() 和 get_throttle_setpoint() 读取。
	 *
	 */
	void update(float pitch, float altitude, float hgt_setpoint, float EAS_setpoint, float equivalent_airspeed,
		    float eas_to_tas, float throttle_min, float throttle_setpoint_max,
		    float throttle_trim, float pitch_limit_min, float pitch_limit_max, float target_climbrate,
		    float target_sinkrate, float speed_deriv_forward, float hgt_rate, float hgt_rate_sp = NAN);

	void resetIntegrals()
	{
		// 中文：外部模块在模式切换或需要清除历史误差时调用，避免积分遗留影响新阶段。
		_control.resetIntegrals();
	}

	void set_detect_underspeed_enabled(bool enabled) { _control_flag.detect_underspeed_enabled = enabled; }; ///< 中文：启用/禁用低空速保护检测。

	// setters for parameters
	// 中文：以下 setter 由 fw_lateral_longitudinal_control::parameters_update() 调用，把 PX4 参数写入 TECS。
	void set_airspeed_measurement_std_dev(float std_dev) {_airspeed_filter_param.airspeed_measurement_std_dev = std_dev;};
	void set_airspeed_rate_measurement_std_dev(float std_dev) {_airspeed_filter_param.airspeed_rate_measurement_std_dev = std_dev;};
	void set_airspeed_filter_process_std_dev(float std_dev) {_airspeed_filter_param.airspeed_rate_noise_std_dev = std_dev;};

	void set_integrator_gain_throttle(float gain) { _control_param.integrator_gain_throttle = gain;};
	void set_integrator_gain_pitch(float gain) { _control_param.integrator_gain_pitch = gain; };

	void set_max_sink_rate(float max_sink_rate) { _control_param.max_sink_rate = max_sink_rate; _reference_param.max_sink_rate = max_sink_rate; };
	void set_min_sink_rate(float min_sink_rate) { _control_param.min_sink_rate = min_sink_rate; };
	void set_max_climb_rate(float climb_rate) { _control_param.max_climb_rate = climb_rate; _reference_param.max_climb_rate = climb_rate; };

	void set_altitude_rate_ff(float altitude_rate_ff) { _control_param.altitude_setpoint_gain_ff = altitude_rate_ff; };
	void set_altitude_error_time_constant(float time_const) { _control_param.altitude_error_gain = 1.0f / math::max(time_const, 0.1f); };
	void set_fast_descend_altitude_error(float altitude_error) { _fast_descend_alt_err = altitude_error; };

	void set_equivalent_airspeed_min(float airspeed) { _equivalent_airspeed_min = airspeed; }
	void set_equivalent_airspeed_max(float airspeed) { _equivalent_airspeed_max = airspeed; }
	void set_equivalent_airspeed_trim(float airspeed) { _control_param.equivalent_airspeed_trim = airspeed; _airspeed_filter_param.equivalent_airspeed_trim = airspeed; }

	void set_pitch_damping(float damping) { _control_param.pitch_damping_gain = damping; }
	void set_vertical_accel_limit(float limit) { _reference_param.vert_accel_limit = limit; _control_param.vert_accel_limit = limit; };

	void set_speed_weight(float weight) { _control_param.pitch_speed_weight = weight; };
	void set_airspeed_error_time_constant(float time_const) { _control_param.airspeed_error_gain = 1.0f / math::max(time_const, 0.1f); };

	void set_throttle_damp(float throttle_damp) { _control_param.throttle_damping_gain = throttle_damp; };
	void set_throttle_slewrate(float slewrate) { _control_param.throttle_slewrate = slewrate; };

	void set_roll_throttle_compensation(float compensation) { _control_param.load_factor_correction = compensation; };
	void set_load_factor(float load_factor) { _control_param.load_factor = load_factor; };

	void set_ste_rate_time_const(float time_const) { _control_param.ste_rate_time_const = time_const; };

	void set_seb_rate_ff_gain(float ff_gain) { _control_param.seb_rate_ff = ff_gain; };

	/**
	 * Handle the altitude reset
	 * 中文：处理 EKF 高度 reset。高度估计若突然跳变，重置 TECS 高度参考，避免 pitch/throttle 出现假误差尖峰。
	 *
	 * If the estimation system resets the height in one discrete step this
	 * will gracefully even out the reset over time.
	 */
	void handle_alt_step(float altitude, float altitude_rate)
	{
		TECSAltitudeReferenceModel::AltitudeReferenceState init_state{ .alt = altitude,
				.alt_rate = altitude_rate};

		// reset altitude reference model.
		// 中文：只重置高度参考模型，不清空空速滤波器和能量控制积分器。
		_altitude_reference_model.initialize(init_state);
	}

	float get_pitch_setpoint() {return _control.getPitchSetpoint();} ///< 中文：读取 TECS 输出俯仰角目标 [rad]。
	float get_throttle_setpoint() {return _control.getThrottleSetpoint();} ///< 中文：读取 TECS 输出油门目标 [0,1]。

	/**
	 * Returns the altitude tracking time constant
	 * 中文：返回高度误差时间常数，即 1 / altitude_error_gain。
	 */
	float get_altitude_error_time_constant() const
	{
		return 1.0f / math::max(_control_param.altitude_error_gain, 0.01f);
	}

	uint64_t timestamp() { return _update_timestamp; }
	float get_underspeed_ratio() { return _control.getRatioUndersped(); }

private:
	/**
	 * @brief Initialize the control parameters
	 * 中文：每个周期根据输入限制更新 TECS 内部参数，例如 EAS->TAS 后的空速上下限、pitch/throttle 限制。
	 *
	 */
	void initControlParams(float target_climbrate, float target_sinkrate, float eas_to_tas, float pitch_limit_max,
			       float pitch_limit_min, float throttle_min, float throttle_setpoint_max, float throttle_trim);

	/**
	 * @brief calculate true airspeed setpoint
	 *
	 * Calculate true airspeed setpoint based on input and fast descend ratio
	 * 中文：EAS 目标转换为 TAS 目标；快速下降时会向 tas_max 混合，促使飞机以更高空速下降。
	 *
	 * @param eas_to_tas is the eas to tas conversion factor
	 * @param eas_setpoint is the desired equivalent airspeed setpoint [m/s]
	 * @return true airspeed setpoint[m/s]
	 */
	float calcTrueAirspeedSetpoint(float eas_to_tas, float eas_setpoint);

	/**
	 * @brief Initialize the control loop
	 * 中文：TECS 总初始化。dt 过大或第一次运行时调用，初始化三个子模块和快速下降状态。
	 *
	 */
	void initialize(const float altitude, const float altitude_rate, const float equivalent_airspeed,
			float eas_to_tas);

	TECSControl 			_control;			///< Control submodule；中文：能量控制核心。
	TECSAirspeedFilter 		_airspeed_filter;		///< Airspeed filter submodule；中文：空速滤波模块。
	TECSAltitudeReferenceModel 	_altitude_reference_model;	///< Setpoint reference model submodule；中文：高度/爬升率参考模型。

	hrt_abstime _update_timestamp{0};				///< last timestamp of the update function call；中文：上次 TECS 更新时刻。

	float _equivalent_airspeed_min{10.0f};				///< equivalent airspeed demand lower limit (m/sec)；中文：EAS 下限，外部参数通常来自 FW_AIRSPD_MIN/STALL。
	float _equivalent_airspeed_max{20.0f};				///< equivalent airspeed demand upper limit (m/sec)；中文：EAS 上限，通常来自 FW_AIRSPD_MAX。
	float _fast_descend_alt_err{-1.f};	 				///< Altitude difference between current altitude to altitude setpoint needed to descend with higher airspeed [m]；中文：触发快速下降的高度误差阈值。
	float _fast_descend{0.f};					///< Value for fast descend in [0,1]. continuous value used to flatten the high speed value out when close to target altitude；中文：快速下降连续混合系数。
	hrt_abstime _enabled_fast_descend_timestamp{0U};		///< timestamp at activation of fast descend mode；中文：快速下降启动时间，用于 2 秒渐入。

	static constexpr float DT_MIN = 0.001f;				///< minimum allowed value of _dt (sec)；中文：更新间隔太小则保持上一周期输出。
	static constexpr float DT_MAX = 1.0f;				///< max value of _dt allowed before a filter state reset is performed (sec)；中文：更新间隔太大则重置 TECS。
	static constexpr hrt_abstime FAST_DESCEND_RAMP_UP_TIME = 2_s; 	///< Ramp up time until fast descend is fully engaged；中文：快速下降从 0 到 1 的渐入时间。

	DebugOutput _debug_status{};

	// Params
	/// Airspeed filter parameters.
	/// 中文：空速滤波默认参数，会被 FW_T_SPD_* 参数覆盖。
	TECSAirspeedFilter::Param _airspeed_filter_param{
		.equivalent_airspeed_trim = 15.0f,
		.airspeed_measurement_std_dev = 0.2f,
		.airspeed_rate_measurement_std_dev = 0.05f,
		.airspeed_rate_noise_std_dev = 0.02f
	};
	/// Reference model parameters.
	/// 中文：高度参考模型默认参数，会被最大爬升/下沉、垂向加速度等参数覆盖。
	TECSAltitudeReferenceModel::Param _reference_param{
		.target_climbrate = 2.0f,
		.target_sinkrate = 2.0f,
		.jerk_max = 1000.0f,
		.vert_accel_limit = 0.0f,
		.max_climb_rate = 2.0f,
		.max_sink_rate = 2.0f,
	};
	/// Control parameters.
	/// 中文：能量控制默认参数。实际飞行中多数由 fixed-wing 参数或飞行阶段动态写入。
	TECSControl::Param _control_param{
		.max_sink_rate = 5.0f,
		.min_sink_rate = 2.0f,
		.max_climb_rate = 5.0f,
		.vert_accel_limit = 0.0f,
		.equivalent_airspeed_trim = 15.0f,
		.tas_min = 10.0f,
		.tas_max = 20.0f,
		.pitch_max = 0.5f,
		.pitch_min = -0.5f,
		.throttle_trim = 0.0f,
		.throttle_max = 1.0f,
		.throttle_min = 0.1f,
		.altitude_error_gain = 0.2f,
		.altitude_setpoint_gain_ff = 0.0f,
		.tas_error_percentage = 0.15f,
		.airspeed_error_gain = 0.1f,
		.ste_rate_time_const = 0.1f,
		.seb_rate_ff = 1.0f,
		.pitch_speed_weight = 1.0f,
		.integrator_gain_pitch = 0.0f,
		.pitch_damping_gain = 0.0f,
		.integrator_gain_throttle = 0.0f,
		.throttle_damping_gain = 0.0f,
		.throttle_slewrate = 0.0f,
		.load_factor_correction = 0.0f,
		.load_factor = 1.0f,
		.fast_descend = 0.f
	};

	TECSControl::Flag _control_flag{
		.airspeed_enabled = false,
		.detect_underspeed_enabled = false,
	};

	/**
	 * @brief Set fast descend value
	 * 中文：根据当前高度是否远高于目标高度，计算快速下降混合比例。
	 *
	 * @param alt_setpoint is the altitude setpoint
	 * @param alt is the current altitude
	 */
	void _setFastDescend(float alt_setpoint, float alt);
};
