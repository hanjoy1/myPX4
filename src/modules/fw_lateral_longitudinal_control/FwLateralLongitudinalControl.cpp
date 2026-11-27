/****************************************************************************
 *
 * Copyright (c) 2025 PX4 Development Team. All rights reserved.
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

#include "FwLateralLongitudinalControl.hpp"
#include <px4_platform_common/events.h>

/*
 * 固定翼横纵向控制模块。
 *
 * 通信链路位置：
 *   fixed_wing_lateral_setpoint      -> 滚转姿态设定值
 *   fixed_wing_longitudinal_setpoint -> 俯仰/油门姿态设定值
 *   合并输出                         -> vehicle_attitude_setpoint
 *
 * 纵向控制由 TECS 完成：高度或高度变化率、等效空速需求被转换为俯仰和油门。
 * 横向控制把航迹角或空速方向需求转换为横向加速度，再转换为滚转角设定值。
 *
 * 这是高层固定翼制导与姿态/角速度内环之间的边界。复现到 Simulink 时，
 * 本模块应直接位于 fw_att_control 上游。
 */

using math::constrain;
using math::max;
using math::radians;

using matrix::Dcmf;
using matrix::Eulerf;
using matrix::Quatf;
using matrix::Vector2f;

// [m/s] maximum reference altitude rate threshhold
// 判定“平飞”的高度参考变化率阈值；TECS 参考高度几乎不动时才认为是 level flight。
static constexpr float MAX_ALT_REF_RATE_FOR_LEVEL_FLIGHT = 0.1f;
// [us] time after which the wind estimate is disabled if no longer updating
// 风估计超过该时间未更新后失效，避免横向制导继续使用过期风场。
static constexpr hrt_abstime WIND_EST_TIMEOUT = 10_s;
// [s] slew rate with which we change altitude time constant
// 低高度条件下 TECS 高度时间常数会渐变，避免参数突变导致俯仰/油门跳变。
static constexpr float TECS_ALT_TIME_CONST_SLEW_RATE = 1.0f;

static constexpr float HORIZONTAL_EVH_FACTOR_COURSE_VALID{3.f}; ///< Factor of velocity standard deviation above which course calculation is considered good enough
static constexpr float HORIZONTAL_EVH_FACTOR_COURSE_INVALID{2.f}; ///< Factor of velocity standard deviation below which course calculation is considered unsafe
static constexpr float COS_HEADING_TRACK_ANGLE_NOT_PUSHED_BACK{0.09f}; ///< Cos of Heading to track angle below which it is assumed that the vehicle is not pushed back by the wind ~cos(85°)
static constexpr float COS_HEADING_TRACK_ANGLE_PUSHED_BACK{0.f}; ///< Cos of Heading to track angle above which it is assumed that the vehicle is pushed back by the wind

// [s] Timeout that has to pass in roll-constraining failsafe before warning is triggered
static constexpr uint64_t ROLL_WARNING_TIMEOUT = 2_s;

// [-] Can-run threshold needed to trigger the roll-constraining failsafe warning
static constexpr float ROLL_WARNING_CAN_RUN_THRESHOLD = 0.9f;

// [m/s/s] slew rate limit for airspeed setpoint changes
// 空速设定值斜率限制，避免 TECS 因空速目标突变而产生过大俯仰/油门瞬态。
static constexpr float ASPD_SP_SLEW_RATE = 1.f;

/*
 * 构造函数。
 *
 * is_vtol=false 时发布普通 vehicle_attitude_setpoint；
 * is_vtol=true 时发布 VTOL 固定翼虚拟姿态 setpoint，供 VTOL 控制链路混合。
 */
FwLateralLongitudinalControl::FwLateralLongitudinalControl(bool is_vtol) :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_attitude_sp_pub(is_vtol ? ORB_ID(fw_virtual_attitude_setpoint) : ORB_ID(vehicle_attitude_setpoint)),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	// 这些状态话题用于日志、调参和上层状态判断，启动时先 advertise。
	_tecs_status_pub.advertise();
	_flight_phase_estimation_pub.advertise();
	_fixed_wing_lateral_status_pub.advertise();

	// 初始化 TECS、性能模型、滚转斜率限制和横向控制参数。
	parameters_update();
	_airspeed_slew_rate_controller.setSlewRate(ASPD_SP_SLEW_RATE);
}

// 析构函数：释放性能计数器。
FwLateralLongitudinalControl::~FwLateralLongitudinalControl()
{
	perf_free(_loop_perf);
}
void
FwLateralLongitudinalControl::parameters_update()
{
	/*
	 * 将 PX4 参数同步到 TECS、性能模型、横向控制限制和空速方向控制器。
	 * 对外环来说，这一步等价于 Simulink 单步计算前加载所有可调常量。
	 */
	updateParams();
	// 性能模型提供最小/最大/巡航空速、最大爬升率、最小下沉率等机型相关能力。
	_performance_model.updateParameters();
	_performance_model.runSanityChecks();

	// 将性能模型和参数中的限制写入 TECS。
	_tecs.set_max_climb_rate(_performance_model.getMaximumClimbRate(_air_density));
	_tecs.set_max_sink_rate(_param_fw_t_sink_max.get());
	_tecs.set_min_sink_rate(_performance_model.getMinimumSinkRate(_air_density));
	_tecs.set_equivalent_airspeed_trim(_performance_model.getCalibratedTrimAirspeed());
	_tecs.set_equivalent_airspeed_min(_performance_model.getMinimumCalibratedAirspeed(getLoadFactor(), _flaps_setpoint));
	_tecs.set_equivalent_airspeed_max(_performance_model.getMaximumCalibratedAirspeed());
	_tecs.set_throttle_damp(_param_fw_t_thr_damping.get());
	_tecs.set_integrator_gain_throttle(_param_fw_t_thr_integ.get());
	_tecs.set_integrator_gain_pitch(_param_fw_t_I_gain_pit.get());
	_tecs.set_throttle_slewrate(_param_fw_thr_slew_max.get());
	_tecs.set_vertical_accel_limit(_param_fw_t_vert_acc.get());
	_tecs.set_roll_throttle_compensation(_param_fw_t_rll2thr.get());
	_tecs.set_pitch_damping(_param_fw_t_ptch_damp.get());
	_tecs.set_altitude_error_time_constant(_param_fw_t_h_error_tc.get());
	_tecs.set_fast_descend_altitude_error(_param_fw_t_fast_alt_err.get());
	_tecs.set_altitude_rate_ff(_param_fw_t_hrate_ff.get());
	_tecs.set_airspeed_error_time_constant(_param_fw_t_tas_error_tc.get());
	_tecs.set_ste_rate_time_const(_param_ste_rate_time_const.get());
	_tecs.set_seb_rate_ff_gain(_param_seb_rate_ff.get());
	_tecs.set_airspeed_measurement_std_dev(_param_speed_standard_dev.get());
	_tecs.set_airspeed_rate_measurement_std_dev(_param_speed_rate_standard_dev.get());
	_tecs.set_airspeed_filter_process_std_dev(_param_process_noise_standard_dev.get());

	_roll_slew_rate.setSlewRate(radians(_param_fw_pn_r_slew_max.get()));

	// TECS 高度时间常数不是直接跳变，而通过 slew rate 平滑改变。
	_tecs_alt_time_const_slew_rate.setSlewRate(TECS_ALT_TIME_CONST_SLEW_RATE);
	_tecs_alt_time_const_slew_rate.setForcedValue(_param_fw_t_h_error_tc.get() * _param_fw_thrtc_sc.get());

	// 空速方向控制器使用 NPFG 周期和阻尼换算 P 增益。
	_airspeed_direction_control.setPGainFromPeriodAndDamping(_param_npfg_damping.get(), _param_npfg_period.get());
}

void FwLateralLongitudinalControl::Run()
{
	/*
	 * 外环主回调。横向制导和 TECS 都需要位置、速度和高度状态，因此该模块由
	 * vehicle_local_position 驱动。横向/纵向设定值会被缓存，使制导发布频率较低时
	 * 控制器仍能持续运行。
	 */
	if (should_exit()) {
		_local_pos_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	/* only run controller if position changed */
	// check for parameter updates
	if (_parameter_update_sub.updated()) {
		// 参数变化时刷新 TECS、性能模型和横向控制配置。
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		parameters_update();
	}

	if (_local_pos_sub.update(&_local_pos)) {

		// 由 local position 时间戳计算外环步长，并限制到 [1ms, 100ms]。
		const float control_interval = math::constrain((_local_pos.timestamp - _last_time_loop_ran) * 1e-6f,
					       0.001f, 0.1f);
		_last_time_loop_ran = _local_pos.timestamp;

		// 读取模式管理器发布的控制器配置覆盖量，或在没有覆盖时使用默认参数。
		updateControllerConfiguration();

		// speed_weight 决定 TECS 在空速保持和高度保持之间的能量分配偏好。
		_tecs.set_speed_weight(_long_configuration.speed_weight);
		// 低高度时增大高度控制响应速度，但通过 slew rate 平滑变化。
		updateTECSAltitudeTimeConstant(checkLowHeightConditions()
					       || _long_configuration.enforce_low_height_condition, control_interval);
		_tecs.set_altitude_error_time_constant(_tecs_alt_time_const_slew_rate.getState());

		if (_vehicle_air_data_sub.updated()) {
			_vehicle_air_data_sub.update();
			// 空气密度影响性能模型估计的爬升/下沉能力，进而影响 TECS 限制。
			_air_density = PX4_ISFINITE(_vehicle_air_data_sub.get().rho) ? _vehicle_air_data_sub.get().rho : _air_density;
			_tecs.set_max_climb_rate(_performance_model.getMaximumClimbRate(_air_density));
			_tecs.set_min_sink_rate(_performance_model.getMinimumSinkRate(_air_density));
		}

		if (_vehicle_landed_sub.updated()) {
			// 落地状态用于 TECS update 中避免错误积分/状态判断。
			vehicle_land_detected_s landed{};
			_vehicle_landed_sub.copy(&landed);
			_landed = landed.landed;
		}

		_flight_phase_estimation_pub.get().flight_phase = flight_phase_estimation_s::FLIGHT_PHASE_UNKNOWN;

		// 控制模式和机型状态决定本周期是否应该运行固定翼外环。
		_vehicle_status_sub.update();
		_control_mode_sub.update();

		if (_flaps_setpoint_sub.updated()) {
			// 襟翼会改变失速/最小空速，性能模型需要用它更新最小等效空速。
			normalized_unsigned_setpoint_s flaps_setpoint{};
			_flaps_setpoint_sub.copy(&flaps_setpoint);
			_flaps_setpoint = flaps_setpoint.normalized_setpoint;
		}

		// 汇总风、空速、高度、姿态等状态，填入 lateral/longitudinal control state。
		update_control_state();

		if (_control_mode_sub.get().flag_control_manual_enabled && _control_mode_sub.get().flag_control_altitude_enabled
		    && _local_pos.z_reset_counter != _z_reset_counter) {
			if (_control_mode_sub.get().flag_control_altitude_enabled && _local_pos.z_reset_counter != _z_reset_counter) {
				// make TECS accept step in altitude and demanded altitude
				// 高度估计发生 reset 时，通知 TECS 同步内部参考，避免因高度跳变产生大俯仰/油门。
				_tecs.handle_alt_step(_long_control_state.altitude_msl, _long_control_state.height_rate);
			}
		}

		// 只有位置/速度/加速度/高度/爬升率控制之一使能，并且处于固定翼或转换阶段，才运行外环。
		const bool should_run = (_control_mode_sub.get().flag_control_position_enabled ||
					 _control_mode_sub.get().flag_control_velocity_enabled ||
					 _control_mode_sub.get().flag_control_acceleration_enabled ||
					 _control_mode_sub.get().flag_control_altitude_enabled ||
					 _control_mode_sub.get().flag_control_climb_rate_enabled) &&
					(_vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING
					 || _vehicle_status_sub.get().in_transition_mode);

		if (should_run) {

			// ----- Longitudinal ------
			float pitch_sp{NAN};
			float throttle_sp{NAN};

			if (_fw_longitudinal_ctrl_sub.updated()) {
				// 纵向 setpoint 来自 fw_mode_manager，包含高度/爬升率/空速/直接俯仰油门等字段。
				_fw_longitudinal_ctrl_sub.copy(&_long_control_sp);
			}

			// 对空速 setpoint 做默认值、风补偿、制导最小空速和斜率限制处理。
			const float airspeed_sp_eas = adapt_airspeed_setpoint(control_interval, _long_control_sp.equivalent_airspeed,
						      _min_airspeed_from_guidance, _lateral_control_state.wind_speed.length());

			// If the both altitude and height rate are set, set altitude setpoint to NAN
			// 高度变化率直接控制优先级更高；如果 height_rate 有效，则高度 setpoint 置 NaN。
			const float altitude_sp = PX4_ISFINITE(_long_control_sp.height_rate) ? NAN : _long_control_sp.altitude;

			/*
			 * 纵向通道：
			 * TECS 接收高度或高度变化率、等效空速、俯仰/油门限制以及爬升/下降目标。
			 * 如果 setpoint 消息没有显式给出 pitch_direct/throttle_direct，
			 * 就使用 TECS 输出的俯仰和油门设定值。
			 */
			tecs_update_pitch_throttle(control_interval, altitude_sp,
						   airspeed_sp_eas,
						   _long_configuration.pitch_min,
						   _long_configuration.pitch_max,
						   _long_configuration.throttle_min,
						   _long_configuration.throttle_max,
						   _long_configuration.sink_rate_target,
						   _long_configuration.climb_rate_target,
						   _long_configuration.disable_underspeed_protection,
						   _long_control_sp.height_rate
						  );

			pitch_sp = PX4_ISFINITE(_long_control_sp.pitch_direct) ? _long_control_sp.pitch_direct : _tecs.get_pitch_setpoint();
			throttle_sp = PX4_ISFINITE(_long_control_sp.throttle_direct) ? _long_control_sp.throttle_direct :
				      _tecs.get_throttle_setpoint();
			// pitch_direct/throttle_direct 用于起飞、降落等特殊阶段；普通巡航使用 TECS 输出。

			// ----- Lateral ------
			float roll_sp {NAN};

			if (_fw_lateral_ctrl_sub.updated()) {
				// We store the update of _fw_lateral_ctrl_sub in a member variable instead of only local such that we can run
				// the controllers also without new setpoints.
				// 横向 setpoint 缓存在成员变量中，即使下一周期没有新消息也能继续按上一目标控制。
				_fw_lateral_ctrl_sub.copy(&_lat_control_sp);
			}

			float airspeed_direction_sp{NAN};
			float lateral_accel_sp {NAN};
			const Vector2f airspeed_vector = _lateral_control_state.ground_speed - _lateral_control_state.wind_speed;
			// 空速矢量 = 地速矢量 - 风速矢量，横向空速方向控制基于该矢量计算航向误差。

			if (PX4_ISFINITE(_lat_control_sp.course) && !PX4_ISFINITE(_lat_control_sp.airspeed_direction)) {
				// only use the course setpoint if it's finite but airspeed_direction is not
				/*
				 * course 表示期望地面航迹方向。有风时，飞机空速矢量必须相对期望航迹
				 * 有一定偏置才能飞出正确地面航迹。CourseToAirspeedRefMapper 负责计算
				 * 这个相对空气的方向，并可能提高 TECS 所需的最小空速。
				 */

				airspeed_direction_sp = _course_to_airspeed.mapCourseSetpointToHeadingSetpoint(
								_lat_control_sp.course, _lateral_control_state.wind_speed,
								airspeed_sp_eas);

				// Note: the here updated _min_airspeed_from_guidance is only used in the next iteration
				// in the longitudinal controller.
				// 这里得到的最小空速会在下一轮纵向控制中生效，用于抵抗逆风导致的倒退。
				const float max_true_airspeed = _performance_model.getMaximumCalibratedAirspeed() * _long_control_state.eas2tas;
				_min_airspeed_from_guidance = _course_to_airspeed.getMinAirspeedForCurrentBearing(
								      _lat_control_sp.course, _lateral_control_state.wind_speed,
								      max_true_airspeed, _param_fw_gnd_spd_min.get())
							      / _long_control_state.eas2tas;

			} else if (PX4_ISFINITE(_lat_control_sp.airspeed_direction)) {
				// If the airspeed_direction is finite we use that instead of the course.
				// 若上游直接给出空速方向，则不再做 course->airspeed_direction 的风修正映射。

				airspeed_direction_sp = _lat_control_sp.airspeed_direction;
				_min_airspeed_from_guidance = 0.f; // reset if no longer in course control

			} else {
				_min_airspeed_from_guidance = 0.f; // reset if no longer in course control
			}

			if (PX4_ISFINITE(airspeed_direction_sp)) {
				// 当前空速方向由空速矢量 atan2 得到，再由控制器输出所需横向加速度。
				const float heading = atan2f(airspeed_vector(1), airspeed_vector(0));
				lateral_accel_sp = _airspeed_direction_control.controlHeading(airspeed_direction_sp, heading,
						   airspeed_vector.norm());
			}

			if (PX4_ISFINITE(_lat_control_sp.lateral_acceleration)) {
				/*
				 * 高层制导可以直接给横向加速度指令。如果航向/航迹控制
				 * 已经算出了横向加速度，这里把直接指令作为附加偏置叠加；
				 * 如果前面没有有效输出，则直接使用该横向加速度指令。
				 */
				lateral_accel_sp = PX4_ISFINITE(lateral_accel_sp) ? lateral_accel_sp + _lat_control_sp.lateral_acceleration :
						   _lat_control_sp.lateral_acceleration;
			}

			if (!PX4_ISFINITE(lateral_accel_sp)) {
				// 没有任何有效横向指令时，输出 0 横向加速度，相当于保持不主动转弯。
				lateral_accel_sp = 0.f; // mitigation if no valid setpoint is received: 0 lateral acceleration
			}

			lateral_accel_sp = getCorrectedLateralAccelSetpoint(lateral_accel_sp);
			lateral_accel_sp = math::constrain(lateral_accel_sp, -_lateral_configuration.lateral_accel_max,
							   _lateral_configuration.lateral_accel_max);
			/*
			 * 固定翼主要通过坡度角产生横向加速度。因此横向控制最后一步
			 * 是把横向加速度需求映射为滚转角设定，再交给姿态控制器。
			 */
			roll_sp = mapLateralAccelerationToRollAngle(lateral_accel_sp);

			fixed_wing_lateral_status_s fixed_wing_lateral_status{};
			fixed_wing_lateral_status.timestamp = hrt_absolute_time();
			fixed_wing_lateral_status.lateral_acceleration_setpoint = lateral_accel_sp;
			fixed_wing_lateral_status.can_run_factor = _can_run_factor;

			_fixed_wing_lateral_status_pub.publish(fixed_wing_lateral_status);

			// additional is_finite checks that should not be necessary, but are kept for safety
			// 额外 NaN 保护：即使上游或控制器异常，也保证发布的姿态 setpoint 有有限值。
			float roll_body = PX4_ISFINITE(roll_sp) ? roll_sp : 0.0f;
			float pitch_body = PX4_ISFINITE(pitch_sp) ? pitch_sp : 0.0f;
			const float yaw_body = _yaw; // yaw is not controlled in fixed wing, need to set it though for quaternion generation
			const float thrust_body_x = PX4_ISFINITE(throttle_sp) ? throttle_sp : 0.0f;

			if (_control_mode_sub.get().flag_control_manual_enabled) {
				// 手动高度/位置模式下再次限制滚转和俯仰，确保不会超过人工可接受的固定翼限制。
				roll_body = constrain(roll_body, -radians(_param_fw_r_lim.get()),
						      radians(_param_fw_r_lim.get()));
				pitch_body = constrain(pitch_body, radians(_param_fw_p_lim_min.get()),
						       radians(_param_fw_p_lim_max.get()));
			}

			// roll slew rate
			// 对滚转 setpoint 做斜率限制，避免航迹切换时滚转角阶跃过大。
			roll_body = _roll_slew_rate.update(roll_body, control_interval);

			// 将 roll/pitch/yaw 重新打包成姿态四元数，供 fw_att_control 消费。
			_att_sp.timestamp = hrt_absolute_time();
			const Quatf q(Eulerf(roll_body, pitch_body, yaw_body));
			q.copyTo(_att_sp.q_d);

			_att_sp.thrust_body[0] = thrust_body_x;

			// 发布姿态+油门 setpoint，进入固定翼姿态内环。
			_attitude_sp_pub.publish(_att_sp);

		}

		_z_reset_counter = _local_pos.z_reset_counter;
	}

	perf_end(_loop_perf);
}

void FwLateralLongitudinalControl::updateControllerConfiguration()
{
	/*
	 * 更新横向/纵向控制器配置。
	 * 上游可通过 configuration topic 临时覆盖默认限制；没有覆盖时使用参数默认值。
	 */
	if (_lateral_configuration.timestamp == 0) {
		// 首次运行时，用最大滚转角换算横向最大加速度：a_y = g * tan(phi_max)。
		_lateral_configuration.timestamp = _local_pos.timestamp;
		_lateral_configuration.lateral_accel_max = tanf(radians(_param_fw_r_lim.get())) * CONSTANTS_ONE_G;

	}

	if (_long_configuration.timestamp == 0) {
		// 首次运行时填充纵向控制默认限制，包括俯仰、油门、爬升/下沉等。
		setDefaultLongitudinalControlConfiguration();
	}

	if (_long_control_configuration_sub.updated() || _parameter_update_sub.updated()) {
		// 纵向配置变化或参数变化时重新计算纵向控制限制。
		longitudinal_control_configuration_s configuration_in{};
		_long_control_configuration_sub.copy(&configuration_in);
		updateLongitudinalControlConfiguration(configuration_in);
	}

	if (_lateral_control_configuration_sub.updated() || _parameter_update_sub.updated()) {
		// 横向配置变化或参数变化时重新计算横向最大加速度。
		lateral_control_configuration_s configuration_in{};
		_lateral_control_configuration_sub.copy(&configuration_in);
		_lateral_configuration.timestamp = configuration_in.timestamp;

		if (PX4_ISFINITE(configuration_in.lateral_accel_max)) {
			// 上游配置不能突破 FW_R_LIM 对应的物理/安全滚转限制。
			_lateral_configuration.lateral_accel_max = min(configuration_in.lateral_accel_max, tanf(radians(
						_param_fw_r_lim.get())) * CONSTANTS_ONE_G);

		} else {
			_lateral_configuration.lateral_accel_max = tanf(radians(_param_fw_r_lim.get())) * CONSTANTS_ONE_G;
		}
	}
}

void
FwLateralLongitudinalControl::tecs_update_pitch_throttle(const float control_interval, float alt_sp, float airspeed_sp,
		float pitch_min_rad, float pitch_max_rad, float throttle_min,
		float throttle_max, const float desired_max_sinkrate,
		const float desired_max_climbrate,
		bool disable_underspeed_detection, float hgt_rate_sp)
{
	/*
	 * TECS 更新入口。
	 * 输入：高度/高度变化率、等效空速、俯仰/油门限制、爬升/下沉限制。
	 * 输出：内部 TECS pitch/throttle setpoint，并发布 tecs_status 供日志观察。
	 */
	bool tecs_is_running = true;

	// do not run TECS if vehicle is a VTOL and we are in rotary wing mode or in transition
	if (_vehicle_status_sub.get().is_vtol
	    && (_vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
		|| _vehicle_status_sub.get().in_transition_mode)) {
		// VTOL 旋翼模式或转换模式下不运行固定翼 TECS，避免和 VTOL 控制逻辑冲突。
		tecs_is_running = false;
		return;

	}

	const float throttle_trim_compensated = _performance_model.getTrimThrottle(throttle_min,
						throttle_max, airspeed_sp, _air_density);
	// 性能模型根据空速、空气密度和油门限制估算巡航/配平油门。

	_tecs.set_detect_underspeed_enabled(!disable_underspeed_detection);
	// 某些特殊阶段可关闭 underspeed 保护，例如起飞/降落的直接控制阶段。

	// HOTFIX: the airspeed rate estimate using acceleration in body-forward direction has shown to lead to high biases
	// when flying tight turns. It's in this case much safer to just set the estimated airspeed rate to 0.
	const float airspeed_rate_estimate = 0.f;

	// 调用 TECS 主更新，注意俯仰角和俯仰限制都扣除了平飞俯仰偏置 FW_PSP_OFF。
	_tecs.update(_long_control_state.pitch_rad - radians(_param_fw_psp_off.get()),
		     _long_control_state.altitude_msl,
		     alt_sp,
		     airspeed_sp,
		     _long_control_state.airspeed_eas,
		     _long_control_state.eas2tas,
		     throttle_min,
		     throttle_max,
		     throttle_trim_compensated,
		     pitch_min_rad - radians(_param_fw_psp_off.get()),
		     pitch_max_rad - radians(_param_fw_psp_off.get()),
		     desired_max_climbrate,
		     desired_max_sinkrate,
		     airspeed_rate_estimate,
		     _long_control_state.height_rate,
		     hgt_rate_sp);

	tecs_status_publish(alt_sp, airspeed_sp, airspeed_rate_estimate, throttle_trim_compensated);

	if (tecs_is_running && !_vehicle_status_sub.get().in_transition_mode
	    && (_vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING)) {
		// 根据 TECS 参考高度/高度变化率粗略估计当前处于爬升、下降还是平飞阶段。
		const TECS::DebugOutput &tecs_output{_tecs.getStatus()};

		// Check level flight: the height rate setpoint is not set or set to 0 and we are close to the target altitude and target altitude is not moving
		if ((fabsf(tecs_output.height_rate_reference) < MAX_ALT_REF_RATE_FOR_LEVEL_FLIGHT) &&
		    fabsf(_long_control_state.altitude_msl - tecs_output.altitude_reference) < _param_nav_fw_alt_rad.get()) {
			_flight_phase_estimation_pub.get().flight_phase = flight_phase_estimation_s::FLIGHT_PHASE_LEVEL;

		} else if (((tecs_output.altitude_reference - _long_control_state.altitude_msl) >= _param_nav_fw_alt_rad.get()) ||
			   (tecs_output.height_rate_reference >= MAX_ALT_REF_RATE_FOR_LEVEL_FLIGHT)) {
			_flight_phase_estimation_pub.get().flight_phase = flight_phase_estimation_s::FLIGHT_PHASE_CLIMB;

		} else if (((_long_control_state.altitude_msl - tecs_output.altitude_reference) >= _param_nav_fw_alt_rad.get()) ||
			   (tecs_output.height_rate_reference <= -MAX_ALT_REF_RATE_FOR_LEVEL_FLIGHT)) {
			_flight_phase_estimation_pub.get().flight_phase = flight_phase_estimation_s::FLIGHT_PHASE_DESCEND;

		} else {
			//We can't infer the flight phase , do nothing, estimation is reset at each step
		}
	}
}

void
FwLateralLongitudinalControl::tecs_status_publish(float alt_sp, float equivalent_airspeed_sp,
		float true_airspeed_derivative_raw, float throttle_trim)
{
	/*
	 * 发布 TECS 调试/状态消息。
	 * 这些字段基本是一比一转发 TECS 内部状态，便于日志分析和 Simulink 对照验证。
	 */
	tecs_status_s tecs_status{};

	const TECS::DebugOutput &debug_output{_tecs.getStatus()};

	tecs_status.altitude_sp = alt_sp;
	tecs_status.altitude_reference = debug_output.altitude_reference;
	tecs_status.altitude_time_constant = _tecs.get_altitude_error_time_constant();
	tecs_status.height_rate_reference = debug_output.height_rate_reference;
	tecs_status.height_rate_direct = debug_output.height_rate_direct;
	tecs_status.height_rate_setpoint = debug_output.control.altitude_rate_control;
	tecs_status.height_rate = -_local_pos.vz;
	tecs_status.equivalent_airspeed_sp = equivalent_airspeed_sp;
	tecs_status.true_airspeed_sp = debug_output.true_airspeed_sp;
	tecs_status.true_airspeed_filtered = debug_output.true_airspeed_filtered;
	tecs_status.true_airspeed_derivative_sp = debug_output.control.true_airspeed_derivative_control;
	tecs_status.true_airspeed_derivative = debug_output.true_airspeed_derivative;
	tecs_status.true_airspeed_derivative_raw = true_airspeed_derivative_raw;
	tecs_status.total_energy_rate = debug_output.control.total_energy_rate_estimate;
	tecs_status.total_energy_balance_rate = debug_output.control.energy_balance_rate_estimate;
	tecs_status.total_energy_rate_sp = debug_output.control.total_energy_rate_sp;
	tecs_status.total_energy_balance_rate_sp = debug_output.control.energy_balance_rate_sp;
	tecs_status.throttle_integ = debug_output.control.throttle_integrator;
	tecs_status.pitch_integ = debug_output.control.pitch_integrator;
	tecs_status.throttle_sp = _tecs.get_throttle_setpoint();
	tecs_status.pitch_sp_rad = _tecs.get_pitch_setpoint();
	tecs_status.throttle_trim = throttle_trim;
	tecs_status.underspeed_ratio = _tecs.get_underspeed_ratio();
	tecs_status.fast_descend_ratio = debug_output.fast_descend;

	tecs_status.timestamp = hrt_absolute_time();

	// 发布后可在 ulog 中查看高度、空速、能量、积分和 underspeed 等 TECS 内部量。
	_tecs_status_pub.publish(tecs_status);
}

int FwLateralLongitudinalControl::task_spawn(int argc, char *argv[])
{
	// 模块启动入口；带 "vtol" 参数时发布 VTOL 固定翼虚拟姿态 setpoint。
	bool is_vtol = false;

	if (argc > 1) {
		if (strcmp(argv[1], "vtol") == 0) {
			is_vtol = true;
		}
	}

	FwLateralLongitudinalControl *instance = new FwLateralLongitudinalControl(is_vtol);

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

bool
FwLateralLongitudinalControl::init()
{
	// 注册 local_position 回调：位置估计一更新，本模块就被调度运行。
	if (!_local_pos_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

int FwLateralLongitudinalControl::custom_command(int argc, char *argv[])
{
	// 该模块目前没有额外 CLI 子命令；任何未知命令都打印使用说明。
	return print_usage("unknown command");
}

int FwLateralLongitudinalControl::print_usage(const char *reason)
{
	// 输出 nsh/module 命令行帮助，便于在 PX4 shell 中启动或排查模块。
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
fw_lat_lon_control computes attitude and throttle setpoints from lateral and longitudinal control setpoints.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("fw_lat_lon_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

void FwLateralLongitudinalControl::update_control_state() {
	/*
	 * 汇总控制器本周期所需的状态量。PX4 中这些量来自不同 uORB topic；
	 * 复现时建议也拆成“高度/空速/姿态/风”四个输入模块，最后汇总成控制状态总线。
	 */
	updateAltitudeAndHeightRate();
	updateAirspeed();
	updateAttitude();
	updateWind();

	// 地速只取水平 NE 分量，用于 NPFG 横向制导。
	_lateral_control_state.ground_speed = Vector2f(_local_pos.vx, _local_pos.vy);
}

void FwLateralLongitudinalControl::updateWind() {
	// 更新风估计。NPFG 可使用风速把地速/空速关系处理得更准确。
	if (_wind_sub.updated()) {
		wind_s wind{};
		_wind_sub.update(&wind);

		// assumes wind is valid if finite
		// 中文：PX4 这里用有限数作为风估计有效性的基本判据。
		_wind_valid = PX4_ISFINITE(wind.windspeed_north)
			      && PX4_ISFINITE(wind.windspeed_east);

		_time_wind_last_received = hrt_absolute_time();

		_lateral_control_state.wind_speed(0) = wind.windspeed_north;
		_lateral_control_state.wind_speed(1) = wind.windspeed_east;

	} else {
		// invalidate wind estimate usage (and correspondingly NPFG, if enabled) after subscription timeout
		// 中文：超过超时时间没有收到风估计，则停止使用旧风速，避免横向制导依赖过期信息。
		_wind_valid = _wind_valid && (hrt_absolute_time() - _time_wind_last_received) < WIND_EST_TIMEOUT;
	}

	if (!_wind_valid) {
		// 无有效风估计时，显式置零；下游逻辑仍会根据 _wind_valid 区分“无风”和“未知风”。
		_lateral_control_state.wind_speed.setZero();
	}
}

void FwLateralLongitudinalControl::updateAltitudeAndHeightRate() {
	// local_position 使用 NED 坐标，z 向下；TECS 使用高度向上，因此这里要取负号。
	float ref_alt{0.f};

	if (_local_pos.z_global && PX4_ISFINITE(_local_pos.ref_alt)) {
		// 有全局高度参考时，把局部 z 转换为 AMSL 高度。
		ref_alt = _local_pos.ref_alt;
	}

	_long_control_state.altitude_msl = -_local_pos.z + ref_alt; // Altitude AMSL in meters
	// 中文：AMSL 表示海平面以上高度，单位 m。
	_long_control_state.height_rate = -_local_pos.vz;

}

void FwLateralLongitudinalControl::updateAttitude() {
	// 读取当前姿态并转换为 TECS/横向控制需要的欧拉角。
	vehicle_attitude_s att;

	if (_vehicle_attitude_sub.update(&att)) {

		Dcmf R{Quatf(att.q)};

		// if the vehicle is a tailsitter we have to rotate the attitude by the pitch offset
		// between multirotor and fixed wing flight
		// 中文：尾座式 VTOL 在多旋翼/固定翼坐标定义之间差 90 度俯仰，这里旋转到固定翼语义。
		if (_vehicle_status_sub.get().is_vtol_tailsitter) {
			const Dcmf R_offset{Eulerf{0.f, M_PI_2_F, 0.f}};
			R = R * R_offset;
		}

		// TECS 只直接需要俯仰；横向航迹保护还会用到航向角。
		const Eulerf euler_angles(R);
		_long_control_state.pitch_rad = euler_angles.theta();
		_yaw = euler_angles.psi();

		// load factor due to banking
		// 中文：坡度越大，维持高度所需升力越大；TECS 用载荷因子补偿爬升/油门需求。
		const float load_factor_from_bank_angle = 1.0f / max(cosf(euler_angles.phi()), FLT_EPSILON);
		_tecs.set_load_factor(load_factor_from_bank_angle);
	}
}

void FwLateralLongitudinalControl::updateAirspeed() {
	// 读取 validated airspeed，并决定 TECS 是否启用空速闭环。

	airspeed_validated_s airspeed_validated;

	if (_param_fw_use_airspd.get() && _airspeed_validated_sub.update(&airspeed_validated)) {

		// do not use synthetic airspeed as this would create a thrust loop
		// 中文：synthetic airspeed 往往由推力/模型推断，若反馈给 TECS 会形成不真实的推力闭环。
		if (PX4_ISFINITE(airspeed_validated.calibrated_airspeed_m_s)
		    && PX4_ISFINITE(airspeed_validated.true_airspeed_m_s)
		    && airspeed_validated.airspeed_source != airspeed_validated_s::SOURCE_SYNTHETIC) {

			_time_airspeed_last_valid = airspeed_validated.timestamp;
			_long_control_state.airspeed_eas = airspeed_validated.calibrated_airspeed_m_s;
			// EAS 到 TAS 的比例受限，避免传感器瞬态或低空速除法把 TECS 比例放大到异常。
			_long_control_state.eas2tas = constrain(airspeed_validated.true_airspeed_m_s / airspeed_validated.calibrated_airspeed_m_s, 0.9f, 2.0f);
		}
	}

	// no airspeed updates for one second --> declare invalid
	// 中文：超过 1 秒无有效更新则退化为空速不可用模式。
	const bool airspeed_valid = hrt_elapsed_time(&_time_airspeed_last_valid) < 1_s;

	if (!airspeed_valid) {
		// 无有效空速时按 EAS=TAS 近似，避免遗留比例影响后续周期。
		_long_control_state.eas2tas = 1.f;
	}

	// 通知 TECS 当前是否可以依赖空速反馈。
	_tecs.enable_airspeed(airspeed_valid);
}

float
FwLateralLongitudinalControl::adapt_airspeed_setpoint(const float control_interval, float calibrated_airspeed_setpoint,
		float calibrated_min_airspeed_guidance, float wind_speed)
{
	/*
	 * 空速设定值适配：
	 * 1. 无有效上游空速命令时使用巡航/配平空速；
	 * 2. 根据风和横向制导需求抬高最小空速；
	 * 3. 限幅到性能模型允许范围；
	 * 4. 用 slew-rate 控制器限制空速设定突变。
	 */
	float system_min_airspeed = _performance_model.getMinimumCalibratedAirspeed(getLoadFactor(), _flaps_setpoint);

	const float system_max_airspeed = _performance_model.getMaximumCalibratedAirspeed();

	// airspeed setpoint adjustments
	// 中文：上游没有明确空速 setpoint 时，回退到飞机性能模型给出的配平空速。
	if (!PX4_ISFINITE(calibrated_airspeed_setpoint) || calibrated_airspeed_setpoint <= FLT_EPSILON) {
		calibrated_airspeed_setpoint = _performance_model.getCalibratedTrimAirspeed();

		// Aditional option to increase the min airspeed setpoint based on wind estimate for more stability in higher winds.
		// 中文：有风且启用比例时，提高最小空速，增强大风下的航迹保持余量。
		if (_wind_valid && _param_fw_wind_arsp_sc.get() > FLT_EPSILON) {
			system_min_airspeed = math::min(system_min_airspeed + _param_fw_wind_arsp_sc.get() *
			wind_speed, system_max_airspeed);
		}
	}

	// increase setpoint to at what's at least required for the lateral guidance
	// 中文：横向制导可能需要更高空速才能满足期望转弯半径/横向加速度。
	calibrated_airspeed_setpoint = math::max(calibrated_airspeed_setpoint, calibrated_min_airspeed_guidance);

	// constrain airspeed to feasible range
	// 中文：最终空速命令必须落在性能模型的最小/最大可飞范围内。
	calibrated_airspeed_setpoint = math::constrain(calibrated_airspeed_setpoint, system_min_airspeed, system_max_airspeed);

	if (!PX4_ISFINITE(_airspeed_slew_rate_controller.getState())) {

		// initialize the airspeed setpoint
		// 中文：首次运行时用当前空速初始化斜率限制器，可避免 setpoint 突跳。
		if (PX4_ISFINITE(_long_control_state.airspeed_eas) && _long_control_state.airspeed_eas < system_min_airspeed) {
			// current airpseed is below minimum - init with minimum
			// 中文：当前空速低于下限，直接从下限开始。
			_airspeed_slew_rate_controller.setForcedValue(system_min_airspeed);

		} else if (PX4_ISFINITE(_long_control_state.airspeed_eas) && _long_control_state.airspeed_eas > system_max_airspeed) {
			// current airpseed is above maximum - init with maximum
			// 中文：当前空速高于上限，直接从上限开始。
			_airspeed_slew_rate_controller.setForcedValue(system_max_airspeed);

		} else if (PX4_ISFINITE(_long_control_state.airspeed_eas)) {
			// current airpseed is between min and max - init with current
			// 中文：当前空速在范围内，则保持连续性，从实测空速开始。
			_airspeed_slew_rate_controller.setForcedValue(_long_control_state.airspeed_eas);

		} else {
			// current airpseed is invalid - init with setpoint
			// 中文：没有实测空速时，只能从目标空速初始化。
			_airspeed_slew_rate_controller.setForcedValue(calibrated_airspeed_setpoint);
		}
	} else {
		// update slew rate state
		// 中文：后续周期按最大变化率逐步逼近目标空速，同时守住范围边界。
		if (_airspeed_slew_rate_controller.getState() < system_min_airspeed) {
			// current airpseed setpoint is below minimum - reset to minimum
			// 中文：斜率状态若因参数变化落到下限以下，强制夹回下限。
			_airspeed_slew_rate_controller.setForcedValue(system_min_airspeed);

		} else if (_airspeed_slew_rate_controller.getState() > system_max_airspeed) {
			// current airpseed setpoint is above maximum - reset to maximum
			// 中文：斜率状态若高于上限，强制夹回上限。
			_airspeed_slew_rate_controller.setForcedValue(system_max_airspeed);

		} else if (PX4_ISFINITE(_long_control_state.airspeed_eas)) {
			// current airpseed setpoint is between min and max - update
			// 中文：有真实空速状态时才进行正常斜率更新。
			_airspeed_slew_rate_controller.update(calibrated_airspeed_setpoint, control_interval);

		}
	}

	return _airspeed_slew_rate_controller.getState();
}

bool FwLateralLongitudinalControl::checkLowHeightConditions() const
{
	// Are conditions for low-height
	// 中文：判断是否处于近地高度，用于 TECS 低高度时的油门/高度响应调整。
	return _param_fw_t_thr_low_hgt.get() >= 0.f && _local_pos.dist_bottom_valid
	       && _local_pos.dist_bottom < _param_fw_t_thr_low_hgt.get();
}

void FwLateralLongitudinalControl::updateTECSAltitudeTimeConstant(const bool is_low_height, const float dt)
{
	// Target time constant for the TECS altitude tracker
	// 中文：TECS 高度误差时间常数越小，追高度越积极；这里可在低高度时按比例调整。
	float alt_tracking_tc = _param_fw_t_h_error_tc.get();

	if (is_low_height) {
		// If low-height conditions satisfied, compute target time constant for altitude tracking
		// 中文：低高度条件满足时，通过参数缩放时间常数，让近地段高度控制特性不同于巡航段。
		alt_tracking_tc *= _param_fw_thrtc_sc.get();
	}

	// 对时间常数本身做斜率限制，避免 TECS 内部动态突然改变。
	_tecs_alt_time_const_slew_rate.update(alt_tracking_tc, dt);
}

float FwLateralLongitudinalControl::getGuidanceQualityFactor(const vehicle_local_position_s &local_pos, const bool is_wind_valid) const
{
	/*
	 * 计算横向制导可信度系数。
	 * 有有效风估计时 NPFG 可自行处理退化情况；无风估计时需要检查地速质量和
	 * “机头方向是否基本仍向前飞”，否则地速航迹可能与真实空速方向相反。
	 */
	if (is_wind_valid) {
		// If we have a valid wind estimate, npfg is able to handle all degenerated cases
		// 中文：有风估计时，NPFG 可由空速/地速/风三者恢复出合理制导关系。
		return 1.f;
	}

	// NPFG can run without wind information as long as the system is not flying backwards and has a minimal ground speed
	// Check the minimal ground speed. if it is greater than twice the standard deviation, we assume that we can infer a valid track angle
	// 中文：地速要明显大于水平速度估计误差，才认为航迹角可信。
	const Vector2f ground_vel(local_pos.vx, local_pos.vy);
	const float ground_speed(ground_vel.norm());
	const float low_ground_speed_factor(math::constrain((ground_speed - HORIZONTAL_EVH_FACTOR_COURSE_INVALID *
									    local_pos.evh) / ((HORIZONTAL_EVH_FACTOR_COURSE_VALID - HORIZONTAL_EVH_FACTOR_COURSE_INVALID)*local_pos.evh),
							    0.f, 1.f));

	// Check that the angle between heading and track is not off too much. if it is greater than 90° we will be pushed back from the wind and the npfg will propably give a roll command in the wrong direction.
	// 中文：若航向与地速方向夹角过大，可能是逆风倒退；此时横向滚转指令需要降额。
	const Vector2f heading_vector(matrix::Dcm2f(local_pos.heading)*Vector2f({1.f, 0.f}));
	const Vector2f ground_vel_norm(ground_vel.normalized());
	const float flying_forward_factor(math::constrain((heading_vector.dot(ground_vel_norm) -
							   COS_HEADING_TRACK_ANGLE_PUSHED_BACK) / ((COS_HEADING_TRACK_ANGLE_NOT_PUSHED_BACK -
												    COS_HEADING_TRACK_ANGLE_PUSHED_BACK)),
							  0.f, 1.f));

	return flying_forward_factor * low_ground_speed_factor;
}
float FwLateralLongitudinalControl::getCorrectedLateralAccelSetpoint(float lateral_accel_sp)
{
	// Scale the npfg output to zero if npfg is not certain for correct output
	// 中文：当 NPFG 输入条件不可靠时，将横向加速度按可信度从 1 平滑降到 0。
	_can_run_factor = math::constrain(getGuidanceQualityFactor(_local_pos, _wind_valid), 0.f, 1.f);

	hrt_abstime now{hrt_absolute_time()};

	// Warn the user when the scale is less than 90% for at least 2 seconds (disable in transition)
	// 中文：如果滚转指令长时间被降额，向用户发事件告警，便于定位风/速度估计问题。

	// If the npfg was not running before, reset the user warning variables.
	// 中文：距离上次调用太久说明制导刚恢复，重置告警计时。
	if ((now - _time_since_last_npfg_call) > ROLL_WARNING_TIMEOUT) {
		_need_report_npfg_uncertain_condition = true;
		_time_since_first_reduced_roll = 0U;
	}

	if (_vehicle_status_sub.get().in_transition_mode || _can_run_factor > ROLL_WARNING_CAN_RUN_THRESHOLD || _landed) {
		// NPFG reports a good condition or we are in transition, reset the user warning variables.
		// 中文：过渡模式、已降落或可信度恢复后，不再累计不确定告警。
		_need_report_npfg_uncertain_condition = true;
		_time_since_first_reduced_roll = 0U;

	} else if (_need_report_npfg_uncertain_condition) {
		if (_time_since_first_reduced_roll == 0U) {
			_time_since_first_reduced_roll = now;
		}

		if ((now - _time_since_first_reduced_roll) > ROLL_WARNING_TIMEOUT) {
			_need_report_npfg_uncertain_condition = false;
			events::send(events::ID("npfg_roll_command_uncertain"), events::Log::Warning,
				     "Roll command reduced due to uncertain velocity/wind estimates!");
		}

	} else {
		// Nothing to do, already reported.
		// 中文：同一次持续不确定状态只报告一次，避免日志刷屏。
	}

	_time_since_last_npfg_call = now;

	// 实际输出为“可信度系数 * 原始横向加速度 setpoint”。
	return _can_run_factor * (lateral_accel_sp);
}
float FwLateralLongitudinalControl::mapLateralAccelerationToRollAngle(float lateral_acceleration_sp) const {
	// 协调转弯近似：横向加速度 ay = g * tan(phi)，因此 phi = atan(ay / g)。
	return  atanf(lateral_acceleration_sp / CONSTANTS_ONE_G);
}

void FwLateralLongitudinalControl::setDefaultLongitudinalControlConfiguration() {
	// 将纵向控制约束恢复为参数默认值；没有外部配置 topic 时使用这组限制。
	_long_configuration.timestamp = hrt_absolute_time();
	_long_configuration.pitch_min = radians(_param_fw_p_lim_min.get());
	_long_configuration.pitch_max = radians(_param_fw_p_lim_max.get());
	_long_configuration.throttle_min = _param_fw_thr_min.get();
	_long_configuration.throttle_max = _param_fw_thr_max.get();
	_long_configuration.climb_rate_target = _param_climbrate_target.get();
	_long_configuration.sink_rate_target = _param_sinkrate_target.get();
	_long_configuration.disable_underspeed_protection = false;
	_long_configuration.enforce_low_height_condition = false;
}

void FwLateralLongitudinalControl::updateLongitudinalControlConfiguration(const longitudinal_control_configuration_s &configuration_in) {
	/*
	 * 接收上游模式管理器给出的纵向约束。
	 * 每个字段都允许 NaN 表示“使用默认参数”，有效字段则按飞机硬限制再次限幅。
	 */
	_long_configuration.timestamp = configuration_in.timestamp;

	if (PX4_ISFINITE(configuration_in.pitch_min)) {
		// 最小俯仰不能低于参数下限，也不能超过固定翼最大俯仰限制。
		_long_configuration.pitch_min = math::constrain(configuration_in.pitch_min, radians(_param_fw_p_lim_min.get()), radians(_param_fw_p_lim_max.get()));
	} else {
		_long_configuration.pitch_min = radians(_param_fw_p_lim_min.get());
	}

	if (PX4_ISFINITE(configuration_in.pitch_max)) {
		// 最大俯仰至少要高于当前 pitch_min，避免 TECS 得到反向约束区间。
		_long_configuration.pitch_max = math::constrain(configuration_in.pitch_max, _long_configuration.pitch_min, radians(_param_fw_p_lim_max.get()));
	} else {
		_long_configuration.pitch_max = radians(_param_fw_p_lim_max.get());
	}

	if (PX4_ISFINITE(configuration_in.throttle_min)) {
		// 油门下限由上游阶段控制，但仍受全局最小/最大油门限制。
		_long_configuration.throttle_min = math::constrain(configuration_in.throttle_min, _param_fw_thr_min.get(), _param_fw_thr_max.get());
	} else {
		_long_configuration.throttle_min = _param_fw_thr_min.get();
	}

	if (PX4_ISFINITE(configuration_in.throttle_max)) {
		// 油门上限不能低于当前 throttle_min。
		_long_configuration.throttle_max = math::constrain(configuration_in.throttle_max, _long_configuration.throttle_min, _param_fw_thr_max.get());
	} else {
		_long_configuration.throttle_max = _param_fw_thr_max.get();
	}

	if (PX4_ISFINITE(configuration_in.climb_rate_target)) {
		// 目标爬升率为非负幅值，方向由高度误差/TECS 内部决定。
		_long_configuration.climb_rate_target = math::max(0.0f, configuration_in.climb_rate_target);
	} else {
		_long_configuration.climb_rate_target = _param_climbrate_target.get();
	}

	if (PX4_ISFINITE(configuration_in.sink_rate_target)) {
		// 目标下降率同样按非负幅值存储。
		_long_configuration.sink_rate_target = math::max(0.0f, configuration_in.sink_rate_target);
	} else {
		_long_configuration.sink_rate_target = _param_sinkrate_target.get();
	}

	if (PX4_ISFINITE(configuration_in.speed_weight)) {
		// TECS speed_weight 范围 0..2：越大越偏向保速度，越小越偏向保高度。
		_long_configuration.speed_weight = math::constrain(configuration_in.speed_weight, 0.f, 2.f);
	} else {
		_long_configuration.speed_weight = _param_t_spdweight.get();
	}
}

float FwLateralLongitudinalControl::getLoadFactor() const
{
	// 根据当前姿态 setpoint 的滚转角估计载荷因子，用于襟翼/性能模型最小空速修正。
	float load_factor_from_bank_angle = 1.f;

	const float roll_body = Eulerf(Quatf(_att_sp.q_d)).phi();

	if (PX4_ISFINITE(roll_body)) {
		// 简化协调转弯模型：n = 1 / cos(phi)，并用 FLT_EPSILON 防止 90 度附近除零。
		load_factor_from_bank_angle = 1.f / math::max(cosf(roll_body), FLT_EPSILON);
	}

	return load_factor_from_bank_angle;
}

extern "C" __EXPORT int fw_lat_lon_control_main(int argc, char *argv[])
{
	// C 链接入口，供 PX4 模块加载器调用 C++ ModuleBase 主函数。
	return FwLateralLongitudinalControl::main(argc, argv);
}
