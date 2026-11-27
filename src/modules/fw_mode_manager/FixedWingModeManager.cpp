/****************************************************************************
 *
 *   Copyright (c) 2013-2025 PX4 Development Team. All rights reserved.
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

#include "FixedWingModeManager.hpp"

#include <px4_platform_common/events.h>
#include <uORB/topics/longitudinal_control_configuration.h>

/*
 * 固定翼模式管理模块。
 *
 * 通信链路位置：
 *   任务/手动/Offboard/导航输入
 *        -> fixed_wing_lateral_setpoint
 *        -> fixed_wing_longitudinal_setpoint
 *
 * 本文件是固定翼制导状态机。它解释任务位置点、手动杆量、车辆命令、
 * 起飞/降落阶段，并发布 FwLateralLongitudinalControl 消费的横向/纵向设定值。
 * 它还负责襟翼、扰流板、起落架、跑道转向和降落状态等固定翼辅助命令。
 *
 * 复现到 MATLAB/Simulink 时，可把本模块看成上游调度器/状态机：
 * 它生成与 PX4 uORB 通信中相同的两个 setpoint 总线。
 */

using math::constrain;
using math::max;
using math::min;
using math::radians;

using matrix::Dcmf;
using matrix::Eulerf;
using matrix::Quatf;
using matrix::Vector2f;
using matrix::Vector2d;
using matrix::Vector3f;
using matrix::wrap_pi;

const fixed_wing_lateral_setpoint_s empty_lateral_control_setpoint = {.timestamp = 0, .course = NAN, .airspeed_direction = NAN, .lateral_acceleration = NAN};
const fixed_wing_longitudinal_setpoint_s empty_longitudinal_control_setpoint = {.timestamp = 0, .altitude = NAN, .height_rate = NAN, .equivalent_airspeed = NAN, .pitch_direct = NAN, .throttle_direct = NAN};

FixedWingModeManager::FixedWingModeManager() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")),
	_launchDetector(this),
	_runway_takeoff(this)
#ifdef CONFIG_FIGURE_OF_EIGHT
	, _figure_eight(_directional_guidance, _wind_vel)
#endif // CONFIG_FIGURE_OF_EIGHT
{
	// limit to 50 Hz
	/*
	 * 制导状态机不需要 IMU 级别的高频运行。PX4 通过 local position 更新把该模块
	 * 限制在最高 50 Hz；在 Simulink 中复现 uORB 时序时也应参考这个节奏。
	 */
	_local_pos_sub.set_interval_ms(20);

	_pos_ctrl_landing_status_pub.advertise();
	_launch_detection_status_pub.advertise();
	_landing_gear_pub.advertise();
	_flaps_setpoint_pub.advertise();
	_spoilers_setpoint_pub.advertise();
	_fixed_wing_lateral_guidance_status_pub.advertise();
	_fixed_wing_runway_control_pub.advertise();

	parameters_update();
}

FixedWingModeManager::~FixedWingModeManager()
{
	perf_free(_loop_perf);
}

bool
FixedWingModeManager::init()
{
	if (!_local_pos_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void
FixedWingModeManager::parameters_update()
{
	/*
	 * 加载 NPFG/方向制导参数和固定翼模式限制。这些参数决定 position_setpoint_triplet
	 * 如何被转换成 fixed_wing_lateral_setpoint 与 fixed_wing_longitudinal_setpoint。
	 */
	updateParams();

	_directional_guidance.setPeriod(_param_npfg_period.get());
	_directional_guidance.setDamping(_param_npfg_damping.get());
	_directional_guidance.enablePeriodLB(_param_npfg_en_period_lb.get());
	_directional_guidance.enablePeriodUB(_param_npfg_en_period_ub.get());
	_directional_guidance.setRollTimeConst(_param_npfg_roll_time_const.get());
	_directional_guidance.setSwitchDistanceMultiplier(_param_npfg_switch_distance_multiplier.get());
	_directional_guidance.setPeriodSafetyFactor(_param_npfg_period_safety_factor.get());
}

void
FixedWingModeManager::vehicle_control_mode_poll()
{
	if (_control_mode_sub.updated()) {
		const bool was_armed = _control_mode.flag_armed;

		if (_control_mode_sub.copy(&_control_mode)) {

			// reset state when arming
			/*
			 * 起飞和降落状态属于单次飞行状态。解锁沿触发时重置，避免新一次飞行继承
			 * 上一次飞行残留的发射、跑道或降落状态。
			 */
			if (!was_armed && _control_mode.flag_armed) {
				reset_takeoff_state();
				reset_landing_state();
			}
		}
	}
}

void
FixedWingModeManager::vehicle_command_poll()
{
	vehicle_command_s vehicle_command;

	while (_vehicle_command_sub.update(&vehicle_command)) {
		if (vehicle_command.command == vehicle_command_s::VEHICLE_CMD_DO_GO_AROUND) {
			/*
			 * 复飞命令在这里转换为降落中止请求。只有自动降落阶段且尚未越过不可返航点时，
			 * 该中止请求才有效。
			 */
			// only abort landing before point of no return (horizontal and vertical)
			if (_control_mode.flag_control_auto_enabled &&
			    _position_setpoint_current_valid &&
			    (_pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LAND)) {

				updateLandingAbortStatus(position_controller_landing_status_s::ABORTED_BY_OPERATOR);
			}

		} else if (vehicle_command.command == vehicle_command_s::VEHICLE_CMD_DO_CHANGE_SPEED) {

			/*
			 * 固定翼速度修改命令按空速命令处理。自动模式下修改当前任务 setpoint；
			 * 手动位置/高度模式下则更新手动空速目标。
			 */
			if ((static_cast<uint8_t>(vehicle_command.param1 + .5f) == vehicle_command_s::SPEED_TYPE_AIRSPEED)) {
				if (vehicle_command.param2 > FLT_EPSILON) {	// param2 is an equivalent airspeed setpoint
					if (_control_mode_current == FW_POSCTRL_MODE_AUTO) {
						_pos_sp_triplet.current.cruising_speed = vehicle_command.param2;

					} else if (_control_mode_current == FW_POSCTRL_MODE_MANUAL_ALTITUDE
						   || _control_mode_current == FW_POSCTRL_MODE_MANUAL_POSITION) {
						_commanded_manual_airspeed_setpoint = vehicle_command.param2;
					}

				}
			}

		}
	}
}

void
FixedWingModeManager::airspeed_poll()
{
	/*
	 * 阶段切换判断使用真实 calibrated airspeed。这里有意忽略 synthetic airspeed，
	 * 因为发射、襟翼收放和起飞转换等状态机决策需要更可靠的实测空速。
	 */
	airspeed_validated_s airspeed_validated;

	if (_param_fw_use_airspd.get() && _airspeed_validated_sub.update(&airspeed_validated)) {

		// do not use synthetic airspeed as it's for the use here not reliable enough
		if (PX4_ISFINITE(airspeed_validated.calibrated_airspeed_m_s)
		    && airspeed_validated.airspeed_source != airspeed_validated_s::SOURCE_SYNTHETIC) {

			_airspeed_eas = airspeed_validated.calibrated_airspeed_m_s;
		}
	}

	// no airspeed updates for one second --> declare invalid
	// this flag is used for some logic like: exiting takeoff, flaps retraction
	_airspeed_valid = hrt_elapsed_time(&_time_airspeed_last_valid) < 1_s;
}

void
FixedWingModeManager::wind_poll(const hrt_abstime now)
{
	if (_wind_sub.updated()) {
		wind_s wind;
		_wind_sub.update(&wind);

		// assumes wind is valid if finite
		_wind_valid = PX4_ISFINITE(wind.windspeed_north)
			      && PX4_ISFINITE(wind.windspeed_east);

		_time_wind_last_received = now;

		_wind_vel(0) = wind.windspeed_north;
		_wind_vel(1) = wind.windspeed_east;

	} else {
		// invalidate wind estimate usage (and correspondingly NPFG, if enabled) after subscription timeout
		_wind_valid = _wind_valid && (now - _time_wind_last_received) < WIND_EST_TIMEOUT;
	}

	if (!_wind_valid) {
		_wind_vel(0) = 0.f;
		_wind_vel(1) = 0.f;
	}
}

void
FixedWingModeManager::manual_control_setpoint_poll()
{
	/*
	 * 手动位置/高度模式会把杆量转换为高度变化率和空速命令。
	 * FW_POS_STK_CONF 可以交换两个杆量分配，以适配飞手习惯或机型需求。
	 */
	_sticks.checkAndUpdateStickInputs();

	_manual_control_setpoint_for_height_rate = _sticks.getPitch();
	_manual_control_setpoint_for_airspeed = _sticks.getThrottleZeroCentered();

	if (_param_fw_pos_stk_conf.get() & STICK_CONFIG_SWAP_STICKS_BIT) {
		/* Alternate stick allocation (similar concept as for multirotor systems:
		 * demanding up/down with the throttle stick, and move faster/break with the pitch one.
		 */
		// 中文：交换后，油门杆控制爬升/下降，俯仰杆控制目标空速增减。

		_manual_control_setpoint_for_height_rate = -_sticks.getThrottleZeroCentered();
		_manual_control_setpoint_for_airspeed = _sticks.getPitch();
	}
}

void
FixedWingModeManager::vehicle_attitude_poll()
{
	// 读取当前姿态和角速度，用于手动航向保持、起飞发射状态和 VTOL 过渡判断。
	vehicle_attitude_s vehicle_attitude;

	if (_vehicle_attitude_sub.update(&vehicle_attitude)) {
		vehicle_angular_velocity_s angular_velocity{};
		_vehicle_angular_velocity_sub.copy(&angular_velocity);
		const Vector3f rates{angular_velocity.xyz};

		Dcmf R{Quatf(vehicle_attitude.q)};

		// if the vehicle is a tailsitter we have to rotate the attitude by the pitch offset
		// between multirotor and fixed wing flight
		// 中文：尾座式 VTOL 在多旋翼姿态系和固定翼姿态系之间有 90 度俯仰偏置。
		if (_vehicle_status.is_vtol_tailsitter) {
			const Dcmf R_offset{Eulerf{0.f, M_PI_2_F, 0.f}};
			R = R * R_offset;

			// 尾座式固定翼语义下的偏航角速度对应原机体系 roll rate。
			_yawrate = rates(0);

		} else {
			_yawrate = rates(2);
		}

		const Eulerf euler_angles(R);
		_yaw = euler_angles(2);

		// 把局部坐标加速度旋到机体系，发射检测使用机体 X 轴前向加速度。
		const Vector3f body_acceleration = R.transpose() * Vector3f{_local_pos.ax, _local_pos.ay, _local_pos.az};
		_body_acceleration_x = body_acceleration(0);

		// 机体 X 向速度可用于状态判断/日志排查，保持和 PX4 原逻辑一致。
		const Vector3f body_velocity = R.transpose() * Vector3f{_local_pos.vx, _local_pos.vy, _local_pos.vz};
		_body_velocity_x = body_velocity(0);
	}
}

float
FixedWingModeManager::get_manual_airspeed_setpoint()
{
	// 将手动杆量或外部速度命令转换为 EAS 空速 setpoint；NaN 表示由下游使用默认巡航空速。
	float manual_airspeed_setpoint = NAN;

	if (_param_fw_pos_stk_conf.get() & STICK_CONFIG_ENABLE_AIRSPEED_SP_MANUAL_BIT) {
		// neutral throttle corresponds to trim airspeed
		// 中文：杆量 -1/0/1 分别映射到最小/配平/最大空速。
		manual_airspeed_setpoint = math::interpolateNXY(_manual_control_setpoint_for_airspeed,
		{-1.f, 0.f, 1.f},
		{_param_fw_airspd_min.get(), _param_fw_airspd_trim.get(), _param_fw_airspd_max.get()});

	} else if (PX4_ISFINITE(_commanded_manual_airspeed_setpoint)) {
		// override stick by commanded airspeed
		// 中文：若不允许杆量控制空速，但收到 DO_CHANGE_SPEED，则用命令值覆盖。
		manual_airspeed_setpoint = _commanded_manual_airspeed_setpoint;
	}

	return manual_airspeed_setpoint;
}

void
FixedWingModeManager::landing_status_publish()
{
	// 发布降落状态，Navigator/日志可据此知道是否拉平、是否中止降落以及触地点偏移。
	position_controller_landing_status_s pos_ctrl_landing_status = {};

	pos_ctrl_landing_status.lateral_touchdown_offset = _lateral_touchdown_position_offset;
	pos_ctrl_landing_status.flaring = _flare_states.flaring;
	pos_ctrl_landing_status.abort_status = _landing_abort_status;
	pos_ctrl_landing_status.timestamp = hrt_absolute_time();

	_pos_ctrl_landing_status_pub.publish(pos_ctrl_landing_status);
}

void
FixedWingModeManager::updateLandingAbortStatus(const uint8_t new_abort_status)
{
	// prevent automatic aborts if already flaring, but allow manual aborts
	// 中文：进入 flare 后通常不可自动中止；但飞手手动复飞命令仍允许覆盖。
	if (!_flare_states.flaring || new_abort_status == position_controller_landing_status_s::ABORTED_BY_OPERATOR) {

		// only announce changes
		// 中文：只在状态真正变化时发事件，避免重复报警。
		if (new_abort_status > 0 && _landing_abort_status != new_abort_status) {

			switch (new_abort_status) {
			case (position_controller_landing_status_s::ABORTED_BY_OPERATOR): {
					events::send(events::ID("fixedwing_position_control_landing_abort_status_operator_abort"), events::Log::Critical,
						     "Landing aborted by operator");
					break;
				}

			case (position_controller_landing_status_s::TERRAIN_NOT_FOUND): {
					events::send(events::ID("fixedwing_position_control_landing_abort_status_terrain_not_found"), events::Log::Critical,
						     "Landing aborted: terrain measurement not found");
					break;
				}

			case (position_controller_landing_status_s::TERRAIN_TIMEOUT): {
					events::send(events::ID("fixedwing_position_control_landing_abort_status_terrain_timeout"), events::Log::Critical,
						     "Landing aborted: terrain estimate timed out");
					break;
				}

			default: {
					events::send(events::ID("fixedwing_position_control_landing_abort_status_unknown_criterion"), events::Log::Critical,
						     "Landing aborted: unknown criterion");
				}
			}
		}

		_landing_abort_status = (new_abort_status >= position_controller_landing_status_s::UNKNOWN_ABORT_CRITERION) ?
					position_controller_landing_status_s::UNKNOWN_ABORT_CRITERION : new_abort_status;
		// 状态改变后立即发布，保证其他模块本周期即可看到降落中止结果。
		landing_status_publish();
	}
}

float
FixedWingModeManager::getManualHeightRateSetpoint()
{
	// 手动高度模式：把杆量映射为高度变化率 setpoint，上推爬升、下推下降。
	float height_rate_setpoint = 0.f;

	if (_manual_control_setpoint_for_height_rate >= FLT_EPSILON) {
		// 正向杆量对应下降率；PX4 高度率 setpoint 采用“向上为正”的符号。
		height_rate_setpoint = math::interpolate<float>(math::deadzone(_manual_control_setpoint_for_height_rate,
				       kStickDeadBand), 0, 1.f, 0.f, -_param_sinkrate_target.get());

	} else {
		// 负向杆量对应爬升率。
		height_rate_setpoint = math::interpolate<float>(math::deadzone(_manual_control_setpoint_for_height_rate,
				       kStickDeadBand), -1., 0.f, _param_climbrate_target.get(), 0.f);
	}

	return height_rate_setpoint;
}

void
FixedWingModeManager::updateManualTakeoffStatus()
{
	// 手动模式刚起飞时，在达到可控空速前会限制俯仰，避免 TECS 为保空速过早低头。
	if (!_completed_manual_takeoff) {
		const bool at_controllable_airspeed = _airspeed_eas > _param_fw_airspd_min.get()
						      || !_airspeed_valid;
		const bool is_hovering = _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
					 && _control_mode.flag_armed;
		_completed_manual_takeoff = (!_landed && at_controllable_airspeed) || is_hovering;
	}
}

void
FixedWingModeManager::set_control_mode_current(const hrt_abstime &now)
{
	/* only run position controller in fixed-wing mode and during transitions for VTOL */
	// 中文：普通多旋翼模式不发布固定翼 setpoint；VTOL 过渡期间仍需要固定翼制导参与。
	if (_vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING && !_vehicle_status.in_transition_mode) {
		_control_mode_current = FW_POSCTRL_MODE_OTHER;
		return; // do not publish the setpoint
	}

	const FW_POSCTRL_MODE previous_position_control_mode = _control_mode_current;

	_skipping_takeoff_detection = false;
	const bool doing_backtransition = _vehicle_status.in_transition_mode && !_vehicle_status.in_transition_to_fw;

	// Offboard: 有位置控制标志时，trajectory_setpoint 转换成伪 mission setpoint 处理。
	if (_control_mode.flag_control_offboard_enabled && _position_setpoint_current_valid
	    && _control_mode.flag_control_position_enabled) {
		if (PX4_ISFINITE(_pos_sp_triplet.current.vx) && PX4_ISFINITE(_pos_sp_triplet.current.vy)
		    && PX4_ISFINITE(_pos_sp_triplet.current.vz)) {
			// Offboard position with velocity setpoints
			// 中文：同时有速度 setpoint 时，按路径切向/曲率控制处理。
			_control_mode_current = FW_POSCTRL_MODE_AUTO_PATH;
			return;

		} else {
			// Offboard position setpoint only
			// 中文：只有位置 setpoint 时，按普通自动位置点处理。
			_control_mode_current = FW_POSCTRL_MODE_AUTO;
			return;
		}

	} else if ((_control_mode.flag_control_auto_enabled && _control_mode.flag_control_position_enabled)
		   && (_position_setpoint_current_valid
		       || _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_IDLE)) {

		// Enter this mode only if the current waypoint has valid 3D position setpoints or is of type IDLE.
		// A setpoint of type IDLE can be published by Navigator without a valid position, and is handled here in FW_POSCTRL_MODE_AUTO.
		// 中文：自动模式依赖 Navigator 发布 position_setpoint_triplet，IDLE 是特殊允许的无位置 setpoint。

		if (doing_backtransition) {
			// VTOL 固定翼回多旋翼过渡时，用线跟踪或航向保持控制横向。
			_control_mode_current = FW_POSCTRL_MODE_TRANSITION_TO_HOVER_LINE_FOLLOW;

		} else if (_pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF) {

			if (_vehicle_status.is_vtol && _vehicle_status.in_transition_mode) {
				// VTOL 前转换中的 takeoff 点被当作普通位置点，避免重复进入固定翼起飞逻辑。
				_control_mode_current = FW_POSCTRL_MODE_AUTO;

				// in this case we want the waypoint handled as a position setpoint -- a submode in control_auto()
				// 中文：修改临时副本/当前点类型，使 control_auto 使用普通航点控制。
				_pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;

			} else {
				// 有水平定位则按导航起飞；无水平定位则只做爬升/平翼起飞。
				_control_mode_current = _local_pos.xy_valid ? FW_POSCTRL_MODE_AUTO_TAKEOFF : FW_POSCTRL_MODE_AUTO_TAKEOFF_NO_NAV;

				if (previous_position_control_mode != FW_POSCTRL_MODE_AUTO_TAKEOFF_NO_NAV
				    && previous_position_control_mode != FW_POSCTRL_MODE_AUTO_TAKEOFF && !_landed) {
					// skip takeoff detection when switching from any other mode, auto or manual,
					// while already in air.
					// TODO: find a better place for / way of doing this
					// 中文：如果飞机已经在空中切入 takeoff setpoint，跳过发射/跑道检测，直接进入飞行状态。
					_skipping_takeoff_detection = true;
				}
			}

		} else if (_pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LAND) {

			// Use _position_setpoint_previous_valid to determine if landing should be straight or circular.
			// Straight landings are currently only possible in Missions, and there the previous WP
			// is valid, and circular ones are used outside of Missions, as the land mode sets prev_valid=false.
			// 中文：任务降落有上一航点，形成直线下滑道；独立 Land 模式没有上一点，使用盘旋降落。
			if (_position_setpoint_previous_valid) {
				_control_mode_current = FW_POSCTRL_MODE_AUTO_LANDING_STRAIGHT;

			} else {
				_control_mode_current = FW_POSCTRL_MODE_AUTO_LANDING_CIRCULAR;
			}

		} else {
			_control_mode_current = FW_POSCTRL_MODE_AUTO;
		}

	} else if (_control_mode.flag_control_auto_enabled
		   && _control_mode.flag_control_climb_rate_enabled
		   && _control_mode.flag_armed // only enter this modes if armed, as pure failsafe modes
		   && !_control_mode.flag_control_position_enabled) {

		// failsafe modes engaged if position estimate is invalidated
		// 中文：自动模式丢水平位置后，先固定坡度盘旋保高，超时后转为定下降率。

		if (previous_position_control_mode != FW_POSCTRL_MODE_AUTO_ALTITUDE
		    && previous_position_control_mode != FW_POSCTRL_MODE_AUTO_CLIMBRATE) {
			// reset timer the first time we switch into this mode
			_time_in_fixed_bank_loiter = now;
		}

		if (doing_backtransition) {
			// we handle loss of position control during backtransition as a special case
			// 中文：回多旋翼过渡且无位置控制时，只保持进入过渡时的航向。
			_control_mode_current = FW_POSCTRL_MODE_TRANSITION_TO_HOVER_HEADING_HOLD;

		} else if (hrt_elapsed_time(&_time_in_fixed_bank_loiter) < (_param_nav_gpsf_lt.get() * 1_s)
			   && !_vehicle_status.in_transition_mode) {
			if (previous_position_control_mode != FW_POSCTRL_MODE_AUTO_ALTITUDE) {
				// Need to init because last loop iteration was in a different mode
				events::send(events::ID("fixedwing_position_control_fb_loiter"), events::Log::Critical,
					     "Start loiter with fixed bank angle");
			}

			_control_mode_current = FW_POSCTRL_MODE_AUTO_ALTITUDE;

		} else {
			if (previous_position_control_mode != FW_POSCTRL_MODE_AUTO_CLIMBRATE && !_vehicle_status.in_transition_mode) {
				events::send(events::ID("fixedwing_position_control_descend"), events::Log::Critical, "Start descending");
			}

			_control_mode_current = FW_POSCTRL_MODE_AUTO_CLIMBRATE;
		}


	} else if (_control_mode.flag_control_manual_enabled && _control_mode.flag_control_position_enabled) {
		if (previous_position_control_mode != FW_POSCTRL_MODE_MANUAL_POSITION) {
			/* Need to init because last loop iteration was in a different mode */
			// 中文：刚进入手动位置模式时，航向保持从当前航向开始，不继承旧航线。
			_hdg_hold_yaw = _yaw; // yaw is not controlled, so set setpoint to current yaw
			_hdg_hold_enabled = false; // this makes sure the waypoints are reset below
			_yaw_lock_engaged = false;
		}

		_control_mode_current = FW_POSCTRL_MODE_MANUAL_POSITION;

	} else if (_control_mode.flag_control_manual_enabled && _control_mode.flag_control_altitude_enabled) {

		_control_mode_current = FW_POSCTRL_MODE_MANUAL_ALTITUDE;

	} else {
		_control_mode_current = FW_POSCTRL_MODE_OTHER;
	}
}

void
FixedWingModeManager::update_in_air_states(const hrt_abstime now)
{
	/* reset flag when airplane landed */
	// 中文：落地后手动起飞完成标志清零，下一次起飞重新执行近地保护逻辑。
	if (_landed) {
		_completed_manual_takeoff = false;
	}
}

void
FixedWingModeManager::move_position_setpoint_for_vtol_transition(position_setpoint_s &current_sp)
{
	// TODO: velocity, altitude, or just a heading hold position mode should be used for this, not position
	// shifting hacks
	// 中文：VTOL 前转换时人为把目标点前移，目的是在固定翼过渡过程中维持明确前向航迹。

	if (_vehicle_status.in_transition_to_fw) {

		if (!PX4_ISFINITE(_transition_waypoint(0))) {
			double lat_transition, lon_transition;

			// Create a virtual waypoint HDG_HOLD_DIST_NEXT meters in front of the vehicle which the path navigation controller can track
			// during the transition. Use the current yaw setpoint to determine the transition heading, as that one in turn
			// is set to the transition heading by Navigator, or current yaw if setpoint is not valid.
			// 中文：虚拟航点只在第一次进入前转换时生成，后续周期保持同一目标，避免航迹来回跳动。
			const float transition_heading = PX4_ISFINITE(current_sp.yaw) ? current_sp.yaw : _yaw;
			waypoint_from_heading_and_distance(_current_latitude, _current_longitude, transition_heading, HDG_HOLD_DIST_NEXT,
							   &lat_transition,
							   &lon_transition);

			_transition_waypoint(0) = lat_transition;
			_transition_waypoint(1) = lon_transition;
		}

		current_sp.lat = _transition_waypoint(0);
		current_sp.lon = _transition_waypoint(1);

	} else {
		/* reset transition waypoint, will be set upon entering front transition */
		// 中文：不在前转换时清空缓存，下次过渡重新以当时航向生成虚拟航点。
		_transition_waypoint(0) = static_cast<double>(NAN);
		_transition_waypoint(1) = static_cast<double>(NAN);
	}
}

void
FixedWingModeManager::control_auto(const float control_interval, const Vector2d &curr_pos,
				   const Vector2f &ground_speed, const position_setpoint_s &pos_sp_prev, const position_setpoint_s &pos_sp_curr,
				   const position_setpoint_s &pos_sp_next)
{
	/*
	 * 自动模式总调度：
	 * 根据当前 mission/offboard setpoint 类型，调用位置、速度、盘旋、八字或空闲控制。
	 * 每个子函数都会发布横向 fixed_wing_lateral_setpoint 和纵向 fixed_wing_longitudinal_setpoint。
	 */
	position_setpoint_s current_sp = pos_sp_curr;
	move_position_setpoint_for_vtol_transition(current_sp);

	// 可能将普通 POSITION 临时转换为 LOITER，用盘旋先达到高度。
	const uint8_t position_sp_type = handle_setpoint_type(current_sp, pos_sp_next);

	_position_sp_type = position_sp_type;

	if (position_sp_type == position_setpoint_s::SETPOINT_TYPE_LOITER
	    || current_sp.type == position_setpoint_s::SETPOINT_TYPE_LOITER) {
#ifdef CONFIG_FIGURE_OF_EIGHT

		if (current_sp.loiter_pattern == position_setpoint_s::LOITER_TYPE_FIGUREEIGHT) {
			// 八字盘旋状态用于地面站显示。
			publishFigureEightStatus(current_sp);

		} else
#endif // CONFIG_FIGURE_OF_EIGHT
		{
			// 普通盘旋状态用于地面站/日志显示 orbit。
			publishOrbitStatus(current_sp);
		}
	}

	// 核心状态分发：只根据 setpoint 类型决定横纵向 setpoint 生成方式。
	switch (position_sp_type) {
	case position_setpoint_s::SETPOINT_TYPE_IDLE: {
			control_idle();
			break;
		}

	case position_setpoint_s::SETPOINT_TYPE_POSITION:
		control_auto_position(control_interval, curr_pos, ground_speed, pos_sp_prev, current_sp);
		break;

	case position_setpoint_s::SETPOINT_TYPE_VELOCITY:
		control_auto_velocity(control_interval, curr_pos, ground_speed, current_sp);
		break;

	case position_setpoint_s::SETPOINT_TYPE_LOITER:
#ifdef CONFIG_FIGURE_OF_EIGHT
		if (current_sp.loiter_pattern == position_setpoint_s::LOITER_TYPE_FIGUREEIGHT) {
			controlAutoFigureEight(control_interval, curr_pos, ground_speed, current_sp);

		} else
#endif // CONFIG_FIGURE_OF_EIGHT
		{
			control_auto_loiter(control_interval, curr_pos, ground_speed, current_sp, pos_sp_next);

		}

		break;
	}

#ifdef CONFIG_FIGURE_OF_EIGHT

	/* reset loiter state */
	// 中文：离开八字盘旋后重置内部几何状态，避免下次进入时继承旧半圈状态。
	if ((position_sp_type != position_setpoint_s::SETPOINT_TYPE_LOITER) ||
	    ((position_sp_type == position_setpoint_s::SETPOINT_TYPE_LOITER) &&
	     (current_sp.loiter_pattern != position_setpoint_s::LOITER_TYPE_FIGUREEIGHT))) {
		_figure_eight.resetPattern();
	}

#endif // CONFIG_FIGURE_OF_EIGHT

	if (!_vehicle_status.in_transition_to_fw) {
		// 发布可视化用 local position setpoint；前转换阶段不发布，避免与 VTOL 导航语义冲突。
		publishLocalPositionSetpoint(current_sp);
	}
}

void FixedWingModeManager::control_idle()
{
	// IDLE setpoint：横向加速度为 0、俯仰/油门直接给 0，并把油门配置上限/下限锁为 0。
	const hrt_abstime  now = hrt_absolute_time();
	fixed_wing_lateral_setpoint_s lateral_ctrl_sp {empty_lateral_control_setpoint};
	lateral_ctrl_sp.timestamp = now;
	lateral_ctrl_sp.lateral_acceleration = 0.0f;
	_lateral_ctrl_sp_pub.publish(lateral_ctrl_sp);

	fixed_wing_longitudinal_setpoint_s long_contrl_sp {empty_longitudinal_control_setpoint};
	long_contrl_sp.timestamp = now;
	long_contrl_sp.pitch_direct = 0.f;
	long_contrl_sp.throttle_direct = 0.0f;
	_longitudinal_ctrl_sp_pub.publish(long_contrl_sp);

	_ctrl_configuration_handler.setThrottleMax(0.0f);
	_ctrl_configuration_handler.setThrottleMin(0.0f);
}

void
FixedWingModeManager::control_auto_fixed_bank_alt_hold()
{
	// GPS/位置失效保护阶段 1：固定坡度开环盘旋，同时纵向保持 0 高度率。
	const hrt_abstime now = hrt_absolute_time();
	const fixed_wing_longitudinal_setpoint_s fw_longitudinal_control_sp = {
		.timestamp = now,
		.altitude = NAN,
		.height_rate = 0.f,
		.equivalent_airspeed = NAN,
		.pitch_direct = NAN,
		.throttle_direct = NAN
	};

	_longitudinal_ctrl_sp_pub.publish(fw_longitudinal_control_sp);

	float throttle_max = _param_fw_thr_max.get();

	// Special case: if z or vz estimate is invalid we cannot control height anymore. To prevent a
	// "climb-away" we set the thrust to MIN in that case.
	// 中文：高度或垂向速度也失效时，不能安全保高，因此把油门上限压到最小，避免失控爬升。
	if (_landed || !_local_pos.z_valid || !_local_pos.v_z_valid) {
		throttle_max = _param_fw_thr_min.get();
	}

	_ctrl_configuration_handler.setThrottleMax(throttle_max);

	fixed_wing_lateral_setpoint_s lateral_ctrl_sp = empty_lateral_control_setpoint;
	lateral_ctrl_sp.timestamp = hrt_absolute_time();
	const float roll_body = math::radians(_param_nav_gpsf_r.get()); // open loop loiter bank angle
	// 中文：横向仍发布由固定坡度换算出的横向加速度 setpoint。
	lateral_ctrl_sp.lateral_acceleration = rollAngleToLateralAccel(roll_body);
	_lateral_ctrl_sp_pub.publish(lateral_ctrl_sp);
}

void
FixedWingModeManager::control_auto_descend()
{
	// Hard-code descend rate to 0.5m/s. This is a compromise to give the system to recover,
	// but not letting it drift too far away.
	// 中文：GPS failsafe 阶段 2：超时后不再保高，固定 0.5 m/s 缓慢下降。
	const float descend_rate = 0.5f;

	const hrt_abstime now = hrt_absolute_time();

	const fixed_wing_longitudinal_setpoint_s fw_longitudinal_control_sp = {
		.timestamp = now,
		.altitude = NAN,
		.height_rate = -descend_rate,
		.equivalent_airspeed = NAN,
		.pitch_direct = NAN,
		.throttle_direct = NAN
	};

	_longitudinal_ctrl_sp_pub.publish(fw_longitudinal_control_sp);

	_ctrl_configuration_handler.setThrottleMax((_landed
			|| !_local_pos.v_z_valid) ? _param_fw_thr_min.get() : _param_fw_thr_max.get());

	fixed_wing_lateral_setpoint_s lateral_ctrl_sp = empty_lateral_control_setpoint;
	lateral_ctrl_sp.timestamp = now;
	const float roll_body = math::radians(_param_nav_gpsf_r.get()); // open loop loiter bank angle
	lateral_ctrl_sp.lateral_acceleration = rollAngleToLateralAccel(roll_body);
	_lateral_ctrl_sp_pub.publish(lateral_ctrl_sp);
}

uint8_t
FixedWingModeManager::handle_setpoint_type(const position_setpoint_s &pos_sp_curr,
		const position_setpoint_s &pos_sp_next)
{
	// 把 Navigator 的 position_setpoint 类型映射为本模块内部真正执行的类型。
	uint8_t position_sp_type = pos_sp_curr.type;

	if (!_control_mode.flag_control_position_enabled && _control_mode.flag_control_velocity_enabled) {
		// Offboard 速度控制：没有位置闭环但有速度闭环时，强制按 VELOCITY 处理。
		return position_setpoint_s::SETPOINT_TYPE_VELOCITY;
	}

	Vector2d curr_wp{0, 0};

	/* current waypoint (the one currently heading for) */
	// 中文：当前位置点用于判断是否接近航点、是否需要转入盘旋爬/降高度。
	curr_wp = Vector2d(pos_sp_curr.lat, pos_sp_curr.lon);

	const float acc_rad = _directional_guidance.switchDistance(500.0f);

	const bool approaching_vtol_backtransition = _vehicle_status.is_vtol
			&& pos_sp_curr.type == position_setpoint_s::SETPOINT_TYPE_POSITION && _position_setpoint_current_valid
			&& pos_sp_next.type == position_setpoint_s::SETPOINT_TYPE_LAND && _position_setpoint_next_valid;

	// check if we should switch to loiter but only if we are not expecting a backtransition to happen
	// 中文：普通航点若水平已接近但高度还没到，临时转为 loiter 达高度。
	if (pos_sp_curr.type == position_setpoint_s::SETPOINT_TYPE_POSITION && !approaching_vtol_backtransition) {

		float dist_xy = -1.f;
		float dist_z = -1.f;

		const float dist = get_distance_to_point_global_wgs84(
					   (double)curr_wp(0), (double)curr_wp(1), pos_sp_curr.alt,
					   _current_latitude, _current_longitude, _current_altitude,
					   &dist_xy, &dist_z);

		const float acc_rad_z = (PX4_ISFINITE(pos_sp_curr.alt_acceptance_radius)
					 && pos_sp_curr.alt_acceptance_radius > FLT_EPSILON) ? pos_sp_curr.alt_acceptance_radius :
					_param_nav_fw_alt_rad.get();

		// Achieve position setpoint altitude via loiter when laterally close to WP.
		// Detect if system has switchted into a Loiter before (check _position_sp_type), and in that
		// case remove the dist_xy check (not switch out of Loiter until altitude is reached).
		// 中文：一旦进入这种临时 loiter，会保持到高度满足，不会因为水平距离变化来回切换。
		if ((!_vehicle_status.in_transition_mode) && (dist >= 0.f)
		    && (dist_z > acc_rad_z)
		    && (dist_xy < acc_rad || _position_sp_type == position_setpoint_s::SETPOINT_TYPE_LOITER)) {

			// SETPOINT_TYPE_POSITION -> SETPOINT_TYPE_LOITER
			position_sp_type = position_setpoint_s::SETPOINT_TYPE_LOITER;
		}
	}

	return position_sp_type;
}

void
FixedWingModeManager::control_auto_position(const float control_interval, const Vector2d &curr_pos,
		const Vector2f &ground_speed, const position_setpoint_s &pos_sp_prev, const position_setpoint_s &pos_sp_curr)
{
	// 自动普通航点控制：纵向跟踪高度/空速，横向跟踪上一航点到当前航点的线段。
	const float acc_rad = _directional_guidance.switchDistance(500.0f);
	const float target_airspeed = pos_sp_curr.cruising_speed > FLT_EPSILON ? pos_sp_curr.cruising_speed : NAN;

	// waypoint is a plain navigation waypoint
	// 中文：默认高度 setpoint 就是当前航点高度，后面 FOH 可能修改它。
	float position_sp_alt = pos_sp_curr.alt;

	// Altitude first order hold (FOH)
	// 中文：高度一阶保持：沿上一航点到当前航点的水平距离线性插值高度，避免高度 setpoint 阶跃。
	if (_position_setpoint_previous_valid &&
	    ((pos_sp_prev.type == position_setpoint_s::SETPOINT_TYPE_POSITION) ||
	     (pos_sp_prev.type == position_setpoint_s::SETPOINT_TYPE_LOITER))
	   ) {
		const float d_curr_prev = get_distance_to_next_waypoint(pos_sp_curr.lat, pos_sp_curr.lon, pos_sp_prev.lat,
					  pos_sp_prev.lon);

		// Do not try to find a solution if the last waypoint is inside the acceptance radius of the current one
		// 中文：若两航点太近，线性插值几何退化，直接使用当前航点高度。
		if (d_curr_prev > math::max(acc_rad, fabsf(pos_sp_curr.loiter_radius))) {
			// Calculate distance to current waypoint
			const float d_curr = get_distance_to_next_waypoint(pos_sp_curr.lat, pos_sp_curr.lon, _current_latitude,
					     _current_longitude);

			// Save distance to waypoint if it is the smallest ever achieved, however make sure that
			// _min_current_sp_distance_xy is never larger than the distance between the current and the previous wp
			// 中文：使用“到当前航点的历史最小距离”，保证高度 setpoint 沿航线单调推进。
			_min_current_sp_distance_xy = math::min(d_curr, _min_current_sp_distance_xy, d_curr_prev);

			// if the minimal distance is smaller than the acceptance radius, we should be at waypoint alt
			// navigator will soon switch to the next waypoint item (if there is one) as soon as we reach this altitude
			if (_min_current_sp_distance_xy > math::max(acc_rad, fabsf(pos_sp_curr.loiter_radius))) {
				// The setpoint is set linearly and such that the system reaches the current altitude at the acceptance
				// radius around the current waypoint
				// 中文：在进入接受半径前完成高度过渡；进入后保持当前航点高度。
				const float delta_alt = (pos_sp_curr.alt - pos_sp_prev.alt);
				const float grad = -delta_alt / (d_curr_prev - math::max(acc_rad, fabsf(pos_sp_curr.loiter_radius)));
				const float a = pos_sp_prev.alt - grad * d_curr_prev;

				position_sp_alt = a + grad * _min_current_sp_distance_xy;
			}
		}
	}

	const fixed_wing_longitudinal_setpoint_s fw_longitudinal_control_sp = {
		.timestamp = hrt_absolute_time(),
		.altitude = position_sp_alt,
		.height_rate = NAN,
		.equivalent_airspeed = target_airspeed,
		.pitch_direct = NAN,
		.throttle_direct = NAN
	};

	_longitudinal_ctrl_sp_pub.publish(fw_longitudinal_control_sp);

	float throttle_min = NAN;
	float throttle_max = NAN;

	if (pos_sp_curr.gliding_enabled) {
		/* enable gliding with this waypoint */
		// 中文：滑翔航点关闭油门，并把 TECS speed_weight 调到 2，优先保持速度。
		throttle_min = 0.0;
		throttle_max = 0.0;
		_ctrl_configuration_handler.setSpeedWeight(2.f);
	}

	_ctrl_configuration_handler.setThrottleMax(throttle_max);
	_ctrl_configuration_handler.setThrottleMin(throttle_min);

	Vector2f curr_pos_local{_local_pos.x, _local_pos.y};
	Vector2f curr_wp_local = _global_local_proj_ref.project(pos_sp_curr.lat, pos_sp_curr.lon);

	DirectionalGuidanceOutput sp{};

	if (_position_setpoint_previous_valid && pos_sp_prev.type != position_setpoint_s::SETPOINT_TYPE_TAKEOFF) {
		// 有上一航点时沿航段飞行，NPFG 计算航迹角和横向加速度前馈。
		Vector2f prev_wp_local = _global_local_proj_ref.project(pos_sp_prev.lat, pos_sp_prev.lon);
		sp = navigateWaypoints(prev_wp_local, curr_wp_local, curr_pos_local, ground_speed, _wind_vel);

	} else {
		// 无上一航点或上一点是起飞点时，直接飞向当前航点。
		sp = navigateWaypoint(curr_wp_local, curr_pos_local, ground_speed, _wind_vel);
	}

	fixed_wing_lateral_setpoint_s lateral_ctrl_sp{empty_lateral_control_setpoint};
	lateral_ctrl_sp.timestamp = hrt_absolute_time();
	lateral_ctrl_sp.course = sp.course_setpoint;
	lateral_ctrl_sp.lateral_acceleration = sp.lateral_acceleration_feedforward;
	_lateral_ctrl_sp_pub.publish(lateral_ctrl_sp);
}

void
FixedWingModeManager::control_auto_velocity(const float control_interval, const Vector2d &curr_pos,
		const Vector2f &ground_speed, const position_setpoint_s &pos_sp_curr)
{
	//Offboard velocity control
	// 中文：Offboard 速度控制只取水平速度方向作为航向目标，垂向速度交给纵向 setpoint。
	Vector2f target_velocity{pos_sp_curr.vx, pos_sp_curr.vy};
	const float target_bearing = wrap_pi(atan2f(target_velocity(1), target_velocity(0)));

	const Vector2f curr_pos_local{_local_pos.x, _local_pos.y};
	const DirectionalGuidanceOutput sp = navigateBearing(curr_pos_local, target_bearing, ground_speed, _wind_vel);

	fixed_wing_lateral_setpoint_s fw_lateral_ctrl_sp{empty_lateral_control_setpoint};
	fw_lateral_ctrl_sp.timestamp = hrt_absolute_time();
	fw_lateral_ctrl_sp.course = sp.course_setpoint;
	fw_lateral_ctrl_sp.lateral_acceleration = sp.lateral_acceleration_feedforward;
	_lateral_ctrl_sp_pub.publish(fw_lateral_ctrl_sp);

	const float target_airspeed = pos_sp_curr.cruising_speed > FLT_EPSILON ? pos_sp_curr.cruising_speed : NAN;

	const fixed_wing_longitudinal_setpoint_s fw_longitudinal_control_sp = {
		.timestamp = hrt_absolute_time(),
		.altitude = pos_sp_curr.alt,
		.height_rate = pos_sp_curr.vz,
		.equivalent_airspeed = target_airspeed,
		.pitch_direct = NAN,
		.throttle_direct = NAN
	};

	_longitudinal_ctrl_sp_pub.publish(fw_longitudinal_control_sp);

	if (pos_sp_curr.gliding_enabled) {
		// Offboard 也支持滑翔标志：关闭油门并偏向保速度。
		_ctrl_configuration_handler.setThrottleMin(0.0f);
		_ctrl_configuration_handler.setThrottleMax(0.0f);
		_ctrl_configuration_handler.setSpeedWeight(2.0f);
	}
}

void
FixedWingModeManager::control_auto_loiter(const float control_interval, const Vector2d &curr_pos,
		const Vector2f &ground_speed, const position_setpoint_s &pos_sp_curr,
		const position_setpoint_s &pos_sp_next)
{
	// 自动盘旋控制：横向跟踪圆轨迹，纵向保持盘旋点高度/空速。
	// current waypoint (the one currently heading for)
	const Vector2d curr_wp = Vector2d(pos_sp_curr.lat, pos_sp_curr.lon);

	float loiter_radius = fabsf(pos_sp_curr.loiter_radius);
	bool loiter_direction_ccw = pos_sp_curr.loiter_direction_counter_clockwise;

	if (loiter_radius < FLT_EPSILON) {
		// 航点未给有效半径时使用 NAV_LOITER_RAD；负值表示逆时针。
		loiter_radius = fabsf(_param_nav_loiter_rad.get());
		loiter_direction_ccw = _param_nav_loiter_rad.get() < -FLT_EPSILON;
	}

	Vector2f curr_pos_local{_local_pos.x, _local_pos.y};
	Vector2f curr_wp_local{_global_local_proj_ref.project(curr_wp(0), curr_wp(1))};
	Vector2f vehicle_to_loiter_center{curr_wp_local - curr_pos_local};

	const bool close_to_circle = vehicle_to_loiter_center.norm() < loiter_radius + _directional_guidance.switchDistance(
					     500);

	bool enforce_low_height{false};

	float target_airspeed = pos_sp_curr.cruising_speed > FLT_EPSILON ? pos_sp_curr.cruising_speed : NAN;

	if (pos_sp_next.type == position_setpoint_s::SETPOINT_TYPE_LAND && _position_setpoint_next_valid
	    && close_to_circle && _param_fw_lnd_earlycfg.get()) {
		// We're in a loiter directly before a landing WP. Enable our landing configuration (flaps,
		// landing airspeed and potentially tighter altitude control) already such that we don't
		// have to do this switch (which can cause significant altitude errors) close to the ground.
		// 中文：降落前盘旋提前进入降落构型，避免近地才放襟翼/改空速引起高度扰动。
		enforce_low_height = true;

		if (_param_fw_lnd_airspd.get() > FLT_EPSILON) {
			target_airspeed = _param_fw_lnd_airspd.get();
		}

		_flaps_setpoint = _param_fw_flaps_lnd_scl.get();
		_spoilers_setpoint = _param_fw_spoilers_lnd.get();
		_new_landing_gear_position = landing_gear_s::GEAR_DOWN;
	}

	const DirectionalGuidanceOutput sp = navigateLoiter(curr_wp_local, curr_pos_local, loiter_radius,
					     loiter_direction_ccw,
					     ground_speed,
					     _wind_vel);

	fixed_wing_lateral_setpoint_s fw_lateral_ctrl_sp{empty_lateral_control_setpoint};
	fw_lateral_ctrl_sp.timestamp = hrt_absolute_time();
	fw_lateral_ctrl_sp.course = sp.course_setpoint;
	fw_lateral_ctrl_sp.lateral_acceleration = sp.lateral_acceleration_feedforward;

	_lateral_ctrl_sp_pub.publish(fw_lateral_ctrl_sp);

	if (_landing_abort_status) {
		// 降落中止后通常先直线爬升到安全高度，再恢复普通盘旋。
		if (pos_sp_curr.alt - _current_altitude  < kClearanceAltitudeBuffer) {
			// aborted landing complete, normal loiter over landing point
			// 中文：已经回到降落点上方安全高度，清除中止状态。
			updateLandingAbortStatus(position_controller_landing_status_s::NOT_ABORTED);

		} else {
			// continue straight until vehicle has sufficient altitude
			// 中文：爬升高度不足时禁用横向加速度，优先直线爬离地面。
			_ctrl_configuration_handler.setLateralAccelMax(0.0f);

			// keep flaps in landing configuration if the airspeed is below the min airspeed (keep deployed if airspeed not valid)
			// 中文：空速低或不可用时保持降落襟翼，避免复飞早期失速余量不足。
			if (!_airspeed_valid || _airspeed_eas < _param_fw_airspd_min.get()) {
				_flaps_setpoint = _param_fw_flaps_lnd_scl.get();

			} else {
				_flaps_setpoint = 0.f;
			}
		}

		enforce_low_height = true;
	}

	const fixed_wing_longitudinal_setpoint_s fw_longitudinal_control_sp = {
		.timestamp = hrt_absolute_time(),
		.altitude = pos_sp_curr.alt,
		.height_rate = NAN,
		.equivalent_airspeed = target_airspeed,
		.pitch_direct = NAN,
		.throttle_direct = NAN
	};

	_longitudinal_ctrl_sp_pub.publish(fw_longitudinal_control_sp);

	if (pos_sp_curr.gliding_enabled) {
		_ctrl_configuration_handler.setThrottleMin(0.0f);
		_ctrl_configuration_handler.setThrottleMax(0.0f);
		_ctrl_configuration_handler.setSpeedWeight(2.0f);
	}

	_ctrl_configuration_handler.setEnforceLowHeightCondition(enforce_low_height);
}

#ifdef CONFIG_FIGURE_OF_EIGHT
void
FixedWingModeManager::controlAutoFigureEight(const float control_interval, const Vector2d &curr_pos,
		const Vector2f &ground_speed, const position_setpoint_s &pos_sp_curr)
{
	// airspeed settings
	// 中文：八字盘旋复用普通纵向控制，只是横向路径由 FigureEight 几何模块生成。
	const float target_airspeed = pos_sp_curr.cruising_speed > FLT_EPSILON ? pos_sp_curr.cruising_speed : NAN;

	Vector2f curr_pos_local{_local_pos.x, _local_pos.y};

	FigureEight::FigureEightPatternParameters params;
	params.center_pos_local = _global_local_proj_ref.project(pos_sp_curr.lat, pos_sp_curr.lon);
	params.loiter_direction_counter_clockwise = pos_sp_curr.loiter_direction_counter_clockwise;
	params.loiter_minor_radius = pos_sp_curr.loiter_minor_radius;
	params.loiter_orientation = pos_sp_curr.loiter_orientation;
	params.loiter_radius = pos_sp_curr.loiter_radius;

	const DirectionalGuidanceOutput sp = _figure_eight.updateSetpoint(curr_pos_local, ground_speed, params);

	fixed_wing_lateral_setpoint_s fw_lateral_ctrl_sp{empty_lateral_control_setpoint};
	fw_lateral_ctrl_sp.timestamp = hrt_absolute_time();
	fw_lateral_ctrl_sp.course = sp.course_setpoint;
	fw_lateral_ctrl_sp.lateral_acceleration = sp.lateral_acceleration_feedforward;

	_lateral_ctrl_sp_pub.publish(fw_lateral_ctrl_sp);

	_closest_point_on_path = _figure_eight.getClosestPoint();

	const fixed_wing_longitudinal_setpoint_s fw_longitudinal_control_sp = {
		.timestamp = hrt_absolute_time(),
		.altitude = pos_sp_curr.alt,
		.height_rate = NAN,
		.equivalent_airspeed = target_airspeed,
		.pitch_direct = NAN,
		.throttle_direct = NAN
	};

	_longitudinal_ctrl_sp_pub.publish(fw_longitudinal_control_sp);

	if (pos_sp_curr.gliding_enabled) {
		// 八字滑翔同样关闭油门并优先保速度。
		_ctrl_configuration_handler.setThrottleMin(0.0f);
		_ctrl_configuration_handler.setThrottleMax(0.0f);
		_ctrl_configuration_handler.setSpeedWeight(2.0f);
	}
}

void FixedWingModeManager::publishFigureEightStatus(const position_setpoint_s pos_sp)
{
	// 发布八字盘旋状态，供 MAVLink/地面站显示中心点、半径和方向。
	figure_eight_status_s figure_eight_status{};
	figure_eight_status.timestamp = hrt_absolute_time();
	figure_eight_status.major_radius = pos_sp.loiter_radius * (pos_sp.loiter_direction_counter_clockwise ? -1.f : 1.f);
	figure_eight_status.minor_radius = pos_sp.loiter_minor_radius;
	figure_eight_status.orientation = pos_sp.loiter_orientation;
	figure_eight_status.frame = 5; //MAV_FRAME_GLOBAL_INT
	figure_eight_status.x = static_cast<int32_t>(pos_sp.lat * 1e7);
	figure_eight_status.y = static_cast<int32_t>(pos_sp.lon * 1e7);
	figure_eight_status.z = pos_sp.alt;

	_figure_eight_status_pub.publish(figure_eight_status);
}
#endif // CONFIG_FIGURE_OF_EIGHT

void
FixedWingModeManager::control_auto_path(const float control_interval, const Vector2d &curr_pos,
					const Vector2f &ground_speed, const position_setpoint_s &pos_sp_curr)
{
	// Offboard 路径切向控制：位置点给路径上一点，速度方向给路径切线，半径字段可表达曲率。
	const float target_airspeed = pos_sp_curr.cruising_speed > FLT_EPSILON ? pos_sp_curr.cruising_speed : NAN;

	Vector2f curr_pos_local{_local_pos.x, _local_pos.y};
	Vector2f curr_wp_local = _global_local_proj_ref.project(pos_sp_curr.lat, pos_sp_curr.lon);

	// Navigate directly on position setpoint and path tangent
	// 中文：如果 Offboard 同时给了加速度，则由法向加速度估计路径曲率。
	const matrix::Vector2f velocity_2d(pos_sp_curr.vx, pos_sp_curr.vy);
	const float curvature = PX4_ISFINITE(_pos_sp_triplet.current.loiter_radius) ? 1 /
				_pos_sp_triplet.current.loiter_radius :
				0.0f;
	const DirectionalGuidanceOutput sp = navigatePathTangent(curr_pos_local, curr_wp_local, velocity_2d.normalized(),
					     ground_speed, _wind_vel, curvature);

	fixed_wing_lateral_setpoint_s fw_lateral_ctrl_sp{empty_lateral_control_setpoint};
	fw_lateral_ctrl_sp.timestamp = hrt_absolute_time();
	fw_lateral_ctrl_sp.course = sp.course_setpoint;
	fw_lateral_ctrl_sp.lateral_acceleration = sp.lateral_acceleration_feedforward;
	_lateral_ctrl_sp_pub.publish(fw_lateral_ctrl_sp);

	const fixed_wing_longitudinal_setpoint_s fw_longitudinal_control_sp = {
		.timestamp = hrt_absolute_time(),
		.altitude = pos_sp_curr.alt,
		.height_rate = NAN,
		.equivalent_airspeed = target_airspeed,
		.pitch_direct = NAN,
		.throttle_direct = NAN
	};

	_longitudinal_ctrl_sp_pub.publish(fw_longitudinal_control_sp);

	if (pos_sp_curr.gliding_enabled) {
		// 滑翔路径关闭油门，并让 TECS 更偏向速度保持。
		_ctrl_configuration_handler.setThrottleMin(0.0f);
		_ctrl_configuration_handler.setThrottleMax(0.0f);
		_ctrl_configuration_handler.setSpeedWeight(2.0f);
	}
}

void
FixedWingModeManager::control_auto_takeoff(const hrt_abstime &now, const float control_interval,
		const Vector2d &global_position, const Vector2f &ground_speed, const position_setpoint_s &pos_sp_curr)
{
	// 自动导航起飞：有水平位置估计时，沿跑道/发射点到起飞航点方向爬升。
	if (!_control_mode.flag_armed) {
		// 未解锁时保持起飞状态清零，避免上次尝试的状态泄漏。
		reset_takeoff_state();
	}

	// for now taking current position setpoint altitude as clearance altitude. this is the altitude we need to
	// clear all occlusions in the takeoff path
	// 中文：Navigator 的 takeoff 高度被视为越障高度。
	const float clearance_altitude_amsl = pos_sp_curr.alt;

	// set the altitude to something above the clearance altitude to ensure the vehicle climbs past the value
	// (navigator will accept the takeoff as complete once crossing the clearance altitude)
	// 中文：纵向 setpoint 稍高于越障高度，确保实际轨迹穿越 Navigator 的完成条件。
	const float altitude_setpoint_amsl = clearance_altitude_amsl + kClearanceAltitudeBuffer;

	const Vector2f local_2D_position{_local_pos.x, _local_pos.y};

	const float takeoff_airspeed = (_param_fw_tko_airspd.get() > FLT_EPSILON) ? _param_fw_tko_airspd.get() :
				       _param_fw_airspd_min.get();

	if (_runway_takeoff.runwayTakeoffEnabled()) {
		// 跑道起飞：RunwayTakeoff 子状态机负责滑跑、抬轮、爬升阶段的俯仰/油门限制。
		if (!_runway_takeoff.isInitialized()) {
			_runway_takeoff.init(now);
			_takeoff_init_position = global_position;
			_takeoff_ground_alt = _current_altitude;
			_launch_current_yaw = _yaw;
			// _airspeed_slew_rate_controller.setForcedValue(takeoff_airspeed); // TODO

			events::send(events::ID("fixedwing_position_control_takeoff"), events::Log::Info, "Takeoff on runway");
		}

		if (_skipping_takeoff_detection) {
			// 空中切入 takeoff 时直接视为已飞行，跳过滑跑状态。
			_runway_takeoff.forceSetFlyState();
		}

		// 更新跑道起飞状态机，输入包括目标空速、当前空速、相对起飞高度和越障高度。
		_runway_takeoff.update(now, takeoff_airspeed, _airspeed_eas, _current_altitude - _takeoff_ground_alt,
				       clearance_altitude_amsl - _takeoff_ground_alt);

		const Vector2f start_pos_local = _global_local_proj_ref.project(_takeoff_init_position(0), _takeoff_init_position(1));
		const Vector2f takeoff_waypoint_local = _global_local_proj_ref.project(pos_sp_curr.lat, pos_sp_curr.lon);

		// by default set the takeoff bearing to the takeoff yaw, but override in a mission takeoff with bearing to takeoff WP
		// 中文：默认沿起飞瞬间航向；任务起飞则可改为起飞点到 takeoff waypoint 的方位。
		float takeoff_bearing = _launch_current_yaw;

		if (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION) {
			// the bearing from runway start to the takeoff waypoint is followed until the clearance altitude is exceeded
			// 中文：任务模式下，越障高度之前沿跑道起点到起飞航点的方向。
			const Vector2f takeoff_bearing_vector = takeoff_waypoint_local - start_pos_local;

			if (takeoff_bearing_vector.norm() > FLT_EPSILON) {
				takeoff_bearing = atan2f(takeoff_bearing_vector(1), takeoff_bearing_vector(0));
			}
		}

		const DirectionalGuidanceOutput sp = navigateLine(start_pos_local, takeoff_bearing, local_2D_position, ground_speed,
						     _wind_vel);

		fixed_wing_lateral_setpoint_s fw_lateral_ctrl_sp{empty_lateral_control_setpoint};
		fw_lateral_ctrl_sp.timestamp = now;
		fw_lateral_ctrl_sp.course = sp.course_setpoint;
		fw_lateral_ctrl_sp.lateral_acceleration = sp.lateral_acceleration_feedforward;

		_lateral_ctrl_sp_pub.publish(fw_lateral_ctrl_sp);

		const float roll_wingtip_strike = getMaxRollAngleNearGround(_current_altitude, _takeoff_ground_alt);
		// 近地限制横向加速度，防止大滚转导致翼尖触地。
		_ctrl_configuration_handler.setLateralAccelMax(rollAngleToLateralAccel(roll_wingtip_strike));

		const float pitch_max = _runway_takeoff.getMaxPitch(math::radians(_param_fw_p_lim_max.get()));
		const float pitch_min = _runway_takeoff.getMinPitch(math::radians(_takeoff_pitch_min.get()),
					math::radians(_param_fw_p_lim_min.get()));

		const fixed_wing_longitudinal_setpoint_s fw_longitudinal_control_sp = {
			.timestamp = now,
			.altitude = altitude_setpoint_amsl,
			.height_rate = NAN,
			.equivalent_airspeed = takeoff_airspeed,
			.pitch_direct = _runway_takeoff.getPitch(),
			.throttle_direct = _runway_takeoff.getThrottle(_param_fw_thr_idle.get())
		};

		_longitudinal_ctrl_sp_pub.publish(fw_longitudinal_control_sp);

		_ctrl_configuration_handler.setPitchMin(pitch_min);
		_ctrl_configuration_handler.setPitchMax(pitch_max);
		_ctrl_configuration_handler.setClimbRateTarget(_param_fw_t_clmb_max.get());
		// 起飞阶段禁用 underspeed 保护，避免 TECS 因低空速而压低机头。
		_ctrl_configuration_handler.setDisableUnderspeedProtection(true);

		_flaps_setpoint = _param_fw_flaps_to_scl.get();

		// retract ladning gear once passed the climbout state
		// 中文：通过爬升离地阶段后收起起落架。
		if (_runway_takeoff.getState() > RunwayTakeoffState::CLIMBOUT) {
			_new_landing_gear_position = landing_gear_s::GEAR_UP;
		}

		fixed_wing_runway_control_s fw_runway_control{};
		fw_runway_control.timestamp = now;
		fw_runway_control.wheel_steering_enabled = true;
		// 跑道滑跑期间允许用 yaw 杆微调前轮转向。
		fw_runway_control.wheel_steering_nudging_rate = _param_rwto_nudge.get() ? _sticks.getYaw() : 0.f;

		_fixed_wing_runway_control_pub.publish(fw_runway_control);

	} else {
		/* Perform launch detection */
		// 中文：非跑道起飞使用手抛/弹射发射检测，检测到后才开始真正控制爬升。
		if (!_skipping_takeoff_detection && _param_fw_laun_detcn_on.get() &&
		    _launchDetector.getLaunchDetected() < launch_detection_status_s::STATE_FLYING) {

			if (_control_mode.flag_armed) {
				/* Perform launch detection */

				/* Detect launch using body X (forward) acceleration */
				// 中文：用机体 X 轴前向加速度判断发射动作。
				_launchDetector.update(control_interval, _body_acceleration_x);
			}

		} else	{
			/* no takeoff detection --> fly */
			// 中文：关闭发射检测或需要跳过检测时，直接进入飞行状态。
			_launchDetector.forceSetFlyState();
		}

		if (!_launch_detected && _launchDetector.getLaunchDetected() > launch_detection_status_s::STATE_WAITING_FOR_LAUNCH) {
			// 首次检测到发射时锁定起飞原点、高度和航向。
			_launch_detected = true;
			_takeoff_init_position = global_position;
			_takeoff_ground_alt = _current_altitude;
			_launch_current_yaw = _yaw;
		}

		const Vector2f launch_local_position = _global_local_proj_ref.project(_takeoff_init_position(0),
						       _takeoff_init_position(1));

		// by default set the takeoff bearing to the takeoff yaw, but override in a mission takeoff with bearing to takeoff WP
		// 中文：发射起飞默认沿发射瞬间航向；若 takeoff 航点足够远则朝航点方向爬升。
		float takeoff_bearing = _launch_current_yaw;
		const float distance_to_takeoff_wp = get_distance_to_next_waypoint(_takeoff_init_position(0), _takeoff_init_position(1),
						     pos_sp_curr.lat, pos_sp_curr.lon);

		if (pos_sp_curr.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF && distance_to_takeoff_wp > 10.f) {
			// if a takeoff waypoint is set (and not straight above current location), use the bearing to the takeoff waypoint
			// 中文：起飞点不是当前正上方时，用起飞点到 waypoint 的方位作为爬升航向。
			const Vector2f takeoff_waypoint_local = _global_local_proj_ref.project(pos_sp_curr.lat, pos_sp_curr.lon);
			const Vector2f takeoff_bearing_vector = takeoff_waypoint_local - launch_local_position;

			if (takeoff_bearing_vector.norm() > FLT_EPSILON) {
				takeoff_bearing = atan2f(takeoff_bearing_vector(1), takeoff_bearing_vector(0));
			}
		}

		/* Set control values depending on the detection state */
		// 中文：根据发射状态决定是等待发射还是进入爬升控制。
		if (_launchDetector.getLaunchDetected() > launch_detection_status_s::STATE_WAITING_FOR_LAUNCH) {
			/* Launch has been detected, hence we have to control the plane. */
			// 中文：发射已确认，横向沿发射线，纵向给最大爬升率和起飞空速。

			const DirectionalGuidanceOutput sp = navigateLine(launch_local_position, takeoff_bearing, local_2D_position,
							     ground_speed,
							     _wind_vel);

			fixed_wing_lateral_setpoint_s fw_lateral_ctrl_sp{empty_lateral_control_setpoint};
			fw_lateral_ctrl_sp.timestamp = now;
			fw_lateral_ctrl_sp.course = sp.course_setpoint;
			fw_lateral_ctrl_sp.lateral_acceleration = sp.lateral_acceleration_feedforward;

			_lateral_ctrl_sp_pub.publish(fw_lateral_ctrl_sp);

			const float roll_wingtip_strike = getMaxRollAngleNearGround(_current_altitude, _takeoff_ground_alt);
			_ctrl_configuration_handler.setLateralAccelMax(rollAngleToLateralAccel(roll_wingtip_strike));

			const float max_takeoff_throttle = (_launchDetector.getLaunchDetected() < launch_detection_status_s::STATE_FLYING) ?
							   _param_fw_thr_idle.get() : NAN;
			// 发射后但尚未进入 FLYING 前，把油门上限压到 idle；进入 FLYING 后交给 TECS/默认限制。
			const fixed_wing_longitudinal_setpoint_s fw_longitudinal_control_sp = {
				.timestamp = now,
				.altitude = NAN,
				.height_rate = _param_fw_t_clmb_max.get(),
				.equivalent_airspeed = takeoff_airspeed,
				.pitch_direct = NAN,
				.throttle_direct = NAN
			};

			_longitudinal_ctrl_sp_pub.publish(fw_longitudinal_control_sp);

			_ctrl_configuration_handler.setPitchMin(radians(_takeoff_pitch_min.get()));
			_ctrl_configuration_handler.setThrottleMax(max_takeoff_throttle);
			_ctrl_configuration_handler.setClimbRateTarget(_param_fw_t_clmb_max.get());
			_ctrl_configuration_handler.setDisableUnderspeedProtection(true);

			//float yaw_body = _yaw; // yaw is not controlled, so set setpoint to current yaw

		} else {
			// 等待发射：平翼、最小起飞俯仰、idle 油门，姿态控制器不应积累横向误差。
			fixed_wing_lateral_setpoint_s fw_lateral_ctrl_sp{empty_lateral_control_setpoint};
			fw_lateral_ctrl_sp.timestamp = now;
			fw_lateral_ctrl_sp.lateral_acceleration = 0.f;
			/* Tell the attitude controller to stop integrating while we are waiting for the launch */
			// 中文：横向 setpoint 为 0，避免未发射时滚转积分器积累。
			_lateral_ctrl_sp_pub.publish(fw_lateral_ctrl_sp);

			fixed_wing_longitudinal_setpoint_s long_control_sp{empty_longitudinal_control_setpoint};
			long_control_sp.timestamp = now;
			long_control_sp.pitch_direct = radians(_takeoff_pitch_min.get());
			long_control_sp.throttle_direct = _param_fw_thr_idle.get();
			_longitudinal_ctrl_sp_pub.publish(long_control_sp);
		}

		launch_detection_status_s launch_detection_status;
		// 发布发射检测状态，供日志和地面站查看 WAITING/LAUNCHED/FLYING。
		launch_detection_status.timestamp = now;
		launch_detection_status.launch_detection_state = _launchDetector.getLaunchDetected();
		_launch_detection_status_pub.publish(launch_detection_status);
	}

	_flaps_setpoint = _param_fw_flaps_to_scl.get();

	if (!_vehicle_status.in_transition_to_fw) {
		// 起飞阶段也发布 local position setpoint 供可视化/日志使用。
		publishLocalPositionSetpoint(pos_sp_curr);
	}
}

void
FixedWingModeManager::control_auto_takeoff_no_nav(const hrt_abstime &now, const float control_interval,
		const float current_setpoint_altitude_amsl)
{
	// 无水平导航起飞：只给平翼横向 setpoint 和爬升纵向 setpoint，用于位置不可用的起飞保护。
	if (!_control_mode.flag_armed) {
		reset_takeoff_state();
	}

	const float takeoff_airspeed = (_param_fw_tko_airspd.get() > FLT_EPSILON) ? _param_fw_tko_airspd.get() :
				       _param_fw_airspd_min.get();

	if (_runway_takeoff.runwayTakeoffEnabled()) {
		// 无导航跑道起飞仍使用 RunwayTakeoff 状态机，但横向保持平翼。
		if (!_runway_takeoff.isInitialized()) {
			_runway_takeoff.init(now);
			_takeoff_ground_alt = _current_altitude;
			_launch_current_yaw = _yaw;
		}

		if (_skipping_takeoff_detection) {
			_runway_takeoff.forceSetFlyState();
		}

		const float clearance_altitude_amsl = _current_altitude + 20.f; // hard code it to 20m above takeoff point
		// 中文：无导航时硬编码越障高度为当前高度上方 20 m。

		_runway_takeoff.update(now, takeoff_airspeed, _airspeed_eas, _current_altitude - _takeoff_ground_alt,
				       clearance_altitude_amsl - _takeoff_ground_alt);

		fixed_wing_lateral_setpoint_s fw_lateral_ctrl_sp{empty_lateral_control_setpoint};
		fw_lateral_ctrl_sp.timestamp = now;
		fw_lateral_ctrl_sp.lateral_acceleration = 0.f; // level wings
		// 中文：无水平位置控制时不做横向转弯，只保持平翼。

		_lateral_ctrl_sp_pub.publish(fw_lateral_ctrl_sp);

		const float pitch_max = _runway_takeoff.getMaxPitch(math::radians(_param_fw_p_lim_max.get()));
		const float pitch_min = _runway_takeoff.getMinPitch(math::radians(_takeoff_pitch_min.get()),
					math::radians(_param_fw_p_lim_min.get()));

		const fixed_wing_longitudinal_setpoint_s fw_longitudinal_control_sp = {
			.timestamp = now,
			.altitude = NAN,
			.height_rate = _param_fw_t_clmb_max.get(),
			.equivalent_airspeed = takeoff_airspeed,
			.pitch_direct = _runway_takeoff.getPitch(),
			.throttle_direct = _runway_takeoff.getThrottle(_param_fw_thr_idle.get())
		};

		_longitudinal_ctrl_sp_pub.publish(fw_longitudinal_control_sp);

		_ctrl_configuration_handler.setPitchMin(pitch_min);
		_ctrl_configuration_handler.setPitchMax(pitch_max);
		_ctrl_configuration_handler.setDisableUnderspeedProtection(true);

		_flaps_setpoint = _param_fw_flaps_to_scl.get();

		// retract ladning gear once passed the climbout state
		// 中文：通过爬升离地阶段后收起起落架。
		if (_runway_takeoff.getState() > RunwayTakeoffState::CLIMBOUT) {
			_new_landing_gear_position = landing_gear_s::GEAR_UP;
		}

		fixed_wing_runway_control_s fw_runway_control{};
		fw_runway_control.timestamp = now;
		fw_runway_control.wheel_steering_enabled = true;
		fw_runway_control.wheel_steering_nudging_rate = _param_rwto_nudge.get() ? _sticks.getYaw() : 0.f;

		_fixed_wing_runway_control_pub.publish(fw_runway_control);

	} else {
		/* Perform launch detection */
		// 中文：无导航非跑道起飞同样只做发射检测和纵向爬升。
		if (!_skipping_takeoff_detection && _param_fw_laun_detcn_on.get() &&
		    _launchDetector.getLaunchDetected() < launch_detection_status_s::STATE_FLYING) {

			if (_control_mode.flag_armed) {
				/* Detect launch using body X (forward) acceleration */
				// 中文：用机体前向加速度确认手抛/弹射发射。
				_launchDetector.update(control_interval, _body_acceleration_x);
			}

		} else	{
			/* no takeoff detection --> fly */
			// 中文：不启用检测时直接切为飞行状态。
			_launchDetector.forceSetFlyState();
		}

		if (!_launch_detected && _launchDetector.getLaunchDetected() > launch_detection_status_s::STATE_WAITING_FOR_LAUNCH) {
			_launch_detected = true;
			_takeoff_ground_alt = _current_altitude;
		}

		/* Launch has been detected, hence we have to control the plane. */
		// 中文：无导航起飞的横向命令始终为平翼，纵向命令为最大爬升。

		fixed_wing_lateral_setpoint_s fw_lateral_ctrl_sp{empty_lateral_control_setpoint};
		fw_lateral_ctrl_sp.timestamp = now;
		fw_lateral_ctrl_sp.lateral_acceleration = 0.f; // level wings

		_lateral_ctrl_sp_pub.publish(fw_lateral_ctrl_sp);

		const float max_takeoff_throttle = (_launchDetector.getLaunchDetected() < launch_detection_status_s::STATE_FLYING) ?
						   _param_fw_thr_idle.get() : NAN;
		const fixed_wing_longitudinal_setpoint_s fw_longitudinal_control_sp = {
			.timestamp = now,
			.altitude = NAN,
			.height_rate = _param_fw_t_clmb_max.get(),
			.equivalent_airspeed = takeoff_airspeed,
			.pitch_direct = NAN,
			.throttle_direct = NAN
		};

		_longitudinal_ctrl_sp_pub.publish(fw_longitudinal_control_sp);

		_ctrl_configuration_handler.setPitchMin(radians(_takeoff_pitch_min.get()));
		_ctrl_configuration_handler.setThrottleMax(max_takeoff_throttle);
		_ctrl_configuration_handler.setClimbRateTarget(_param_fw_t_clmb_max.get());
		_ctrl_configuration_handler.setDisableUnderspeedProtection(true);

		launch_detection_status_s launch_detection_status;
		launch_detection_status.timestamp = now;
		launch_detection_status.launch_detection_state = _launchDetector.getLaunchDetected();
		_launch_detection_status_pub.publish(launch_detection_status);
	}

	_flaps_setpoint = _param_fw_flaps_to_scl.get();
}

void
FixedWingModeManager::control_auto_landing_straight(const hrt_abstime &now, const float control_interval,
		const Vector2f &ground_speed, const position_setpoint_s &pos_sp_prev, const position_setpoint_s &pos_sp_curr)
{
	// 直线自动降落：沿上一航点到降落点形成下滑道，近地进入 flare。
	const float airspeed_land = (_param_fw_lnd_airspd.get() > FLT_EPSILON) ? _param_fw_lnd_airspd.get() :
				    _param_fw_airspd_min.get();

	_ctrl_configuration_handler.setEnforceLowHeightCondition(true);

	// now handle position
	// 中文：降落点从全局坐标投影到本地平面，后续几何全部在 local XY 中计算。
	const Vector2f local_position{_local_pos.x, _local_pos.y};
	Vector2f local_land_point = _global_local_proj_ref.project(pos_sp_curr.lat, pos_sp_curr.lon);

	initializeAutoLanding(now, pos_sp_prev, pos_sp_curr.alt, local_position, local_land_point);

	// touchdown may get nudged by manual inputs
	// 中文：允许飞手用 yaw 杆横向微调触地点或进近线。
	local_land_point = calculateTouchdownPosition(control_interval, local_land_point);
	const Vector2f landing_approach_vector = calculateLandingApproachVector();

	// calculate the altitude setpoint based on the landing glide slope
	// 中文：沿进近方向计算到触地点的距离，再由下滑角得到相对高度 setpoint。
	const float along_track_dist_to_touchdown = -landing_approach_vector.unit_or_zero().dot(
				local_position - local_land_point);
	const float glide_slope = _landing_approach_entrance_rel_alt / _landing_approach_entrance_offset_vector.norm();

	// NOTE: this relative altitude can go below zero, this is intentional. in the case the vehicle is tracking the glide
	// slope at an offset above the track, making the altitude setpoint constant on intersection with terrain causes
	// an increase in throttle (to slow down and smoothly intersect the flattened altitude setpoint), which is undesirable
	// directly before the flare. instead, we keep the steady state behavior, and let the flare get triggered once at
	// the desired altitude
	// 中文：允许相对高度略低于 0 是有意的，避免 flare 前高度 setpoint 被强行压平造成油门突增。
	const float glide_slope_rel_alt = math::min(along_track_dist_to_touchdown * glide_slope,
					  _landing_approach_entrance_rel_alt);

	const bool abort_on_terrain_measurement_timeout = checkLandingAbortBitMask(_param_fw_lnd_abort.get(),
			position_controller_landing_status_s::TERRAIN_NOT_FOUND);
	const bool abort_on_terrain_timeout = checkLandingAbortBitMask(_param_fw_lnd_abort.get(),
					      position_controller_landing_status_s::TERRAIN_TIMEOUT);
	const float terrain_alt = getLandingTerrainAltitudeEstimate(now, pos_sp_curr.alt, abort_on_terrain_measurement_timeout,
				  abort_on_terrain_timeout);
	// 根据参数选择是相对地形保持下滑道，还是相对任务降落点高度保持下滑道。
	const float glide_slope_reference_alt = (_param_fw_lnd_useter.get() ==
						TerrainEstimateUseOnLanding::kFollowTerrainRelativeLandingGlideSlope) ? terrain_alt : pos_sp_curr.alt;

	float altitude_setpoint;

	if (_current_altitude > glide_slope_reference_alt + glide_slope_rel_alt) {
		// descend to the glide slope
		// 中文：飞机高于下滑道时，目标高度设为下滑道高度。
		altitude_setpoint = glide_slope_reference_alt + glide_slope_rel_alt;

	} else {
		// continue horizontally
		// 中文：飞机低于/已接触下滑道时，不继续降低 setpoint，而是先水平保持当前高度。
		altitude_setpoint = _current_altitude;
	}

	// flare at the maximum of the altitude determined by the time before touchdown and a minimum flare altitude
	// 中文：flare 触发高度取“按当前下降率和 flare 时间推算的高度”与最小 flare 高度的较大值。
	const float flare_rel_alt = math::max(_param_fw_lnd_fl_time.get() * _local_pos.vz, _param_fw_lnd_flalt.get());


	// the terrain estimate (if enabled) is always used to determine the flaring altitude
	// 中文：即使下滑道不用地形，flare 触发高度也会使用地形估计，以贴合真实离地高度。
	if ((_current_altitude < terrain_alt + flare_rel_alt) || _flare_states.flaring) {
		// flare and land with minimal speed

		// flaring is a "point of no return"
		// 中文：首次进入 flare 时记录开始时间和初始下降率，后续按时间平滑过渡。
		if (!_flare_states.flaring) {
			_flare_states.flaring = true;
			_flare_states.start_time = now;
			_flare_states.initial_height_rate_setpoint = -_local_pos.vz;
			events::send(events::ID("fixedwing_position_control_landing_flaring"), events::Log::Info,
				     "Landing, flaring");
		}

		// ramp in flare limits and setpoints with the flare time, command a soft touchdown
		// 中文：flare_ramp_interpolator 从 0 到 1，逐渐收油门、调整俯仰限制和下降率。
		const float seconds_since_flare_start = hrt_elapsed_time(&_flare_states.start_time) * 1.e-6f;
		const float flare_ramp_interpolator = math::constrain(seconds_since_flare_start / _param_fw_lnd_fl_time.get(), 0.0f,
						      1.0f);

		/* lateral guidance first, because npfg will adjust the airspeed setpoint if necessary */
		// 中文：先算横向制导，因为 NPFG 可能会提高所需最小空速。
		const Vector2f local_approach_entrance = local_land_point - landing_approach_vector;

		const DirectionalGuidanceOutput sp = navigateLine(local_approach_entrance, local_land_point, local_position,
						     ground_speed,
						     _wind_vel);
		fixed_wing_lateral_setpoint_s fw_lateral_ctrl_sp{empty_lateral_control_setpoint};
		fw_lateral_ctrl_sp.timestamp = now;
		fw_lateral_ctrl_sp.course = sp.course_setpoint;
		fw_lateral_ctrl_sp.lateral_acceleration = sp.lateral_acceleration_feedforward;
		_lateral_ctrl_sp_pub.publish(fw_lateral_ctrl_sp);

		const float roll_wingtip_strike = getMaxRollAngleNearGround(_current_altitude, _takeoff_ground_alt);
		_ctrl_configuration_handler.setLateralAccelMax(rollAngleToLateralAccel(roll_wingtip_strike));

		/* longitudinal guidance */
		// 中文：纵向 flare 阶段：高度 setpoint 沿原下滑道，下降率逐渐过渡到 flare sink。

		const float flare_ramp_interpolator_sqrt = sqrtf(flare_ramp_interpolator);

		// Use separate ramp for the altitude setpoint that's starting only when the other is finished
		// to allow motor to be ramped down before height rate setpoint is adapted for flaring.
		// 中文：下降率 ramp 延后启动，先让油门降下来，再改变下降率，减小触地前推力扰动。
		const float flare_hieght_rate_interpolator = math::constrain((seconds_since_flare_start -
				_param_fw_lnd_fl_time.get()) / (_param_fw_lnd_fl_time.get()), 0.f, 1.f);
		const float flare_hieght_rate_interpolator_sqrt = sqrt(flare_hieght_rate_interpolator);

		const float height_rate_setpoint = flare_hieght_rate_interpolator_sqrt * (-_param_fw_lnd_fl_sink.get()) +
						   (1.0f - flare_hieght_rate_interpolator_sqrt) * _flare_states.initial_height_rate_setpoint;

		float pitch_min_rad = flare_ramp_interpolator_sqrt * radians(_param_fw_lnd_fl_pmin.get()) +
				      (1.0f - flare_ramp_interpolator_sqrt) * radians(_param_fw_p_lim_min.get());
		float pitch_max_rad = flare_ramp_interpolator_sqrt * radians(_param_fw_lnd_fl_pmax.get()) +
				      (1.0f - flare_ramp_interpolator_sqrt) * radians(_param_fw_p_lim_max.get());

		if (_param_fw_lnd_td_time.get() > FLT_EPSILON) {
			// 触地后可逐渐把俯仰限制拉到跑道起飞/滑跑俯仰参数，用于轮式飞机落地滑跑姿态。
			const float touchdown_time = math::max(_param_fw_lnd_td_time.get(), _param_fw_lnd_fl_time.get());

			const float touchdown_interpolator = math::constrain((seconds_since_flare_start - touchdown_time) /
							     POST_TOUCHDOWN_CLAMP_TIME, 0.0f,
							     1.0f);

			pitch_max_rad = touchdown_interpolator * math::radians(_param_rwto_psp.get()) + (1.0f - touchdown_interpolator) *
					pitch_max_rad;
			pitch_min_rad = touchdown_interpolator * math::radians(_param_rwto_psp.get()) + (1.0f - touchdown_interpolator) *
					pitch_min_rad;
		}

		// idle throttle may be >0 for internal combustion engines
		// normally set to zero for electric motors
		// 中文：flare 中油门上限从最大油门渐变到 idle，兼容油机 idle 非零的情况。
		const float throttle_max = flare_ramp_interpolator_sqrt * _param_fw_thr_idle.get() +
					   (1.0f - flare_ramp_interpolator_sqrt) *
					   _param_fw_thr_max.get();

		const fixed_wing_longitudinal_setpoint_s fw_longitudinal_control_sp = {
			.timestamp = now,
			.altitude = altitude_setpoint,
			.height_rate = height_rate_setpoint,
			.equivalent_airspeed = airspeed_land,
			.pitch_direct = NAN,
			.throttle_direct = NAN
		};

		_longitudinal_ctrl_sp_pub.publish(fw_longitudinal_control_sp);

		_ctrl_configuration_handler.setPitchMin(pitch_min_rad);
		_ctrl_configuration_handler.setPitchMax(pitch_max_rad);
		_ctrl_configuration_handler.setThrottleMax(throttle_max);
		_ctrl_configuration_handler.setThrottleMin(_param_fw_thr_idle.get());
		_ctrl_configuration_handler.setDisableUnderspeedProtection(true);

	} else {

		// follow the glide slope
		// 中文：未进入 flare 时，按直线下滑道正常进近。

		/* lateral guidance */
		// 中文：横向沿进近入口到触地点的直线飞行。

		const Vector2f local_approach_entrance = local_land_point - landing_approach_vector;

		const DirectionalGuidanceOutput sp = navigateLine(local_approach_entrance, local_land_point, local_position,
						     ground_speed,
						     _wind_vel);
		fixed_wing_lateral_setpoint_s fw_lateral_ctrl_sp{empty_lateral_control_setpoint};
		fw_lateral_ctrl_sp.timestamp = hrt_absolute_time();
		fw_lateral_ctrl_sp.course = sp.course_setpoint;
		fw_lateral_ctrl_sp.lateral_acceleration = sp.lateral_acceleration_feedforward;
		_lateral_ctrl_sp_pub.publish(fw_lateral_ctrl_sp);

		_ctrl_configuration_handler.setLateralAccelMax(rollAngleToLateralAccel(getMaxRollAngleNearGround(_current_altitude,
				terrain_alt)));

		/* longitudinal guidance */
		// 中文：纵向跟踪下滑道高度，并放宽最大下降率以允许按下滑角下降。

		// Open the desired max sink rate to encompass the glide slope.
		// x/sqrt(x^2+1) = sin(arctan(x))
		// 中文：由下滑坡度和降落空速估算需要的下降率幅值。
		const float glide_slope_sink_rate = airspeed_land * glide_slope / sqrtf(glide_slope * glide_slope + 1.0f);
		const float desired_max_sinkrate = math::max(glide_slope_sink_rate, _param_sinkrate_target.get());

		const fixed_wing_longitudinal_setpoint_s fw_longitudinal_control_sp = {
			.timestamp = hrt_absolute_time(),
			.altitude = altitude_setpoint,
			.height_rate = NAN,
			.equivalent_airspeed = airspeed_land,
			.pitch_direct = NAN,
			.throttle_direct = NAN
		};

		_longitudinal_ctrl_sp_pub.publish(fw_longitudinal_control_sp);

		_ctrl_configuration_handler.setThrottleMin(_param_fw_thr_idle.get());
		_ctrl_configuration_handler.setThrottleMax(_landed ? _param_fw_thr_idle.get() : NAN);
		_ctrl_configuration_handler.setSinkRateTarget(desired_max_sinkrate);
	}

	fixed_wing_runway_control_s fw_runway_control{};
	fw_runway_control.timestamp = now;
	fw_runway_control.wheel_steering_enabled = true;
	// 降落滑跑阶段允许 yaw 杆微调方向；是否启用由 FW_LND_NUDGE 决定。
	fw_runway_control.wheel_steering_nudging_rate = _param_fw_lnd_nudge.get() > LandingNudgingOption::kNudgingDisabled ?
			_sticks.getYaw() : 0.f;

	_fixed_wing_runway_control_pub.publish(fw_runway_control);

	_flaps_setpoint = _param_fw_flaps_lnd_scl.get();
	_spoilers_setpoint = _param_fw_spoilers_lnd.get();

	// deploy gear as soon as we're in land mode, if not already done before
	// 中文：进入降落模式即放起落架，若降落前盘旋已提前放下，这里保持一致。
	_new_landing_gear_position = landing_gear_s::GEAR_DOWN;

	if (!_vehicle_status.in_transition_to_fw) {
		publishLocalPositionSetpoint(pos_sp_curr);
	}

	landing_status_publish();
}

void
FixedWingModeManager::control_auto_landing_circular(const hrt_abstime &now, const float control_interval,
		const Vector2f &ground_speed, const position_setpoint_s &pos_sp_curr)
{
	// 盘旋自动降落：无上一航点时使用降落点盘旋下降，近地 flare。
	const float airspeed_land = (_param_fw_lnd_airspd.get() > FLT_EPSILON) ? _param_fw_lnd_airspd.get() :
				    _param_fw_airspd_min.get();

	_ctrl_configuration_handler.setEnforceLowHeightCondition(true);


	const Vector2f local_position{_local_pos.x, _local_pos.y};
	Vector2f local_landing_orbit_center = _global_local_proj_ref.project(pos_sp_curr.lat, pos_sp_curr.lon);

	if (_time_started_landing == 0) {
		// save time at which we started landing and reset landing abort status
		// 中文：首次进入盘旋降落时初始化降落状态和计时。
		reset_landing_state();
		_time_started_landing = now;
	}

	const bool abort_on_terrain_timeout = checkLandingAbortBitMask(_param_fw_lnd_abort.get(),
					      position_controller_landing_status_s::TERRAIN_TIMEOUT);
	const float terrain_alt = getLandingTerrainAltitudeEstimate(now, pos_sp_curr.alt, false, abort_on_terrain_timeout);

	// flare at the maximum of the altitude determined by the time before touchdown and a minimum flare altitude
	// 中文：盘旋降落同样使用离地高度阈值触发 flare。
	const float flare_rel_alt = math::max(_param_fw_lnd_fl_time.get() * _local_pos.vz, _param_fw_lnd_flalt.get());

	float loiter_radius = fabsf(pos_sp_curr.loiter_radius);
	bool loiter_direction_ccw = pos_sp_curr.loiter_direction_counter_clockwise;

	if (loiter_radius < FLT_EPSILON) {
		// 使用默认盘旋半径及其符号确定方向。
		loiter_radius = fabsf(_param_nav_loiter_rad.get());
		loiter_direction_ccw = _param_nav_loiter_rad.get() < -FLT_EPSILON;
	}

	if ((_current_altitude < terrain_alt + flare_rel_alt) || _flare_states.flaring) {
		// flare and land with minimal speed

		// flaring is a "point of no return"
		// 中文：首次进入 flare 后锁定状态，后续不再回到普通盘旋下降。
		if (!_flare_states.flaring) {
			_flare_states.flaring = true;
			_flare_states.start_time = now;
			_flare_states.initial_height_rate_setpoint = -_local_pos.vz;
			events::send(events::ID("fixedwing_position_control_landing_circle_flaring"), events::Log::Info,
				     "Landing, flaring");
		}

		// ramp in flare limits and setpoints with the flare time, command a soft touchdown
		// 中文：盘旋 flare 的纵向限制与直线降落相同，只是横向仍跟踪圆。
		const float seconds_since_flare_start = hrt_elapsed_time(&_flare_states.start_time) * 1.e-6f;
		const float flare_ramp_interpolator = math::constrain(seconds_since_flare_start / _param_fw_lnd_fl_time.get(), 0.0f,
						      1.0f);

		/* lateral guidance first, because npfg will adjust the airspeed setpoint if necessary */
		// 中文：横向继续盘旋，直到落地检测或上层模式退出。
		const DirectionalGuidanceOutput sp = navigateLoiter(local_landing_orbit_center, local_position, loiter_radius,
						     loiter_direction_ccw,
						     ground_speed, _wind_vel);
		fixed_wing_lateral_setpoint_s fw_lateral_ctrl_sp{empty_lateral_control_setpoint};
		fw_lateral_ctrl_sp.timestamp = now;
		fw_lateral_ctrl_sp.course = sp.course_setpoint;
		fw_lateral_ctrl_sp.lateral_acceleration = sp.lateral_acceleration_feedforward;

		_lateral_ctrl_sp_pub.publish(fw_lateral_ctrl_sp);
		/* longitudinal guidance */
		// 中文：纵向用 height_rate setpoint 软着陆，油门逐渐降到 idle。

		const float flare_ramp_interpolator_sqrt = sqrtf(flare_ramp_interpolator);

		const float height_rate_setpoint = flare_ramp_interpolator_sqrt * (-_param_fw_lnd_fl_sink.get()) +
						   (1.0f - flare_ramp_interpolator_sqrt) * _flare_states.initial_height_rate_setpoint;

		float pitch_min_rad = flare_ramp_interpolator_sqrt * radians(_param_fw_lnd_fl_pmin.get()) +
				      (1.0f - flare_ramp_interpolator_sqrt) * radians(_param_fw_p_lim_min.get());
		float pitch_max_rad = flare_ramp_interpolator_sqrt * radians(_param_fw_lnd_fl_pmax.get()) +
				      (1.0f - flare_ramp_interpolator_sqrt) * radians(_param_fw_p_lim_max.get());

		if (_param_fw_lnd_td_time.get() > FLT_EPSILON) {
			// 触地后俯仰上下限渐变到跑道滑跑俯仰目标。
			const float touchdown_time = math::max(_param_fw_lnd_td_time.get(), _param_fw_lnd_fl_time.get());

			const float touchdown_interpolator = math::constrain((seconds_since_flare_start - touchdown_time) /
							     POST_TOUCHDOWN_CLAMP_TIME, 0.0f, 1.0f);

			pitch_max_rad = touchdown_interpolator * math::radians(_param_rwto_psp.get()) + (1.0f - touchdown_interpolator) *
					pitch_max_rad;
			pitch_min_rad = touchdown_interpolator * math::radians(_param_rwto_psp.get()) + (1.0f - touchdown_interpolator) *
					pitch_min_rad;
		}

		// idle throttle may be >0 for internal combustion engines
		// normally set to zero for electric motors
		// 中文：油门上限按 flare ramp 从最大油门收敛到 idle。
		const float throttle_max = flare_ramp_interpolator_sqrt * _param_fw_thr_idle.get() +
					   (1.0f - flare_ramp_interpolator_sqrt) *
					   _param_fw_thr_max.get();
		const fixed_wing_longitudinal_setpoint_s fw_longitudinal_control_sp = {
			.timestamp = now,
			.altitude = NAN,
			.height_rate = height_rate_setpoint,
			.equivalent_airspeed = airspeed_land,
			.pitch_direct = NAN,
			.throttle_direct = NAN
		};

		_longitudinal_ctrl_sp_pub.publish(fw_longitudinal_control_sp);

		_ctrl_configuration_handler.setPitchMin(pitch_min_rad);
		_ctrl_configuration_handler.setPitchMax(pitch_max_rad);
		_ctrl_configuration_handler.setThrottleMax(throttle_max);
		_ctrl_configuration_handler.setThrottleMin(_param_fw_thr_idle.get());
		_ctrl_configuration_handler.setDisableUnderspeedProtection(true);

	} else {

		// follow the glide slope
		// 中文：未 flare 前盘旋下降，横向保持圆轨迹，纵向直接给下降率。
		const DirectionalGuidanceOutput sp = navigateLoiter(local_landing_orbit_center, local_position, loiter_radius,
						     loiter_direction_ccw,
						     ground_speed, _wind_vel);
		fixed_wing_lateral_setpoint_s fw_lateral_ctrl_sp{empty_lateral_control_setpoint};
		fw_lateral_ctrl_sp.timestamp = now;
		fw_lateral_ctrl_sp.course = sp.course_setpoint;
		fw_lateral_ctrl_sp.lateral_acceleration = sp.lateral_acceleration_feedforward;

		_lateral_ctrl_sp_pub.publish(fw_lateral_ctrl_sp);

		/* longitudinal guidance */
		// 中文：根据降落下滑角和降落空速计算盘旋下降率。

		// Open the desired max sink rate to encompass the glide slope.
		// x/sqrt(x^2+1) = sin(arctan(x))
		// 中文：sin(arctan(x)) 把坡度换算为空速方向上的垂向分量。
		const float glide_slope = math::radians(_param_fw_lnd_ang.get());
		const float glide_slope_sink_rate = airspeed_land * glide_slope / sqrtf(glide_slope * glide_slope + 1.0f);
		const float desired_max_sinkrate = math::max(glide_slope_sink_rate, _param_sinkrate_target.get());

		const fixed_wing_longitudinal_setpoint_s fw_longitudinal_control_sp = {
			.timestamp = now,
			.altitude = NAN,
			.height_rate = -glide_slope_sink_rate,
			.equivalent_airspeed = airspeed_land,
			.pitch_direct = NAN,
			.throttle_direct = NAN
		};

		_longitudinal_ctrl_sp_pub.publish(fw_longitudinal_control_sp);

		_ctrl_configuration_handler.setThrottleMin(_param_fw_thr_idle.get());
		_ctrl_configuration_handler.setThrottleMax(_landed ? _param_fw_thr_idle.get() : NAN);
		_ctrl_configuration_handler.setSinkRateTarget(desired_max_sinkrate);
	}

	fixed_wing_runway_control_s fw_runway_control{};
	fw_runway_control.timestamp = now;
	fw_runway_control.wheel_steering_enabled = true;
	// 降落滑跑/近地阶段发布跑道转向控制。
	fw_runway_control.wheel_steering_nudging_rate = _param_fw_lnd_nudge.get() > LandingNudgingOption::kNudgingDisabled ?
			_sticks.getYaw() : 0.f;

	_fixed_wing_runway_control_pub.publish(fw_runway_control);

	_ctrl_configuration_handler.setLateralAccelMax(rollAngleToLateralAccel(getMaxRollAngleNearGround(_current_altitude,
			terrain_alt)));

	_flaps_setpoint = _param_fw_flaps_lnd_scl.get();
	_spoilers_setpoint = _param_fw_spoilers_lnd.get();

	if (!_vehicle_status.in_transition_to_fw) {
		publishLocalPositionSetpoint(pos_sp_curr);
	}

	landing_status_publish();
	publishOrbitStatus(pos_sp_curr);
}

void
FixedWingModeManager::control_manual_altitude(const float control_interval, const Vector2d &curr_pos,
		const Vector2f &ground_speed)
{
	// 手动高度模式：纵向由杆量控制高度率，横向直接由 roll 杆映射为横向加速度。
	updateManualTakeoffStatus();

	const float height_rate_sp = getManualHeightRateSetpoint();

	// TECS may try to pitch down to gain airspeed if we underspeed, constrain the pitch when underspeeding if we are
	// just passed launch
	// 中文：起飞完成前限制最小俯仰，防止 TECS 因低空速要求低头。
	const float min_pitch = (_completed_manual_takeoff) ? NAN :
				MIN_PITCH_DURING_MANUAL_TAKEOFF;

	float throttle_max = NAN;

	// enable the operator to kill the throttle on ground
	// 中文：落地且空速杆接近低端时允许飞手彻底关油门。
	if (_landed && (_manual_control_setpoint_for_airspeed < THROTTLE_THRESH)) {
		throttle_max = 0.0f;
	}

	const fixed_wing_longitudinal_setpoint_s fw_longitudinal_control_sp = {
		.timestamp = hrt_absolute_time(),
		.altitude = NAN,
		.height_rate = height_rate_sp,
		.equivalent_airspeed = get_manual_airspeed_setpoint(),
		.pitch_direct = NAN,
		.throttle_direct = NAN
	};

	_longitudinal_ctrl_sp_pub.publish(fw_longitudinal_control_sp);

	_ctrl_configuration_handler.setPitchMin(min_pitch);
	_ctrl_configuration_handler.setThrottleMax(throttle_max);

	const float roll_body = _sticks.getRoll() * radians(_param_fw_r_lim.get());
	// 将手动 roll 杆的期望滚转角转为横向加速度，由下游横纵向控制再转回姿态 setpoint。
	const DirectionalGuidanceOutput sp = {.course_setpoint = NAN, .lateral_acceleration_feedforward = rollAngleToLateralAccel(roll_body)};
	fixed_wing_lateral_setpoint_s fw_lateral_ctrl_sp{empty_lateral_control_setpoint};
	fw_lateral_ctrl_sp.timestamp = hrt_absolute_time();
	fw_lateral_ctrl_sp.course = sp.course_setpoint;
	fw_lateral_ctrl_sp.lateral_acceleration = sp.lateral_acceleration_feedforward;
	_lateral_ctrl_sp_pub.publish(fw_lateral_ctrl_sp);
}

void
FixedWingModeManager::control_manual_position(const hrt_abstime now, const float control_interval,
		const Vector2d &curr_pos,
		const Vector2f &ground_speed)
{
	// 手动位置模式：纵向同手动高度，横向在杆量居中时航向保持，杆量输入时直接滚转。
	updateManualTakeoffStatus();

	const float height_rate_sp = getManualHeightRateSetpoint();

	// TECS may try to pitch down to gain airspeed if we underspeed, constrain the pitch when underspeeding if we are
	// just passed launch
	// 中文：手动位置模式同样在起飞初期限制 TECS 俯仰下限。
	const float min_pitch = (_completed_manual_takeoff) ? NAN :
				MIN_PITCH_DURING_MANUAL_TAKEOFF;

	float throttle_max = NAN;

	// enable the operator to kill the throttle on ground
	// 中文：地面低油门杆允许关闭油门上限。
	if (_landed && (_manual_control_setpoint_for_airspeed < THROTTLE_THRESH)) {
		throttle_max = 0.0f;
	}

	if (_local_pos.xy_reset_counter != _xy_reset_counter) {
		// 记录 EKF 水平位置重置时间，航向保持会短时间重置航线参考。
		_time_last_xy_reset = now;
	}

	/* heading control */
	// TODO: either make it course hold (easier) or a real heading hold (minus all the complexity here)
	// 中文：当 roll/yaw 杆居中时启用航向保持，用 NPFG 沿一条固定航向直线飞。
	if (fabsf(_sticks.getRoll()) < HDG_HOLD_MAN_INPUT_THRESH &&
	    fabsf(_sticks.getYaw()) < HDG_HOLD_MAN_INPUT_THRESH) {

		/* heading / roll is zero, lock onto current heading */
		// 中文：偏航角速度足够小时锁定当前航向，避免飞机还在转弯时立即锁错方向。
		if (fabsf(_yawrate) < HDG_HOLD_YAWRATE_THRESH && !_yaw_lock_engaged) {
			// little yaw movement, lock to current heading
			_yaw_lock_engaged = true;
		}

		/* user tries to do a takeoff in heading hold mode, reset the yaw setpoint on every iteration
			to make sure the plane does not start rolling
		*/
		// 中文：手动起飞未完成前不真正启用航向保持，防止地面/低速阶段自动滚转。
		if (!_completed_manual_takeoff) {
			_hdg_hold_enabled = false;
			_yaw_lock_engaged = true;
		}

		if (_yaw_lock_engaged) {

			const Vector2f curr_pos_local{_local_pos.x, _local_pos.y};

			if (!_hdg_hold_enabled) {
				// just switched back from non heading-hold to heading hold
				// 中文：刚进入航向保持时，固定当前位置作为航线上的一点。
				_hdg_hold_enabled = true;
				_hdg_hold_yaw = _yaw;

				_hdg_hold_position = curr_pos_local;
			}

			// if there's a reset-by-fusion, the ekf needs some time to converge,
			// therefore we go into track holiding for 2 seconds
			// 中文：EKF 位置重置后短时间刷新航线起点，避免局部坐标跳变导致横向误差突变。
			if (now - _time_last_xy_reset < 2_s) {
				_hdg_hold_position = curr_pos_local;
			}

			const DirectionalGuidanceOutput sp = navigateLine(_hdg_hold_position, _hdg_hold_yaw, curr_pos_local, ground_speed,
							     _wind_vel);
			fixed_wing_lateral_setpoint_s fw_lateral_ctrl_sp{empty_lateral_control_setpoint};
			fw_lateral_ctrl_sp.timestamp = now;
			fw_lateral_ctrl_sp.course = sp.course_setpoint;
			fw_lateral_ctrl_sp.lateral_acceleration = sp.lateral_acceleration_feedforward;

			_lateral_ctrl_sp_pub.publish(fw_lateral_ctrl_sp);
		}
	}

	const fixed_wing_longitudinal_setpoint_s fw_longitudinal_control_sp = {
		.timestamp = hrt_absolute_time(),
		.altitude = NAN,
		.height_rate = height_rate_sp,
		.equivalent_airspeed = get_manual_airspeed_setpoint(),
		.pitch_direct = NAN,
		.throttle_direct = NAN
	};

	_longitudinal_ctrl_sp_pub.publish(fw_longitudinal_control_sp);

	_ctrl_configuration_handler.setPitchMin(min_pitch);
	_ctrl_configuration_handler.setThrottleMax(throttle_max);

	if (!_yaw_lock_engaged || fabsf(_sticks.getRoll()) >= HDG_HOLD_MAN_INPUT_THRESH ||
	    fabsf(_sticks.getYaw()) >= HDG_HOLD_MAN_INPUT_THRESH) {

		// 有明显 roll/yaw 杆量时退出航向保持，改为直接滚转控制。
		_hdg_hold_enabled = false;
		_yaw_lock_engaged = false;

		const float roll_body = _sticks.getRoll() * radians(_param_fw_r_lim.get());
		fixed_wing_lateral_setpoint_s fw_lateral_ctrl_sp{empty_lateral_control_setpoint};
		fw_lateral_ctrl_sp.timestamp = hrt_absolute_time();
		fw_lateral_ctrl_sp.lateral_acceleration = rollAngleToLateralAccel(roll_body);
		_lateral_ctrl_sp_pub.publish(fw_lateral_ctrl_sp);
	}
}

float FixedWingModeManager::rollAngleToLateralAccel(float roll_body) const
{
	// 协调转弯模型：横向加速度 ay = g * tan(phi)。
	return tanf(roll_body) * CONSTANTS_ONE_G;
}

void FixedWingModeManager::control_backtransition_heading_hold()
{
	// VTOL 回转换且无位置控制时，固定进入过渡时的航向/空速方向。
	if (!PX4_ISFINITE(_backtrans_heading)) {
		_backtrans_heading = _local_pos.heading;
	}

	fixed_wing_lateral_setpoint_s fw_lateral_ctrl_sp{empty_lateral_control_setpoint};
	fw_lateral_ctrl_sp.timestamp = hrt_absolute_time();
	fw_lateral_ctrl_sp.airspeed_direction = _backtrans_heading;
	_lateral_ctrl_sp_pub.publish(fw_lateral_ctrl_sp);
}

void FixedWingModeManager::control_backtransition_line_follow(const Vector2f &ground_speed,
		const position_setpoint_s &pos_sp_curr)
{
	// VTOL 回转换有位置控制时，从过渡开始点到目标点做直线跟踪。
	Vector2f curr_pos_local{_local_pos.x, _local_pos.y};
	Vector2f curr_wp_local = _global_local_proj_ref.project(pos_sp_curr.lat, pos_sp_curr.lon);

	// Set the position where the backtransition started the first ime we pass through here.
	// Will get reset if not in transition anymore.
	// 中文：首次进入回转换时锁定起点，退出过渡后在 Run() 中清空。
	if (!_lpos_where_backtrans_started.isAllFinite()) {
		_lpos_where_backtrans_started = curr_pos_local;
	}

	DirectionalGuidanceOutput sp = {.course_setpoint = NAN, .lateral_acceleration_feedforward = 0.f};

	if (_control_mode.flag_control_position_enabled) {
		// 位置控制可用时，横向沿回转换起点到当前 setpoint 的直线。
		sp = navigateLine(_lpos_where_backtrans_started, curr_wp_local, curr_pos_local, ground_speed, _wind_vel);
	}

	fixed_wing_lateral_setpoint_s fw_lateral_ctrl_sp{empty_lateral_control_setpoint};
	fw_lateral_ctrl_sp.timestamp = hrt_absolute_time();
	fw_lateral_ctrl_sp.course = sp.course_setpoint;
	fw_lateral_ctrl_sp.lateral_acceleration = sp.lateral_acceleration_feedforward;
	_lateral_ctrl_sp_pub.publish(fw_lateral_ctrl_sp);
}

void
FixedWingModeManager::Run()
{
	/*
	 * 主循环由 local_position 回调触发。
	 * 通信顺序大致为：
	 *   读取状态/参数/任务 setpoint -> 判定模式 -> 调用对应控制函数发布横纵向 setpoint
	 *   -> 发布纵向配置、横向制导状态、襟翼/扰流板/起落架等辅助 topic。
	 */
	if (should_exit()) {
		_local_pos_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	_vehicle_status_sub.update(&_vehicle_status);

	/* only run controller if position changed and we are not running an external mode*/
	// 中文：外部导航模式下不生成 PX4 内部固定翼 setpoint，只重置配置发布计时。

	const bool is_external_nav_state = (_vehicle_status.nav_state >= vehicle_status_s::NAVIGATION_STATE_EXTERNAL1)
					   && (_vehicle_status.nav_state <= vehicle_status_s::NAVIGATION_STATE_EXTERNAL8);

	if (is_external_nav_state) {
		// this will cause the configuration handler to publish immediately the next time an internal flight
		// mode is active
		// 中文：回到内部模式后立即重新发布默认纵向配置，避免外部模式遗留配置。
		_ctrl_configuration_handler.resetLastPublishTime();

	} else if (_local_pos_sub.update(&_local_pos)) {

		const hrt_abstime now = _local_pos.timestamp;

		const float control_interval = math::constrain((now - _last_time_position_control_called) * 1e-6f,
					       MIN_AUTO_TIMESTEP, MAX_AUTO_TIMESTEP);
		// 控制周期做上下限保护，避免调度抖动导致状态机或积分逻辑过激。
		_last_time_position_control_called = now;

		// check for parameter updates
		if (_parameter_update_sub.updated()) {
			// clear update
			parameter_update_s pupdate;
			_parameter_update_sub.copy(&pupdate);

			// update parameters from storage
			parameters_update();
		}

		vehicle_global_position_s gpos;

		if (_global_pos_sub.update(&gpos)) {
			// 保存当前经纬度，供全局 waypoint 距离/投影使用。
			_current_latitude = gpos.lat;
			_current_longitude = gpos.lon;
		}

		if (_local_pos.z_global && PX4_ISFINITE(_local_pos.ref_alt)) {
			// 使用 EKF 提供的参考高度把 local z 转成 AMSL。
			_reference_altitude = _local_pos.ref_alt;

		} else {
			_reference_altitude = 0.f;
		}

		_current_altitude = -_local_pos.z + _reference_altitude; // Altitude AMSL in meters

		// handle estimator reset events. we only adjust setpoins for manual modes
		// 中文：估计器坐标重置只调整手动位置模式的航向保持参考。
		if (_control_mode.flag_control_manual_enabled) {
			// adjust navigation waypoints in position control mode
			if (_control_mode.flag_control_altitude_enabled && _control_mode.flag_control_velocity_enabled
			    && _local_pos.xy_reset_counter != _xy_reset_counter) {

				// reset heading hold flag, which will re-initialise position control
				_hdg_hold_enabled = false;
			}
		}

		// Convert Local setpoints to global setpoints
		// 中文：维护本地 XY 和全局经纬度之间的投影参考，Offboard 本地点要靠它转换为全局航点。
		if (!_global_local_proj_ref.isInitialized()
		    || (_global_local_proj_ref.getProjectionReferenceTimestamp() != _local_pos.ref_timestamp)
		    || (_local_pos.xy_reset_counter != _xy_reset_counter)) {

			double reference_latitude = 0.;
			double reference_longitude = 0.;

			if (_local_pos.xy_global && PX4_ISFINITE(_local_pos.ref_lat) && PX4_ISFINITE(_local_pos.ref_lon)) {
				reference_latitude = _local_pos.ref_lat;
				reference_longitude = _local_pos.ref_lon;
			}

			_global_local_proj_ref.initReference(reference_latitude, reference_longitude,
							     _local_pos.ref_timestamp);
		}

		if (_control_mode.flag_control_offboard_enabled) {
			// Offboard trajectory_setpoint 转换成 position_setpoint_triplet 的 current 点。
			trajectory_setpoint_s trajectory_setpoint;

			if (_trajectory_setpoint_sub.update(&trajectory_setpoint)) {
				bool valid_setpoint = false;
				_pos_sp_triplet = {}; // clear any existing
				_pos_sp_triplet.timestamp = trajectory_setpoint.timestamp;
				_pos_sp_triplet.current.timestamp = trajectory_setpoint.timestamp;
				_pos_sp_triplet.current.cruising_speed = NAN; // ignored
				_pos_sp_triplet.current.cruising_throttle = NAN; // ignored
				_pos_sp_triplet.current.vx = NAN;
				_pos_sp_triplet.current.vy = NAN;
				_pos_sp_triplet.current.vz = NAN;
				_pos_sp_triplet.current.lat = static_cast<double>(NAN);
				_pos_sp_triplet.current.lon = static_cast<double>(NAN);
				_pos_sp_triplet.current.alt = NAN;

				if (Vector3f(trajectory_setpoint.position).isAllFinite()) {
					// 有位置 setpoint：本地 NED -> 全局 lat/lon/alt。
					if (_global_local_proj_ref.isInitialized()) {
						double lat;
						double lon;
						_global_local_proj_ref.reproject(trajectory_setpoint.position[0], trajectory_setpoint.position[1], lat, lon);
						valid_setpoint = true;
						_pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;
						_pos_sp_triplet.current.lat = lat;
						_pos_sp_triplet.current.lon = lon;
						_pos_sp_triplet.current.alt = _reference_altitude - trajectory_setpoint.position[2];
					}

				}

				if (Vector3f(trajectory_setpoint.velocity).isAllFinite()) {
					// 有速度 setpoint：记录 vx/vy/vz，用于 AUTO_PATH 或 VELOCITY 分支。
					valid_setpoint = true;
					_pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;
					_pos_sp_triplet.current.vx = trajectory_setpoint.velocity[0];
					_pos_sp_triplet.current.vy = trajectory_setpoint.velocity[1];
					_pos_sp_triplet.current.vz = trajectory_setpoint.velocity[2];

					if (Vector3f(trajectory_setpoint.acceleration).isAllFinite()) {
						// 由速度切向与水平加速度法向估计曲率半径，用 loiter_radius 字段传递。
						Vector2f velocity_sp_2d(trajectory_setpoint.velocity[0], trajectory_setpoint.velocity[1]);
						Vector2f normalized_velocity_sp_2d = velocity_sp_2d.normalized();
						Vector2f acceleration_sp_2d(trajectory_setpoint.acceleration[0], trajectory_setpoint.acceleration[1]);
						Vector2f acceleration_normal = acceleration_sp_2d - acceleration_sp_2d.dot(normalized_velocity_sp_2d) *
									       normalized_velocity_sp_2d;
						float direction = -normalized_velocity_sp_2d.cross(acceleration_normal.normalized());
						_pos_sp_triplet.current.loiter_radius = direction * velocity_sp_2d.norm() * velocity_sp_2d.norm() /
											acceleration_normal.norm();

					} else {
						_pos_sp_triplet.current.loiter_radius = NAN;
					}
				}

				_position_setpoint_current_valid = valid_setpoint;
			}

		} else {
			if (_pos_sp_triplet_sub.update(&_pos_sp_triplet)) {
				// Navigator mission setpoint：分别校验 previous/current/next 三个点是否具备完整 3D 信息。

				_position_setpoint_previous_valid = PX4_ISFINITE(_pos_sp_triplet.previous.lat)
								    && PX4_ISFINITE(_pos_sp_triplet.previous.lon)
								    && PX4_ISFINITE(_pos_sp_triplet.previous.alt);

				_position_setpoint_current_valid = PX4_ISFINITE(_pos_sp_triplet.current.lat)
								   && PX4_ISFINITE(_pos_sp_triplet.current.lon)
								   && PX4_ISFINITE(_pos_sp_triplet.current.alt);

				_position_setpoint_next_valid = PX4_ISFINITE(_pos_sp_triplet.next.lat)
								&& PX4_ISFINITE(_pos_sp_triplet.next.lon)
								&& PX4_ISFINITE(_pos_sp_triplet.next.alt);

				// reset the altitude foh (first order hold) logic
				// 中文：切换到新 triplet 后重置高度 FOH 的最小距离记忆。
				_min_current_sp_distance_xy = FLT_MAX;
			}
		}

		// 更新所有输入 topic；顺序保持 PX4 原逻辑，后面的模式函数使用这些缓存状态。
		airspeed_poll();
		manual_control_setpoint_poll();
		vehicle_attitude_poll();
		vehicle_command_poll();
		vehicle_control_mode_poll();
		wind_poll(now);

		if (_vehicle_land_detected_sub.updated()) {
			vehicle_land_detected_s vehicle_land_detected;

			if (_vehicle_land_detected_sub.update(&vehicle_land_detected)) {
				_landed = vehicle_land_detected.landed;
			}
		}

		if (!_vehicle_status.in_transition_mode) {
			// reset position of backtransition start if not in transition
			// 中文：退出 VTOL 过渡后清空回转换参考点/航向。
			_lpos_where_backtrans_started = Vector2f(NAN, NAN);
			_backtrans_heading = NAN;
		}


		Vector2d curr_pos(_current_latitude, _current_longitude);
		Vector2f ground_speed(_local_pos.vx, _local_pos.vy);

		set_control_mode_current(now);

		update_in_air_states(now);

		// restore nominal TECS parameters in case changed intermittently (e.g. in landing handling)
		// 中文：每周期先恢复默认配置，具体模式函数再覆盖需要改变的限制。

		// restore lateral-directional guidance parameters (changed in takeoff mode)
		// 中文：起飞等模式可能调整 NPFG 参数，这里恢复参数默认周期。
		_directional_guidance.setPeriod(_param_npfg_period.get());

		// by default no flaps/spoilers, is overwritten below in certain modes
		// 中文：默认不放襟翼/扰流板，起飞/降落/提前降落构型会覆盖。
		_flaps_setpoint = 0.f;
		_spoilers_setpoint = 0.f;

		// by default set speed weight to the param value, can be overwritten inside the methods below
		// 中文：TECS 速度权重默认来自参数，滑翔/降落等模式可覆盖。
		_ctrl_configuration_handler.setSpeedWeight(_param_t_spdweight.get());

		if (_control_mode_current != FW_POSCTRL_MODE_AUTO_LANDING_STRAIGHT
		    && _control_mode_current != FW_POSCTRL_MODE_AUTO_LANDING_CIRCULAR) {
			// 离开降落模式即复位降落状态。
			reset_landing_state();
		}

		if (_control_mode_current != FW_POSCTRL_MODE_AUTO_TAKEOFF
		    && _control_mode_current != FW_POSCTRL_MODE_AUTO_TAKEOFF_NO_NAV) {
			// 离开起飞模式即复位起飞状态机。
			reset_takeoff_state();
		}

		int8_t old_landing_gear_position = _new_landing_gear_position;
		_new_landing_gear_position = landing_gear_s::GEAR_KEEP; // is overwritten in Takeoff and Land

		switch (_control_mode_current) {
		// 根据当前固定翼控制模式调用对应 setpoint 生成函数。
		case FW_POSCTRL_MODE_AUTO: {
				control_auto(control_interval, curr_pos, ground_speed, _pos_sp_triplet.previous, _pos_sp_triplet.current,
					     _pos_sp_triplet.next);
				break;
			}

		case FW_POSCTRL_MODE_AUTO_ALTITUDE: {
				control_auto_fixed_bank_alt_hold();
				break;
			}

		case FW_POSCTRL_MODE_AUTO_CLIMBRATE: {
				control_auto_descend();
				break;
			}

		case FW_POSCTRL_MODE_AUTO_LANDING_STRAIGHT: {
				control_auto_landing_straight(now, control_interval, ground_speed, _pos_sp_triplet.previous,
							      _pos_sp_triplet.current);
				break;
			}

		case FW_POSCTRL_MODE_AUTO_LANDING_CIRCULAR: {
				control_auto_landing_circular(now, control_interval, ground_speed, _pos_sp_triplet.current);
				break;
			}

		case FW_POSCTRL_MODE_AUTO_PATH: {
				control_auto_path(control_interval, curr_pos, ground_speed, _pos_sp_triplet.current);
				break;
			}

		case FW_POSCTRL_MODE_AUTO_TAKEOFF: {
				control_auto_takeoff(now, control_interval, curr_pos, ground_speed, _pos_sp_triplet.current);
				break;
			}

		case FW_POSCTRL_MODE_AUTO_TAKEOFF_NO_NAV: {
				control_auto_takeoff_no_nav(_local_pos.timestamp, control_interval, _pos_sp_triplet.current.alt);
				break;
			}

		case FW_POSCTRL_MODE_MANUAL_POSITION: {
				control_manual_position(now, control_interval, curr_pos, ground_speed);
				break;
			}

		case FW_POSCTRL_MODE_MANUAL_ALTITUDE: {
				control_manual_altitude(control_interval, curr_pos, ground_speed);
				break;
			}

		case FW_POSCTRL_MODE_OTHER: {
				break;
			}

		case FW_POSCTRL_MODE_TRANSITION_TO_HOVER_LINE_FOLLOW: {
				control_backtransition_line_follow(ground_speed, _pos_sp_triplet.current);
				break;
			}

		case FW_POSCTRL_MODE_TRANSITION_TO_HOVER_HEADING_HOLD: {
				control_backtransition_heading_hold();
				break;
			}
		}

		if (_control_mode_current != FW_POSCTRL_MODE_OTHER) {
			// 把本周期累计的 pitch/throttle/climb/sink/横向加速度限制发布给下游控制器。
			_ctrl_configuration_handler.update(now);
		}

		// only publish status in full FW mode
		// 中文：只在固定翼或 VTOL 过渡中发布横向制导状态，便于日志分析 NPFG。
		if (_vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING
		    || _vehicle_status.in_transition_mode) {
			publish_lateral_guidance_status(now);

		}

		// if there's any change in landing gear setpoint publish it
		// 中文：起落架命令只有变化且非 KEEP 时才发布。
		if (_new_landing_gear_position != old_landing_gear_position
		    && _new_landing_gear_position != landing_gear_s::GEAR_KEEP) {

			landing_gear_s landing_gear = {};
			landing_gear.landing_gear = _new_landing_gear_position;
			landing_gear.timestamp = now;
			_landing_gear_pub.publish(landing_gear);
		}

		// In Manual modes flaps and spoilers are directly controlled in the Attitude controller and not published here
		// 中文：手动模式襟翼/扰流板在姿态控制器中由 RC 直接处理；自动模式才由 mode manager 发布。
		if (_control_mode.flag_control_auto_enabled
		    && _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {
			normalized_unsigned_setpoint_s flaps_setpoint;
			flaps_setpoint.normalized_setpoint = _flaps_setpoint;
			flaps_setpoint.timestamp = now;
			_flaps_setpoint_pub.publish(flaps_setpoint);

			normalized_unsigned_setpoint_s spoilers_setpoint;
			spoilers_setpoint.normalized_setpoint = _spoilers_setpoint;
			spoilers_setpoint.timestamp = now;
			_spoilers_setpoint_pub.publish(spoilers_setpoint);
		}

		_xy_reset_counter = _local_pos.xy_reset_counter;

		perf_end(_loop_perf);
	}
}

void
FixedWingModeManager::reset_takeoff_state()
{
	// 清空跑道起飞和发射检测状态，使下一次起飞从 WAITING/初始化阶段开始。
	_runway_takeoff.reset();

	_launchDetector.reset();

	_launch_detected = false;

	_takeoff_ground_alt = _current_altitude;
}

void
FixedWingModeManager::reset_landing_state()
{
	// 清空降落计时、flare 状态、触地点偏移和地形高度缓存。
	_time_started_landing = 0;

	_flare_states = FlareStates{};

	_lateral_touchdown_position_offset = 0.0f;

	_last_time_terrain_alt_was_valid = 0;

	// reset abort land, unless loitering after an abort
	// 中文：除“中止后正在盘旋”情况外，离开降落时清除中止状态。
	if ((_landing_abort_status && (_pos_sp_triplet.current.type != position_setpoint_s::SETPOINT_TYPE_LOITER)) ||
	    (_landing_abort_status && _param_fw_lnd_abort.get() == 0)) {

		updateLandingAbortStatus(position_controller_landing_status_s::NOT_ABORTED);
	}
}

float FixedWingModeManager::getMaxRollAngleNearGround(const float altitude, const float terrain_altitude) const
{
	// we want the wings level when at the wing height above ground
	// 中文：当离地高度低于机翼高度时要求平翼，随着高度升高线性放开滚转限制。
	const float height_above_ground = math::max(altitude - (terrain_altitude + _param_fw_wing_height.get()), 0.0f);

	// this is a conservative (linear) approximation of the roll angle that would cause wing tip strike
	// roll strike = arcsin( 2 * height / span )
	// d(roll strike)/d(height) = 2 / span / cos(2 * height / span)
	// d(roll strike)/d(height) (@height=0) = 2 / span
	// roll strike ~= 2 * height / span
	// 中文：用线性近似代替 arcsin，更保守且计算简单；最终还受全局最大滚转角限制。

	return  math::constrain(2.f * height_above_ground / _param_fw_wing_span.get(), 0.f,
				math::radians(_param_fw_r_lim.get()));
}


void
FixedWingModeManager::initializeAutoLanding(const hrt_abstime &now, const position_setpoint_s &pos_sp_prev,
		const float land_point_altitude, const Vector2f &local_position, const Vector2f &local_land_point)
{
	// 初始化直线降落几何：确定进近入口相对降落点的高度和水平偏移。
	if (_time_started_landing == 0) {

		float height_above_land_point;
		Vector2f local_approach_entrance;

		// set the landing approach entrance location when we have just started the landing and store it
		// NOTE: the landing approach vector is relative to the land point. ekf resets may cause a local frame
		// jump, so we reference to the land point, which is globally referenced and will update
		// 中文：进近向量用“相对降落点”的形式保存，可减小 EKF local frame 跳变影响。
		if (_position_setpoint_previous_valid) {
			// 任务降落：上一航点就是进近入口。
			height_above_land_point = pos_sp_prev.alt - land_point_altitude;
			local_approach_entrance = _global_local_proj_ref.project(pos_sp_prev.lat, pos_sp_prev.lon);

		} else {
			// no valid previous waypoint, construct one from the glide slope and direction from current
			// position to land point
			// 中文：没有上一航点时，用当前位置临时构造进近入口。

			// NOTE: this is not really a supported use case at the moment, this is just bandaiding any
			// ill-advised usage of the current implementation
			// 中文：这是兜底逻辑，不是推荐的任务设计方式。

			// TODO: proper handling of on-the-fly landing points would need to involve some more sophisticated
			// landing pattern generation and corresponding logic

			height_above_land_point = _current_altitude - land_point_altitude;
			local_approach_entrance = local_position;
		}

		_landing_approach_entrance_rel_alt = math::max(height_above_land_point, FLT_EPSILON);

		const Vector2f landing_approach_vector = local_land_point - local_approach_entrance;
		float landing_approach_distance = landing_approach_vector.norm();

		const float max_glide_slope = tanf(math::radians(_param_fw_lnd_ang.get()));
		const float glide_slope = _landing_approach_entrance_rel_alt / landing_approach_distance;

		if (glide_slope > max_glide_slope) {
			// rescale the landing distance - this will have the same effect as dropping down the approach
			// entrance altitude on the vehicle's behavior. if we reach here.. it means the navigator checks
			// didn't work, or something is using the control_auto_landing_straight() method inappropriately
			// 中文：若下滑角过陡，拉长等效进近距离，使控制器看到的下滑角不超过参数上限。
			landing_approach_distance = _landing_approach_entrance_rel_alt / max_glide_slope;
		}

		if (landing_approach_vector.norm_squared() > FLT_EPSILON) {
			_landing_approach_entrance_offset_vector = -landing_approach_vector.unit_or_zero() * landing_approach_distance;

		} else {
			// land in direction of airframe
			// 中文：进近向量退化时，使用当前机头方向构造降落进近方向。
			_landing_approach_entrance_offset_vector = Vector2f({cosf(_yaw), sinf(_yaw)}) * landing_approach_distance;
		}

		// save time at which we started landing and reset landing abort status
		// 中文：初始化完成后重置降落状态并记录开始时间。
		reset_landing_state();
		_time_started_landing = now;
	}
}

Vector2f
FixedWingModeManager::calculateTouchdownPosition(const float control_interval, const Vector2f &local_land_position)
{
	// 根据飞手 yaw 杆对触地点做横向微调；flare 后不再允许改变触地点。
	if (fabsf(_sticks.getYaw()) > MANUAL_TOUCHDOWN_NUDGE_INPUT_DEADZONE
	    && _param_fw_lnd_nudge.get() > LandingNudgingOption::kNudgingDisabled
	    && !_flare_states.flaring) {
		// laterally nudge touchdown location with yaw stick
		// positive is defined in the direction of a right hand turn starting from the approach vector direction
		// 中文：正方向定义为沿进近方向开始右转的一侧。
		const float signed_deadzone_threshold = MANUAL_TOUCHDOWN_NUDGE_INPUT_DEADZONE * math::signNoZero(
				_sticks.getYaw());
		_lateral_touchdown_position_offset += (_sticks.getYaw() - signed_deadzone_threshold) *
						      MAX_TOUCHDOWN_POSITION_NUDGE_RATE * control_interval;
		_lateral_touchdown_position_offset = math::constrain(_lateral_touchdown_position_offset, -_param_fw_lnd_td_off.get(),
						     _param_fw_lnd_td_off.get());
	}

	const Vector2f approach_unit_vector = -_landing_approach_entrance_offset_vector.unit_or_zero();
	const Vector2f approach_unit_normal_vector{-approach_unit_vector(1), approach_unit_vector(0)};

	// 把标量横向偏移投影到进近法向，得到修正后的本地触地点。
	return local_land_position + approach_unit_normal_vector * _lateral_touchdown_position_offset;
}

Vector2f
FixedWingModeManager::calculateLandingApproachVector() const
{
	// 根据 FW_LND_NUDGE 模式决定是偏移整条进近路径，还是改变进近角到新的触地点。
	Vector2f landing_approach_vector = -_landing_approach_entrance_offset_vector;
	const Vector2f approach_unit_vector = landing_approach_vector.unit_or_zero();
	const Vector2f approach_unit_normal_vector{-approach_unit_vector(1), approach_unit_vector(0)};

	if (_param_fw_lnd_nudge.get() == LandingNudgingOption::kNudgeApproachAngle) {
		// nudge the approach angle -- i.e. we adjust the approach vector to reach from the original approach
		// entrance position to the newly nudged touchdown point
		// NOTE: this lengthens the landing distance.. which will adjust the glideslope height slightly
		// 中文：改变进近角会让进近距离变长/变短，因此下滑道高度也会随之轻微变化。
		landing_approach_vector += approach_unit_normal_vector * _lateral_touchdown_position_offset;
	}

	// if _param_fw_lnd_nudge.get() == LandingNudgingOption::kNudgingDisabled, no nudging
	// 中文：禁用 nudging 时保持原始进近向量。

	// if _param_fw_lnd_nudge.get() == LandingNudgingOption::kNudgeApproachPath, the full path (including approach
	// entrance point) is nudged with the touchdown point, which does not require any additions to the approach vector
	// 中文：偏移整条进近路径时，进近入口和触地点一起平移，所以向量本身不变。

	return landing_approach_vector;
}

float
FixedWingModeManager::getLandingTerrainAltitudeEstimate(const hrt_abstime &now, const float land_point_altitude,
		const bool abort_on_terrain_measurement_timeout, const bool abort_on_terrain_timeout)
{
	// 获取降落使用的地形高度估计；无有效地形时根据参数决定是否中止降落。
	if (_param_fw_lnd_useter.get() > TerrainEstimateUseOnLanding::kDisableTerrainEstimation) {

		if (_local_pos.dist_bottom_valid) {

			// terrain_alt = 参考高度 + 当前 local 高度 - 距地高度。
			const float terrain_estimate = _local_pos.ref_alt + -_local_pos.z - _local_pos.dist_bottom;
			_last_valid_terrain_alt_estimate = terrain_estimate;
			_last_time_terrain_alt_was_valid = now;

			return terrain_estimate;
		}

		if (_last_time_terrain_alt_was_valid == 0) {
			// 降落开始后一直没有地形测距，则先用任务降落点高度兜底。

			const bool terrain_first_measurement_timed_out = (now - _time_started_landing) > TERRAIN_ALT_FIRST_MEASUREMENT_TIMEOUT;

			if (terrain_first_measurement_timed_out && abort_on_terrain_measurement_timeout) {
				// 若配置要求首次地形测量超时中止，则触发 terrain not found。
				updateLandingAbortStatus(position_controller_landing_status_s::TERRAIN_NOT_FOUND);
			}

			return land_point_altitude;
		}

		if (!_local_pos.dist_bottom_valid) {
			// 曾经有过地形高度，但当前测距失效；短时间内使用最后一次有效估计。

			const bool terrain_timed_out = (now - _last_time_terrain_alt_was_valid) > TERRAIN_ALT_TIMEOUT;

			if (terrain_timed_out && abort_on_terrain_timeout) {
				// 连续超时且配置允许时，中止降落。
				updateLandingAbortStatus(position_controller_landing_status_s::TERRAIN_TIMEOUT);
			}

			return _last_valid_terrain_alt_estimate;
		}
	}

	// 不使用地形估计时，降落参考就是任务降落点高度。
	return land_point_altitude;
}

bool FixedWingModeManager::checkLandingAbortBitMask(const uint8_t automatic_abort_criteria_bitmask,
		uint8_t landing_abort_criterion)
{
	// landing abort status contains a manual criterion at abort_status==1, need to subtract 2 to directly compare
	// to automatic criteria bits from the parameter FW_LND_ABORT
	// 中文：FW_LND_ABORT 参数只编码自动中止条件，手动中止不参与 bitmask 比较。
	if (landing_abort_criterion <= 1) {
		return false;
	}

	landing_abort_criterion -= 2;

	// 判断对应自动中止条件位是否置位。
	return ((1 << landing_abort_criterion) & automatic_abort_criteria_bitmask) == (1 << landing_abort_criterion);
}

void FixedWingModeManager::publishLocalPositionSetpoint(const position_setpoint_s &current_waypoint)
{
	// 发布本地位置 setpoint，主要用于日志、可视化和兼容其他状态显示。
	vehicle_local_position_setpoint_s local_position_setpoint{};
	local_position_setpoint.timestamp = hrt_absolute_time();

	Vector2f current_setpoint;

	current_setpoint = _closest_point_on_path;

	// XY 是 NPFG 最近路径点，Z 从 AMSL 高度转换回 NED local z。
	local_position_setpoint.x = current_setpoint(0);
	local_position_setpoint.y = current_setpoint(1);
	local_position_setpoint.z = _reference_altitude - current_waypoint.alt;
	local_position_setpoint.yaw = NAN;
	local_position_setpoint.yawspeed = NAN;
	local_position_setpoint.vx = NAN;
	local_position_setpoint.vy = NAN;
	local_position_setpoint.vz = NAN;
	local_position_setpoint.acceleration[0] = NAN;
	local_position_setpoint.acceleration[1] = NAN;
	local_position_setpoint.acceleration[2] = NAN;
	_local_pos_sp_pub.publish(local_position_setpoint);
}

void FixedWingModeManager::publishOrbitStatus(const position_setpoint_s pos_sp)
{
	// 发布普通 loiter/orbit 状态，地面站用它画盘旋圆。
	orbit_status_s orbit_status{};
	orbit_status.timestamp = hrt_absolute_time();
	float loiter_radius = pos_sp.loiter_radius * (pos_sp.loiter_direction_counter_clockwise ? -1.f : 1.f);

	if (fabsf(loiter_radius) < FLT_EPSILON) {
		// 航点半径无效时使用默认 loiter 半径，符号表示方向。
		loiter_radius = _param_nav_loiter_rad.get();
	}

	orbit_status.radius = loiter_radius;
	orbit_status.frame = 0; // MAV_FRAME::MAV_FRAME_GLOBAL
	orbit_status.x = static_cast<double>(pos_sp.lat);
	orbit_status.y = static_cast<double>(pos_sp.lon);
	orbit_status.z = pos_sp.alt;
	orbit_status.yaw_behaviour = orbit_status_s::ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TANGENT_TO_CIRCLE;
	_orbit_status_pub.publish(orbit_status);
}

DirectionalGuidanceOutput FixedWingModeManager::navigateWaypoints(const Vector2f &start_waypoint,
		const Vector2f &end_waypoint,
		const Vector2f &vehicle_pos, const Vector2f &ground_vel, const Vector2f &wind_vel)
{
	// 沿两个航点构成的线段导航，并处理线段前方/后方的退化情况。
	const Vector2f start_waypoint_to_end_waypoint = end_waypoint - start_waypoint;
	const Vector2f start_waypoint_to_vehicle = vehicle_pos - start_waypoint;
	const Vector2f end_waypoint_to_vehicle = vehicle_pos - end_waypoint;

	if (start_waypoint_to_end_waypoint.norm() < FLT_EPSILON) {
		// degenerate case: the waypoints are on top of each other, this should only happen when someone uses this
		// method incorrectly. just as a safe guard, call the singular waypoint navigation method.
		// 中文：起点终点重合时退化为单点导航。
		return navigateWaypoint(end_waypoint, vehicle_pos, ground_vel, wind_vel);
	}

	if ((start_waypoint_to_end_waypoint.dot(start_waypoint_to_vehicle) < -FLT_EPSILON)
	    && (start_waypoint_to_vehicle.norm() > _directional_guidance.switchDistance(500.0f))) {
		// we are in front of the start waypoint, fly directly to it until we are within switch distance
		// 中文：飞机还在线段起点之前且离起点较远，先飞向起点。
		return navigateWaypoint(start_waypoint, vehicle_pos, ground_vel, wind_vel);
	}

	if (start_waypoint_to_end_waypoint.dot(end_waypoint_to_vehicle) > FLT_EPSILON) {
		// we are beyond the end waypoint, fly back to it
		// NOTE: this logic ideally never gets executed, as a waypoint switch should happen before passing the
		// end waypoint. however this included here as a safety precaution if any navigator (module) switch condition
		// is missed for any reason. in the future this logic should all be handled in one place in a dedicated
		// flight mode state machine.
		// 中文：正常 Navigator 应在终点前切航点；这里作为漏切换时的保护，飞回终点。
		return navigateWaypoint(end_waypoint, vehicle_pos, ground_vel, wind_vel);
	}

	// follow the line segment between the start and end waypoints
	// 中文：在线段范围内，调用直线跟踪 NPFG 封装。
	return navigateLine(start_waypoint, end_waypoint, vehicle_pos, ground_vel, wind_vel);
}

DirectionalGuidanceOutput FixedWingModeManager::navigateWaypoint(const Vector2f &waypoint_pos,
		const Vector2f &vehicle_pos,
		const Vector2f &ground_vel, const Vector2f &wind_vel)
{
	// 单航点导航：路径切向取飞机当前位置指向航点方向，曲率为 0。
	const Vector2f vehicle_to_waypoint = waypoint_pos - vehicle_pos;

	if (vehicle_to_waypoint.norm() < FLT_EPSILON) {
		// degenerate case: the vehicle is on top of the single waypoint. (can happen). maintain the last npfg command.
		// 中文：飞机恰在航点上时没有唯一切向，返回空输出以保持上一条有效 NPFG 命令。
		return DirectionalGuidanceOutput{};
	}

	const Vector2f unit_path_tangent = vehicle_to_waypoint.normalized();
	// 单点导航的“路径最近点”就是航点本身。
	_closest_point_on_path = waypoint_pos;

	const float path_curvature = 0.f;
	DirectionalGuidanceOutput sp = _directional_guidance.guideToPath(vehicle_pos, ground_vel, wind_vel, unit_path_tangent,
				       _closest_point_on_path, path_curvature);

	return sp;
}

DirectionalGuidanceOutput FixedWingModeManager::navigateLine(const Vector2f &point_on_line_1,
		const Vector2f &point_on_line_2,
		const Vector2f &vehicle_pos, const Vector2f &ground_vel, const Vector2f &wind_vel)
{
	// 两点直线导航：根据线段方向构造单位切向，并计算飞机在直线上的投影点。
	const Vector2f line_segment = point_on_line_2 - point_on_line_1;

	if (line_segment.norm() <= FLT_EPSILON) {
		// degenerate case: line segment has zero length. maintain the last npfg command.
		// 中文：线段长度为 0 时没有有效航线，保持上一 NPFG 命令。
		return DirectionalGuidanceOutput{};
	}

	const Vector2f unit_path_tangent = line_segment.normalized();

	const Vector2f point_1_to_vehicle = vehicle_pos - point_on_line_1;
	// 最近路径点为当前位置在线上的正交投影。
	_closest_point_on_path = point_on_line_1 + point_1_to_vehicle.dot(unit_path_tangent) * unit_path_tangent;

	const float path_curvature = 0.f;
	const DirectionalGuidanceOutput sp = _directional_guidance.guideToPath(vehicle_pos, ground_vel, wind_vel,
					     unit_path_tangent,
					     _closest_point_on_path, path_curvature);

	return sp;
}

DirectionalGuidanceOutput FixedWingModeManager::navigateLine(const Vector2f &point_on_line,
		const float line_bearing,
		const Vector2f &vehicle_pos, const Vector2f &ground_vel, const Vector2f &wind_vel)
{
	// 点 + 方位直线导航：用于航向保持、起飞航向线等没有终点的直线路径。
	const Vector2f unit_path_tangent{cosf(line_bearing), sinf(line_bearing)};

	const Vector2f point_on_line_to_vehicle = vehicle_pos - point_on_line;
	// 用点和单位切向计算最近投影点。
	_closest_point_on_path = point_on_line + point_on_line_to_vehicle.dot(unit_path_tangent) * unit_path_tangent;

	const float path_curvature = 0.f;
	const DirectionalGuidanceOutput sp = _directional_guidance.guideToPath(vehicle_pos, ground_vel, wind_vel,
					     unit_path_tangent,
					     _closest_point_on_path, path_curvature);

	return sp;
}

DirectionalGuidanceOutput FixedWingModeManager::navigateLoiter(const Vector2f &loiter_center,
		const Vector2f &vehicle_pos,
		float radius, bool loiter_direction_counter_clockwise, const Vector2f &ground_vel, const Vector2f &wind_vel)
{
	// 圆轨迹导航：构造圆上最近点、该点切向和曲率，然后交给 NPFG。
	const float loiter_direction_multiplier = loiter_direction_counter_clockwise ? -1.f : 1.f;

	Vector2f vector_center_to_vehicle = vehicle_pos - loiter_center;
	const float dist_to_center = vector_center_to_vehicle.norm();

	// find the direction from the circle center to the closest point on its perimeter
	// from the vehicle position
	// 中文：先确定圆心指向圆上最近点的径向单位向量。
	Vector2f unit_vec_center_to_closest_pt;

	if (dist_to_center < 0.1f) {
		// the logic breaks down at the circle center, employ some mitigation strategies
		// until we exit this region
		// 中文：飞机几乎在圆心时径向方向不确定，用地速方向或默认北向兜底。
		if (ground_vel.norm() < 0.1f) {
			// arbitrarily set the point in the northern top of the circle
			// 中文：地速也太小时任选北向圆周点。
			unit_vec_center_to_closest_pt = Vector2f{1.0f, 0.0f};

		} else {
			// set the point in the direction we are moving
			// 中文：有地速时，用运动方向决定圆周最近方向。
			unit_vec_center_to_closest_pt = ground_vel.normalized();
		}

	} else {
		// set the point in the direction of the aircraft
		// 中文：正常情况用圆心到飞机方向作为最近圆周点径向。
		unit_vec_center_to_closest_pt = vector_center_to_vehicle.normalized();
	}

	// 90 deg clockwise rotation * loiter direction
	// 中文：径向向量旋转 90 度得到切向，再乘方向符号决定顺/逆时针。
	const Vector2f unit_path_tangent = loiter_direction_multiplier * Vector2f{-unit_vec_center_to_closest_pt(1), unit_vec_center_to_closest_pt(0)};

	// 圆曲率为 1/r，方向符号表示左/右转。
	const float path_curvature = loiter_direction_multiplier / radius;
	_closest_point_on_path = unit_vec_center_to_closest_pt * radius + loiter_center;
	return _directional_guidance.guideToPath(vehicle_pos, ground_vel, wind_vel, unit_path_tangent,
			loiter_center + unit_vec_center_to_closest_pt * radius, path_curvature);
}

DirectionalGuidanceOutput FixedWingModeManager::navigatePathTangent(const matrix::Vector2f &vehicle_pos,
		const matrix::Vector2f &position_setpoint,
		const matrix::Vector2f &tangent_setpoint,
		const matrix::Vector2f &ground_vel, const matrix::Vector2f &wind_vel, const float &curvature)
{
	// Offboard 路径切向导航：外部直接给路径点、切向方向和曲率。
	if (tangent_setpoint.norm() <= FLT_EPSILON) {
		// degenerate case: no direction. maintain the last npfg command.
		// 中文：切向向量为 0 时没有有效路径方向，保持上一 NPFG 输出。
		return DirectionalGuidanceOutput{};
	}

	const Vector2f unit_path_tangent{tangent_setpoint.normalized()};
	// 最近路径点由 Offboard 直接指定。
	_closest_point_on_path = position_setpoint;
	return _directional_guidance.guideToPath(vehicle_pos, ground_vel, wind_vel, tangent_setpoint.normalized(),
			position_setpoint,
			curvature);
}

DirectionalGuidanceOutput FixedWingModeManager::navigateBearing(const matrix::Vector2f &vehicle_pos, float bearing,
		const Vector2f &ground_vel, const Vector2f &wind_vel)
{
	// 纯方位导航：当前位置即路径点，bearing 定义一条穿过当前位置的期望航迹线。
	const Vector2f unit_path_tangent = Vector2f{cosf(bearing), sinf(bearing)};
	_closest_point_on_path = vehicle_pos;
	return _directional_guidance.guideToPath(vehicle_pos, ground_vel, wind_vel, unit_path_tangent, vehicle_pos, 0.0f);
}

void FixedWingModeManager::publish_lateral_guidance_status(const hrt_abstime now)
{
	// 发布 NPFG 内部状态，便于观察航迹误差、可行性、周期自适应和风估计有效性。
	fixed_wing_lateral_guidance_status_s fixed_wing_lateral_guidance_status{};

	fixed_wing_lateral_guidance_status.timestamp = now;
	fixed_wing_lateral_guidance_status.course_setpoint = _directional_guidance.getCourseSetpoint();
	fixed_wing_lateral_guidance_status.lateral_acceleration_ff = _directional_guidance.getLateralAccelerationSetpoint();
	fixed_wing_lateral_guidance_status.bearing_feas = _directional_guidance.getBearingFeasibility();
	fixed_wing_lateral_guidance_status.bearing_feas_on_track = _directional_guidance.getBearingFeasibilityOnTrack();
	fixed_wing_lateral_guidance_status.signed_track_error = _directional_guidance.getSignedTrackError();
	fixed_wing_lateral_guidance_status.track_error_bound = _directional_guidance.getTrackErrorBound();
	fixed_wing_lateral_guidance_status.adapted_period = _directional_guidance.getAdaptedPeriod();
	fixed_wing_lateral_guidance_status.wind_est_valid = _wind_valid;

	_fixed_wing_lateral_guidance_status_pub.publish(fixed_wing_lateral_guidance_status);
}

int FixedWingModeManager::task_spawn(int argc, char *argv[])
{
	// 模块启动入口，创建 WorkItem 实例并注册回调。
	FixedWingModeManager *instance = new FixedWingModeManager();

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

int FixedWingModeManager::custom_command(int argc, char *argv[])
{
	// 当前无自定义命令，统一打印 usage。
	return print_usage("unknown command");
}

int FixedWingModeManager::print_usage(const char *reason)
{
	// 输出 PX4 shell 使用说明。
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements the setpoint generation for all PX4-internal fixed-wing modes, height-rate control and higher.
It takes the current mode state of the vehicle as input and outputs setpoints consumed by the fixed-wing
lateral-longitudinal controller and and controllers below that (attitude, rate).

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("fw_mode_manager", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int fw_mode_manager_main(int argc, char *argv[])
{
	// C ABI 入口，PX4 模块加载器通过它进入 C++ ModuleBase。
	return FixedWingModeManager::main(argc, argv);
}
