/****************************************************************************
 *
 *   Copyright (c) 2013-2023 PX4 Development Team. All rights reserved.
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

#include "FixedwingRateControl.hpp"

/*
 * 固定翼角速度控制模块。
 *
 * 通信链路位置：
 *   vehicle_rates_setpoint -> vehicle_torque_setpoint
 *   vehicle_rates_setpoint -> vehicle_thrust_setpoint
 *
 * 本模块使用三轴 PID + 前馈闭合机体系角速度环。为了精确复现 PX4，还需要保留
 * 固定翼特有逻辑：基于空速的舵面效率缩放、配平调度、来自 control_allocator
 * 饱和反馈的抗积分饱和、手动直通模式、Acro 角速度设定值生成，以及滚转到偏航的前馈。
 *
 * 本模块输出仍然是抽象的归一化力矩/推力；实际副翼、升降舵、方向舵和油门分配
 * 由后续 control_allocator 完成。
 */

using namespace time_literals;
using namespace matrix;

using math::constrain;
using math::interpolate;
using math::radians;

/*
 * 构造函数。
 *
 * vtol=false 时发布普通固定翼 torque/thrust 话题；
 * vtol=true 时发布 VTOL 固定翼虚拟 torque/thrust 话题，供 VTOL 控制器混合。
 */
FixedwingRateControl::FixedwingRateControl(bool vtol) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_actuator_controls_status_pub(vtol ? ORB_ID(actuator_controls_status_1) : ORB_ID(actuator_controls_status_0)),
	_vehicle_thrust_setpoint_pub(vtol ? ORB_ID(vehicle_thrust_setpoint_virtual_fw) : ORB_ID(vehicle_thrust_setpoint)),
	_vehicle_torque_setpoint_pub(vtol ? ORB_ID(vehicle_torque_setpoint_virtual_fw) : ORB_ID(vehicle_torque_setpoint)),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	// VT_FW_DIFTHR_EN 不是本模块 DEFINE_PARAMETERS 中的固定参数，因此手动查找句柄。
	_handle_param_vt_fw_difthr_en = param_find("VT_FW_DIFTHR_EN");

	/* fetch initial parameter values */
	// 初始化 PID、积分限幅、差动推力配置等参数。
	parameters_update();

	// 提前 advertise rate_ctrl_status，便于日志/调参工具读取积分状态。
	_rate_ctrl_status_pub.advertise();
}

/*
 * 析构函数：释放性能计数器。
 */
FixedwingRateControl::~FixedwingRateControl()
{
	perf_free(_loop_perf);
}

bool
FixedwingRateControl::init()
{
	// 注册 vehicle_angular_velocity 回调，角速度更新是 rate controller 的主触发源。
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

int
FixedwingRateControl::parameters_update()
{
	/*
	 * 将 PID 增益和积分限幅加载到通用 RateControl 辅助类。
	 * PX4 角速度控制接口中的轴顺序固定为 [roll, pitch, yaw]。
	 */
	const Vector3f rate_p = Vector3f(_param_fw_rr_p.get(), _param_fw_pr_p.get(), _param_fw_yr_p.get());
	const Vector3f rate_i = Vector3f(_param_fw_rr_i.get(), _param_fw_pr_i.get(), _param_fw_yr_i.get());
	const Vector3f rate_d = Vector3f(_param_fw_rr_d.get(), _param_fw_pr_d.get(), _param_fw_yr_d.get());

	_rate_control.setPidGains(rate_p, rate_i, rate_d);

	// 三轴积分上限分别对应 FW_RR_IMAX、FW_PR_IMAX、FW_YR_IMAX。
	_rate_control.setIntegratorLimit(
		Vector3f(_param_fw_rr_imax.get(), _param_fw_pr_imax.get(), _param_fw_yr_imax.get()));

	if (_handle_param_vt_fw_difthr_en != PARAM_INVALID) {
		// VTOL 固定翼阶段可选择用差动推力辅助某些轴，此处读取 bitmask。
		param_get(_handle_param_vt_fw_difthr_en, &_param_vt_fw_difthr_en);
	}


	return PX4_OK;
}

void
FixedwingRateControl::vehicle_manual_poll()
{
	/*
	 * 手动输入处理。
	 *
	 * 手动 + 固定翼/转换阶段时，该函数有两种职责：
	 * 1. Acro/rate 模式：由杆量生成 vehicle_rates_setpoint；
	 * 2. 纯手动直通：绕过 PID，直接生成 torque/thrust setpoint。
	 */
	if (_vcontrol_mode.flag_control_manual_enabled && _in_fw_or_transition_wo_tailsitter_transition) {

		// Always copy the new manual setpoint, even if it wasn't updated, to fill the actuators with valid values
		// 即使没有新消息，也 copy 最近一次杆量，保证控制输出不因消息频率降低而清空。
		if (_manual_control_setpoint_sub.copy(&_manual_control_setpoint)) {

			if (_vcontrol_mode.flag_control_rates_enabled &&
			    !_vcontrol_mode.flag_control_attitude_enabled) {

				/*
				 * Acro/角速度模式：
				 * 遥控杆直接生成机体系角速度设定值。PX4 中这里会先在模块内部发布
				 * vehicle_rates_setpoint，再由同一模块后续控制路径消费它。
				 */

				if (_vehicle_status.is_vtol_tailsitter && _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {
					// the rate_sp must always be published in body (hover) frame
					// 尾座式的公开接口保持悬停机体系：固定翼滚转杆和偏航杆需要交换/变号。
					_rates_sp.roll = _manual_control_setpoint.yaw * radians(_param_fw_acro_z_max.get());
					_rates_sp.yaw = -_manual_control_setpoint.roll * radians(_param_fw_acro_x_max.get());

				} else {
					// 普通固定翼：roll/yaw 杆量直接乘以 Acro 最大角速度得到 rate setpoint。
					_rates_sp.roll = _manual_control_setpoint.roll * radians(_param_fw_acro_x_max.get());
					_rates_sp.yaw = _manual_control_setpoint.yaw * radians(_param_fw_acro_z_max.get());
				}

				_rates_sp.timestamp = hrt_absolute_time();
				// 俯仰杆取负号符合 PX4 手动输入约定；油门从 [-1, 1] 映射到 [0, 1]。
				_rates_sp.pitch = -_manual_control_setpoint.pitch * radians(_param_fw_acro_y_max.get());
				_rates_sp.thrust_body[0] = (_manual_control_setpoint.throttle + 1.f) * .5f;

				// 这里发布的 rate setpoint 会被本模块后续 rate-control 分支再次读取。
				_rate_sp_pub.publish(_rates_sp);

			} else {
				/*
				 * 手动直通模式：
				 * 绕过角速度 PID，直接把杆量映射为归一化力矩命令。这里仍然叠加
				 * 按空速缩放后的配平，使直通路径和闭环模式具有相同的舵面中立偏置。
				 */
				const float airspeed_scaling_sq = _airspeed_scaling * _airspeed_scaling;

				// 三轴杆量分别映射为归一化 torque，叠加按空速缩放后的舵面 trim。
				_vehicle_torque_setpoint.xyz[0] = math::constrain(_manual_control_setpoint.roll * _param_fw_man_r_sc.get() +
								  _param_trim_roll.get() * airspeed_scaling_sq, -1.f, 1.f);
				_vehicle_torque_setpoint.xyz[1] = math::constrain(-_manual_control_setpoint.pitch * _param_fw_man_p_sc.get() +
								  _param_trim_pitch.get() * airspeed_scaling_sq, -1.f, 1.f);
				_vehicle_torque_setpoint.xyz[2] = math::constrain(_manual_control_setpoint.yaw * _param_fw_man_y_sc.get() +
								  _param_trim_yaw.get() * airspeed_scaling_sq, -1.f, 1.f);

				// 手动油门同样从 [-1, 1] 映射到 [0, 1]。
				_vehicle_thrust_setpoint.xyz[0] = math::constrain((_manual_control_setpoint.throttle + 1.f) * .5f, 0.f, 1.f);
			}
		}
	}
}

void
FixedwingRateControl::vehicle_land_detected_poll()
{
	// 更新落地状态。落地时 rate PID 积分会被复位，防止地面误差造成积分残留。
	if (_vehicle_land_detected_sub.updated()) {
		vehicle_land_detected_s vehicle_land_detected {};

		if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)) {
			_landed = vehicle_land_detected.landed;
		}
	}
}

float FixedwingRateControl::get_airspeed_and_update_scaling()
{
	/*
	 * 气动舵面的控制效能随动压变化。FW_ARSP_SCALE_EN 使能时，PX4 使用
	 * (FW_AIRSPD_TRIM / constrained_airspeed)^2 对 PID 输出做补偿。
	 */
	_airspeed_validated_sub.update();
	const bool airspeed_valid = PX4_ISFINITE(_airspeed_validated_sub.get().calibrated_airspeed_m_s)
				    && (hrt_elapsed_time(&_airspeed_validated_sub.get().timestamp) < 1_s);

	// if no airspeed measurement is available out best guess is to use the trim airspeed
	// 没有可靠空速时使用巡航配平空速，作为舵效缩放和配平调度的默认基准。
	float airspeed = _param_fw_airspd_trim.get();

	if (_param_fw_use_airspd.get() && airspeed_valid) {
		/* prevent numerical drama by requiring 0.5 m/s minimal speed */
		// 空速至少取 0.5 m/s，避免后续除法过大或不稳定。
		airspeed = math::max(0.5f, _airspeed_validated_sub.get().calibrated_airspeed_m_s);

	} else {
		// VTOL: if we have no airspeed available and we are in hover mode then assume the lowest airspeed possible
		// this assumption is good as long as the vehicle is not hovering in a headwind which is much larger
		// than the stall airspeed
		// VTOL 悬停时没有固定翼空速测量意义，使用失速空速作为保守舵效估计。
		if (_vehicle_status.is_vtol && _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
		    && !_vehicle_status.in_transition_mode) {
			airspeed = _param_fw_airspd_stall.get();
		}
	}

	/*
	 * For scaling our actuators using anything less than the stall
	 * speed doesn't make any sense - its the strongest reasonable deflection we
	 * want to do in flight and it's the baseline a human pilot would choose.
	 *
	 * Forcing the scaling to this value allows reasonable handheld tests.
	 */


	if (_param_fw_arsp_scale_en.get()) {
		// 缩放时不允许使用低于失速空速的值，否则会放大控制量到不合理程度。
		const float min_airspeed = math::max(_param_fw_airspd_stall.get(), 0.1f);
		const float airspeed_constrained = math::max(airspeed, min_airspeed);
		// 该比例后面会平方使用，对应舵面力矩约随空速平方变化的近似。
		_airspeed_scaling = _param_fw_airspd_trim.get() / airspeed_constrained;

	} else {
		// 禁用空速缩放时，所有舵面控制量按原始 PID 输出使用。
		_airspeed_scaling = 1.0f;
	}

	return airspeed;
}

void FixedwingRateControl::Run()
{
	/*
	 * 主工作队列回调。它随 angular velocity 更新运行，计算当前空速缩放，
	 * 处理手动/Acro 设定值生成；如果角速度控制使能，则运行通用三轴 rate PID。
	 */
	if (should_exit()) {
		_vehicle_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	// only run controller if angular velocity changed
	if (_vehicle_angular_velocity_sub.updated() || (hrt_elapsed_time(&_last_run) > 20_ms)) { //TODO rate!

		// only update parameters if they changed
		// 参数更新频率远低于控制频率，通过订阅通知按需刷新，减少运行时开销。
		bool params_updated = _parameter_update_sub.updated();

		// check for parameter updates
		if (params_updated) {
			// clear update
			parameter_update_s pupdate;
			_parameter_update_sub.copy(&pupdate);

			// update parameters from storage
			updateParams();
			parameters_update();
		}

		float dt = 0.f;

		static constexpr float DT_MIN = 0.002f;
		static constexpr float DT_MAX = 0.04f;

		vehicle_angular_velocity_s vehicle_angular_velocity{};

		if (_vehicle_angular_velocity_sub.copy(&vehicle_angular_velocity)) {
			// 用角速度采样时间戳计算 dt，使 PID 积分/微分项与传感器采样对齐。
			dt = math::constrain((vehicle_angular_velocity.timestamp_sample - _last_run) * 1e-6f, DT_MIN, DT_MAX);
			_last_run = vehicle_angular_velocity.timestamp_sample;
		}

		if (dt < DT_MIN || dt > DT_MAX) {
			// 时间戳异常时退回系统时间估算，并继续使用 dt 限幅保护。
			const hrt_abstime time_now_us = hrt_absolute_time();
			dt = math::constrain((time_now_us - _last_run) * 1e-6f, DT_MIN, DT_MAX);
			_last_run = time_now_us;
		}

		vehicle_angular_velocity_s angular_velocity{};
		_vehicle_angular_velocity_sub.copy(&angular_velocity);

		Vector3f rates(angular_velocity.xyz);
		Vector3f angular_accel{angular_velocity.xyz_derivative};

		// Tailsitter: rotate setpoint from hover to fixed-wing frame (controller is in fixed-wing frame, interface in hover)
		// 尾座式公开接口为悬停机体系，而固定翼 rate controller 在固定翼机体系中计算。
		if (_vehicle_status.is_vtol_tailsitter) {
			rates = Vector3f(-angular_velocity.xyz[2], angular_velocity.xyz[1], angular_velocity.xyz[0]);
			angular_accel = Vector3f(-angular_velocity.xyz_derivative[2], angular_velocity.xyz_derivative[1],
						 angular_velocity.xyz_derivative[0]);
		}

		// vehicle status update must be before the vehicle_control_mode poll, otherwise rate sp are not published during whole transition
		// 先更新 vehicle_status，确保 VTOL 转换状态影响本周期是否运行固定翼控制。
		_vehicle_status_sub.update(&_vehicle_status);
		const bool is_in_transition_except_tailsitter = _vehicle_status.in_transition_mode
				&& !_vehicle_status.is_vtol_tailsitter;
		const bool is_fixed_wing = _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING;
		_in_fw_or_transition_wo_tailsitter_transition =  is_fixed_wing || is_in_transition_except_tailsitter;

		_vehicle_control_mode_sub.update(&_vcontrol_mode);

		// 先更新一次落地状态，供手动处理和积分复位判断使用。
		vehicle_land_detected_poll();

		// 手动模式可能在这里生成 rate setpoint 或直接生成 torque/thrust setpoint。
		vehicle_manual_poll();

		// 手动处理后再更新一次，保持与 PX4 原逻辑一致，避免落地状态滞后。
		vehicle_land_detected_poll();

		/* if we are in rotary wing mode, do nothing */
		// 纯多旋翼模式不运行固定翼 rate controller；VTOL 由 VTOL 控制器接管。
		if (_vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING && !_vehicle_status.is_vtol) {
			perf_end(_loop_perf);
			return;
		}

		if (_vcontrol_mode.flag_control_rates_enabled) {

			const float airspeed = get_airspeed_and_update_scaling();

			/* reset integrals where needed */
			if (_rates_sp.reset_integral) {
				// 上游姿态控制器显式要求复位积分，通常发生在落地或非固定翼阶段。
				_rate_control.resetIntegral();
			}

			// Reset integrators if the aircraft is on ground or not in a state where the fw attitude controller is run
			if (_landed || !_in_fw_or_transition_wo_tailsitter_transition) {

				// 双重保护：只要落地或当前不是固定翼控制阶段，就不允许积分继续保留。
				_rate_control.resetIntegral();
			}

			// Update saturation status from control allocation feedback
			// TODO: send the unallocated value directly for better anti-windup
			Vector3<bool> diffthr_enabled(
				_param_vt_fw_difthr_en & static_cast<int32_t>(VTOLFixedWingDifferentialThrustEnabledBit::ROLL_BIT),
				_param_vt_fw_difthr_en & static_cast<int32_t>(VTOLFixedWingDifferentialThrustEnabledBit::PITCH_BIT),
				_param_vt_fw_difthr_en & static_cast<int32_t>(VTOLFixedWingDifferentialThrustEnabledBit::YAW_BIT)
			);

			if (_vehicle_status.is_vtol_tailsitter) {
				// Swap roll and yaw
				// 尾座式固定翼/悬停坐标轴映射会交换 roll/yaw，对应饱和标志也要交换。
				diffthr_enabled.swapRows(0, 2);
			}

			// saturation handling for axis controlled by differential thrust (VTOL only)
			control_allocator_status_s control_allocator_status;

			// Set saturation flags for VTOL differential thrust feature
			// If differential thrust is enabled in an axis, assume it's the only torque authority and only update saturation using matrix 0 allocating the motors.
			if (_control_allocator_status_subs[0].update(&control_allocator_status)) {
				for (size_t i = 0; i < 3; i++) {
					if (diffthr_enabled(i)) {
						// 差动推力控制的轴使用电机矩阵的未分配力矩判断是否已经饱和。
						_rate_control.setPositiveSaturationFlag(i, control_allocator_status.unallocated_torque[i] > FLT_EPSILON);
						_rate_control.setNegativeSaturationFlag(i, control_allocator_status.unallocated_torque[i] < -FLT_EPSILON);
					}
				}
			}

			// Set saturation flags for control surface controlled axes
			if (_control_allocator_status_subs[_vehicle_status.is_vtol ? 1 : 0].update(&control_allocator_status)) {
				for (size_t i = 0; i < 3; i++) {
					if (!diffthr_enabled(i)) {
						// 舵面控制的轴使用舵面矩阵的未分配力矩判断饱和，用于积分抗 windup。
						_rate_control.setPositiveSaturationFlag(i, control_allocator_status.unallocated_torque[i] > FLT_EPSILON);
						_rate_control.setNegativeSaturationFlag(i, control_allocator_status.unallocated_torque[i] < -FLT_EPSILON);
					}
				}
			}

			/* bi-linear interpolation over airspeed for actuator trim scheduling */
			Vector3f trim(_param_trim_roll.get(), _param_trim_pitch.get(), _param_trim_yaw.get());
			trim *= _airspeed_scaling * _airspeed_scaling;
			/*
			 * 配平调度：不同空速下舵面零位可能不同。PX4 在
			 * 最小空速/巡航空速、巡航空速/最大空速之间做线性插值，
			 * 用来补偿随动压变化而变化的滚转、俯仰、偏航配平量。
			 */

			if (airspeed < _param_fw_airspd_trim.get()) {
				// 低于巡航空速：从最小空速增量线性插值到巡航空速增量 0。
				trim(0) += interpolate(airspeed, _param_fw_airspd_min.get(), _param_fw_airspd_trim.get(),
						       _param_fw_dtrim_r_vmin.get(),
						       0.0f);
				trim(1) += interpolate(airspeed, _param_fw_airspd_min.get(), _param_fw_airspd_trim.get(),
						       _param_fw_dtrim_p_vmin.get(),
						       0.0f);
				trim(2) += interpolate(airspeed, _param_fw_airspd_min.get(), _param_fw_airspd_trim.get(),
						       _param_fw_dtrim_y_vmin.get(),
						       0.0f);

			} else {
				// 高于巡航空速：从巡航空速增量 0 线性插值到最大空速增量。
				trim(0) += interpolate(airspeed, _param_fw_airspd_trim.get(), _param_fw_airspd_max.get(), 0.0f,
						       _param_fw_dtrim_r_vmax.get());
				trim(1) += interpolate(airspeed, _param_fw_airspd_trim.get(), _param_fw_airspd_max.get(), 0.0f,
						       _param_fw_dtrim_p_vmax.get());
				trim(2) += interpolate(airspeed, _param_fw_airspd_trim.get(), _param_fw_airspd_max.get(), 0.0f,
						       _param_fw_dtrim_y_vmax.get());
			}

			if (_vcontrol_mode.flag_control_rates_enabled) {
				// 读取最新 vehicle_rates_setpoint；自动/姿态模式来自 fw_att_control，Acro 模式可能来自本模块。
				_rates_sp_sub.update(&_rates_sp);

				Vector3f body_rates_setpoint = Vector3f(_rates_sp.roll, _rates_sp.pitch, _rates_sp.yaw);

				// Tailsitter: rotate setpoint from hover to fixed-wing frame (controller is in fixed-wing frame, interface in hover)
				if (_vehicle_status.is_vtol_tailsitter) {
					body_rates_setpoint = Vector3f(-_rates_sp.yaw, _rates_sp.pitch, _rates_sp.roll);
				}

				const Vector3f gain_ff(_param_fw_rr_ff.get(), _param_fw_pr_ff.get(), _param_fw_yr_ff.get());
				// FF 增益先除以 airspeed_scaling，后面 control_u 再乘 scaling^2，保持 PX4 原始补偿形式。
				const Vector3f scaled_gain_ff = gain_ff / _airspeed_scaling;
				_rate_control.setFeedForwardGain(scaled_gain_ff);

				// Run attitude RATE controllers which need the desired attitudes from above, add trim.
				const Vector3f angular_acceleration_setpoint = _rate_control.update(rates, body_rates_setpoint, angular_accel, dt,
						_landed);

				/*
				 * RateControl 输出的是尚未做固定翼空速修正的归一化控制量。
				 * 固定翼舵面效率随空速变化，因此这里先乘以空速缩放的平方，
				 * 再叠加配平，最后限制到 [-1, 1] 的归一化力矩需求。
				 */
				Vector3f control_u = angular_acceleration_setpoint * _airspeed_scaling * _airspeed_scaling;

				// Special case yaw in Acro: if the parameter FW_ACRO_YAW_EN is not set then don't rate-control yaw
				if (!_vcontrol_mode.flag_control_attitude_enabled && _vcontrol_mode.flag_control_manual_enabled
				    && !_param_fw_acro_yaw_en.get()) {
					// Acro 默认不闭环偏航角速度，避免方向舵控制器和固定翼自然协调转弯相互打架。
					control_u(2) = _manual_control_setpoint.yaw * _param_fw_man_y_sc.get();
					_rate_control.resetIntegral(2);
				}

				if (control_u.isAllFinite()) {
					// 正常情况：PID 输出 + trim 后限幅，形成归一化三轴 torque setpoint。
					matrix::constrain(control_u + trim, -1.f, 1.f).copyTo(_vehicle_torque_setpoint.xyz);

				} else {
					// 安全保护：控制量非法时清积分，仅输出 trim，避免 NaN 进入分配器。
					_rate_control.resetIntegral();
					trim.copyTo(_vehicle_torque_setpoint.xyz);
				}

				/* throttle passed through if it is finite */
				// 固定翼 rate controller 不调节油门，只把上游 thrust_body[0] 转发为推力设定。
				_vehicle_thrust_setpoint.xyz[0] = PX4_ISFINITE(_rates_sp.thrust_body[0]) ? _rates_sp.thrust_body[0] : 0.0f;

				/* scale effort by battery status */
				// 可选电池缩放：低电压时按 battery.scale 放大油门，补偿电压下降带来的推力降低。
				if (_param_fw_bat_scale_en.get() && _vehicle_thrust_setpoint.xyz[0] > 0.1f) {

					if (_battery_status_sub.updated()) {
						battery_status_s battery_status{};

						if (_battery_status_sub.copy(&battery_status) && battery_status.connected && battery_status.scale > 0.f) {
							_battery_scale = battery_status.scale;
						}
					}

					_vehicle_thrust_setpoint.xyz[0] *= _battery_scale;
				}
			}

			// publish rate controller status
			// 发布积分状态，供日志、调参和自整定模块观察内环积分行为。
			rate_ctrl_status_s rate_ctrl_status{};
			_rate_control.getRateControlStatus(rate_ctrl_status);
			rate_ctrl_status.timestamp = hrt_absolute_time();

			_rate_ctrl_status_pub.publish(rate_ctrl_status);

		} else {
			// full manual
			// 纯手动直通不运行 PID，积分必须清零。
			_rate_control.resetIntegral();
		}

		// Add feed-forward from roll control output to yaw control output
		// This can be used to counteract the adverse yaw effect when rolling the plane
		/*
		 * 不利偏航前馈：固定翼滚转时可能产生反向偏航趋势，因此在发布
		 * torque 前把一部分滚转控制量混入偏航控制量，用方向舵抵消耦合。
		 * 这是固定翼特有的耦合补偿，不放在通用 RateControl 类中。
		 */
		_vehicle_torque_setpoint.xyz[2] = math::constrain(_vehicle_torque_setpoint.xyz[2] + _param_fw_rll_to_yaw_ff.get() *
						  _vehicle_torque_setpoint.xyz[0], -1.f, 1.f);

		// Tailsitter: rotate back to body frame from airspeed frame
		// 尾座式在发布 torque 前要从固定翼控制坐标系转回公开的悬停机体系接口。
		if (_vehicle_status.is_vtol_tailsitter) {
			const float helper = _vehicle_torque_setpoint.xyz[0];
			_vehicle_torque_setpoint.xyz[0] = _vehicle_torque_setpoint.xyz[2];
			_vehicle_torque_setpoint.xyz[2] = -helper;
		}

		/* Only publish if any of the proper modes are enabled */
		// 只有控制模式实际需要固定翼输出时才发布，避免向分配器发送无效控制量。
		if (_vcontrol_mode.flag_control_rates_enabled ||
		    _vcontrol_mode.flag_control_attitude_enabled ||
		    _vcontrol_mode.flag_control_manual_enabled) {
			{
				_vehicle_thrust_setpoint.timestamp = hrt_absolute_time();
				_vehicle_thrust_setpoint.timestamp_sample = angular_velocity.timestamp_sample;
				_vehicle_thrust_setpoint_pub.publish(_vehicle_thrust_setpoint);

				_vehicle_torque_setpoint.timestamp = hrt_absolute_time();
				_vehicle_torque_setpoint.timestamp_sample = angular_velocity.timestamp_sample;
				_vehicle_torque_setpoint_pub.publish(_vehicle_torque_setpoint);
			}
		}

		updateActuatorControlsStatus(dt);

		// Manual flaps/spoilers control, also active in VTOL Hover. Is handled and published in FW Position controller/VTOL module if Auto.
		// 手动模式下襟翼/扰流板由本模块根据遥控输入发布；自动模式由模式管理/VTOL 模块处理。
		if (_vcontrol_mode.flag_control_manual_enabled) {

			// Flaps control
			float flaps_control = 0.f; // default to no flaps

			/* map flaps by default to manual if valid */
			// manual_control_setpoint.flaps 有效时直接作为襟翼归一化指令，负值不参与。
			if (PX4_ISFINITE(_manual_control_setpoint.flaps)) {
				flaps_control = math::max(_manual_control_setpoint.flaps, 0.f); // do not consider negative switch settings
			}

			normalized_unsigned_setpoint_s flaps_setpoint;
			flaps_setpoint.timestamp = hrt_absolute_time();
			flaps_setpoint.normalized_setpoint = flaps_control;
			_flaps_setpoint_pub.publish(flaps_setpoint);

			// Spoilers control
			float spoilers_control = 0.f; // default to no spoilers

			switch (_param_fw_spoilers_man.get()) {
			case 0:
				// 0：手动扰流板禁用，保持 0。
				break;

			case 1:
				// do not consider negative switch settings
				// 1：复用 flaps 通道控制扰流板。
				spoilers_control = PX4_ISFINITE(_manual_control_setpoint.flaps) ? math::max(_manual_control_setpoint.flaps, 0.f) : 0.f;
				break;

			case 2:
				// do not consider negative switch settings
				// 2：使用 aux1 通道控制扰流板。
				spoilers_control = PX4_ISFINITE(_manual_control_setpoint.aux1) ? math::max(_manual_control_setpoint.aux1, 0.f) : 0.f;
				break;
			}

			normalized_unsigned_setpoint_s spoilers_setpoint;
			spoilers_setpoint.timestamp = hrt_absolute_time();
			spoilers_setpoint.normalized_setpoint = spoilers_control;
			_spoilers_setpoint_pub.publish(spoilers_setpoint);
		}
	}

	// backup schedule
	// 兜底调度：即使角速度回调暂时未触发，也保证约 20ms 后再次运行。
	ScheduleDelayed(20_ms);

	perf_end(_loop_perf);
}

void FixedwingRateControl::updateActuatorControlsStatus(float dt)
{
	/*
	 * 统计控制量变化强度，用于 actuator_controls_status。
	 * 固定翼舵面消耗主要来自舵面移动而非保持位置，所以这里统计 torque setpoint
	 * 的变化量平方，而不是控制量本身平方。
	 */
	for (int i = 0; i < 3; i++) {

		// We assume that the attitude is actuated by control surfaces
		// consuming power only when they move
		const float control_signal = _vehicle_torque_setpoint.xyz[i] - _control_prev[i];
		_control_prev[i] = _vehicle_torque_setpoint.xyz[i];

		_control_energy[i] += control_signal * control_signal * dt;
	}

	_energy_integration_time += dt;

	if (_energy_integration_time > 500e-3f) {
		// 每 0.5s 发布一次平均控制功率，降低日志频率并提供平滑指标。

		actuator_controls_status_s status;
		status.timestamp = _vehicle_torque_setpoint.timestamp;

		for (int i = 0; i < 3; i++) {
			status.control_power[i] = _control_energy[i] / _energy_integration_time;
			_control_energy[i] = 0.f;
		}

		_actuator_controls_status_pub.publish(status);
		_energy_integration_time = 0.f;
	}
}

int FixedwingRateControl::task_spawn(int argc, char *argv[])
{
	/*
	 * 模块启动入口。
	 * 命令行参数带 "vtol" 时使用 VTOL 固定翼虚拟 torque/thrust 话题。
	 */
	bool vtol = false;

	if (argc > 1) {
		if (strcmp(argv[1], "vtol") == 0) {
			vtol = true;
		}
	}

	FixedwingRateControl *instance = new FixedwingRateControl(vtol);

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

int FixedwingRateControl::custom_command(int argc, char *argv[])
{
	// 本模块没有自定义运行时命令，未知命令统一打印用法。
	return print_usage("unknown command");
}

int FixedwingRateControl::print_usage(const char *reason)
{
	// px4 命令行帮助信息，供 nsh/mavlink shell 查看模块用途和启动参数。
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
fw_rate_control is the fixed-wing rate controller.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("fw_rate_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int fw_rate_control_main(int argc, char *argv[])
{
	return FixedwingRateControl::main(argc, argv);
}
