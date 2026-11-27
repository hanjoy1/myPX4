/****************************************************************************
 *
 *   Copyright (c) 2020-2022 PX4 Development Team. All rights reserved.
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
 * @file fw_autotune_attitude_control.cpp
 *
 * @author Mathieu Bresciani <mathieu@auterion.com>
 */

#include "fw_autotune_attitude_control.hpp"

/*
 * 固定翼姿态自整定模块。
 *
 * 通信链路位置：
 *   观测内环响应 + 注入激励 -> 更新控制器增益
 *
 * 该模块运行在正常固定翼控制链路旁边，而不是替代内环控制。它发布
 * autotune_attitude_control_status，让 fw_att_control 叠加辨识用角速度激励；
 * 随后估计飞机响应，并把优化后的角速度/姿态控制增益写回参数。
 */

using namespace matrix;

FwAutotuneAttitudeControl::FwAutotuneAttitudeControl(bool is_vtol) :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::hp_default),
	_vehicle_torque_setpoint_sub(this, ORB_ID(vehicle_torque_setpoint), is_vtol ? 1 : 0),
	_actuator_controls_status_sub(is_vtol ? ORB_ID(actuator_controls_status_1) : ORB_ID(actuator_controls_status_0))
{
	// 根据是否 VTOL 选择固定翼控制组实例，普通固定翼使用 instance 0，VTOL 固定翼使用 instance 1。
	_autotune_attitude_control_status_pub.advertise();
	reset();
}

FwAutotuneAttitudeControl::~FwAutotuneAttitudeControl()
{
	perf_free(_cycle_perf);
}

bool FwAutotuneAttitudeControl::init()
{
	// 参数更新触发模块运行，便于通过 FW_AT_START 或 AUX 开关启动/停止自整定。
	if (!_parameter_update_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	_signal_filter.setParameters(_publishing_dt_s, .2f); // runs in the slow publishing loop
	// 中文：信号滤波器工作在较慢的发布循环，用于生成“增量 rate_sp”，避免阶跃过尖。

	if (!_vehicle_torque_setpoint_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void FwAutotuneAttitudeControl::reset()
{
	// 将启动参数复位，防止模块重启后自动继续上一次自整定请求。
	_param_fw_at_start.reset();
}

void FwAutotuneAttitudeControl::Run()
{
	/*
	 * 自整定主循环：
	 * 读取 torque setpoint 与角速度响应 -> 更新系统辨识模型 -> 计算候选 PID/FF 增益
	 * -> 发布 autotune_attitude_control_status，其中 rate_sp 是要叠加到姿态控制器的激励信号。
	 */
	if (should_exit()) {
		_parameter_update_sub.unregisterCallback();
		_vehicle_torque_setpoint_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	// check for parameter updates
	// 中文：参数变化可能启动/停止自整定，也可能改变整定轴、信号类型和应用方式。
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();
		updateStateMachine(hrt_absolute_time());
	}

	if (_vehicle_status_sub.updated()) {
		// 缓存解锁状态和导航模式，用于状态机安全检查。
		vehicle_status_s vehicle_status;

		if (_vehicle_status_sub.copy(&vehicle_status)) {
			_armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
			_nav_state = vehicle_status.nav_state;
		}
	}

	_aux_switch_en = isAuxEnableSwitchEnabled();

	// new control data needed every iteration
	// 中文：idle 且 AUX 未启用时不运行；非 idle 时每次必须有新的 torque setpoint 才更新辨识。
	if ((_state == state::idle && !_aux_switch_en)
	    || !_vehicle_torque_setpoint_sub.updated()) {

		return;
	}

	if (_actuator_controls_status_sub.updated()) {
		// 控制功率用于 test 阶段判断新参数是否引起明显高频/大幅控制输出。
		actuator_controls_status_s controls_status;

		if (_actuator_controls_status_sub.copy(&controls_status)) {
			_control_power = Vector3f(controls_status.control_power);
		}
	}


	if (_vehicle_angular_velocity_sub.updated()) {
		// 角速度是系统辨识的输出 y。
		vehicle_angular_velocity_s vehicle_angular_velocity;

		if (_vehicle_angular_velocity_sub.copy(&vehicle_angular_velocity)) {
			_angular_velocity = Vector3f(vehicle_angular_velocity.xyz);
		}
	}

	vehicle_torque_setpoint_s vehicle_torque_setpoint;

	if (!_vehicle_torque_setpoint_sub.copy(&vehicle_torque_setpoint)) {
		// torque setpoint 是系统辨识输入 u；没有输入就不更新。
		return;
	}

	perf_begin(_cycle_perf);

	const hrt_abstime timestamp_sample = vehicle_torque_setpoint.timestamp;

	// collect sample interval average for filters
	// 中文：统计平均采样间隔，用于配置系统辨识滤波器和离散模型时间步。
	if (_last_run > 0) {
		// Guard against too small (< 0.125ms) and too large (> 20ms) dt's.
		// 中文：对 dt 限幅，避免调度异常污染平均采样周期。
		const float dt = math::constrain(((timestamp_sample - _last_run) * 1e-6f), 0.000125f, 0.02f);
		_interval_sum += dt;
		_interval_count++;

	} else {
		_interval_sum = 0.f;
		_interval_count = 0.f;
	}

	_last_run = timestamp_sample;

	checkFilters();

	// 根据当前整定轴选择 torque setpoint 分量作为输入、对应角速度作为输出。
	if (_state == state::roll || _state == state::roll_amp_detection) {
		_sys_id.update(_input_scale * vehicle_torque_setpoint.xyz[0],
			       _angular_velocity(0));

	} else if (_state == state::pitch || _state == state::pitch_amp_detection) {
		_sys_id.update(_input_scale * vehicle_torque_setpoint.xyz[1],
			       _angular_velocity(1));

	} else if (_state == state::yaw || _state == state::yaw_amp_detection) {
		_sys_id.update(_input_scale * vehicle_torque_setpoint.xyz[2],
			       _angular_velocity(2));
	}

	if (hrt_elapsed_time(&_last_publish) > _publishing_dt_hrt || _last_publish == 0) {
		// 慢速发布循环：更新状态机、计算候选增益、发布激励 rate_sp 和调试状态。
		const hrt_abstime now = hrt_absolute_time();
		updateStateMachine(now);

		Vector<float, 5> coeff = _sys_id.getCoefficients();
		// 输入曾按 _input_scale 缩放，计算实际模型增益时需要乘回来。
		coeff(2) *= _input_scale;
		coeff(3) *= _input_scale;
		coeff(4) *= _input_scale;

		const Vector3f num(coeff(2), coeff(3), coeff(4));
		const Vector3f den(1.f, coeff(0), coeff(1));
		_kiff(2) = (1.f + coeff(0) + coeff(1)) / (coeff(2) + coeff(3) + coeff(4)); // inverse of the static gain
		// 中文：KFF 取静态增益倒数，使前馈能直接抵消一阶稳态误差。
		const Vector3f num_design = num * _kiff(2); // PID algorithm design works better with systems having unit static gain
		// 中文：GMVC PID 设计在单位静态增益模型上更稳定，因此先用 KFF 归一化分子。
		Vector3f kid = pid_design::computePidGmvc(num_design, den, _sample_interval_avg, 0.2f, 0.f, 0.4f);
		_kiff(0) = kid(0);
		_kiff(1) = kid(1);

		// To compute the attitude gain, use the following empirical rule:
		// "An error of 60 degrees should produce the maximum control output"
		// or K_att * (K_rate + K_ff) * rad(60) = 1
		// 中文：姿态 P 增益用经验规则反推，并限制在 1..5。
		_attitude_p = math::constrain(1.f / (math::radians(60.f) * (_kiff(0) + _kiff(2))), 1.f, 5.f);

		const Vector<float, 5> &coeff_var = _sys_id.getVariances();
		Vector3f rate_sp{};

		if (_sys_id.areFiltersInitialized()) {
			// 滤波器准备好后，按当前状态输出幅值检测信号或系统辨识信号。
			if (_state == state::roll_amp_detection || _state == state::pitch_amp_detection || _state == state::yaw_amp_detection) {
				rate_sp = getAmplitudeDetectionSignal();

			} else {
				rate_sp = getIdentificationSignal();
			}
		}

		autotune_attitude_control_status_s status{};
		// 发布完整自整定状态：模型系数、方差、fitness、候选增益和激励 rate_sp。
		status.timestamp = now;
		coeff.copyTo(status.coeff);
		coeff_var.copyTo(status.coeff_var);
		status.fitness = _sys_id.getFitness();
		status.dt_model = _sample_interval_avg;
		status.innov = _sys_id.getInnovation();
		status.u_filt = _sys_id.getFilteredInputData();
		status.y_filt = _sys_id.getFilteredOutputData();
		status.kc = _kiff(0);
		status.ki = _kiff(1);
		status.kd = kid(2); // FW rate controller has no derivative gain
		status.kff = _kiff(2);
		status.att_p = _attitude_p;
		rate_sp.copyTo(status.rate_sp);
		status.state = static_cast<int>(_state);
		_autotune_attitude_control_status_pub.publish(status);

		_last_publish = now;
	}

	perf_end(_cycle_perf);
}

void FwAutotuneAttitudeControl::checkFilters()
{
	// 根据累计采样周期判断滤波器是否需要初始化或重置。
	if (_interval_count > 1000) {
		// calculate sensor update rate
		// 中文：使用 1000 个样本估计平均控制周期，降低调度抖动影响。
		_sample_interval_avg = _interval_sum / _interval_count;
		const float update_rate_hz = 1.f / _sample_interval_avg;

		// check if sample rate error is greater than 1%
		// 中文：采样率变化超过 1% 时重配滤波器。
		bool reset_filters = false;

		if ((fabsf(update_rate_hz - _filter_sample_rate) / _filter_sample_rate) > 0.01f) {
			reset_filters = true;
		}

		if (reset_filters || !_are_filters_initialized) {
			// 配置系统辨识滤波器：低通跟随 gyro cutoff，高通去除低频偏置，并设置遗忘因子。
			_are_filters_initialized = true;
			_filter_sample_rate = update_rate_hz;
			_sys_id.setLpfCutoffFrequency(_filter_sample_rate, _param_imu_gyro_cutoff.get());
			_sys_id.setHpfCutoffFrequency(_filter_sample_rate, .5f);
			_sys_id.setForgettingFactor(60.f, _sample_interval_avg);
			_sys_id.setFitnessLpfTimeConstant(1.f, _sample_interval_avg);
		}

		// reset sample interval accumulator
		// 中文：清空统计窗口，下一轮继续估计采样周期。
		_last_run = 0;
	}
}

bool FwAutotuneAttitudeControl::isAuxEnableSwitchEnabled()
{
	// 根据 FW_AT_MAN_AUX 选择手动 AUX 通道，大于 0.5 表示启动自整定。
	manual_control_setpoint_s manual_control_setpoint{};
	_manual_control_setpoint_sub.copy(&manual_control_setpoint);

	float aux_enable_channel = 0;

	switch (_param_fw_at_man_aux.get()) {
	case 0:
		// 0 表示不使用 AUX 开关。
		return false;

	case 1:
		aux_enable_channel = manual_control_setpoint.aux1;
		break;

	case 2:
		aux_enable_channel = manual_control_setpoint.aux2;
		break;

	case 3:
		aux_enable_channel = manual_control_setpoint.aux3;
		break;

	case 4:
		aux_enable_channel = manual_control_setpoint.aux4;
		break;

	case 5:
		aux_enable_channel = manual_control_setpoint.aux5;
		break;

	case 6:
		aux_enable_channel = manual_control_setpoint.aux6;
		break;

	default:
		// 参数超出支持范围时视为未启用。
		return false;
	}

	return aux_enable_channel > .5f;
}

void FwAutotuneAttitudeControl::updateStateMachine(hrt_abstime now)
{
	// when identifying an axis, check if the estimate has converged
	// 中文：fitness 小于阈值并持续至少 5 秒后，认为该轴辨识已收敛。
	const float converged_thr = 1.f;

	const float temp[5] = {0.f, 0.f, 0.f, 0.f, 0.f};
	const Vector<float, 5> sys_id_init(temp);

	switch (_state) {
	case state::idle:
		// 空闲状态：等待 FW_AT_START 参数或 AUX 开关触发。
		if (_param_fw_at_start.get() || _aux_switch_en) {

			mavlink_log_info(&_mavlink_log_pub, "Autotune started");
			_state = state::init;
			_state_start_time = now;
			// 记录启动时飞行模式，若整定中模式变化则安全中止。
			_start_flight_mode = _nav_state;
		}

		break;

	case state::init:
		// 初始化状态：等待滤波器/采样率准备完成，然后从 roll 幅值检测开始。
		if (_are_filters_initialized) {
			_state = state::roll_amp_detection;
			_amplitude_detection_state = amplitudeDetectionState::init;
			_state_start_time = now;
			_sys_id.reset(sys_id_init);
			// first step needs to be shorter to keep the drone centered
			// 中文：阶跃信号第一段更短，避免飞机偏离太多。
			_steps_counter = 5;
			_max_steps = 10;
			_signal_sign = 1;
			_input_scale = 1.f / _param_fw_rr_p.get();
			// 输入按当前 P 增益倒数缩放，让不同轴的辨识幅值更接近。
			_signal_filter.reset(0.f);
			_gains_backup_available = false;
		}

		break;

	case state::roll_amp_detection: {
			if (!(_param_fw_at_axes.get() & Axes::roll)) {
				// Should not tune this axis, skip
				// 中文：参数未选择 roll 轴时，直接跳过到 roll_pause。
				_state = state::roll_pause;
				break;
			}

			const float abs_roll_rate = fabsf(_angular_velocity(0));
			const float target = min(kTargetRollRate, math::radians(_param_fw_r_rmax.get()));

			updateAmplitudeDetectionState(now, abs_roll_rate, target);

			if (_amplitude_detection_state == amplitudeDetectionState::complete) {

				// 幅值确定后进入 roll 系统辨识。
				_state = state::roll;
				_state_start_time = now;
			}

			break;
		}

	case state::roll:

		// roll 辨识收敛后保存候选增益，并暂停 2 秒让飞机稳定。
		if ((_sys_id.getFitness() < converged_thr)
		    && ((now - _state_start_time) > 5_s)) {
			copyGains(0);

			// wait for the drone to stabilize
			_state = state::roll_pause;
			_state_start_time = now;
		}

		break;

	case state::roll_pause:
		// roll 和 pitch 之间的稳定等待；结束后初始化 pitch 辨识。
		if ((now - _state_start_time) > 2_s) {
			_state = state::pitch_amp_detection;
			_amplitude_detection_state = amplitudeDetectionState::init;
			_state_start_time = now;
			_sys_id.reset(sys_id_init);
			_input_scale = 1.f / _param_fw_pr_p.get();
			_signal_sign = 1;
			// first step needs to be shorter to keep the drone centered
			// 中文：新轴重新从短阶跃开始。
			_steps_counter = 5;
			_max_steps = 10;
			_signal_filter.reset(0.f);
		}

		break;

	case state::pitch_amp_detection: {
			if (!(_param_fw_at_axes.get() & Axes::pitch)) {
				// Should not tune this axis, skip
				// 中文：未选择 pitch 轴则跳过到 pitch_pause。
				_state = state::pitch_pause;
				break;
			}

			const float abs_pitch_rate = fabsf(_angular_velocity(1));
			const float max_pitch_rate = min(_param_fw_p_rmax_pos.get(), _param_fw_p_rmax_neg.get());
			const float target = min(kTargetPitchRate, math::radians(max_pitch_rate));

			updateAmplitudeDetectionState(now, abs_pitch_rate, target);

			if (_amplitude_detection_state == amplitudeDetectionState::complete) {

				// 幅值确定后进入 pitch 系统辨识。
				_state = state::pitch;
				_state_start_time = now;
			}

			break;
		}

	case state::pitch:

		// pitch 辨识收敛后保存候选增益。
		if ((_sys_id.getFitness() < converged_thr)
		    && ((now - _state_start_time) > 5_s)) {
			copyGains(1);
			_state = state::pitch_pause;
			_state_start_time = now;
		}

		break;

	case state::pitch_pause:
		// pitch 和 yaw 之间的稳定等待；结束后初始化 yaw 辨识。
		if ((now - _state_start_time) > 2_s) {
			_state = state::yaw_amp_detection;
			_amplitude_detection_state = amplitudeDetectionState::init;
			_state_start_time = now;
			_sys_id.reset(sys_id_init);
			_input_scale = 1.f / _param_fw_yr_p.get();
			_signal_sign = 1;
			// first step needs to be shorter to keep the drone centered
			// 中文：yaw 辨识也从短阶跃开始。
			_steps_counter = 5;
			_max_steps = 10;
			_signal_filter.reset(0.f);
		}

		break;

	case state::yaw_amp_detection: {
			if (!(_param_fw_at_axes.get() & Axes::yaw)) {
				// Should not tune this axis, skip
				// 中文：未选择 yaw 轴则跳过到 yaw_pause。
				_state = state::yaw_pause;
				break;
			}

			const float abs_yaw_rate = fabsf(_angular_velocity(2));
			const float target = min(kTargetYawRate, math::radians(_param_fw_y_rmax.get()));

			updateAmplitudeDetectionState(now, abs_yaw_rate, target);

			if (_amplitude_detection_state == amplitudeDetectionState::complete) {

				// 幅值确定后进入 yaw 系统辨识。
				_state = state::yaw;
				_state_start_time = now;
			}

			break;
		}

	case state::yaw:
		// yaw 辨识收敛后保存候选增益，并进入最终验证。
		if ((_sys_id.getFitness() < converged_thr)
		    && ((now - _state_start_time) > 5_s)) {
			copyGains(2);
			_state = state::yaw_pause;
			_state_start_time = now;
		}

		break;

	case state::yaw_pause:
		// 所有轴结束后清掉信号滤波器，进入增益合理性验证。
		_signal_filter.reset(0.f);
		_state = state::verification;

	// fallthrough
	// 中文：yaw_pause 有意直接落入 verification。
	case state::verification:
		// 验证候选增益正值且不过大；通过则应用，否则失败。
		_state = areGainsGood()
			 ? state::apply
			 : state::fail;

		_state_start_time = now;
		break;

	case state::apply: {
			// 根据 FW_AT_APPLY 决定立即测试、等待落锁保存，或只报告完成不写参数。
			mavlink_log_info(&_mavlink_log_pub, "Autotune finished successfully");

			if ((_param_fw_at_apply.get() == 1)) {
				// 方式 1：等待 disarm 后保存，降低飞行中写参风险。
				_state = state::wait_for_disarm;

			} else if (_param_fw_at_apply.get() == 2) {
				// 方式 2：飞行中临时写入新参数并测试，失败则回退。
				backupAndSaveGainsToParams();
				_state = state::test;

			} else {
				_state = state::complete;
			}

			_state_start_time = now;
		}

		break;

	case state::wait_for_disarm:
		// 等待飞机上锁后把候选增益写入参数。
		if (!_armed) {
			saveGainsToParams();
			_state = state::complete;
			_state_start_time = now;
		}

		break;

	case state::test:
		// 新增益飞行测试阶段：若控制功率明显增大，认为参数不安全并回退。
		if ((now - _state_start_time) > 4_s) {
			_state = state::complete;
			_state_start_time = now;

		} else if ((now - _state_start_time) < 4_s
			   && (now - _state_start_time) > 1_s
			   && _control_power.longerThan(0.1f)) {
			_state = state::fail;
			revertParamGains();
			_state_start_time = now;
		}

		break;

	case state::complete:

	// fallthrough
	case state::fail:

		// Wait a bit in that state to make sure
		// the other components are aware of the final result
		// 中文：保持 complete/fail 至少 2 秒，让日志和其他模块看到最终状态。
		if ((now - _state_start_time) > 2_s) {

			// Don't reset until aux switch is back to disabled
			// 中文：若由 AUX 开关启动，则必须等开关关闭后才回 idle，避免立刻再次启动。
			if (_param_fw_at_man_aux.get() && _aux_switch_en) {
				break;
			}

			orb_advert_t mavlink_log_pub = nullptr;
			mavlink_log_info(&mavlink_log_pub, "Autotune returned to idle");
			_state = state::idle;
			_param_fw_at_start.set(false);
			_param_fw_at_start.commit();
		}

		break;
	}

	// In case of convergence timeout
	// the identification sequence is aborted immediately
	// 中文：任一辨识阶段超时、AUX 关闭或飞行模式变化，都立即中止进入 fail。
	if (_state != state::wait_for_disarm && _state != state::idle && _state != state::fail && _state != state::complete) {
		if (now - _state_start_time > 20_s
		    || (_param_fw_at_man_aux.get() && !_aux_switch_en)
		    || _start_flight_mode != _nav_state) {
			orb_advert_t mavlink_log_pub = nullptr;
			mavlink_log_critical(&mavlink_log_pub, "Autotune aborted before finishing");
			_state = state::fail;
			_state_start_time = now;
		}
	}
}

void FwAutotuneAttitudeControl::copyGains(int index)
{
	// 将当前轴计算出的 K/P/I/FF 和姿态 P 保存到候选增益数组。
	if (index <= 2) {
		_rate_k(index) = _kiff(0);
		_rate_i(index) = _kiff(1);
		_rate_ff(index) = _kiff(2);
		_att_p(index) = _attitude_p;
	}
}

bool FwAutotuneAttitudeControl::areGainsGood() const
{
	// 对候选增益做基本安全检查：必须为正，且不能明显过大。
	bool are_positive = true;
	bool are_small_enough = true;

	for (int i = 0; i < 3; i++) {
		// 只检查用户选择参与自整定的轴。
		if (((i == 0) && (_param_fw_at_axes.get() & Axes::roll))
		    || ((i == 1) && (_param_fw_at_axes.get() & Axes::pitch))
		    || ((i == 2) && (_param_fw_at_axes.get() & Axes::yaw))) {
			are_positive &= _rate_k(i) > 0.f
					&& _rate_i(i) > 0.f
					&& _rate_ff(i) > 0.f;

			are_small_enough &= _rate_k(i) < 4.0f
					    && _rate_i(i) < 10.f
					    && _rate_ff(i) < 2.f;

			if (i < 3) {
				// There is no yaw attitude controller
				// 中文：固定翼 yaw 没有姿态外环，但这里保持原逻辑检查数组范围内的 att_p。
				are_positive &=	_att_p(i) > 0.f;
				are_small_enough &= _att_p(i) < 12.f;
			}
		}
	}

	return are_positive && are_small_enough;
}

void FwAutotuneAttitudeControl::backupAndSaveGainsToParams()
{
	// 飞行中测试新参数前，先备份旧参数，再写入新参数。
	float backup_gains[11] = {};
	backup_gains[0] = _param_fw_rr_p.get();
	backup_gains[1] = _param_fw_rr_i.get();
	backup_gains[2] = _param_fw_rr_ff.get();
	backup_gains[3] = _param_fw_r_tc.get();
	backup_gains[4] = _param_fw_pr_p.get();
	backup_gains[5] = _param_fw_pr_i.get();
	backup_gains[6] = _param_fw_pr_ff.get();
	backup_gains[7] = _param_fw_p_tc.get();
	backup_gains[8] = _param_fw_yr_p.get();
	backup_gains[9] = _param_fw_yr_i.get();
	backup_gains[10] = _param_fw_yr_ff.get();

	saveGainsToParams();

	// 写入后把内存中的候选数组恢复为旧参数，便于 test 失败时调用 saveGainsToParams() 回退。
	_rate_k(0) = backup_gains[0];
	_rate_i(0) = backup_gains[1] / _rate_k(0);
	_rate_ff(0) = backup_gains[2];
	_att_p(0) = 1.f / backup_gains[3];
	_rate_k(1) = backup_gains[4];
	_rate_i(1) = backup_gains[5] / _rate_k(1);
	_rate_ff(1) = backup_gains[6];
	_att_p(1) = 1.f / backup_gains[7];
	_rate_k(2) = backup_gains[8];
	_rate_i(2) = backup_gains[9] / _rate_k(2);
	_rate_ff(2) = backup_gains[10];

	_gains_backup_available = true;
}

void FwAutotuneAttitudeControl::revertParamGains()
{
	// 若有备份，则把旧增益重新写回参数。
	if (_gains_backup_available) {
		saveGainsToParams();
	}
}

void FwAutotuneAttitudeControl::saveGainsToParams()
{
	// 将候选增益写入 PX4 参数。I 参数以 K * Ki 的形式存储，姿态时间常数为 1 / att_p。
	if (_param_fw_at_axes.get() & Axes::roll) {
		_param_fw_rr_p.set(_rate_k(0));
		_param_fw_rr_i.set(_rate_k(0) * _rate_i(0));
		_param_fw_rr_ff.set(_rate_ff(0));
		_param_fw_r_tc.set(1.f / _att_p(0));
		_param_fw_rr_p.commit_no_notification();
		_param_fw_rr_i.commit_no_notification();
		_param_fw_rr_ff.commit_no_notification();

		if (_param_fw_at_axes.get() == Axes::roll) {
			// 最后一次 commit 触发参数更新通知。
			_param_fw_r_tc.commit();

		} else {
			_param_fw_r_tc.commit_no_notification();
		}
	}

	if (_param_fw_at_axes.get() & Axes::pitch) {
		// pitch 轴保存角速度 PID/FF 和姿态时间常数。
		_param_fw_pr_p.set(_rate_k(1));
		_param_fw_pr_i.set(_rate_k(1) * _rate_i(1));
		_param_fw_pr_ff.set(_rate_ff(1));
		_param_fw_p_tc.set(1.f / _att_p(1));
		_param_fw_pr_p.commit_no_notification();
		_param_fw_pr_i.commit_no_notification();
		_param_fw_pr_ff.commit_no_notification();

		if (!(_param_fw_at_axes.get() & Axes::yaw)) {
			_param_fw_p_tc.commit();

		} else {
			_param_fw_p_tc.commit_no_notification();
		}
	}

	if (_param_fw_at_axes.get() & Axes::yaw) {
		// yaw 轴只保存角速度环参数，没有姿态外环时间常数。
		_param_fw_yr_p.set(_rate_k(2));
		_param_fw_yr_i.set(_rate_k(2) * _rate_i(2));
		_param_fw_yr_ff.set(_rate_ff(2));
		_param_fw_yr_p.commit_no_notification();
		_param_fw_yr_i.commit_no_notification();
		_param_fw_yr_ff.commit();
	}
}

void FwAutotuneAttitudeControl::updateAmplitudeDetectionState(const hrt_abstime now,
		const float rate, const float target_rate)
{
	// 自动寻找合适激励幅值：逐步增大正弦幅值，直到连续两个周期都能达到目标角速度。

	const hrt_abstime time_since_last_amplitude_increase = now - _time_last_amplitude_increase;

	switch (_amplitude_detection_state) {
	case amplitudeDetectionState::init:
		_signal_amp = 0.1f;  // start with an initial amplitude of 0.1 rad/s
		// 中文：从 0.1 rad/s 初始幅值开始，避免一开始就给过大激励。
		_rate_reached = false;
		_time_last_amplitude_increase = _state_start_time;
		_amplitude_detection_state = amplitudeDetectionState::first_period;

		break;


	case amplitudeDetectionState::first_period:

		// 第一个 1 秒窗口内观察角速度是否达到目标。
		if (!_rate_reached && rate >= target_rate) {
			_rate_reached = true;
		}

		if (time_since_last_amplitude_increase >= 1_s) {
			// 第一个窗口达到目标则进入第二窗口确认，否则增大幅值。
			_amplitude_detection_state = _rate_reached ?
						     amplitudeDetectionState::second_period :
						     amplitudeDetectionState::increase_amplitude;

			// reset flag
			_rate_reached = false;
		}

		break;

	case amplitudeDetectionState::second_period:
		// 第二个 1 秒窗口再次确认，降低偶然峰值导致幅值过小的风险。
		if (!_rate_reached && rate >= target_rate) {
			_rate_reached = true;
		}

		if (time_since_last_amplitude_increase >= 2_s) {
			// 两个窗口都达到目标后接受当前幅值；否则继续增大。
			_amplitude_detection_state = _rate_reached ?
						     amplitudeDetectionState::set_amplitude :
						     amplitudeDetectionState::increase_amplitude;
			// reset flag
			_rate_reached = false;
		}

		break;


	case amplitudeDetectionState::increase_amplitude:

		// 按固定步长增加激励幅值，超过最大值则使用最大允许幅值。
		_signal_amp += kSignalAmpStep;
		_time_last_amplitude_increase = now;

		_amplitude_detection_state = (_signal_amp > kSignalAmpMax) ?
					     amplitudeDetectionState::set_amplitude :
					     amplitudeDetectionState::first_period;


		break;

	case amplitudeDetectionState::set_amplitude:

		// 回退一步作为最终幅值，避免刚达到阈值的幅值偏激进。
		_signal_amp = math::constrain(_signal_amp - kSignalAmpStep, 0.1f, kSignalAmpMax);
		_amplitude_detection_state = amplitudeDetectionState::complete;

		break;

	case amplitudeDetectionState::complete:
		break;
	}
}

const Vector3f FwAutotuneAttitudeControl::getAmplitudeDetectionSignal()
{
	// 幅值检测阶段使用 1 Hz 正弦信号，输出到当前待整定轴的 rate_sp。
	const hrt_abstime now = hrt_absolute_time();
	const float t = static_cast<float>(now - _state_start_time) * 1e-6f;

	float signal = sinf(2.0f * M_PI_F * t);

	signal *= _signal_amp;
	// scaleInputSignal 会根据当前状态把标量信号放到 roll/pitch/yaw 对应分量。
	Vector3f rate_sp = scaleInputSignal(signal);

	return rate_sp;
}

const Vector3f FwAutotuneAttitudeControl::getIdentificationSignal()
{
	// 系统辨识阶段生成用户选择的激励信号：阶跃、线性扫频或对数扫频。
	const hrt_abstime now = hrt_absolute_time();
	const float t = static_cast<float>(now - _state_start_time) * 1e-6f;
	float signal = 0.0f;

	switch (_param_fw_sysid_signal_type.get()) {
	case  static_cast<int32_t>(SignalType::kStep): {
			// 变长阶跃序列：逐渐缩短持续步数，丰富频率内容。
			if (_steps_counter > _max_steps) {
				_signal_sign = (_signal_sign == 1) ? 0 : 1;
				_steps_counter = 0;

				if (_max_steps > 1) {
					_max_steps--;

				} else {
					_max_steps = 5;
				}
			}

			_steps_counter++;
			signal = float(_signal_sign);
		}
		break;

	case static_cast<int32_t>(SignalType::kLinearSineSweep): {

			// 线性扫频：频率从 f0 线性变化到 f1。
			signal = signal_generator::getLinearSineSweep(_param_fw_at_sysid_f0.get(),
					_param_fw_at_sysid_f1.get(),
					_param_fw_sysid_time.get(), t);
		}
		break;

	case static_cast<int32_t>(SignalType::kLogSineSweep): {
			// 对数扫频：低频停留更多时间，常用于宽频系统辨识。
			signal = signal_generator::getLogSineSweep(_param_fw_at_sysid_f0.get(), _param_fw_at_sysid_f1.get(),
					_param_fw_sysid_time.get(), t);
		}
		break;

	default:
		// 未知信号类型时输出 0，避免意外激励。
		signal = 0.f;
		break;
	}


	signal *= _signal_amp;
	// 最终幅值由幅值检测阶段确定。
	Vector3f rate_sp = scaleInputSignal(signal);

	return rate_sp;
}

const Vector3f FwAutotuneAttitudeControl::scaleInputSignal(const float signal)
{
	// 把标量激励转换为三轴 rate_sp，并按各轴姿态时间常数/角速度上限限幅。
	float signal_scaled = 0.f;
	Vector3f rate_sp{};

	if (_state == state::roll_amp_detection || _state == state::roll || _state == state::test) {
		// Scale the signal such that the attitude controller is
		// able to cancel it completely at an attitude error of pi/8
		// 中文：按 roll 姿态时间常数缩放，使 pi/8 姿态误差足以抵消激励。
		signal_scaled = math::min(signal * M_PI_F / (8.f * _param_fw_r_tc.get()), math::radians(_param_fw_r_rmax.get()));
		// 输出增量 rate_sp：当前缩放信号减去滤波器状态，形成较平滑的注入。
		rate_sp(0) = signal_scaled - _signal_filter.getState();
	}

	if (_state == state::pitch_amp_detection || _state ==  state::pitch || _state == state::test) {
		// pitch 正/负最大角速度取较小值作为安全上限。
		const float pitch_rate_max_deg = math::min(_param_fw_p_rmax_pos.get(), _param_fw_p_rmax_neg.get());
		signal_scaled = math::min(signal * M_PI_F / (8.f * _param_fw_p_tc.get()), math::radians(pitch_rate_max_deg));
		rate_sp(1) = signal_scaled - _signal_filter.getState();

	}

	if (_state == state::yaw_amp_detection || _state ==  state::yaw) {
		// Do not send a signal that produces more than a full deflection of the rudder
		// 中文：yaw 激励同时受舵面满偏对应控制量和 yaw rate 上限约束。
		signal_scaled = math::min(signal, 1.f / (_param_fw_yr_ff.get() + _param_fw_yr_p.get()),
					  math::radians(_param_fw_y_rmax.get()));
		rate_sp(2) = signal_scaled - _signal_filter.getState();
	}

	// 更新滤波器状态，下一周期用它计算增量注入。
	_signal_filter.update(signal_scaled);

	return rate_sp;
}

int FwAutotuneAttitudeControl::task_spawn(int argc, char *argv[])
{
	// 模块启动入口；带 "vtol" 参数时订阅 VTOL 固定翼控制组。
	bool is_vtol = false;

	if (argc > 1) {
		if (strcmp(argv[1], "vtol") == 0) {
			is_vtol = true;
		}
	}

	FwAutotuneAttitudeControl *instance = new FwAutotuneAttitudeControl(is_vtol);

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

int FwAutotuneAttitudeControl::custom_command(int argc, char *argv[])
{
	// 当前无额外 CLI 子命令。
	return print_usage("unknown command");
}

int FwAutotuneAttitudeControl::print_status()
{
	// 打印模块周期性能计数器。
	perf_print_counter(_cycle_perf);

	return 0;
}

int FwAutotuneAttitudeControl::print_usage(const char *reason)
{
	// 输出 PX4 shell 使用说明。
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("fw_autotune_attitude_control", "autotune");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int fw_autotune_attitude_control_main(int argc, char *argv[])
{
	// C ABI 入口，PX4 模块加载器通过它启动 C++ 模块。
	return FwAutotuneAttitudeControl::main(argc, argv);
}
