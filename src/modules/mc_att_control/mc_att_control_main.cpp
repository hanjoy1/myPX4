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

/**
 * @file mc_att_control_main.cpp
 * Multicopter attitude controller.
 *
 * @author Lorenz Meier		<lorenz@px4.io>
 * @author Anton Babushkin	<anton.babushkin@me.com>
 * @author Sander Smeets	<sander@droneslab.com>
 * @author Matthias Grob	<maetugr@gmail.com>
 * @author Beat Küng		<beat-kueng@gmx.net>
 *
 */

#include "mc_att_control.hpp"

#include <drivers/drv_hrt.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>

#include "AttitudeControl/AttitudeControlMath.hpp"

using namespace matrix;

// 构造多旋翼姿态控制模块：选择普通/VTOL attitude setpoint topic，初始化参数和手动油门斜率限制。
MulticopterAttitudeControl::MulticopterAttitudeControl(bool vtol) :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_vehicle_attitude_setpoint_pub(vtol ? ORB_ID(mc_virtual_attitude_setpoint) : ORB_ID(vehicle_attitude_setpoint)),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")),
	_vtol(vtol)
{
	parameters_updated();
	// Rate of change 5% per second -> 1.6 seconds to ramp to default 8% MPC_MANTHR_MIN
	// 手动最小油门每秒最多变化 5%，从 0 爬升到默认 8% 大约需要 1.6 秒。
	_manual_throttle_minimum.setSlewRate(0.05f);
	// Rate of change 50% per second -> 2 seconds to ramp to 100%
	// 手动最大油门每秒最多变化 50%，从 0 爬升到 100% 大约需要 2 秒。
	_manual_throttle_maximum.setSlewRate(0.5f);
	// Rate of change 5% per second -> 6 seconds to ramp 30% if hover thrust parameter is off
	// 悬停推力估计用于油门曲线时也做斜率限制，避免曲线中点突然跳变。
	_hover_thrust_slew_rate.setSlewRate(0.05f);
}

// 析构时释放性能计数器。
MulticopterAttitudeControl::~MulticopterAttitudeControl()
{
	perf_free(_loop_perf);
}

// 注册 vehicle_attitude 回调；之后姿态估计更新会触发 Run()。
bool
MulticopterAttitudeControl::init()
{
	if (!_vehicle_attitude_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

// 参数刷新后，把 PX4 参数转换成姿态控制器和手动控制逻辑直接使用的内部量。
void
MulticopterAttitudeControl::parameters_updated()
{
	// Store some of the parameters in a more convenient way & precompute often-used values
	// 将 roll/pitch/yaw 姿态 P 增益写入 AttitudeControl，并应用 yaw 权重。
	_attitude_control.setProportionalGain(Vector3f(_param_mc_roll_p.get(), _param_mc_pitch_p.get(), _param_mc_yaw_p.get()),
					      _param_mc_yaw_weight.get());

	// angular rate limits
	// 姿态 P 环输出的是角速度设定值，因此这里设置 roll/pitch/yaw 角速度硬限制。
	using math::radians;
	_attitude_control.setRateLimit(Vector3f(radians(_param_mc_rollrate_max.get()), radians(_param_mc_pitchrate_max.get()),
						radians(_param_mc_yawrate_max.get())));

	// Update from hover thrust parameter if there's no valid estimate in use
	// 如果没有有效 HTE，则用 MPC_THR_HOVER 强制设置油门曲线中点。
	if (!PX4_ISFINITE(_hover_thrust_estimate)) {
		_hover_thrust_slew_rate.setForcedValue(_param_mpc_thr_hover.get());
	}

	// 手动最大倾角参数是角度制，控制内部统一使用弧度。
	_man_tilt_max = math::radians(_param_mpc_man_tilt_max.get());
}

// 手动油门摇杆 [-1,1] 到归一化推力的映射曲线。
float
MulticopterAttitudeControl::throttle_curve(float throttle_stick_input)
{
	float thrust = 0.f;

	// throttle_stick_input is in range [-1, 1]
	// 油门摇杆输入范围是 [-1, 1]，输出推力是 [manual_min, MPC_THR_MAX] 附近的归一化值。
	switch (_param_mpc_thr_curve.get()) {
	case 1: // no rescaling
		// 不以悬停推力重映射：摇杆从最低到最高线性映射到最小/最大推力。
		thrust = math::interpolate(throttle_stick_input, -1.f, 1.f,
					   _manual_throttle_minimum.getState(), _param_mpc_thr_max.get());
		break;

	case 2: // rescale to hover thrust param at 0 stick input
		// 使用参数 MPC_THR_HOVER 作为摇杆中位点推力。
		thrust = math::interpolateNXY(throttle_stick_input,
		{-1.f, 0.f, 1.f},
		{_manual_throttle_minimum.getState(), _param_mpc_thr_hover.get(), _param_mpc_thr_max.get()});
		break;

	default: // 0 or other: rescale to HTE value
		// 默认使用 HTE/斜率限制后的悬停推力作为摇杆中位点推力。
		thrust = math::interpolateNXY(throttle_stick_input,
		{-1.f, 0.f, 1.f},
		{_manual_throttle_minimum.getState(), _hover_thrust_slew_rate.getState(), _param_mpc_thr_max.get()});
		break;
	}

	// 最终还要受手动最大油门斜率限制，spoolup 前该值会被强制为 0。
	return math::min(thrust, _manual_throttle_maximum.getState());
}

// 在纯手动/Stabilized 姿态模式下，根据摇杆输入生成并发布 vehicle_attitude_setpoint。
void
MulticopterAttitudeControl::generate_attitude_setpoint(const Quatf &q, float dt)
{
	vehicle_attitude_setpoint_s attitude_setpoint{};

	// Avoid accumulating absolute yaw error with arming stick gesture
	// 解锁摇杆动作通常是低油门 + yaw 输入，此时不要累积绝对 yaw 误差。
	const bool arming_gesture = (_manual_control_setpoint.throttle < -.9f) && (_param_mc_airmode.get() != 2);

	if (arming_gesture || !_heading_good_for_control) {
		// yaw setpoint 为 NaN 表示 yaw 解锁/不保持绝对航向，只输出 yaw rate。
		_yaw_setpoint_stabilized = NAN;
	}

	const float yaw = Eulerf(q).psi();
	const float yaw_stick_input = math::expo_deadzone(_manual_control_setpoint.yaw, .6f, _param_man_deadzone.get());
	// StickYaw 根据摇杆输入生成 yaw rate，并在无 yaw 输入时维持/更新 yaw lock。
	_stick_yaw.generateYawSetpoint(attitude_setpoint.yaw_sp_move_rate, _yaw_setpoint_stabilized, yaw_stick_input, yaw, dt,
				       _unaided_heading);

	/*
	 * Input mapping for roll & pitch setpoints
	 * ----------------------------------------
	 * We control the following 2 angles:
	 * - tilt angle, given by sqrt(roll*roll + pitch*pitch)
	 * - the direction of the maximum tilt in the XY-plane, which also defines the direction of the motion
	 *
	 * This allows a simple limitation of the tilt angle, the vehicle flies towards the direction that the stick
	 * points to, and changes of the stick input are linear.
	 */
	/*
	 * 中文说明：
	 * roll/pitch 摇杆不是直接当欧拉角，而是映射为“倾角大小 + 倾斜方向”。
	 * 这样更容易限制总倾角，且摇杆指向哪里，飞机就朝哪里倾斜/运动。
	 */
	_man_roll_input_filter.setParameters(dt, _param_mc_man_tilt_tau.get());
	_man_pitch_input_filter.setParameters(dt, _param_mc_man_tilt_tau.get());

	// we want to fly towards the direction of (roll, pitch)
	// roll/pitch 摇杆经过一阶滤波后映射到倾角向量；pitch 取负号是为了符合机体系/摇杆方向约定。
	Vector2f v = Vector2f(_man_roll_input_filter.update(_manual_control_setpoint.roll * _man_tilt_max),
			      -_man_pitch_input_filter.update(_manual_control_setpoint.pitch * _man_tilt_max));
	float v_norm = v.norm(); // the norm of v defines the tilt angle
	// 倾角向量的模长就是总倾角。

	if (v_norm > _man_tilt_max) { // limit to the configured maximum tilt angle
		// 如果总倾角超过手动最大倾角，则保持方向不变、缩小模长。
		v *= _man_tilt_max / v_norm;
	}

	Quatf q_sp_rp = AxisAnglef(v(0), v(1), 0.f);
	// Make sure there's a valid attitude quaternion with no yaw error when yaw is unlocked (NAN)
	// yaw 解锁时用当前 yaw 构造 setpoint，避免产生无意义的 yaw 误差。
	const float yaw_setpoint = PX4_ISFINITE(_yaw_setpoint_stabilized) ? _yaw_setpoint_stabilized : yaw;
	// The axis angle can change the yaw as well (noticeable at higher tilt angles).
	// This is the formula by how much the yaw changes:
	//   let a := tilt angle, b := atan(y/x) (direction of maximum tilt)
	//   yaw = atan(-2 * sin(b) * cos(b) * sin^2(a/2) / (1 - 2 * cos^2(b) * sin^2(a/2))).
	// 由于 roll/pitch 轴角在大倾角时也会引入 yaw 分量，所以先单独构造 yaw 四元数再组合。
	const Quatf q_sp_yaw(cosf(yaw_setpoint / 2.f), 0.f, 0.f, sinf(yaw_setpoint / 2.f));

	if (_vtol) {
		// Modify the setpoints for roll and pitch such that they reflect the user's intention even
		// if a large yaw error(yaw_sp - yaw) is present. In the presence of a yaw error constructing
		// an attitude setpoint from the yaw setpoint will lead to unexpected attitude behaviour from
		// the user's view as the tilt will not be aligned with the heading of the vehicle.
		// VTOL 中如果当前 yaw 与 yaw setpoint 差很大，直接用 yaw setpoint 组合倾角会让用户感觉倾斜方向不对。
		// 这里修正 roll/pitch 倾角，使它更符合用户以当前机头方向为参考的摇杆意图。

		AttitudeControlMath::correctTiltSetpointForYawError(q_sp_rp, q, q_sp_yaw);
	}

	// Align the desired tilt with the yaw setpoint
	// 将期望倾角旋转到 yaw setpoint 对应的航向下，得到完整姿态设定值。
	Quatf q_sp = q_sp_yaw * q_sp_rp;

	q_sp.copyTo(attitude_setpoint.q_d);

	// PX4 中 thrust_body[2] 为负表示沿 -body_z 方向产生升力。
	attitude_setpoint.thrust_body[2] = -throttle_curve(_manual_control_setpoint.throttle);

	attitude_setpoint.timestamp = hrt_absolute_time();
	_vehicle_attitude_setpoint_pub.publish(attitude_setpoint);
}

// 姿态控制主循环：由 vehicle_attitude 更新触发，输出 vehicle_rates_setpoint。
void
MulticopterAttitudeControl::Run()
{
	if (should_exit()) {
		// 模块退出时取消姿态回调并清理资源。
		_vehicle_attitude_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	// Check if parameters have changed
	// 检查参数是否更新。
	if (_parameter_update_sub.updated()) {
		// clear update
		// 读取并清除参数更新标志。
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();
		parameters_updated();
	}

	// Update hover thrust for stick scaling
	// 更新手动油门曲线使用的悬停推力估计。
	if (_hover_thrust_estimate_sub.updated()) {
		hover_thrust_estimate_s hover_thrust_estimate;

		if (_hover_thrust_estimate_sub.update(&hover_thrust_estimate)) {
			if (hover_thrust_estimate.valid) {
				// 有效估计限制在合理范围内，防止异常估计影响油门曲线。
				_hover_thrust_estimate = math::constrain(hover_thrust_estimate.hover_thrust, .05f, .9f);

			} else {
				// Possibly bad estimate before it got invalid, slew back to parameter
				// 估计失效时回到参数值；外层 slew rate 会让变化平滑。
				_hover_thrust_estimate = _param_mpc_thr_hover.get();
			}
		}
	}

	// run controller on attitude updates
	// 姿态控制器只在新的 vehicle_attitude 到来时运行。
	vehicle_attitude_s v_att;

	if (_vehicle_attitude_sub.update(&v_att)) {

		// Guard against too small (< 0.2ms) and too large (> 20ms) dt's.
		// 限制 dt，避免异常采样间隔影响滤波、yaw 设定值生成和 slew rate。
		const float dt = math::constrain(((v_att.timestamp_sample - _last_run) * 1e-6f), 0.0002f, 0.02f);
		_last_run = v_att.timestamp_sample;

		const Quatf q{v_att.q};

		/* check for updates in other topics */
		/* 更新其他输入 topic：手动摇杆、控制模式、车辆状态、落地状态、航向质量等。 */
		_manual_control_setpoint_sub.update(&_manual_control_setpoint);
		_vehicle_control_mode_sub.update(&_vehicle_control_mode);

		if (_vehicle_status_sub.updated()) {
			vehicle_status_s vehicle_status;

			if (_vehicle_status_sub.copy(&vehicle_status)) {
				// vehicle_status 决定当前是否旋翼、是否 VTOL、是否转换模式、是否 tailsitter。
				_vehicle_type_rotary_wing = (vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);
				_vtol = vehicle_status.is_vtol;
				_vtol_in_transition_mode = vehicle_status.in_transition_mode;
				_vtol_tailsitter = vehicle_status.is_vtol_tailsitter;

				const bool armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
				// 解锁后经过 COM_SPOOLUP_TIME 才认为电机预转完成，手动最大油门才会逐渐放开。
				_spooled_up = armed && hrt_elapsed_time(&vehicle_status.armed_time) > _param_com_spoolup_time.get() * 1_s;
			}
		}

		if (_vehicle_land_detected_sub.updated()) {
			vehicle_land_detected_s vehicle_land_detected;

			if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)) {
				// 落地时手动最小油门会缓慢回到 0，空中则回到 MPC_MANTHR_MIN。
				_landed = vehicle_land_detected.landed;
			}
		}

		if (_vehicle_local_position_sub.updated()) {
			vehicle_local_position_s vehicle_local_position;

			if (_vehicle_local_position_sub.copy(&vehicle_local_position)) {
				// heading_good_for_control 用于决定手动 yaw lock 是否可靠。
				_heading_good_for_control = vehicle_local_position.heading_good_for_control;
				_unaided_heading = vehicle_local_position.unaided_heading;
			}
		}

		// during transitions VTOL module generates attitude setpoints
		// VTOL 转换期间通常由 VTOL 模块生成姿态 setpoint，普通多旋翼姿态控制只在 hover/tailsitter transition 情况运行。
		const bool is_hovering = (_vehicle_type_rotary_wing && !_vtol_in_transition_mode);
		const bool is_tailsitter_transition = (_vtol_tailsitter && _vtol_in_transition_mode);

		const bool run_att_ctrl = _vehicle_control_mode.flag_control_attitude_enabled
					  && (is_hovering || is_tailsitter_transition);

		if (run_att_ctrl) {
			// Generate the attitude setpoint from stick inputs if we are in Manual/Stabilized mode
			// 如果是纯手动/Stabilized 姿态模式，则由本模块根据摇杆生成 attitude_setpoint。
			if (_vehicle_control_mode.flag_control_manual_enabled &&
			    !_vehicle_control_mode.flag_control_altitude_enabled &&
			    !_vehicle_control_mode.flag_control_velocity_enabled &&
			    !_vehicle_control_mode.flag_control_position_enabled) {

				generate_attitude_setpoint(q, dt);

			} else {
				// 非纯手动姿态模式时，姿态 setpoint 通常来自位置/高度/任务等上游模块。
				// 此时重置手动 roll/pitch 滤波和 yaw lock，避免下次切回手动时沿用旧状态。
				_man_roll_input_filter.reset(0.f);
				_man_pitch_input_filter.reset(0.f);
				_yaw_setpoint_stabilized = NAN;
				_stick_yaw.reset(Eulerf(q).psi(), _unaided_heading);
			}

			// Check for new attitude setpoint
			// 检查是否有新的 vehicle_attitude_setpoint，通常来自 mc_pos_control 或本模块手动生成。
			if (_vehicle_attitude_setpoint_sub.updated()) {
				vehicle_attitude_setpoint_s vehicle_attitude_setpoint;

				if (_vehicle_attitude_setpoint_sub.copy(&vehicle_attitude_setpoint)
				    && (vehicle_attitude_setpoint.timestamp > _last_attitude_setpoint)) {

					// 将期望姿态四元数和 yaw rate 前馈交给 AttitudeControl。
					_attitude_control.setAttitudeSetpoint(Quatf(vehicle_attitude_setpoint.q_d), vehicle_attitude_setpoint.yaw_sp_move_rate);
					// 推力设定值不参与姿态 P 环计算，但要随 rates setpoint 继续传给下游 rate controller/mixer。
					_thrust_setpoint_body = Vector3f(vehicle_attitude_setpoint.thrust_body);
					_last_attitude_setpoint = vehicle_attitude_setpoint.timestamp;
				}
			}

			// Check for a heading reset
			// 检查 EKF 姿态/航向 reset。reset 后当前姿态参考系跳变，需要同步修正内部 yaw/setpoint 状态。
			if (_quat_reset_counter != v_att.quat_reset_counter) {
				const Quatf delta_q_reset(v_att.delta_q_reset);
				const float delta_psi = Eulerf(delta_q_reset).psi();

				// Only offset the yaw setpoint when the heading is locked
				// 只有 yaw 处于锁定状态时才平移 yaw setpoint；如果 yaw 解锁为 NaN，就不需要修正。
				if (PX4_ISFINITE(_yaw_setpoint_stabilized)) {
					_yaw_setpoint_stabilized = wrap_pi(_yaw_setpoint_stabilized + delta_psi);
				}

				// 通知 StickYaw 处理 EKF 航向 reset，避免手动 yaw lock 产生跳变。
				_stick_yaw.ekfResetHandler(delta_psi);

				if (v_att.timestamp > _last_attitude_setpoint) {
					// adapt existing attitude setpoint unless it was generated after the current attitude estimate
					// 如果当前 attitude setpoint 早于姿态 reset，则把它也旋转同样的 delta，保持物理目标姿态不变。
					_attitude_control.adaptAttitudeSetpoint(delta_q_reset);
				}

				// 更新 reset counter，避免重复处理同一次 reset。
				_quat_reset_counter = v_att.quat_reset_counter;
			}

			// 姿态 P 环：当前姿态 q + 期望姿态 qd -> 机体系 roll/pitch/yaw 角速度设定值。
			Vector3f rates_sp = _attitude_control.update(q);

			const hrt_abstime now = hrt_absolute_time();
			autotune_attitude_control_status_s pid_autotune;

			if (_autotune_attitude_control_status_sub.copy(&pid_autotune)) {
				// 姿态自整定运行时，会向角速度设定值叠加测试激励。
				if ((pid_autotune.state == autotune_attitude_control_status_s::STATE_ROLL
				     || pid_autotune.state == autotune_attitude_control_status_s::STATE_PITCH
				     || pid_autotune.state == autotune_attitude_control_status_s::STATE_YAW
				     || pid_autotune.state == autotune_attitude_control_status_s::STATE_TEST)
				    && ((now - pid_autotune.timestamp) < 1_s)) {
					// 只接受 1 秒内的新鲜 autotune setpoint，避免使用过期激励。
					rates_sp += Vector3f(pid_autotune.rate_sp);
				}
			}

			// publish rate setpoint
			// 发布给 mc_rate_control 的角速度设定值。推力向量原样随消息继续传递。
			vehicle_rates_setpoint_s rates_setpoint{};
			rates_setpoint.roll = rates_sp(0);
			rates_setpoint.pitch = rates_sp(1);
			rates_setpoint.yaw = rates_sp(2);
			_thrust_setpoint_body.copyTo(rates_setpoint.thrust_body);
			rates_setpoint.timestamp = hrt_absolute_time();

			_vehicle_rates_setpoint_pub.publish(rates_setpoint);

		} else {
			// 姿态控制未运行时重置手动输入滤波和 yaw lock 状态，避免恢复时跳变。
			_man_roll_input_filter.reset(0.f);
			_man_pitch_input_filter.reset(0.f);
			_yaw_setpoint_stabilized = NAN;
			_stick_yaw.reset(Eulerf(q).psi(), _unaided_heading);
		}

		if (_landed) {
			// 落地时手动最小油门缓慢回到 0，避免地面误给最小推力。
			_manual_throttle_minimum.update(0.f, dt);

		} else {
			// 空中时手动最小油门缓慢爬升到 MPC_MANTHR_MIN，避免油门过低导致失控。
			_manual_throttle_minimum.update(_param_mpc_manthr_min.get(), dt);
		}

		if (_spooled_up) {
			// 电机预转完成后，手动最大油门逐步放开到 100%。
			_manual_throttle_maximum.update(1.f, dt);

		} else {
			// 预转未完成时强制最大油门为 0，确保 spoolup 阶段不能起飞。
			_manual_throttle_maximum.setForcedValue(0.f);
		}

		if (PX4_ISFINITE(_hover_thrust_estimate)) {
			// 有有效 HTE 时，用 slew rate 平滑更新手动油门曲线中点。
			_hover_thrust_slew_rate.update(_hover_thrust_estimate, dt);
		}
	}

	perf_end(_loop_perf);
}

// PX4 模块启动入口：解析 vtol 参数，创建对象并注册回调。
int MulticopterAttitudeControl::task_spawn(int argc, char *argv[])
{
	bool vtol = false;

	if (argc > 1) {
		if (strcmp(argv[1], "vtol") == 0) {
			vtol = true;
		}
	}

	MulticopterAttitudeControl *instance = new MulticopterAttitudeControl(vtol);

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

// 处理自定义命令；当前没有自定义命令。
int MulticopterAttitudeControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

// 打印模块说明和命令行用法。
int MulticopterAttitudeControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements the multicopter attitude controller. It takes attitude
setpoints (`vehicle_attitude_setpoint`) as inputs and outputs a rate setpoint.

The controller has a P loop for angular error

Publication documenting the implemented Quaternion Attitude Control:
Nonlinear Quadrocopter Attitude Control (2013)
by Dario Brescianini, Markus Hehn and Raffaello D'Andrea
Institute for Dynamic Systems and Control (IDSC), ETH Zurich

https://www.research-collection.ethz.ch/bitstream/handle/20.500.11850/154099/eth-7387-01.pdf

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mc_att_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}


/**
 * Multicopter attitude control app start / stop handling function
 * 多旋翼姿态控制模块 C 语言导出入口，转发给 ModuleBase。
 */
extern "C" __EXPORT int mc_att_control_main(int argc, char *argv[])
{
	return MulticopterAttitudeControl::main(argc, argv);
}
