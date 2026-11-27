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

#include "FixedwingAttitudeControl.hpp"

/*
 * 固定翼姿态控制模块。
 *
 * 通信链路位置：
 *   vehicle_attitude_setpoint -> vehicle_rates_setpoint
 *
 * 这是固定翼内环中的姿态外环。模块读取上游给出的期望姿态四元数
 * 和油门指令，然后生成角速度控制器需要的机体系角速度设定值。
 * 滚转和俯仰按姿态误差闭环；固定翼正常飞行时不直接控制绝对偏航角，
 * 而是根据滚转、俯仰和空速生成协调转弯所需的偏航角速度。
 *
 * 复现到 MATLAB/Simulink 时，可把本文件看成一个离散子系统：
 * 输入为姿态、状态、控制模式等总线，输出为 vehicle_rates_setpoint 总线。
 */

using namespace time_literals;
using namespace matrix;

using math::constrain;
using math::radians;

/*
 * 构造函数。
 *
 * vtol=false 时发布普通固定翼话题 vehicle_attitude_setpoint；
 * vtol=true 时发布 VTOL 固定翼虚拟话题 fw_virtual_attitude_setpoint。
 * 这样同一套固定翼姿态控制逻辑可以被纯固定翼和 VTOL 固定翼阶段复用。
 */
FixedwingAttitudeControl::FixedwingAttitudeControl(bool vtol) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_attitude_sp_pub(vtol ? ORB_ID(fw_virtual_attitude_setpoint) : ORB_ID(vehicle_attitude_setpoint)),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	/* fetch initial parameter values */
	// 启动时先读取一次参数，确保各个子控制器的时间常数、限幅和增益有效。
	parameters_update();

	// 提前 advertise 跑道轮话题，使后续发布前 uORB 通道已经建立。
	_landing_gear_wheel_pub.advertise();
}

/*
 * 析构函数。
 *
 * 释放性能计数器资源；控制器内部没有动态分配的其他资源。
 */
FixedwingAttitudeControl::~FixedwingAttitudeControl()
{
	perf_free(_loop_perf);
}

bool
FixedwingAttitudeControl::init()
{
	/*
	 * 将本模块注册为 vehicle_attitude 更新的回调消费者。
	 * 姿态更新是固定翼姿态外环的主触发源：每来一次新姿态，就尝试运行一次控制。
	 */
	if (!_att_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void
FixedwingAttitudeControl::parameters_update()
{
	/*
	 * 将 PX4 参数转换为控制器内部使用的单位。
	 *
	 * 姿态外环使用“时间常数”形式：姿态误差除以时间常数得到欧拉角速度设定值。
	 * 最大机体系角速度限制在参数中以 deg/s 存储，这里统一转换为 rad/s。
	 */
	_roll_ctrl.set_time_constant(_param_fw_r_tc.get());
	_roll_ctrl.set_max_rate(radians(_param_fw_r_rmax.get()));

	_pitch_ctrl.set_time_constant(_param_fw_p_tc.get());
	_pitch_ctrl.set_max_rate_pos(radians(_param_fw_p_rmax_pos.get()));
	_pitch_ctrl.set_max_rate_neg(radians(_param_fw_p_rmax_neg.get()));

	_yaw_ctrl.set_max_rate(radians(_param_fw_y_rmax.get()));

	_wheel_ctrl.set_k_p(_param_fw_wr_p.get());
	_wheel_ctrl.set_k_i(_param_fw_wr_i.get());
	_wheel_ctrl.set_k_ff(_param_fw_wr_ff.get());
	_wheel_ctrl.set_integrator_max(_param_fw_wr_imax.get());
	_wheel_ctrl.set_max_rate(radians(_param_fw_w_rmax.get()));
	_wheel_ctrl.set_time_constant(0.1f);
}

void
FixedwingAttitudeControl::vehicle_manual_poll(const float yaw_body)
{
	/*
	 * 读取手动输入并在“手动 + 固定翼/转换阶段”下生成姿态设定值。
	 * yaw_body 传入当前偏航角，用于构造完整四元数，但不代表控制偏航角。
	 */
	if (_vcontrol_mode.flag_control_manual_enabled && _in_fw_or_transition_wo_tailsitter_transition) {

		// Always copy the new manual setpoint, even if it wasn't updated, to fill the actuators with valid values
		// 即使没有 updated，也尝试 copy 最近一次手动杆量，保证输出始终有有效值填充。
		if (_manual_control_setpoint_sub.copy(&_manual_control_setpoint)) {

			if (!_vcontrol_mode.flag_control_climb_rate_enabled && _vcontrol_mode.flag_control_attitude_enabled) {

				/*
				 * 增稳模式：
				 * 滚转杆和俯仰杆直接映射为姿态角指令。四元数中的偏航角使用当前偏航角，
				 * 因为固定翼姿态控制器不跟踪偏航姿态本身。油门作为归一化 thrust_body[0]
				 * 继续向下游传递。
				 */

				const float roll_body = _manual_control_setpoint.roll * radians(_param_fw_man_r_max.get());

				// PX4 手动俯仰杆向前通常为负，所以这里取负号；再叠加平飞俯仰偏置 FW_PSP_OFF。
				float pitch_body = -_manual_control_setpoint.pitch * radians(_param_fw_man_p_max.get())
						   + radians(_param_fw_psp_off.get());

				// 手动俯仰最终限制在最大手动俯仰角范围内，避免杆量和偏置叠加后超限。
				pitch_body = constrain(pitch_body,
						       -radians(_param_fw_man_p_max.get()), radians(_param_fw_man_p_max.get()));

				// PX4 手动油门杆范围是 [-1, 1]，固定翼油门需要转换到 [0, 1]。
				_att_sp.thrust_body[0] = (_manual_control_setpoint.throttle + 1.f) * .5f;

				// 用 roll/pitch 指令和当前 yaw 组装姿态四元数，作为下游统一姿态 setpoint。
				const Quatf q(Eulerf(roll_body, pitch_body, yaw_body));
				q.copyTo(_att_sp.q_d);

				_att_sp.timestamp = hrt_absolute_time();

				_attitude_sp_pub.publish(_att_sp);
			}
		}
	}
}

void
FixedwingAttitudeControl::vehicle_attitude_setpoint_poll()
{
	/*
	 * 读取上游姿态 setpoint。
	 * 自动模式下它通常来自 fw_lateral_longitudinal_control；
	 * 手动增稳模式下可能由本模块 vehicle_manual_poll() 刚刚发布/更新。
	 */
	if (_att_sp_sub.update(&_att_sp)) {
		/*
		 * 保留上游生成的推力/油门指令。姿态模块只负责计算角速度设定值，
		 * 油门会透传给角速度模块，后者再发布 vehicle_thrust_setpoint。
		 */
		_rates_sp.thrust_body[0] = _att_sp.thrust_body[0];
		_rates_sp.thrust_body[1] = _att_sp.thrust_body[1];
		_rates_sp.thrust_body[2] = _att_sp.thrust_body[2];
	}
}

void
FixedwingAttitudeControl::vehicle_land_detected_poll()
{
	/*
	 * 更新落地状态。
	 * 落地时需要复位积分，防止地面静止误差累积到下一次飞行。
	 */
	if (_vehicle_land_detected_sub.updated()) {
		vehicle_land_detected_s vehicle_land_detected {};

		if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)) {
			_landed = vehicle_land_detected.landed;
		}
	}
}

float FixedwingAttitudeControl::get_airspeed_constrained()
{
	/*
	 * 协调转弯偏航控制依赖空速。FW_USE_AIRSPD 使能且空速测量新鲜时优先使用
	 * calibrated airspeed；否则退回到巡航配平空速。最终空速会被限制在合理范围，
	 * 避免偏航角速度计算中出现除以过小空速的奇异情况。
	 */
	_airspeed_validated_sub.update();
	const bool airspeed_valid = PX4_ISFINITE(_airspeed_validated_sub.get().calibrated_airspeed_m_s)
				    && (hrt_elapsed_time(&_airspeed_validated_sub.get().timestamp) < 1_s);

	// if no airspeed measurement is available out best guess is to use the trim airspeed
	// 没有可靠空速时使用巡航配平空速，这是固定翼协调转弯计算的默认估计。
	float airspeed = _param_fw_airspd_trim.get();

	if (_param_fw_use_airspd.get() && airspeed_valid) {
		/* prevent numerical drama by requiring 0.5 m/s minimal speed */
		// 即使传感器很小，也至少给 0.5 m/s，避免后面 g/airspeed 出现数值问题。
		airspeed = math::max(0.5f, _airspeed_validated_sub.get().calibrated_airspeed_m_s);

	} else {
		// VTOL: if we have no airspeed available and we are in hover mode then assume the lowest airspeed possible
		// this assumption is good as long as the vehicle is not hovering in a headwind which is much larger
		// than the stall airspeed
		// VTOL 悬停阶段没有固定翼空速概念，使用失速空速作为最保守的舵效估计。
		if (_vehicle_status.is_vtol && _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
		    && !_vehicle_status.in_transition_mode) {
			airspeed = _param_fw_airspd_stall.get();
		}
	}

	return math::constrain(airspeed, _param_fw_airspd_stall.get(), _param_fw_airspd_max.get());
}

void FixedwingAttitudeControl::Run()
{
	/*
	 * 主工作队列回调。通常由 vehicle_attitude 更新触发；如果姿态更新慢于预期，
	 * 也会按周期运行。dt 限幅用于防止更新停顿或突发更新导致控制步长异常。
	 */
	if (should_exit()) {
		_att_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	// only run controller if attitude changed
	if (_att_sub.updated() || (hrt_elapsed_time(&_last_run) > 20_ms)) {

		// only update parameters if they changed
		// PX4 参数系统通过 parameter_update 通知变化，避免每个周期都重新读取参数。
		const bool params_updated = _parameter_update_sub.updated();

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

		vehicle_attitude_s att{};

		if (_att_sub.copy(&att)) {
			// 使用姿态消息采样时间计算 dt，使控制周期和估计器输出时间对齐。
			dt = math::constrain((att.timestamp_sample - _last_run) * 1e-6f, DT_MIN, DT_MAX);
			_last_run = att.timestamp_sample;

			// get current rotation matrix and euler angles from control state quaternions
			// 将估计姿态四元数转成方向余弦矩阵，后面统一从矩阵提取欧拉角。
			_R = matrix::Quatf(att.q);
		}

		if (dt < DT_MIN || dt > DT_MAX) {
			// 如果姿态消息时间戳异常，则退回到当前系统时间估算 dt，并继续限幅保护。
			const hrt_abstime time_now_us = hrt_absolute_time();
			dt = math::constrain((time_now_us - _last_run) * 1e-6f, DT_MIN, DT_MAX);
			_last_run = time_now_us;
		}

		if (_vehicle_status.is_vtol_tailsitter) {
			/* vehicle is a tailsitter, we need to modify the estimated attitude for fw mode
			 *
			 * Since the VTOL airframe is initialized as a multicopter we need to
			 * modify the estimated attitude for the fixed wing operation.
			 * Since the neutral position of the vehicle in fixed wing mode is -90 degrees rotated around
			 * the pitch axis compared to the neutral position of the vehicle in multicopter mode
			 * we need to swap the roll and the yaw axis (1st and 3rd column) in the rotation matrix.
			 * Additionally, in order to get the correct sign of the pitch, we need to multiply
			 * the new x axis of the rotation matrix with -1
			 *
			 * original:			modified:
			 *
			 * Rxx  Ryx  Rzx		-Rzx  Ryx  Rxx
			 * Rxy	Ryy  Rzy		-Rzy  Ryy  Rxy
			 * Rxz	Ryz  Rzz		-Rzz  Ryz  Rxz
			 * */
			matrix::Dcmf R_adapted = _R;		//modified rotation matrix

			/* move z to x */
			// 尾座式在固定翼模式下机体轴定义相对多旋翼悬停坐标系旋转，需要把 z 轴搬到 x 轴。
			R_adapted(0, 0) = _R(0, 2);
			R_adapted(1, 0) = _R(1, 2);
			R_adapted(2, 0) = _R(2, 2);

			/* move x to z */
			// 原来的 x 轴变为新的 z 轴，完成固定翼坐标系所需的轴交换。
			R_adapted(0, 2) = _R(0, 0);
			R_adapted(1, 2) = _R(1, 0);
			R_adapted(2, 2) = _R(2, 0);

			/* change direction of pitch (convert to right handed system) */
			// 反转新 x 轴方向，使转换后的姿态仍满足右手坐标系约定。
			R_adapted(0, 0) = -R_adapted(0, 0);
			R_adapted(1, 0) = -R_adapted(1, 0);
			R_adapted(2, 0) = -R_adapted(2, 0);

			/* fill in new attitude data */
			_R = R_adapted;
		}

		const matrix::Eulerf euler_angles(_R);

		// 手动增稳模式可能在这里根据杆量生成新的姿态 setpoint。
		vehicle_manual_poll(euler_angles.psi());

		// 读取最新姿态 setpoint，并把其中的 thrust_body 透传到 rates setpoint。
		vehicle_attitude_setpoint_poll();

		// vehicle status update must be before the vehicle_control_mode poll, otherwise rate sp are not published during whole transition
		// 先更新 vehicle_status，再更新 control_mode，确保 VTOL 转换期间固定翼 rate setpoint 不会漏发。
		_vehicle_status_sub.update(&_vehicle_status);
		const bool is_in_transition_except_tailsitter = _vehicle_status.in_transition_mode
				&& !_vehicle_status.is_vtol_tailsitter;
		const bool is_fixed_wing = _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING;
		_in_fw_or_transition_wo_tailsitter_transition =  is_fixed_wing || is_in_transition_except_tailsitter;

		_vehicle_control_mode_sub.update(&_vcontrol_mode);

		// 落地状态影响积分复位标志和前轮控制器积分。
		vehicle_land_detected_poll();

		/* if we are in rotary wing mode, do nothing */
		// 纯多旋翼模式不运行固定翼姿态控制；VTOL 会由对应 VTOL 控制模块处理。
		if (_vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING && !_vehicle_status.is_vtol) {
			perf_end(_loop_perf);
			return;
		}

		if (_vcontrol_mode.flag_control_rates_enabled) {

			/* Reset integrators if the aircraft is on ground
			 * or a multicopter (but not transitioning VTOL or tailsitter)
			 */
			if (_landed
			    || !_in_fw_or_transition_wo_tailsitter_transition) {

				// 地面或非固定翼阶段时要求下游 rate controller 清积分，避免积分带入飞行。
				_rates_sp.reset_integral = true;
				_wheel_ctrl.reset_integrator();

			} else {
				_rates_sp.reset_integral = false;
			}

			/* Run attitude controllers */

			if (_vcontrol_mode.flag_control_attitude_enabled && _in_fw_or_transition_wo_tailsitter_transition) {
				const Quatf q_sp(_att_sp.q_d);

				if (q_sp.isAllFinite()) {
					/*
					 * 外环换算关系：
					 *   滚转姿态误差 -> 机体系滚转角速度设定值
					 *   俯仰姿态误差 -> 机体系俯仰角速度设定值
					 *   协调转弯关系 -> 机体系偏航角速度设定值
					 *
					 * 偏航控制器不使用“偏航角误差”，而是根据滚转指令、
					 * 当前姿态和空速计算协调转弯需要的偏航角速度。
					 */
					const Eulerf euler_sp(q_sp);
					const float roll_sp = euler_sp.phi();
					const float pitch_sp = euler_sp.theta();

					_roll_ctrl.control_roll(roll_sp, _yaw_ctrl.get_euler_rate_setpoint(), euler_angles.phi(),
								euler_angles.theta());
					_pitch_ctrl.control_pitch(pitch_sp, _yaw_ctrl.get_euler_rate_setpoint(), euler_angles.phi(),
								  euler_angles.theta());
					_yaw_ctrl.control_yaw(roll_sp, _pitch_ctrl.get_euler_rate_setpoint(), euler_angles.phi(),
							      euler_angles.theta(), get_airspeed_constrained());

					/* Update input data for rate controllers */
					Vector3f body_rates_setpoint = Vector3f(_roll_ctrl.get_body_rate_setpoint(), _pitch_ctrl.get_body_rate_setpoint(),
										_yaw_ctrl.get_body_rate_setpoint());

					autotune_attitude_control_status_s pid_autotune;
					matrix::Vector3f bodyrate_autotune_ff;

					if (_autotune_attitude_control_status_sub.copy(&pid_autotune)) {
						// 自整定模块激活时，会向 body rate setpoint 叠加辨识激励。
						if ((pid_autotune.state == autotune_attitude_control_status_s::STATE_ROLL
						     || pid_autotune.state == autotune_attitude_control_status_s::STATE_PITCH
						     || pid_autotune.state == autotune_attitude_control_status_s::STATE_YAW
						     || pid_autotune.state == autotune_attitude_control_status_s::STATE_ROLL_AMPLITUDE_DETECTION
						     || pid_autotune.state == autotune_attitude_control_status_s::STATE_PITCH_AMPLITUDE_DETECTION
						     || pid_autotune.state == autotune_attitude_control_status_s::STATE_YAW_AMPLITUDE_DETECTION
						     || pid_autotune.state == autotune_attitude_control_status_s::STATE_TEST)
						    && ((hrt_absolute_time() - pid_autotune.timestamp) < 1_s)) {

							bodyrate_autotune_ff = matrix::Vector3f(pid_autotune.rate_sp);
							body_rates_setpoint += bodyrate_autotune_ff;
						}
					}

					/* add yaw rate setpoint from sticks in all attitude-controlled modes */
					if (_vcontrol_mode.flag_control_manual_enabled) {
						/*
						 * 手动偏航杆量是叠加项：飞手可以在不改变滚转/俯仰
						 * 姿态目标的情况下，对协调转弯偏航角速度进行微调。
						 */
						body_rates_setpoint(2) += math::constrain(_manual_control_setpoint.yaw * radians(_param_man_yr_max.get()),
									  -radians(_param_fw_y_rmax.get()), radians(_param_fw_y_rmax.get()));
					}

					// Tailsitter: transform from FW to hover frame (all interfaces are in hover (body) frame)
					// 尾座式发布接口仍使用悬停机体系，因此固定翼计算出的角速度要再映射回接口坐标系。
					if (_vehicle_status.is_vtol_tailsitter) {
						body_rates_setpoint = Vector3f(body_rates_setpoint(2), body_rates_setpoint(1), -body_rates_setpoint(0));
					}

					/* Publish the rate setpoint for analysis once available */
					_rates_sp.roll = body_rates_setpoint(0);
					_rates_sp.pitch = body_rates_setpoint(1);
					_rates_sp.yaw = body_rates_setpoint(2);

					_rates_sp.timestamp = hrt_absolute_time();

					_rate_sp_pub.publish(_rates_sp);
				}
			}
		}

		// steering wheel control
		fixed_wing_runway_control_s runway_control{};
		_fixed_wing_runway_control_sub.copy(&runway_control);
		/*
		 * 跑道轮转向只在自动起飞/自动降落通过 fixed_wing_runway_control
		 * 明确使能时工作；它和空中的方向舵/偏航控制是两套逻辑。
		 */
		const bool runway_control_recent = hrt_elapsed_time(&runway_control.timestamp) < 1_s;
		const bool wheel_controller_enabled = _param_fw_w_en.get() && _vcontrol_mode.flag_control_auto_enabled
						      && runway_control_recent && runway_control.wheel_steering_enabled;

		float groundspeed_scale = 1.f;
		float wheel_u = 0.f;

		if (wheel_controller_enabled) {
			if (_local_pos_sub.updated()) {
				vehicle_local_position_s vehicle_local_position;

				if (_local_pos_sub.copy(&vehicle_local_position)) {
					// 前轮控制需要地速估计，用 local velocity 的水平分量计算。
					_groundspeed = sqrtf(vehicle_local_position.vx * vehicle_local_position.vx + vehicle_local_position.vy *
							     vehicle_local_position.vy);
				}
			}

			// Use stall airspeed to calculate ground speed scaling region. Don't scale below gspd_scaling_trim
			// 地速越高，前轮偏转越敏感；用失速空速作为缩放基准，避免高速滑跑时过度转向。
			float gspd_scaling_trim = (_param_fw_airspd_stall.get());

			if (_groundspeed > gspd_scaling_trim) {
				groundspeed_scale = gspd_scaling_trim / _groundspeed;

			}

			// set now yaw setpoint once we're entering the first time
			// 第一次进入轮转向控制时锁定当前航向，后续控制器围绕这个航向保持直线滑跑。
			if (!PX4_ISFINITE(_steering_wheel_yaw_setpoint)) {
				_steering_wheel_yaw_setpoint = euler_angles.psi();
			}

			_wheel_ctrl.control_attitude(_steering_wheel_yaw_setpoint, euler_angles.psi());

			vehicle_angular_velocity_s angular_velocity{};
			_vehicle_rates_sub.copy(&angular_velocity);

			const float wheel_controller_output = wheel_controller_enabled ? _wheel_ctrl.control_bodyrate(dt,
							      angular_velocity.xyz[2], _groundspeed,
							      groundspeed_scale) : 0.f;

			// 自动前轮控制输出叠加跑道起降逻辑给出的人工/自动微调量。
			wheel_u = wheel_controller_output + runway_control.wheel_steering_nudging_rate;

		} else {
			// 未使能自动轮转向时清积分，并让偏航杆直接控制前轮。
			_wheel_ctrl.reset_integrator();
			_steering_wheel_yaw_setpoint = NAN;
			wheel_u = _manual_control_setpoint.yaw; // direct yaw stick to wheel steering
		}

		_landing_gear_wheel.normalized_wheel_setpoint = PX4_ISFINITE(wheel_u) ? wheel_u : 0.f;
		// 发布归一化前轮转向指令，后续由分配/输出层映射到实际舵机。
		_landing_gear_wheel.timestamp = hrt_absolute_time();
		_landing_gear_wheel_pub.publish(_landing_gear_wheel);
	}

	// backup schedule
	// 兜底调度：即使回调暂时没有触发，也保证最多约 20ms 后再次运行。
	ScheduleDelayed(20_ms);

	perf_end(_loop_perf);
}

int FixedwingAttitudeControl::task_spawn(int argc, char *argv[])
{
	/*
	 * 模块启动入口。
	 * 命令行参数带 "vtol" 时，使用 VTOL 固定翼阶段的话题实例。
	 */
	bool vtol = false;

	if (argc > 1) {
		if (strcmp(argv[1], "vtol") == 0) {
			vtol = true;
		}
	}

	FixedwingAttitudeControl *instance = new FixedwingAttitudeControl(vtol);

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

int FixedwingAttitudeControl::custom_command(int argc, char *argv[])
{
	// 本模块没有自定义运行时命令，未知命令统一打印用法。
	return print_usage("unknown command");
}

int FixedwingAttitudeControl::print_usage(const char *reason)
{
	// px4 命令行帮助信息，供 nsh/mavlink shell 中查看模块用途和参数。
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
fw_att_control is the fixed wing attitude controller.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("fw_att_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int fw_att_control_main(int argc, char *argv[])
{
	return FixedwingAttitudeControl::main(argc, argv);
}
