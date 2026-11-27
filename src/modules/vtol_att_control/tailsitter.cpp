/****************************************************************************
 *
 *   Copyright (c) 2015-2023 PX4 Development Team. All rights reserved.
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
* @file tailsitter.cpp
*
* @author Roman Bapst 		<bapstroman@gmail.com>
* @author David Vorsin     <davidvorsin@gmail.com>
*
*/

#include "tailsitter.h"
#include "vtol_att_control_main.h"

using namespace matrix;

Tailsitter::Tailsitter(VtolAttitudeControl *attc) :
	VtolType(attc)
{
	// attc 是主 VtolAttitudeControl 对象。VtolType 构造函数会通过它的 getter
	// 保存各输入缓存和最终输出消息的地址，因此本类可以直接读写主模块的数据。
}

void
Tailsitter::parameters_update()
{
	// 读取 Tailsitter 自己及 VtolType 公共基类声明的参数。
	// 这里只刷新参数，不执行状态切换。
	VtolType::updateParams();

}

void Tailsitter::update_vtol_state()
{
	/* simple logic using a two way switch to perform transitions.
	 * after flipping the switch the vehicle will start tilting in MC control mode, picking up
	 * forward speed. After the vehicle has picked up enough and sufficient pitch angle the uav will go into FW mode.
	 * For the backtransition the pitch is controlled in MC mode again and switches to full MC control reaching the sufficient pitch angle.
	*/


	/*
	 * 状态机优先级：
	 * 1. 固定翼系统故障/quadchute：最高优先级，立即要求 MC；
	 * 2. 当前请求 MC：保持 MC、开始后转换或中止前转换；
	 * 3. 当前请求 FW：开始/继续前转换或保持 FW。
	 */
	if (_vtol_vehicle_status->fixed_wing_system_failure) {
		// Failsafe event, switch to MC mode immediately
		// 若从非 MC 状态直接触发故障，记录时刻以支持随后推力连续处理。
		if (_vtol_mode != vtol_mode::MC_MODE) {
			_transition_start_timestamp = hrt_absolute_time();
		}

		_vtol_mode = vtol_mode::MC_MODE;

	} else if (!_attc->is_fixed_wing_requested()) {
		// 主模块当前请求旋翼模式（可能来自模式开关、vehicle_command 或 failsafe）。

		switch (_vtol_mode) { // user switchig to MC mode
		case vtol_mode::MC_MODE:
			// 已经处于悬停，不需要改变状态。
			break;

		case vtol_mode::FW_MODE:
			// 从固定翼收到 MC 请求：复位转换计时/高度基准，然后开始后转换。
			resetTransitionStates();
			_vtol_mode = vtol_mode::TRANSITION_BACK;
			break;

		case vtol_mode::TRANSITION_FRONT_P1:
			// failsafe into multicopter mode
			// 前转换尚未完成时取消 FW 请求，可直接回 MC；此时仍由 MC 控制链掌握飞机。
			_vtol_mode = vtol_mode::MC_MODE;
			break;

		case vtol_mode::TRANSITION_BACK:
			// 后转换期间实际姿态仍采用原始尾座式/MC 坐标系欧拉角。
			const float pitch = Eulerf(Quatf(_v_att->q)).theta();

			// check if we have reached pitch angle to switch to MC mode
			// 姿态已经接近竖直，或后转换持续时间达到上限，结束转换并进入纯 MC。
			if (pitch >= PITCH_THRESHOLD_AUTO_TRANSITION_TO_MC || _time_since_trans_start > _param_vt_b_trans_dur.get()) {
				_vtol_mode = vtol_mode::MC_MODE;
			}

			break;
		}

	} else {  // user switchig to FW mode
		// 主模块当前请求固定翼模式。

		switch (_vtol_mode) {
		case vtol_mode::MC_MODE:
			// initialise a front transition
			// 从悬停开始前转换；转换姿态由 MC 姿态/角速度控制器执行。
			_vtol_mode = vtol_mode::TRANSITION_FRONT_P1;
			resetTransitionStates();
			break;

		case vtol_mode::FW_MODE:
			// 已经处于固定翼模式。
			break;

		case vtol_mode::TRANSITION_FRONT_P1: {

				// isFrontTransitionCompleted() 在尾座式中检查姿态和可用空速，
				// 基类还允许未解锁/已着陆时在地面直接完成模式切换。
				if (isFrontTransitionCompleted()) {
					_vtol_mode = vtol_mode::FW_MODE;
					// 记录切换完成时间：TECS 接管等待和最初 50 ms 推力保持都会使用它。
					_trans_finished_ts = hrt_absolute_time();
				}

				break;
			}

		case vtol_mode::TRANSITION_BACK:
			// failsafe into fixed wing mode
			// 后转换期间重新请求 FW：直接终止后转换，回到 FW 控制。
			_vtol_mode = vtol_mode::FW_MODE;
			_trans_finished_ts = hrt_absolute_time();
			break;
		}
	}

	// 把尾座式内部状态映射为通用 VTOL 状态，主循环只依赖这个通用枚举。
	switch (_vtol_mode) {
	case vtol_mode::MC_MODE:
		_common_vtol_mode = mode::ROTARY_WING;
		// 清除标志，保证下次进入转换时重新计算转换初始姿态和旋转轴。
		_flag_was_in_trans_mode = false;
		break;

	case vtol_mode::FW_MODE:
		_common_vtol_mode = mode::FIXED_WING;
		_flag_was_in_trans_mode = false;
		break;

	case vtol_mode::TRANSITION_FRONT_P1:
		_common_vtol_mode = mode::TRANSITION_TO_FW;
		break;

	case vtol_mode::TRANSITION_BACK:
		_common_vtol_mode = mode::TRANSITION_TO_MC;
		break;
	}
}

void Tailsitter::update_transition_state()
{
	// 基类更新转换 dt、已用时间、quadchute 检查，并保存最新 MC 推力。
	VtolType::update_transition_state();

	const hrt_abstime now = hrt_absolute_time();

	// we need the incoming (virtual) mc attitude setpoints to be recent, otherwise return (means the previous setpoint stays active)
	// 转换由 MC 姿态控制器执行，所以必须有新鲜的 MC 虚拟姿态/推力目标。
	// 超过 1 s 没有更新时不构造新目标，主模块继续保留上一目标，避免使用陈旧输入跳变。
	if (_mc_virtual_att_sp->timestamp < (now - 1_s)) {
		return;
	}

	if (!_flag_was_in_trans_mode) {
		// 本次转换只执行一次的初始化。
		_flag_was_in_trans_mode = true;

		if (_vtol_mode == vtol_mode::TRANSITION_BACK) {
			// calculate rotation axis for transition.
			// -dcm_z() 是当前机体推力轴在世界系中的方向；它也是尾座式当前机头/推进方向的几何参考。
			_q_trans_start = Quatf(_v_att->q);
			Vector3f z = -_q_trans_start.dcm_z();
			// 与世界系向上方向叉乘，得到把当前飞行姿态转向竖直悬停姿态的旋转轴。
			_trans_rot_axis = z.cross(Vector3f(0.f, 0.f, -1.f));

			// as heading setpoint we choose the heading given by the direction the vehicle points
			// 后转换保持进入转换时的水平机头方向，避免抬头过程中无意改变航向。
			const float yaw_sp = atan2f(z(1), z(0));

			// the intial attitude setpoint for a backtransition is a combination of the current fw pitch setpoint,
			// the yaw setpoint and zero roll since we want wings level transition.
			// If for some reason the fw attitude setpoint is not recent then don't use it and assume 0 pitch
			if (_fw_virtual_att_sp->timestamp > (now - 1_s)) {
				// 使用最新固定翼目标俯仰，但强制 roll=0，让机翼水平开始后转换。
				const float pitch_body = Eulerf(Quatf(_fw_virtual_att_sp->q_d)).theta();
				_q_trans_start = Eulerf(0.f, pitch_body, yaw_sp);

			} else {
				// 固定翼目标过期时使用零俯仰作为安全退化值。
				_q_trans_start = Eulerf(0.f, 0.f, yaw_sp);
			}

			// attitude during transitions are controlled by mc attitude control so rotate the desired attitude to the
			// multirotor frame
			// 固定翼平飞零姿态与尾座式 MC 悬停零姿态相差绕 Y 轴 -90°。
			// 转换阶段由 MC 姿态控制器执行，因此必须把固定翼语义目标换到 MC 坐标系。
			_q_trans_start = _q_trans_start * Quatf(Eulerf(0, -M_PI_2_F, 0));

		} else if (_vtol_mode == vtol_mode::TRANSITION_FRONT_P1) {
			// initial attitude setpoint for the transition should be with wings level
			// 前转换从当前 MC 目标的 pitch/yaw 开始，但将 roll 清零，避免带坡度进入俯转。
			const Eulerf setpoint_euler(Quatf(_mc_virtual_att_sp->q_d));
			_q_trans_start = Eulerf(0.f, setpoint_euler.theta(), setpoint_euler.psi());
			// 当前机体 X 轴与世界向上方向的叉乘决定前转换俯转轴。
			Vector3f x = Dcmf(Quatf(_v_att->q)) * Vector3f(1.f, 0.f, 0.f);
			_trans_rot_axis = -x.cross(Vector3f(0.f, 0.f, -1.f));
		}

		// 当前转换目标从初始目标开始，后续按时间沿旋转轴推进。
		_q_trans_sp = _q_trans_start;
	}

	// ensure input quaternions are exactly normalized because acosf(1.00001) == NaN
	// 浮点误差可能让理论上位于 [-1,1] 的余弦略越界；归一化并在下方 constrain 防止 acosf NaN。
	_q_trans_sp.normalize();

	// tilt angle (zero if vehicle nose points up (hover))
	// tilt=0 表示尾座式机头竖直向上；tilt≈pi/2 表示接近固定翼平飞。
	const float cos_tilt = math::constrain(_q_trans_sp(0) * _q_trans_sp(0) - _q_trans_sp(1) * _q_trans_sp(1) -
					       _q_trans_sp(2) * _q_trans_sp(2) + _q_trans_sp(3) * _q_trans_sp(3), -1.f, 1.f);
	const float tilt = acosf(cos_tilt);

	if (_vtol_mode == vtol_mode::TRANSITION_FRONT_P1) {

		// calculate pitching rate - and constrain to at least 0.1s transition time
		// 用 VT_F_TRANS_DUR 把完整 90° 前转换变为恒定角速度；0.1 s 下限避免除零/过大角速度。
		const float trans_pitch_rate = M_PI_2_F / math::max(_param_vt_f_trans_dur.get(), 0.1f);

		// FW_PSP_OFF 是固定翼平飞俯仰偏置。达到相应目标倾角后停止继续旋转。
		if (tilt < M_PI_2_F - math::radians(_param_fw_psp_off.get())) {
			_q_trans_sp = Quatf(AxisAnglef(_trans_rot_axis,
						       _time_since_trans_start * trans_pitch_rate)) * _q_trans_start;
		}

	} else if (_vtol_mode == vtol_mode::TRANSITION_BACK) {

		// calculate pitching rate - and constrain to at least 0.1s transition time
		// 后转换同样按 VT_B_TRANS_DUR 以恒定角速度完成约 90° 抬头。
		const float trans_pitch_rate = M_PI_2_F / math::max(_param_vt_b_trans_dur.get(), 0.1f);

		// 距竖直小于约 0.01 rad 后停止积分旋转，避免越过悬停方向。
		if (tilt > 0.01f) {
			_q_trans_sp = Quatf(AxisAnglef(_trans_rot_axis,
						       _time_since_trans_start * trans_pitch_rate)) * _q_trans_start;
		}
	}

	// 转换姿态由本函数生成，但 MC 位置/高度控制器给出的 -body-Z 推力仍被保留。
	_v_att_sp->thrust_body[2] = _mc_virtual_att_sp->thrust_body[2];

	if (_vtol_mode == vtol_mode::TRANSITION_BACK) {
		// 后转换开始的前 0.5 s，将最后固定翼推力平滑混入新的 MC 推力，防止电机指令突变。
		const float progress = math::constrain(_time_since_trans_start / B_TRANS_THRUST_BLENDING_DURATION, 0.f, 1.f);
		blendThrottleBeginningBackTransition(progress);
	}

	_v_att_sp->timestamp = hrt_absolute_time();

	// 保留欧拉角构造用于调试/可读性；真正发布的是归一化四元数 q_d。
	const Eulerf euler_sp(_q_trans_sp);
	_q_trans_sp.copyTo(_v_att_sp->q_d);
}

void Tailsitter::waiting_on_tecs()
{
	// copy the last trust value from the front transition
	// 刚切到 FW 时 TECS 可能尚未发布有效 +body-X 油门。
	// 将最后 MC -body-Z 推力取反后写入 FW +body-X 通道，保持推进连续。
	_v_att_sp->thrust_body[0] = -_last_thr_in_mc;
}

void Tailsitter::update_fw_state()
{
	// 尾座式在稳定 FW 阶段没有额外姿态处理，复用基类：
	// 复制 FW 虚拟姿态目标、等待 TECS、混合接管油门并检查 quadchute。
	VtolType::update_fw_state();

}

/**
* Write data to actuator output topic.
*/
void Tailsitter::fill_actuator_outputs()
{
	/*
	 * 这里的 _torque/_thrust_setpoint_0/1 都是指针，指向主模块
	 * VtolAttitudeControl 持有的四个最终消息对象。函数返回后，主模块会发布同一块内存。
	 *
	 * instance 0：主要送往电机控制分配矩阵；
	 * instance 1：主要送往气动舵面控制分配矩阵。
	 *
	 * 每周期先全部清零，再根据当前状态填写，避免上一状态的某个轴残留。
	 */
	_torque_setpoint_0->timestamp = hrt_absolute_time();
	_torque_setpoint_0->timestamp_sample = _vehicle_torque_setpoint_virtual_mc->timestamp_sample;
	_torque_setpoint_0->xyz[0] = 0.f;
	_torque_setpoint_0->xyz[1] = 0.f;
	_torque_setpoint_0->xyz[2] = 0.f;

	_torque_setpoint_1->timestamp = hrt_absolute_time();
	_torque_setpoint_1->timestamp_sample = _vehicle_torque_setpoint_virtual_fw->timestamp_sample;
	_torque_setpoint_1->xyz[0] = 0.f;
	_torque_setpoint_1->xyz[1] = 0.f;
	_torque_setpoint_1->xyz[2] = 0.f;

	_thrust_setpoint_0->timestamp = hrt_absolute_time();
	_thrust_setpoint_0->timestamp_sample = _vehicle_thrust_setpoint_virtual_mc->timestamp_sample;
	_thrust_setpoint_0->xyz[0] = 0.f;
	_thrust_setpoint_0->xyz[1] = 0.f;
	_thrust_setpoint_0->xyz[2] = 0.f;

	_thrust_setpoint_1->timestamp = hrt_absolute_time();
	_thrust_setpoint_1->timestamp_sample = _vehicle_thrust_setpoint_virtual_fw->timestamp_sample;
	_thrust_setpoint_1->xyz[0] = 0.f;
	_thrust_setpoint_1->xyz[1] = 0.f;
	_thrust_setpoint_1->xyz[2] = 0.f;

	// ---------------- instance 0：电机 ----------------
	if (_vtol_mode == vtol_mode::FW_MODE) {

		// 固定翼控制器的推进力沿 +body-X；尾座式同一组电机在 MC/分配器接口中使用 -body-Z。
		// 因此需要 X -> Z 并反号：FW +X 正油门对应输出消息中的负 Z 推力。
		_thrust_setpoint_0->xyz[2] = -_vehicle_thrust_setpoint_virtual_fw->xyz[0];

		/* allow differential thrust if enabled */
		// 若启用固定翼差动推力，将 FW 三轴力矩按比例送到电机控制实例。
		// 实际左右电机如何产生这些力矩由 control_allocator 的 effectiveness matrix 决定。
		if (_param_vt_fw_difthr_en.get() & static_cast<int32_t>(VtFwDifthrEnBits::YAW_BIT)) {
			// 尾座式 MC/FW 坐标语义不同，FW roll 力矩映射到电机实例 X 通道，比例参数沿用 YAW 命名。
			_torque_setpoint_0->xyz[0] = _vehicle_torque_setpoint_virtual_fw->xyz[0] * _param_vt_fw_difthr_s_y.get();
		}

		if (_param_vt_fw_difthr_en.get() & static_cast<int32_t>(VtFwDifthrEnBits::PITCH_BIT)) {
			_torque_setpoint_0->xyz[1] = _vehicle_torque_setpoint_virtual_fw->xyz[1] * _param_vt_fw_difthr_s_p.get();
		}

		if (_param_vt_fw_difthr_en.get() & static_cast<int32_t>(VtFwDifthrEnBits::ROLL_BIT)) {
			_torque_setpoint_0->xyz[2] = _vehicle_torque_setpoint_virtual_fw->xyz[2] * _param_vt_fw_difthr_s_r.get();
		}

		// for the short period after switching to FW where there is no thrust published yet from the FW controller,
		// keep publishing the last MC thrust to keep the motors running
		if (hrt_elapsed_time(&_trans_finished_ts) < 50_ms) {
			// FW rate/TECS 消息链刚接管时可能有一个调度空窗，短暂沿用最后 MC 推力且不施加差动力矩。
			_thrust_setpoint_0->xyz[2] = _last_thr_in_mc;
			_torque_setpoint_0->xyz[0] = 0.f;
			_torque_setpoint_0->xyz[1] = 0.f;
			_torque_setpoint_0->xyz[2] = 0.f;
		}

	} else {
		// MC、前转换、后转换都由 MC rate controller 控制电机，直接采用其 -body-Z 推力。
		_thrust_setpoint_0->xyz[2] = _vehicle_thrust_setpoint_virtual_mc->xyz[2];

		// for the short period after starting the backtransition where there is no thrust published yet from the MC controller,
		// keep publishing the last FW thrust to keep the motors running
		if (_vtol_mode != vtol_mode::TRANSITION_FRONT_P1 && hrt_elapsed_time(&_transition_start_timestamp) < 50_ms) {
			// 后转换刚开始时 MC 输出可能尚未更新，将最后 FW +X 推力反号后临时放到 -Z 通道。
			_thrust_setpoint_0->xyz[2] = -_last_thr_in_fw_mode;
		}

		// 转换阶段姿态由 MC 控制器跟踪，所以三轴电机力矩直接来自 MC rate controller。
		_torque_setpoint_0->xyz[0] = _vehicle_torque_setpoint_virtual_mc->xyz[0];
		_torque_setpoint_0->xyz[1] = _vehicle_torque_setpoint_virtual_mc->xyz[1];
		_torque_setpoint_0->xyz[2] = _vehicle_torque_setpoint_virtual_mc->xyz[2];
	}

	// ---------------- instance 1：气动舵面 ----------------
	// VT_ELEV_MC_LOCK=true 且处于纯 MC 悬停时锁住舵面；转换和 FW 状态始终允许舵面响应。
	if (!_param_vt_elev_mc_lock.get() || _vtol_mode != vtol_mode::MC_MODE) {
		// 舵面使用固定翼 rate controller 产生的三轴虚拟力矩；具体 elevon/servo 混控由 control_allocator 完成。
		_torque_setpoint_1->xyz[0] = _vehicle_torque_setpoint_virtual_fw->xyz[0];
		_torque_setpoint_1->xyz[1] = _vehicle_torque_setpoint_virtual_fw->xyz[1];
		_torque_setpoint_1->xyz[2] = _vehicle_torque_setpoint_virtual_fw->xyz[2];
	}
}


bool Tailsitter::isFrontTransitionCompletedBase()
{
	// 只有“真实空速传感器被允许且数据有效”时，才把空速作为转换完成门限。
	const bool airspeed_triggers_transition = PX4_ISFINITE(_attc->get_calibrated_airspeed());

	bool transition_to_fw = false;
	const float pitch = Eulerf(Quatf(_v_att->q)).theta();

	// 首先要求机体至少俯转到 -60°。有空速时还要求达到 VT_ARSP_TRANS；
	// 无可用空速时仅用姿态条件，避免因缺少空速计永远无法完成转换。
	if (pitch <= PITCH_THRESHOLD_AUTO_TRANSITION_TO_FW) {
		if (airspeed_triggers_transition) {
			transition_to_fw = _attc->get_calibrated_airspeed() >= _param_vt_arsp_trans.get() ;

		} else {
			transition_to_fw = true;
		}
	}

	return transition_to_fw;
}

void Tailsitter::blendThrottleAfterFrontTransition(float scale)
{
	// note: MC throttle is negative (as in negative z), while FW throttle is positive (positive x)
	// scale=0：完全沿用最后 MC 推力；scale=1：完全采用当前 TECS/FW 推力。
	// -_last_thr_in_mc 用于把 MC 负 Z 符号转换为 FW 正 X 油门符号。
	_v_att_sp->thrust_body[0] = scale * _v_att_sp->thrust_body[0] + (1.f - scale) * (-_last_thr_in_mc);
}

void Tailsitter::blendThrottleBeginningBackTransition(float scale)
{
	// scale=0：将最后 FW 正 X 推力转换为 MC 负 Z 推力；
	// scale=1：完全采用 MC 位置/高度控制器当前给出的负 Z 推力。
	_v_att_sp->thrust_body[2] = scale * _v_att_sp->thrust_body[2] + (1.f - scale) * (-_last_thr_in_fw_mode);
}
