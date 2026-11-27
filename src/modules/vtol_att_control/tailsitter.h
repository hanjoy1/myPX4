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
* @file tailsitter.h
*
* @author Roman Bapst 		<bapstroman@gmail.com>
* @author David Vorsin     <davidvorsin@gmail.com>
*
*/

#ifndef TAILSITTER_H
#define TAILSITTER_H

#include "vtol_type.h"

#include <parameters/param.h>
#include <drivers/drv_hrt.h>
#include <matrix/matrix/math.hpp>

// 尾座式自动前转换完成的俯仰角门限 [rad]。
// PX4 姿态约定下，悬停时尾座式 pitch 接近 0；机头向前倒、接近平飞时 pitch 向负方向变化。
// 达到 -60° 只表示姿态条件满足；有有效空速时还必须同时满足 VT_ARSP_TRANS。
static constexpr float PITCH_THRESHOLD_AUTO_TRANSITION_TO_FW = -1.05f; // -60°

// 自动后转换完成的俯仰角门限 [rad]。
// 后转换过程中 pitch 从接近平飞的负角度回到 0；达到 -15° 后可切回纯 MC 控制。
static constexpr float PITCH_THRESHOLD_AUTO_TRANSITION_TO_MC = -0.26f; // -15°

// 后转换刚开始时，固定翼 +X 推力向多旋翼 -Z 推力平滑切换的时间 [s]。
static constexpr float B_TRANS_THRUST_BLENDING_DURATION = 0.5f;

/**
 * 尾座式 VTOL 类型实现。
 *
 * 类本身不重新实现 MC/FW 姿态与角速度控制器，而是负责：
 * 1. 管理 MC、前转换、FW、后转换四个状态；
 * 2. 在转换期间生成约 90° 的目标姿态轨迹；
 * 3. 选择 MC/FW 虚拟控制器输出并进行尾座式坐标轴映射；
 * 4. 把结果写入 VtolAttitudeControl 持有的最终 torque/thrust 消息。
 */
class Tailsitter : public VtolType
{

public:
	Tailsitter(VtolAttitudeControl *_att_controller);
	~Tailsitter() override = default;

	void update_vtol_state() override;
	void update_transition_state() override;
	void update_fw_state() override;
	void fill_actuator_outputs() override;
	void waiting_on_tecs() override;
	void blendThrottleAfterFrontTransition(float scale) override;
	void blendThrottleBeginningBackTransition(float scale);

private:
	enum class vtol_mode {
		MC_MODE = 0,			/**< 机头朝上悬停，使用 MC 控制链 */
		TRANSITION_FRONT_P1,	/**< 前转换：从竖直悬停姿态俯转到固定翼姿态 */
		TRANSITION_BACK,		/**< 后转换：从固定翼姿态抬头回到竖直悬停姿态 */
		FW_MODE					/**< 固定翼平飞，使用 FW 控制链 */
	};

	// 尾座式专用细分状态；随后会映射为所有 VTOL 共用的 mode 枚举，供主模块调度。
	vtol_mode _vtol_mode{vtol_mode::MC_MODE};

	// false 表示刚进入一次转换，update_transition_state() 需要初始化起始姿态和旋转轴。
	bool _flag_was_in_trans_mode = false;

	matrix::Quatf _q_trans_start;       // 本次前/后转换开始时的目标姿态
	matrix::Quatf _q_trans_sp;          // 随转换时间推进的当前目标姿态
	matrix::Vector3f _trans_rot_axis;   // 世界/机体几何计算得到的转换旋转轴

	void parameters_update() override;

	// 尾座式使用“实际俯仰角 + 可选空速”判断前转换是否结束。
	bool isFrontTransitionCompletedBase() override;

	DEFINE_PARAMETERS_CUSTOM_PARENT(VtolType,
					(ParamFloat<px4::params::FW_PSP_OFF>) _param_fw_psp_off
				       )


};
#endif
