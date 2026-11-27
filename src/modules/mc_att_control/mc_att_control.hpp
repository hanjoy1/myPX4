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

#pragma once

#include <matrix/matrix/math.hpp>
#include <perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/autotune_attitude_control_status.h>
#include <uORB/topics/hover_thrust_estimate.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <lib/mathlib/math/filter/AlphaFilter.hpp>
#include <lib/slew_rate/SlewRate.hpp>
#include <lib/stick_yaw/StickYaw.hpp>

#include <AttitudeControl.hpp>

using namespace time_literals;

/**
 * 多旋翼姿态控制模块。
 *
 * 中文阅读提示：
 * - 本模块位于位置控制之后、角速度控制之前。
 * - 输入主要是 vehicle_attitude 当前姿态和 vehicle_attitude_setpoint 期望姿态/推力。
 * - 输出是 vehicle_rates_setpoint，也就是机体系 roll/pitch/yaw 角速度设定值和推力。
 * - Manual/Stabilized 且没有高度/速度/位置控制时，本模块会根据摇杆直接生成 attitude_setpoint。
 *
 * 典型控制链路：
 * mc_pos_control -> vehicle_attitude_setpoint -> mc_att_control
 * mc_att_control -> vehicle_rates_setpoint -> mc_rate_control
 */
class MulticopterAttitudeControl : public ModuleBase<MulticopterAttitudeControl>, public ModuleParams,
	public px4::WorkItem
{
public:
	MulticopterAttitudeControl(bool vtol = false);
	~MulticopterAttitudeControl() override;

	/** @see ModuleBase
	 * 中文：模块启动入口，解析命令行参数并创建 work queue 实例。
	 */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase
	 * 中文：处理自定义命令；本模块没有额外命令时打印 usage。
	 */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase
	 * 中文：打印模块说明和启动参数。
	 */
	static int print_usage(const char *reason = nullptr);

	/** 初始化姿态订阅 callback，成功后由 vehicle_attitude 更新触发 Run()。 */
	bool init();

private:
	/** 姿态控制主循环：读取姿态和 setpoint，计算角速度设定值并发布。 */
	void Run() override;

	/**
	 * initialize some vectors/matrices from parameters
	 * 根据参数初始化姿态控制器内部增益、角速度限制和手动倾角限制等缓存。
	 */
	void parameters_updated();

	/**
	 * 将手动油门摇杆输入映射为归一化推力。
	 * @param throttle_stick_input 手动油门摇杆输入，范围 [-1, 1]
	 * @return 映射后的归一化推力，范围受最小/最大手动油门和 MPC_THR_MAX 限制
	 */
	float throttle_curve(float throttle_stick_input);

	/**
	 * Generate & publish an attitude setpoint from stick inputs
	 * 根据手动摇杆输入生成并发布姿态设定值，主要用于 Manual/Stabilized 模式。
	 * @param q 当前姿态四元数
	 * @param dt 控制周期 [s]
	 */
	void generate_attitude_setpoint(const matrix::Quatf &q, float dt);

	AttitudeControl _attitude_control; /**< 姿态控制算法类：期望姿态 + 当前姿态 -> 角速度设定值 */
	StickYaw _stick_yaw{this}; ///< 手动 yaw 摇杆处理器，负责 yaw 锁定和 yaw rate 设定值生成

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s}; ///< 参数更新订阅，限频 1 秒

	uORB::Subscription _hover_thrust_estimate_sub{ORB_ID(hover_thrust_estimate)}; ///< 悬停推力估计，用于手动油门曲线缩放
	uORB::Subscription _vehicle_attitude_setpoint_sub{ORB_ID(vehicle_attitude_setpoint)}; ///< 期望姿态/推力输入，通常来自位置控制或手动模式生成
	uORB::Subscription _autotune_attitude_control_status_sub{ORB_ID(autotune_attitude_control_status)}; ///< 姿态自整定附加角速度激励输入
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)}; ///< 手动摇杆输入
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)}; ///< 当前控制模式
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)}; ///< 落地检测状态
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)}; ///< 本地位置，主要读取 heading_good_for_control 等航向质量信息
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)}; ///< 车辆状态，包含 armed、VTOL 状态、机型等

	uORB::SubscriptionCallbackWorkItem _vehicle_attitude_sub{this, ORB_ID(vehicle_attitude)}; ///< 当前姿态订阅，是本模块 Run() 的主要触发源

	uORB::Publication<vehicle_rates_setpoint_s>     _vehicle_rates_setpoint_pub{ORB_ID(vehicle_rates_setpoint)};    /**< 发布给角速度控制器的角速度/推力设定值 */
	uORB::Publication<vehicle_attitude_setpoint_s>  _vehicle_attitude_setpoint_pub; ///< 手动/Stabilized 模式下由摇杆生成的姿态设定值输出

	manual_control_setpoint_s       _manual_control_setpoint {};    /**< 最新手动摇杆输入 */
	vehicle_control_mode_s          _vehicle_control_mode {};       /**< 最新控制模式标志 */

	perf_counter_t  _loop_perf;             /**< 控制循环耗时性能计数器 */

	matrix::Vector3f _thrust_setpoint_body; /**< 机体系三维推力设定值，随 vehicle_rates_setpoint 一起发布 */

	float _hover_thrust_estimate{NAN}; ///< 当前悬停推力估计；NaN 表示没有有效估计，使用参数 MPC_THR_HOVER
	SlewRate<float> _hover_thrust_slew_rate{.5f}; ///< 手动油门曲线使用的悬停推力斜率限制，避免 HTE 变化直接造成油门曲线跳变

	float _yaw_setpoint_stabilized{0.f}; ///< Stabilized 手动模式下锁定/积分出来的 yaw 设定值，NaN 表示 yaw 解锁
	bool _heading_good_for_control{true}; // 初始化为 true：即使 local position 从未发布，也允许 heading lock
	float _unaided_heading{NAN}; // 初始化为 NaN：local position 从未发布时不干扰 heading lock
	float _man_tilt_max{0.f};			/**< 手动模式允许的最大倾角 [rad] */

	SlewRate<float> _manual_throttle_minimum{0.f}; ///< 落地时为 0，空中缓慢爬升到 MPC_MANTHR_MIN，避免手动最小油门突变
	SlewRate<float> _manual_throttle_maximum{0.f}; ///< 未完成电机预转时为 0，spooled up 后缓慢爬升到 1，防止预转阶段起飞
	AlphaFilter<float> _man_roll_input_filter; ///< 手动 roll 输入低通滤波器，用 MC_MAN_TILT_TAU 设置时间常数
	AlphaFilter<float> _man_pitch_input_filter; ///< 手动 pitch 输入低通滤波器

	hrt_abstime _last_run{0}; ///< 上一次姿态控制运行的姿态 sample 时间戳
	hrt_abstime _last_attitude_setpoint{0}; ///< 最近一次接受的 attitude setpoint 时间戳

	bool _spooled_up{false}; ///< 电机是否已完成预转；用于确保 spoolup 时间内无法起飞
	bool _landed{true}; ///< 当前是否落地
	bool _vehicle_type_rotary_wing{true}; ///< 当前车辆类型是否为旋翼
	bool _vtol{false}; ///< 当前是否为 VTOL
	bool _vtol_tailsitter{false}; ///< 当前 VTOL 是否为 tailsitter
	bool _vtol_in_transition_mode{false}; ///< VTOL 是否处于固定翼/多旋翼转换阶段

	uint8_t _quat_reset_counter{0}; ///< 姿态估计 quaternion reset 计数器，用于检测 EKF 航向/姿态 reset

	DEFINE_PARAMETERS(
		// 姿态控制相关参数：airmode、手动倾角滤波、姿态 P 增益、yaw 权重和角速度限制
		(ParamInt<px4::params::MC_AIRMODE>)         _param_mc_airmode,
		(ParamFloat<px4::params::MC_MAN_TILT_TAU>)  _param_mc_man_tilt_tau,

		(ParamFloat<px4::params::MC_ROLL_P>)        _param_mc_roll_p,
		(ParamFloat<px4::params::MC_PITCH_P>)       _param_mc_pitch_p,
		(ParamFloat<px4::params::MC_YAW_P>)         _param_mc_yaw_p,
		(ParamFloat<px4::params::MC_YAW_WEIGHT>)    _param_mc_yaw_weight,

		(ParamFloat<px4::params::MC_ROLLRATE_MAX>)  _param_mc_rollrate_max,
		(ParamFloat<px4::params::MC_PITCHRATE_MAX>) _param_mc_pitchrate_max,
		(ParamFloat<px4::params::MC_YAWRATE_MAX>)   _param_mc_yawrate_max,

		/* Stabilized mode params */
		// Stabilized/Manual 模式参数：摇杆死区、最大手动倾角、手动油门曲线和电机预转时间
		(ParamFloat<px4::params::MAN_DEADZONE>) _param_man_deadzone,
		(ParamFloat<px4::params::MPC_MAN_TILT_MAX>) _param_mpc_man_tilt_max,
		(ParamFloat<px4::params::MPC_MANTHR_MIN>) _param_mpc_manthr_min,
		(ParamFloat<px4::params::MPC_THR_MAX>) _param_mpc_thr_max,
		(ParamFloat<px4::params::MPC_THR_HOVER>) _param_mpc_thr_hover,
		(ParamInt<px4::params::MPC_THR_CURVE>) _param_mpc_thr_curve,

		(ParamFloat<px4::params::COM_SPOOLUP_TIME>) _param_com_spoolup_time
	)
};
