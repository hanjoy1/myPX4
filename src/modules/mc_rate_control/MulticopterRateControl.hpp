/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
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

#include <lib/rate_control/rate_control.hpp>
#include <lib/mathlib/math/filter/AlphaFilter.hpp>
#include <lib/matrix/matrix/math.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/systemlib/mavlink_log.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/actuator_controls_status.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/control_allocator_status.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/rate_ctrl_status.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>

using namespace time_literals;

/**
 * 多旋翼角速度控制模块。
 *
 * 中文阅读提示：
 * - 本模块位于姿态控制之后、控制分配之前。
 * - 输入通常是 vehicle_rates_setpoint，也就是 mc_att_control 输出的机体系 roll/pitch/yaw rate 设定值。
 * - 另一个输入是 vehicle_angular_velocity，即陀螺仪/估计器给出的当前机体系角速度。
 * - 输出是 vehicle_torque_setpoint 和 vehicle_thrust_setpoint，交给 control allocator 分配到电机。
 *
 * 典型控制链路：
 * mc_att_control -> vehicle_rates_setpoint -> mc_rate_control
 * mc_rate_control -> vehicle_torque_setpoint / vehicle_thrust_setpoint -> control_allocator -> actuators
 */
class MulticopterRateControl : public ModuleBase<MulticopterRateControl>, public ModuleParams, public px4::WorkItem
{
public:
	MulticopterRateControl(bool vtol = false);
	~MulticopterRateControl() override;

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

	/** 初始化角速度订阅 callback，成功后由 vehicle_angular_velocity 更新触发 Run()。 */
	bool init();

private:
	/** 角速度控制主循环：读取当前角速度和 rate setpoint，运行 PID，发布 torque/thrust setpoint。 */
	void Run() override;

	/**
	 * initialize some vectors/matrices from parameters
	 * 根据参数刷新 PID 增益、积分限幅、前馈增益、Acro 最大角速度和 yaw torque 低通滤波器。
	 */
	void parameters_updated();

	/**
	 * 统计控制输出能量并周期性发布 actuator_controls_status。
	 * @param vehicle_torque_setpoint 当前发布的三轴 torque setpoint
	 * @param dt 控制周期 [s]
	 */
	void updateActuatorControlsStatus(const vehicle_torque_setpoint_s &vehicle_torque_setpoint, float dt);

	RateControl _rate_control; ///< 公共角速度 PID 控制器：rates/rates_sp/angular_accel -> torque setpoint

	uORB::Subscription _battery_status_sub{ORB_ID(battery_status)}; ///< 电池缩放系数输入，用于低电压补偿 torque/thrust setpoint
	uORB::Subscription _control_allocator_status_sub{ORB_ID(control_allocator_status)}; ///< 控制分配反馈，用于获取力矩饱和方向并做积分抗饱和
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)}; ///< Acro 模式下直接由摇杆生成 rate setpoint
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)}; ///< 当前控制模式标志
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)}; ///< 落地/可能落地状态，用于限制积分
	uORB::Subscription _vehicle_rates_setpoint_sub{ORB_ID(vehicle_rates_setpoint)}; ///< 上游姿态控制器或其他模块给出的角速度设定值
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)}; ///< 车辆状态，判断 vehicle_type 等

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s}; ///< 参数更新订阅，限频 1 秒

	uORB::SubscriptionCallbackWorkItem _vehicle_angular_velocity_sub{this, ORB_ID(vehicle_angular_velocity)}; ///< 当前机体系角速度，是 Run() 的主要触发源

	uORB::Publication<actuator_controls_status_s>	_actuator_controls_status_pub{ORB_ID(actuator_controls_status_0)}; ///< 发布控制输出功率/能量统计
	uORB::PublicationMulti<rate_ctrl_status_s>	_controller_status_pub{ORB_ID(rate_ctrl_status)}; ///< 发布 PID 积分项等状态，便于日志/调参
	uORB::Publication<vehicle_rates_setpoint_s>	_vehicle_rates_setpoint_pub{ORB_ID(vehicle_rates_setpoint)}; ///< Acro 模式下发布由摇杆生成的 rate setpoint
	uORB::Publication<vehicle_thrust_setpoint_s>	_vehicle_thrust_setpoint_pub; ///< 发布推力设定值给 control allocator
	uORB::Publication<vehicle_torque_setpoint_s>	_vehicle_torque_setpoint_pub; ///< 发布力矩设定值给 control allocator

	vehicle_control_mode_s	_vehicle_control_mode{}; ///< 当前控制模式
	vehicle_status_s	_vehicle_status{}; ///< 当前车辆状态

	bool _landed{true}; ///< 是否已落地
	bool _maybe_landed{true}; ///< 是否可能落地；可能落地时积分也应保守处理

	hrt_abstime _last_run{0}; ///< 上一次角速度控制运行的 angular_velocity sample 时间戳

	perf_counter_t	_loop_perf;			/**< 控制循环耗时性能计数器 */

	// keep setpoint values between updates
	// 在 setpoint topic 没有新数据时，保留上一轮设定值。
	matrix::Vector3f _acro_rate_max;		/**< Acro 模式下 roll/pitch/yaw 最大角速度 [rad/s] */
	matrix::Vector3f _rates_setpoint{}; ///< 当前机体系角速度设定值 [rad/s]

	float _battery_status_scale{0.0f}; ///< 电池电压补偿缩放系数，>0 时用于缩放 torque/thrust setpoint
	matrix::Vector3f _thrust_setpoint{}; ///< 当前机体系推力设定值，通常来自 vehicle_rates_setpoint

	float _energy_integration_time{0.0f}; ///< 控制能量积分窗口累计时间
	float _control_energy[4] {}; ///< 三轴 torque setpoint 平方积分，用于 actuator_controls_status

	AlphaFilter<float> _output_lpf_yaw; ///< yaw torque 输出低通滤波器，用于降低高频 yaw 力矩

	DEFINE_PARAMETERS(
		// roll rate PID 参数
		(ParamFloat<px4::params::MC_ROLLRATE_P>) _param_mc_rollrate_p,
		(ParamFloat<px4::params::MC_ROLLRATE_I>) _param_mc_rollrate_i,
		(ParamFloat<px4::params::MC_RR_INT_LIM>) _param_mc_rr_int_lim,
		(ParamFloat<px4::params::MC_ROLLRATE_D>) _param_mc_rollrate_d,
		(ParamFloat<px4::params::MC_ROLLRATE_FF>) _param_mc_rollrate_ff,
		(ParamFloat<px4::params::MC_ROLLRATE_K>) _param_mc_rollrate_k,

		// pitch rate PID 参数
		(ParamFloat<px4::params::MC_PITCHRATE_P>) _param_mc_pitchrate_p,
		(ParamFloat<px4::params::MC_PITCHRATE_I>) _param_mc_pitchrate_i,
		(ParamFloat<px4::params::MC_PR_INT_LIM>) _param_mc_pr_int_lim,
		(ParamFloat<px4::params::MC_PITCHRATE_D>) _param_mc_pitchrate_d,
		(ParamFloat<px4::params::MC_PITCHRATE_FF>) _param_mc_pitchrate_ff,
		(ParamFloat<px4::params::MC_PITCHRATE_K>) _param_mc_pitchrate_k,

		// yaw rate PID 参数和 yaw torque 输出滤波
		(ParamFloat<px4::params::MC_YAWRATE_P>) _param_mc_yawrate_p,
		(ParamFloat<px4::params::MC_YAWRATE_I>) _param_mc_yawrate_i,
		(ParamFloat<px4::params::MC_YR_INT_LIM>) _param_mc_yr_int_lim,
		(ParamFloat<px4::params::MC_YAWRATE_D>) _param_mc_yawrate_d,
		(ParamFloat<px4::params::MC_YAWRATE_FF>) _param_mc_yawrate_ff,
		(ParamFloat<px4::params::MC_YAWRATE_K>) _param_mc_yawrate_k,
		(ParamFloat<px4::params::MC_YAW_TQ_CUTOFF>) _param_mc_yaw_tq_cutoff,

		// Acro 模式手动摇杆到角速度设定值的映射参数
		(ParamFloat<px4::params::MC_ACRO_R_MAX>) _param_mc_acro_r_max,
		(ParamFloat<px4::params::MC_ACRO_P_MAX>) _param_mc_acro_p_max,
		(ParamFloat<px4::params::MC_ACRO_Y_MAX>) _param_mc_acro_y_max,
		(ParamFloat<px4::params::MC_ACRO_EXPO>) _param_mc_acro_expo,			/**< roll/pitch expo 摇杆曲线形状 */
		(ParamFloat<px4::params::MC_ACRO_EXPO_Y>) _param_mc_acro_expo_y,				/**< yaw expo 摇杆曲线形状 */
		(ParamFloat<px4::params::MC_ACRO_SUPEXPO>) _param_mc_acro_supexpo,		/**< roll/pitch superexpo 摇杆曲线形状 */
		(ParamFloat<px4::params::MC_ACRO_SUPEXPOY>) _param_mc_acro_supexpoy,		/**< yaw superexpo 摇杆曲线形状 */

		// 电池电压补偿开关
		(ParamBool<px4::params::MC_BAT_SCALE_EN>) _param_mc_bat_scale_en
	)
};
