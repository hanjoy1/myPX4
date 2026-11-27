/****************************************************************************
 *
 *   Copyright (c) 2013-2020 PX4 Development Team. All rights reserved.
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
 * Multicopter position controller.
 *
 * 中文阅读提示：
 * 这是 mc_pos_control 模块入口，运行在 work queue 上，由 vehicle_local_position 更新触发。
 * 它负责订阅飞行模式、轨迹设定值、约束、落地状态等 uORB topic，然后调用 PositionControl
 * 生成 vehicle_attitude_setpoint。真正的 PID 控制逻辑在 PositionControl/PositionControl.cpp。
 */

#pragma once

#include "PositionControl/PositionControl.hpp"
#include "Takeoff/Takeoff.hpp"
#include "GotoControl/GotoControl.hpp"

#include <drivers/drv_hrt.h>
#include <lib/mathlib/math/filter/AlphaFilter.hpp>
#include <lib/mathlib/math/filter/NotchFilter.hpp>
#include <lib/mathlib/math/WelfordMean.hpp>
#include <lib/perf/perf_counter.h>
#include <lib/slew_rate/SlewRateYaw.hpp>
#include <lib/systemlib/mavlink_log.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/hover_thrust_estimate.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_constraints.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>

using namespace time_literals;

class MulticopterPositionControl : public ModuleBase<MulticopterPositionControl>, public ModuleParams,
	public px4::ScheduledWorkItem
{
public:
	MulticopterPositionControl(bool vtol = false);
	~MulticopterPositionControl() override;

	/** @see ModuleBase
	 * 中文：模块启动入口，由 px4 task/module 框架调用；解析参数并创建控制器实例。
	 */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase
	 * 中文：处理模块自定义命令；本模块没有额外命令时会打印 usage。
	 */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase
	 * 中文：打印模块说明和命令行使用方法。
	 */
	static int print_usage(const char *reason = nullptr);

	/** 初始化订阅回调并启动第一次调度。 */
	bool init();

private:
	/** work queue 主循环：读取状态和设定值，执行位置控制，发布姿态/位置设定值。 */
	void Run() override;

	TakeoffHandling _takeoff; /**< 起飞状态机和爬升速度斜坡，避免解锁/离地时推力突变 */

	orb_advert_t _mavlink_log_pub{nullptr};

	uORB::PublicationData<takeoff_status_s>              _takeoff_status_pub{ORB_ID(takeoff_status)}; ///< 发布起飞状态和当前倾角限制
	uORB::Publication<vehicle_attitude_setpoint_s>	     _vehicle_attitude_setpoint_pub{ORB_ID(vehicle_attitude_setpoint)}; /**< 发布给姿态控制器的姿态/推力设定值 */
	uORB::Publication<vehicle_local_position_setpoint_s> _local_pos_sp_pub{ORB_ID(vehicle_local_position_setpoint)};	/**< 发布内部位置控制设定值，供日志和落地检测等模块使用 */

	uORB::SubscriptionCallbackWorkItem _local_pos_sub{this, ORB_ID(vehicle_local_position)};	/**< 本地位置更新回调，是本模块主循环触发源 */

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s}; ///< 参数更新订阅，限频为 1 秒

	uORB::Subscription _hover_thrust_estimate_sub{ORB_ID(hover_thrust_estimate)}; ///< 悬停推力估计输入
	uORB::Subscription _trajectory_setpoint_sub{ORB_ID(trajectory_setpoint)}; ///< 位置控制要跟踪的轨迹设定值输入
	uORB::Subscription _vehicle_constraints_sub{ORB_ID(vehicle_constraints)}; ///< 起飞/速度等车辆约束输入
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)}; ///< 当前控制模式输入
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)}; ///< 落地检测状态输入

	hrt_abstime _time_stamp_last_loop{0};		/**< 上一次控制循环的时间戳 */
	hrt_abstime _time_position_control_enabled{0}; ///< 最近一次位置控制被启用的时间戳

	trajectory_setpoint_s _setpoint{PositionControl::empty_trajectory_setpoint}; /**< 当前输入轨迹设定值，NaN 表示对应轴不控制 */
	trajectory_setpoint_s _last_valid_setpoint{PositionControl::empty_trajectory_setpoint}; /**< 最近一次可执行设定值，用于短时输入异常回退 */
	vehicle_control_mode_s _vehicle_control_mode{}; /**< 当前控制模式标志，如位置控制/offboard/armed */

	vehicle_constraints_s _vehicle_constraints { ///< 当前车辆约束，默认速度约束为 NaN 表示未指定
		.timestamp = 0,
		.speed_up = NAN,
		.speed_down = NAN,
		.want_takeoff = false,
	};

	vehicle_land_detected_s _vehicle_land_detected { ///< 当前落地检测状态，初始化为已落地的保守状态
		.timestamp = 0,
		.freefall = false,
		.ground_contact = true,
		.maybe_landed = true,
		.landed = true,
	};

	DEFINE_PARAMETERS(
		// Position Control
		// 位置/速度串级控制增益、速度限制、倾角限制、悬停推力和速度滤波参数
		(ParamFloat<px4::params::MPC_XY_P>)         _param_mpc_xy_p,
		(ParamFloat<px4::params::MPC_Z_P>)          _param_mpc_z_p,
		(ParamFloat<px4::params::MPC_XY_VEL_P_ACC>) _param_mpc_xy_vel_p_acc,
		(ParamFloat<px4::params::MPC_XY_VEL_I_ACC>) _param_mpc_xy_vel_i_acc,
		(ParamFloat<px4::params::MPC_XY_VEL_D_ACC>) _param_mpc_xy_vel_d_acc,
		(ParamFloat<px4::params::MPC_Z_VEL_P_ACC>)  _param_mpc_z_vel_p_acc,
		(ParamFloat<px4::params::MPC_Z_VEL_I_ACC>)  _param_mpc_z_vel_i_acc,
		(ParamFloat<px4::params::MPC_Z_VEL_D_ACC>)  _param_mpc_z_vel_d_acc,
		(ParamFloat<px4::params::MPC_XY_VEL_MAX>)   _param_mpc_xy_vel_max,
		(ParamFloat<px4::params::MPC_Z_V_AUTO_UP>)  _param_mpc_z_v_auto_up,
		(ParamFloat<px4::params::MPC_Z_VEL_MAX_UP>) _param_mpc_z_vel_max_up,
		(ParamFloat<px4::params::MPC_Z_V_AUTO_DN>)  _param_mpc_z_v_auto_dn,
		(ParamFloat<px4::params::MPC_Z_VEL_MAX_DN>) _param_mpc_z_vel_max_dn,
		(ParamFloat<px4::params::MPC_TILTMAX_AIR>)  _param_mpc_tiltmax_air,
		(ParamFloat<px4::params::MPC_THR_HOVER>)    _param_mpc_thr_hover,
		(ParamBool<px4::params::MPC_USE_HTE>)       _param_mpc_use_hte,
		(ParamBool<px4::params::MPC_ACC_DECOUPLE>)  _param_mpc_acc_decouple,

		(ParamFloat<px4::params::MPC_VEL_LP>)       _param_mpc_vel_lp,
		(ParamFloat<px4::params::MPC_VEL_NF_FRQ>)   _param_mpc_vel_nf_frq,
		(ParamFloat<px4::params::MPC_VEL_NF_BW>)    _param_mpc_vel_nf_bw,
		(ParamFloat<px4::params::MPC_VELD_LP>)      _param_mpc_veld_lp,

		// Takeoff / Land
		// 起飞/降落相关参数：电机预转、起飞斜坡、起降速度、落地倾角和推力限制
		(ParamFloat<px4::params::COM_SPOOLUP_TIME>) _param_com_spoolup_time, /**< 解锁后等待电机转速建立的时间 */
		(ParamBool<px4::params::COM_THROW_EN>)      _param_com_throw_en, /**< 是否启用抛飞起飞 */
		(ParamFloat<px4::params::MPC_TKO_RAMP_T>)   _param_mpc_tko_ramp_t,   /**< 平滑起飞斜坡时间常数 */
		(ParamFloat<px4::params::MPC_TKO_SPEED>)    _param_mpc_tko_speed,
		(ParamFloat<px4::params::MPC_LAND_SPEED>)   _param_mpc_land_speed,

		(ParamFloat<px4::params::MPC_VEL_MANUAL>)   _param_mpc_vel_manual,
		(ParamFloat<px4::params::MPC_VEL_MAN_BACK>) _param_mpc_vel_man_back,
		(ParamFloat<px4::params::MPC_VEL_MAN_SIDE>) _param_mpc_vel_man_side,
		(ParamFloat<px4::params::MPC_XY_CRUISE>)    _param_mpc_xy_cruise,
		(ParamFloat<px4::params::MPC_LAND_ALT2>)    _param_mpc_land_alt2,    /**< 低于该高度后限制下降速度 */
		(ParamInt<px4::params::MPC_ALT_MODE>)       _param_mpc_alt_mode,
		(ParamFloat<px4::params::MPC_TILTMAX_LND>)  _param_mpc_tiltmax_lnd,  /**< 降落和平滑起飞阶段的最大倾角 */
		(ParamFloat<px4::params::MPC_THR_MIN>)      _param_mpc_thr_min,
		(ParamFloat<px4::params::MPC_THR_MAX>)      _param_mpc_thr_max,
		(ParamFloat<px4::params::MPC_THR_XY_MARG>)  _param_mpc_thr_xy_marg,

		(ParamFloat<px4::params::SYS_VEHICLE_RESP>) _param_sys_vehicle_resp,
		(ParamFloat<px4::params::MPC_ACC_HOR>)      _param_mpc_acc_hor,
		(ParamFloat<px4::params::MPC_ACC_DOWN_MAX>) _param_mpc_acc_down_max,
		(ParamFloat<px4::params::MPC_ACC_UP_MAX>)   _param_mpc_acc_up_max,
		(ParamFloat<px4::params::MPC_ACC_HOR_MAX>)  _param_mpc_acc_hor_max,
		(ParamFloat<px4::params::MPC_JERK_AUTO>)    _param_mpc_jerk_auto,
		(ParamFloat<px4::params::MPC_JERK_MAX>)     _param_mpc_jerk_max,
		(ParamFloat<px4::params::MPC_MAN_Y_MAX>)    _param_mpc_man_y_max,
		(ParamFloat<px4::params::MPC_MAN_Y_TAU>)    _param_mpc_man_y_tau,

		(ParamFloat<px4::params::MPC_XY_VEL_ALL>)   _param_mpc_xy_vel_all,
		(ParamFloat<px4::params::MPC_Z_VEL_ALL>)    _param_mpc_z_vel_all,

		(ParamFloat<px4::params::MPC_XY_ERR_MAX>) _param_mpc_xy_err_max,
		(ParamFloat<px4::params::MPC_YAWRAUTO_MAX>) _param_mpc_yawrauto_max,
		(ParamFloat<px4::params::MPC_YAWRAUTO_ACC>) _param_mpc_yawrauto_acc
	);

	math::WelfordMean<float> _sample_interval_s{}; ///< 控制周期均值，用于根据实际采样频率更新滤波器参数

	AlphaFilter<matrix::Vector2f> _vel_xy_lp_filter{}; ///< 水平速度低通滤波器
	AlphaFilter<float> _vel_z_lp_filter{}; ///< 垂向速度低通滤波器

	math::NotchFilter<matrix::Vector2f> _vel_xy_notch_filter{}; ///< 水平速度陷波滤波器，用于抑制特定频率振动
	math::NotchFilter<float> _vel_z_notch_filter{}; ///< 垂向速度陷波滤波器

	AlphaFilter<matrix::Vector2f> _vel_deriv_xy_lp_filter{}; ///< 水平速度导数低通滤波器，作为加速度估计
	AlphaFilter<float> _vel_deriv_z_lp_filter{}; ///< 垂向速度导数低通滤波器

	GotoControl _goto_control; ///< 将 goto_setpoint 平滑成 trajectory_setpoint
	PositionControl _control; ///< 核心串级 PID 位置控制器

	hrt_abstime _last_warn{0}; /**< 上一次发送 warning 的时间，用于限频 */

	bool _hover_thrust_initialized{false}; ///< 悬停推力是否已经初始化到 PositionControl

	/** Timeout in us for trajectory data to get considered invalid
	 * 中文：trajectory_setpoint 超过该时间未更新后视为过期。
	 */
	static constexpr uint64_t TRAJECTORY_STREAM_TIMEOUT_US = 500_ms;

	/** During smooth-takeoff, below ALTITUDE_THRESHOLD the yaw-control is turned off and tilt is limited
	 * 中文：平滑起飞时，低于该高度阈值会关闭 yaw 控制并限制倾角。
	 */
	static constexpr float ALTITUDE_THRESHOLD = 0.3f;

	static constexpr float MAX_SAFE_TILT_DEG = 89.f; // 超过该倾角后 tanf 等计算容易出现数值问题

	SlewRate<float> _tilt_limit_slew_rate; ///< 倾角限制的斜率限制器，避免起飞/飞行状态切换时倾角上限跳变

	uint8_t _vxy_reset_counter{0}; ///< EKF 水平速度 reset 计数器缓存
	uint8_t _vz_reset_counter{0}; ///< EKF 垂向速度 reset 计数器缓存
	uint8_t _xy_reset_counter{0}; ///< EKF 水平位置 reset 计数器缓存
	uint8_t _z_reset_counter{0}; ///< EKF 垂向位置 reset 计数器缓存
	uint8_t _heading_reset_counter{0}; ///< EKF 航向 reset 计数器缓存

	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle time")}; ///< 性能计数器，记录控制循环耗时

	/**
	 * Update our local parameter cache.
	 * Parameter update can be forced when argument is true.
	 * 更新本地参数缓存。
	 * force 为 true 时即使没有 parameter_update 消息也强制刷新。
	 * @param force forces parameter update.
	 * @param force 是否强制更新参数
	 */
	void parameters_update(bool force);

	/**
	 * Check for validity of positon/velocity states.
	 * 从 vehicle_local_position 提取并滤波当前位置/速度/加速度状态，同时处理状态有效性。
	 * @param local_pos 本地位置估计消息
	 * @param dt_s 控制周期 [s]
	 * @return PositionControl 可直接使用的状态结构体，失效轴会被置为 NaN
	 */
	PositionControlStates set_vehicle_states(const vehicle_local_position_s &local_pos, const float dt_s);

	/**
	 * Generate setpoint to bridge no executable setpoint being available.
	 * Used to handle transitions where no proper setpoint was generated yet and when the received setpoint is invalid.
	 * This should only happen briefly when transitioning and never during mode operation or by design.
	 * 生成 failsafe 设定值，用于没有可执行 setpoint 或接收到无效 setpoint 的短暂过渡。
	 * 正常模式运行中不应依赖这个路径。
	 * @param now 当前时间戳
	 * @param states 当前控制状态
	 * @param warn 是否允许输出警告
	 * @return 可交给 PositionControl 执行的安全轨迹设定值
	 */
	trajectory_setpoint_s generateFailsafeSetpoint(const hrt_abstime &now, const PositionControlStates &states, bool warn);

	/**
	 * @brief adjust existing (or older) setpoint with any EKF reset deltas and update the local counters
	 * @brief 用 EKF reset delta 修正已有或较旧的 setpoint，并更新本地 reset 计数器。
	 *
	 * @param[in] vehicle_local_position struct containing EKF reset deltas and counters
	 * @param[in] vehicle_local_position 包含 EKF reset delta 和计数器的本地位置消息
	 * @param[out] setpoint trajectory setpoint struct to be adjusted
	 * @param[out] setpoint 需要按 EKF reset 进行同步修正的轨迹设定值
	 */
	void adjustSetpointForEKFResets(const vehicle_local_position_s &vehicle_local_position,
					trajectory_setpoint_s &setpoint);
};
