/****************************************************************************
 *
 *   Copyright (c) 2015-2022 PX4 Development Team. All rights reserved.
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
* @file vtol_type.h
*
* @author Roman Bapst 		<bapstroman@gmail.com>
* @author Sander Smeets		<sander@droneslab.com>
* @author Andreas Antener	<andreas@uaventure.com>
*
*/

#ifndef VTOL_TYPE_H
#define VTOL_TYPE_H

#include <drivers/drv_hrt.h>
#include <lib/mathlib/mathlib.h>
#include <px4_platform_common/module_params.h>


static constexpr float kFlapSlewRateVtol = 1.f; // minimum time from none to full flap deflection [s]
static constexpr float kSpoilerSlewRateVtol = 1.f; // minimum time from none to full spoiler deflection [s]


// 所有 VTOL 构型共用的高层状态，数值必须与 msg/vtol_vehicle_status.msg 一一对应。
// Tailsitter 内部更细的 MC/FW/前后转换状态最终都会映射到这里，供主模块统一调度。
enum class mode {
	TRANSITION_TO_FW = 1,  // 前转换：尾座式由 MC 姿态控制器跟踪转换目标
	TRANSITION_TO_MC = 2,  // 后转换：尾座式同样由 MC 姿态控制器跟踪
	ROTARY_WING = 3,       // 纯悬停/多旋翼状态
	FIXED_WING = 4         // 纯固定翼状态
};

enum class vtol_type {
	TAILSITTER = 0, // VT_TYPE=0 时主模块实例化 Tailsitter
	TILTROTOR,
	STANDARD
};

enum VtolForwardActuationMode {
	DISABLE = 0,
	ENABLE_WITHOUT_LAND,
	ENABLE_ABOVE_MPC_LAND_ALT1,
	ENABLE_ABOVE_MPC_LAND_ALT2,
	ENABLE_ALL_MODES,
	ENABLE_ABOVE_MPC_LAND_ALT1_WITHOUT_LAND,
	ENABLE_ABOVE_MPC_LAND_ALT2_WITHOUT_LAND
};

// VT_FW_DIFTHR_EN 位掩码：决定固定翼状态下哪些 FW 力矩可以通过电机差动实现。
enum class VtFwDifthrEnBits : int32_t {
	YAW_BIT = (1 << 0),
	ROLL_BIT = (1 << 1),
	PITCH_BIT = (1 << 2),
};

enum class QuadchuteReason {
	None = 0,
	TransitionTimeout,
	ExternalCommand,
	MinimumAltBreached,
	UncommandedDescent,
	TransitionAltitudeLoss,
	MaximumPitchExceeded,
	MaximumRollExceeded,
};

class VtolAttitudeControl;

class VtolType : public ModuleParams
{
public:

	// att_controller 指向唯一的 VtolAttitudeControl 主对象。
	// 构造函数通过主对象 getter 取得输入缓存和最终输出对象的地址，派生类不复制这些消息。
	VtolType(VtolAttitudeControl *att_controller);
	VtolType(const VtolType &) = delete;
	VtolType &operator=(const VtolType &) = delete;

	virtual ~VtolType() = default;

	/**
	 * Initialise.
	 */
	bool init();

	/**
	 * Update vtol state.
	 */
	virtual void update_vtol_state() = 0;

	/**
	 * Update transition state.
	 * 尾座式重载后生成前/后转换目标姿态；基类部分维护时间、推力记忆和 quadchute。
	 */
	virtual void update_transition_state() = 0;

	/**
	 * Update multicopter state.
	 * 将 MC 虚拟姿态目标复制为最终 vehicle_attitude_setpoint。
	 */
	virtual void update_mc_state();

	/**
	 * Update fixed wing state.
	 * 将 FW 虚拟姿态目标复制为最终目标，并处理 TECS 延迟接管和 quadchute。
	 */
	virtual void update_fw_state();

	/**
	 * Write control values to actuator output topics.
	 * 派生类通过下方四个输出指针填写主模块的 torque/thrust instance 0/1 消息。
	 */
	virtual void fill_actuator_outputs() = 0;

	/**
	 * Special handling opportunity for the time right after transition to FW
	 * before TECS is running.
	 */
	virtual void waiting_on_tecs() {}

	/**
	 *  @brief Indicates if quadchute is enabled.
	 *
	 * @return     true if enabled
	 */
	bool isQuadchuteEnabled();

	/**
	 *  @brief Evaluates quadchute conditions and returns a reson for quadchute.
	 *
	 * @return     QuadchuteReason, can be None
	 */
	QuadchuteReason getQuadchuteReason();

	/**
	 *  @brief Indicates if the vehicle is lower than VT_FW_MIN_ALT above the local origin.
	 *
	 * @return     true if below threshold
	 */
	bool isMinAltBreached();

	/**
	 * @brief Indicates if conditions are met for uncommanded-descent quad-chute.
	 *
	 * @return true if integrated height rate error larger than threshold
	 */
	bool isUncommandedDescent();

	/**
	 * @brief Indicates if there is an altitude loss higher than specified threshold during a VTOL transition to FW
	 *
	 * @return true if error larger than threshold
	 */
	bool isFrontTransitionAltitudeLoss();

	/**
	 *  @brief Indicates if the absolute value of the vehicle pitch angle exceeds the threshold defined by VT_FW_QC_P
	 *
	 * @return     true if exeeded
	 */
	bool isPitchExceeded();

	/**
	 *  @brief Indicates if the absolute value of the vehicle roll angle exceeds the threshold defined by VT_FW_QC_R
	 *
	 * @return     true if exeeded
	 */
	bool isRollExceeded();

	/**
	 *  @brief Indicates if the front transition duration has exceeded the timeout definded by VT_TRANS_TIMEOUT
	 *
	 * @return     true if exeeded
	 */
	bool isFrontTransitionTimeout();

	/**
	 *  @brief Special handling of QuadchuteReason::ReasonExternal
	 */
	void handleSpecialExternalCommandQuadchute();

	/**
	 * Checks for fixed-wing failsafe condition and issues abort request if needed.
	 */
	void check_quadchute_condition();

	/**
	 * Returns true if we're allowed to do a mode transition on the ground.
	 */
	bool can_transition_on_ground();

	/**
	 * Pusher assist in hover (pusher/pull for standard VTOL, motor tilt for tiltrotor)
	 */
	float pusher_assist();

	virtual void blendThrottleAfterFrontTransition(float scale) {};

	mode get_mode() {return _common_vtol_mode;}

	/**
	 * @return Minimum front transition time scaled for air density (if available) [s]
	*/
	float getMinimumFrontTransitionTime() const;

	/**
	 * @return Front transition timeout scaled for air density (if available) [s]
	*/
	float getFrontTransitionTimeout() const;

	/**
	* @return Minimum open-loop front transition time scaled for air density (if available) [s]
	*/
	float getOpenLoopFrontTransitionTime() const;

	/**
	 *
	 * @return The calibrated blending airspeed [m/s]
	 */
	float getBlendAirspeed() const;

	/**
	 *
	 * @return The calibrated transition airspeed [m/s]
	 */
	float getTransitionAirspeed() const;

	virtual void parameters_update() = 0;


	/**
	 * @brief Resets the transition timer states.
	 *
	 */
	void resetTransitionStates();

	/**
	 * @brief Handle EKF position resets.
	 *
	 */
	void handleEkfResets();

protected:
	VtolAttitudeControl *_attc; // 非拥有型指针：指向创建本对象的 VTOL 主模块
	mode _common_vtol_mode;     // 映射后的通用状态，供 VtolAttitudeControl::Run() 调度

	static constexpr const int num_outputs_max = 8;

	/*
	 * 以下均为非拥有型指针，全部指向 VtolAttitudeControl 中实际存储的消息对象。
	 * 这样派生类能直接读主循环最新订阅值、写主循环稍后要发布的最终值，
	 * 不需要在每个虚函数调用中传递大量参数，也不会产生两份不同步的消息副本。
	 */
	struct vehicle_attitude_s		*_v_att;			// EKF 当前姿态
	struct vehicle_attitude_setpoint_s	*_v_att_sp;			// 最终统一姿态目标（主模块发布）
	struct vehicle_attitude_setpoint_s *_mc_virtual_att_sp;	// MC 控制链候选姿态/推力目标
	struct vehicle_attitude_setpoint_s *_fw_virtual_att_sp;	// FW 控制链候选姿态/推力目标
	struct vehicle_control_mode_s		*_v_control_mode;	// 当前控制使能标志
	struct vtol_vehicle_status_s 		*_vtol_vehicle_status;	// VTOL 状态和 FW 故障标志
	struct vehicle_torque_setpoint_s 		*_vehicle_torque_setpoint_virtual_mc; // MC rate 输出
	struct vehicle_torque_setpoint_s 		*_vehicle_torque_setpoint_virtual_fw; // FW rate 输出
	struct vehicle_thrust_setpoint_s 		*_vehicle_thrust_setpoint_virtual_mc; // MC -Z 推力
	struct vehicle_thrust_setpoint_s 		*_vehicle_thrust_setpoint_virtual_fw; // FW +X 推力
	struct vehicle_local_position_s			*_local_pos;		// 高度/转换保护使用
	struct vehicle_local_position_setpoint_s	*_local_pos_sp;		// 位置目标/辅助逻辑使用
	struct airspeed_validated_s 			*_airspeed_validated;					// airspeed
	struct tecs_status_s				*_tecs_status;
	struct vehicle_land_detected_s			*_land_detected;

	// 最终输出消息地址：Tailsitter::fill_actuator_outputs() 写入，
	// VtolAttitudeControl::Run() 随后通过 PublicationMulti 发布同一对象。
	struct vehicle_torque_setpoint_s 		*_torque_setpoint_0; // instance 0，主要用于电机
	struct vehicle_torque_setpoint_s 		*_torque_setpoint_1; // instance 1，主要用于舵面
	struct vehicle_thrust_setpoint_s 		*_thrust_setpoint_0; // instance 0 推力
	struct vehicle_thrust_setpoint_s 		*_thrust_setpoint_1; // instance 1 推力（尾座式通常为零）

	float _mc_roll_weight = 1.0f;	// weight for multicopter attitude controller roll output
	float _mc_pitch_weight = 1.0f;	// weight for multicopter attitude controller pitch output
	float _mc_yaw_weight = 1.0f;	// weight for multicopter attitude controller yaw output
	float _mc_throttle_weight = 1.0f;	// weight for multicopter throttle command. Used to avoid

	// motors spinning up or cutting too fast when doing transitions.
	float _thrust_transition = 0.0f;	// thrust value applied during a front transition (tailsitter & tiltrotor only)
	float _last_thr_in_fw_mode = 0.0f; // 最近一次 FW +body-X 推力，用于后转换连续接管
	float _last_thr_in_mc = 0.f;       // 最近一次 MC -body-Z 推力，用于前转换完成后的连续接管

	hrt_abstime _trans_finished_ts = 0;         // 最近一次前转换完成时间
	hrt_abstime _transition_start_timestamp{0}; // 当前前/后转换开始时间
	float _time_since_trans_start{0};           // 当前转换已持续秒数

	bool _tecs_running = false;        // 前转换后是否已经收到新的 TECS 状态
	hrt_abstime _tecs_running_ts = 0;  // TECS 首次恢复的时刻，用于确认 FW 姿态目标也已更新

	hrt_abstime _last_loop_ts = 0;
	float _transition_dt = 0;

	float _quadchute_ref_alt{NAN};	// altitude (AMSL) reference to compute quad-chute altitude loss condition

	bool _quadchute_command_treated{false};

	bool isFrontTransitionCompleted();
	virtual bool isFrontTransitionCompletedBase();

	float _local_position_z_start_of_transition{0.f}; // altitude at start of transition

	int _altitude_reset_counter{0};

	DEFINE_PARAMETERS_CUSTOM_PARENT(ModuleParams,
					(ParamBool<px4::params::VT_ELEV_MC_LOCK>) _param_vt_elev_mc_lock,
					(ParamFloat<px4::params::VT_FW_MIN_ALT>) _param_vt_fw_min_alt,
					(ParamFloat<px4::params::VT_QC_ALT_LOSS>) _param_vt_qc_alt_loss,
					(ParamInt<px4::params::VT_FW_QC_P>) _param_vt_fw_qc_p,
					(ParamInt<px4::params::VT_FW_QC_R>) _param_vt_fw_qc_r,
					(ParamFloat<px4::params::VT_QC_T_ALT_LOSS>) _param_vt_qc_t_alt_loss,
					(ParamInt<px4::params::VT_FW_QC_HMAX>) _param_quadchute_max_height,
					(ParamFloat<px4::params::VT_F_TR_OL_TM>) _param_vt_f_tr_ol_tm,
					(ParamFloat<px4::params::VT_TRANS_MIN_TM>) _param_vt_trans_min_tm,

					(ParamFloat<px4::params::VT_F_TRANS_DUR>) _param_vt_f_trans_dur,
					(ParamFloat<px4::params::VT_B_TRANS_DUR>) _param_vt_b_trans_dur,
					(ParamFloat<px4::params::VT_ARSP_TRANS>) _param_vt_arsp_trans,
					(ParamFloat<px4::params::VT_F_TRANS_THR>) _param_vt_f_trans_thr,
					(ParamFloat<px4::params::VT_ARSP_BLEND>) _param_vt_arsp_blend,
					(ParamFloat<px4::params::VT_TRANS_TIMEOUT>) _param_vt_trans_timeout,
					(ParamFloat<px4::params::MPC_XY_CRUISE>) _param_mpc_xy_cruise,
					(ParamInt<px4::params::VT_FW_DIFTHR_EN>) _param_vt_fw_difthr_en,
					(ParamFloat<px4::params::VT_FW_DIFTHR_S_Y>) _param_vt_fw_difthr_s_y,
					(ParamFloat<px4::params::VT_FW_DIFTHR_S_P>) _param_vt_fw_difthr_s_p,
					(ParamFloat<px4::params::VT_FW_DIFTHR_S_R>) _param_vt_fw_difthr_s_r,
					(ParamFloat<px4::params::VT_B_DEC_I>) _param_vt_b_dec_i,
					(ParamFloat<px4::params::VT_B_DEC_MSS>) _param_vt_b_dec_mss,

					(ParamFloat<px4::params::VT_PITCH_MIN>) _param_vt_pitch_min,
					(ParamFloat<px4::params::VT_FWD_THRUST_SC>) _param_vt_fwd_thrust_sc,
					(ParamInt<px4::params::VT_FWD_THRUST_EN>) _param_vt_fwd_thrust_en,
					(ParamFloat<px4::params::MPC_LAND_ALT1>) _param_mpc_land_alt1,
					(ParamFloat<px4::params::MPC_LAND_ALT2>) _param_mpc_land_alt2,
					(ParamFloat<px4::params::VT_LND_PITCH_MIN>) _param_vt_lnd_pitch_min,
					(ParamFloat<px4::params::WEIGHT_BASE>) _param_weight_base,
					(ParamFloat<px4::params::WEIGHT_GROSS>) _param_weight_gross

				       )

private:
	hrt_abstime _throttle_blend_start_ts{0};	// time at which we start blending between transition throttle and fixed wing throttle

	bool shouldBlendThrottleAfterFrontTransition() { return _throttle_blend_start_ts != 0; };

	void stopBlendingThrottleAfterFrontTransition() { _throttle_blend_start_ts = 0; }

	/**
	 * @return Transition time scale factor for density.
	*/
	float getFrontTransitionTimeFactor() const;

};

#endif
