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
 * @file EKF2.hpp
 * Implementation of the attitude and position estimator.
 *
 * @author Roman Bapst
 */

#ifndef EKF2_HPP
#define EKF2_HPP

#include "EKF/ekf.h"

#include "EKF2Selector.hpp"
#include "mathlib/math/filter/AlphaFilter.hpp"

#include <float.h>

#include <containers/LockGuard.hpp>
#include <drivers/drv_hrt.h>
#include <lib/mathlib/mathlib.h>
#include <lib/perf/perf_counter.h>
#include <lib/systemlib/mavlink_log.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/time.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/ekf2_timestamps.h>
#include <uORB/topics/estimator_bias.h>
#include <uORB/topics/estimator_bias3d.h>
#include <uORB/topics/estimator_event_flags.h>
#include <uORB/topics/estimator_innovations.h>
#include <uORB/topics/estimator_sensor_bias.h>
#include <uORB/topics/estimator_states.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/estimator_status_flags.h>
#include <uORB/topics/launch_detection_status.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/sensor_selection.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_imu.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/yaw_estimator_status.h>

#if defined(CONFIG_EKF2_AIRSPEED)
# include <uORB/topics/airspeed.h>
# include <uORB/topics/airspeed_validated.h>
#endif // CONFIG_EKF2_AIRSPEED

#if defined(CONFIG_EKF2_AUXVEL)
# include <uORB/topics/landing_target_pose.h>
#endif // CONFIG_EKF2_AUXVEL

#if defined(CONFIG_EKF2_BAROMETER)
# include <uORB/topics/vehicle_air_data.h>
#endif // CONFIG_EKF2_BAROMETER

#if defined(CONFIG_EKF2_GNSS)
# include <uORB/topics/estimator_gps_status.h>
# include <uORB/topics/sensor_gps.h>
#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_MAGNETOMETER)
# include <uORB/topics/vehicle_magnetometer.h>
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
# include <uORB/topics/vehicle_optical_flow.h>
# include <uORB/topics/vehicle_optical_flow_vel.h>
#endif // CONFIG_EKF2_OPTICAL_FLOW

#if defined(CONFIG_EKF2_RANGE_FINDER)
# include <uORB/topics/distance_sensor.h>
#endif // CONFIG_EKF2_RANGE_FINDER

#if defined(CONFIG_EKF2_WIND)
# include <uORB/topics/wind.h>
#endif // CONFIG_EKF2_WIND

extern pthread_mutex_t ekf2_module_mutex;

// 生命周期锁供 start/stop/status/select_instance 等命令使用，保护全局实例管理。
// 滤波计算由工作队列调用 Run() 驱动；一次 Run() 不等于一定完成一次主 EKF 预测。

// 中文导读：EKF2 是 PX4 模块封装类，持有一个 Ekf 算法实例。成员大体分为：
// uORB 传感器订阅、结果发布、PX4 参数绑定，以及设备选择/校准状态。
// 真正的滤波状态与协方差位于成员 _ekf 中。

class EKF2 final : public ModuleParams, public px4::ScheduledWorkItem
{
public:
	// 禁止无参构造：每个实例必须指定单/多实例模式、工作队列和是否回放。
	// ModuleParams 提供参数更新；ScheduledWorkItem 提供调度，具体计算在 Run() 中。
	EKF2() = delete;
	EKF2(bool multi_mode, const px4::wq_config_t &config, bool replay_mode);
	~EKF2() override;

	// 命令入口调用此函数创建 EKF 实例；多实例模式还会创建 EKF2Selector。
	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	// 处理未被 ekf2_main() 识别的命令，当前实现输出用法提示。
	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	// 输出命令行帮助；reason 非空时附带原因。
	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	// 输出本实例估计有效性和性能统计，verbose 控制算法内部详细状态输出。
	int print_status(bool verbose = false);

	// 原子停止标志由命令线程写入，工作队列在 Run() 开头检查并注销 IMU 回调。
	bool should_exit() const { return _task_should_exit.load(); }

	void request_stop() { _task_should_exit.store(true); }

	static void lock_module() { pthread_mutex_lock(&ekf2_module_mutex); }
	static bool trylock_module() { return (pthread_mutex_trylock(&ekf2_module_mutex) == 0); }
	static void unlock_module() { pthread_mutex_unlock(&ekf2_module_mutex); }

#if defined(CONFIG_EKF2_MULTI_INSTANCE)
	// imu/mag 是传感器 uORB 实例号；本函数绑定输入，并核对输出 topic 实例号。
	bool multi_init(int imu, int mag);
#endif // CONFIG_EKF2_MULTI_INSTANCE

	// EKF 输出实例号与输入 IMU 实例号不一定相同。
	int instance() const { return _instance; }

private:

	static constexpr uint8_t MAX_NUM_IMUS = 4;
	static constexpr uint8_t MAX_NUM_MAGS = 4;

	// 主工作函数：参数/命令 -> IMU -> 外部观测 -> 延迟 EKF 更新 -> 输出和诊断。
	// IMU 消息回调触发运行，ScheduleDelayed() 提供超时备用调度。
	void Run() override;

	// Publish* 把算法内部表示转换成 uORB 消息；Update* 执行反向转换，并记录
	// 相对 IMU 的话题时间差，便于检查输入时序；它不直接等同于传感器的物理测量延迟。
	// 提前注册日志需要的输出 topic，详细诊断 topic 按配置和融合参数选择注册。
	void AdvertiseTopics();
	// 校正过期的磁力计模式，并保证最大融合延迟不小于已配置的传感器延迟。
	void VerifyParams();

	// 发布各观测源自己的观测、创新、创新方差、检验比值和融合结果。
	void PublishAidSourceStatus(const hrt_abstime &timestamp);
	// 发布输出预测器的当前姿态以及姿态重置增量，可按输入 IMU 节拍运行。
	void PublishAttitude(const hrt_abstime &timestamp);

#if defined(CONFIG_EKF2_BAROMETER)
	// 发布气压高度相对参考高度的偏置，不是 IMU 加速度计零偏。
	void PublishBaroBias(const hrt_abstime &timestamp);
#endif // CONFIG_EKF2_BAROMETER

#if defined(CONFIG_EKF2_RANGE_FINDER)
	// 测距高度偏置发布接口声明，当前 EKF2.cpp 未使用该接口。
	void PublishRngHgtBias(const hrt_abstime &timestamp);
#endif // CONFIG_EKF2_RANGE_FINDER

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	// 合并视觉水平位置的两个偏置和视觉高度偏置，发布为三维偏置诊断。
	void PublishEvPosBias(const hrt_abstime &timestamp);
#endif // CONFIG_EKF2_EXTERNAL_VISION
	// 将一维 BiasEstimator 状态打包，并区分测量时间 timestamp_sample 与发布时间。
	estimator_bias_s fillEstimatorBiasMsg(const BiasEstimator::status &status, uint64_t timestamp_sample_us,
					      uint64_t timestamp, uint32_t device_id = 0);
	// 信息事件是一次性事件，如“开始 GNSS 融合”“位置重置到视觉”。
	void PublishEventFlags(const hrt_abstime &timestamp);
	// 发布经纬度与海拔，同时报告有效性、重置和航位推算状态。
	void PublishGlobalPosition(const hrt_abstime &timestamp);
	// 以下三个消息分别表示残差本身、相对门限的归一化检验量、残差不确定度。
	// 三者均来自延迟融合端；理解故障时应结合对应观测源的融合状态一起看。
	void PublishInnovations(const hrt_abstime &timestamp);
	void PublishInnovationTestRatios(const hrt_abstime &timestamp);
	void PublishInnovationVariances(const hrt_abstime &timestamp);
	// 发布局部 NED 位置/速度/加速度、参考原点、航向、离地高度和控制限制。
	void PublishLocalPosition(const hrt_abstime &timestamp);
	// 用统一里程计格式发布当前姿态、位置、速度、角速度及各自方差。
	void PublishOdometry(const hrt_abstime &timestamp, const imuSample &imu_sample);
	// 发布陀螺/加速度计/磁力计零偏、方差、硬件 ID 和稳定性标志。
	void PublishSensorBias(const hrt_abstime &timestamp);
	// 发布延迟端名义状态和误差协方差对角线，两者维数/索引可能不同。
	void PublishStates(const hrt_abstime &timestamp);
	// 汇总有效性、创新检验、故障、重置计数和输出预测器跟踪误差。
	void PublishStatus(const hrt_abstime &timestamp);
	// 将内部位域展开为具名布尔字段；变化时立即发布，否则约 1 Hz 更新。
	void PublishStatusFlags(const hrt_abstime &timestamp);
#if defined(CONFIG_EKF2_WIND)
	// 风估计状态有效时发布 N/E 风向量、方差及相关观测诊断。
	void PublishWindEstimate(const hrt_abstime &timestamp);
#endif // CONFIG_EKF2_WIND

#if defined(CONFIG_EKF2_AIRSPEED)
	// 优先使用经过验证的空速；该话题长期无更新时回退到原始空速。
	void UpdateAirspeedSample(ekf2_timestamps_s &ekf2_timestamps);
#endif // CONFIG_EKF2_AIRSPEED
#if defined(CONFIG_EKF2_AUXVEL)
	// 将静止着陆目标相对飞行器的速度取反，形成飞行器水平速度观测。
	void UpdateAuxVelSample(ekf2_timestamps_s &ekf2_timestamps);
#endif // CONFIG_EKF2_AUXVEL
#if defined(CONFIG_EKF2_BAROMETER)
	// 输入气压高度和空气密度；设备/校准变化通过样本 reset 标志传给算法层。
	void UpdateBaroSample(ekf2_timestamps_s &ekf2_timestamps);
#endif // CONFIG_EKF2_BAROMETER
#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	// 检查视觉数据的坐标系、有限性和方差；位置/速度/姿态可分别有效。
	// 返回值表示存在可提交的视觉观测，不表示该观测已通过 EKF 创新检验。
	bool UpdateExtVisionSample(ekf2_timestamps_s &ekf2_timestamps);
#endif // CONFIG_EKF2_EXTERNAL_VISION
#if defined(CONFIG_EKF2_GNSS)
	// 使用低通后的大地水准面高度差转换：椭球高 = 海拔高 + 高度差。
	float altEllipsoidToAmsl(float ellipsoid_alt) const;
	float altAmslToEllipsoid(float amsl_alt) const;

	// 分别发布 GNSS 质量检查、高度偏置，以及 GSF 各航向模型的诊断。
	void PublishGpsStatus(const hrt_abstime &timestamp);
	void PublishGnssHgtBias(const hrt_abstime &timestamp);
	void PublishYawEstimatorStatus(const hrt_abstime &timestamp);
	// 输入经纬度、海拔、NED 速度、精度和双天线航向，并维护高度基准转换。
	void UpdateGpsSample(ekf2_timestamps_s &ekf2_timestamps);
#endif // CONFIG_EKF2_GNSS
#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	// 将光流/陀螺积分除以积分时间得到角速率，并转换符号与测量中点时间。
	bool UpdateFlowSample(ekf2_timestamps_s &ekf2_timestamps);
	// 输出由光流得到的速度和旋转补偿诊断，便于检查光流输入与 IMU 的一致性。
	void PublishOpticalFlowVel(const hrt_abstime &timestamp);
#endif // CONFIG_EKF2_OPTICAL_FLOW
#if defined(CONFIG_EKF2_MAGNETOMETER)
	// 输入机体系磁场，识别更换磁力计或校准参数更新事件。
	void UpdateMagSample(ekf2_timestamps_s &ekf2_timestamps);
#endif // CONFIG_EKF2_MAGNETOMETER
#if defined(CONFIG_EKF2_RANGE_FINDER)
	// 从多实例测距消息中选择朝下且新鲜的传感器；超时后重新选择。
	void UpdateRangeSample(ekf2_timestamps_s &ekf2_timestamps);
#endif // CONFIG_EKF2_RANGE_FINDER

	// 将在空中、静止、固定翼、地效和等待发射等状态送入延迟时间队列。
	void UpdateSystemFlagsSample(ekf2_timestamps_s &ekf2_timestamps);

	// Used to check, save and use learned accel/gyro/mag biases
	// 零偏稳定性记录：本结构用于判断“是否已稳定学习”，本身不写传感器校准参数。
	// last_us 为上一次满足学习条件的时间，total_time_us 累计有效学习时长。
	// bias 保存稳定性比较的基准；cal_available 为稳定性判定结果。
	struct InFlightCalibration {
		hrt_abstime last_us{0};         ///< last time the EKF was operating a mode that estimates accelerometer biases (uSec)
		hrt_abstime total_time_us{0};   ///< accumulated calibration time since the last save
		matrix::Vector3f bias{};
		bool cal_available{false};      ///< true when an unsaved valid calibration for the XYZ accelerometer bias is available
	};

	// 公用稳定性检查：偏置有效、方差足够小且变化不大时累计时间，超过 10 秒置有效。
	// bias_valid 表示估计本身可信；learning_valid 还要求当前允许学习该偏置。
	void UpdateCalibration(const hrt_abstime &timestamp, InFlightCalibration &cal, const matrix::Vector3f &bias,
			       const matrix::Vector3f &bias_variance, float bias_limit, bool bias_valid, bool learning_valid);
	void UpdateAccelCalibration(const hrt_abstime &timestamp);
	void UpdateGyroCalibration(const hrt_abstime &timestamp);
#if defined(CONFIG_EKF2_MAGNETOMETER)
	void UpdateMagCalibration(const hrt_abstime &timestamp);
#endif // CONFIG_EKF2_MAGNETOMETER

	// publish helper for estimator_aid_source topics
	// 模板同时支持 1D/2D/3D 观测源消息。只在样本时间推进时发布，避免重复发送。
	template <typename T>
	void PublishAidSourceStatus(const hrt_abstime &timestamp, const T &status, hrt_abstime &status_publish_last,
				    uORB::PublicationMulti<T> &pub)
	{
		if (status.timestamp_sample > status_publish_last) {
			// 复制后补上实例和发布时间，不修改算法保存的原始状态。
			// publish if updated
			T status_out{status};
			status_out.estimator_instance = _instance;
			status_out.timestamp = _replay_mode ? timestamp : hrt_absolute_time();
			pub.publish(status_out);

			// record timestamp sample
			status_publish_last = status.timestamp_sample;
		}
	}

	static constexpr float sq(float x) { return x * x; };

	// 回放时主要输出消息使用输入数据时间，使结果与日志时间轴一致。
	const bool _replay_mode{false};			///< true when we use replay data from a log
	// 多实例时结果先进入 estimator_*；由选择器转发到最终 vehicle_*。
	const bool _multi_mode;
	int _instance{0};

	px4::atomic_bool _task_should_exit{false};

	// time slip monitoring
	// 比较时间戳推进量与陀螺积分总时长，用于发现丢样或积分时间不一致。
	uint64_t _integrated_time_us = 0;	///< integral of gyro delta time from start (uSec)
	uint64_t _start_time_us = 0;		///< system time at EKF start (uSec)
	int64_t _last_time_slip_us = 0;		///< Last time slip (uSec)

	// 主 EKF 更新时间与 IMU 消息代际跳变计数，分别用于性能和输入连续性诊断。
	perf_counter_t _ekf_update_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": EKF update")};
	perf_counter_t _msg_missed_imu_perf{perf_alloc(PC_COUNT, MODULE_NAME": IMU message missed")};

	// 稳定性记录与硬件校准计数分开：前者来自在线估计，后者由传感器链路报告。
	InFlightCalibration _accel_cal{};
	InFlightCalibration _gyro_cal{};

	uint8_t _accel_calibration_count{0};
	uint8_t _gyro_calibration_count{0};

	// device_id 标识具体硬件；检测到硬件更换时需要重新估计对应零偏。
	uint32_t _device_id_accel{0};
	uint32_t _device_id_gyro{0};

	// 缓存上次已发布的值，实现“变化触发 + 周期保底”的偏置发布策略。
	Vector3f _last_accel_bias_published{};
	Vector3f _last_gyro_bias_published{};

	hrt_abstime _last_sensor_bias_published{0};

	// _status_*_pub_last 保存已发布的样本时间，供模板发布函数去重。
	hrt_abstime _status_fake_hgt_pub_last{0};
	hrt_abstime _status_fake_pos_pub_last{0};

#if defined(CONFIG_EKF2_MAGNETOMETER)
	// 磁力计组：硬件/校准识别、磁偏角更新、在线偏置稳定性和观测诊断。
	uint32_t _device_id_mag {0};

	// Used to control saving of mag declination to be used on next startup
	bool _mag_decl_saved = false;	///< true when the magnetic declination has been saved

	InFlightCalibration _mag_cal{};
	uint8_t _mag_calibration_count{0};
	Vector3f _last_mag_bias_published{};

	hrt_abstime _status_mag_pub_last{0};

	uORB::Subscription _magnetometer_sub{ORB_ID(vehicle_magnetometer)};

	uORB::PublicationMulti<estimator_aid_source3d_s> _estimator_aid_src_mag_pub{ORB_ID(estimator_aid_src_mag)};
#endif // CONFIG_EKF2_MAGNETOMETER

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	// 同一条视觉里程计可提供高度、水平位置、三维速度和航向四类观测。
	// 每类观测独立发布融合状态，不能仅凭收到视觉消息判断全部通道已激活。
	uORB::PublicationMulti<estimator_aid_source1d_s> _estimator_aid_src_ev_hgt_pub {ORB_ID(estimator_aid_src_ev_hgt)};
	uORB::PublicationMulti<estimator_aid_source2d_s> _estimator_aid_src_ev_pos_pub{ORB_ID(estimator_aid_src_ev_pos)};
	uORB::PublicationMulti<estimator_aid_source3d_s> _estimator_aid_src_ev_vel_pub{ORB_ID(estimator_aid_src_ev_vel)};
	uORB::PublicationMulti<estimator_aid_source1d_s> _estimator_aid_src_ev_yaw_pub{ORB_ID(estimator_aid_src_ev_yaw)};
	hrt_abstime _status_ev_hgt_pub_last{0};
	hrt_abstime _status_ev_pos_pub_last{0};
	hrt_abstime _status_ev_vel_pub_last{0};
	hrt_abstime _status_ev_yaw_pub_last{0};

	matrix::Vector3f _last_ev_bias_published{};

	uORB::Subscription _ev_odom_sub{ORB_ID(vehicle_visual_odometry)};

	uORB::PublicationMulti<estimator_bias3d_s> _estimator_ev_pos_bias_pub{ORB_ID(estimator_ev_pos_bias)};
#endif // CONFIG_EKF2_EXTERNAL_VISION

#if defined(CONFIG_EKF2_AUXVEL)
	// 辅助速度来自静止着陆目标的相对速度，不是另一个完整导航滤波器。
	uORB::Subscription _landing_target_pose_sub {ORB_ID(landing_target_pose)};

	uORB::PublicationMulti<estimator_aid_source2d_s> _estimator_aid_src_aux_vel_pub{ORB_ID(estimator_aid_src_aux_vel)};
	hrt_abstime _status_aux_vel_pub_last{0};
#endif // CONFIG_EKF2_AUXVEL

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	// 光流原始输入、光流速度诊断与 EKF 创新诊断是三个不同层次的话题。
	uORB::Subscription _vehicle_optical_flow_sub {ORB_ID(vehicle_optical_flow)};
	uORB::PublicationMulti<vehicle_optical_flow_vel_s> _estimator_optical_flow_vel_pub{ORB_ID(estimator_optical_flow_vel)};

	uORB::PublicationMulti<estimator_aid_source2d_s> _estimator_aid_src_optical_flow_pub{ORB_ID(estimator_aid_src_optical_flow)};
	hrt_abstime _status_optical_flow_pub_last{0};
	hrt_abstime _optical_flow_vel_pub_last{0};
#endif // CONFIG_EKF2_OPTICAL_FLOW

#if defined(CONFIG_EKF2_BAROMETER)
	// 气压设备变化/重新校准时，旧的高度偏置不再适用。
	uint8_t _baro_calibration_count {0};
	uint32_t _device_id_baro{0};
	hrt_abstime _status_baro_hgt_pub_last{0};

	float _last_baro_bias_published{};

	uORB::Subscription _airdata_sub{ORB_ID(vehicle_air_data)};

	uORB::PublicationMulti<estimator_bias_s> _estimator_baro_bias_pub{ORB_ID(estimator_baro_bias)};
	uORB::PublicationMulti<estimator_aid_source1d_s> _estimator_aid_src_baro_hgt_pub {ORB_ID(estimator_aid_src_baro_hgt)};
#endif // CONFIG_EKF2_BAROMETER

#if defined(CONFIG_EKF2_DRAG_FUSION)
	// 阻力融合诊断用于多旋翼风估计，观测通常来自 IMU 比力与阻力模型。
	uORB::PublicationMulti<estimator_aid_source2d_s> _estimator_aid_src_drag_pub {ORB_ID(estimator_aid_src_drag)};
	hrt_abstime _status_drag_pub_last{0};
#endif // CONFIG_EKF2_DRAG_FUSION

#if defined(CONFIG_EKF2_AIRSPEED)
	// 同时订阅原始与验证后空速，时间记录用于决定能否回退到原始输入。
	uORB::Subscription _airspeed_sub {ORB_ID(airspeed)};
	uORB::Subscription _airspeed_validated_sub{ORB_ID(airspeed_validated)};

	float _airspeed_scale_factor{1.0f}; ///< scale factor correction applied to airspeed measurements
	hrt_abstime _airspeed_validated_timestamp_last{0};

	uORB::PublicationMulti<estimator_aid_source1d_s> _estimator_aid_src_airspeed_pub {ORB_ID(estimator_aid_src_airspeed)};
	hrt_abstime _status_airspeed_pub_last{0};
#endif // CONFIG_EKF2_AIRSPEED

#if defined(CONFIG_EKF2_SIDESLIP)
	uORB::PublicationMulti<estimator_aid_source1d_s> _estimator_aid_src_sideslip_pub {ORB_ID(estimator_aid_src_sideslip)};
	hrt_abstime _status_sideslip_pub_last {0};
#endif // CONFIG_EKF2_SIDESLIP

	// 面向地面站/日志的提示通道，与滤波数据 topic 分开。
	orb_advert_t _mavlink_log_pub{nullptr};

	// 参数变更通知按 1 s 间隔节流；首次 Run() 仍会主动载入一次参数。
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	// 传感器选择和飞行状态输入用于设备切换、约束启停及系统状态采样。
	uORB::Subscription _sensor_selection_sub{ORB_ID(sensor_selection)};
	uORB::Subscription _status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _launch_detection_status_sub{ORB_ID(launch_detection_status)};

	// 支持外部原点/位置/航向/风估计命令，并向命令来源发送处理结果。
	uORB::Subscription _vehicle_command_sub{ORB_ID(vehicle_command)};
	uORB::Publication<vehicle_command_ack_s> _vehicle_command_ack_pub{ORB_ID(vehicle_command_ack)};

	// 普通 Subscription 由 Run() 主动读取；CallbackWorkItem 在新消息到来时唤醒本工作项。
	// 单实例使用 sensor_combined，多实例绑定指定的 vehicle_imu 实例。
	uORB::SubscriptionCallbackWorkItem _sensor_combined_sub{this, ORB_ID(sensor_combined)};
	uORB::SubscriptionCallbackWorkItem _vehicle_imu_sub{this, ORB_ID(vehicle_imu)};

#if defined(CONFIG_EKF2_RANGE_FINDER)
	hrt_abstime _status_rng_hgt_pub_last {0};

	uORB::PublicationMulti<estimator_aid_source1d_s> _estimator_aid_src_rng_hgt_pub{ORB_ID(estimator_aid_src_rng_hgt)};

	// 保存多个测距订阅；_distance_sensor_selected 为选中实例，-1 表示需要选择。
	uORB::SubscriptionMultiArray<distance_sensor_s> _distance_sensor_subs{ORB_ID::distance_sensor};
	hrt_abstime _last_range_sensor_update{0};
	int _distance_sensor_selected{-1}; // because we can have several distance sensor instances with different orientations
#endif // CONFIG_EKF2_RANGE_FINDER

	// 回调注册失败时 Run() 会延迟重试，成功后主要由 IMU 消息驱动。
	bool _callback_registered{false};

	hrt_abstime _last_event_flags_publish{0};
	hrt_abstime _last_status_flags_publish{0};

	// 缓存三类位域，用于检测状态变化；对应 changes 记录变化次数而非传感器数量。
	uint64_t _filter_control_status{0};
	uint32_t _filter_fault_status{0};
	uint32_t _innov_check_fail_status{0};

	uint32_t _filter_control_status_changes{0};
	uint32_t _filter_fault_status_changes{0};
	uint32_t _innov_check_fail_status_changes{0};
	uint32_t _filter_information_event_changes{0};

	// 日志输出：相对时间戳、信息事件、创新/检验量/方差、偏置、状态和健康报告。
	// PublicationMulti 允许多个 EKF 在同名 topic 的不同实例号上发布。
	uORB::PublicationMulti<ekf2_timestamps_s>            _ekf2_timestamps_pub{ORB_ID(ekf2_timestamps)};
	uORB::PublicationMultiData<estimator_event_flags_s>  _estimator_event_flags_pub{ORB_ID(estimator_event_flags)};
	uORB::PublicationMulti<estimator_innovations_s>      _estimator_innovation_test_ratios_pub{ORB_ID(estimator_innovation_test_ratios)};
	uORB::PublicationMulti<estimator_innovations_s>      _estimator_innovation_variances_pub{ORB_ID(estimator_innovation_variances)};
	uORB::PublicationMulti<estimator_innovations_s>      _estimator_innovations_pub{ORB_ID(estimator_innovations)};
	uORB::PublicationMulti<estimator_sensor_bias_s>      _estimator_sensor_bias_pub{ORB_ID(estimator_sensor_bias)};
	uORB::PublicationMulti<estimator_states_s>           _estimator_states_pub{ORB_ID(estimator_states)};
	uORB::PublicationMulti<estimator_status_flags_s>     _estimator_status_flags_pub{ORB_ID(estimator_status_flags)};
	uORB::PublicationMulti<estimator_status_s>           _estimator_status_pub{ORB_ID(estimator_status)};

	uORB::PublicationMulti<estimator_aid_source1d_s> _estimator_aid_src_fake_hgt_pub{ORB_ID(estimator_aid_src_fake_hgt)};
	uORB::PublicationMulti<estimator_aid_source2d_s> _estimator_aid_src_fake_pos_pub{ORB_ID(estimator_aid_src_fake_pos)};

	// publications with topic dependent on multi-mode
	// 具体 topic ID 在构造函数中选择；单实例可直接供控制器消费。
	uORB::PublicationMulti<vehicle_attitude_s>           _attitude_pub;
	uORB::PublicationMulti<vehicle_local_position_s>     _local_position_pub;
	uORB::PublicationMulti<vehicle_global_position_s>    _global_position_pub;
	uORB::PublicationMulti<vehicle_odometry_s>           _odometry_pub;

#if defined(CONFIG_EKF2_WIND)
	uORB::PublicationMulti<wind_s>              _wind_pub;
#endif // CONFIG_EKF2_WIND

#if defined(CONFIG_EKF2_GNSS)

	// 大地水准面高度差 = 椭球高 - 海拔高；低通以平滑两种高度输出的转换。
	uint64_t _last_geoid_height_update_us{0};
	static constexpr float kGeoidHeightLpfTimeConstant = 10.f;
	AlphaFilter<float> _geoid_height_lpf;  ///< height offset between AMSL and ellipsoid

	hrt_abstime _last_gps_status_published{0};

	hrt_abstime _status_gnss_hgt_pub_last{0};
	hrt_abstime _status_gnss_pos_pub_last{0};
	hrt_abstime _status_gnss_vel_pub_last{0};

	float _last_gnss_hgt_bias_published{};

	// 上游输出的 GNSS 消息已包含经纬度、NED 速度、精度及可选双天线航向。
	uORB::Subscription _vehicle_gps_position_sub{ORB_ID(vehicle_gps_position)};

	uORB::PublicationMulti<estimator_bias_s> _estimator_gnss_hgt_bias_pub{ORB_ID(estimator_gnss_hgt_bias)};
	uORB::PublicationMulti<estimator_gps_status_s> _estimator_gps_status_pub{ORB_ID(estimator_gps_status)};
	uORB::PublicationMulti<estimator_aid_source1d_s> _estimator_aid_src_gnss_hgt_pub{ORB_ID(estimator_aid_src_gnss_hgt)};
	uORB::PublicationMulti<estimator_aid_source2d_s> _estimator_aid_src_gnss_pos_pub{ORB_ID(estimator_aid_src_gnss_pos)};
	uORB::PublicationMulti<estimator_aid_source3d_s> _estimator_aid_src_gnss_vel_pub{ORB_ID(estimator_aid_src_gnss_vel)};

	// GSF 多模型航向诊断，与 GNSS 双天线航向观测诊断分别发布。
	uORB::PublicationMulti<yaw_estimator_status_s> _yaw_est_pub{ORB_ID(yaw_estimator_status)};

# if defined(CONFIG_EKF2_GNSS_YAW)
	hrt_abstime _status_gnss_yaw_pub_last {0};
	uORB::PublicationMulti<estimator_aid_source1d_s> _estimator_aid_src_gnss_yaw_pub {ORB_ID(estimator_aid_src_gnss_yaw)};
# endif // CONFIG_EKF2_GNSS_YAW
#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_GRAVITY_FUSION)
	hrt_abstime _status_gravity_pub_last {0};
	uORB::PublicationMulti<estimator_aid_source3d_s> _estimator_aid_src_gravity_pub{ORB_ID(estimator_aid_src_gravity)};
#endif // CONFIG_EKF2_GRAVITY_FUSION

	// 一个封装实例持有一个算法实例：状态、协方差、历史缓冲和输出预测器均在其中。
	Ekf _ekf;

	// 指向 _ekf 内部参数对象；ParamExt 参数包装器直接绑定这些字段。
	parameters *_params;	///< pointer to ekf parameter struct (located in _ekf class instance)

	// DEFINE_PARAMETERS 生成参数成员和更新逻辑。
	// ParamFloat/ParamBool 保存模块层参数；ParamExtFloat/ParamExtInt 绑定算法参数存储。
	// 参数名称、默认值、范围和单位可对照 module.yaml 与 params_*.yaml。
	// 以下 DELAY 通常为 ms，PREDICT_US 为 us，NOISE 通常为标准差；具体以参数定义为准。
	DEFINE_PARAMETERS(
		// 详细日志、预测节拍、延迟窗口、IMU 学习/融合开关和速度数值限制。
		(ParamBool<px4::params::EKF2_LOG_VERBOSE>) _param_ekf2_log_verbose,
		(ParamExtInt<px4::params::EKF2_PREDICT_US>) _param_ekf2_predict_us,
		(ParamExtFloat<px4::params::EKF2_DELAY_MAX>) _param_ekf2_delay_max,
		(ParamExtInt<px4::params::EKF2_IMU_CTRL>) _param_ekf2_imu_ctrl,
		(ParamExtFloat<px4::params::EKF2_VEL_LIM>) _param_ekf2_vel_lim,

#if defined(CONFIG_EKF2_AUXVEL)
		(ParamExtFloat<px4::params::EKF2_AVEL_DELAY>)
		_param_ekf2_avel_delay,	///< auxiliary velocity measurement delay relative to the IMU (mSec)
#endif // CONFIG_EKF2_AUXVEL

		(ParamExtFloat<px4::params::EKF2_GYR_NOISE>)
		_param_ekf2_gyr_noise,	///< IMU angular rate noise used for covariance prediction (rad/sec)
		(ParamExtFloat<px4::params::EKF2_ACC_NOISE>)
		_param_ekf2_acc_noise,	///< IMU acceleration noise use for covariance prediction (m/sec**2)

		// process noise
		// 零偏过程噪声决定预测时允许零偏变化的速度，与观测噪声的作用不同。
		(ParamExtFloat<px4::params::EKF2_GYR_B_NOISE>)
		_param_ekf2_gyr_b_noise,	///< process noise for IMU rate gyro bias prediction (rad/sec**2)
		(ParamExtFloat<px4::params::EKF2_ACC_B_NOISE>)
		_param_ekf2_acc_b_noise,///< process noise for IMU accelerometer bias prediction (m/sec**3)

#if defined(CONFIG_EKF2_WIND)
		// 水平风状态过程噪声，允许风估计随环境变化而调整。
		(ParamExtFloat<px4::params::EKF2_WIND_NSD>) _param_ekf2_wind_nsd,
#endif // CONFIG_EKF2_WIND

		(ParamExtFloat<px4::params::EKF2_NOAID_NOISE>) _param_ekf2_noaid_noise,

#if defined(CONFIG_EKF2_GNSS)
		// GNSS_CTRL 为观测通道位掩码；MODE 决定融合重置策略；DELAY 用于时间补偿。
		(ParamExtInt<px4::params::EKF2_GPS_CTRL>) _param_ekf2_gps_ctrl,
		(ParamExtInt<px4::params::EKF2_GPS_MODE>) _param_ekf2_gps_mode,
		(ParamExtFloat<px4::params::EKF2_GPS_DELAY>) _param_ekf2_gps_delay,

		// 天线机体系位置用于与 IMU 安装位置求差，补偿旋转引起的杆臂速度/位置差。
		(ParamExtFloat<px4::params::EKF2_GPS_POS_X>) _param_ekf2_gps_pos_x,
		(ParamExtFloat<px4::params::EKF2_GPS_POS_Y>) _param_ekf2_gps_pos_y,
		(ParamExtFloat<px4::params::EKF2_GPS_POS_Z>) _param_ekf2_gps_pos_z,

		(ParamExtFloat<px4::params::EKF2_GPS_V_NOISE>) _param_ekf2_gps_v_noise,
		(ParamExtFloat<px4::params::EKF2_GPS_P_NOISE>) _param_ekf2_gps_p_noise,

		// GATE 设置创新一致性阈值，CHECK/REQ_* 设置输入质量检查及健康持续时间。
		(ParamExtFloat<px4::params::EKF2_GPS_P_GATE>) _param_ekf2_gps_p_gate,
		(ParamExtFloat<px4::params::EKF2_GPS_V_GATE>) _param_ekf2_gps_v_gate,

		(ParamExtInt<px4::params::EKF2_GPS_CHECK>) _param_ekf2_gps_check,
		(ParamExtFloat<px4::params::EKF2_REQ_EPH>)    _param_ekf2_req_eph,
		(ParamExtFloat<px4::params::EKF2_REQ_EPV>)    _param_ekf2_req_epv,
		(ParamExtFloat<px4::params::EKF2_REQ_SACC>)   _param_ekf2_req_sacc,
		(ParamExtInt<px4::params::EKF2_REQ_NSATS>)    _param_ekf2_req_nsats,
		(ParamExtFloat<px4::params::EKF2_REQ_PDOP>)   _param_ekf2_req_pdop,
		(ParamExtFloat<px4::params::EKF2_REQ_HDRIFT>) _param_ekf2_req_hdrift,
		(ParamExtFloat<px4::params::EKF2_REQ_VDRIFT>) _param_ekf2_req_vdrift,
		(ParamExtInt<px4::params::EKF2_REQ_FIX>)      _param_ekf2_req_fix,
		(ParamFloat<px4::params::EKF2_REQ_GPS_H>)     _param_ekf2_req_gps_h,

		// Used by EKF-GSF experimental yaw estimator
		// GSF_TAS 用于备用航向估计器；GPS_YAW_OFF 是双天线基线相对机头的安装偏角。
		(ParamExtFloat<px4::params::EKF2_GSF_TAS>) _param_ekf2_gsf_tas,
		(ParamFloat<px4::params::EKF2_GPS_YAW_OFF>) _param_ekf2_gps_yaw_off,
#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_BAROMETER)
		// 气压高度的观测配置；GND_* 处理近地压力扰动的相关条件。
		(ParamExtInt<px4::params::EKF2_BARO_CTRL>) _param_ekf2_baro_ctrl,///< barometer control selection
		(ParamExtFloat<px4::params::EKF2_BARO_DELAY>) _param_ekf2_baro_delay,
		(ParamExtFloat<px4::params::EKF2_BARO_NOISE>) _param_ekf2_baro_noise,
		(ParamExtFloat<px4::params::EKF2_BARO_GATE>) _param_ekf2_baro_gate,
		(ParamExtFloat<px4::params::EKF2_GND_EFF_DZ>) _param_ekf2_gnd_eff_dz,
		(ParamExtFloat<px4::params::EKF2_GND_MAX_HGT>) _param_ekf2_gnd_max_hgt,

# if defined(CONFIG_EKF2_BARO_COMPENSATION)
		// Corrections for static pressure position error where Ps_error = Ps_meas - Ps_truth
		// 按机体不同来流方向配置静压误差系数，利用相对气流估计补偿压力偏差。
		(ParamExtFloat<px4::params::EKF2_ASPD_MAX>) _param_ekf2_aspd_max,
		(ParamExtFloat<px4::params::EKF2_PCOEF_XP>) _param_ekf2_pcoef_xp,
		(ParamExtFloat<px4::params::EKF2_PCOEF_XN>) _param_ekf2_pcoef_xn,
		(ParamExtFloat<px4::params::EKF2_PCOEF_YP>) _param_ekf2_pcoef_yp,
		(ParamExtFloat<px4::params::EKF2_PCOEF_YN>) _param_ekf2_pcoef_yn,
		(ParamExtFloat<px4::params::EKF2_PCOEF_Z>) _param_ekf2_pcoef_z,
# endif // CONFIG_EKF2_BARO_COMPENSATION
#endif // CONFIG_EKF2_BAROMETER

#if defined(CONFIG_EKF2_AIRSPEED)
		// 空速噪声以等效空速配置，算法会结合 EAS/TAS 比例处理真空速不确定度。
		(ParamExtFloat<px4::params::EKF2_ASP_DELAY>)
		_param_ekf2_asp_delay, ///< airspeed measurement delay relative to the IMU (mSec)
		(ParamExtFloat<px4::params::EKF2_TAS_GATE>)
		_param_ekf2_tas_gate, ///< True Airspeed innovation consistency gate size (STD)
		(ParamExtFloat<px4::params::EKF2_EAS_NOISE>)
		_param_ekf2_eas_noise, ///< measurement noise used for airspeed fusion (m/sec)

		// control of airspeed fusion
		(ParamExtFloat<px4::params::EKF2_ARSP_THR>)
		_param_ekf2_arsp_thr, ///< A value of zero will disabled airspeed fusion. Any positive value sets the minimum airspeed which will be used (m/sec)
#endif // CONFIG_EKF2_AIRSPEED

#if defined(CONFIG_EKF2_SIDESLIP)
		// 合成侧滑约束的开关、观测噪声和创新门限。
		(ParamExtFloat<px4::params::EKF2_BETA_GATE>) _param_ekf2_beta_gate,
		(ParamExtFloat<px4::params::EKF2_BETA_NOISE>) _param_ekf2_beta_noise,
		(ParamExtInt<px4::params::EKF2_FUSE_BETA>) _param_ekf2_fuse_beta,
#endif // CONFIG_EKF2_SIDESLIP

#if defined(CONFIG_EKF2_MAGNETOMETER)
		// MAG_E/B_NOISE 为磁场状态过程噪声；MAG_NOISE/HEAD_NOISE 为观测噪声。
		// MAG_TYPE 选择使用方式，MAG_CHECK/CHK_* 控制磁场强度、倾角等输入检查。
		(ParamExtFloat<px4::params::EKF2_MAG_DELAY>) _param_ekf2_mag_delay,
		(ParamExtFloat<px4::params::EKF2_MAG_E_NOISE>) _param_ekf2_mag_e_noise,
		(ParamExtFloat<px4::params::EKF2_MAG_B_NOISE>) _param_ekf2_mag_b_noise,
		(ParamExtFloat<px4::params::EKF2_HEAD_NOISE>) _param_ekf2_head_noise,
		(ParamExtFloat<px4::params::EKF2_MAG_NOISE>) _param_ekf2_mag_noise,
		(ParamExtFloat<px4::params::EKF2_MAG_DECL>) _param_ekf2_mag_decl,
		(ParamExtFloat<px4::params::EKF2_HDG_GATE>) _param_ekf2_hdg_gate,
		(ParamExtFloat<px4::params::EKF2_MAG_GATE>) _param_ekf2_mag_gate,
		(ParamExtInt<px4::params::EKF2_DECL_TYPE>) _param_ekf2_decl_type,
		(ParamExtInt<px4::params::EKF2_MAG_TYPE>) _param_ekf2_mag_type,
		(ParamExtFloat<px4::params::EKF2_MAG_ACCLIM>) _param_ekf2_mag_acclim,
		(ParamExtInt<px4::params::EKF2_MAG_CHECK>) _param_ekf2_mag_check,
		(ParamExtFloat<px4::params::EKF2_MAG_CHK_STR>) _param_ekf2_mag_chk_str,
		(ParamExtFloat<px4::params::EKF2_MAG_CHK_INC>) _param_ekf2_mag_chk_inc,
		(ParamExtInt<px4::params::EKF2_SYNT_MAG_Z>) _param_ekf2_synt_mag_z,
#endif // CONFIG_EKF2_MAGNETOMETER

		// 多个高度源可同时融合；参考源决定长期高度基准，其余源可估计相对偏置。
		(ParamExtInt<px4::params::EKF2_HGT_REF>) _param_ekf2_hgt_ref,    ///< selects the primary source for height data

		(ParamExtInt<px4::params::EKF2_NOAID_TOUT>)
		_param_ekf2_noaid_tout,	///< maximum lapsed time from last fusion of measurements that constrain drift before the EKF will report the horizontal nav solution invalid (uSec)

#if defined(CONFIG_EKF2_TERRAIN) || defined(CONFIG_EKF2_OPTICAL_FLOW) || defined(CONFIG_EKF2_RANGE_FINDER)
		(ParamExtFloat<px4::params::EKF2_MIN_RNG>) _param_ekf2_min_rng,
#endif // CONFIG_EKF2_TERRAIN || CONFIG_EKF2_OPTICAL_FLOW || CONFIG_EKF2_RANGE_FINDER
#if defined(CONFIG_EKF2_TERRAIN)
		// 地形过程噪声和坡度参数描述地面高度随时间/水平运动变化的不确定度。
		(ParamExtFloat<px4::params::EKF2_TERR_NOISE>) _param_ekf2_terr_noise,
		(ParamExtFloat<px4::params::EKF2_TERR_GRAD>) _param_ekf2_terr_grad,
#endif // CONFIG_EKF2_TERRAIN
#if defined(CONFIG_EKF2_RANGE_FINDER)
		// range finder fusion
		// 包含测距本底噪声、随距离增长项、条件辅助速度/高度限制和质量持续时间。
		(ParamExtInt<px4::params::EKF2_RNG_CTRL>) _param_ekf2_rng_ctrl,
		(ParamExtFloat<px4::params::EKF2_RNG_DELAY>) _param_ekf2_rng_delay,
		(ParamExtFloat<px4::params::EKF2_RNG_NOISE>) _param_ekf2_rng_noise,
		(ParamExtFloat<px4::params::EKF2_RNG_SFE>) _param_ekf2_rng_sfe,
		(ParamExtFloat<px4::params::EKF2_RNG_GATE>) _param_ekf2_rng_gate,
		(ParamExtFloat<px4::params::EKF2_RNG_PITCH>) _param_ekf2_rng_pitch,
		(ParamExtFloat<px4::params::EKF2_RNG_A_VMAX>) _param_ekf2_rng_a_vmax,
		(ParamExtFloat<px4::params::EKF2_RNG_A_HMAX>) _param_ekf2_rng_a_hmax,
		(ParamExtFloat<px4::params::EKF2_RNG_QLTY_T>) _param_ekf2_rng_qlty_t,
		(ParamExtFloat<px4::params::EKF2_RNG_K_GATE>) _param_ekf2_rng_k_gate,
		(ParamExtFloat<px4::params::EKF2_RNG_FOG>) _param_ekf2_rng_fog,
		(ParamExtFloat<px4::params::EKF2_RNG_POS_X>) _param_ekf2_rng_pos_x,
		(ParamExtFloat<px4::params::EKF2_RNG_POS_Y>) _param_ekf2_rng_pos_y,
		(ParamExtFloat<px4::params::EKF2_RNG_POS_Z>) _param_ekf2_rng_pos_z,
#endif // CONFIG_EKF2_RANGE_FINDER

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
		// vision estimate fusion
		// EV_CTRL 按位选择位置/速度/高度/航向；EV_QMIN 是质量要求，EV_DELAY 用于时间匹配。
		// EV_NOISE_MD 决定使用消息方差还是参数噪声，具体分量处理在 UpdateExtVisionSample()。
		(ParamExtFloat<px4::params::EKF2_EV_DELAY>)
		_param_ekf2_ev_delay, ///< off-board vision measurement delay relative to the IMU (mSec)

		(ParamExtInt<px4::params::EKF2_EV_CTRL>) _param_ekf2_ev_ctrl,	 ///< external vision (EV) control selection
		(ParamInt<px4::params::EKF2_EV_NOISE_MD>) _param_ekf2_ev_noise_md, ///< determine source of vision observation noise
		(ParamExtInt<px4::params::EKF2_EV_QMIN>) _param_ekf2_ev_qmin,
		(ParamExtFloat<px4::params::EKF2_EVP_NOISE>)
		_param_ekf2_evp_noise, ///< default position observation noise for exernal vision measurements (m)
		(ParamExtFloat<px4::params::EKF2_EVV_NOISE>)
		_param_ekf2_evv_noise, ///< default velocity observation noise for exernal vision measurements (m/s)
		(ParamExtFloat<px4::params::EKF2_EVA_NOISE>)
		_param_ekf2_eva_noise, ///< default angular observation noise for exernal vision measurements (rad)
		(ParamExtFloat<px4::params::EKF2_EVV_GATE>)
		_param_ekf2_evv_gate, ///< external vision velocity innovation consistency gate size (STD)
		(ParamExtFloat<px4::params::EKF2_EVP_GATE>)
		_param_ekf2_evp_gate, ///< external vision position innovation consistency gate size (STD)

		// 视觉传感器参考点在机体系中的位置，用于补偿它与 IMU 之间的安装杆臂。
		(ParamExtFloat<px4::params::EKF2_EV_POS_X>)
		_param_ekf2_ev_pos_x, ///< X position of VI sensor focal point in body frame (m)
		(ParamExtFloat<px4::params::EKF2_EV_POS_Y>)
		_param_ekf2_ev_pos_y, ///< Y position of VI sensor focal point in body frame (m)
		(ParamExtFloat<px4::params::EKF2_EV_POS_Z>)
		_param_ekf2_ev_pos_z, ///< Z position of VI sensor focal point in body frame (m)
#endif // CONFIG_EKF2_EXTERNAL_VISION
#if defined(CONFIG_EKF2_OPTICAL_FLOW)
		// optical flow fusion
		// 光流噪声随质量变化；OF_GYR_SRC 选择旋转补偿来源，OF_POS_* 用于杆臂处理。
		(ParamExtInt<px4::params::EKF2_OF_CTRL>)
		_param_ekf2_of_ctrl, ///< optical flow fusion selection
		(ParamExtInt<px4::params::EKF2_OF_GYR_SRC>)
		_param_ekf2_of_gyr_src,
		(ParamExtFloat<px4::params::EKF2_OF_DELAY>)
		_param_ekf2_of_delay, ///< optical flow measurement delay relative to the IMU (mSec) - this is to the middle of the optical flow integration interval
		(ParamExtFloat<px4::params::EKF2_OF_N_MIN>)
		_param_ekf2_of_n_min, ///< best quality observation noise for optical flow LOS rate measurements (rad/sec)
		(ParamExtFloat<px4::params::EKF2_OF_N_MAX>)
		_param_ekf2_of_n_max, ///< worst quality observation noise for optical flow LOS rate measurements (rad/sec)
		(ParamExtInt<px4::params::EKF2_OF_QMIN>)
		_param_ekf2_of_qmin, ///< minimum acceptable quality integer from  the flow sensor when in air
		(ParamExtInt<px4::params::EKF2_OF_QMIN_GND>)
		_param_ekf2_of_qmin_gnd, ///< minimum acceptable quality integer from  the flow sensor when on ground
		(ParamExtFloat<px4::params::EKF2_OF_GATE>)
		_param_ekf2_of_gate, ///< optical flow fusion innovation consistency gate size (STD)
		(ParamExtFloat<px4::params::EKF2_OF_POS_X>)
		_param_ekf2_of_pos_x, ///< X position of optical flow sensor focal point in body frame (m)
		(ParamExtFloat<px4::params::EKF2_OF_POS_Y>)
		_param_ekf2_of_pos_y, ///< Y position of optical flow sensor focal point in body frame (m)
		(ParamExtFloat<px4::params::EKF2_OF_POS_Z>)
		_param_ekf2_of_pos_z, ///< Z position of optical flow sensor focal point in body frame (m)
#endif // CONFIG_EKF2_OPTICAL_FLOW

#if defined(CONFIG_EKF2_DRAG_FUSION)
		(ParamExtInt<px4::params::EKF2_DRAG_CTRL>) _param_ekf2_drag_ctrl,		///< drag fusion selection
		// Multi-rotor drag specific force fusion
		// BCOEF_* 为机体阻力模型系数，MCOEF 为旋翼动量阻力系数。
		(ParamExtFloat<px4::params::EKF2_DRAG_NOISE>)
		_param_ekf2_drag_noise,	///< observation noise variance for drag specific force measurements (m/sec**2)**2
		(ParamExtFloat<px4::params::EKF2_BCOEF_X>) _param_ekf2_bcoef_x,		///< ballistic coefficient along the X-axis (kg/m**2)
		(ParamExtFloat<px4::params::EKF2_BCOEF_Y>) _param_ekf2_bcoef_y,		///< ballistic coefficient along the Y-axis (kg/m**2)
		(ParamExtFloat<px4::params::EKF2_MCOEF>) _param_ekf2_mcoef,		///< propeller momentum drag coefficient (1/s)
#endif // CONFIG_EKF2_DRAG_FUSION

#if defined(CONFIG_EKF2_GRAVITY_FUSION)
		(ParamExtFloat<px4::params::EKF2_GRAV_NOISE>) _param_ekf2_grav_noise,
#endif // CONFIG_EKF2_GRAVITY_FUSION

		// sensor positions in body frame
		// 机体系 FRD：X 前、Y 右、Z 下；位置单位为米，用于补偿传感器安装偏移。
		(ParamExtFloat<px4::params::EKF2_IMU_POS_X>) _param_ekf2_imu_pos_x,		///< X position of IMU in body frame (m)
		(ParamExtFloat<px4::params::EKF2_IMU_POS_Y>) _param_ekf2_imu_pos_y,		///< Y position of IMU in body frame (m)
		(ParamExtFloat<px4::params::EKF2_IMU_POS_Z>) _param_ekf2_imu_pos_z,		///< Z position of IMU in body frame (m)

		// IMU switch on bias parameters
		// 初始不确定度决定滤波器启动时对零偏和倾斜误差的信任程度。
		(ParamExtFloat<px4::params::EKF2_GBIAS_INIT>)
		_param_ekf2_gbias_init,	///< 1-sigma gyro bias uncertainty at switch on (rad/sec)
		(ParamExtFloat<px4::params::EKF2_ABIAS_INIT>)
		_param_ekf2_abias_init,	///< 1-sigma accelerometer bias uncertainty at switch on (m/sec**2)
		(ParamExtFloat<px4::params::EKF2_ANGERR_INIT>)
		_param_ekf2_angerr_init,	///< 1-sigma tilt error after initial alignment using gravity vector (rad)

		// EKF accel bias learning control
		// 强加速度/快速转动时可抑制零偏学习，避免把运动模型误差误认为传感器零偏。
		(ParamExtFloat<px4::params::EKF2_ABL_LIM>) _param_ekf2_abl_lim,	///< Accelerometer bias learning limit (m/s**2)
		(ParamExtFloat<px4::params::EKF2_ABL_ACCLIM>)
		_param_ekf2_abl_acclim,	///< Maximum IMU accel magnitude that allows IMU bias learning (m/s**2)
		(ParamExtFloat<px4::params::EKF2_ABL_GYRLIM>)
		_param_ekf2_abl_gyrlim,	///< Maximum IMU gyro angular rate magnitude that allows IMU bias learning (m/s**2)
		(ParamExtFloat<px4::params::EKF2_ABL_TAU>)
		_param_ekf2_abl_tau,	///< Time constant used to inhibit IMU delta velocity bias learning (sec)

		(ParamExtFloat<px4::params::EKF2_GYR_B_LIM>) _param_ekf2_gyr_b_lim,	///< Gyro bias learning limit (rad/s)

		// output predictor filter time constants
		// 控制当前输出追踪延迟 EKF 校正的平滑速度，单位秒。
		(ParamFloat<px4::params::EKF2_TAU_VEL>) _param_ekf2_tau_vel,
		(ParamFloat<px4::params::EKF2_TAU_POS>) _param_ekf2_tau_pos
	)
};
#endif // !EKF2_HPP
