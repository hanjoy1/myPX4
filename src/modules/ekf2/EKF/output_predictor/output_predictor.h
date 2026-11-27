/****************************************************************************
 *
 *   Copyright (c) 2022-2024 PX4. All rights reserved.
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

#ifndef EKF_OUTPUT_PREDICTOR_H
#define EKF_OUTPUT_PREDICTOR_H

#include <matrix/math.hpp>

#include "../RingBuffer.h"

#include <lib/geo/geo.h>
#include <lib/lat_lon_alt/lat_lon_alt.hpp>

class OutputPredictor
{
public:
	// 输出预测器不是第二套 EKF，而是由主 EKF 定期校正的高频捷联惯导器。
	// 它使用每一帧原始 IMU 将延迟融合状态传播到当前时刻，供控制器低延迟使用。
	OutputPredictor()
	{
		reset();
	};

	~OutputPredictor() = default;

	// 将输出状态历史整体对齐到融合时刻的 EKF 状态，主要用于滤波器首次初始化。
	void alignOutputFilter(const matrix::Quatf &quat_state, const matrix::Vector3f &vel_state,
			       const LatLonAlt &gpos_state);
	/*
	* Implement a strapdown INS algorithm using the latest IMU data at the current time horizon.
	* Buffer the INS states and calculate the difference with the EKF states at the delayed fusion time horizon.
	* Calculate delta angle, delta velocity and velocity corrections from the differences and apply them at the
	* current time horizon so that the INS states track the EKF states at the delayed fusion time horizon.
	* The inspiration for using a complementary filter to correct for time delays in the EKF
	* is based on the work by A Khosravian:
	* “Recursive Attitude Estimation in the Presence of Multi-rate and Multi-delay Vector Measurements”
	* A Khosravian, J Trumpf, R Mahony, T Hamel, Australian National University
	*
	* 中文：使用当前时刻的全速 IMU 执行捷联惯导预测。输出状态会被保存到历史
	* 缓冲区，再与延迟融合时刻的 EKF 状态比较。比较得到的姿态、速度和位置误差
	* 通过互补滤波反馈到当前输出，从而补偿 EKF 的观测延迟。
	*/
	void calculateOutputStates(const uint64_t time_us, const matrix::Vector3f &delta_angle, const float delta_angle_dt,
				   const matrix::Vector3f &delta_velocity, const float delta_velocity_dt);

	void correctOutputStates(const uint64_t time_delayed_us,
				 const matrix::Quatf &quat_state, const matrix::Vector3f &vel_state, const LatLonAlt &gpos_state,
				 const matrix::Vector3f &gyro_bias, const matrix::Vector3f &accel_bias);

	// 主 EKF 发生状态 reset 时，立即把相同变化量施加到整条输出历史，避免两者失配。
	void resetQuaternion(const matrix::Quatf &quat_change);

	void resetHorizontalVelocityTo(const matrix::Vector2f &delta_horz_vel);
	void resetVerticalVelocityTo(float delta_vert_vel);

	void resetLatLonTo(const double &new_latitude, const double &new_longitude);
	void resetAltitudeTo(float new_altitude, float vert_pos_change);

	void print_status();

	bool allocate(uint8_t size)
	{
		// 两个缓冲区必须与延迟 IMU 缓冲区采用相同长度，才能近似对应同一时间轴。
		if (_output_buffer.allocate(size) && _output_vert_buffer.allocate(size)) {
			reset();
			return true;
		}

		return false;
	}

	void reset();

	// 以下 getter 返回当前时刻输出，而不是主 EKF 延迟融合时刻的状态。
	const matrix::Quatf &getQuaternion() const { return _output_new.quat_nominal; }

	matrix::Vector3f getAngularVelocityAndResetAccumulator();

	// 仅使用去零偏陀螺积分得到的航向，不包含主 EKF 的外部航向修正。
	float getUnaidedYaw() const { return _unaided_yaw; }

	// 返回机体原点在 NED 系中的速度；内部状态位于 IMU 点，因此要减去旋转杆臂速度。
	matrix::Vector3f getVelocity() const { return _output_new.vel - _vel_imu_rel_body_ned; }

	// 返回上次清零以来 IMU 点在 NED 系中的平均速度导数。
	matrix::Vector3f getVelocityDerivative() const;

	void resetVelocityDerivativeAccumulation();

	// 返回机体原点垂直位置导数，使用下方独立的垂直速度积分通道。
	float getVerticalPositionDerivative() const { return _output_vert_new.vert_vel - _vel_imu_rel_body_ned(2); }

	LatLonAlt getLatLonAlt() const
	{
		// 将“机体原点到 IMU”的安装偏移旋转到 NED 系。
		const matrix::Vector3f pos_offset_earth{_R_to_earth_now * _imu_pos_body};
		// 内部位置对应 IMU 点，减去安装偏移后得到机体原点的全球位置。
		return _global_ref + (_output_new.pos - pos_offset_earth);
	}

	// 返回输出预测器相对延迟 EKF 的姿态、速度、位置跟踪误差模长，单位依次为 rad、m/s、m。
	const matrix::Vector3f &getOutputTrackingError() const { return _output_tracking_error; }

	void set_imu_offset(const matrix::Vector3f &offset) { _imu_pos_body = offset; }
	void set_pos_correction_tc(const float tau) { _pos_tau = tau; }
	void set_vel_correction_tc(const float tau) { _vel_tau = tau; }

private:

	/*
	* Calculate a correction to be applied to vert_vel that casues vert_vel_integ to track the EKF
	* down position state at the fusion time horizon using an alternative algorithm to what
	* is used for the vel and pos state tracking. The algorithm applies a correction to the vert_vel
	* state history and propagates vert_vel_integ forward in time using the corrected vert_vel history.
	* This provides an alternative vertical velocity output that is closer to the first derivative
	* of the position but does degrade tracking relative to the EKF state.
	*
	* 中文：垂直备用通道先修正历史垂直速度，再重新向前积分位置。这样输出的垂直
	* 速度更接近位置的一阶导数，但会牺牲一点对 EKF 速度状态的直接跟踪精度。
	*/
	void applyCorrectionToVerticalOutputBuffer(float vert_vel_correction, const float pos_ref_change);

	/*
	* Calculate corrections to be applied to vel and pos output state history.
	* The vel and pos state history are corrected individually so they track the EKF states at
	* the fusion time horizon. This option provides the most accurate tracking of EKF states.
	*
	* 中文：普通通道分别修正整条速度和位置历史，使其直接跟踪 EKF 状态。
	*/
	void applyCorrectionToOutputBuffer(const matrix::Vector3f &vel_correction, const matrix::Vector3f &pos_correction);

	// 平方辅助函数，供自动生成风格的公式使用。
	static constexpr float sq(float var) { return var * var; }

	struct outputSample {
		uint64_t         time_us{0};                       ///< 样本时间戳 (us)
		matrix::Quatf    quat_nominal{1.f, 0.f, 0.f, 0.f}; ///< 描述机体姿态的名义四元数
		matrix::Vector3f vel{0.f, 0.f, 0.f};               ///< 地固 NED 系速度估计 (m/s)
		matrix::Vector3f pos{0.f, 0.f, 0.f};               ///< 相对 _global_ref 的 NED 位置估计 (m)
	};

	// 独立垂直通道：保存垂直速度及其积分，用于提供与位置导数更一致的垂直速度。
	struct outputVert {
		uint64_t time_us{0};          ///< 样本时间戳 (us)
		float    vert_vel{0.f};       ///< 备用算法计算的 NED 垂直速度 (m/s)
		float    vert_vel_integ{0.f}; ///< 垂直速度积分得到的位置 (m)
		float    dt{0.f};             ///< 本段累计积分时间 (s)
	};

	// _output_new.pos 是相对这个全球参考点的 NED 位移。
	LatLonAlt _global_ref{0.0, 0.0, 0.f};

	// 保存从延迟融合时刻到当前时刻的输出快照，长度与 IMU 延迟缓冲区一致。
	RingBuffer<outputSample> _output_buffer{12};
	RingBuffer<outputVert> _output_vert_buffer{12};

	// 最近一次由主 EKF 提供的 IMU 零偏，用于高频输出预测。
	matrix::Vector3f _accel_bias{};
	matrix::Vector3f _gyro_bias{};

	float _dt_update_states_avg{0.005f};  // 平均原始 IMU 更新周期 (s)
	float _dt_correct_states_avg{0.010f}; // 平均主 EKF 校正周期 (s)

	uint64_t _time_last_update_states_us{0};  ///< 最近一次高频输出预测时间 (us)
	uint64_t _time_last_correct_states_us{0}; ///< 最近一次主 EKF 反馈校正时间 (us)

	// 当前非延迟时间轴上的输出状态。
	outputSample _output_new{};		// 当前非延迟时间轴上的完整输出
	outputVert _output_vert_new{};		// 当前非延迟时间轴上的备用垂直输出
	matrix::Matrix3f _R_to_earth_now{};		// 当前时刻 body -> NED 旋转矩阵
	matrix::Vector3f _vel_imu_rel_body_ned{};		// IMU 因机体转动而相对机体原点产生的 NED 速度
	matrix::Vector3f _delta_vel_sum{};	// IMU 点在 NED 系中的累计速度增量 (m/s)
	float _delta_vel_dt{};			// 速度增量累计时间 (s)

	// 互补滤波器状态：姿态使用每帧角增量修正，速度和位置使用误差积分修正。
	matrix::Vector3f _delta_angle_corr{};	///< 每帧叠加的姿态跟踪角增量 (rad)
	matrix::Vector3f _vel_err_integ{};	///< 速度跟踪误差的离散累加量
	matrix::Vector3f _pos_err_integ{};	///< 位置跟踪误差的离散累加量

	matrix::Vector3f _output_tracking_error{}; ///< 姿态、速度、位置跟踪误差模长 (rad, m/s, m)

	matrix::Vector3f _imu_pos_body{};                ///< IMU 相对机体原点的 FRD 安装位置 (m)

	matrix::Quatf _delta_angle_sum{};
	float _delta_angle_sum_dt{0.f};
	float _unaided_yaw{};

	// 输出互补滤波时间常数。越小跟踪越快，越大输出越平滑。
	float _vel_tau{0.25f};                   ///< 速度状态校正时间常数 (s)
	float _pos_tau{0.25f};                   ///< 位置状态校正时间常数 (s)
};

#endif // !EKF_OUTPUT_PREDICTOR_H
