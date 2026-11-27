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

#include "output_predictor.h"

// 中文导读：主 EKF 为了对齐延迟观测而工作在过去，控制器却需要当前状态。
// 本类保存之后的 IMU 历史，将融合结果前推到现在，并平滑消除预测输出与 EKF
// 之间的误差，从而同时获得低延迟和长期不漂移的输出。

using matrix::AxisAnglef;
using matrix::Dcmf;
using matrix::Eulerf;
using matrix::Quatf;
using matrix::Vector2f;
using matrix::Vector3f;

void OutputPredictor::print_status()
{
	printf("[output predictor] IMU dt: %.6f, EKF dt: %.6f\n",
	       (double)_dt_update_states_avg, (double)_dt_correct_states_avg);

	const matrix::Quatf q_att = _output_buffer.get_newest().quat_nominal;
	const matrix::Eulerf euler = q_att;

	printf("[output predictor] orientation: [%.4f, %.4f, %.4f, %.4f] (Euler [%.3f, %.3f, %.3f])\n",
	       (double)q_att(0), (double)q_att(1), (double)q_att(2), (double)q_att(3),
	       (double)euler.phi(), (double)euler.theta(), (double)euler.psi());

	printf("[output predictor] velocity: [%.3f, %.3f, %.3f]\n",
	       (double)_output_buffer.get_newest().vel(0), (double)_output_buffer.get_newest().vel(1),
	       (double)_output_buffer.get_newest().vel(2));

	printf("[output predictor] position: [%.3f, %.3f, %.3f]\n",
	       (double)_output_buffer.get_newest().pos(0), (double)_output_buffer.get_newest().pos(1),
	       (double)_output_buffer.get_newest().pos(2));

	printf("[output predictor] tracking error, angular: %.6f rad, velocity: %.4f m/s, position: %.4f m\n",
	       (double)_output_tracking_error(0), (double)_output_tracking_error(1), (double)_output_tracking_error(2));

	printf("[output predictor] output buffer: %d/%d (%d Bytes)\n",
	       _output_buffer.entries(), _output_buffer.get_length(), _output_buffer.get_total_size());

	printf("[output predictor] output vert buffer: %d/%d (%d Bytes)\n",
	       _output_vert_buffer.entries(), _output_vert_buffer.get_length(), _output_vert_buffer.get_total_size());
}

void OutputPredictor::alignOutputFilter(const Quatf &quat_state, const Vector3f &vel_state, const LatLonAlt &gpos_state)
{
	// 最旧输出样本与主 EKF 位于同一个延迟融合时间附近。
	const outputSample &output_delayed = _output_buffer.get_oldest();

	// 计算延迟时刻从预测器姿态旋转到 EKF 姿态所需的四元数差。
	Quatf q_delta{quat_state * output_delayed.quat_nominal.inversed()};
	q_delta.normalize();

	// 计算延迟时刻 EKF 与预测器的速度差。
	const Vector3f vel_delta = vel_state - output_delayed.vel;

	// 将延迟输出的局部位置归零，并把 EKF 全球位置设为新的输出参考点。
	const Vector3f pos_delta = -output_delayed.pos;
	_global_ref = gpos_state;

	// 同一个变化量施加到整条历史，保持历史样本之间的相对运动不变。
	for (uint8_t i = 0; i < _output_buffer.get_length(); i++) {
		_output_buffer[i].quat_nominal = q_delta * _output_buffer[i].quat_nominal;
		_output_buffer[i].quat_nominal.normalize();
		_output_buffer[i].vel += vel_delta;
		_output_buffer[i].pos += pos_delta;
	}

	_output_new = _output_buffer.get_newest();
}

void OutputPredictor::reset()
{
	// 清空当前输出、零偏、反馈修正和积分器；缓冲区保留已分配容量但清零内容。
	// TODO: who resets the output buffer content?
	_output_new = {};
	_output_vert_new = {};

	_accel_bias.setZero();
	_gyro_bias.setZero();

	_time_last_update_states_us = 0;
	_time_last_correct_states_us = 0;

	_R_to_earth_now.setIdentity();
	_vel_imu_rel_body_ned.setZero();
	_delta_vel_sum.setZero();
	_delta_vel_dt = 0.f;

	_delta_angle_sum.setIdentity();
	_delta_angle_sum_dt = 0.f;

	_delta_angle_corr.setZero();

	_vel_err_integ.setZero();
	_pos_err_integ.setZero();

	_output_tracking_error.setZero();

	for (uint8_t index = 0; index < _output_buffer.get_length(); index++) {
		_output_buffer[index] = {};
	}

	for (uint8_t index = 0; index < _output_vert_buffer.get_length(); index++) {
		_output_vert_buffer[index] = {};
	}
}

void OutputPredictor::resetQuaternion(const Quatf &quat_change)
{
	// 主 EKF 姿态 reset 后，将相同四元数变化左乘到每个历史输出。
	for (uint8_t i = 0; i < _output_buffer.get_length(); i++) {
		_output_buffer[i].quat_nominal = quat_change * _output_buffer[i].quat_nominal;
	}

	// _output_new 是缓冲区最新值的独立副本，因此也需要单独修正。
	_output_new.quat_nominal = quat_change * _output_new.quat_nominal;
}

void OutputPredictor::resetHorizontalVelocityTo(const Vector2f &delta_horz_vel)
{
	// 速度 reset 是估计坐标跳变而非真实加速度，故整条历史统一平移相同增量。
	for (uint8_t index = 0; index < _output_buffer.get_length(); index++) {
		_output_buffer[index].vel.xy() += delta_horz_vel;
	}

	_output_new.vel.xy() += delta_horz_vel;
}

void OutputPredictor::resetVerticalVelocityTo(float delta_vert_vel)
{
	// 普通输出和独立垂直通道都必须同步，避免两个对外速度产生不一致。
	for (uint8_t index = 0; index < _output_buffer.get_length(); index++) {
		_output_buffer[index].vel(2) += delta_vert_vel;
		_output_vert_buffer[index].vert_vel += delta_vert_vel;
	}

	_output_new.vel(2) += delta_vert_vel;
	_output_vert_new.vert_vel += delta_vert_vel;
}

void OutputPredictor::resetLatLonTo(const double &new_latitude, const double &new_longitude)
{
	// 局部 pos 不变，只替换它对应的全球经纬度参考。
	_global_ref.setLatitudeDeg(new_latitude);
	_global_ref.setLongitudeDeg(new_longitude);
}

void OutputPredictor::resetAltitudeTo(const float new_altitude, const float vert_pos_change)
{
	// 通过改变全球参考高度同步高度 reset；vert_pos_change 暂未参与当前实现。
	_global_ref.setAltitude(new_altitude);
}

void OutputPredictor::calculateOutputStates(const uint64_t time_us, const Vector3f &delta_angle,
		const float delta_angle_dt, const Vector3f &delta_velocity, const float delta_velocity_dt)
{
	// 对每个最新 IMU 样本执行实时捷联传播，形成供控制器使用的姿态、速度和位置。
	// 使用当前时间轴上的全速 IMU，并低通估计实际 IMU 更新周期。
	if (_time_last_update_states_us != 0) {
		const float dt = math::constrain((time_us - _time_last_update_states_us) * 1e-6f, 0.0001f, 0.03f);
		_dt_update_states_avg = 0.8f * _dt_update_states_avg + 0.2f * dt;
	}

	_time_last_update_states_us = time_us;

	// 去除主 EKF 估计的 IMU 零偏；角增量还叠加互补滤波产生的姿态跟踪修正。
	const Vector3f delta_angle_bias_scaled = _gyro_bias * delta_angle_dt;
	const Vector3f delta_angle_corrected(delta_angle - delta_angle_bias_scaled + _delta_angle_corr);

	const Vector3f delta_vel_bias_scaled = _accel_bias * delta_velocity_dt;
	const Vector3f delta_velocity_corrected(delta_velocity - delta_vel_bias_scaled);

	_output_new.time_us = time_us;
	_output_vert_new.time_us = time_us;

	const Quatf dq(AxisAnglef{delta_angle_corrected});

	// 右乘本帧增量四元数，完成机体系角增量的姿态传播。
	const Quatf quat_nominal_before_update = _output_new.quat_nominal;
	_output_new.quat_nominal = _output_new.quat_nominal * dq;

	// 每次传播后归一化，抑制浮点误差破坏单位四元数约束。
	_output_new.quat_nominal.normalize();

	// 当前姿态对应的 body -> NED 旋转矩阵。
	_R_to_earth_now = Dcmf(_output_new.quat_nominal);

	// 加速度计速度增量原本在机体系，将其旋转到 NED 系。
	Vector3f delta_vel_earth{_R_to_earth_now * delta_velocity_corrected};

	// 加速度计测量的是比力；NED 的 D 轴向下，因此在 D 分量加上重力增量。
	delta_vel_earth(2) += CONSTANTS_ONE_G * delta_velocity_dt;

	// 累积 NED 速度增量，供 getVelocityDerivative() 计算平均加速度。
	_delta_vel_sum += delta_vel_earth;
	_delta_vel_dt += delta_velocity_dt;

	// 保存旧速度，位置更新使用新旧速度均值进行梯形积分。
	const Vector3f vel_last(_output_new.vel);

	// 更新普通三维速度，同时更新独立垂直通道的速度。
	_output_new.vel += delta_vel_earth;
	_output_vert_new.vert_vel += delta_vel_earth(2);

	// 使用梯形积分更新 NED 位置和独立垂直通道的位置积分量。
	const Vector3f delta_pos_NED = (_output_new.vel + vel_last) * (delta_velocity_dt * 0.5f);
	_output_new.pos += delta_pos_NED;
	_output_vert_new.vert_vel_integ += delta_pos_NED(2);

	// 保存独立垂直通道在两次 EKF 校正之间累计的积分时间。
	_output_vert_new.dt += delta_velocity_dt;

	// 杆臂补偿：IMU 不在机体原点时，转动会产生 omega x r 的附加速度。
	if (delta_angle_dt > 0.001f) {
		// 用本帧角增量除以积分时间，得到平均角速度。
		const Vector3f ang_rate = delta_angle_corrected / delta_angle_dt;

		// 计算 IMU 相对机体原点的切向速度 omega x r。
		const Vector3f vel_imu_rel_body = ang_rate % _imu_pos_body;

		// 转到 NED 系；getter 会从 IMU 速度中减去它，得到机体原点速度。
		_vel_imu_rel_body_ned = _R_to_earth_now * vel_imu_rel_body;
	}

	// 辅助纯惯性航向只去陀螺零偏，不加入 _delta_angle_corr，可用于观察无辅助漂移。
	const Quatf delta_quat_unaided = Quatf(AxisAnglef(delta_angle - delta_angle_bias_scaled));
	const float yaw_state = Eulerf(quat_nominal_before_update).psi();
	const Quatf quat_unaided = quat_nominal_before_update * delta_quat_unaided;
	const float yaw_without_aiding = Eulerf(quat_unaided).psi();
	// 累积传播前后的航向差，并限制到 [-pi, pi]。
	const float unaided_delta_yaw = yaw_without_aiding - yaw_state;
	_unaided_yaw = matrix::wrap_pi(_unaided_yaw + unaided_delta_yaw);

	// 累积去零偏角增量，供外部按需读取平均角速度；读取后会清零。
	_delta_angle_sum *= delta_quat_unaided;
	_delta_angle_sum.normalize();
	_delta_angle_sum_dt += delta_angle_dt;
}

void OutputPredictor::correctOutputStates(const uint64_t time_delayed_us,
		const Quatf &quat_state, const Vector3f &vel_state, const LatLonAlt &gpos_state, const matrix::Vector3f &gyro_bias,
		const matrix::Vector3f &accel_bias)
{
	// 找到延迟融合时刻，计算 EKF 与预测器误差，再把修正平滑施加到整条输出历史。
	// 低通估计主 EKF 校正周期，用于保持互补滤波增益稳定。
	if (_time_last_correct_states_us != 0) {
		const float dt = math::constrain((time_delayed_us - _time_last_correct_states_us) * 1e-6f, 0.0001f, 0.03f);
		_dt_correct_states_avg = 0.8f * _dt_correct_states_avg + 0.2f * dt;
	}

	_time_last_correct_states_us = time_delayed_us;

	// 保存主 EKF 最新零偏，后续每帧高频 IMU 预测都会使用。
	_gyro_bias = gyro_bias;
	_accel_bias = accel_bias;

	// 每个主 EKF 周期保存当前输出快照，使输出历史与延迟 IMU 时间轴近似一致。
	_output_buffer.push(_output_new);
	_output_vert_buffer.push(_output_vert_new);

	// 最旧输出状态近似对应 EKF 融合时刻，用它与刚更新的延迟 EKF 状态比较。
	// TODO: there is no guarantee that data is at delayed fusion horizon
	//       Shouldnt we use pop_first_older_than?
	// TODO 中文：目前不能严格保证该样本恰好位于融合时刻，可考虑按时间戳查找。
	const outputSample &output_delayed = _output_buffer.get_oldest();
	const outputVert &output_vert_delayed = _output_vert_buffer.get_oldest();

	// 姿态误差定义为 EKF^-1 * 预测器，表示预测器相对 EKF 的旋转偏差。
	const Quatf q_error((quat_state.inversed() * output_delayed.quat_nominal).normalized());

	// 小角度下 delta_theta ~= +/-2*q.xyz；符号选择保证使用最短旋转方向。
	const float scalar = (q_error(0) >= 0.0f) ? -2.f : 2.f;

	const Vector3f delta_ang_error{scalar * q_error(1), scalar * q_error(2), scalar * q_error(3)};

	// 根据实际延迟自适应姿态反馈增益，使延迟变化时仍保持约 0.7 的阻尼比。
	const uint64_t time_latest_us = _time_last_update_states_us;
	const float time_delay = fmaxf((time_latest_us - time_delayed_us) * 1e-6f, _dt_update_states_avg);
	const float att_gain = 0.5f * _dt_update_states_avg / time_delay;

	// 生成下一帧 IMU 要叠加的角增量修正，并记录姿态跟踪误差模长。
	_delta_angle_corr = delta_ang_error * att_gain;
	_output_tracking_error(0) = delta_ang_error.norm();

	/*
	* Loop through the output filter state history and apply the corrections to the velocity and position states.
	* This method is too expensive to use for the attitude states due to the quaternion operations required
	* but because it eliminates the time delay in the 'correction loop' it allows higher tracking gains
	* to be used and reduces tracking error relative to EKF states.
	*
	* 中文：速度和位置修正直接作用于整条历史，消除了校正环路延迟；姿态若对
	* 全部历史执行四元数运算代价较高，因此采用上方逐帧角增量反馈。
	*/

	// 一阶互补滤波增益近似为 dt/tau；约束 tau 可防止增益过大或跟踪过慢。
	const float vel_gain = _dt_correct_states_avg / math::constrain(_vel_tau, _dt_correct_states_avg, 10.f);
	const float pos_gain = _dt_correct_states_avg / math::constrain(_pos_tau, _dt_correct_states_avg, 10.f);

	const Vector3f pos_state = gpos_state - _global_ref;

	// 独立垂直通道同时比较垂直速度和“速度积分位置”误差。
	const float vert_vel_err = (vel_state(2) - output_vert_delayed.vert_vel);
	const float vert_vel_integ_err = (pos_state(2) - output_vert_delayed.vert_vel_integ);

	// 使用类似 PD 的反馈形成垂直速度修正，1.1 系数对应约 5% 超调的调节目标。
	const float vert_vel_correction = vert_vel_integ_err * pos_gain + vert_vel_err * vel_gain * 1.1f;

	applyCorrectionToVerticalOutputBuffer(vert_vel_correction, pos_state(2));

	// 普通通道直接计算延迟时刻三维速度和位置跟踪误差。
	const Vector3f vel_err(vel_state - output_delayed.vel);
	const Vector3f pos_err(pos_state - output_delayed.pos);

	_output_tracking_error(1) = vel_err.norm();
	_output_tracking_error(2) = pos_err.norm();

	// 比例项快速跟踪，较弱积分项消除长期稳态速度误差。
	_vel_err_integ += vel_err;
	const Vector3f vel_correction = vel_err * vel_gain + _vel_err_integ * sq(vel_gain) * 0.1f;

	// 位置通道采用相同的比例加弱积分结构。
	_pos_err_integ += pos_err;
	const Vector3f pos_correction = pos_err * pos_gain + _pos_err_integ * sq(pos_gain) * 0.1f;

	// 每周期将 _global_ref 移到最新 EKF 全球位置，故需扣除参考点变化，保证
	// “全球参考 + 局部位移”所表示的绝对位置连续。
	const Vector3f pos_correction_with_ref_change = pos_correction - pos_state;
	applyCorrectionToOutputBuffer(vel_correction, pos_correction_with_ref_change);

	_global_ref = gpos_state;
}

void OutputPredictor::applyCorrectionToVerticalOutputBuffer(const float vert_vel_correction, const float pos_ref_change)
{
	// 从最旧样本向前修正每个垂直速度，再用修正后的速度历史重新积分垂直位置。
	uint8_t index = _output_vert_buffer.get_oldest_index();

	const uint8_t size = _output_vert_buffer.get_length();

	for (uint8_t counter = 0; counter < (size - 1); counter++) {
		const uint8_t index_next = (index + 1) % size;
		outputVert &current_state = _output_vert_buffer[index];
		outputVert &next_state = _output_vert_buffer[index_next];

		// 第一个样本和所有后续样本都施加相同速度修正。
		if (counter == 0) {
			current_state.vert_vel += vert_vel_correction;
		}

		next_state.vert_vel += vert_vel_correction;

		// 用相邻修正速度作梯形积分，并消除全球参考点变化造成的坐标平移。
		next_state.vert_vel_integ = current_state.vert_vel_integ + (current_state.vert_vel + next_state.vert_vel) * 0.5f *
					    next_state.dt - pos_ref_change;

		// 环形索引到末尾后回到 0。
		index = (index + 1) % size;
	}

	// 当前输出取修正后历史中的最新状态。
	_output_vert_new = _output_vert_buffer.get_newest();

	// 清零累计时间，开始下一段全速 IMU 垂直积分。
	_output_vert_new.dt = 0.0f;
}

void OutputPredictor::applyCorrectionToOutputBuffer(const Vector3f &vel_correction, const Vector3f &pos_correction)
{
	// 整条历史统一平移相同速度和位置修正，保留历史样本之间的相对运动。
	for (uint8_t index = 0; index < _output_buffer.get_length(); index++) {
		// 对每个历史速度施加相同常量修正。
		_output_buffer[index].vel += vel_correction;

		// 对每个历史位置施加相同常量修正。
		_output_buffer[index].pos += pos_correction;
	}

	// 将修正后的最新历史样本作为当前对外输出。
	_output_new = _output_buffer.get_newest();
}

matrix::Vector3f OutputPredictor::getVelocityDerivative() const
{
	// 返回自上次清零以来的平均 NED 加速度；没有有效累计时间时返回零向量。
	if (_delta_vel_dt > FLT_EPSILON) {
		return _delta_vel_sum / _delta_vel_dt;

	} else {
		return matrix::Vector3f(0.f, 0.f, 0.f);
	}
}

void OutputPredictor::resetVelocityDerivativeAccumulation()
{
	// 调用者读取完平均加速度后，通过此函数开始新的统计窗口。
	_delta_vel_dt = 0.f;
	_delta_vel_sum.setZero();
}

Vector3f OutputPredictor::getAngularVelocityAndResetAccumulator()
{
	// 将累计旋转转换为等效轴角并除以累计时间，得到平均机体系角速度。
	Vector3f angular_velocity;

	if (_delta_angle_sum_dt > FLT_EPSILON) {
		angular_velocity = AxisAnglef(_delta_angle_sum) / _delta_angle_sum_dt;
	}

	// 该 getter 同时消费数据，返回后立即清零累计旋转和时间。
	_delta_angle_sum.setIdentity();
	_delta_angle_sum_dt = 0.f;

	return angular_velocity;
}
