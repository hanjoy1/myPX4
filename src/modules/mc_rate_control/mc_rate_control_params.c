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

/**
 * @file mc_rate_control_params.c
 *
 * Parameters for multicopter rate controller
 *
 * 中文阅读提示：
 * 这里定义多旋翼角速度控制器参数。角速度控制器输入是机体系角速度误差，
 * 输出是归一化 torque setpoint，随后由 control allocator 分配给电机。
 *
 * 三轴 PID 参数最终在 MulticopterRateControl::parameters_updated() 中写入 RateControl。
 */

/**
 * Roll rate P gain
 *
 * Roll rate proportional gain, i.e. control output for angular speed error 1 rad/s.
 *
 * 中文：Roll 角速度 P 增益。roll rate 误差为 1 rad/s 时，P 项产生多少控制输出。
 * 增大该值会提升角速度响应，但过大可能导致振荡。
 *
 * @min 0.01
 * @max 0.5
 * @decimal 3
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_ROLLRATE_P, 0.15f);

/**
 * Roll rate I gain
 *
 * Roll rate integral gain. Can be set to compensate static thrust difference or gravity center offset.
 *
 * 中文：Roll 角速度 I 增益。用于补偿静态力矩偏差，例如电机/桨差异或重心偏移。
 * I 增益过大可能造成低频振荡或恢复慢，过小则抗持续扰动能力不足。
 *
 * @min 0.0
 * @decimal 3
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_ROLLRATE_I, 0.2f);

/**
 * Roll rate integrator limit
 *
 * Roll rate integrator limit. Can be set to increase the amount of integrator available to counteract disturbances or reduced to improve settling time after large roll moment trim changes.
 *
 * 中文：Roll 角速度积分项限幅。增大可提供更多抗持续扰动能力，
 * 减小可在大 trim/扰动变化后更快收敛，降低积分 windup 风险。
 *
 * @min 0.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_RR_INT_LIM, 0.30f);

/**
 * Roll rate D gain
 *
 * Roll rate differential gain. Small values help reduce fast oscillations. If value is too big oscillations will appear again.
 *
 * 中文：Roll 角速度 D 增益。D 项基于角加速度/角速度变化，适当增加可抑制快速振荡；
 * 过大则会放大噪声并重新引入高频振荡。
 *
 * @min 0.0
 * @max 0.01
 * @decimal 4
 * @increment 0.0005
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_ROLLRATE_D, 0.003f);

/**
 * Roll rate feedforward
 *
 * Improves tracking performance.
 *
 * 中文：Roll 角速度前馈增益。直接根据 rate setpoint 给控制输出，
 * 可改善指令跟踪，但过大可能使响应过冲。
 *
 * @min 0.0
 * @decimal 4
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_ROLLRATE_FF, 0.0f);

/**
 * Roll rate controller gain
 *
 * Global gain of the controller.
 *
 * This gain scales the P, I and D terms of the controller:
 * output = MC_ROLLRATE_K * (MC_ROLLRATE_P * error
 * 			     + MC_ROLLRATE_I * error_integral
 * 			     + MC_ROLLRATE_D * error_derivative)
 * Set MC_ROLLRATE_P=1 to implement a PID in the ideal form.
 * Set MC_ROLLRATE_K=1 to implement a PID in the parallel form.
 *
 * 中文：Roll 角速度控制器全局增益 K。实际 P/I/D 项都会乘以 K。
 * 这使得可以在并联 PID 形式和理想 PID 形式之间调参。
 *
 * @min 0.01
 * @max 5.0
 * @decimal 4
 * @increment 0.0005
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_ROLLRATE_K, 1.0f);

/**
 * Pitch rate P gain
 *
 * Pitch rate proportional gain, i.e. control output for angular speed error 1 rad/s.
 *
 * 中文：Pitch 角速度 P 增益，作用和 MC_ROLLRATE_P 类似，只是作用在 pitch 轴。
 *
 * @min 0.01
 * @max 0.6
 * @decimal 3
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_PITCHRATE_P, 0.15f);

/**
 * Pitch rate I gain
 *
 * Pitch rate integral gain. Can be set to compensate static thrust difference or gravity center offset.
 *
 * 中文：Pitch 角速度 I 增益，用于补偿 pitch 轴持续偏差或重心/推力不对称。
 *
 * @min 0.0
 * @decimal 3
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_PITCHRATE_I, 0.2f);

/**
 * Pitch rate integrator limit
 *
 * Pitch rate integrator limit. Can be set to increase the amount of integrator available to counteract disturbances or reduced to improve settling time after large pitch moment trim changes.
 *
 * 中文：Pitch 积分项限幅，控制 pitch 轴积分项最大可用控制量。
 *
 * @min 0.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_PR_INT_LIM, 0.30f);

/**
 * Pitch rate D gain
 *
 * Pitch rate differential gain. Small values help reduce fast oscillations. If value is too big oscillations will appear again.
 *
 * 中文：Pitch 角速度 D 增益，用于抑制快速振荡；过大容易放大噪声。
 *
 * @min 0.0
 * @decimal 4
 * @increment 0.0005
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_PITCHRATE_D, 0.003f);

/**
 * Pitch rate feedforward
 *
 * Improves tracking performance.
 *
 * 中文：Pitch 角速度前馈增益，用于提升 pitch rate setpoint 跟踪性能。
 *
 * @min 0.0
 * @decimal 4
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_PITCHRATE_FF, 0.0f);

/**
 * Pitch rate controller gain
 *
 * Global gain of the controller.
 *
 * This gain scales the P, I and D terms of the controller:
 * output = MC_PITCHRATE_K * (MC_PITCHRATE_P * error
 * 			     + MC_PITCHRATE_I * error_integral
 * 			     + MC_PITCHRATE_D * error_derivative)
 * Set MC_PITCHRATE_P=1 to implement a PID in the ideal form.
 * Set MC_PITCHRATE_K=1 to implement a PID in the parallel form.
 *
 * 中文：Pitch 角速度控制器全局增益 K。实际 pitch P/I/D 都会乘以该值。
 *
 * @min 0.01
 * @max 5.0
 * @decimal 4
 * @increment 0.0005
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_PITCHRATE_K, 1.0f);

/**
 * Yaw rate P gain
 *
 * Yaw rate proportional gain, i.e. control output for angular speed error 1 rad/s.
 *
 * 中文：Yaw 角速度 P 增益。yaw 轴控制能力通常弱于 roll/pitch，过大容易饱和或振荡。
 *
 * @min 0.0
 * @max 0.6
 * @decimal 3
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_YAWRATE_P, 0.2f);

/**
 * Yaw rate I gain
 *
 * Yaw rate integral gain. Can be set to compensate static thrust difference or gravity center offset.
 *
 * 中文：Yaw 角速度 I 增益，用于补偿持续 yaw 力矩偏差，例如电机反扭矩不均衡。
 *
 * @min 0.0
 * @decimal 3
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_YAWRATE_I, 0.1f);

/**
 * Yaw rate integrator limit
 *
 * Yaw rate integrator limit. Can be set to increase the amount of integrator available to counteract disturbances or reduced to improve settling time after large yaw moment trim changes.
 *
 * 中文：Yaw 积分项限幅。yaw 轴容易饱和，积分限幅过大可能导致恢复慢或 windup。
 *
 * @min 0.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_YR_INT_LIM, 0.30f);

/**
 * Yaw rate D gain
 *
 * Yaw rate differential gain. Small values help reduce fast oscillations. If value is too big oscillations will appear again.
 *
 * 中文：Yaw 角速度 D 增益。默认通常为 0，避免 yaw 轴噪声和电机加减速带来高频力矩。
 *
 * @min 0.0
 * @decimal 4
 * @increment 0.0005
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_YAWRATE_D, 0.0f);

/**
 * Yaw rate feedforward
 *
 * Improves tracking performance.
 *
 * 中文：Yaw 角速度前馈增益，用于改善 yaw rate setpoint 跟踪。
 *
 * @min 0.0
 * @decimal 4
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_YAWRATE_FF, 0.0f);

/**
 * Yaw rate controller gain
 *
 * Global gain of the controller.
 *
 * This gain scales the P, I and D terms of the controller:
 * output = MC_YAWRATE_K * (MC_YAWRATE_P * error
 * 			     + MC_YAWRATE_I * error_integral
 * 			     + MC_YAWRATE_D * error_derivative)
 * Set MC_YAWRATE_P=1 to implement a PID in the ideal form.
 * Set MC_YAWRATE_K=1 to implement a PID in the parallel form.
 *
 * 中文：Yaw 角速度控制器全局增益 K。实际 yaw P/I/D 都会乘以该值。
 *
 * @min 0.01
 * @max 5.0
 * @decimal 4
 * @increment 0.0005
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_YAWRATE_K, 1.0f);

/**
 * Battery power level scaler
 *
 * This compensates for voltage drop of the battery over time by attempting to
 * normalize performance across the operating range of the battery. The copter
 * should constantly behave as if it was fully charged with reduced max acceleration
 * at lower battery percentages. i.e. if hover is at 0.5 throttle at 100% battery,
 * it will still be 0.5 at 60% battery.
 *
 * 中文：电池电压缩放补偿开关。启用后根据 battery_status.scale 缩放 torque/thrust setpoint，
 * 用于补偿电池电压下降导致的控制效能变弱。目标是让同样的归一化控制输入在不同电量下表现更一致。
 *
 * @boolean
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_INT32(MC_BAT_SCALE_EN, 0);

/**
 * Low pass filter cutoff frequency for yaw torque setpoint
 *
 * Reduces vibrations by lowering high frequency torque caused by rotor acceleration.
 * 0 disables the filter
 *
 * 中文：Yaw torque setpoint 低通滤波截止频率。
 * 用于降低由电机/桨加减速引起的高频 yaw 力矩振动。设为 0 表示关闭滤波。
 *
 * @min 0
 * @max 10
 * @unit Hz
 * @decimal 3
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_YAW_TQ_CUTOFF, 2.f);
