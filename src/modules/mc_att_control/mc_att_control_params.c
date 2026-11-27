/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
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
 * @file mc_att_control_params.c
 * Parameters for multicopter attitude controller.
 *
 * 中文阅读提示：
 * 这里定义多旋翼姿态控制器相关参数。姿态控制器输出的是角速度设定值，
 * 所以姿态 P 增益可以理解为“1 rad 姿态误差对应多少 rad/s 角速度命令”。
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Anton Babushkin <anton@px4.io>
 */

/**
 * Roll P gain
 *
 * Roll proportional gain, i.e. desired angular speed in rad/s for error 1 rad.
 *
 * 中文：Roll 姿态比例增益。roll 姿态误差为 1 rad 时，期望产生多少 rad/s 的 roll rate setpoint。
 * 增大该值会让 roll 姿态跟踪更快，但过大可能引起振荡或角速度控制饱和。
 *
 * @min 0.0
 * @max 12
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_ROLL_P, 4.0f);

/**
 * Pitch P gain
 *
 * Pitch proportional gain, i.e. desired angular speed in rad/s for error 1 rad.
 *
 * 中文：Pitch 姿态比例增益。pitch 姿态误差为 1 rad 时，期望产生多少 rad/s 的 pitch rate setpoint。
 * 作用和 MC_ROLL_P 类似，只是作用在 pitch 轴。
 *
 * @min 0.0
 * @max 12
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_PITCH_P, 4.0f);

/**
 * Yaw P gain
 *
 * Yaw proportional gain, i.e. desired angular speed in rad/s for error 1 rad.
 *
 * 中文：Yaw 姿态比例增益。yaw 姿态误差为 1 rad 时，期望产生多少 rad/s 的 yaw rate setpoint。
 * yaw 通道控制能力通常弱于 roll/pitch，调参时要注意不要过激。
 *
 * @min 0.0
 * @max 5
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_YAW_P, 2.8f);

/**
 * Yaw weight
 *
 * A fraction [0,1] deprioritizing yaw compared to roll and pitch in non-linear attitude control.
 * Deprioritizing yaw is necessary because multicopters have much less control authority
 * in yaw compared to the other axes and it makes sense because yaw is not critical for
 * stable hovering or 3D navigation.
 *
 * For yaw control tuning use MC_YAW_P. This ratio has no impact on the yaw gain.
 *
 * 中文：Yaw 权重，用于在非线性姿态控制中相对 roll/pitch 降低 yaw 的优先级。
 * 多旋翼 yaw 控制力通常小于 roll/pitch，而且悬停和三维导航更依赖推力方向，
 * 因此控制器会优先保证 roll/pitch，也就是优先保证机体 z 轴/推力方向。
 *
 * 注意：调 yaw 闭环快慢应使用 MC_YAW_P；该权重主要影响姿态误差分解中的 yaw 优先级。
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_YAW_WEIGHT, 0.4f);

/**
 * Max roll rate
 *
 * Limit for roll rate in manual and auto modes (except acro).
 * Has effect for large rotations in autonomous mode, to avoid large control
 * output and mixer saturation.
 *
 * This is not only limited by the vehicle's properties, but also by the maximum
 * measurement rate of the gyro.
 *
 * 中文：Roll 角速度设定值上限，作用于手动和自动模式，Acro 除外。
 * 大角度姿态误差时，该限制可以避免姿态 P 环输出过大导致 mixer 饱和。
 * 实际可设上限不仅受机体能力限制，也受陀螺仪最大测量速率限制。
 *
 * @unit deg/s
 * @min 0.0
 * @max 1800.0
 * @decimal 1
 * @increment 5
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_ROLLRATE_MAX, 220.0f);

/**
 * Max pitch rate
 *
 * Limit for pitch rate in manual and auto modes (except acro).
 * Has effect for large rotations in autonomous mode, to avoid large control
 * output and mixer saturation.
 *
 * This is not only limited by the vehicle's properties, but also by the maximum
 * measurement rate of the gyro.
 *
 * 中文：Pitch 角速度设定值上限，作用和 MC_ROLLRATE_MAX 类似，只是作用在 pitch 轴。
 * 大角度姿态误差或自动模式快速姿态变化时，该值限制输出给 rate controller 的 pitch rate setpoint。
 *
 * @unit deg/s
 * @min 0.0
 * @max 1800.0
 * @decimal 1
 * @increment 5
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_PITCHRATE_MAX, 220.0f);

/**
 * Max yaw rate
 *
 * 中文：Yaw 角速度设定值上限，限制姿态控制器输出的 yaw rate setpoint。
 * yaw 通道容易受电机/桨反扭矩能力限制，过大可能导致饱和。
 *
 * @unit deg/s
 * @min 0.0
 * @max 1800.0
 * @decimal 1
 * @increment 5
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_YAWRATE_MAX, 200.0f);

/**
 * Manual tilt input filter time constant
 *
 * Setting this parameter to 0 disables the filter
 *
 * 中文：手动 roll/pitch 倾角输入的一阶滤波时间常数。
 * 设为 0 表示禁用滤波，摇杆输入会更直接；增大该值会让手动倾角变化更平滑但响应更慢。
 *
 * @unit s
 * @min 0.0
 * @max 2.0
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MC_MAN_TILT_TAU, 0.0f);
