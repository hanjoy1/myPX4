/****************************************************************************
 *
 *   Copyright (C) 2018 - 2019 PX4 Development Team. All rights reserved.
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
 * @file ControlMath.hpp
 *
 * Simple functions for vector manipulation that do not fit into matrix lib.
 * These functions are specific for controls.
 *
 * 中文阅读提示：
 * 这里放的是位置控制器用到的几何/向量工具函数，尤其是“推力矢量 -> 姿态四元数”。
 * 多旋翼本质上通过改变机体 z 轴方向来改变推力方向，因此 body_z 是很多函数的关键变量。
 */

#pragma once

#include <matrix/matrix/math.hpp>
#include <uORB/topics/vehicle_attitude_setpoint.h>

namespace ControlMath
{
/**
 * Converts thrust vector and yaw set-point to a desired attitude.
 * 中文：把 NED 坐标下的期望推力矢量和 yaw 设定值转换为姿态设定值。
 * @param thr_sp desired 3D thrust vector
 * @param thr_sp 期望三维推力矢量，NED 坐标系
 * @param yaw_sp the desired yaw
 * @param yaw_sp 期望航向角 [rad]
 * @param att_sp attitude setpoint to fill
 * @param att_sp 待填充的姿态设定值消息
 */
void thrustToAttitude(const matrix::Vector3f &thr_sp, const float yaw_sp, vehicle_attitude_setpoint_s &att_sp);

/**
 * Limits the tilt angle between two unit vectors
 * 中文：把 body_unit 限制在 world_unit 周围 max_angle 以内，常用于限制机体倾角。
 * @param body_unit unit vector that will get adjusted if angle is too big
 * @param body_unit 要被限制的单位向量，若夹角过大会被原地修改
 * @param world_unit fixed vector to measure the angle against
 * @param world_unit 用来测量夹角的参考单位向量，通常是世界竖直方向
 * @param max_angle maximum tilt angle between vectors in radians
 * @param max_angle 允许的最大夹角 [rad]
 */
void limitTilt(matrix::Vector3f &body_unit, const matrix::Vector3f &world_unit, const float max_angle);

/**
 * Converts a body z vector and yaw set-point to a desired attitude.
 * 中文：已知期望机体 z 轴方向和 yaw，构造完整姿态矩阵/四元数。
 * @param body_z a world frame 3D vector in direction of the desired body z axis
 * @param body_z 期望机体 z 轴在世界坐标系中的方向
 * @param yaw_sp the desired yaw setpoint
 * @param yaw_sp 期望航向角 [rad]
 * @param att_sp attitude setpoint to fill
 * @param att_sp 待填充的姿态设定值消息
 */
void bodyzToAttitude(matrix::Vector3f body_z, const float yaw_sp, vehicle_attitude_setpoint_s &att_sp);

/**
 * Outputs the sum of two vectors but respecting the limits and priority.
 * The sum of two vectors are constraint such that v0 has priority over v1.
 * This means that if the length of (v0+v1) exceeds max, then it is constraint such
 * that v0 has priority.
 *
 * @param v0 a 2D vector that has priority given the maximum available magnitude.
 * @param v0 优先级更高的二维向量，例如位置误差闭环产生的速度
 * @param v1 a 2D vector that less priority given the maximum available magnitude.
 * @param v1 优先级较低的二维向量，例如速度前馈
 * @param max 输出向量允许的最大模长
 * @return 2D vector
 * @return 限幅后的二维向量，尽量保留 v0，并在剩余空间内加入 v1
 *
 * 中文：常用于“位置闭环速度 + 前馈速度”的限幅，v0 优先、v1 在剩余空间里缩放。
 */
matrix::Vector2f constrainXY(const matrix::Vector2f &v0, const matrix::Vector2f &v1, const float &max);

/**
 * This method was used for smoothing the corners along two lines.
 * 该函数曾用于平滑两条线段之间的拐角。
 *
 * @param sphere_c
 * @param sphere_c 球心位置
 * @param sphere_r
 * @param sphere_r 球半径
 * @param line_a
 * @param line_a 线段起点
 * @param line_b
 * @param line_b 线段终点
 * @param res
 * @param res 输出的交点或最近点
 * return boolean
 * @return true 表示存在交点或线段退化；false 表示无交点并返回最近点
 *
 * Note: this method is not used anywhere and first requires review before usage.
 * 注意：该函数当前未被使用，重新使用前需要先审查逻辑。
 */
bool cross_sphere_line(const matrix::Vector3f &sphere_c, const float sphere_r, const matrix::Vector3f &line_a,
		       const matrix::Vector3f &line_b, matrix::Vector3f &res);

/**
 * Adds e.g. feed-forward to the setpoint making sure existing or added NANs have no influence on control.
 * This function is udeful to support all the different setpoint combinations of position, velocity, acceleration with NAN representing an uncommitted value.
 * 将前馈等附加量加到 setpoint 上，同时保证已有或新增的 NaN 不影响控制。
 * 该函数用于支持位置/速度/加速度的各种组合，其中 NaN 表示该项未设置。
 * @param setpoint existing possibly NAN setpoint to add to
 * @param setpoint 可能为 NaN 的原设定值，会被原地更新
 * @param addition value/NAN to add to the setpoint
 * @param addition 要叠加的值；若为 NaN 则不贡献该项
 */
void addIfNotNan(float &setpoint, const float addition);

/**
 * _addIfNotNan for Vector3f treating each element individually
 * Vector3f 版本的 addIfNotNan，对每个元素分别处理。
 * @see _addIfNotNan
 */
void addIfNotNanVector3f(matrix::Vector3f &setpoint, const matrix::Vector3f &addition);

/**
 * Overwrites elements of a Vector3f which are NaN with zero
 * 将 Vector3f 中为 NaN 的元素替换为 0。
 * @param vector possibly containing NAN elements
 * @param vector 可能包含 NaN 的向量，会被原地修改
 */
void setZeroIfNanVector3f(matrix::Vector3f &vector);
}
