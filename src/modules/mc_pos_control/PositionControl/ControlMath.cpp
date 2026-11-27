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
 * @file ControlMath.cpp
 */

#include "ControlMath.hpp"
#include <px4_platform_common/defines.h>
#include <float.h>
#include <mathlib/mathlib.h>

using namespace matrix;

namespace ControlMath
{
// 将期望推力矢量和 yaw 转成姿态设定值，同时写入机体系 z 向推力。
void thrustToAttitude(const Vector3f &thr_sp, const float yaw_sp, vehicle_attitude_setpoint_s &att_sp)
{
	// PX4 的推力设定值 thr_sp 指向“期望加速度/推力矢量”，而机体 body_z 轴与推力方向相反。
	bodyzToAttitude(-thr_sp, yaw_sp, att_sp);
	// 姿态控制器使用机体系 z 轴推力，负号表示沿 -body_z 产生升力。
	att_sp.thrust_body[2] = -thr_sp.length();
}

// 限制 body_unit 相对 world_unit 的最大夹角，常用于限制多旋翼最大倾角。
void limitTilt(Vector3f &body_unit, const Vector3f &world_unit, const float max_angle)
{
	// determine tilt
	// 计算当前向量相对参考向量的倾角。
	// 通过点积得到两个单位向量夹角；world_unit 通常是 NED 的竖直向下方向 (0,0,1)。
	const float dot_product_unit = body_unit.dot(world_unit);
	float angle = acosf(dot_product_unit);
	// limit tilt
	// 将倾角限制到 max_angle。
	// 如果倾角超过上限，就沿原来的倾斜方向把角度截断到 max_angle。
	angle = math::min(angle, max_angle);
	Vector3f rejection = body_unit - (dot_product_unit * world_unit);

	// corner case exactly parallel vectors
	// 完全平行时 rejection 为零，手动指定一个方向避免归一化失败。
	if (rejection.norm_squared() < FLT_EPSILON) {
		rejection(0) = 1.f;
	}

	body_unit = cosf(angle) * world_unit + sinf(angle) * rejection.unit();
}

// 根据期望机体 z 轴方向和 yaw 构造完整姿态，并写入四元数设定值。
void bodyzToAttitude(Vector3f body_z, const float yaw_sp, vehicle_attitude_setpoint_s &att_sp)
{
	// zero vector, no direction, set safe level value
	// 零向量没有方向，使用安全的水平姿态作为退化输出。
	// 推力向量为 0 时无法定义姿态，退化为水平姿态，保证输出仍可执行。
	if (body_z.norm_squared() < FLT_EPSILON) {
		body_z(2) = 1.f;
	}

	body_z.normalize();

	// vector of desired yaw direction in XY plane, rotated by PI/2
	// y_C 是由期望 yaw 定义的“世界水平面内机体 y 轴参考方向”。
	const Vector3f y_C{-sinf(yaw_sp), cosf(yaw_sp), 0.f};

	// desired body_x axis, orthogonal to body_z
	// body_x 由 y_C 和 body_z 的叉乘得到，确保和期望推力方向正交。
	Vector3f body_x = y_C % body_z;

	// keep nose to front while inverted upside down
	if (body_z(2) < 0.f) {
		// 倒飞时翻转 body_x，尽量保持机头朝前。
		body_x = -body_x;
	}

	if (fabsf(body_z(2)) < 0.000001f) {
		// desired thrust is in XY plane, set X downside to construct correct matrix,
		// but yaw component will not be used actually
		// 当推力几乎水平时，yaw 自由度退化；这里选一个稳定的 body_x 来构造矩阵。
		body_x.zero();
		body_x(2) = 1.f;
	}

	body_x.normalize();

	// desired body_y axis
	// 由 body_z 和 body_x 叉乘得到期望 body_y，组成右手坐标系。
	const Vector3f body_y = body_z % body_x;

	Dcmf R_sp;

	// fill rotation matrix
	// 方向余弦矩阵的三列分别是期望机体 x/y/z 轴在世界坐标中的方向。
	for (int i = 0; i < 3; i++) {
		R_sp(i, 0) = body_x(i);
		R_sp(i, 1) = body_y(i);
		R_sp(i, 2) = body_z(i);
	}

	// copy quaternion setpoint to attitude setpoint topic
	// 把旋转矩阵转换为四元数并写入姿态设定值消息。
	const Quatf q_sp{R_sp};
	q_sp.copyTo(att_sp.q_d);
}

// 对两个水平向量求和并限幅，v0 优先保留，v1 在剩余可用模长内缩放。
Vector2f constrainXY(const Vector2f &v0, const Vector2f &v1, const float &max)
{
	if (Vector2f(v0 + v1).norm() <= max) {
		// vector does not exceed maximum magnitude
		// 合速度没有超过限制，直接返回。
		return v0 + v1;

	} else if (v0.length() >= max) {
		// the magnitude along v0, which has priority, already exceeds maximum.
		// 优先级更高的 v0 已经超限，只保留 v0 方向并限制到 max。
		return v0.normalized() * max;

	} else if (fabsf(Vector2f(v1 - v0).norm()) < 0.001f) {
		// the two vectors are equal
		// 两个向量几乎相等，直接沿 v0 方向限幅。
		return v0.normalized() * max;

	} else if (v0.length() < 0.001f) {
		// the first vector is 0.
		// v0 近似为 0 时，没有优先方向，直接沿 v1 方向限幅。
		return v1.normalized() * max;

	} else {
		// vf = final vector with ||vf|| <= max
		// s = scaling factor
		// u1 = unit of v1
		// vf = v0 + v1 = v0 + s * u1
		// constraint: ||vf|| <= max
		//
		// solve for s: ||vf|| = ||v0 + s * u1|| <= max
		//
		// Derivation:
		// For simplicity, replace v0 -> v, u1 -> u
		// 				   		   v0(0/1/2) -> v0/1/2
		// 				   		   u1(0/1/2) -> u0/1/2
		//
		// ||v + s * u||^2 = (v0+s*u0)^2+(v1+s*u1)^2+(v2+s*u2)^2 = max^2
		// v0^2+2*s*u0*v0+s^2*u0^2 + v1^2+2*s*u1*v1+s^2*u1^2 + v2^2+2*s*u2*v2+s^2*u2^2 = max^2
		// s^2*(u0^2+u1^2+u2^2) + s*2*(u0*v0+u1*v1+u2*v2) + (v0^2+v1^2+v2^2-max^2) = 0
		//
		// quadratic equation:
		// -> s^2*a + s*b + c = 0 with solution: s1/2 = (-b +- sqrt(b^2 - 4*a*c))/(2*a)
		//
		// b = 2 * u.dot(v)
		// a = 1 (because u is normalized)
		// c = (v0^2+v1^2+v2^2-max^2) = -max^2 + ||v||^2
		//
		// sqrt(b^2 - 4*a*c) =
		// 		sqrt(4*u.dot(v)^2 - 4*(||v||^2 - max^2)) = 2*sqrt(u.dot(v)^2 +- (||v||^2 -max^2))
		//
		// s1/2 = ( -2*u.dot(v) +- 2*sqrt(u.dot(v)^2 - (||v||^2 -max^2)) / 2
		//      =  -u.dot(v) +- sqrt(u.dot(v)^2 - (||v||^2 -max^2))
		// m = u.dot(v)
		// s = -m + sqrt(m^2 - c)
		//
		//
		//
		// notes:
		// 	- s (=scaling factor) needs to be positive
		// 	- (max - ||v||) always larger than zero, otherwise it never entered this if-statement
		// 推导要点：求一个正的缩放因子 s，使 v0 + s * unit(v1) 正好落在半径 max 的圆上。
		// 中文：求 v0 + s * unit(v1) 落在半径 max 圆上的正解。
		// 这样既保留 v0，又尽可能沿 v1 方向加入前馈量。
		Vector2f u1 = v1.normalized();
		float m = u1.dot(v0);
		float c = v0.dot(v0) - max * max;
		float s = -m + sqrtf(m * m - c);
		return v0 + u1 * s;
	}
}

// 计算球与线段/直线的交点或最近点；当前 mc_pos_control 未实际使用。
bool cross_sphere_line(const Vector3f &sphere_c, const float sphere_r,
		       const Vector3f &line_a, const Vector3f &line_b, Vector3f &res)
{
	// project center of sphere on line  normalized AB
	// 将球心投影到 AB 直线上，并使用归一化的 AB 方向。
	// 将球心投影到线段 AB 所在直线；该函数目前未在 mc_pos_control 中使用。
	Vector3f ab_norm = line_b - line_a;

	if (ab_norm.length() < 0.01f) {
		return true;
	}

	ab_norm.normalize();
	Vector3f d = line_a + ab_norm * ((sphere_c - line_a) * ab_norm);
	float cd_len = (sphere_c - d).length();

	if (sphere_r > cd_len) {
		// we have triangle CDX with known CD and CX = R, find DX
		// 已知直角三角形 CDX 中 CD 和半径 CX，计算沿直线方向的 DX。
		float dx_len = sqrtf(sphere_r * sphere_r - cd_len * cd_len);

		if ((sphere_c - line_b) * ab_norm > 0.f) {
			// target waypoint is already behind us
			// 目标点已经在身后，结果钳到线段终点。
			res = line_b;

		} else {
			// target is in front of us
			// 目标点仍在前方，取前向交点。
			res = d + ab_norm * dx_len; // vector A->B on line
		}

		return true;

	} else {

		// have no roots, return D
		// 没有交点时返回投影点 D，并按线段端点做钳位。
		res = d; // go directly to line

		// previous waypoint is still in front of us
		// 起点仍在前方时，结果钳到起点。
		if ((sphere_c - line_a) * ab_norm < 0.f) {
			res = line_a;
		}

		// target waypoint is already behind us
		// 终点已经在身后时，结果钳到终点。
		if ((sphere_c - line_b) * ab_norm > 0.f) {
			res = line_b;
		}

		return false;
	}
}

// 将 addition 加到 setpoint 上，NaN 表示“不设置/不贡献”。
void addIfNotNan(float &setpoint, const float addition)
{
	if (PX4_ISFINITE(setpoint) && PX4_ISFINITE(addition)) {
		// No NAN, add to the setpoint
		// 原设定值和附加量都有效：做前馈叠加。
		setpoint += addition;

	} else if (!PX4_ISFINITE(setpoint)) {
		// Setpoint NAN, take addition
		// 原设定值未设置：如果 addition 有效，就用 addition 接管该轴。
		setpoint = addition;
	}

	// Addition is NAN or both are NAN, nothing to do
	// addition 为 NaN 时表示“不贡献这一项”，保持原值。
}

// Vector3f 版本的 addIfNotNan，对三个轴分别执行 NaN 兼容叠加。
void addIfNotNanVector3f(Vector3f &setpoint, const Vector3f &addition)
{
	for (int i = 0; i < 3; i++) {
		addIfNotNan(setpoint(i), addition(i));
	}
}

// 把 Vector3f 中的 NaN 元素替换为 0，便于后续限幅/向量运算。
void setZeroIfNanVector3f(Vector3f &vector)
{
	// Adding zero vector overwrites elements that are NaN with zero
	// 利用 addIfNotNanVector3f 的语义，添加零向量会把 NaN 元素替换为 0。
	addIfNotNanVector3f(vector, Vector3f());
}

} // ControlMath
