/****************************************************************************
 *
 *   Copyright (c) 2022-2025 PX4 Development Team. All rights reserved.
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
 * @file FigureEight.cpp
 * Helper class for fixed wing position controller when flying a figure 8 loiter pattern.
 *
 * 中文说明：固定翼 8 字盘旋/等待航线辅助类。
 * 该模块根据当前位置、地速、风估计和 8 字航线参数生成横向制导输出；
 * 下游仍然使用普通 fixed_wing_lateral_setpoint 风格的接口。
 */

#include "FigureEight.hpp"

#include "lib/geo/geo.h"
#include <lib/matrix/matrix/math.hpp>

using namespace matrix;

static constexpr float NORMALIZED_MAJOR_RADIUS{1.0f};
static constexpr bool NORTH_CIRCLE_IS_COUNTER_CLOCKWISE{false};
static constexpr bool SOUTH_CIRCLE_IS_COUNTER_CLOCKWISE{true};
static constexpr float DEFAULT_MAJOR_TO_MINOR_AXIS_RATIO{2.5f};
static constexpr float MINIMAL_FEASIBLE_MAJOR_TO_MINOR_AXIS_RATIO{2.0f};

/*
 * 8 字航线几何约定：
 * 先在归一化坐标系中定义南北两个圆及其入口/出口点，再按实际半径和朝向
 * 缩放、旋转到本地坐标系。这样可以把几何计算和制导计算分开。
 */

FigureEight::FigureEight(DirectionalGuidance &npfg, matrix::Vector2f &wind_vel) :
	ModuleParams(nullptr),
	_directional_guidance(npfg),
	_wind_vel(wind_vel)
{

}

void FigureEight::resetPattern()
{
	// 重置当前航段，使下一次 updateSetpoint() 重新根据位置初始化 8 字航线段。
	// Set the current segment invalid
	_current_segment = FigureEightSegment::SEGMENT_UNDEFINED;
	_pos_passed_circle_center_along_major_axis = false;
}

DirectionalGuidanceOutput FigureEight::updateSetpoint(const matrix::Vector2f &curr_pos_local,
		const matrix::Vector2f &ground_speed,
		const FigureEightPatternParameters &parameters)
{
	/*
	 * 8 字航线主入口：
	 * 1. 清洗输入参数，保证半径和长短轴比例可行；
	 * 2. 计算两个圆和切换点；
	 * 3. 初始化或更新当前航段；
	 * 4. 调用 DirectionalGuidance 生成横向制导输出。
	 */
	// Sanitize inputs
	FigureEightPatternParameters valid_parameters{sanitizeParameters(parameters)};

	// Calculate the figure eight pattern points.
	FigureEightPatternPoints pattern_points;
	calculateFigureEightPoints(pattern_points, valid_parameters);

	// Do the figure of eight initialization if needed.
	initializePattern(curr_pos_local, ground_speed, valid_parameters, pattern_points);

	// Check if we need to switch to next segment
	updateSegment(curr_pos_local, valid_parameters,  pattern_points);

	// Apply control logic based on segment
	return applyControl(curr_pos_local, ground_speed, valid_parameters, pattern_points);
}

FigureEight::FigureEightPatternParameters FigureEight::sanitizeParameters(const FigureEightPatternParameters
		&parameters)
{
	// 清洗 8 字参数：缺失短轴/长轴时用 loiter 参数补齐，并保证几何可行。
	FigureEightPatternParameters valid_parameters{parameters};

	if (!PX4_ISFINITE(parameters.loiter_minor_radius)) {
		// 未给短轴半径时，使用默认盘旋半径绝对值。
		valid_parameters.loiter_minor_radius = fabsf(_param_nav_loiter_rad.get());
	}

	if (!PX4_ISFINITE(parameters.loiter_radius)) {
		// 未给长轴半径时，按默认长短轴比例生成，并继承默认 loiter 方向。
		valid_parameters.loiter_radius = DEFAULT_MAJOR_TO_MINOR_AXIS_RATIO * valid_parameters.loiter_minor_radius;
		valid_parameters.loiter_direction_counter_clockwise = _param_nav_loiter_rad.get() < -FLT_EPSILON;
	}

	// 长轴至少要达到短轴的最小可行比例，否则两个圆和交叉线段几何会退化。
	valid_parameters.loiter_radius = math::max(valid_parameters.loiter_radius,
					 MINIMAL_FEASIBLE_MAJOR_TO_MINOR_AXIS_RATIO * valid_parameters.loiter_minor_radius);

	return valid_parameters;
}

void FigureEight::initializePattern(const matrix::Vector2f &curr_pos_local, const matrix::Vector2f &ground_speed,
				    const FigureEightPatternParameters &parameters, FigureEightPatternPoints pattern_points)
{
	// Initialize the currently active segment, if it hasn't been active yet, or the pattern has been changed.
	// 中文：首次进入或参数改变时，根据飞机位置/地速选择最合理的初始航段。
	if ((_current_segment == FigureEightSegment::SEGMENT_UNDEFINED) || (_active_parameters != parameters)) {
		Vector2f center_to_pos_local;
		calculatePositionToCenterNormalizedRotated(center_to_pos_local, curr_pos_local, parameters);
		Vector2f ground_speed_rotated = Dcm2f(-calculateRotationAngle(parameters)) * ground_speed;

		Vector2f north_center_to_pos_local{center_to_pos_local - pattern_points.normalized_north_circle_offset};
		Vector2f south_center_to_pos_local{center_to_pos_local - pattern_points.normalized_south_circle_offset};
		const bool north_is_closer = north_center_to_pos_local.norm() < south_center_to_pos_local.norm();

		// Get the normalized switch distance.
		// 中文：NPFG 切换距离按归一化长轴半径缩放，方便在单位几何中比较。
		float switch_distance_normalized = _directional_guidance.switchDistance(FLT_MAX) * NORMALIZED_MAJOR_RADIUS /
						   parameters.loiter_radius;

		//Far away from current figure of eight. Fly towards closer circle
		// 中文：离 8 字中心较远时，先进入较近的圆，不强行追交叉线。

		if (center_to_pos_local.norm() > NORMALIZED_MAJOR_RADIUS + switch_distance_normalized) {
			if (north_is_closer) {
				_current_segment = FigureEightSegment::SEGMENT_CIRCLE_NORTH;

			} else {
				_current_segment = FigureEightSegment::SEGMENT_CIRCLE_SOUTH;
			}

			_pos_passed_circle_center_along_major_axis = true;

		} else {
			if (north_is_closer) {
				const bool is_circling_counter_clockwise{north_center_to_pos_local.cross(ground_speed_rotated) < 0.f};

				if ((ground_speed_rotated(0) > 0.f) && (is_circling_counter_clockwise == NORTH_CIRCLE_IS_COUNTER_CLOCKWISE)) {
					// Flying north and right rotation
					// 中文：位置/速度方向已经接近北圆正确切入姿态，直接跟北圆。
					_current_segment = FigureEightSegment::SEGMENT_CIRCLE_NORTH;
					_pos_passed_circle_center_along_major_axis = true;

				} else {
					// Flying to the entry of the south circle
					// 中文：否则先飞向南圆入口点，进入完整 8 字循环。
					_current_segment = FigureEightSegment::SEGMENT_POINT_SOUTHWEST;
					_pos_passed_circle_center_along_major_axis = false;
				}

			} else {
				const bool is_circling_counter_clockwise{south_center_to_pos_local.cross(ground_speed_rotated) < 0.f};

				if ((ground_speed_rotated(0) < 0.f) && (is_circling_counter_clockwise == SOUTH_CIRCLE_IS_COUNTER_CLOCKWISE)) {
					// Flying south and right rotation
					// 中文：已经适合南圆切入时直接跟南圆。
					_current_segment = FigureEightSegment::SEGMENT_CIRCLE_SOUTH;
					_pos_passed_circle_center_along_major_axis = true;

				} else {
					// Flying to the entry of the north circle
					// 中文：否则先飞向北圆入口点。
					_current_segment = FigureEightSegment::SEGMENT_POINT_NORTHWEST;
					_pos_passed_circle_center_along_major_axis = false;
				}
			}
		}

		_active_parameters = parameters;
	}
}

void FigureEight::calculateFigureEightPoints(FigureEightPatternPoints &pattern_points,
		const FigureEightPatternParameters &parameters)
{
	// 在归一化坐标系中计算两个圆心、入口点和出口点。
	const float normalized_minor_radius = (parameters.loiter_minor_radius / parameters.loiter_radius) *
					      NORMALIZED_MAJOR_RADIUS;
	// 过渡角由两个圆的切线几何确定，用于连接北/南圆的斜线段。
	const float cos_transition_angle = parameters.loiter_minor_radius / (parameters.loiter_radius -
					   parameters.loiter_minor_radius);
	const float sin_transition_angle = sqrtf(1.0f - cos_transition_angle * cos_transition_angle);
	pattern_points.normalized_north_circle_offset = Vector2f{NORMALIZED_MAJOR_RADIUS - normalized_minor_radius, 0.0f};
	pattern_points.normalized_north_entry_offset =  Vector2f{NORMALIZED_MAJOR_RADIUS - ((normalized_minor_radius) * (1.0f + cos_transition_angle)),
								 -normalized_minor_radius * sin_transition_angle};
	pattern_points.normalized_north_exit_offset = Vector2f{NORMALIZED_MAJOR_RADIUS - ((normalized_minor_radius) * (1.0f + cos_transition_angle)),
							       normalized_minor_radius * sin_transition_angle};
	pattern_points.normalized_south_circle_offset = Vector2f{-NORMALIZED_MAJOR_RADIUS + normalized_minor_radius, 0.0f};
	pattern_points.normalized_south_entry_offset = Vector2f{-NORMALIZED_MAJOR_RADIUS + ((normalized_minor_radius) * (1.0f + cos_transition_angle)),
								-normalized_minor_radius * sin_transition_angle};
	pattern_points.normalized_south_exit_offset = Vector2f{-NORMALIZED_MAJOR_RADIUS + ((normalized_minor_radius) * (1.0f + cos_transition_angle)),
							       normalized_minor_radius * sin_transition_angle};
}

void FigureEight::updateSegment(const matrix::Vector2f &curr_pos_local, const FigureEightPatternParameters &parameters,
				const FigureEightPatternPoints &pattern_points)
{
	// 根据当前位置判断是否从当前航段切换到下一航段。
	Vector2f center_to_pos_local;
	calculatePositionToCenterNormalizedRotated(center_to_pos_local, curr_pos_local, parameters);

	// Get the normalized switch distance.
	// 中文：所有切换判断都在归一化/旋转后的 8 字坐标系中进行。
	float switch_distance_normalized = _directional_guidance.switchDistance(FLT_MAX) * NORMALIZED_MAJOR_RADIUS /
					   parameters.loiter_radius;

	// Update segment if segment exit condition has been reached
	// 中文：航段顺序为：北圆 -> 东北到西南线 -> 南圆 -> 东南到西北线 -> 北圆。
	switch (_current_segment) {
	case FigureEightSegment::SEGMENT_CIRCLE_NORTH: {
			if (center_to_pos_local(0) > pattern_points.normalized_north_circle_offset(0)) {
				// 确认飞机已越过北圆中心主轴，避免过早切出圆。
				_pos_passed_circle_center_along_major_axis = true;
			}

			Vector2f vector_to_exit_normalized = pattern_points.normalized_north_exit_offset - center_to_pos_local;

			/* Exit condition: Switch distance away from north-east point of north circle and at least once was above the circle center. Failsafe action, if poor tracking,
			-                       switch to next if the vehicle is on the east side and below the north exit point. */
			// 中文：接近北圆出口且已绕过中心，或跟踪较差但几何上已越过出口区域，则切到交叉线段。
			if (_pos_passed_circle_center_along_major_axis &&
			    ((vector_to_exit_normalized.norm() < switch_distance_normalized) ||
			     ((center_to_pos_local(0) < pattern_points.normalized_north_exit_offset(0)) &&
			      (center_to_pos_local(1) > FLT_EPSILON) &&
			      (center_to_pos_local.norm() < NORMALIZED_MAJOR_RADIUS)))) {
				_current_segment = FigureEightSegment::SEGMENT_NORTHEAST_SOUTHWEST;
			}
		}
		break;

	case FigureEightSegment::SEGMENT_POINT_SOUTHWEST: // fall through
	case FigureEightSegment::SEGMENT_NORTHEAST_SOUTHWEST: {
			// 从北圆或当前位置飞向南圆入口，进入此段后重新等待“越过圆心”条件。
			_pos_passed_circle_center_along_major_axis = false;
			Vector2f vector_to_exit_normalized = pattern_points.normalized_south_entry_offset - center_to_pos_local;

			/* Exit condition: Switch distance away from south-west point of south circle. Failsafe action, if poor tracking,
			switch to next if the vehicle is on the west side and below entry point of the south circle or has left the radius. */
			// 中文：接近南圆入口或已越过入口保护区域时，切入南圆。
			if ((vector_to_exit_normalized.norm() < switch_distance_normalized) ||
			    ((center_to_pos_local(0) < pattern_points.normalized_south_entry_offset(0)) && (center_to_pos_local(1) < FLT_EPSILON))
			    ||
			    (center_to_pos_local(0) < -NORMALIZED_MAJOR_RADIUS)) {
				_current_segment = FigureEightSegment::SEGMENT_CIRCLE_SOUTH;
			}
		}
		break;

	case FigureEightSegment::SEGMENT_CIRCLE_SOUTH: {
			if (center_to_pos_local(0) < pattern_points.normalized_south_circle_offset(0)) {
				// 确认飞机已越过南圆中心主轴，避免过早切出圆。
				_pos_passed_circle_center_along_major_axis = true;
			}

			Vector2f vector_to_exit_normalized = pattern_points.normalized_south_exit_offset - center_to_pos_local;

			/* Exit condition: Switch distance away from south-east point of south circle and at least once was below the circle center. Failsafe action, if poor tracking,
			-                       switch to next if the vehicle is on the east side and above the south exit point. */
			// 中文：接近南圆出口且已绕过中心，或几何上已越过出口区域，则切到下一条交叉线段。
			if (_pos_passed_circle_center_along_major_axis &&
			    ((vector_to_exit_normalized.norm() < switch_distance_normalized) ||
			     ((center_to_pos_local(0) > pattern_points.normalized_south_exit_offset(0)) &&
			      (center_to_pos_local(1) > FLT_EPSILON) &&
			      (center_to_pos_local.norm() < NORMALIZED_MAJOR_RADIUS)))) {
				_current_segment = FigureEightSegment::SEGMENT_SOUTHEAST_NORTHWEST;
			}

		}
		break;

	case FigureEightSegment::SEGMENT_POINT_NORTHWEST: // Fall through
	case FigureEightSegment::SEGMENT_SOUTHEAST_NORTHWEST: {
			// 从南圆或当前位置飞向北圆入口。
			_pos_passed_circle_center_along_major_axis = false;
			Vector2f vector_to_exit_normalized = pattern_points.normalized_north_entry_offset - center_to_pos_local;

			/* Exit condition: Switch distance away from north-west point of north circle. Failsafe action, if poor tracking,
			switch to next if the vehicle is on the west side and above entry point of the north circle or has left the radius. */
			// 中文：接近北圆入口或已越过入口保护区域时，切入北圆。
			if ((vector_to_exit_normalized.norm() < switch_distance_normalized) ||
			    ((center_to_pos_local(0) > pattern_points.normalized_north_entry_offset(0)) && (center_to_pos_local(1) < FLT_EPSILON))
			    ||
			    (center_to_pos_local(0) > NORMALIZED_MAJOR_RADIUS)) {
				_current_segment = FigureEightSegment::SEGMENT_CIRCLE_NORTH;
			}
		}
		break;

	case FigureEightSegment::SEGMENT_UNDEFINED:
	default:
		break;
	}
}

DirectionalGuidanceOutput FigureEight::applyControl(const matrix::Vector2f &curr_pos_local,
		const matrix::Vector2f &ground_speed,
		const FigureEightPatternParameters &parameters,
		const FigureEightPatternPoints &pattern_points)
{
	// 根据当前航段选择“圆跟踪”或“直线跟踪”控制。
	Vector2f center_to_pos_local;
	calculatePositionToCenterNormalizedRotated(center_to_pos_local, curr_pos_local, parameters);

	switch (_current_segment) {
	case FigureEightSegment::SEGMENT_CIRCLE_NORTH: {
			return applyCircle(NORTH_CIRCLE_IS_COUNTER_CLOCKWISE, pattern_points.normalized_north_circle_offset, curr_pos_local,
					   ground_speed, parameters);
		}
		break;

	case FigureEightSegment::SEGMENT_NORTHEAST_SOUTHWEST: {
			// Follow path from north-east to south-west
			// 中文：跟踪北圆出口到南圆入口的交叉直线段。
			return applyLine(pattern_points.normalized_north_exit_offset, pattern_points.normalized_south_entry_offset,
					 curr_pos_local,
					 ground_speed, parameters);
		}
		break;

	case FigureEightSegment::SEGMENT_CIRCLE_SOUTH: {
			return applyCircle(SOUTH_CIRCLE_IS_COUNTER_CLOCKWISE, pattern_points.normalized_south_circle_offset, curr_pos_local,
					   ground_speed, parameters);
		}
		break;

	case FigureEightSegment::SEGMENT_SOUTHEAST_NORTHWEST: {
			// follow path from south-east to north-west
			// 中文：跟踪南圆出口到北圆入口的交叉直线段。
			return applyLine(pattern_points.normalized_south_exit_offset, pattern_points.normalized_north_entry_offset,
					 curr_pos_local,
					 ground_speed, parameters);
		}
		break;

	case FigureEightSegment::SEGMENT_POINT_SOUTHWEST: {
			// Follow path from current position to south-west
			// 中文：初始化过渡段，从当前位置飞到南圆入口。
			return applyLine(center_to_pos_local, pattern_points.normalized_south_entry_offset, curr_pos_local,
					 ground_speed, parameters);
		}
		break;

	case FigureEightSegment::SEGMENT_POINT_NORTHWEST: {
			// Follow path from current position to north-west
			// 中文：初始化过渡段，从当前位置飞到北圆入口。
			return applyLine(center_to_pos_local, pattern_points.normalized_north_entry_offset, curr_pos_local,
					 ground_speed, parameters);
		}
		break;

	case FigureEightSegment::SEGMENT_UNDEFINED:
	default:
		return DirectionalGuidanceOutput{};
		break;
	}
}

void FigureEight::calculatePositionToCenterNormalizedRotated(matrix::Vector2f &center_to_pos_local_normalized_rotated,
		const matrix::Vector2f &curr_pos_local, const FigureEightPatternParameters &parameters) const
{
	// 把本地位置转换到“以 8 字中心为原点、长轴归一化、已旋转到标准方向”的坐标系。
	Vector2f center_to_pos_local = curr_pos_local - parameters.center_pos_local;

	// normalize position with respect to radius
	// 中文：长短轴坐标统一按长轴半径归一化，便于与标准几何点比较。
	Vector2f center_to_pos_local_normalized;
	center_to_pos_local_normalized(0) = center_to_pos_local(0) * NORMALIZED_MAJOR_RADIUS / parameters.loiter_radius;
	center_to_pos_local_normalized(1) = center_to_pos_local(1) * NORMALIZED_MAJOR_RADIUS / parameters.loiter_radius;

	// rotate position with respect to figure eight orientation and direction.
	// 中文：按 8 字朝向和方向旋转，使标准几何始终可复用。
	center_to_pos_local_normalized_rotated = Dcm2f(-calculateRotationAngle(parameters)) * center_to_pos_local_normalized;
}

float FigureEight::calculateRotationAngle(const FigureEightPatternParameters &parameters) const
{
	// rotate position with respect to figure eight orientation and direction.
	// 中文：基础旋转角来自任务给定的 8 字朝向。
	float yaw_rotation = parameters.loiter_orientation;

	// figure eight pattern is symmetric, changing the direction is the same as a rotation by 180° around center
	// 中文：8 字关于中心对称，反向飞等效于整体旋转 180 度。
	if (parameters.loiter_direction_counter_clockwise) {
		yaw_rotation += M_PI_F;
	}

	return yaw_rotation;
}

DirectionalGuidanceOutput FigureEight::applyCircle(bool loiter_direction_counter_clockwise,
		const matrix::Vector2f &normalized_circle_offset,
		const matrix::Vector2f &curr_pos_local, const matrix::Vector2f &ground_speed,
		const FigureEightPatternParameters &parameters)
{
	// 跟踪 8 字中的一个圆段：计算实际圆心、切向、曲率，并调用 NPFG。
	const float loiter_direction_multiplier = loiter_direction_counter_clockwise ? -1.f : 1.f;

	// 将归一化圆心偏移缩放回真实半径，并旋转到实际 8 字方向。
	Vector2f circle_offset = normalized_circle_offset * (parameters.loiter_radius / NORMALIZED_MAJOR_RADIUS);
	Vector2f circle_offset_rotated = Dcm2f(calculateRotationAngle(parameters)) * circle_offset;
	Vector2f circle_center = parameters.center_pos_local + circle_offset_rotated;


	const Vector2f vector_center_to_vehicle = curr_pos_local - circle_center;
	const float dist_to_center = vector_center_to_vehicle.norm();

	Vector2f unit_vec_center_to_closest_pt = vector_center_to_vehicle.normalized();

	if (dist_to_center < 0.1f) {
		// the logic breaks down at the circle center, employ some mitigation strategies
		// until we exit this region
		// 中文：飞机几乎在圆心时径向方向不唯一，用地速方向或默认方向兜底。
		if (ground_speed.norm() < 0.1f) {
			// arbitrarily set the point in the northern top of the circle
			// 中文：低地速时任选圆上北向点。
			unit_vec_center_to_closest_pt = Vector2f{1.0f, 0.0f};

		} else {
			// set the point in the direction we are moving
			// 中文：有地速时用运动方向决定圆周最近点方向。
			unit_vec_center_to_closest_pt = ground_speed.normalized();
		}
	}

	// 圆切向为径向旋转 90 度，并按顺/逆时针符号修正。
	const Vector2f unit_path_tangent = loiter_direction_multiplier * Vector2f{-unit_vec_center_to_closest_pt(1), unit_vec_center_to_closest_pt(0)};

	// 曲率为 1 / 短轴半径，方向由 loiter_direction_multiplier 表示。
	float path_curvature = loiter_direction_multiplier / parameters.loiter_minor_radius;
	_target_bearing = atan2f(unit_path_tangent(1), unit_path_tangent(0));
	_closest_point_on_path = unit_vec_center_to_closest_pt * parameters.loiter_minor_radius + circle_center;
	return _directional_guidance.guideToPath(curr_pos_local, ground_speed, _wind_vel, unit_path_tangent,
			_closest_point_on_path, path_curvature);

}

DirectionalGuidanceOutput FigureEight::applyLine(const matrix::Vector2f &normalized_line_start_offset,
		const matrix::Vector2f &normalized_line_end_offset, const matrix::Vector2f &curr_pos_local,
		const matrix::Vector2f &ground_speed, const FigureEightPatternParameters &parameters)
{
	// 跟踪 8 字中的直线段：把归一化起止点缩放/旋转到本地坐标，再调用 NPFG。
	const Dcm2f rotation_matrix(calculateRotationAngle(parameters));

	// Calculate start offset depending on radius
	// 中文：起点偏移从归一化坐标恢复为真实本地坐标。
	const Vector2f start_offset = normalized_line_start_offset * (parameters.loiter_radius / NORMALIZED_MAJOR_RADIUS);
	const Vector2f start_offset_rotated = rotation_matrix * start_offset;
	const Vector2f line_segment_start_position = parameters.center_pos_local + start_offset_rotated;

	const Vector2f end_offset = normalized_line_end_offset * (parameters.loiter_radius / NORMALIZED_MAJOR_RADIUS);
	// 终点偏移同样恢复并旋转。
	const Vector2f end_offset_rotated = rotation_matrix * end_offset;
	const Vector2f line_segment_end_position = parameters.center_pos_local + end_offset_rotated;

	const Vector2f path_tangent = line_segment_end_position - line_segment_start_position;
	const Vector2f unit_path_tangent = path_tangent.normalized();
	_target_bearing = atan2f(unit_path_tangent(1), unit_path_tangent(0));
	const Vector2f vector_A_to_vehicle = curr_pos_local - line_segment_start_position;
	// 最近路径点为飞机在该直线段所在无限直线上的投影。
	_closest_point_on_path = line_segment_start_position + vector_A_to_vehicle.dot(unit_path_tangent) * unit_path_tangent;
	return _directional_guidance.guideToPath(curr_pos_local, ground_speed, _wind_vel, path_tangent.normalized(),
			line_segment_start_position,
			0.0f);
}
