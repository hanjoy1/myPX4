#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
    Copyright (c) 2022-2024 PX4 Development Team
    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in
    the documentation and/or other materials provided with the
    distribution.
    3. Neither the name PX4 nor the names of its contributors may be
    used to endorse or promote products derived from this software
    without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
    OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
    AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.

File: derivation_yaw_estimator.py
Description: 使用 SymForce 推导 EKF-GSF 航向估计器的协方差预测和速度观测更新公式。

该脚本只负责符号推导和生成 C++ 头文件，不会在飞行时运行。生成的公式由
EKFGSF_yaw.cpp 调用。这里的三维误差状态为 [delta_v_N, delta_v_E, delta_yaw]，
它属于应急航向估计器，并不是 PX4 主 EKF 的完整状态。
"""

import symforce
# 让 SymForce 在可能出现除零的位置保留符号 epsilon。代码生成后，调用者可以
# 提供合适的数值保护，避免符号化阶段擅自使用一个固定常数。
symforce.set_epsilon_to_symbol()

import symforce.symbolic as sf
from symforce.values import Values

# 复用主 EKF 的代码生成工具，将符号表达式转换为符合 PX4 风格的 C++ 函数。
import os, sys
derivation_utils_dir = os.path.dirname(os.path.abspath(__file__)) + "/../../python/ekf_derivation/utils"
sys.path.append(derivation_utils_dir)
import derivation_utils

# 名义状态：东北方向速度 vel=[v_N, v_E]，以及二维旋转 R（即航向角）。
# 航向采用 Rot2 而不是一个裸标量，可以借助旋转群运算自然处理 +/-pi 环绕。
State = Values(
    vel = sf.V2(),
    R = sf.Rot2()
)

# SymForce 函数参数需要固定尺寸的矩阵类型。State 的切空间维数为 3：
# 两个速度误差和一个航向误差。
class VTangent(sf.Matrix):
    SHAPE = (State.tangent_dim(), 1)

class MTangent(sf.Matrix):
    SHAPE = (State.tangent_dim(), State.tangent_dim())

def rot2_small_angle(angle: sf.V1):
    # 小角度近似：cos(delta_yaw) ~= 1，sin(delta_yaw) ~= delta_yaw。
    # 它用于构造误差旋转，避免在误差状态线性化时引入三角函数。
    return sf.Rot2(sf.Complex(1, angle[0]))

def yaw_est_predict_covariance(
        state: VTangent,
        P: MTangent,
        d_vel: sf.V2,
        d_vel_var: sf.Scalar,
        d_ang: sf.Scalar,
        d_ang_var: sf.Scalar,
):
    """推导一个航向子模型的协方差预测 P_new = F P F^T + G R_u G^T。

    state 是名义状态；d_vel 是机体系水平速度增量；d_ang 是偏航角增量。
    d_vel_var 和 d_ang_var 分别表示这两个 IMU 输入的噪声方差。
    此函数只生成协方差公式，名义状态的实际更新在 EKFGSF_yaw.cpp 中完成。
    """
    # 将扁平的三维切空间输入还原为 Values，便于用 vel 和 R 访问状态。
    state = State.from_tangent(state)
    # 转为一维向量，以便使用 SymForce 李群/切空间相关接口。
    d_ang = sf.V1(d_ang)

    # 定义小误差状态 delta_x=[delta_v_N, delta_v_E, delta_yaw]。
    state_error = Values(
        vel = sf.V2.symbolic("delta_vel"),
        yaw = sf.V1.symbolic("delta_yaw")
    )

    # “真实状态”用名义状态加一个无穷小误差来表示。速度误差直接相加，
    # 航向误差则右乘到名义旋转上。
    state_t = Values(
        vel = state["vel"] + state_error["vel"],
        R = state["R"] * rot2_small_angle(state_error["yaw"])
    )

    # 定义 IMU 输入噪声：二维速度增量噪声 a_n 和偏航角增量噪声 w_n。
    noise = Values(
        d_vel = sf.V2.symbolic("a_n"),
        d_ang = sf.V1.symbolic("w_n"),
    )

    # 真实输入 = 测得输入 - 测量噪声。这里的符号约定决定了后续 G 的符号，
    # 但在 Q=G R_u G^T 中不会改变最终噪声协方差的正定性。
    input_t = Values(
        d_vel = d_vel - noise["d_vel"],
        d_ang = d_ang - noise["d_ang"]
    )

    # 使用真实状态和真实输入传播一步，得到带状态误差及输入噪声的状态。
    # d_vel 在机体系中，因此需要用航向旋转到 N-E 导航坐标系。
    state_t_pred = Values(
        vel = state_t["vel"] + state_t["R"] * input_t["d_vel"],
        R = state_t["R"] * rot2_small_angle(input_t["d_ang"])
    )

    # 使用无误差的名义状态和测得输入传播一步。
    state_pred = Values(
        vel = state["vel"] + state["R"] * d_vel,
        R = state["R"] * rot2_small_angle(d_ang)
    )

    # 由“真实传播结果 - 名义传播结果”构造下一时刻误差状态。
    # 旋转误差不能直接相减，应使用 R_nominal^-1 * R_true 得到相对旋转。
    delta_rot = (state_pred["R"].inverse() * state_t_pred["R"])
    state_error_pred = Values(
        vel = state_t_pred["vel"] - state_pred["vel"],
        # 小角度下复数旋转的虚部约等于 delta_yaw。simplify 还会消去 R^-1*R。
        yaw = sf.simplify(delta_rot.z.imag)
    )

    # EKF 雅可比在零误差、零噪声处进行一阶线性化。
    zero_state_error = {state_error[key]: state_error[key].zero() for key in state_error.keys()}
    zero_noise = {noise[key]: noise[key].zero() for key in noise.keys()}

    # 状态转移矩阵 F = partial(delta_x_next) / partial(delta_x)。
    F = VTangent(state_error_pred.to_storage()).jacobian(state_error).subs(zero_state_error).subs(zero_noise)

    # 噪声映射矩阵 G = partial(delta_x_next) / partial(IMU noise)。这里假设进入
    # 航向估计器的角增量和速度增量已经完成主 EKF 陀螺仪/加速度计零偏补偿。
    G = VTangent(state_error_pred.to_storage()).jacobian(noise).subs(zero_state_error).subs(zero_noise)

    # 输入噪声协方差。两个水平速度增量暂时使用相同方差，且彼此不相关。
    var_u = sf.Matrix.diag([d_vel_var, d_vel_var, d_ang_var])

    # 把 IMU 输入噪声映射到误差状态空间，得到离散过程噪声 Q。
    Q = G * var_u * G.T

    # 标准 EKF 协方差预测方程。
    P_new = F * P * F.T + Q

    # 只生成对角线和上三角公式。协方差矩阵对称，下三角在 C++ 实现中镜像复制，
    # 可以减少生成代码长度和飞控运行时的重复计算。
    for index in range(State.tangent_dim()):
        for j in range(State.tangent_dim()):
            if index > j:
                P_new[index,j] = 0

    return P_new

def yaw_est_compute_measurement_update(
        P: MTangent,
        vel_obs_var: sf.Scalar,
        epsilon : sf.Scalar
):
    """推导 N-E 速度观测更新所需的矩阵。

    观测模型为 z=[v_N, v_E]，因此只直接观测状态的前两维。虽然 H 的航向列
    为零，但速度与航向在 P 中存在互协方差，所以速度观测仍能间接修正航向。
    """
    # H = partial(h(x))/partial(x)，状态顺序为 [v_N, v_E, yaw]。
    H = sf.Matrix([[1,0,0],
                   [0,1,0]])

    # 假设 N、E 速度观测具有相同方差，且两轴测量噪声互不相关。
    R = sf.Matrix([[vel_obs_var , 0],
                   [0 , vel_obs_var]])

    # 创新协方差 S = H P H^T + R。
    S = H * P * H.T + R
    S_det = S[0, 0] * S[1, 1] - S[1, 0] * S[0, 1]
    # 使用带符号 epsilon 的受保护除法，防止 S 接近奇异时直接除零。
    S_det_inv = derivation_utils.add_epsilon_sign(1 / S_det, S_det, epsilon)

    # 对 2x2 矩阵显式求逆，比生成通用矩阵求逆代码更简单、确定。
    S_inv = sf.M22([[S[1, 1], -S[0, 1]], [-S[1, 0], S[0, 0]]]) * S_det_inv
    # 卡尔曼增益。实际状态修正为 delta_x = K * innovation，由 C++ 调用方完成。
    K = (P * H.T) * S_inv
    # 协方差更新。这里生成简化形式 P_new=(I-KH)P。
    P_new = P - K * H * P

    # 同样只输出上三角和对角线；下三角由实现按对称性补齐。
    for index in range(State.tangent_dim()):
        for j in range(State.tangent_dim()):
            if index > j:
                P_new[index,j] = 0

    return (S_inv, S_det_inv, K, P_new)

print("Derive yaw estimator equations...")
# 分别生成协方差预测和速度观测更新头文件。output_names 指定返回元组在生成的
# C++ 函数中对应的输出变量名称。
derivation_utils.generate_px4_function(yaw_est_predict_covariance, output_names=None)
derivation_utils.generate_px4_function(yaw_est_compute_measurement_update, output_names=["S_inv", "S_det_inv", "K", "P_new"])
