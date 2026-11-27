#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
    Copyright (c) 2022-2023 PX4 Development Team
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

File: derivation.py
Description:
    Derivation of an error-state EKF based on
    Sola, Joan. "Quaternion kinematics for the error-state Kalman filter." arXiv preprint arXiv:1711.02508 (2017).
    The derivation is directly done in discrete-time as this allows us to define the desired type of discretization
    for each element while defining the equations (easier than a continuous-time derivation followed by a block-wise discretization).
"""

# 中文阅读导航：本文件在开发/构建阶段进行符号推导，并将公式生成为 generated/*.h。
# 飞控运行时调用生成的 C++，例如 covariance.cpp 中的 sym::PredictCovariance()。
# 运行文件末尾会写生成目录；学习公式时可先阅读函数定义，再看末尾生成哪些输出。
#
# 建议顺序：State -> predict_covariance -> jacobian_chain_rule -> 一个熟悉的观测模型
# -> 对应 compute_* -> generate_px4_function。先理解模型，再看生成头文件的代数展开。
#
# 全文件的数学约定：
# 1. 导航系 n 使用 NED：北、东、下；机体系 b 使用 FRD：前、右、下。
#    q 表示机体系到导航系旋转，C = C_b^n；q * v_b 相当于 C * v_b。
#    q.inverse() * v_n 相当于 C.T * v_n。这里 * 的意义由对象类型决定。
# 2. 名义状态记作 x，真实状态记作 x_t，误差记作 delta_x = x_t - x。
#    姿态不能直接相减，使用 q_t = delta_q * q，delta_q 约为 [1, delta_theta/2]。
#    这是左乘姿态误差，delta_theta 在导航系表达；不要与机体系右乘误差混用。
# 3. 默认配置：名义存储 25 维，误差状态 24 维；四元数存 4 个数，旋转自由度是 3。
#    P、K、H 都对应误差状态；禁用磁场或风状态时，维数和索引会随之改变。
# 4. 对标量观测 h(x)，数学上 H = dh/d(delta_x) 为 1 x N 行向量。
#    本脚本常返回 H.T，即 N x 1 列向量，以匹配 PX4 C++ 的 VectorState 存储方式。
#    标量创新方差 S = H*P*H.T + R；R 是观测噪声方差，不是标准差。
# 5. 多数显式创新采用 innovation = prediction - measurement，C++ 状态修正对应 -K*innovation。
#    状态修正、门限、启停、超时和重置逻辑在 C++ 中，未全部放进本文件。
# 6. 本文件中的 R 有局部不同含义：预测函数中可能是姿态旋转对象，观测函数参数中常是
#    噪声方差，风初始化中则是输入误差协方差矩阵；应结合所在函数区分。
#
# 符号工具读法：symbolic()/Symbol() 创建代数未知量，jacobian() 求解析偏导，
# subs() 做符号代入，.T 是转置，[0,0] 将 1 x 1 矩阵取为标量。
# 这些运算在这里构造公式，并不是用真实传感器数据做一次数值 EKF 更新。
# 公式中的三种“零”要区分：零误差是线性化点，零白噪声是均值假设，零观测则是
# 某些具体运动约束的假设；它们都不意味着对应方差也为零。

import argparse

import symforce
# 将数学运算使用的 epsilon 保留为符号，让生成函数能按需要接收数值保护参数。
# epsilon 用于范数、atan2、除法附近的正则化，不代表传感器观测噪声。
symforce.set_epsilon_to_symbol()

import symforce.symbolic as sf
from symforce import typing as T
from symforce import ops
from symforce.values import Values

import sympy as sp
# 导入代码生成、状态结构生成与带符号 epsilon 保护等工具。
from utils.derivation_utils import *

# Initialize parser
# 两个命令行开关会真正删除状态与误差状态中的分量，并裁减相关生成函数。
# 它们用于构建配置裁减，不是运行时融合启停参数。
parser = argparse.ArgumentParser()

parser.add_argument("--disable_mag", action='store_true', help="disable mag")
parser.add_argument("--disable_wind", action='store_true', help="disable wind")

# Read arguments from command line
args = parser.parse_args()

# The state vector is organized in an ordered dictionary
# Values 保持字段顺序，该顺序决定生成结构的存储排列与误差索引。
# quat_nominal: q_b^n；vel: NED 地速 m/s；pos: 位置 m。
# gyro_bias: 机体系角速度零偏 rad/s；accel_bias: 机体系比力零偏 m/s^2。
# mag_I: 导航系地磁 Gauss；mag_B: 机体系磁偏置 Gauss。
# wind_vel: 导航系水平风速 m/s；terrain: 与垂直位置共用基准的地形向下坐标 m。
# C++ 实际全局位置还由 _gpos 管理，不能把这里的 pos 直接视为始终累积的局部输出位置。
State = Values(
    quat_nominal = sf.Rot3(),
    vel = sf.V3(),
    pos = sf.V3(),
    gyro_bias = sf.V3(),
    accel_bias = sf.V3(),
    mag_I = sf.V3(),
    mag_B = sf.V3(),
    wind_vel = sf.V2(),
    terrain = sf.V1()
)

if args.disable_mag:
    del State["mag_I"]
    del State["mag_B"]

if args.disable_wind:
    del State["wind_vel"]

# 保存某个子状态在误差向量中的起始索引 idx 和自由度 dof。
# 默认姿态 idx=0/dof=3，速度 idx=3/dof=3；它们与四元数存储向量索引不同。
class IdxDof():
    def __init__(self, idx, dof):
        self.idx = idx
        self.dof = dof

def BuildTangentStateIndex():
    # 切空间 tangent space 是旋转等非欧氏状态的局部小误差空间。
    # 对普通向量 tangent_dim 等于元素数，对 Rot3 则为 3，而 storage_dim 为 4。
    # Build a dictionary that can be used to access elements of vectors
    # and matrices defined in the state tangent space (e.g.: P, K and H)
    tangent_state_index = {}
    idx = 0
    for key in State.keys_recursive():
        dof = State[key].tangent_dim()
        tangent_state_index[key] = IdxDof(idx, dof)
        idx += dof
    return tangent_state_index

tangent_idx = BuildTangentStateIndex()

# 固定符号矩阵形状，供符号计算与代码生成识别输入输出维度。
# VState 为名义存储列向量，VTangent 为误差列向量，MTangent 为误差协方差方阵。
class VState(sf.Matrix):
    SHAPE = (State.storage_dim(), 1)

class VTangent(sf.Matrix):
    SHAPE = (State.tangent_dim(), 1)

class MTangent(sf.Matrix):
    SHAPE = (State.tangent_dim(), State.tangent_dim())

def vstate_to_state(v: VState):
    # C++ StateSample 向量中的四元数采用 [w,x,y,z]，SymForce 原生存储为 [x,y,z,w]。
    # 先按字段恢复 Values，再显式重排四元数，保证后续旋转对应同一物理姿态。
    # 这里只改变解释/排列方式，没有执行姿态更新或估计误差注入。
    state = State.from_storage(v)
    q_px4 = state["quat_nominal"].to_storage()
    state["quat_nominal"] = sf.Rot3(sf.Quaternion(xyz=sf.V3(q_px4[1], q_px4[2], q_px4[3]), w=q_px4[0]))
    return state

def predict_covariance(
    state: VState,
    P: MTangent,
    accel: sf.V3,
    accel_var: sf.V3,
    gyro: sf.V3,
    gyro_var: sf.Scalar,
    dt: sf.Scalar
) -> MTangent:

    # 推导目标：delta_x_next ~= A*delta_x + G*n，随后 P_next = A*P*A.T + G*Sigma_n*G.T。
    # 输入 accel、gyro 是机体系测得的比力/角速度，不是已经去零偏的值；dt 单位为秒。
    # C++ 调用时用 IMU 增量除以各自积分时间，恢复本段平均比力和角速度。
    # accel_var 是三轴比力噪声方差，gyro_var 是各轴共用的角速度噪声方差。
    #
    # 这里采用离散一阶误差模型，位置使用 p_next = p + v*dt。
    # ekf.cpp 的名义状态预测还使用梯形位置积分、地球自转和运输项等处理，
    # 因此应将这里视为协方差的线性化模型，而不是完整复制 C++ 的全部积分步骤。
    state = vstate_to_state(state)
    # 同一个已知重力项同时出现在真实与名义预测中，相减后抵消，不进入这里的 A/G。
    g = sf.Symbol("g") # does not appear in the jacobians

    # 每个分量都是用于求导的独立符号误差，不是运行时另存的一套真实传感器状态。
    # 非姿态分量即使符号名字未写 delta，放在 state_error 中仍表示误差。
    state_error = Values(
        theta = sf.V3.symbolic("delta_theta"),
        vel = sf.V3.symbolic("delta_v"),
        pos = sf.V3.symbolic("delta_p"),
        gyro_bias = sf.V3.symbolic("delta_w_b"),
        accel_bias = sf.V3.symbolic("delta_a_b"),
        mag_I = sf.V3.symbolic("mag_I"),
        mag_B = sf.V3.symbolic("mag_B"),
        wind_vel = sf.V2.symbolic("wind_vel"),
        terrain = sf.V1.symbolic("terrain")
    )

    if args.disable_mag:
        del state_error["mag_I"]
        del state_error["mag_B"]

    if args.disable_wind:
        del state_error["wind_vel"]

    # True state kinematics
    # 构造“名义值 + 假设误差”的真实状态，然后比较两者各自预测后的差异。
    state_t = Values()

    for key in state.keys():
        if key == "quat_nominal":
            # Create true quaternion using small angle approximation of the error rotation
            # 小角度：delta_q = [cos(|theta|/2), sin(|theta|/2)*theta/|theta|]
            #                  ~= [1, theta/2]；此处用 xyz/w 参数显式构造，与存储顺序无关。
            # 左乘说明误差在导航系表达；近似四元数只服务于零误差附近的线性化。
            state_t["quat_nominal"] = sf.Rot3(sf.Quaternion(xyz=(state_error["theta"] / 2), w=1)) * state["quat_nominal"]
        else:
            # 速度、位置、零偏、磁场、风和地形采用普通加性误差。
            state_t[key] = state[key] + state_error[key]

    # 模型：测量 = 真实输入 + 真实零偏 + 白噪声，故真实输入 = 测量 - 真实零偏 - 噪声。
    # 这里仅显式引入 3 轴加速度噪声和 3 轴陀螺噪声；其他状态过程噪声由 C++ 额外加入。
    noise = Values(
        accel = sf.V3.symbolic("a_n"),
        gyro = sf.V3.symbolic("w_n"),
    )

    input_t = Values(
        accel = accel - state_t["accel_bias"] - noise["accel"],
        gyro = gyro - state_t["gyro_bias"] - noise["gyro"]
    )

    # 真实状态的离散运动：用真实姿态转比力到 NED，再加向下为正的重力。
    # 角增量来自机体系，故运动预测时姿态增量在右侧：q_next = q * dq(omega*dt)。
    # “运动增量右乘”与前面的“误差左乘”各自表达不同坐标中的旋转，不能交换次序。
    R_t = state_t["quat_nominal"]
    state_t_pred = state_t.copy()
    state_t_pred["quat_nominal"] = state_t["quat_nominal"] * sf.Rot3(sf.Quaternion(xyz=(input_t["gyro"] * dt / 2), w=1))
    state_t_pred["vel"] = state_t["vel"] + (R_t * input_t["accel"] + sf.V3(0, 0, g)) * dt
    state_t_pred["pos"] = state_t["pos"] + state_t["vel"] * dt

    # Nominal state kinematics
    # 名义预测只有估计的零偏可用，不知道真实零偏误差和某次白噪声样本。
    # 用相同运动模型预测名义状态，才能通过两套预测的差提取误差动态。
    input = Values(
        accel = accel - state["accel_bias"],
        gyro = gyro - state["gyro_bias"]
    )

    R = state["quat_nominal"]
    state_pred = state.copy()
    state_pred["quat_nominal"] = state["quat_nominal"] * sf.Rot3(sf.Quaternion(xyz=(input["gyro"] * dt / 2), w=1))
    state_pred["vel"] = state["vel"] + (R * input["accel"] + sf.V3(0, 0, g)) * dt
    state_pred["pos"] = state["pos"] + state["vel"] * dt

    # Error state kinematics
    # 欧氏状态：delta_v_next = v_t_next - v_next，其他加性状态同理。
    # 姿态：delta_q_next = q_t_next * inverse(q_next)，由其向量部恢复小角度。
    state_error_pred = Values()
    for key in state_error.keys():
        if key == "theta":
            delta_q = sf.Quaternion.from_storage(state_t_pred["quat_nominal"].to_storage()) * sf.Quaternion.from_storage(state_pred["quat_nominal"].to_storage()).conj()
            state_error_pred["theta"] = 2 * sf.V3(delta_q.x, delta_q.y, delta_q.z) # Use small angle approximation to obtain a simpler jacobian
        else:
            state_error_pred[key] = state_t_pred[key] - state_pred[key]

    # Simplify angular error state prediction
    # 仅在姿态误差传播表达式中舍去 dt^2 项，并利用单位四元数 q.w^2+q.xyz^2=1 简化。
    # 这不意味着协方差中所有 dt^2 都应去掉：G*Sigma_n*G.T 中仍包含正常的 dt^2 项。
    for i in range(state_error_pred["theta"].storage_dim()):
        state_error_pred["theta"][i] = sp.expand(state_error_pred["theta"][i]).subs(dt**2, 0) # do not consider dt**2 effects in the derivation
        q_est = sf.Quaternion.from_storage(state["quat_nominal"].to_storage())
        state_error_pred["theta"][i] = sp.factor(state_error_pred["theta"][i]).subs(q_est.w**2 + q_est.x**2 + q_est.y**2 + q_est.z**2, 1) # unit norm quaternion

    # EKF 在误差为零、白噪声均值为零处线性化：先保留符号误差求导，再代入零。
    # 若先把误差全部置零，就会失去对误差的敏感度，无法得到正确雅可比。
    zero_state_error = {state_error[key]: state_error[key].zero() for key in state_error.keys()}
    zero_noise = {noise[key]: noise[key].zero() for key in noise.keys()}

    # State propagation jacobian
    # A = d(delta_x_next)/d(delta_x)，对应常见教材中的 F，尺寸 N x N。
    # G = d(delta_x_next)/d(n)，尺寸 N x 6；不是状态协方差，也不是卡尔曼增益。
    # 忽略高阶项时可这样理解主要耦合，C 为当前机体到导航系旋转：
    #   delta_theta_next ~= delta_theta - C*delta_bg*dt - C*n_g*dt
    #   delta_v_next ~= delta_v - [C*(accel-ba)]_x*delta_theta*dt
    #                              - C*delta_ba*dt - C*n_a*dt
    #   delta_p_next = delta_p + delta_v*dt
    # [a]_x 表示叉乘矩阵，[a]_x*b = a cross b；姿态误差因此会转化为速度误差。
    # 简化例子：固定姿态且 C=I，仅保留同一轴 [delta_v,delta_ba]，则
    #   A_sub = [[1,-dt],[0,1]]，delta_v_next = delta_v - delta_ba*dt - n_a*dt。
    # 对应 P_vv_next = P_vv - dt*(P_vb+P_bv) + dt^2*P_bb + dt^2*var(n_a)。
    # P_vb_next = P_vb - dt*P_bb：即使最初速度与零偏不相关，传播也会建立互协方差。
    # 之后速度观测可通过 K_bias = P_bv/S 修正零偏，这就是“速度观测帮助估计 IMU 零偏”的来源。
    A = VTangent(state_error_pred.to_storage()).jacobian(state_error).subs(zero_state_error).subs(zero_noise)
    G = VTangent(state_error_pred.to_storage()).jacobian(noise).subs(zero_state_error).subs(zero_noise)

    # Covariance propagation
    # 噪声顺序与 noise 的字段顺序一致：[n_ax,n_ay,n_az,n_gx,n_gy,n_gz]。
    # 对角形式假设各轴/两种传感器白噪声互不相关；G 自身已包含时间和坐标变换因素。
    var_u = sf.Matrix.diag([accel_var[0], accel_var[1], accel_var[2], gyro_var, gyro_var, gyro_var])
    P_new = A * P * A.T + G * var_u * G.T

    # mag_I/mag_B/wind/terrain/零偏等在这里按常值模型复制，相关误差块仍参与矩阵传播。
    # 它们的额外随机游走方差由 covariance.cpp 在调用生成函数后加入，不能在此重复添加。
    # Generate the equations for the upper triangular matrix and the diagonal only
    # Since the matrix is symmetric, the lower triangle does not need to be derived
    # and can simply be copied in the implementation
    # 仅生成上三角可减少公式数量与计算量；这里清零下三角是生成接口约定。
    # C++ predictCovariance() 随后把上三角复制到下三角，恢复完整对称 P。
    for index in range(state.tangent_dim()):
        for j in range(state.tangent_dim()):
            if index > j:
                P_new[index,j] = 0

    return P_new

def jacobian_chain_rule(expr: sf.Scalar , state: State):
    # 将对“存储状态”的导数转换成对“误差状态”的导数：
    #   H = dh/d(delta_x) = (dh/dx_storage) * (dx_storage/d(delta_x))。
    # 默认维度：(1 x 25) * (25 x 24) = (1 x 24)。expr 必须是一个标量观测分量。
    # 对四元数直接求出的 4 个偏导，不能直接用作三维姿态误差的 H。
    # First compute the jacobian in the parameter space
    dh_dx = sf.V1(expr).jacobian(state, tangent_space=False)

    # B = dx_storage/d(delta_x)：姿态块为 4 x 3，其余加性状态块为单位阵。
    class MStorageTangent(sf.Matrix):
        SHAPE = (State.storage_dim(), State.tangent_dim())

    # Then compute the jarobian mapping infinitesimal elements of the parameter space to the error state
    # Note that this jacobian only depends on the structure of the EKF
    dx_derror = MStorageTangent()
    q = sf.Quaternion.from_storage(state["quat_nominal"].to_storage())
    # p 是辅助扰动四元数，不是位置状态。令 p ~= [xyz=delta_theta/2, w=1]。
    p = sf.Quaternion.symbolic('p')

    # pq = p*q：对左侧变量 p 求导，得到“右乘固定 q”的四元数乘积矩阵 R(q)。
    # SymForce 存储为 [x,y,z,w]，所以 dp/dtheta 的前三行是 I/2，最后一行是零。
    # dp.w = 1 在零误差处对小角度的一阶导数为零，因此没有第四个独立姿态误差。
    pq = p * q
    qR = sf.M41(pq.to_storage()).jacobian(sf.M41(p.to_storage())) # Right quaternion product matrix
    dx_derror[0:4, 0:3] = qR / 2 * sf.M43([[1, 0, 0],
                                           [0, 1, 0],
                                           [0, 0, 1],
                                           [0, 0, 0]])

    # The rest of the matrix is trivial
    # 一个四维四元数替代三维姿态误差，故后续存储索引 i 对应误差索引 j=i-1。
    for i in range(4, State.storage_dim()):
        for j in range(3, State.tangent_dim()):
            if (i == j+1):
                dx_derror[i, j] = 1

    # Finally use the chain rule: dh/derror = dh/dx * dx/derror
    # 返回行向量 H；调用方如需 PX4 VectorState 列向量，会显式返回 H.T。
    H = dh_dx * dx_derror
    return H

def compute_airspeed_innov_and_innov_var(
        state: VState,
        P: MTangent,
        airspeed: sf.Scalar,
        R: sf.Scalar,
        epsilon: sf.Scalar
) -> (sf.Scalar, sf.Scalar):

    # 真空速观测模型：TAS_pred = |v_ground^n - [wind_N,wind_E,0]|。
    # 假设不估计垂直风，但地速仍保留垂直分量；风向量方向是“吹向哪里”。
    # airspeed 单位 m/s，R 单位 (m/s)^2；实际返回标量创新与标量创新方差。
    state = vstate_to_state(state)
    wind = sf.V3(state["wind_vel"][0], state["wind_vel"][1], 0.0)
    vel_rel = state["vel"] - wind
    airspeed_pred = vel_rel.norm(epsilon=epsilon)

    # 空速只约束相对气流速度的模长，单独一个标量不能完整确定三维地速和风。
    # epsilon 正则化相对气流接近零时的范数/导数；不改变 R 的物理含义。
    innov = airspeed_pred - airspeed

    H = jacobian_chain_rule(airspeed_pred, state)
    innov_var = (H * P * H.T + R)[0,0]

    return (innov, innov_var)

def compute_airspeed_h(
        state: VState,
        epsilon: sf.Scalar
) -> (VTangent, VTangent):

    # 为真正执行更新的 C++ 单独生成 H，避免不需要创新时重复计算全部输出。
    # 远离零速时：dh/dv = vel_rel.T/|vel_rel|，dh/dwind_NE = -dh/dv_NE。
    # 模型直接依赖速度和风，姿态没有显式项；其他状态仍可通过互协方差获得修正。
    # 实际返回一个 N x 1 的 H.T，而不是两个观测量。
    state = vstate_to_state(state)
    wind = sf.V3(state["wind_vel"][0], state["wind_vel"][1], 0.0)
    vel_rel = state["vel"] - wind
    airspeed_pred = vel_rel.norm(epsilon=epsilon)
    H = jacobian_chain_rule(airspeed_pred, state)

    return H.T

def compute_wind_init_and_cov_from_airspeed(
        v_local: sf.V3,
        heading: sf.Scalar,
        airspeed: sf.Scalar,
        v_var: sf.V3,
        heading_var: sf.Scalar,
        sideslip_var: sf.Scalar,
        airspeed_var: sf.Scalar,
) -> (sf.V2, sf.M22):

    # 假设水平飞行，用地速减去机头/侧滑方向上的空速，得到初始水平风：
    #   wind_N = v_N - TAS*cos(heading + beta)
    #   wind_E = v_E - TAS*sin(heading + beta)
    # heading/beta 为 rad；这里用符号 beta 推导误差传播，最后才令 beta=0。
    # Initialise wind states assuming horizontal flight
    sideslip = sf.Symbol("beta")
    wind = sf.V2(v_local[0] - airspeed * sf.cos(heading + sideslip), v_local[1] - airspeed * sf.sin(heading + sideslip))
    # J 是风对 [v_N,v_E,heading,beta,TAS] 的 2 x 5 雅可比。
    # 这是初始化误差传播 J*R*J.T，不是主 EKF 的 1 x N 观测雅可比。
    J = wind.jacobian([v_local[0], v_local[1], heading, sideslip, airspeed])

    # 五个输入误差暂按互不相关处理；参数 *_var 全部是方差。
    R = sf.M55()
    R[0,0] = v_var[0]
    R[1,1] = v_var[1]
    R[2,2] = heading_var
    R[3,3] = sideslip_var
    R[4,4] = airspeed_var

    P = J * R * J.T

    # Assume zero sideslip
    # 零侧滑是均值假设，不是把侧滑不确定度设为零：sideslip_var 对 P 的贡献仍保留。
    # 若在求导之前直接删除 beta，就会漏掉这部分初始风估计不确定度。
    P = P.subs({sideslip: 0.0})
    wind = wind.subs({sideslip: 0.0})
    return (wind, P)

def compute_wind_init_and_cov_from_wind_speed_and_direction(
        wind_speed: sf.Scalar,
        wind_direction: sf.Scalar,
        wind_speed_var: sf.Scalar,
        wind_direction_var: sf.Scalar
)-> (sf.V2, sf.V2):
    # 极坐标风观测转换成 N/E 分量：wind = [speed*cos(direction), speed*sin(direction)]。
    # direction 是导航系“吹向”的方位角，北为 0、向东转为正；气象“来向”需由 C++ 先转换。
    # 两输入独立时，2 x 2 雅可比将风速/风向方差传播到风向量协方差。
    wind = sf.V2(wind_speed * sf.cos(wind_direction), wind_speed * sf.sin(wind_direction))
    H = wind.jacobian([wind_speed, wind_direction])
    R = sf.Matrix.diag([wind_speed_var, wind_direction_var])

    P = H * R * H.T
    # 此接口只返回两个对角方差；变换本来可能产生非零 N/E 相关项，但这里没有输出它。
    P_diag = sf.V2(P[0,0], P[1,1])
    return (wind, P_diag)

def predict_sideslip(
        state: State,
        epsilon: sf.Scalar
) -> (sf.Scalar):

    # 先由地速减风得到导航系相对气流，再旋转到机体系。
    # 本模型的侧滑角定义为 beta_pred = atan2(v_air_body_y, v_air_body_x)。
    # 它表示水平机体平面内气流相对前向的偏角，不是地速航迹与机头方向直接相减。
    # 忽略 epsilon 时，令机体系相对气流为 u，有 d(beta)/du=[-u_y,u_x,0]/(u_x^2+u_y^2)。
    # 水平相对气流接近零时方向敏感度增大，数值保护不能代替飞行条件和输入有效性检查。
    wind = sf.V3(state["wind_vel"][0], state["wind_vel"][1], 0.0)
    vel_rel = state["vel"] - wind
    relative_wind_body = state["quat_nominal"].inverse() * vel_rel

    sideslip_pred = sf.atan2(relative_wind_body[1], relative_wind_body[0], epsilon)

    return sideslip_pred

def compute_sideslip_innov_and_innov_var(
        state: VState,
        P: MTangent,
        R: sf.Scalar,
        epsilon: sf.Scalar
) -> (sf.Scalar, sf.Scalar, sf.Scalar):

    # 合成零侧滑观测：z_beta=0，故创新就是 beta_pred，R 表示这一运动假设的不确定度。
    # 零侧滑需要相应飞行条件成立；启停/机型限制由 sideslip_fusion.cpp 负责。
    # 实际输出两个标量：innovation 与 innovation_variance，单位分别 rad 与 rad^2。
    state = vstate_to_state(state)
    sideslip_pred = predict_sideslip(state, epsilon);

    innov = sideslip_pred - 0.0

    H = jacobian_chain_rule(sideslip_pred, state)
    innov_var = (H * P * H.T + R)[0,0]

    return (innov, innov_var)

def compute_sideslip_h(
        state: VState,
        epsilon: sf.Scalar
) -> (VTangent, VTangent):

    # 生成侧滑对误差状态的 H.T，涉及姿态、地速与水平风；C++ 再计算 K=P*H.T/S。
    # 实际返回一个 N 维列向量，与上面的创新计算函数分开供运行时按需调用。
    state = vstate_to_state(state)
    sideslip_pred = predict_sideslip(state, epsilon);

    H = jacobian_chain_rule(sideslip_pred, state)

    return (H.T)

def predict_vel_body(
        state: VState
) -> (sf.V3):
    # 机体系速度观测模型：v_body = C_n^b * v_NED，单位 m/s。
    # 这是相对地面的机体系速度，不减风；可用于机体系表达的外部视觉速度观测。
    # 传感器参考点与 IMU 的杆臂等修正由对应 C++ 入口/融合代码处理。
    # 在本文左乘姿态误差约定下，一阶变化为
    #   delta(v_body) = C.T*[v_n]_x*delta_theta + C.T*delta_v。
    # 所以三轴 H 的姿态块为 C.T*[v_n]_x，速度块为 C.T；标量 H_i 取相应一行。
    vel = state["vel"]
    R_to_body = state["quat_nominal"].inverse()
    return R_to_body * vel

def compute_body_vel_innov_var_h(
        state: VState,
        P: MTangent,
        R: sf.V3,
) -> (sf.V3, VTangent, VTangent, VTangent):
    # 三轴速度分别形成标量观测：H_i = d(v_body_i)/d(delta_x)，S_i=H_i*P*H_i.T+R_i。
    # R 为三轴观测方差向量，假设观测噪声在该输入坐标系中按对角形式表示。
    # 输出是三个创新方差和三个 H.T，不是完整的 3 x 3 创新协方差矩阵。
    # 即使速度本身精确，姿态不确定度也会影响机体系速度的预测，所以 H 包含姿态块。
    state = vstate_to_state(state)
    meas_pred = predict_vel_body(state)
    Hx = jacobian_chain_rule(meas_pred[0], state)
    Hy = jacobian_chain_rule(meas_pred[1], state)
    Hz = jacobian_chain_rule(meas_pred[2], state)
    innov_var = sf.V3((Hx * P * Hx.T + R[0])[0,0],
                        (Hy * P * Hy.T + R[1])[0,0],
                        (Hz * P * Hz.T + R[2])[0,0])

    return (innov_var, Hx.T, Hy.T, Hz.T)

def compute_body_vel_y_innov_var(
        state: VState,
        P: MTangent,
        R: sf.Scalar
) -> (sf.Scalar):
    # C++ 按 X/Y/Z 顺序融合；前一轴更新 P 后，本函数用新 P 重算 Y 轴 S。
    # 线性化状态由调用方传入，外部视觉融合当前会保留本轮初始线性化点。
    # 这里只输出 S_y；H_y 可由先前一起计算的雅可比复用。
    state = vstate_to_state(state)
    meas_pred = predict_vel_body(state)
    Hy = jacobian_chain_rule(meas_pred[1], state)
    innov_var = (Hy * P * Hy.T + R)[0,0]

    return (innov_var)

def compute_body_vel_z_innov_var(
        state: VState,
        P: MTangent,
        R: sf.Scalar
) -> (sf.Scalar):
    # 与 Y 轴同理，重算 Z 轴创新方差以考虑前面标量更新对 P 的影响。
    state = vstate_to_state(state)
    meas_pred = predict_vel_body(state)
    Hz = jacobian_chain_rule(meas_pred[2], state)
    innov_var = (Hz * P * Hz.T + R)[0,0]

    return (innov_var)

def predict_mag_body(state) -> sf.V3:
    # 磁力计观测模型：m_pred^b = C_n^b(q)*m_I^n + m_B^b，单位 Gauss。
    # m_I 是导航系地磁，m_B 是机体系附加磁场/硬铁类偏置；二者处于不同坐标系。
    # 机体旋转时地磁在机体系中的投影改变，而固定机体偏置不随姿态旋转。
    # 模型对姿态和两类磁状态均敏感，这也是磁观测能约束航向与学习磁偏置的原因。
    # 一阶变化为 delta(m_body) = C.T*[m_I]_x*delta_theta + C.T*delta_mI + delta_mB，
    # 对应三块雅可比分别为 C.T*[m_I]_x、C.T 和 I，与机体系速度的推导形式相似。
    mag_field_earth = state["mag_I"]
    mag_bias_body = state["mag_B"]

    mag_body = state["quat_nominal"].inverse() * mag_field_earth + mag_bias_body
    return mag_body

def compute_mag_innov_innov_var_and_hx(
        state: VState,
        P: MTangent,
        meas: sf.V3,
        R: sf.Scalar,
        epsilon: sf.Scalar
) -> (sf.V3, sf.V3, VTangent):

    # 一次计算三轴创新 r=m_pred-m_meas、三轴 S，以及先融合的 X 轴 H.T。
    # R 为每轴共用的观测噪声方差 Gauss^2；输出 S 仅保存各轴对角项。
    # 后续 Y/Z 轴会在前一轴更新 P 后调用各自函数重新计算，避免直接复用旧 S。
    state = vstate_to_state(state)
    meas_pred = predict_mag_body(state);

    innov = meas_pred - meas

    innov_var = sf.V3()
    Hx = jacobian_chain_rule(meas_pred[0], state)
    innov_var[0] = (Hx * P * Hx.T + R)[0,0]
    Hy = jacobian_chain_rule(meas_pred[1], state)
    innov_var[1] = (Hy * P * Hy.T + R)[0,0]
    Hz = jacobian_chain_rule(meas_pred[2], state)
    innov_var[2] = (Hz * P * Hz.T + R)[0,0]

    return (innov, innov_var, Hx.T)

def compute_mag_y_innov_var_and_h(
        state: VState,
        P: MTangent,
        R: sf.Scalar,
        epsilon: sf.Scalar
) -> (sf.Scalar, VTangent):

    # 生成磁场 Y 轴的标量 S 与 H.T；状态线性化点和最新 P 都由调用者提供。
    # 本函数只计算更新所需公式，是否拒绝该观测由 mag_fusion.cpp 处理。
    state = vstate_to_state(state)
    meas_pred = predict_mag_body(state);

    H = jacobian_chain_rule(meas_pred[1], state)
    innov_var = (H * P * H.T + R)[0,0]

    return (innov_var, H.T)

def compute_mag_z_innov_var_and_h(
        state: VState,
        P: MTangent,
        R: sf.Scalar,
        epsilon: sf.Scalar
) -> (sf.Scalar, VTangent):

    # 磁场 Z 轴顺序更新所需的 S 与 H.T，计算方式与 Y 轴对应。
    state = vstate_to_state(state)
    meas_pred = predict_mag_body(state);

    H = jacobian_chain_rule(meas_pred[2], state)
    innov_var = (H * P * H.T + R)[0,0]

    return (innov_var, H.T)

def compute_yaw_innov_var_and_h(
        state: VState,
        P: MTangent,
        R: sf.Scalar
) -> (sf.Scalar, VTangent):

    # 采用“小航向创新”的四元数误差模型，而不是直接对完整 Euler atan2 航向函数求导。
    # q 是估计姿态，r 是用于推导的参考/测量姿态四元数；这里的 r 不是创新变量。
    # delta_q = q*inverse(r) 在导航系表达，2*delta_q.z 是小旋转的导航系 Z 分量。
    # 该函数只生成 H 和 S，不输出实际航向观测或创新；实际角度差与 wrap_pi 在 C++ 中处理。
    state = vstate_to_state(state)
    q = sf.Quaternion.from_storage(state["quat_nominal"].to_storage())
    r = sf.Quaternion.symbolic('r')
    delta_q = q * r.conj() # create a quaternion error of the measurement at the origin
    delta_meas_pred = 2 * delta_q.z # Use small angle approximation to obtain a simpler jacobian

    H = jacobian_chain_rule(delta_meas_pred, state)
    # 先对估计姿态求导，保持参考 r 独立；随后令 r=q，在零姿态差处取得线性化雅可比。
    # 若求导前就令二者相同，delta_q 会直接化为单位旋转，从而丢失所需导数。
    H = H.subs({r.w: q.w, r.x: q.x, r.y: q.y, r.z: q.z}) # assume innovation is small

    for i in range(State.tangent_dim()):
        # 使用单位四元数约束进一步消去冗余项；不能将结果误读为任意大创新下的精确模型。
        H[i] = sp.factor(H[i]).subs(q.w**2 + q.x**2 + q.y**2 + q.z**2, 1) # unit norm quaternion
    innov_var = (H * P * H.T + R)[0,0]

    return (innov_var, H.T)

def compute_mag_declination_pred_innov_var_and_h(
        state: VState,
        P: MTangent,
        R: sf.Scalar,
        epsilon: sf.Scalar
) -> (sf.Scalar, sf.Scalar, VTangent):

    # 磁偏角模型：declination = atan2(m_I_E, m_I_N)，北为 0、偏东为正，单位 rad。
    # 只直接依赖导航系水平地磁分量，可用于给地磁状态施加方向参考。
    # 当水平地磁接近零时方向本身缺少物理可定义性；epsilon 仅处理数值正则化。
    state = vstate_to_state(state)
    meas_pred = sf.atan2(state["mag_I"][1], state["mag_I"][0], epsilon=epsilon)

    H = jacobian_chain_rule(meas_pred, state)
    innov_var = (H * P * H.T + R)[0,0]

    return (meas_pred, innov_var, H.T)

def predict_hagl(state):
    # HAGL = Height Above Ground Level，离地高度由同一垂直基准的两个向下坐标相减：
    #   hagl = terrain_D - vehicle_D。
    # 例如飞行器 D=-10 m、地面 D=0 m，则离地高度为 +10 m；不能把符号次序反过来。
    # terrain 是一维向量所以取 [0]，pos 是三维向量所以取向下分量 [2]。
    return state["terrain"][0] - state["pos"][2]

def predict_opt_flow(state, epsilon):
    # 模型输入是已按约定表达的光流角速率，单位 rad/s；这里预测平移造成的视线角速率。
    # 下视、局部平面几何模型使用机体系/传感器约定；旋转光流补偿和杆臂处理见 C++。
    R_to_body = state["quat_nominal"].inverse()

    # Calculate earth relative velocity in a non-rotating sensor frame
    # v_b = C_n^b*v_n 是相对地面的平移速度，光流观察地面，故这里不减风速。
    rel_vel_sensor = R_to_body * state["vel"]

    # Divide by range to get predicted angular LOS rates relative to X and Y
    # axes. Note these are rates in a non-rotating sensor frame
    # 平移视线角速率约等于横向速度 / 沿视线距离：绕 X 的光流由 v_body_y 产生，
    # 绕 Y 的光流由 -v_body_x 产生，符号来自 EKF 的图像旋转约定。
    hagl = predict_hagl(state)
    # 给高度增加保符号的小量，避免除零；再用 Abs(hagl) 保持距离量为非负。
    # 该保护让生成式在边界附近可计算，但不能让负高度或零高度变成物理可信观测。
    hagl = add_epsilon_sign(hagl, hagl, epsilon)
    R_to_earth = state["quat_nominal"].to_rotation_matrix()
    flow_pred = sf.V2()
    # C[2,2] 是机体 Z 轴在导航系向下轴上的投影。在下视有效几何范围内，
    # 斜距 range ~= |hagl|/C[2,2]，因此 1/range ~= C[2,2]/|hagl|。
    # 水平姿态 C[2,2]=1 时，退化为 [v_body_y/h, -v_body_x/h]。
    # H 因而会同时涉及姿态、速度、垂直位置和地形，而非只包含水平速度。
    flow_pred[0] =  rel_vel_sensor[1] / sf.Abs(hagl) * R_to_earth[2, 2]
    flow_pred[1] = -rel_vel_sensor[0] / sf.Abs(hagl) * R_to_earth[2, 2]

    return flow_pred

def compute_flow_xy_innov_var_and_hx(
        state: VState,
        P: MTangent,
        R: sf.Scalar,
        epsilon: sf.Scalar
) -> (sf.V2, VTangent):
    # 生成两轴 S 和第一轴 H.T；两轴噪声暂共用同一个标量 R，单位 (rad/s)^2。
    # C++ 的 optical_flow_control.cpp 用这些量进行初始创新检验，实际创新在 C++ 中计算。
    # 光流只给出“速度/距离”的比例，距离/地形及其他运动约束会影响速度可观测性。
    state = vstate_to_state(state)
    meas_pred = predict_opt_flow(state, epsilon)

    innov_var = sf.V2()
    Hx = jacobian_chain_rule(meas_pred[0], state)
    innov_var[0] = (Hx * P * Hx.T + R)[0,0]
    Hy = jacobian_chain_rule(meas_pred[1], state)
    innov_var[1] = (Hy * P * Hy.T + R)[0,0]

    return (innov_var, Hx.T)

def compute_flow_y_innov_var_and_h(
        state: VState,
        P: MTangent,
        R: sf.Scalar,
        epsilon: sf.Scalar
) -> (sf.Scalar, VTangent):
    # 第一轴融合改变 P 后重算第二轴 S 和 H.T；是否沿用初始线性化状态由调用者决定。
    # 当前光流顺序融合保留该轮初始 state_vector，创新则会结合更新后的状态重算。
    state = vstate_to_state(state)
    meas_pred = predict_opt_flow(state, epsilon)

    Hy = jacobian_chain_rule(meas_pred[1], state)
    innov_var = (Hy * P * Hy.T + R)[0,0]

    return (innov_var, Hy.T)

def compute_hagl_innov_var(
        P: MTangent,
        R: sf.Scalar,
) -> (sf.Scalar):
    # 离地高度 h=terrain_D-pos_D 是线性观测，H 仅两项非零：H_posD=-1、H_terrain=+1。
    # S = P_DD + P_tt - P_Dt - P_tD + R，t 表示地形误差索引。
    # 若位置与地形相关，不能只把两项方差相加而忽略交叉协方差。
    # H 与当前名义状态数值无关，故函数最终只需 P 和 R 输入。
    state = VState.symbolic("state")
    state = vstate_to_state(state)
    meas_pred = predict_hagl(state)

    H = jacobian_chain_rule(meas_pred, state)
    innov_var = (H * P * H.T + R)[0,0]

    return (innov_var)

def compute_hagl_h(
) -> (VTangent):
    # 生成常量 H.T，供 range_height_fusion.cpp 用 K=P*H.T/S 融合垂直距离观测。
    # 因为 H 不依赖状态值，这个生成函数不需要任何运行时输入参数。
    # 是否允许修正飞行器高度、地形或两者，由 C++ 对 K 的相关分量进行选择。
    state = VState.symbolic("state")
    state = vstate_to_state(state)
    meas_pred = predict_hagl(state)

    H = jacobian_chain_rule(meas_pred, state)

    return (H.T)

def compute_gnss_yaw_pred_innov_var_and_h(
        state: VState,
        P: MTangent,
        antenna_yaw_offset: sf.Scalar,
        R: sf.Scalar,
        epsilon: sf.Scalar
) -> (sf.Scalar, sf.Scalar, VTangent):

    # 双天线 GNSS 测量的是天线基线在水平面的方位，不一定等于机体前向轴方位。
    # antenna_yaw_offset 为基线相对机头的机体系平面安装角，单位 rad。
    # 用实际姿态旋转基线后再投影，可考虑俯仰/横滚对非前向安装基线的影响。
    state = vstate_to_state(state)
    R_to_earth = state["quat_nominal"]

    # define antenna vector in body frame
    # 单位基线 a_b=[cos(offset),sin(offset),0]，本模型假设基线位于机体 XY 平面。
    # 基线长度不影响预测方位角，因此不需要输入物理长度。
    ant_vec_bf = sf.V3(sf.cos(antenna_yaw_offset), sf.sin(antenna_yaw_offset), 0)

    # rotate into earth frame
    ant_vec_ef = R_to_earth * ant_vec_bf

    # Calculate the yaw angle from the projection
    # a_n=C_b^n*a_b，heading_pred=atan2(a_n_E,a_n_N)。
    # 若基线几乎竖直，水平投影接近零，航向在物理上接近不可定义；C++ 还需检查几何条件。
    # epsilon 提供 atan2 的数值保护，实际角度创新与 [-pi,pi] 环绕由调用方处理。
    meas_pred = sf.atan2(ant_vec_ef[1], ant_vec_ef[0], epsilon=epsilon)

    H = jacobian_chain_rule(meas_pred, state)
    innov_var = (H * P * H.T + R)[0,0]

    return (meas_pred, innov_var, H.T)

def predict_drag(
        state: State,
        rho: sf.Scalar,
        cd: sf.Scalar,
        cm: sf.Scalar,
        epsilon: sf.Scalar
) -> (sf.Scalar):
    # 多旋翼机体水平阻力比力模型，实际输出二维向量 [a_drag_x,a_drag_y]，单位 m/s^2。
    # u = C_n^b * (v_NED - [wind_N,wind_E,0]) 是机体相对空气的速度。
    # 模型含机体二次阻力和旋翼动量线性阻力：
    #   a_drag_xy = -0.5*rho*cd*|u|*u_xy - cm*u_xy。
    # rho 单位 kg/m^3；这里的 cd 是运行时传入的逆弹道系数，单位 m^2/kg，
    # 不能直接等同于空气动力学中无量纲的阻力系数 C_D；cm 单位 1/s。
    # 负号表示阻力与相对气流速度相反，二次项使用完整三维 |u| 而不是仅 |u_xy|。
    R_to_body = state["quat_nominal"].inverse()

    wind = sf.V3(state["wind_vel"][0], state["wind_vel"][1], 0.0)
    vel_rel = state["vel"] - wind
    vel_rel_body = R_to_body * vel_rel
    vel_rel_body_xy = sf.V2(vel_rel_body[0], vel_rel_body[1])

    bluff_body_drag = -0.5 * rho * cd * vel_rel_body_xy * vel_rel_body.norm(epsilon=epsilon)
    momentum_drag = -cm * vel_rel_body_xy

    return bluff_body_drag + momentum_drag


def compute_drag_x_innov_var_and_h(
        state: VState,
        P: MTangent,
        rho: sf.Scalar,
        cd: sf.Scalar,
        cm: sf.Scalar,
        R: sf.Scalar,
        epsilon: sf.Scalar
) -> (sf.Scalar, VTangent):

    # 对阻力模型 X 分量求 H，并计算标量创新方差；R 单位 (m/s^2)^2。
    # C++ 用预处理后的水平比力与预测阻力比较；模型系数选择/修正和融合条件也在 C++ 中。
    # 轴向 cd 可以由调用者分别给定，不能假定 X/Y 机体阻力系数相同。
    state = vstate_to_state(state)
    meas_pred = predict_drag(state, rho, cd, cm, epsilon)
    Hx = jacobian_chain_rule(meas_pred[0], state)
    innov_var = (Hx * P * Hx.T + R)[0,0]

    return (innov_var, Hx.T)

def compute_drag_y_innov_var_and_h(
        state: VState,
        P: MTangent,
        rho: sf.Scalar,
        cd: sf.Scalar,
        cm: sf.Scalar,
        R: sf.Scalar,
        epsilon: sf.Scalar
) -> (sf.Scalar, VTangent):

    # 与 X 轴对应的 Y 分量公式；使用调用方传入的状态线性化点和更新后的 P。
    # 阻力观测通过相对气流约束风与相关状态，模型可靠性取决于系数和运动条件。
    state = vstate_to_state(state)
    meas_pred = predict_drag(state, rho, cd, cm, epsilon)
    Hy = jacobian_chain_rule(meas_pred[1], state)
    innov_var = (Hy * P * Hy.T + R)[0,0]

    return (innov_var, Hy.T)

def predict_gravity_direction(state: State):
    # 静止或平动加速度足够小时，加速度计归一化比力近似指向“上”，而非重力向下。
    # 因此导航系参考为 [0,0,-1]，预测单位比力方向 h=C_n^b*[0,0,-1]。
    # 这是无量纲方向观测，不是以 m/s^2 为单位的完整重力加速度模型。
    # 重力方向只有两个独立姿态约束，不能单独提供绝对航向。
    # get transform from earth to body frame
    R_to_body = state["quat_nominal"].inverse()

    # the innovation is the error between measured acceleration
    #  and predicted (body frame), assuming no body acceleration
    return R_to_body * sf.Matrix([0,0,-1])

def compute_gravity_xyz_innov_var_and_hx(
        state: VState,
        P: MTangent,
        R: sf.Scalar
) -> (sf.V3, VTangent):

    # 三个方向分量按标量顺序更新准备 S；R 是归一化方向分量的观测噪声方差。
    # 实际输出三轴 S 和第一轴 H.T，既没有输出 K，也没有在这里执行融合。
    # C++ 还会选择合适运动条件，避免将真实平动加速度误当成姿态倾斜。
    # 若用一个三维单位方向作为观测，其各分量误差受单位范数约束；这里采用的逐轴标量
    # 噪声模型是一种实现近似，不能据“分三轴更新”推断绝对航向因此变得可观。
    state = vstate_to_state(state)
    meas_pred = predict_gravity_direction(state)

    # initialize outputs
    innov_var = sf.V3()
    H = [None] * 3

    # calculate observation jacobian (H), kalman gain (K), and innovation variance (S)
    #  for each axis
    # 即使有三个分量，也不能把单位方向理解为三个独立绝对姿态观测；其几何约束秩为 2。
    for i in range(3):
        H[i] = jacobian_chain_rule(meas_pred[i], state)
        innov_var[i] = (H[i] * P * H[i].T + R)[0,0]

    return (innov_var, H[0].T)

def compute_gravity_y_innov_var_and_h(
        state: VState,
        P: MTangent,
        R: sf.Scalar
) -> (sf.V3, VTangent, VTangent, VTangent):

    # 顺序更新中的 Y 轴；实际返回标量 S_y 与 N 维 H_y.T。
    # 初始线性化状态可固定，但 P 已因前一轴融合而改变，因此 S 要重新计算。
    state = vstate_to_state(state)
    meas_pred = predict_gravity_direction(state)

    # calculate observation jacobian (H), kalman gain (K), and innovation variance (S)
    H = jacobian_chain_rule(meas_pred[1], state)
    innov_var = (H * P * H.T + R)[0,0]

    return (innov_var, H.T)

def compute_gravity_z_innov_var_and_h(
        state: VState,
        P: MTangent,
        R: sf.Scalar
) -> (sf.V3, VTangent, VTangent, VTangent):

    # 顺序更新中的 Z 轴；实际返回标量 S_z 与 N 维 H_z.T。
    # 这里的函数参数/返回量与 C++ gravity_fusion.cpp 的生成函数调用逐一对应。
    state = vstate_to_state(state)
    meas_pred = predict_gravity_direction(state)

    # calculate observation jacobian (H), kalman gain (K), and innovation variance (S)
    H = jacobian_chain_rule(meas_pred[2], state)
    innov_var = (H * P * H.T + R)[0,0]

    return (innov_var, H.T)

# 下面才真正触发推导与代码生成，前面的 def 主要用于描述符号公式。
# generate_px4_function() 由工具模块封装 SymForce Codegen：以符号输入执行函数，
# 简化表达式并输出 C++，随后将矩阵与数学接口替换成 PX4 可用的版本。
# 输出目录是进程当前工作目录下的 generated/；CMake 会为源码/构建目录设置工作位置。
# 生成时会写对应头文件，单纯学习公式不需要重新运行这里的生成过程。
# output_names 给输出命名；Python snake_case 会对应生成头文件中的 C++ 函数名，
# 例如 predict_covariance -> predict_covariance.h 中的 sym::PredictCovariance()。
print("Derive EKF2 equations...")
generate_px4_function(predict_covariance, output_names=None)

if not args.disable_mag:
    # 只有启用磁状态时才生成地磁向量和磁偏角观测公式。
    generate_px4_function(compute_mag_declination_pred_innov_var_and_h, output_names=["pred", "innov_var", "H"])
    generate_px4_function(compute_mag_innov_innov_var_and_hx, output_names=["innov", "innov_var", "Hx"])
    generate_px4_function(compute_mag_y_innov_var_and_h, output_names=["innov_var", "H"])
    generate_px4_function(compute_mag_z_innov_var_and_h, output_names=["innov_var", "H"])

if not args.disable_wind:
    # 空速、侧滑和阻力模型都使用风状态；风初始化另生成输入误差传播公式。
    generate_px4_function(compute_airspeed_h, output_names=None)
    generate_px4_function(compute_airspeed_innov_and_innov_var, output_names=["innov", "innov_var"])
    generate_px4_function(compute_drag_x_innov_var_and_h, output_names=["innov_var", "Hx"])
    generate_px4_function(compute_drag_y_innov_var_and_h, output_names=["innov_var", "Hy"])
    generate_px4_function(compute_sideslip_h, output_names=None)
    generate_px4_function(compute_sideslip_innov_and_innov_var, output_names=["innov", "innov_var"])
    generate_px4_function(compute_wind_init_and_cov_from_airspeed, output_names=["wind", "P_wind"])
    generate_px4_function(compute_wind_init_and_cov_from_wind_speed_and_direction, output_names=["wind", "P_wind"])

# 以下模型不依赖磁场或风状态，因此在两类可选状态裁减后仍会生成。
# 航向误差模型与双天线 GNSS 航向模型分别生成，二者的观测几何定义不同。
generate_px4_function(compute_yaw_innov_var_and_h, output_names=["innov_var", "H"])
generate_px4_function(compute_flow_xy_innov_var_and_hx, output_names=["innov_var", "H"])
generate_px4_function(compute_flow_y_innov_var_and_h, output_names=["innov_var", "H"])
generate_px4_function(compute_hagl_innov_var, output_names=["innov_var"])
generate_px4_function(compute_hagl_h, output_names=["H"])
generate_px4_function(compute_gnss_yaw_pred_innov_var_and_h, output_names=["meas_pred", "innov_var", "H"])
generate_px4_function(compute_gravity_xyz_innov_var_and_hx, output_names=["innov_var", "Hx"])
generate_px4_function(compute_gravity_y_innov_var_and_h, output_names=["innov_var", "Hy"])
generate_px4_function(compute_gravity_z_innov_var_and_h, output_names=["innov_var", "Hz"])
generate_px4_function(compute_body_vel_innov_var_h, output_names=["innov_var", "Hx", "Hy", "Hz"])
generate_px4_function(compute_body_vel_y_innov_var, output_names=["innov_var"])
generate_px4_function(compute_body_vel_z_innov_var, output_names=["innov_var"])

# 最后输出 StateSample、存储向量接口和 State::{idx,dof,size} 等误差索引到 state.h。
# 因而状态定义、协方差/增益维数和生成公式必须使用同一配置，不能混用不同配置的产物。
generate_px4_state(State, tangent_idx)
