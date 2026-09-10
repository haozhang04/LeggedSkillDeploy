# Copyright (c) 2024-2025 zh

import torch


# 约定：
# - 所有输入均为 batch 形式，不做自动 reshape。
# - 四元数格式统一为 [w, x, y, z]，shape 为 [B, 4]。
# - 三维向量 shape 为 [B, 3]，旋转矩阵 shape 为 [B, 3, 3]。


# 用四元数对向量做逆旋转，常用于坐标系变换
# 展开式：v' = (2w^2 - 1)v - 2w(q_vec x v) + 2q_vec(q_vec · v)
def quat_rotate_inverse(q: torch.Tensor, v: torch.Tensor) -> torch.Tensor:
    q_w = q[:, 0]
    q_vec = q[:, 1:4]
    shape = q.shape
    a = v * (2.0 * q_w ** 2 - 1.0).unsqueeze(-1)
    b = torch.cross(q_vec, v, dim=-1) * q_w.unsqueeze(-1) * 2.0
    c = q_vec * torch.bmm(q_vec.view(shape[0], 1, 3), v.view(shape[0], 3, 1)).squeeze(-1) * 2.0
    return a - b + c


# 归一化四元数，避免后续旋转计算受四元数模长误差影响
# q_hat = q / ||q||
def quat_normalize(q: torch.Tensor) -> torch.Tensor:
    return q / q.norm(dim=1, keepdim=True).clamp(min=1e-8)


# 四元数Hamilton 乘法
# q1 * q2 = [w1*w2 - dot(v1, v2), w1*v2 + w2*v1 + cross(v1, v2)]
def quat_multiply(q1: torch.Tensor, q2: torch.Tensor) -> torch.Tensor:
    w1, x1, y1, z1 = q1.unbind(dim=1)
    w2, x2, y2, z2 = q2.unbind(dim=1)
    return torch.stack(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ],
        dim=1,
    )


# 四元数共轭，用于构造逆旋转或相对旋转
# conj([w, x, y, z]) = [w, -x, -y, -z]
def quat_conjugate(q: torch.Tensor) -> torch.Tensor:
    return torch.stack([q[:, 0], -q[:, 1], -q[:, 2], -q[:, 3]], dim=1)


# 把旋转轴和旋转角转换为四元数
# q = [cos(theta/2), u * sin(theta/2)]
def quat_from_axis_angle(axis: torch.Tensor, angle: torch.Tensor) -> torch.Tensor:
    angle = angle.view(-1, 1)
    half_angle = angle * 0.5
    sin_half = torch.sin(half_angle)
    return torch.cat([torch.cos(half_angle), axis * sin_half], dim=1)


# 从完整姿态中只保留 z 轴 yaw，用于只对齐水平朝向。
# yaw = atan2(2*(w*z + x*y), 1 - 2*(y^2 + z^2))
# q_yaw = [cos(yaw/2), 0, 0, sin(yaw/2)]
def quat_yaw_only(q: torch.Tensor) -> torch.Tensor:
    siny_cosp = 2.0 * (q[:, 0] * q[:, 3] + q[:, 1] * q[:, 2])
    cosy_cosp = 1.0 - 2.0 * (q[:, 2] * q[:, 2] + q[:, 3] * q[:, 3])
    yaw = torch.atan2(siny_cosp, cosy_cosp)
    half_yaw = yaw * 0.5
    zeros = torch.zeros_like(half_yaw)
    return torch.stack([torch.cos(half_yaw), zeros, zeros, torch.sin(half_yaw)], dim=1)


# 把四元数转换为 3x3 旋转矩阵
# R = [[1-2(y^2+z^2), 2(xy-wz),     2(xz+wy)],
#      [2(xy+wz),     1-2(x^2+z^2), 2(yz-wx)],
#      [2(xz-wy),     2(yz+wx),     1-2(x^2+y^2)]]
def quat_to_rotation_matrix(q: torch.Tensor) -> torch.Tensor:
    q = quat_normalize(q)
    w, x, y, z = q.unbind(dim=1)
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z

    row0 = torch.stack([1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)], dim=1)
    row1 = torch.stack([2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)], dim=1)
    row2 = torch.stack([2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)], dim=1)
    return torch.stack([row0, row1, row2], dim=1)


# 在两个姿态四元数之间做球面线性插值，用于平滑插值 root 姿态
# theta = acos(dot(q0, q1))
# slerp(q0, q1, t) =
#     sin((1-t)*theta) / sin(theta) * q0
#     + sin(t*theta) / sin(theta) * q1
# dot < 0 时取 -q1 走较短圆弧；接近重合时退化为归一化线性插值。
def slerp(q0: torch.Tensor, q1: torch.Tensor, blend: float) -> torch.Tensor:
    dot = torch.sum(q0 * q1, dim=1, keepdim=True)
    q1_adjusted = torch.where(dot < 0.0, -q1, q1)
    dot = torch.abs(dot).clamp(-1.0, 1.0)

    close = dot > 0.9995
    lerp = quat_normalize(q0 + blend * (q1_adjusted - q0))

    theta = torch.acos(dot)
    sin_theta = torch.sin(theta).clamp(min=1e-8)
    w0 = torch.sin((1.0 - blend) * theta) / sin_theta
    w1 = torch.sin(blend * theta) / sin_theta
    slerp_q = q0 * w0 + q1_adjusted * w1
    slerp_q = quat_normalize(slerp_q)
    return torch.where(close, lerp, slerp_q)
