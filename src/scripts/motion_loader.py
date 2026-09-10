# Copyright (c) 2024-2025 zh

import torch
import numpy as np
import os
from pathlib import Path
from src.utils.math import (
    quat_conjugate,
    quat_from_axis_angle,
    quat_multiply,
    quat_normalize,
    quat_to_rotation_matrix,
    quat_yaw_only,
    slerp,
)


class MotionLoader:
    """Load CSV motion data and provide time-interpolated joint references."""

    def __init__(self, policy_dir: str, motion_params: dict, device: str = "cpu"):
        self.motion_file = os.path.join(policy_dir, motion_params["motion_file"])
        self.fps = float(motion_params["fps"])
        self.dt = 1.0 / self.fps
        self.time_start = float(motion_params["time_start"])
        self.time_end = float(motion_params["time_end"])
        self.device = torch.device(device)

        self.index_0_ = 0
        self.index_1_ = 0
        self.blend_ = 0.0
        self.init_quat = torch.tensor([[1.0, 0.0, 0.0, 0.0]], dtype=torch.float32, device=self.device)

        self._load_motion()

        # print(
        #     f"[MotionLoader] {Path(self.motion_file).name} | "
        #     f"frames={self.num_frames}, joints={self.num_joints}, "
        #     f"fps={self.fps:.1f}, duration={self.duration:.2f}s, "
        #     f"time={self.time_start:.2f}-{self.time_start + self.duration:.2f}s"
        # )

    def _load_motion(self):
        # 加载csv文件
        motion = np.loadtxt(self.motion_file, delimiter=",", dtype=np.float32)
        motion = torch.as_tensor(motion, device=self.device)

        # 参考数据的总时间
        total_frames = int(motion.shape[0])
        motion_duration = total_frames * self.dt

        if self.time_start < 0.0:
            raise ValueError(f"Invalid time_start: {self.time_start}")
        if self.time_end <= self.time_start:
            raise ValueError(f"Invalid time_end: {self.time_end}")
        if self.time_start >= motion_duration:
            raise ValueError(f"time_start exceeds motion duration: {self.time_start:.3f}s")

        # 裁剪参考数据
        start_frame = int(round(self.time_start * self.fps))
        end_frame = min(total_frames, int(round(self.time_end * self.fps)))

        frame_slice = slice(start_frame, end_frame)
        motion = motion[frame_slice]

        # 保存修改后的数据
        self.root_positions = motion[:, 0:3]
        self.root_quaternions = motion[:, [6, 3, 4, 5]]
        self.dof_positions = motion[:, 7:]
        self.num_frames = int(self.dof_positions.shape[0])
        self.num_joints = int(self.dof_positions.shape[1])
        self.duration = float(self.num_frames * self.dt)

        # 计算速度插值
        vel = (self.dof_positions[1:] - self.dof_positions[:-1]) / self.dt
        self.dof_velocities = torch.cat([vel, vel[-1:]], dim=0)

    def update(self, time_s: float):
        """ 更新插值 """
        phase = min(max(float(time_s) / max(self.duration, 1e-8), 0.0), 1.0)
        frame_float = phase * max(self.num_frames - 1, 0)
        self.index_0_ = int(frame_float)
        self.index_1_ = min(self.index_0_ + 1, self.num_frames - 1)
        self.blend_ = frame_float - self.index_0_

    def reset(self, robot_base_quat: torch.Tensor, time_s: float = 0.0):
        self.update(time_s)
        ref_yaw = quat_yaw_only(self.root_quaternion())
        robot_yaw = quat_yaw_only(robot_base_quat)
        self.init_quat = quat_multiply(robot_yaw, quat_conjugate(ref_yaw))

    def joint_pos(self) -> torch.Tensor:
        pos0 = self.dof_positions[self.index_0_ : self.index_0_ + 1]
        pos1 = self.dof_positions[self.index_1_ : self.index_1_ + 1]
        return pos0 * (1.0 - self.blend_) + pos1 * self.blend_

    def root_position(self) -> torch.Tensor:
        pos0 = self.root_positions[self.index_0_ : self.index_0_ + 1]
        pos1 = self.root_positions[self.index_1_ : self.index_1_ + 1]
        return pos0 * (1.0 - self.blend_) + pos1 * self.blend_

    def joint_vel(self) -> torch.Tensor:
        vel0 = self.dof_velocities[self.index_0_ : self.index_0_ + 1]
        vel1 = self.dof_velocities[self.index_1_ : self.index_1_ + 1]
        return vel0 * (1.0 - self.blend_) + vel1 * self.blend_

    def root_quaternion(self) -> torch.Tensor:
        q0 = self.root_quaternions[self.index_0_ : self.index_0_ + 1]
        q1 = self.root_quaternions[self.index_1_ : self.index_1_ + 1]
        return slerp(q0, q1, self.blend_)

    def torso_quat_w(self, root_quat: torch.Tensor, waist_angles: torch.Tensor) -> torch.Tensor:
        axes = torch.eye(3, dtype=root_quat.dtype, device=root_quat.device)
        q_yaw = quat_from_axis_angle(axes[2].expand(root_quat.shape[0], -1), waist_angles[:, 0])
        q_roll = quat_from_axis_angle(axes[0].expand(root_quat.shape[0], -1), waist_angles[:, 1])
        q_pitch = quat_from_axis_angle(axes[1].expand(root_quat.shape[0], -1), waist_angles[:, 2])
        torso_quat = quat_multiply(root_quat, q_yaw)
        torso_quat = quat_multiply(torso_quat, q_roll)
        torso_quat = quat_multiply(torso_quat, q_pitch)
        return quat_normalize(torso_quat)

    def anchor_quat_w(self) -> torch.Tensor:
        joint_pos = self.joint_pos()
        waist_angles = joint_pos[:, [12, 13, 14]]
        return self.torso_quat_w(self.root_quaternion(), waist_angles)

    def motion_anchor_ori_b(self, real_quat_w: torch.Tensor, ref_quat_w: torch.Tensor) -> torch.Tensor:
        rot_quat = quat_multiply(quat_conjugate(quat_multiply(self.init_quat, ref_quat_w)), real_quat_w)
        rot = quat_to_rotation_matrix(rot_quat).transpose(1, 2)
        return rot[:, :, :2].reshape(rot.shape[0], 6)
