#  (c) 2024-2025 zh

import os
import torch
import yaml
from pathlib import Path
from src.utils.path_config import PROJECT_ROOT
from src.scripts.motion_loader import MotionLoader
from src.scripts.observation_buffer import ObservationBuffer
from src.scripts.rl_data import LOGGER, STATE, Control, ModelParams, Observations, RobotState
from src.utils.math import quat_rotate_inverse


class RLPolicy:
    """Single policy"""

    def __init__(self, policy_dir: str):
        self.policy_rel_dir = policy_dir
        self.policy_dir = os.path.join(PROJECT_ROOT, f"policy/{policy_dir}")
        path = Path(policy_dir)
        self.plat_name = path.parts[-2] # 平台名称
        self.robot_name = path.parts[-1] # 机器人名称
        self.policy_name = path.name # 策略名称

        # 加载参数
        self.params = ModelParams()
        self._load_config()
        self.rl_time = 0.0

        # 观测
        self.obs = Observations()

        self._init_observations()
        self._init_outputs()
        self._init_history_buffer()

        # 加载本策略所有模型
        self.models = []
        for model_name in self.params.model_names:
            model_path = os.path.join(self.policy_dir, model_name)
            self.models.append(torch.jit.load(model_path))

        # 加载 motion csv 文件
        self.motion_loader = None
        if self.params.motion_params:
            self.motion_loader = MotionLoader(self.policy_dir, self.params.motion_params)


    def _load_config(self):
        """ 加载配置文件 """
        config_path = os.path.join(self.policy_dir, "config.yaml")
        with open(config_path, "r", encoding="utf-8") as f:
            config = yaml.safe_load(f)[self.policy_name]

        self.params.model_names = list(config["model_names"])
        # print(f"{LOGGER.INFO}Loaded {len(self.params.model_names)} models from {self.policy_name}: {self.params.model_names}")

        self.params.dt = config["dt"]
        self.params.decimation = config["decimation"]
        self.params.num_of_dofs = config["num_of_dofs"]
        self.params.num_observations = config["num_observations"]
        self.params.observations = config["observations"]
        self.params.observations_history = config["observations_history"]
        self.params.clip_obs = config["clip_obs"]
        self.params.rl_kp = torch.tensor(config["rl_kp"]).view(1, -1)
        self.params.rl_kd = torch.tensor(config["rl_kd"]).view(1, -1)
        self.params.fixed_kp = torch.tensor(config["fixed_kp"]).view(1, -1)
        self.params.fixed_kd = torch.tensor(config["fixed_kd"]).view(1, -1)
        self.params.action_scale = torch.tensor(config["action_scale"]).view(1, -1)
        self.params.wheel_indices = config["wheel_indices"]
        self.params.ang_vel_scale = config["ang_vel_scale"]
        self.params.dof_pos_scale = config["dof_pos_scale"]
        self.params.dof_vel_scale = config["dof_vel_scale"]
        self.params.commands_scale = torch.tensor(config["commands_scale"]).view(1, -1)
        self.params.torque_limits = torch.tensor(config["torque_limits"]).view(1, -1)
        self.params.default_dof_pos = torch.tensor(config["default_dof_pos"]).view(1, -1)
        self.params.joint_controller_names = config["joint_controller_names"]
        self.params.joint_mapping = config["joint_mapping"]
        self.params.joint_mapping_tensor = torch.tensor(self.params.joint_mapping, dtype=torch.long)
        self.params.waist_joint_indices = config.get("waist_joint_indices")
        self.params.motion_params = config.get("motion_params", {})

    def _init_observations(self):
        """ 初始化观测 """
        self.obs.lin_vel = torch.zeros(1, 3, dtype=torch.float32)
        self.obs.ang_vel = torch.zeros(1, 3, dtype=torch.float32)
        self.obs.gravity_vec = torch.tensor([[0.0, 0.0, -1.0]], dtype=torch.float32)
        self.obs.commands = torch.zeros(1, 3, dtype=torch.float32)
        self.obs.base_quat = torch.zeros(1, 4, dtype=torch.float32)
        self.obs.dof_pos = self.params.default_dof_pos.clone()
        self.obs.dof_vel = torch.zeros(1, self.params.num_of_dofs, dtype=torch.float32)
        self.obs.actions = torch.zeros(1, self.params.num_of_dofs, dtype=torch.float32)
        self.clamped_obs = torch.zeros(1, self.params.num_observations, dtype=torch.float32) # 总观测

    def _init_outputs(self):
        """ 初始化输出 """
        self.output_dof_pos = self.params.default_dof_pos.clone()
        self.output_dof_vel = torch.zeros(1, self.params.num_of_dofs, dtype=torch.float32)
        self.output_dof_tau = torch.zeros(1, self.params.num_of_dofs, dtype=torch.float32)

    def _init_history_buffer(self):
        """ 历史观测缓冲 """ 
        if len(self.params.observations_history) != 0:
            self.history_obs_buf = ObservationBuffer(1, self.params.num_observations, len(self.params.observations_history))

    def init_motion_loader(self, state: RobotState):
        if self.motion_loader is None:
            return
        base_quat = torch.tensor([state.imu.quaternion], dtype=torch.float32)
        self.motion_loader.reset(base_quat)

    def reset(self, state: RobotState):
        """重置策略运行状态。"""
        self.rl_time = 0.0
        self._init_observations()
        self._init_outputs()
        self._init_history_buffer()
        self.init_motion_loader(state)
        if hasattr(self.model, "reset"):
            self.model.reset()

    def _motion_command_obs(self) -> torch.Tensor:
        if self.motion_loader is None:
            return torch.zeros(1, self.params.num_of_dofs * 2, dtype=torch.float32)
        joint_mapping = self.params.joint_mapping_tensor.to(self.motion_loader.device)
        joint_pos_training = self.motion_loader.joint_pos().index_select(1, joint_mapping)
        joint_vel_training = self.motion_loader.joint_vel().index_select(1, joint_mapping)
        return torch.cat([joint_pos_training, joint_vel_training], dim=1)

    def _motion_anchor_ori_obs(self) -> torch.Tensor:
        if self.motion_loader is None:
            return torch.zeros(1, 6, dtype=torch.float32)
        if self.params.num_of_dofs == 12:
            real_quat_w = self.obs.base_quat
            ref_quat_w = self.motion_loader.root_quaternion()
        else:
            waist_angles = torch.tensor([[self.obs.dof_pos[0, idx].item() for idx in self.params.waist_joint_indices]], dtype=torch.float32)
            real_quat_w = self.motion_loader.torso_quat_w(self.obs.base_quat, waist_angles)
            ref_quat_w = self.motion_loader.anchor_quat_w()
        return self.motion_loader.motion_anchor_ori_b(real_quat_w, ref_quat_w)

    def _compute_observation(self):
        """ 组合观测向量 """
        obs_list = []
        for observation in self.params.observations:
            if observation == "ang_vel":
                obs_list.append(self.obs.ang_vel * self.params.ang_vel_scale)
            elif observation == "gravity_vec":
                obs_list.append(quat_rotate_inverse(self.obs.base_quat, self.obs.gravity_vec))
            elif observation == "commands":
                obs_list.append(self.obs.commands * self.params.commands_scale)
            elif observation == "dof_pos":
                dof_pos_rel = self.obs.dof_pos - self.params.default_dof_pos
                for i in self.params.wheel_indices:
                    dof_pos_rel[0, i] = 0.0
                obs_list.append(dof_pos_rel * self.params.dof_pos_scale)
            elif observation == "dof_vel":
                obs_list.append(self.obs.dof_vel * self.params.dof_vel_scale)
            elif observation == "actions":
                obs_list.append(self.obs.actions)
            elif observation == "zero":
                obs_list.append(torch.zeros(1, 3, dtype=torch.float32))
            elif observation == "motion_command":
                obs_list.append(self._motion_command_obs())
            elif observation == "motion_anchor_ori_b":
                obs_list.append(self._motion_anchor_ori_obs())
            else:
                raise ValueError(f"Unsupported obs: {observation}")    

        obs = torch.cat(obs_list, dim=1)
        return torch.clamp(obs, -self.params.clip_obs, self.params.clip_obs)

    def _compute_output(self, actions):
        """ 计算输出位置、速度、力矩 """
        actions_scaled = actions * self.params.action_scale
        pos_actions_scaled = actions_scaled.clone()
        vel_actions_scaled = torch.zeros_like(actions)
        
        for i in self.params.wheel_indices:
            pos_actions_scaled[0][i] = 0.0
            vel_actions_scaled[0][i] = actions_scaled[0][i]
        
        self.output_dof_pos = pos_actions_scaled + self.params.default_dof_pos
        self.output_dof_vel = vel_actions_scaled
        # all_actions_scaled = pos_actions_scaled + vel_actions_scaled
        # self.output_dof_tau = (self.params.rl_kp * (all_actions_scaled + self.params.default_dof_pos - self.obs.dof_pos) - self.params.rl_kd * self.obs.dof_vel)
        # self.output_dof_tau = torch.clamp(self.output_dof_tau, -self.params.torque_limits, self.params.torque_limits)

    def _forward(self, clamped_obs):
        """ 模型推理 """        
        if self.policy_name in ["np3o"]:
            history_obs = self.history_obs_buf.get_obs_vec(self.params.observations_history)
            actions = self.model(clamped_obs, history_obs)
            self.history_obs_buf.insert(clamped_obs)
        elif self.policy_name in ["himloco", "go1", "recoveryloco", "attention", "duow", "go2w_himloco", "g1_amp", "M20"]:
            self.history_obs_buf.insert(clamped_obs)
            history_obs = self.history_obs_buf.get_obs_vec(self.params.observations_history)
            actions = self.model(history_obs)
        elif self.policy_name == "moe":
            actions, (weights, latent) = self.model(clamped_obs)
            # print("weights:", weights.detach().cpu().numpy())
            # print("latent:", latent.detach().cpu().numpy())
        elif self.policy_name in ["M20_lab", "g1_loco", "gangnam_style", "dance_102", "dance1_subject2", "go2_loco", "go2_back_filp", "go2_silde_filp", "go2_jump"]:
            actions = self.model(clamped_obs)
        else:
            raise ValueError(f"Unsupported policy forward path: {self.policy_name}")

        return actions

    def run_inference(self, robot_state: RobotState, control: Control):
        """ 执行推理 50hz """
        if self.motion_loader is not None:
            motion_time = min(self.rl_time, self.motion_loader.duration)
            self.motion_loader.update(motion_time)

        # 更新观测
        for i in range(3):
            self.obs.ang_vel[0, i] = robot_state.imu.gyroscope[i]

        self.obs.commands[0, 0] = control.x
        self.obs.commands[0, 1] = control.y
        self.obs.commands[0, 2] = control.yaw

        for i in range(4):
            self.obs.base_quat[0, i] = robot_state.imu.quaternion[i]

        for i in range(self.params.num_of_dofs):
            self.obs.dof_pos[0, i] = robot_state.motor_state.q[i]
            self.obs.dof_vel[0, i] = robot_state.motor_state.dq[i]

        # 组合观测 - 推理 - 计算输出
        self.clamped_obs = self._compute_observation()
        self.obs.actions = self._forward(self.clamped_obs)
        self._compute_output(self.obs.actions)

        # time
        self.rl_time += 0.02
