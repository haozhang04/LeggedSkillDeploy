#  (c) 2024-2025 zh

import time
import torch
from src.scripts.rl_data import Control, RobotCommand, RobotState
from src.scripts.rl_sdk import LOGGER, STATE, RLPolicy

torch.set_num_threads(4)
torch.set_num_interop_threads(1)
torch.set_grad_enabled(False)

class RLDeploy:
    """Common state machine and multi-policy switching."""

    def __init__(self, policy_dirs: list[str]):
        # 初始化
        if not isinstance(policy_dirs, list):
            raise TypeError("policy_dirs must be a list[str]")

        # 电机接口
        self.temp_robot_state = RobotState()
        self.temp_robot_command = RobotCommand()
        self.robot_state = RobotState()
        self.robot_command = RobotCommand()
        # 控制接口
        self.control = Control()

        # 状态机
        self.running_state = STATE.STATE_WAITING
        self.now_state = RobotState()
        self.getup_percent = 0.0
        # 推理计数器
        self.inference_counter = 0
        # 模型切换
        self.last_model_flag = -1 
        self.auto_return_model_flag = 0

        # 加载所有策略
        self.policies = [RLPolicy(policy_dir) for policy_dir in policy_dirs]
        self.model_entries = []
        for policy_index, policy in enumerate(self.policies):
            for model_index, model_name in enumerate(policy.params.model_names):
                label = f"{policy.policy_rel_dir}/{model_name}"
                self.model_entries.append((policy_index, model_index, label))
        self.model_names = [entry[2] for entry in self.model_entries]
        # print(f"model_entries: {self.model_entries}")
        print(f"{LOGGER.INFO}Loaded {len(self.model_names)} models: {self.model_names}")

        # 使用第一个模型
        self.change_model(0)

        # Debug 频率统计
        # self._enable_hz_logging = True
        self._enable_hz_logging = False
        self._loop_count = 0
        self._last_log_time = time.perf_counter()


    def change_model(self, key: int):
        """ 切换模型 """        
        if key < 0 or key >= len(self.model_entries):
            print(f"{LOGGER.ERROR}Model index {key} out of range")
            return
        if key == self.last_model_flag:
            return

        policy_index, model_index, label = self.model_entries[key]
        self.policy = self.policies[policy_index]
        self.params = self.policy.params
        self.policy.model = self.policy.models[model_index]
        policy_changed = self.last_model_flag < 0 or policy_index != self.model_entries[self.last_model_flag][0]
        if policy_changed:
            self.policy.reset(self.robot_state)
        self.last_model_flag = key    
        print(f"{LOGGER.INFO}Change model [{key}]: {label}")

    def _auto_return_model_when_motion_finished(self):
        if self.policy.motion_loader is None:
            return
        if self.last_model_flag == self.auto_return_model_flag:
            return
        if self.policy.rl_time < self.policy.motion_loader.duration:
            return
        self.control.model_flag = self.auto_return_model_flag
        
    def _get_state(self):
        """ 读取状态 """
        # [w, x, y, z]
        for i in range(4):
            self.robot_state.imu.quaternion[i] = self.temp_robot_state.imu.quaternion[i]
        
        for i in range(3):
            self.robot_state.imu.gyroscope[i] = self.temp_robot_state.imu.gyroscope[i]
        
        for i in range(self.params.num_of_dofs):
            idx = self.params.joint_mapping[i]
            self.robot_state.motor_state.q[i] = self.temp_robot_state.motor_state.q[idx]
            self.robot_state.motor_state.dq[i] = self.temp_robot_state.motor_state.dq[idx]
            self.robot_state.motor_state.tau_est[i] = self.temp_robot_state.motor_state.tau_est[idx]

    def _set_command(self):
        """ 发送命令 """
        for i in range(self.params.num_of_dofs):
            idx = self.params.joint_mapping[i]
            self.temp_robot_command.motor_command.q[idx] = self.robot_command.motor_command.q[i]
            self.temp_robot_command.motor_command.dq[idx] = self.robot_command.motor_command.dq[i]
            self.temp_robot_command.motor_command.tau[idx] = self.robot_command.motor_command.tau[i]
            self.temp_robot_command.motor_command.kp[idx] = self.robot_command.motor_command.kp[i]
            self.temp_robot_command.motor_command.kd[idx] = self.robot_command.motor_command.kd[i]

    def _run_inference(self):
        if self.running_state != STATE.STATE_RL_RUNNING:
            return
        self.policy.run_inference(self.robot_state, self.control)
        self._auto_return_model_when_motion_finished()

    def _state_controller(self):
        """ 状态机控制 """
        state = self.robot_state
        command = self.robot_command
        
        # WAITING 状态
        if self.running_state == STATE.STATE_WAITING:
            for i in range(self.params.num_of_dofs):
                command.motor_command.q[i] = state.motor_state.q[i]
            
            if self.control.control_state == STATE.STATE_POS_GETUP:
                self.control.control_state = STATE.STATE_WAITING
                self.getup_percent = 0.0
                for i in range(self.params.num_of_dofs):
                    self.now_state.motor_state.q[i] = state.motor_state.q[i]
                self.running_state = STATE.STATE_POS_GETUP
                print(f"\n{LOGGER.INFO}Switching to STATE_POS_GETUP")
        
        # GETUP 状态
        elif self.running_state == STATE.STATE_POS_GETUP:
            if self.getup_percent < 1.0:
                self.getup_percent += 1 / 500.0
                self.getup_percent = min(self.getup_percent, 1.0)
                for i in range(self.params.num_of_dofs):
                    command.motor_command.q[i] = (1 - self.getup_percent) * self.now_state.motor_state.q[i] + self.getup_percent * self.params.default_dof_pos[0][i].item()
                    command.motor_command.dq[i] = 0
                    command.motor_command.tau[i] = 0
                    command.motor_command.kp[i] = self.params.fixed_kp[0][i].item()
                    command.motor_command.kd[i] = self.params.fixed_kd[0][i].item()
                print(f"\r{LOGGER.INFO}Getting up {self.getup_percent * 100.0:.1f}%", end="", flush=True)
            
            if self.control.control_state == STATE.STATE_RL_RUNNING:
                self.control.control_state = STATE.STATE_WAITING
                self.running_state = STATE.STATE_RL_RUNNING
                print(f"\n{LOGGER.INFO}Switching to STATE_RL_RUNNING")
            elif self.control.control_state == STATE.STATE_POS_GETDOWN:
                self.control.control_state = STATE.STATE_WAITING
                self.running_state = STATE.STATE_POS_GETDOWN
                print(f"\n{LOGGER.INFO}Switching to STATE_POS_GETDOWN")

        # RL_RUNNING 状态
        elif self.running_state == STATE.STATE_RL_RUNNING:
            print(f"\r{LOGGER.INFO}RL x:{self.control.x:.2f} y:{self.control.y:.2f} yaw:{self.control.yaw:.2f}", end="", flush=True)
            
            num_dofs = self.params.num_of_dofs
            command.motor_command.q[:num_dofs] = self.policy.output_dof_pos[0].detach().numpy()
            command.motor_command.dq[:num_dofs] = self.policy.output_dof_vel[0].detach().numpy()
            command.motor_command.tau[:num_dofs] = [0.0] * num_dofs
            command.motor_command.kp[:num_dofs] = self.params.rl_kp[0].detach().numpy()
            command.motor_command.kd[:num_dofs] = self.params.rl_kd[0].detach().numpy()
            
            if self.control.control_state == STATE.STATE_POS_GETDOWN:
                self.control.control_state = STATE.STATE_WAITING
                self.running_state = STATE.STATE_POS_GETDOWN
                print(f"\n{LOGGER.INFO}Switching to STATE_POS_GETDOWN")
            elif self.control.control_state == STATE.STATE_POS_GETUP:
                self.control.control_state = STATE.STATE_WAITING
                self.getup_percent = 0.0
                for i in range(self.params.num_of_dofs):
                    self.now_state.motor_state.q[i] = state.motor_state.q[i]
                self.running_state = STATE.STATE_POS_GETUP
                print(f"\n{LOGGER.INFO}Switching to STATE_POS_GETUP")
        
        # GETDOWN 状态
        elif self.running_state == STATE.STATE_POS_GETDOWN:
            for i in range(self.params.num_of_dofs):
                command.motor_command.q[i] = self.now_state.motor_state.q[i]
                command.motor_command.dq[i] = 0
                command.motor_command.kp[i] = 0.0
                command.motor_command.kd[i] = 3.0
                command.motor_command.tau[i] = 0
            self.running_state = STATE.STATE_WAITING
            print(f"\r{LOGGER.INFO}Getting down", end="", flush=True)

    def step(self, inference_times: int):
        """单步执行：修改模型 -> 读取状态 -> 推理 -> 状态机 -> 发送命令."""
        self.change_model(self.control.model_flag)

        self._get_state()

        self.inference_counter += 1
        if self.inference_counter >= inference_times:
            self._run_inference()
            self.inference_counter = 0

        self._state_controller()
        self._set_command()

        if self._enable_hz_logging:
            self._loop_count += 1
            now = time.perf_counter()
            if now - self._last_log_time >= 1.0:
                hz = self._loop_count / (now - self._last_log_time)
                print(f"\n{LOGGER.DEBUG}Control loop: {hz:.1f} Hz")
                self._loop_count = 0
                self._last_log_time = now
