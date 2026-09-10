# Copyright (c) 2024-2025 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

from enum import Enum, auto


class LOGGER:
    INFO = "\033[0;37m[INFO]\033[0m "
    WARNING = "\033[0;33m[WARNING]\033[0m "
    ERROR = "\033[0;31m[ERROR]\033[0m "
    DEBUG = "\033[0;32m[DEBUG]\033[0m "

class RobotCommand:
    def __init__(self):
        self.motor_command = self.MotorCommand()

    class MotorCommand:
        def __init__(self):
            self.q = [0.0] * 32
            self.dq = [0.0] * 32
            self.tau = [0.0] * 32
            self.kp = [0.0] * 32
            self.kd = [0.0] * 32

class RobotState:
    def __init__(self):
        self.imu = self.IMU()
        self.motor_state = self.MotorState()

    class IMU:
        def __init__(self):
            self.quaternion = [1.0, 0.0, 0.0, 0.0]  # w, x, y, z
            self.gyroscope = [0.0, 0.0, 0.0]
            self.accelerometer = [0.0, 0.0, 0.0]

    class MotorState:
        def __init__(self):
            self.q = [0.0] * 32
            self.dq = [0.0] * 32
            self.ddq = [0.0] * 32
            self.tau_est = [0.0] * 32
            self.cur = [0.0] * 32

class STATE(Enum):
    STATE_WAITING = 0
    STATE_POS_GETUP = auto()
    STATE_RL_RUNNING = auto()
    STATE_POS_GETDOWN = auto()
    # sim state
    STATE_RESET_SIMULATION = auto()
    STATE_TOGGLE_SIMULATION = auto()

class Control:
    def __init__(self):
        self.control_state = STATE.STATE_WAITING
        self.model_flag: int = 0
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

class ModelParams:
    def __init__(self):
        self.model_names = None
        self.framework = None
        self.dt = None
        self.decimation = None
        self.num_of_dofs = None
        self.num_observations = None
        self.observations = None
        self.observations_history = None
        self.clip_obs = None
        self.fixed_kp = None
        self.fixed_kd = None
        self.rl_kd = None
        self.rl_kp = None
        self.action_scale = None
        self.wheel_indices = None
        self.ang_vel_scale = None
        self.dof_pos_scale = None
        self.dof_vel_scale = None
        self.commands_scale = None
        self.torque_limits = None
        self.default_dof_pos = None
        self.joint_controller_names = None
        self.joint_mapping = None
        self.waist_joint_indices = None
        self.motion_params = None

class Observations:
    def __init__(self):
        self.ang_vel = None
        self.gravity_vec = None
        self.commands = None
        self.base_quat = None
        self.dof_pos = None
        self.dof_vel = None
        self.actions = None
