#  (c) 2024-2025 zh


class PrintHelper:
    """机器人状态和命令打印"""

    def __init__(self, num_of_dofs, print_interval=100):
        self.num_of_dofs = num_of_dofs
        self.print_interval = print_interval
        self.state_counter = 0
        self.command_counter = 0

    def _should_print(self, counter):
        return counter % self.print_interval == 0

    def _format_values(self, values):
        return ", ".join(f"{values[i]:.2f}" for i in range(self.num_of_dofs))

    def print_state(self, robot_state):
        """打印状态信息"""
        self.state_counter += 1

        if self._should_print(self.state_counter):
            print("\n=== Robot State ===")
            print(f"IMU Quat: [{robot_state.imu.quaternion[0]:.2f}, "
                  f"{robot_state.imu.quaternion[1]:.2f}, "
                  f"{robot_state.imu.quaternion[2]:.2f}, "
                  f"{robot_state.imu.quaternion[3]:.2f}]")
            print(f"Gyro: [{robot_state.imu.gyroscope[0]:.2f}, "
                  f"{robot_state.imu.gyroscope[1]:.2f}, "
                  f"{robot_state.imu.gyroscope[2]:.2f}]")
            print(f"Accel: [{robot_state.imu.accelerometer[0]:.2f}, "
                  f"{robot_state.imu.accelerometer[1]:.2f}, "
                  f"{robot_state.imu.accelerometer[2]:.2f}]")
            print(f"Joint q: [{self._format_values(robot_state.motor_state.q)}]")
            print(f"Joint v: [{self._format_values(robot_state.motor_state.dq)}]")
            print(f"Joint t: [{self._format_values(robot_state.motor_state.tau_est)}]")

    def print_command(self, robot_command):
        """打印命令信息"""
        self.command_counter += 1

        if self._should_print(self.command_counter):
            print("\n=== Robot Command ===")
            print(f"Cmd q:  [{self._format_values(robot_command.motor_command.q)}]")
            print(f"Cmd dq: [{self._format_values(robot_command.motor_command.dq)}]")
            print(f"Cmd tau:[{self._format_values(robot_command.motor_command.tau)}]")
            print(f"Cmd kp: [{self._format_values(robot_command.motor_command.kp)}]")
            print(f"Cmd kd: [{self._format_values(robot_command.motor_command.kd)}]")
