#  (c) 2024-2025 zh

from src.utils.helper import PrintHelper


class IOMuJoCo:
    def __init__(self, mj_model, mj_data, num_of_dofs):
        """Initialize MuJoCo I/O interface"""
        self.mj_model = mj_model
        self.mj_data = mj_data
        self.num_of_dofs = num_of_dofs
        self.helper = PrintHelper(self.num_of_dofs)

    def recv(self, robot_state):
        """Receive state from MuJoCo simulation"""
        # Read joint states
        for i in range(self.num_of_dofs):
            robot_state.motor_state.q[i] = float(self.mj_data.sensordata[i])
            robot_state.motor_state.dq[i] = float(self.mj_data.sensordata[i + self.num_of_dofs])
            robot_state.motor_state.tau_est[i] = float(self.mj_data.sensordata[i + 2 * self.num_of_dofs])

        # Read IMU data
        imu_offset = 3 * self.num_of_dofs
        robot_state.imu.quaternion[0] = float(self.mj_data.sensordata[imu_offset + 0])  # w
        robot_state.imu.quaternion[1] = float(self.mj_data.sensordata[imu_offset + 1])  # x
        robot_state.imu.quaternion[2] = float(self.mj_data.sensordata[imu_offset + 2])  # y
        robot_state.imu.quaternion[3] = float(self.mj_data.sensordata[imu_offset + 3])  # z
        
        # Read gyroscope (rad/s)
        robot_state.imu.gyroscope[0] = float(self.mj_data.sensordata[imu_offset + 4])
        robot_state.imu.gyroscope[1] = float(self.mj_data.sensordata[imu_offset + 5])
        robot_state.imu.gyroscope[2] = float(self.mj_data.sensordata[imu_offset + 6])
        
        # Read accelerometer (m/s²)
        robot_state.imu.accelerometer[0] = float(self.mj_data.sensordata[imu_offset + 7])
        robot_state.imu.accelerometer[1] = float(self.mj_data.sensordata[imu_offset + 8])
        robot_state.imu.accelerometer[2] = float(self.mj_data.sensordata[imu_offset + 9])

        # self.helper.print_state(robot_state)

    def send(self, robot_command):
        """Send commands to MuJoCo simulation"""
        # Apply commands
        for i in range(self.num_of_dofs):
            q_actual = self.mj_data.sensordata[i]
            dq_actual = self.mj_data.sensordata[i + self.num_of_dofs]
            
            q_des = robot_command.motor_command.q[i]
            dq_des = robot_command.motor_command.dq[i]
            tau_ff = robot_command.motor_command.tau[i]
            kp = robot_command.motor_command.kp[i]
            kd = robot_command.motor_command.kd[i]

            # Apply control to actuator
            self.mj_data.ctrl[i] = (tau_ff + kp * (q_des - q_actual) + kd * (dq_des - dq_actual))    

        # self.helper.print_command(robot_command)
