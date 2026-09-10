#  (c) 2024-2025 zh
# Gazebo ROS2 通信接口

import threading
import numpy as np
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState, Imu
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from src.utils.helper import PrintHelper


class IOGazebo(Node):
    """Gazebo ROS2 接口 独立线程运行"""

    def __init__(self, num_of_dofs, joint_names):
        # 初始化 ROS2
        if not rclpy.ok():
            rclpy.init()
        
        super().__init__('io_gazebo')

        self.num_of_dofs = num_of_dofs

        # 关节名称映射
        self.joint_names = joint_names

        # 状态缓存
        self._lock = threading.Lock()
        self._shutdown_flag = False
        self.joint_positions = np.zeros(self.num_of_dofs)
        self.joint_velocities = np.zeros(self.num_of_dofs)
        self.joint_efforts = np.zeros(self.num_of_dofs)
        self.imu_quaternion = np.array([1.0, 0.0, 0.0, 0.0])  # w, x, y, z
        self.imu_gyroscope = np.zeros(3)
        self.imu_accelerometer = np.zeros(3)

        # 命令缓存
        self.joint_commands = np.zeros(self.num_of_dofs)

        # QoS 配置 (适用于传感器数据)
        sensor_qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST, depth=10)

        # ROS2 订阅者
        self.joint_state_sub = self.create_subscription(JointState,'/joint_states',self.joint_state_callback,sensor_qos)
        self.imu_sub = self.create_subscription(Imu,'/IMU_data',self.imu_callback,sensor_qos)

        # ROS2 发布者
        self.joint_cmd_pub = self.create_publisher(Float64MultiArray,'/robot_joint_controller/commands',10)

        # Gazebo 服务
        self.reset_sim_client = self.create_client(Empty, '/reset_world')

        # 启动独立线程运行 ROS2 spin
        self._spin_thread = threading.Thread(target=self._spin, daemon=True)
        self._spin_thread.start()

        self.helper = PrintHelper(self.num_of_dofs)

        self.get_logger().info("IOGazebo initialized with independent thread")

    def _spin(self):
        """独立线程运行 ROS2 spin"""
        try:
            while rclpy.ok() and not self._shutdown_flag:
                rclpy.spin_once(self, timeout_sec=0.1)
        except Exception as e:
            if not self._shutdown_flag:
                self.get_logger().error(f"ROS2 spin thread error: {e}")

    def joint_state_callback(self, msg: JointState):
        """关节状态回调"""
        # 建立名称到索引的映射
        name_to_idx = {name: i for i, name in enumerate(msg.name)}

        with self._lock:
            for i, joint_name in enumerate(self.joint_names):
                if joint_name in name_to_idx:
                    idx = name_to_idx[joint_name]
                    if idx < len(msg.position):
                        self.joint_positions[i] = msg.position[idx]
                    if idx < len(msg.velocity):
                        self.joint_velocities[i] = msg.velocity[idx]
                    if idx < len(msg.effort):
                        self.joint_efforts[i] = msg.effort[idx]

    def imu_callback(self, msg: Imu):
        """IMU 数据回调"""
        with self._lock:
            # 四元数: w,x,y,z
            self.imu_quaternion[0] = msg.orientation.w
            self.imu_quaternion[1] = msg.orientation.x
            self.imu_quaternion[2] = msg.orientation.y
            self.imu_quaternion[3] = msg.orientation.z

            # 角速度 (rad/s)
            self.imu_gyroscope[0] = msg.angular_velocity.x
            self.imu_gyroscope[1] = msg.angular_velocity.y
            self.imu_gyroscope[2] = msg.angular_velocity.z

            # 线加速度 (m/s^2)
            self.imu_accelerometer[0] = msg.linear_acceleration.x
            self.imu_accelerometer[1] = msg.linear_acceleration.y
            self.imu_accelerometer[2] = msg.linear_acceleration.z

    def recv(self, robot_state):
        """从 Gazebo 接收状态"""
        with self._lock:
            # 读取关节状态
            for i in range(self.num_of_dofs):
                robot_state.motor_state.q[i] = float(self.joint_positions[i])
                robot_state.motor_state.dq[i] = float(self.joint_velocities[i])
                robot_state.motor_state.tau_est[i] = float(self.joint_efforts[i])

            # 读取 IMU 数据
            for i in range(4):
                robot_state.imu.quaternion[i] = self.imu_quaternion[i]

            for i in range(3):
                robot_state.imu.gyroscope[i] = self.imu_gyroscope[i]
                robot_state.imu.accelerometer[i] = self.imu_accelerometer[i]

        # self.helper.print_state(robot_state)

    def send(self, robot_command):
        """发送命令到 Gazebo"""
        # 读取当前关节状态
        with self._lock:
            q_actual = self.joint_positions.copy()
            dq_actual = self.joint_velocities.copy()

        # 计算所有关节的控制力矩
        for i in range(self.num_of_dofs):
            q_des = robot_command.motor_command.q[i]
            dq_des = robot_command.motor_command.dq[i]
            tau_ff = robot_command.motor_command.tau[i]
            kp = robot_command.motor_command.kp[i]
            kd = robot_command.motor_command.kd[i]

            # PD控制 + 前馈力矩
            tau = tau_ff + kp * (q_des - q_actual[i]) + kd * (dq_des - dq_actual[i])
            self.joint_commands[i] = tau

        # 发布12个tau
        msg = Float64MultiArray()
        msg.data = self.joint_commands.tolist()
        self.joint_cmd_pub.publish(msg)

        # self.helper.print_command(robot_command)

    def reset_simulation(self):
        """重置仿真"""
        if not self.reset_sim_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Service /reset_world not available")
            return False
        
        request = Empty.Request()
        future = self.reset_sim_client.call_async(request)
        return True

    def shutdown(self):
        """关闭 ROS2 节点"""
        self._shutdown_flag = True
        self._spin_thread.join(timeout=1.0)
        self.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
