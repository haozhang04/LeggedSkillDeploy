#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray

from ucl.common import (float_to_hex_fast, kp_to_hex_fast, kd_to_hex_fast,
                        tau_to_hex_fast, genCrc, encryptCrc)
from ucl.lowState import lowState
from ucl.lowCmd import lowCmd
from ucl.unitreeConnection import unitreeConnection, LOW_WIFI_DEFAULTS
from ucl.enums import MotorModeLow
from ucl.complex import motorCmdArray


class UnitreeDogNode(Node):
    """ROS2 节点，用于控制Unitree机器狗并发布传感器数据"""
    
    def __init__(self):
        super().__init__('unitree_dog_node')
        
        # 关节名称定义（12个主要关节）
        self.joint_names = [
            'FR_0', 'FR_1', 'FR_2',  # 前右腿
            'FL_0', 'FL_1', 'FL_2',  # 前左腿
            'RR_0', 'RR_1', 'RR_2',  # 后右腿
            'RL_0', 'RL_1', 'RL_2'   # 后左腿
        ]
        
        # 初始化命令和状态对象
        self.lcmd = lowCmd()
        self.lstate = lowState()
        self.mCmdArr = motorCmdArray()
        
        # ========================
        # 预分配命令缓冲区（优化）
        # ========================
        self._cmd_buffer = bytearray(614)  # 预分配完整命令缓冲区
        # 填充固定头部
        self._cmd_buffer[0:2] = bytes.fromhex('FEEF')  # head
        self._cmd_buffer[2] = 0xff  # levelFlag
        self._cmd_buffer[3] = 0  # frameReserve
        self._cmd_buffer[4:12] = bytearray(8)  # SN
        self._cmd_buffer[12:20] = bytearray(8)  # version
        self._cmd_buffer[20:22] = bytes.fromhex('3ac0')  # bandWidth
        # motorCmd 区域: [22:562] = 540 bytes = 20 motors * 27 bytes
        # bms: [562:566]
        self._cmd_buffer[562:566] = bytes([0, 0, 0, 0])  # bmsCmd
        # wirelessRemote: [566:606]
        self._cmd_buffer[566:606] = bytearray(40)
        # reserve: [606:610]
        self._cmd_buffer[606:610] = bytearray(4)
        # crc: [610:614]
        
        # 预计算 Servo mode 字节
        self._servo_mode = MotorModeLow.Servo.value
        self._motor_cmd_size = 27
        
        # 预计算 12 个电机的 q 偏移量 (每个电机 27 bytes，q 在 offset+1)
        motor_offset = 22
        self._q_offsets = tuple(motor_offset + i * 27 + 1 for i in range(12))
        
        # 缓存上次的参数，用于检测是否需要重新填充 buffer
        self._last_kp = None
        self._last_kd = None
        self._buffer_initialized = False
        
        # ROS2 订阅者：订阅LLC话题（关节控制命令）
        # 消息格式：[q, dq, Kp, Kd, tau] * 12 个关节 (mode固定为Servo)
        self.llc_subscriber = self.create_subscription(
            Float64MultiArray,
            'llc',
            self.llc_callback,
            10
        )
        
        # ROS2 发布者：发布IMU数据
        self.imu_publisher = self.create_publisher(
            Imu,
            'imudata',
            10
        )
        
        # ROS2 发布者：发布关节状态
        self.joint_state_publisher = self.create_publisher(
            JointState,
            'lls',
            10
        )
        
        # 初始化UDP连接
        self.get_logger().info('初始化Unitree机器狗连接...')
        self.conn = unitreeConnection(LOW_WIFI_DEFAULTS)
        
        # 发送初始化命令
        cmd_bytes = self.lcmd.buildCmd(debug=False)
        self.conn.send(cmd_bytes)
        self.get_logger().info('已发送初始化命令')
        
        # 设置数据接收回调并启动接收线程（最后启动，避免回调访问未初始化的属性）
        self.conn.set_data_callback(self._on_udp_data_received)
        self.conn.startRecv()
        
        # 预分配消息对象（性能优化：避免1000Hz下频繁创建对象）
        self._imu_msg = Imu()
        self._imu_msg.header.frame_id = 'imu_link'
        self._joint_state_msg = JointState()
        self._joint_state_msg.header.frame_id = ''
        self._joint_state_msg.name = self.joint_names
        
        self.get_logger().info('Unitree机器狗节点已启动,V_pro')
    
    def llc_callback(self, msg):
        """处理LLC话题（关节控制命令）
        
        消息格式：Float64MultiArray，包含60个元素
        每5个元素对应一个关节：[q, dq, Kp, Kd, tau]
        共12个关节，mode固定为Servo模式
        
        优化版本：只在参数变化时重建 buffer，否则只更新 q 位置
        """
        try:
            data = msg.data
            if len(data) != 60:  # 12关节 * 5参数
                self.get_logger().warn(f'LLC消息长度错误，期望60，实际{len(data)}')
                return
            
            # 提取所有关节的参数
            positions = []
            velocities = []
            torques = []
            kp_list = []
            kd_list = []
            
            for i in range(12):
                idx = i * 5
                positions.append(data[idx])  # q
                velocities.append(data[idx + 1])  # dq
                kp_list.append(data[idx + 2])  # Kp
                kd_list.append(data[idx + 3])  # Kd
                torques.append(data[idx + 4])  # tau
            
            # 假设所有关节使用相同的 Kp 和 Kd
            kp = kp_list[0]
            kd = kd_list[0]
            
            # 检查是否需要重新填充 buffer（参数变化时）
            need_reinit = (
                not self._buffer_initialized or
                kp != self._last_kp or
                kd != self._last_kd
            )
            
            if need_reinit:
                # 首次调用或参数变化，填充完整 buffer
                kp_bytes = kp_to_hex_fast(kp)
                kd_bytes = kd_to_hex_fast(kd)
                reserve_bytes = b'\x00' * 12
                
                motor_offset = 22
                for i in range(12):
                    offset = motor_offset + i * self._motor_cmd_size
                    
                    # mode (1 byte)
                    self._cmd_buffer[offset] = self._servo_mode
                    # q 跳过，后面统一填充
                    # dq (4 bytes)
                    self._cmd_buffer[offset+5:offset+9] = float_to_hex_fast(velocities[i])
                    # tau (2 bytes)
                    self._cmd_buffer[offset+9:offset+11] = tau_to_hex_fast(torques[i])
                    # Kp (2 bytes)
                    self._cmd_buffer[offset+11:offset+13] = kp_bytes
                    # Kd (2 bytes)
                    self._cmd_buffer[offset+13:offset+15] = kd_bytes
                    # reserve (12 bytes)
                    self._cmd_buffer[offset+15:offset+27] = reserve_bytes
                
                # 填充剩余 8 个电机 (Unknown1-8) 为零
                zero_motor = b'\x00' * 27
                for i in range(12, 20):
                    offset = motor_offset + i * self._motor_cmd_size
                    self._cmd_buffer[offset:offset+27] = zero_motor
                
                # 更新缓存
                self._last_kp = kp
                self._last_kd = kd
                self._buffer_initialized = True
            
            # 每次调用只更新 q 位置（12 个电机 × 4 bytes）
            buf = self._cmd_buffer
            for i in range(12):
                qoff = self._q_offsets[i]
                buf[qoff:qoff+4] = float_to_hex_fast(positions[i])
            
            # 计算 CRC
            crc = encryptCrc(genCrc(buf[:-6]))
            buf[-4:] = crc
            
            # 发送
            self.conn.send(buf)
            
        except Exception as e:
            self.get_logger().error(f'处理LLC消息时出错: {str(e)}')

    def _on_udp_data_received(self, data):
        """UDP数据接收回调函数
        数据流向：UDP socket -> 此回调 -> 解析 -> ROS2发布
        """
        try:
            if data is not None and len(data) > 0:
                self.lstate.parseData(data)  # 解析UDP数据
                self._publish_sensor_data()  # 立即发布传感器数据
        except Exception as e:
            self.get_logger().error(f'处理UDP数据时出错: {str(e)}')

    def _publish_sensor_data(self):
        """发布IMU数据和关节状态数据（优化：复用预分配对象+列表推导）"""
        try:
            # 更新时间戳（只获取一次）
            timestamp = self.get_clock().now().to_msg()
            
            # 更新 IMU 数据（直接修改预分配对象）
            self._imu_msg.header.stamp = timestamp
            
            # 四元数 (w, x, y, z)
            quat = self.lstate.imu.quaternion
            self._imu_msg.orientation.w = quat[0]
            self._imu_msg.orientation.x = quat[1]
            self._imu_msg.orientation.y = quat[2]
            self._imu_msg.orientation.z = quat[3]
            
            # 角速度 (rad/s)
            gyro = self.lstate.imu.gyroscope
            self._imu_msg.angular_velocity.x = gyro[0]
            self._imu_msg.angular_velocity.y = gyro[1]
            self._imu_msg.angular_velocity.z = gyro[2]
            
            # 加速度 (m/s^2)
            acc = self.lstate.imu.accelerometer
            self._imu_msg.linear_acceleration.x = acc[0]
            self._imu_msg.linear_acceleration.y = acc[1]
            self._imu_msg.linear_acceleration.z = acc[2]
            
            self.imu_publisher.publish(self._imu_msg)
            
            # 更新关节状态数据（使用列表推导式）
            self._joint_state_msg.header.stamp = timestamp
            motors = self.lstate.motorState
            self._joint_state_msg.position = [motors[i].q for i in range(12)]
            self._joint_state_msg.velocity = [motors[i].dq for i in range(12)]
            self._joint_state_msg.effort = [motors[i].tauEst for i in range(12)]
            
            self.joint_state_publisher.publish(self._joint_state_msg)
            
        except Exception as e:
            self.get_logger().error(f'发布传感器数据时出错: {str(e)}')
    
    def destroy_node(self):
        """节点销毁时清理资源"""
        self.conn.stopRecv()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UnitreeDogNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()