#  (c) 2024-2025 zh

from motor_sdk.go1_pro_sdk.ucl.lowCmd import lowCmd
from motor_sdk.go1_pro_sdk.ucl.lowState import lowState
from motor_sdk.go1_pro_sdk.ucl.enums import MotorModeLow
from motor_sdk.go1_pro_sdk.ucl.complex import motorCmdArray
from motor_sdk.go1_pro_sdk.ucl.unitreeConnection import unitreeConnection, LOW_WIRED_DEFAULTS
from motor_sdk.go1_pro_sdk.ucl.common import float_to_hex_fast, kp_to_hex_fast, kd_to_hex_fast, tau_to_hex_fast, genCrc, encryptCrc
from src.utils.helper import PrintHelper


class IOReal_go1:
    """Real robot I/O interface for Unitree robots"""

    LOGGER = "[IOReal_GO1]"

    def __init__(self):
        """Initialize real robot I/O interface"""
        self.num_of_dofs = 12
        
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
        
        # 初始化UDP连接
        self.conn = unitreeConnection(LOW_WIRED_DEFAULTS)
        
        # 发送初始化命令
        cmd_bytes = self.lcmd.buildCmd(debug=False)
        self.conn.send(cmd_bytes)
        
        # 设置数据接收回调并启动接收线程（最后启动，避免回调访问未初始化的属性）
        self.conn.set_data_callback(self._on_udp_data_received)
        self.conn.startRecv()

        self.helper = PrintHelper(self.num_of_dofs)

        print(f"{self.LOGGER} Unitree robot connection initialized")

    def recv(self, robot_state):
        """receive state from real robot"""
        # 读取关节状态
        motors = self.lstate.motorState
        for i in range(self.num_of_dofs):
            robot_state.motor_state.q[i] = float(motors[i].q)
            robot_state.motor_state.dq[i] = float(motors[i].dq)
            robot_state.motor_state.tau_est[i] = float(motors[i].tauEst)

        # 读取IMU数据
        quat = self.lstate.imu.quaternion
        robot_state.imu.quaternion[0] = float(quat[0])  # w
        robot_state.imu.quaternion[1] = float(quat[1])  # x
        robot_state.imu.quaternion[2] = float(quat[2])  # y
        robot_state.imu.quaternion[3] = float(quat[3])  # z

        # 读取陀螺仪 (rad/s)
        gyro = self.lstate.imu.gyroscope
        robot_state.imu.gyroscope[0] = float(gyro[0])
        robot_state.imu.gyroscope[1] = float(gyro[1])
        robot_state.imu.gyroscope[2] = float(gyro[2])

        # 读取加速度计 (m/s²)
        acc = self.lstate.imu.accelerometer
        robot_state.imu.accelerometer[0] = float(acc[0])
        robot_state.imu.accelerometer[1] = float(acc[1])
        robot_state.imu.accelerometer[2] = float(acc[2])

        # self.helper.print_state(robot_state)

    def send(self, robot_command):
        """发送电机命令（优化版本：只在参数变化时重建buffer，否则只更新q位置）"""
        # 提取所有关节的参数
        positions = []
        velocities = []
        torques = []
        kp_list = []
        kd_list = []

        for i in range(12):
            positions.append(robot_command.motor_command.q[i])
            velocities.append(robot_command.motor_command.dq[i])
            kp_list.append(robot_command.motor_command.kp[i])
            kd_list.append(robot_command.motor_command.kd[i])
            torques.append(robot_command.motor_command.tau[i])

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

        # self.helper.print_command(robot_command)

    def _on_udp_data_received(self, data):
        """UDP数据接收回调函数"""
        try:
            if data is not None and len(data) > 0:
                self.lstate.parseData(data)  # 解析UDP数据
        except Exception as e:
            print(f"{self.LOGGER} Error parsing UDP data: {str(e)}")

    def __del__(self):
        """清理资源"""
        try:
            self.conn.stopRecv()
        except Exception:
            pass
