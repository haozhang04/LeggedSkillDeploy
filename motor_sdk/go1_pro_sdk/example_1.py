#FreeDog
from .common import byte_print, decode_version, decode_sn, getVoltage, pretty_print_obj, lib_version
from .common import float_to_hex_fast, kp_to_hex_fast, kd_to_hex_fast, tau_to_hex_fast, genCrc, encryptCrc
from .lowState import lowState
from .lowCmd import lowCmd
from .unitreeConnection import unitreeConnection, LOW_WIFI_DEFAULTS, LOW_WIRED_DEFAULTS
from .enums import GaitType, SpeedLevel, MotorModeLow
from .complex import motorCmd, motorCmdArray
import time
import sys
import math
from pprint import pprint
import struct

import numpy as np

class interfaceFreeDog:
    def __init__(self):
        print("===")
        print(f'Running lib version: {lib_version()}')
        # self.conn = unitreeConnection()
        self.conn = unitreeConnection(LOW_WIRED_DEFAULTS)
        self.conn.startRecv()
        self.lcmd = lowCmd()
        # lcmd.encrypt = True
        self.lstate = lowState()
        self.mCmdArr = motorCmdArray()
        # Send empty command to tell the dog the receive port and initialize the connection
        self.cmd_bytes = self.lcmd.buildCmd(debug=False)
        self.conn.send(self.cmd_bytes)
        self.d = {'FR_0':0, 'FR_1':1, 'FR_2':2,
                    'FL_0':3, 'FL_1':4, 'FL_2':5,
                    'RR_0':6, 'RR_1':7, 'RR_2':8,
                    'RL_0':9, 'RL_1':10, 'RL_2':11 }
        
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
        # motorCmd 区域: [22:562] = 540 bytes = 20 motors * 27 bytes each
        # bms: [562:566]
        self._cmd_buffer[562:566] = bytes([0, 0, 0, 0])  # bmsCmd
        # wirelessRemote: [566:606]
        self._cmd_buffer[566:606] = bytearray(40)
        # reserve: [606:610]
        self._cmd_buffer[606:610] = bytearray(4)
        # crc: [610:614]
        
        # 预计算 Servo mode 字节
        self._servo_mode = MotorModeLow.Servo.value
        
        # 预分配单个电机命令缓冲区 (27 bytes)
        self._motor_cmd_size = 27
        
        # 预计算 12 个电机的 q 偏移量 (每个电机 27 bytes，q 在 offset+1 位置)
        motor_offset = 22
        self._q_offsets = tuple(motor_offset + i * 27 + 1 for i in range(12))
        
        # 缓存上次的参数，用于检测是否需要重新填充 buffer
        self._last_kp = None
        self._last_kd = None
        self._last_has_velocities = None
        self._last_has_torques = None
        self._buffer_initialized = False
        
        # SendTorqueFast 的缓存
        self._torque_buffer_initialized = False
        self._last_pos_stop = None
        self._last_vel_stop = None
        # tau 偏移量 (每个电机 27 bytes，tau 在 offset+9 位置)
        self._tau_offsets = tuple(motor_offset + i * 27 + 9 for i in range(12))
        
        data = self.conn.getData()
        print(data)
        for paket in data:
            print('+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=')
            self.lstate.parseData(paket)
            print(f'SN [{byte_print(self.lstate.SN)}]:\t{decode_sn(self.lstate.SN)}')
            print(f'Ver [{byte_print(self.lstate.version)}]:\t{decode_version(self.lstate.version)}')
            print(f'SOC:\t\t\t{self.lstate.bms.SOC} %')
            print(f'Overall Voltage:\t{getVoltage(self.lstate.bms.cell_vol)} mv')
            print(f'Current:\t\t{self.lstate.bms.current} mA')
            print(f'Cycles:\t\t\t{self.lstate.bms.cycle}')
            print(f'Temps BQ:\t\t{self.lstate.bms.BQ_NTC[0]} °C, {self.lstate.bms.BQ_NTC[1]}°C')
            print(f'Temps MCU:\t\t{self.lstate.bms.MCU_NTC[0]} °C, {self.lstate.bms.MCU_NTC[1]}°C')
            print(f'FootForce:\t\t{self.lstate.footForce}')
            print(f'FootForceEst:\t\t{self.lstate.footForceEst}')
            print(f'IMU Temp:\t\t{self.lstate.imu.temperature}')
            print('+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=')
    
    def ReceiveObservation(self, timeout=0.1):
        start = time.time()
        while True:
            data = self.conn.getData()
            if data:
                for paket in data:
                    self.lstate.parseData(paket)
                return self.lstate
            if time.time() - start > timeout:
                return self.lstate
            time.sleep(0.001)
    
    def SendCommand(self, cmd):
        """原始接口，保持兼容"""
        for motor_id, motor_name in enumerate(self.d):
            self.mCmdArr.setMotorCmd(motor_name, motorCmd(
                mode=MotorModeLow.Servo, 
                q=cmd[motor_id*5], 
                dq=cmd[motor_id*5+1], 
                Kp=cmd[motor_id*5+2], 
                Kd=cmd[motor_id*5+3], 
                tau=cmd[motor_id*5+4]
            ))
        self.lcmd.motorCmd = self.mCmdArr
        cmd_bytes = self.lcmd.buildCmd(debug=False)
        self.conn.send(cmd_bytes)
        return True
    
    def SendCommandFast(self, positions, kp, kd, velocities=None, torques=None):
        """
        优化版本：只在参数变化时重建 buffer，否则只更新 q 位置
        
        Args:
            positions: np.ndarray (12,) 目标位置
            kp: float 位置增益
            kd: float 速度增益
            velocities: np.ndarray (12,) 目标速度，默认为0
            torques: np.ndarray (12,) 前馈力矩，默认为0
        """
        has_velocities = velocities is not None
        has_torques = torques is not None
        
        # 检查是否需要重新填充 buffer（参数变化时）
        need_reinit = (
            not self._buffer_initialized or
            kp != self._last_kp or
            kd != self._last_kd or
            has_velocities != self._last_has_velocities or
            has_torques != self._last_has_torques
        )
        
        if need_reinit:
            # 首次调用或参数变化，填充完整 buffer
            kp_bytes = kp_to_hex_fast(kp)
            kd_bytes = kd_to_hex_fast(kd)
            dq_bytes = b'\x00\x00\x00\x00'
            tau_bytes = b'\x00\x00'
            reserve_bytes = b'\x00' * 12
            
            motor_offset = 22
            for i in range(12):
                offset = motor_offset + i * self._motor_cmd_size
                
                # mode (1 byte)
                self._cmd_buffer[offset] = self._servo_mode
                # q 跳过，后面统一填充
                # dq (4 bytes)
                if has_velocities:
                    self._cmd_buffer[offset+5:offset+9] = float_to_hex_fast(velocities[i])
                else:
                    self._cmd_buffer[offset+5:offset+9] = dq_bytes
                # tau (2 bytes)
                if has_torques:
                    self._cmd_buffer[offset+9:offset+11] = tau_to_hex_fast(torques[i])
                else:
                    self._cmd_buffer[offset+9:offset+11] = tau_bytes
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
            self._last_has_velocities = has_velocities
            self._last_has_torques = has_torques
            self._buffer_initialized = True
            # 注意：使用 position 模式后，torque 模式的 buffer 需要重新初始化
            self._torque_buffer_initialized = False
        
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
        return True
    
    def SendTorqueFast(self, torques, pos_stop=2.146e9, vel_stop=16000.0):
        """
        优化版本：只在参数变化时重建 buffer，否则只更新 tau 位置
        
        Args:
            torques: np.ndarray (12,) 力矩
            pos_stop: float PosStopF 值
            vel_stop: float VelStopF 值
        """
        # 检查是否需要重新填充 buffer
        need_reinit = (
            not self._torque_buffer_initialized or
            pos_stop != self._last_pos_stop or
            vel_stop != self._last_vel_stop
        )
        
        if need_reinit:
            # 预计算固定字节
            pos_bytes = float_to_hex_fast(pos_stop)
            vel_bytes = float_to_hex_fast(vel_stop)
            kp_bytes = b'\x00\x00'
            kd_bytes = b'\x00\x00'
            reserve_bytes = b'\x00' * 12
            
            motor_offset = 22
            for i in range(12):
                offset = motor_offset + i * self._motor_cmd_size
                
                # mode (1 byte)
                self._cmd_buffer[offset] = self._servo_mode
                # q (4 bytes) - PosStopF
                self._cmd_buffer[offset+1:offset+5] = pos_bytes
                # dq (4 bytes) - VelStopF
                self._cmd_buffer[offset+5:offset+9] = vel_bytes
                # tau 跳过，后面统一填充
                # Kp (2 bytes) - 0
                self._cmd_buffer[offset+11:offset+13] = kp_bytes
                # Kd (2 bytes) - 0
                self._cmd_buffer[offset+13:offset+15] = kd_bytes
                # reserve (12 bytes)
                self._cmd_buffer[offset+15:offset+27] = reserve_bytes
            
            # 填充剩余电机
            zero_motor = b'\x00' * 27
            for i in range(12, 20):
                offset = motor_offset + i * self._motor_cmd_size
                self._cmd_buffer[offset:offset+27] = zero_motor
            
            # 更新缓存
            self._last_pos_stop = pos_stop
            self._last_vel_stop = vel_stop
            self._torque_buffer_initialized = True
            # 注意：使用 torque 模式后，position 模式的 buffer 需要重新初始化
            self._buffer_initialized = False
        
        # 每次调用只更新 tau 位置（12 个电机 × 2 bytes）
        buf = self._cmd_buffer
        for i in range(12):
            toff = self._tau_offsets[i]
            buf[toff:toff+2] = tau_to_hex_fast(torques[i])
        
        # 计算 CRC
        crc = encryptCrc(genCrc(buf[:-6]))
        buf[-4:] = crc
        
        # 发送
        self.conn.send(buf)
        return True

    # def Brake(self):
