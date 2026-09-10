#  (c) 2024-2025 zh

"""
Xbox手柄控制模块（Pygame版本）
"""

import threading
import time
import pygame
from dataclasses import dataclass
from src.scripts.rl_sdk import STATE


#------------ data----------------
@dataclass
class SpeedProfile:
    """速度档位"""
    x: float
    y: float
    yaw: float


@dataclass
class XboxDeviceMap:
    """手柄状态映射"""
    time: int = 0
    a: float = 0.0
    b: float = 0.0
    x: float = 0.0
    y: float = 0.0
    lb: float = 0.0
    rb: float = 0.0
    start: float = 0.0
    back: float = 0.0
    home: float = 0.0
    lo: float = 0.0
    ro: float = 0.0
    lx: float = 0.0
    ly: float = 0.0
    rx: float = 0.0
    ry: float = 0.0
    lt: float = 0.0
    rt: float = 0.0
    du: float = 0.0
    dd: float = 0.0
    dl: float = 0.0
    dr: float = 0.0


class XboxControl:
    """Xbox手柄控制类 基于Pygame"""

    LOGGER = "[xbox]"
    SLOW_SPEED = SpeedProfile(x=1.2, y=1.5, yaw=2.0)
    FAST_SPEED = SpeedProfile(x=2.5, y=1.5, yaw=2.0)
    
    #------------ setup----------------
    def __init__(self, rl_control, deadzone: float = 0.05):
        self.control = rl_control
        self.deadzone = float(deadzone)
        self.map = XboxDeviceMap()
        self.speed = self.SLOW_SPEED
        
        self.joystick = None
        self.is_running = False
        self._thread = None
        self.enable_print = False
        self._start()
    
    def __del__(self):
        self.stop()
    
    #------------ connection----------------
    def _connect_joystick(self) -> bool:
        """连接第一个可用的手柄"""
        if pygame.joystick.get_count() > 0:
            try:
                self.joystick = pygame.joystick.Joystick(0)
                self.joystick.init()
                name = self.joystick.get_name()
                print(f"{self.LOGGER} Controller connected: {name}")
                return True
            except Exception as e:
                print(f"{self.LOGGER} Failed to connect joystick: {e}")
                self.joystick = None
                return False
        else:
            print(f"{self.LOGGER} No controller found.")
            return False

    #------------ lifecycle----------------
    def _start(self):
        """启动监听线程"""
        self.is_running = True
        self._thread = threading.Thread(target=self._controller_thread_func, daemon=True)
        self._thread.start()
    
    def stop(self):
        """停止手柄监听"""
        self.is_running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
    
    #------------ loop----------------
    def _controller_thread_func(self):
        """手柄监听线程主循环"""
        # 在子线程中初始化 Pygame，确保 Joystick 对象和事件循环在同一线程
        try:
            pygame.init()
            pygame.joystick.init()
        except Exception as e:
            print(f"{self.LOGGER} Pygame init failed: {e}")
            return

        # 尝试连接手柄
        self._connect_joystick()
        
        clock = pygame.time.Clock()
        
        while self.is_running:
            try:
                # 处理事件队列 - 必须在同一个线程调用
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        self.is_running = False
                    
                    # 设备插拔检测
                    elif event.type == pygame.JOYDEVICEADDED:
                        print(f"{self.LOGGER} Joystick added: {event.device_index}")
                        if self.joystick is None:
                            self._connect_joystick()
                    elif event.type == pygame.JOYDEVICEREMOVED:
                        print(f"{self.LOGGER} Joystick removed: {event.instance_id}")
                        if self.joystick and self.joystick.get_instance_id() == event.instance_id:
                            self.joystick.quit()
                            self.joystick = None
                
                # 只有连接了手柄才读取状态
                if self.joystick:
                    self._read_inputs()
                    self._update_command()
                else:
                    # 低频尝试重连
                    if pygame.joystick.get_count() > 0 and self.joystick is None:
                        self._connect_joystick()
                
                # 60Hz轮询频率
                clock.tick(60)
            except Exception as e:
                print(f"{self.LOGGER} Error in thread loop: {e}")
                time.sleep(1)

        # 退出前清理
        pygame.quit()

    #------------ input----------------
    def _read_inputs(self):
        """读取所有输入状态"""
        try:
            # 轴 (Axes)
            num_axes = self.joystick.get_numaxes()
            if num_axes >= 2:
                self.map.lx = self.joystick.get_axis(0)
                self.map.ly = self.joystick.get_axis(1)
            
            if num_axes >= 4:
                self.map.rx = self.joystick.get_axis(2)
                self.map.ry = self.joystick.get_axis(3)
                
            if num_axes >= 6:
                lt_val = self.joystick.get_axis(4)
                rt_val = self.joystick.get_axis(5)
                self.map.lt = (lt_val + 1.0) / 2.0
                self.map.rt = (rt_val + 1.0) / 2.0

            # 按钮 (Buttons)
            num_buttons = self.joystick.get_numbuttons()
            if num_buttons > 0: self.map.a = self.joystick.get_button(0)
            if num_buttons > 1: self.map.b = self.joystick.get_button(1)
            if num_buttons > 2: self.map.x = self.joystick.get_button(2)
            if num_buttons > 3: self.map.y = self.joystick.get_button(3)
            if num_buttons > 4: self.map.lb = self.joystick.get_button(4)
            if num_buttons > 5: self.map.rb = self.joystick.get_button(5)
            if num_buttons > 6: self.map.back = self.joystick.get_button(6)
            if num_buttons > 7: self.map.start = self.joystick.get_button(7)
            if num_buttons > 8: self.map.lo = self.joystick.get_button(8)
            if num_buttons > 9: self.map.ro = self.joystick.get_button(9)
            if num_buttons > 10: self.map.home = self.joystick.get_button(10)

            # 方向键 (Hats / D-Pad)
            num_hats = self.joystick.get_numhats()
            hat = self.joystick.get_hat(0) if num_hats > 0 else (0, 0)
            # hat is tuple (x, y), values -1, 0, 1
            self.map.du = 1.0 if hat[1] > 0 else 0.0
            self.map.dd = 1.0 if hat[1] < 0 else 0.0
            self.map.dl = 1.0 if hat[0] < 0 else 0.0
            self.map.dr = 1.0 if hat[0] > 0 else 0.0
            
            self.map.time = pygame.time.get_ticks()
            
        except Exception as e:
            print(f"{self.LOGGER} Error reading inputs: {e}")

    #------------ command----------------
    def _update_command(self):
        """更新RL控制命令"""
        # X按钮 - 站起
        if self.map.x == 1:
            self.control.control_state = STATE.STATE_POS_GETUP
        
        # B按钮 - 蹲下
        elif self.map.b == 1:
            self.control.control_state = STATE.STATE_POS_GETDOWN

        # 按钮 - 强化学习
        elif self.map.a == 1:
            self.control.control_state = STATE.STATE_RL_RUNNING
            self.control.model_flag = 0

        elif self.map.y == 1:
            self.control.control_state = STATE.STATE_RL_RUNNING
            self.control.model_flag = 1

        elif self.map.lb == 1:
            self.control.control_state = STATE.STATE_RL_RUNNING
            self.control.model_flag = 2

        elif self.map.rb == 1:
            self.control.control_state = STATE.STATE_RL_RUNNING
            self.control.model_flag = 3

        elif self.map.du == 1:
            self.control.control_state = STATE.STATE_RL_RUNNING
            self.control.model_flag = 4

        elif self.map.dr == 1:
            self.control.control_state = STATE.STATE_RL_RUNNING
            self.control.model_flag = 5

        elif self.map.dd == 1:
            self.control.control_state = STATE.STATE_RL_RUNNING
            self.control.model_flag = 6

        elif self.map.dl == 1:
            self.control.control_state = STATE.STATE_RL_RUNNING
            self.control.model_flag = 7

        # Start按钮
        if self.map.start == 1:
            self.speed = self.SLOW_SPEED
        
        # Back按钮
        if self.map.back == 1:
            self.speed = self.FAST_SPEED
        
        # 更新摇杆值
        self.control.x = -self._apply_deadzone(self.map.ly) * self.speed.x
        self.control.y = -self._apply_deadzone(self.map.lx) * self.speed.y
        self.control.yaw = -self._apply_deadzone(self.map.rx) * self.speed.yaw
        
        if self.enable_print:
            self.print_map()

    #------------ utils----------------
    def _apply_deadzone(self, x: float) -> float:
        """应用死区"""
        if abs(x) < self.deadzone:
            return 0.0
        sign = 1.0 if x > 0 else -1.0
        adjusted = (abs(x) - self.deadzone) / (1.0 - self.deadzone)
        return sign * adjusted

    #------------ debug----------------
    def print_map(self):
        """打印映射状态"""
        print("-" * 30)
        print(f"Time: {self.map.time}")
        print(f"Axes: LX={self.map.lx:.2f} LY={self.map.ly:.2f} RX={self.map.rx:.2f} RY={self.map.ry:.2f}")
        print(f"Trig: LT={self.map.lt:.2f} RT={self.map.rt:.2f}")
        print(f"Btns: A={self.map.a} B={self.map.b} X={self.map.x} Y={self.map.y}")
        print(f"Shld: LB={self.map.lb} RB={self.map.rb}")
        print(f"Misc: Start={self.map.start} Back={self.map.back} Home={self.map.home}")
        print(f"Stck: LO={self.map.lo} RO={self.map.ro}")
        print(f"DPad: U={self.map.du} D={self.map.dd} L={self.map.dl} R={self.map.dr}")
        print("-" * 30)
