#  (c) 2024-2025 zh


"""
Xbox手柄控制模块（SDL2版本）
"""

import threading
import sdl2
from enum import Enum
from dataclasses import dataclass
from src.scripts.rl_sdk import STATE


#------------ data----------------
class ControllerType(Enum):
    """手柄类型枚举"""
    UNKNOWN = 0
    STANDARD = 1
    BEI_TONG = 2


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
    """Xbox手柄控制类"""

    LOGGER = "[xbox]"
    SLOW_SPEED = SpeedProfile(x=1.2, y=1.5, yaw=2.0)
    FAST_SPEED = SpeedProfile(x=2.5, y=1.5, yaw=2.0)
    
    #------------ setup----------------
    def __init__(self, rl_control, deadzone: float = 0.05):
        self.control = rl_control
        self.deadzone = float(deadzone)
        self.map = XboxDeviceMap()
        self.speed = self.SLOW_SPEED
        
        self.controller = None
        self.controller_instance_id = -1
        self.is_running = False
        self.controller_type = ControllerType.UNKNOWN
        self.enable_print = False
        
        self._thread = None
        
        if not self._init_sdl():
            raise RuntimeError("Failed to init SDL2 GameController")
        
        self._start()
    
    def __del__(self):
        self.stop()
    
    #------------ connection----------------
    def _init_sdl(self) -> bool:
        """初始化SDL2"""
        if sdl2.SDL_Init(sdl2.SDL_INIT_GAMECONTROLLER) < 0:
            err_msg = sdl2.SDL_GetError()
            err_str = err_msg.decode("utf-8", "ignore") if err_msg else "Unknown"
            print(f"{self.LOGGER} SDL init failed: {err_str}")
            return False
        
        # 打开第一个可用的手柄
        n = sdl2.SDL_NumJoysticks()
        for i in range(n):
            if sdl2.SDL_IsGameController(i):
                c = sdl2.SDL_GameControllerOpen(i)
                if c:
                    self.controller = c
                    joy = sdl2.SDL_GameControllerGetJoystick(c)
                    self.controller_instance_id = int(sdl2.SDL_JoystickInstanceID(joy))
                    name = sdl2.SDL_GameControllerName(c)
                    name_str = name.decode() if name else "Unknown"
                    print(f"{self.LOGGER} Controller connected: {name_str}")
                    self._detect_controller_type()
                    return True
        
        print(f"{self.LOGGER} No controller found, waiting for connection...")
        return True
    
    def _detect_controller_type(self):
        """检测手柄类型"""
        if not self.controller:
            return
        name = sdl2.SDL_GameControllerName(self.controller)
        controller_name = (name.decode("utf-8", "ignore") if name else "").lower()
        
        if ("bei tong" in controller_name) or ("beitong" in controller_name):
            self.controller_type = ControllerType.BEI_TONG
            print(f"{self.LOGGER} Detected BEI TONG")
        else:
            self.controller_type = ControllerType.STANDARD
            print(f"{self.LOGGER} Detected STANDARD")
    
    #------------ lifecycle----------------
    def _start(self):
        """启动监听线程"""
        self.is_running = True
        self._thread = threading.Thread(target=self._controller_thread_func, daemon=True)
        self._thread.start()
    
    def stop(self):
        """停止手柄监听"""
        if not self.is_running:
            return
        
        self.is_running = False
        
        # 推送退出事件以唤醒等待循环
        try:
            quit_ev = sdl2.SDL_Event()
            quit_ev.type = sdl2.SDL_QUIT
            sdl2.SDL_PushEvent(quit_ev)
        except Exception:
            pass
        
        # 等待线程退出
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        
        # 关闭手柄
        if self.controller:
            try:
                sdl2.SDL_GameControllerClose(self.controller)
            except Exception:
                pass
            self.controller = None

        # 线程安全的SDL清理
        try:
            sdl2.SDL_Quit()
        except Exception:
            pass
    
    #------------ loop----------------
    def _controller_thread_func(self):
        """手柄监听线程主循环"""
        ev = sdl2.SDL_Event()
        while self.is_running:
            # 等待事件（超时100ms）
            has_event = sdl2.SDL_WaitEventTimeout(ev, 100)
            
            if has_event and self.is_running:
                self._process_event(ev)
                
                # 处理队列中剩余的事件
                while sdl2.SDL_PollEvent(ev) and self.is_running:
                    self._process_event(ev)
                
                # 每批事件后更新命令
                self._update_command()

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
        self.control.x = self._apply_deadzone(self.map.ly) * self.speed.x
        self.control.y = -self._apply_deadzone(self.map.lx) * self.speed.y
        self.control.yaw = -self._apply_deadzone(self.map.rx) * self.speed.yaw

        if self.enable_print:
            self.print_map()
    
    #------------ events----------------
    def _process_event(self, ev: sdl2.SDL_Event):
        """处理SDL事件"""
        et = ev.type
        
        if et in (sdl2.SDL_CONTROLLERBUTTONDOWN, sdl2.SDL_CONTROLLERBUTTONUP):
            if ev.cbutton.which == self.controller_instance_id:
                button = ev.cbutton.button
                pressed = (et == sdl2.SDL_CONTROLLERBUTTONDOWN)
                self._update_button(button, pressed)
        
        elif et == sdl2.SDL_CONTROLLERAXISMOTION:
            if ev.caxis.which == self.controller_instance_id:
                axis = ev.caxis.axis
                value = ev.caxis.value
                self._update_axis(axis, value)
        
        elif et == sdl2.SDL_CONTROLLERDEVICEADDED:
            # 手柄插入
            if self.controller is None and sdl2.SDL_IsGameController(ev.cdevice.which):
                c = sdl2.SDL_GameControllerOpen(ev.cdevice.which)
                if c:
                    self.controller = c
                    joy = sdl2.SDL_GameControllerGetJoystick(c)
                    self.controller_instance_id = int(sdl2.SDL_JoystickInstanceID(joy))
                    name = sdl2.SDL_GameControllerName(c)
                    name_str = name.decode() if name else "Unknown"
                    print(f"{self.LOGGER} Controller connected: {name_str}")
                    self._detect_controller_type()
        
        elif et == sdl2.SDL_CONTROLLERDEVICEREMOVED:
            # 手柄移除
            if ev.cdevice.which == self.controller_instance_id:
                sdl2.SDL_GameControllerClose(self.controller)
                self.controller = None
                self.controller_instance_id = -1
                self._reset_map()
                print(f"{self.LOGGER} Controller disconnected, waiting reconnection...")
        
        self.map.time = int(sdl2.SDL_GetTicks())
    
    #------------ input----------------
    def _reset_map(self):
        """重置手柄状态"""
        self.map = XboxDeviceMap()
    
    def _update_button(self, button: int, pressed: bool):
        """更新按钮状态"""
        v = 1.0 if pressed else 0.0
        
        if button == sdl2.SDL_CONTROLLER_BUTTON_A:
            self.map.a = v
        elif button == sdl2.SDL_CONTROLLER_BUTTON_B:
            self.map.b = v
        elif button == sdl2.SDL_CONTROLLER_BUTTON_X:
            self.map.x = v
        elif button == sdl2.SDL_CONTROLLER_BUTTON_Y:
            self.map.y = v
        elif button == sdl2.SDL_CONTROLLER_BUTTON_LEFTSHOULDER:
            self.map.lb = v
        elif button == sdl2.SDL_CONTROLLER_BUTTON_RIGHTSHOULDER:
            self.map.rb = v
        elif button == sdl2.SDL_CONTROLLER_BUTTON_START:
            self.map.start = v
        elif button == sdl2.SDL_CONTROLLER_BUTTON_BACK:
            self.map.back = v
        elif button == sdl2.SDL_CONTROLLER_BUTTON_GUIDE:
            self.map.home = v
        elif button == sdl2.SDL_CONTROLLER_BUTTON_LEFTSTICK:
            self.map.lo = v
        elif button == sdl2.SDL_CONTROLLER_BUTTON_RIGHTSTICK:
            self.map.ro = v
        elif button == sdl2.SDL_CONTROLLER_BUTTON_DPAD_UP:
            self.map.du = v
        elif button == sdl2.SDL_CONTROLLER_BUTTON_DPAD_DOWN:
            self.map.dd = v
        elif button == sdl2.SDL_CONTROLLER_BUTTON_DPAD_LEFT:
            self.map.dl = v
        elif button == sdl2.SDL_CONTROLLER_BUTTON_DPAD_RIGHT:
            self.map.dr = v
    
    def _update_axis(self, axis: int, value: int):
        """更新摇杆/扳机轴状态"""
        if axis == sdl2.SDL_CONTROLLER_AXIS_LEFTX:
            self.map.lx = self._normalize_axis(value)
        elif axis == sdl2.SDL_CONTROLLER_AXIS_LEFTY:
            self.map.ly = -self._normalize_axis(value)
        elif axis == sdl2.SDL_CONTROLLER_AXIS_RIGHTX:
            self.map.rx = self._normalize_axis(value)
        elif axis == sdl2.SDL_CONTROLLER_AXIS_RIGHTY:
            self.map.ry = -self._normalize_axis(value)
        
        elif axis == sdl2.SDL_CONTROLLER_AXIS_TRIGGERLEFT:
            self.map.lt = self._normalize_axis(value)
            # BeiTong特殊映射：LT -> rx
            if self.controller_type == ControllerType.BEI_TONG:
                self.map.rx = (self.map.lt - 0.5) * 2.0
                self.map.lt = 0.0
        
        elif axis == sdl2.SDL_CONTROLLER_AXIS_TRIGGERRIGHT:
            self.map.rt = self._normalize_axis(value)
            # BeiTong特殊映射：RT -> lx
            # if self.controller_type == ControllerType.BEI_TONG:
            #     self.map.lx = (self.map.rt - 0.5) * 2.0
            #     self.map.rt = 0.0

    #------------ utils----------------
    def _normalize_axis(self, value: int) -> float:
        """归一化轴值"""
        return float(value) / 32767.0
    
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
