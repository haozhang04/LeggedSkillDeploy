#  (c) 2024-2025 zh

"""
键盘控制模块
"""

from pynput import keyboard
from src.scripts.rl_sdk import STATE


class KeyboardControl:
    LOGGER = "[kbd]"

    def __init__(self, rl_control):
        self.control = rl_control
        self.listener = None
        self._start_listener()
    
    def __del__(self):
        self.stop()
        
    def on_press(self, key):
        """键盘按下事件处理"""
        #  空格键
        if key == keyboard.Key.space:
            self.control.x = 0
            self.control.y = 0
            self.control.yaw = 0
            return
        elif key == keyboard.Key.backspace:
            self.control.control_state = STATE.STATE_RESET_SIMULATION
            return
        
        # 方向键：切换模型
        if key == keyboard.Key.up:
            self.control.model_flag += 1
            return
        elif key == keyboard.Key.down:
            self.control.model_flag -= 1
            if self.control.model_flag < 0:
                self.control.model_flag = 0
            return
        
        if hasattr(key, 'char') and key.char:
            # 前进/后退控制
            if key.char == 'w':
                self.control.x += 0.1
            elif key.char == 's':
                self.control.x -= 0.1
            
            # 左右平移控制
            elif key.char == 'a':
                self.control.y += 0.1
            elif key.char == 'd':
                self.control.y -= 0.1

            # 旋转控制
            elif key.char == 'q':
                self.control.yaw += 0.1
            elif key.char == 'e':
                self.control.yaw -= 0.1
            
            # 状态控制
            elif key.char == '2':
                self.control.control_state = STATE.STATE_POS_GETUP
            elif key.char == '4':
                self.control.control_state = STATE.STATE_RL_RUNNING
            elif key.char == '3':
                self.control.control_state = STATE.STATE_POS_GETDOWN

            # 暂停仿真
            elif key.char == 'p':
                self.control.control_state = STATE.STATE_TOGGLE_SIMULATION
    
    def _start_listener(self):
        """启动键盘监听"""
        self.listener = keyboard.Listener(on_press=self.on_press)
        self.listener.start()
        print(f"{self.LOGGER} Keyboard Start")
        # print("控制说明:")
        # print("  W/S: 前进/后退")
        # print("  A/D: 左转/右转")
        # print("  Q/E: 左移/右移")
        # print("  空格: 复位所有速度")
        # print("  2: 站起, 4: 初始化RL, 3: 趴下")
        # print("  R: 重置仿真")
    
    def stop(self):
        """停止键盘监听"""
        if self.listener:
            self.listener.stop()
            print(f"{self.LOGGER} Keyboard Stop")
