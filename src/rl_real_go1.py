#  (c) 2024-2025 zh
# Real Go1 robot controller

import time
from src.scripts.rl_data import LOGGER
from src.interface.IOReal_go1 import IOReal_go1
from src.interface.depth.ros_depth_subscriber import ROSDepthSubscriber
from src.scripts.rl_deploy import RLDeploy
from src.input_dev.xbox_control import XboxControl
from src.input_dev.keyboard_control import KeyboardControl
from src.input_dev.phone_web_control import PhoneWebControl


class RealRobot_go1:
    """Real Go1 robot controller."""
    
    def __init__(self, policy_names: list[str], control_dt: float = 0.002):
        self.policy_names = policy_names
        self.control_dt = control_dt

        # RL deploy
        self.rl = RLDeploy(self.policy_names)
        self.depth_subscriber = ROSDepthSubscriber(self.rl.robot_state.depth)

        # IO for real robot
        self.io = IOReal_go1()
        
        # Inputs
        # self.keyboard_ctrl = KeyboardControl(self.rl.robot_state.control)
        self.xbox_ctrl = XboxControl(self.rl.robot_state.control)
        # self.phone_ctrl = PhoneWebControl(self.rl.robot_state.control)

    def run(self):
        """Main loop: single-thread."""
        next_exec_time = time.perf_counter()
        thread_period = self.control_dt
        # Keep inference at 50Hz
        inference_times = int(1.0 / 50.0 / self.control_dt)
        print(f"{LOGGER.INFO}control_hz={1/self.control_dt:.1f}hz, inference_times={inference_times}times")
        
        try:
            while True:
                # 1) Read robot state
                self.io.recv(self.rl.temp_robot_state)
                
                # 2) Run control logic
                self.rl.step(inference_times)

                # 3) Send commands
                self.io.send(self.rl.temp_robot_command)
                
                # 4) Precise control timing
                next_exec_time += thread_period
                now = time.perf_counter()
                sleep_time = next_exec_time - now
                
                if sleep_time > 0:
                    time.sleep(sleep_time)
                elif sleep_time < -thread_period:
                    next_exec_time = now

        finally:
            self.depth_subscriber.close()
            print(f"{LOGGER.INFO}Real robot controller shutdown complete")


def main():
    """Main entry."""
    # sudo chrt -f 99 su -c 'python3 /home/zh/go1_deploy/go1_deploy_ros2/src/rl_real_go1.py' zh
    
    # Quadruped Robot
    policy_names = [
        # "issacgym/go1/himloco",
        # "issacgym/go1/np3o",
        "issacgym/go1/go1wmp",
        "issacgym/go1/moe",
        # "issacgym/go1/go1",
        "issacgym/go1/recoveryloco",
        # "unitree_rl_lab/go2/go2_loco",
        # "unitree_rl_lab/go2/go2_back_filp",
        # "unitree_rl_lab/go2/go2_silde_filp",
        # "unitree_rl_lab/go2/go2_jump",
    ]
    
    controller = RealRobot_go1(policy_names, control_dt=0.001)
    controller.run()

if __name__ == "__main__":
    main()
