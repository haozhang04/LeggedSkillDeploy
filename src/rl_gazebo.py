#  (c) 2024-2025 zh
# Gazebo ROS2 simulation controller

import time
import subprocess
from utils.path_config import URDF_WS
from src.scripts.rl_sdk import LOGGER, STATE
from src.interface.IOGazebo import IOGazebo
from src.scripts.rl_deploy import RLDeploy
from src.input_dev.xbox_control import XboxControl
from src.input_dev.keyboard_control import KeyboardControl
from src.input_dev.phone_web_control import PhoneWebControl

class GazeboRobot:
    """Gazebo ROS2 simulation."""

    def __init__(self, policy_names: list[str], robot_name: str, control_dt: float = 0.002):
        self.policy_names = policy_names
        self.robot_name = robot_name
        self.control_dt = control_dt
        self.gazebo_process = None
        self.rviz_process = None

        # Gazebo simulation
        self._start_gazebo()

        # RViz visualization
        self._start_rviz()

        # RL deploy
        self.rl = RLDeploy(self.policy_names)

        # IO for Gazebo simulation
        self.io = IOGazebo(self.rl.params.num_of_dofs, self.rl.params.joint_controller_names)

        # Inputs
        self.keyboard_ctrl = KeyboardControl(self.rl.control)
        self.xbox_ctrl = XboxControl(self.rl.control)
        self.phone_ctrl = PhoneWebControl(self.rl.control)

    def _start_gazebo(self):
        """Start Gazebo in a new terminal."""
        setup_cmd = f"source {URDF_WS}/install/setup.zsh"
        launch_cmd = f"ros2 launch {self.robot_name}_description gazebo.launch.py"
        full_cmd = f"{setup_cmd} && {launch_cmd}"
        print(full_cmd)

        self.gazebo_process = subprocess.Popen(
            ["gnome-terminal", "--", "zsh", "-c", full_cmd],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL)
        print(f"{LOGGER.INFO}Gazebo started in new terminal")
        time.sleep(1)

    def _start_rviz(self):
        """Start RViz in a new terminal."""
        setup_cmd = f"source {URDF_WS}/install/setup.zsh"
        rviz_config = f"{URDF_WS}/src/robot_common/rviz/check_joint.rviz"
        launch_cmd = f"rviz2 -d {rviz_config}"
        full_cmd = f"{setup_cmd} && {launch_cmd}"

        self.rviz_process = subprocess.Popen(
            ["gnome-terminal", "--", "zsh", "-c", full_cmd],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL)
        print(f"{LOGGER.INFO}RViz started with config: {rviz_config}")
        time.sleep(1)

    def run(self):
        """Main loop: single-thread control."""
        next_exec_time = time.perf_counter()
        thread_period = self.control_dt
        # Keep inference at 50Hz
        inference_times = int(1.0 / 50.0 / self.control_dt)
        print(f"{LOGGER.INFO}control_hz={1/self.control_dt:.1f}hz, inference_times={inference_times}times")

        try:
            while True:
                # 1) Read sim state
                self.io.recv(self.rl.temp_robot_state)

                # 2) Run control logic
                self.rl.step(inference_times)

                # 3) Send commands
                self.io.send(self.rl.temp_robot_command)

                # 4) Handle simulation service requests
                self._reset_simulation()

                # 5) Precise control timing
                next_exec_time += thread_period
                now = time.perf_counter()
                sleep_time = next_exec_time - now

                if sleep_time > 0:
                    time.sleep(sleep_time)
                elif sleep_time < -thread_period:
                    next_exec_time = now

        finally:
            self.shutdown()

    def _reset_simulation(self):
        """Reset simulation when requested."""
        if self.rl.control.control_state != STATE.STATE_RESET_SIMULATION:
            return
        self.rl.control.control_state = STATE.STATE_WAITING
        self.io.reset_simulation()

    def shutdown(self):
        """Shutdown controller resources."""
        self.io.shutdown()
        if self.rviz_process:
            self.rviz_process.terminate()
            print(f"{LOGGER.INFO}RViz terminated")
        if self.gazebo_process:
            self.gazebo_process.terminate()
            print(f"{LOGGER.INFO}Gazebo terminated")
        print(f"{LOGGER.INFO}Gazebo controller shutdown complete")


def main():
    """Main entry."""
    # Quadruped Robot
    # policy_names = [
    #     # "issacgym/go1/himloco",
    #     # "issacgym/go1/np3o",
    #     # "issacgym/go1/moe",
    #     "issacgym/go1/go1",
    #     # "issacgym/go1/recoveryloco",
    #     # "unitree_rl_lab/go2/go2_loco",
    #     "unitree_rl_lab/go2/go2_back_filp",
    #     "unitree_rl_lab/go2/go2_silde_filp",
    #     "unitree_rl_lab/go2/go2_jump",
    # ]

    # Two wheel legged Robot
    policy_names = [
        "issacgym/duow/duow",
    ]

    # Four wheel legged Robot
    # policy_names = [
    #     "issacgym/go2w/go2w_himloco",
    #     "issacgym/M20/M20_lab",
    #     "issacgym/M20/M20",
    # ]

    # Humanoid Robot
    # policy_names = [
    #     "unitree_rl_lab/g1/g1_amp",
    #     "unitree_rl_lab/g1/g1_loco",
    #     "unitree_rl_lab/g1/dance_102",
    #     "unitree_rl_lab/g1/gangnam_style",
    #     "unitree_rl_lab/g1/dance1_subject2",
    # ]

    # Robot name: go1, go2, duow, go2w, m20, g1
    robot_name = "duow"

    controller = GazeboRobot(policy_names, robot_name)
    controller.run()

if __name__ == "__main__":
    main()
