#  (c) 2024-2025 zh
# MuJoCo viewer simulation controller

import time
import platform
import mujoco
import mujoco.viewer
from dm_control import mjcf
from utils.path_config import MJCF_ROOT
from src.scripts.rl_sdk import LOGGER
from src.interface.IOMuJoCo import IOMuJoCo
from src.scripts.rl_deploy import RLDeploy
from src.input_dev.keyboard_control import KeyboardControl
from src.input_dev.phone_web_control import PhoneWebControl

# Select Xbox input module by OS
if platform.system() == "Linux":
    from src.input_dev.xbox_control import XboxControl
else:  # Windows
    from src.input_dev.xbox_control_pygame import XboxControl


class MuJoCoRobot:
    """MuJoCo simulation."""
    
    def __init__(self, policy_names: list[str], robot_xml: str, terrain_xml: str, control_dt: float = 0.002):
        self.policy_names = policy_names
        self.control_dt = control_dt
        self.viewer_counter = 0

        self.mj_model, self.mj_data, base_link_name = self._load_model(robot_xml, terrain_xml)
        self.mj_model.opt.timestep = self.control_dt
        self.mj_viewer = mujoco.viewer.launch_passive(self.mj_model, self.mj_data)
        self.trunk_id = mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_BODY, base_link_name)
        
        # RL deploy
        self.rl = RLDeploy(self.policy_names)

        # IO for MuJoCo simulation
        self.io = IOMuJoCo(self.mj_model, self.mj_data, self.rl.params.num_of_dofs)
        
        # Inputs
        self.keyboard_ctrl = KeyboardControl(self.rl.control)
        self.xbox_ctrl = XboxControl(self.rl.control)
        self.phone_ctrl = PhoneWebControl(self.rl.control)
    
    def _load_model(self, robot_xml: str, terrain_xml: str):
        """Load terrain and robot, then merge into one model."""
        terrain_mjcf = mjcf.from_path(terrain_xml)
        robot_mjcf = mjcf.from_path(robot_xml)

        # Remove robot's built-in floating root joint, then attach a new root freejoint.
        for jnt in robot_mjcf.find_all("joint"):
            if jnt.tag == "freejoint" or getattr(jnt, "type", None) == "free":
                jnt.remove()

        attachment_frame = terrain_mjcf.worldbody.attach(robot_mjcf)
        attachment_frame.add("freejoint", name="root")

        root_bodies = robot_mjcf.worldbody.find_all("body")
        if not root_bodies:
            raise ValueError(f"No root body found in robot xml: {robot_xml}")
        base_link_name = f"{robot_mjcf.model}/{root_bodies[0].name}"
        # print(f"base_link_name = {base_link_name}")
        
        physics = mjcf.Physics.from_mjcf_model(terrain_mjcf)
        return physics.model.ptr, physics.data.ptr, base_link_name
            
    def run(self):
        """Main loop: single-thread."""
        next_exec_time = time.perf_counter()
        thread_period = self.control_dt
        # Keep inference at 50Hz
        inference_times = int(1.0 / 50.0 / self.control_dt)
        print(f"{LOGGER.INFO}control_hz={1/self.control_dt:.1f}hz, inference_times={inference_times}times")
        
        self.mj_viewer.cam.distance = 5
        self.mj_viewer.cam.elevation = -20
        self.mj_viewer.cam.azimuth = 45
        try:
            while self.mj_viewer.is_running():
                # 1) Read sim state
                self.io.recv(self.rl.temp_robot_state)

                # 2) Run control logic
                self.rl.step(inference_times)

                # 3) Send commands
                self.io.send(self.rl.temp_robot_command)

                # 4) Physics step
                mujoco.mj_step(self.mj_model, self.mj_data)

                # 5) Update viewer
                self.viewer_counter += 1
                if self.viewer_counter >= 5:
                    self.mj_viewer.cam.lookat[:] = self.mj_data.xpos[self.trunk_id]
                    self.mj_viewer.sync()
                    self.viewer_counter = 0

                # 6) Precise control timing
                next_exec_time += thread_period
                now = time.perf_counter()
                sleep_time = next_exec_time - now
                
                if sleep_time > 0:
                    time.sleep(sleep_time)
                elif sleep_time < -thread_period:
                    next_exec_time = now

        finally:
            print(f"{LOGGER.INFO}MuJoCo controller shutdown complete")


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
    # robot_xml = str(MJCF_ROOT / "go1" / "go1.xml")
    # robot_xml = str(MJCF_ROOT / "go2" / "go2.xml")

    # Two wheel legged Robot
    # policy_names = [
    #     "issacgym/duow/duow",
    # ]
    # robot_xml = str(MJCF_ROOT / "duow" / "duow.xml")

    # Four wheel legged Robot
    # policy_names = [
    #     "issacgym/go2w/go2w_himloco",
    #     "issacgym/M20/M20_lab",
    #     "issacgym/M20/M20",
    # ]
    # robot_xml = str(MJCF_ROOT / "go2w" / "go2w.xml")
    # robot_xml = str(MJCF_ROOT / "M20" / "M20.xml")

    # Humanoid Robot
    policy_names = [
        "unitree_rl_lab/g1/g1_amp",
        "unitree_rl_lab/g1/g1_loco",
        "unitree_rl_lab/g1/dance_102",
        "unitree_rl_lab/g1/gangnam_style",
        "unitree_rl_lab/g1/dance1_subject2",
    ]
    robot_xml = str(MJCF_ROOT / "g1" / "g1_29dof.xml")

    # Terrain model
    terrain_xml = str(MJCF_ROOT / "terrains" / "empty_world.xml")
    # terrain_xml = str(MJCF_ROOT / "terrains" / "race_track.xml")

    controller = MuJoCoRobot(policy_names, robot_xml, terrain_xml)
    controller.run()

if __name__ == "__main__":
    main()
