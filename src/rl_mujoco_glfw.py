#  (c) 2024-2025 zh
# Unified MuJoCo controller with custom GLFW rendering loop and text overlay

import time
import glfw
import platform
import mujoco
import numpy as np
from dm_control import mjcf
from utils.path_config import MJCF_ROOT
from src.scripts.rl_sdk import LOGGER, STATE
from src.interface.IOMuJoCo import IOMuJoCo
from src.scripts.rl_deploy import RLDeploy
from src.input_dev.keyboard_control import KeyboardControl
from src.input_dev.phone_web_control import PhoneWebControl
from src.utils.mujoco_video_recorder import MuJoCoVideoRecorder

# Select Xbox input module by OS
if platform.system() == "Linux":
    from src.input_dev.xbox_control import XboxControl
else:  # Windows
    from src.input_dev.xbox_control_pygame import XboxControl


class MuJoCoGLFWRobot:
    """MuJoCo GLFW simulation."""
    WINDOW_TITLE = "MuJoCo GLFW"
    WINDOW_WIDTH = 1400
    WINDOW_HEIGHT = 900

    # ---------- initialization ----------
    def __init__(self, policy_names: list[str], robot_xml: str, terrain_xml: str, control_dt: float = 0.002, render_dt: float = 0.02):
        self.policy_names = policy_names
        self.control_dt = control_dt
        self.render_dt = render_dt
        self.paused = False

        # Runtime stats
        self._control_counter = 0
        self._render_counter = 0
        self._control_hz = 0.0
        self._render_fps = 0.0
        self._stats_time = time.perf_counter()

        self.mj_model, self.mj_data, base_link_name = self._load_model(robot_xml, terrain_xml)
        self.mj_model.opt.timestep = self.control_dt
        self.trunk_id = mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_BODY, base_link_name)
        self.video_recorder = MuJoCoVideoRecorder(self.mj_model)

        self.window = None
        self.scene = None
        self.context = None
        self.cam = mujoco.MjvCamera()
        self.opt = mujoco.MjvOption()
        self.trunk_kinematics = {}
        self._button_left = False
        self._button_middle = False
        self._button_right = False
        self._last_x = 0.0
        self._last_y = 0.0
        self._init_glfw(self.WINDOW_TITLE, self.WINDOW_WIDTH, self.WINDOW_HEIGHT)
        self._init_mujoco_vis()

        # RL deploy
        self.rl = RLDeploy(self.policy_names)

        # IO for MuJoCo simulation
        self.io = IOMuJoCo(self.mj_model, self.mj_data, self.rl.params.num_of_dofs)

        # Inputs
        self.keyboard_ctrl = KeyboardControl(self.rl.control)
        self.xbox_ctrl = XboxControl(self.rl.control)
        self.phone_ctrl = PhoneWebControl(self.rl.control)

    def _init_glfw(self, title, width, height):
        if not glfw.init():
            raise RuntimeError("glfw.init() failed")

        glfw.window_hint(glfw.VISIBLE, glfw.TRUE)
        glfw.window_hint(glfw.DOUBLEBUFFER, glfw.TRUE)
        self.window = glfw.create_window(width, height, title, None, None)
        if self.window is None:
            glfw.terminate()
            raise RuntimeError("glfw.create_window() failed")

        glfw.make_context_current(self.window)
        # Disable vsync to prevent blocking the control loop.
        glfw.swap_interval(0)

        glfw.set_key_callback(self.window, self._cb_key)
        glfw.set_mouse_button_callback(self.window, self._cb_mouse_button)
        glfw.set_cursor_pos_callback(self.window, self._cb_cursor_pos)
        glfw.set_scroll_callback(self.window, self._cb_scroll)

    def _init_mujoco_vis(self):
        self.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        self.cam.distance = 5.0
        self.cam.elevation = -20.0
        self.cam.azimuth = 45.0
        self.cam.lookat[:] = [0.0, 0.0, 0.3]

        self.scene = mujoco.MjvScene(self.mj_model, maxgeom=20000)
        self.context = mujoco.MjrContext(self.mj_model, mujoco.mjtFontScale.mjFONTSCALE_150)

    # ---------- callbacks ----------
    def _cb_key(self, window, key, scancode, action, mods):
        del window, scancode, mods
        if action != glfw.PRESS:
            return

        if key == glfw.KEY_ESCAPE:
            glfw.set_window_should_close(self.window, True)
        elif key == glfw.KEY_R:
            self.video_recorder.toggle()

    def _cb_mouse_button(self, window, button, action, mods):
        del button, mods
        self._button_left = glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS
        self._button_middle = glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_MIDDLE) == glfw.PRESS
        self._button_right = glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS
        if action == glfw.PRESS:
            self._last_x, self._last_y = glfw.get_cursor_pos(window)

    def _cb_cursor_pos(self, window, xpos, ypos):
        if not (self._button_left or self._button_middle or self._button_right):
            self._last_x, self._last_y = xpos, ypos
            return

        _, height = glfw.get_window_size(window)
        if height <= 0:
            return

        dx = xpos - self._last_x
        dy = ypos - self._last_y
        self._last_x, self._last_y = xpos, ypos

        if self._button_left:
            action = mujoco.mjtMouse.mjMOUSE_ROTATE_V
        else:
            action = mujoco.mjtMouse.mjMOUSE_ZOOM

        mujoco.mjv_moveCamera(self.mj_model, action, dx / float(height), dy / float(height), self.scene, self.cam)

    def _cb_scroll(self, window, xoffset, yoffset):
        del window, xoffset
        mujoco.mjv_moveCamera(self.mj_model, mujoco.mjtMouse.mjMOUSE_ZOOM, 0.0, -0.05 * yoffset, self.scene, self.cam)

    # ---------- render/window ----------
    def _render(self, text=""):
        glfw.make_context_current(self.window)
        width, height = glfw.get_framebuffer_size(self.window)
        viewport = mujoco.MjrRect(0, 0, width, height)

        mujoco.mjv_updateScene(self.mj_model, self.mj_data, self.opt, None, self.cam, mujoco.mjtCatBit.mjCAT_ALL.value, self.scene)
        self._append_trunk_velocity_viz(self.scene)
        mujoco.mjr_render(viewport, self.scene, self.context)

        if text:
            mujoco.mjr_overlay(mujoco.mjtFont.mjFONT_NORMAL, mujoco.mjtGridPos.mjGRID_TOPLEFT, viewport, text, "", self.context)

        self.video_recorder.capture_frame(self.scene, self.context, self.mj_data.time, text)

        glfw.swap_buffers(self.window)
        glfw.poll_events()

    def _append_trunk_velocity_viz(self, scene):
        """Draw linear and angular velocity arrows above the trunk."""
        if scene.ngeom >= scene.maxgeom - 1:
            return

        anchor_body_z = 0.22  # Lift arrow origin along body z to avoid overlap.
        arrow_width = 0.012

        anchor = self.trunk_kinematics["pos"] + anchor_body_z * self.trunk_kinematics["rot"][:, 2]
        velocity_specs = [(self.trunk_kinematics["lin_vel_world"], 0.2, (0.15, 0.95, 0.25, 0.9)),
                          (self.trunk_kinematics["ang_vel_world"], 0.06, (0.95, 0.55, 0.12, 0.85))]

        for vec, scale, color in velocity_specs:
            norm = float(np.linalg.norm(vec))
            if norm <= 1e-6:
                continue

            direction = vec / norm
            length = norm * scale

            geom = scene.geoms[scene.ngeom]
            mujoco.mjv_initGeom(
                geom,
                int(mujoco.mjtGeom.mjGEOM_ARROW),
                np.zeros(3, dtype=np.float64),
                np.zeros(3, dtype=np.float64),
                np.eye(3, dtype=np.float64).reshape(-1),
                np.asarray(color, dtype=np.float32),
            )
            mujoco.mjv_connector(
                geom,
                int(mujoco.mjtGeom.mjGEOM_ARROW),
                arrow_width,
                anchor,
                anchor + direction * length,
            )
            scene.ngeom += 1

    def _update_trunk_kinematics(self):
        """Update cached trunk pose and velocity terms"""
        pos = np.asarray(self.mj_data.xpos[self.trunk_id], dtype=np.float64)
        rot = np.asarray(self.mj_data.xmat[self.trunk_id], dtype=np.float64).reshape(3, 3)
        cvel = np.asarray(self.mj_data.cvel[self.trunk_id], dtype=np.float64)
        roll = np.arctan2(rot[2, 1], rot[2, 2])
        pitch = np.arctan2(-rot[2, 0], np.hypot(rot[2, 1], rot[2, 2]))
        yaw = np.arctan2(rot[1, 0], rot[0, 0])

        ang_vel_world = cvel[0:3].copy()
        lin_vel_world = cvel[3:6].copy()
        lin_vel_body = rot.T @ lin_vel_world

        self.trunk_kinematics = {"pos": pos,
                                 "rot": rot,
                                 "rpy": np.rad2deg([roll, pitch, yaw]),
                                 "ang_vel_world": ang_vel_world,
                                 "lin_vel_world": lin_vel_world,
                                 "lin_vel_body": lin_vel_body}

    def _close_window(self):
        self.context.free()
        glfw.destroy_window(self.window)
        glfw.terminate()

    # ---------- model/control ----------

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

    def _overlay_text(self):
        """Compose aligned overlay text shown at top-left."""
        cmd = self.rl.control
        state_name = cmd.control_state.name.replace("STATE_", "")
        names = self.rl.model_names
        mf = cmd.model_flag
        model_name = names[mf] if names and 0 <= mf < len(names) else "?"

        px, py, pz = self.trunk_kinematics["pos"]
        rr, rp, ry = self.trunk_kinematics["rpy"]
        bx, by, bz = self.trunk_kinematics["lin_vel_body"]
        wx, wy, wz = self.trunk_kinematics["ang_vel_world"]
        motion_loader = self.rl.policy.motion_loader
        progress = 0.0 if motion_loader is None else min(self.rl.policy.rl_time / motion_loader.duration, 1.0)
        bar_width = 20
        filled = int(round(progress * bar_width))
        bar = "#" * filled + "-" * (bar_width - filled)

        lines = [
            f"sim_t:   {self.mj_data.time:7.2f} s",
            f"fps:     {self._render_fps:6.1f}  ctrl: {self._control_hz:6.1f} Hz",
            f"state:   {state_name}",
            f"model:   [{cmd.model_flag}] {model_name}",
            f"cmd:     x={cmd.x:+5.2f} y={cmd.y:+5.2f} yaw={cmd.yaw:+5.2f}",
            f"pos:     x={px:+6.2f} y={py:+6.2f} z={pz:+6.2f} m",
            f"rpy:     r={rr:+6.2f} p={rp:+6.2f} y={ry:+6.2f} deg",
            f"body lin: bx={bx:+6.2f} by={by:+6.2f} bz={bz:+6.2f} m/s",
            f"body ang: wx={wx:+6.2f} wy={wy:+6.2f} wz={wz:+6.2f} rad/s",
            f"motion:  [{bar}] {progress * 100.0:5.1f}%",
            # "viz: green=world lin.v, orange=world ang.v (arrows @ trunk top, +body z)",
        ]
        return "\n".join(lines)

    def _update_stats(self):
        now = time.perf_counter()
        elapsed = now - self._stats_time
        if elapsed >= 1.0:
            self._control_hz = self._control_counter / elapsed
            self._render_fps = self._render_counter / elapsed
            self._control_counter = 0
            self._render_counter = 0
            self._stats_time = now

    def _render_step(self):
        self._update_trunk_kinematics()
        self.cam.lookat[:] = self.trunk_kinematics["pos"]
        self._render(self._overlay_text())
        self._render_counter += 1

    def _run_periodic(self, now, next_time, period, fn):
        if now < next_time:
            return next_time

        fn()
        next_time += period
        if now - next_time > period:
            next_time = now + period
        return next_time

    def _handle_special_state(self):
        """Handle states that are outside RLDeploy state machine."""
        if self.rl.control.control_state == STATE.STATE_RESET_SIMULATION:
            self.rl.control.control_state = STATE.STATE_WAITING
            self._reset_simulation()
            return True

        if self.rl.control.control_state == STATE.STATE_TOGGLE_SIMULATION:
            self.rl.control.control_state = STATE.STATE_WAITING
            self.paused = not self.paused
            print(f"\n{LOGGER.INFO}Simulation paused={self.paused}")
            return True

        return False

    def _reset_simulation(self):
        """Reset MuJoCo simulation state and keep controller in a safe state."""
        mujoco.mj_resetData(self.mj_model, self.mj_data)
        mujoco.mj_forward(self.mj_model, self.mj_data)

        # Reset control/runtime side to avoid stale commands right after reset.
        # self.rl.running_state = STATE.STATE_WAITING
        self.rl.control.x = 0.0
        self.rl.control.y = 0.0
        self.rl.control.yaw = 0.0
        print(f"\n{LOGGER.INFO}MuJoCo simulation reset")

    # ---------- main loop ----------
    def run(self):
        """Main loop: single-thread control + decoupled render cadence."""
        next_exec_time = time.perf_counter()
        next_render_time = next_exec_time
        thread_period = self.control_dt
        render_period = self.render_dt

        # Keep inference at 50Hz
        inference_times = int(1.0 / 50.0 / self.control_dt)
        print(f"{LOGGER.INFO}control_hz={1/self.control_dt:.1f}hz, inference_times={inference_times}times, render_hz={1/self.render_dt:.1f}hz")

        try:
            while not glfw.window_should_close(self.window):
                if self._handle_special_state():
                    continue

                # 1) Read latest sim state
                self.io.recv(self.rl.temp_robot_state)

                # 2) Run control logic
                self.rl.step(inference_times)

                # 3) Send commands
                self.io.send(self.rl.temp_robot_command)

                # 4) Physics step
                if not self.paused:
                    mujoco.mj_step(self.mj_model, self.mj_data)
                self._control_counter += 1

                # 5) Render at lower cadence than control
                now = time.perf_counter()
                next_render_time = self._run_periodic(now, next_render_time, render_period, self._render_step)

                self._update_stats()

                # 6) Precise control timing
                next_exec_time += thread_period
                now = time.perf_counter()
                sleep_time = next_exec_time - now

                if sleep_time > 0:
                    time.sleep(sleep_time)
                elif sleep_time < -thread_period:
                    next_exec_time = now

        finally:
            self.video_recorder.stop()
            self._close_window()
            print(f"{LOGGER.INFO}MuJoCo GLFW controller shutdown complete")


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
    policy_names = [
        # "issacgym/go2w/go2w_himloco",
        "issacgym/M20/M20_lab",
        "issacgym/M20/M20",
    ]
    # robot_xml = str(MJCF_ROOT / "go2w" / "go2w.xml")
    robot_xml = str(MJCF_ROOT / "M20" / "M20.xml")

    # Humanoid Robot
    # policy_names = [
    #     "unitree_rl_lab/g1/g1_amp",
    #     "unitree_rl_lab/g1/g1_loco",
    #     "unitree_rl_lab/g1/dance_102",
    #     "unitree_rl_lab/g1/gangnam_style",
    #     "unitree_rl_lab/g1/dance1_subject2",
    # ]
    # robot_xml = str(MJCF_ROOT / "g1" / "g1_29dof.xml")


    # Terrain model
    terrain_xml = str(MJCF_ROOT / "terrains" / "empty_world.xml")
    # terrain_xml = str(MJCF_ROOT / "terrains" / "race_track.xml")

    controller = MuJoCoGLFWRobot(policy_names, robot_xml, terrain_xml)
    controller.run()

if __name__ == "__main__":
    main()
