#!/usr/bin/env python3
# Copyright (c) 2026-2027 zh
"""
内容：
    无窗口运行 Go1 WMP 跑酷仿真，默认录制 720p 视频，支持 1080p 和 4K。
    在画面中合成深度图。
    支持指定模型路径，视频以“时间_模型名称.mp4”命名。
    到达终点、超时或持续停滞时自动结束并保存视频。

用法：
    MUJOCO_GL=egl python test/go1_wmp_parkour_4k.py
    MUJOCO_GL=egl python test/go1_wmp_parkour_4k.py --model-path /path/to/policy.pt
    MUJOCO_GL=egl python test/go1_wmp_parkour_4k.py --resolution 1080p --model-path /path/to/policy.pt
    MUJOCO_GL=egl python test/go1_wmp_parkour_4k.py --max-sim-time 90 --stall-time 5
"""

import argparse
import os
import sys
import time
from contextlib import ExitStack
from datetime import datetime
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")

PROJECT_ROOT = Path(__file__).resolve().parents[1]
for import_root in (PROJECT_ROOT, PROJECT_ROOT / "src"):
    if str(import_root) not in sys.path:
        sys.path.insert(0, str(import_root))

import cv2
import imageio.v2 as imageio
import mujoco
import numpy as np

from src.interface.IOMuJoCo import IOMuJoCo
from src.interface.depth.mujoco_depth_camera import MuJoCoDepthCamera
from src.rl_mujoco_glfw import MuJoCoGLFWRobot
from src.scripts.rl_data import LOGGER, STATE
from src.scripts.rl_deploy import RLDeploy
from src.scripts.rl_sdk import RLPolicy
from src.utils.mujoco_video_recorder import MuJoCoVideoRecorder
from src.utils.path_config import MJCF_ROOT


VIDEO_RESOLUTIONS = {"720p": (1280, 720), "1080p": (1920, 1080), "4k": (3840, 2160)}


# ---------- 模型加载 ----------

def load_recording_deploy(policy_names, model_path):
    if model_path is None:
        return RLDeploy(policy_names)

    load_config = RLPolicy._load_config

    def load_recording_config(policy):
        load_config(policy)
        policy.params.model_names = [str(model_path)]

    # 仅在本脚本初始化期间覆盖模型路径，避免先加载默认模型；异常时也恢复原方法。
    try:
        RLPolicy._load_config = load_recording_config
        return RLDeploy(policy_names)
    finally:
        RLPolicy._load_config = load_config


# ---------- 深度相机与录像 ----------

class HeadlessDepthCamera(MuJoCoDepthCamera):
    """复用原有深度处理流程，不创建 OpenCV 窗口。"""

    def __init__(self, model, data, rl):
        named_window = cv2.namedWindow
        cv2.namedWindow = lambda *args, **kwargs: None
        try:
            super().__init__(model, data, rl)
        finally:
            cv2.namedWindow = named_window
        self.show = False


class DepthInsetVideoRecorder(MuJoCoVideoRecorder):
    """在录制画面右上角合成深度图。"""

    def __init__(self, model, model_name, resolution="720p"):
        self.width, self.height = VIDEO_RESOLUTIONS[resolution]
        self.fps = 30
        self.output_dir = PROJECT_ROOT / "recordings"
        self._writer = None
        self._path = None
        self._frame = np.empty((self.height, self.width, 3), dtype=np.uint8)
        self._frame_period = 1.0 / self.fps
        self._next_frame_time = None
        self._frame_count = 0
        # 在创建图形上下文之前设置实际缓冲区大小，避免低分辨率仍按 4K 渲染。
        model.vis.global_.offwidth = self.width
        model.vis.global_.offheight = self.height
        scale = self.height / 2160
        self.inset_size = round(512 * scale)
        self.inset_margin = round(48 * scale)
        self.inset_border = max(1, round(6 * scale))
        self.model_name = Path(model_name).stem
        self.depth_provider = None
        print(f"{self.LOGGER} Video resolution: {self.width}x{self.height}@{self.fps}fps")

    def start(self):
        if self.is_recording:
            return

        self.output_dir.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        self._path = self.output_dir / f"{timestamp}_{self.model_name}.mp4"
        self._writer = imageio.get_writer(
            str(self._path),
            fps=self.fps,
            codec="libx264",
            quality=10,
            macro_block_size=1,
            ffmpeg_params=["-crf", "18", "-preset", "slow"],
        )
        self._next_frame_time = None
        self._frame_count = 0
        print(f"{self.LOGGER} recording started: {self._path}")

    def capture_frame(self, scene, context, sim_time: float, overlay_text: str = ""):
        if not self.is_recording:
            return

        if self._next_frame_time is None:
            self._next_frame_time = sim_time
        if sim_time + 1e-9 < self._next_frame_time:
            return

        viewport = mujoco.MjrRect(0, 0, self.width, self.height)
        previous_buffer = context.currentBuffer
        try:
            mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_OFFSCREEN, context)
            mujoco.mjr_render(viewport, scene, context)
            if overlay_text:
                mujoco.mjr_overlay(
                    mujoco.mjtFont.mjFONT_NORMAL,
                    mujoco.mjtGridPos.mjGRID_TOPLEFT,
                    viewport,
                    overlay_text,
                    "",
                    context,
                )
            mujoco.mjr_readPixels(self._frame, None, viewport, context)
        finally:
            mujoco.mjr_setBuffer(previous_buffer, context)

        frame = np.ascontiguousarray(np.flipud(self._frame))
        self._draw_depth_inset(frame)
        self._writer.append_data(frame)
        self._frame_count += 1
        self._next_frame_time += self._frame_period

    def _draw_depth_inset(self, frame):
        if self.depth_provider is None:
            return

        depth = self.depth_provider()
        if hasattr(depth, "detach"):
            depth = depth.detach().cpu().numpy()
        depth = np.asarray(depth, dtype=np.float32).squeeze()
        depth_display = np.clip(depth + 0.5, 0.0, 1.0)
        depth_display = cv2.resize(depth_display, (self.inset_size, self.inset_size), interpolation=cv2.INTER_LINEAR)
        depth_u8 = np.uint8(depth_display * 255.0)
        depth_rgb = cv2.cvtColor(depth_u8, cv2.COLOR_GRAY2RGB)

        x1 = self.width - self.inset_margin
        x0 = x1 - self.inset_size
        y0 = self.inset_margin
        y1 = y0 + self.inset_size
        border = self.inset_border
        cv2.rectangle(frame, (x0 - border, y0 - border), (x1 + border, y1 + border), (255, 255, 255), border)
        frame[y0:y1, x0:x1] = depth_rgb


# ---------- 仿真控制 ----------

class Go1WMPParkour4K(MuJoCoGLFWRobot):
    """复用现有 GLFW 仿真控制器的无窗口版本。"""

    POLICY_NAMES = ["issacgym/go1/go1wmp"]
    MIN_FORWARD_PROGRESS = 0.2

    def __init__(
        self,
        target_x=59.0,
        target_world_y=0.0,
        target_yaw=0.0,
        x_velocity=1.2,
        y_kp=1.0,
        yaw_kp=1.5,
        control_dt=0.002,
        render_dt=0.02,
        model_path=None,
        resolution="720p",
        max_sim_time=90.0,
        stall_time=5.0,
    ):
        if resolution not in VIDEO_RESOLUTIONS:
            raise ValueError(f"Unsupported video resolution: {resolution}")
        for name, value in (("max_sim_time", max_sim_time), ("stall_time", stall_time)):
            if not np.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be a positive finite number")
        if model_path is not None:
            model_path = Path(model_path).expanduser().resolve()
            if not model_path.is_file():
                raise FileNotFoundError(f"Model file does not exist: {model_path}")
        self.control_dt = control_dt
        self.render_dt = render_dt
        self.paused = False
        self.target_x = float(target_x)
        self.target_world_y = float(target_world_y)
        self.target_yaw = float(target_yaw)
        self.x_velocity = float(x_velocity)
        self.y_kp = float(y_kp)
        self.yaw_kp = float(yaw_kp)
        self.max_sim_time = float(max_sim_time)
        self.stall_time = float(stall_time)
        self.stop_reason = None
        self._progress_x = None
        self._last_forward_progress_time = None

        self._control_counter = 0
        self._render_counter = 0
        self._depth_counter = 0
        self._depth_interval = max(1, round(1.0 / 10.0 / render_dt))
        self._control_hz = 0.0
        self._render_fps = 0.0
        self._stats_time = time.perf_counter()

        self.rl = load_recording_deploy(self.POLICY_NAMES, model_path)

        robot_xml = str(MJCF_ROOT / "go1" / "go1.xml")
        terrain_xml = str(MJCF_ROOT / "terrains" / "parkour.xml")
        self.mj_model, self.mj_data, base_link_name = self._load_model(robot_xml, terrain_xml)
        self.mj_model.opt.timestep = self.control_dt
        self.trunk_id = mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_BODY, base_link_name)

        self.video_recorder = DepthInsetVideoRecorder(self.mj_model, self.rl.params.model_names[0], resolution=resolution)

        self.gl_context = mujoco.GLContext(self.video_recorder.width, self.video_recorder.height)
        self.gl_context.make_current()

        self.cam = mujoco.MjvCamera()
        self.opt = mujoco.MjvOption()
        self._init_mujoco_vis()
        self.cam.azimuth = 90.0
        self.cam.elevation = -25.0
        self.cam.distance = 5.5

        self.depth_camera = HeadlessDepthCamera(self.mj_model, self.mj_data, self.rl)
        self.video_recorder.depth_provider = lambda: self.rl.robot_state.depth.image
        self.io = IOMuJoCo(self.mj_model, self.mj_data, self.rl)

        self._update_trunk_kinematics()
        control = self.rl.robot_state.control
        control.control_state = STATE.STATE_POS_GETUP
        self._update_tracking_command()

    @staticmethod
    def _wrap_angle(angle):
        return np.arctan2(np.sin(angle), np.cos(angle))

    def _world_yaw(self):
        rot = self.trunk_kinematics["rot"]
        return float(np.arctan2(rot[1, 0], rot[0, 0]))

    def _update_tracking_command(self):
        control = self.rl.robot_state.control
        control.model_flag = 0
        control.x = self.x_velocity

        y_error_world = self.target_world_y - self.trunk_kinematics["pos"][1]
        yaw_error = self._wrap_angle(self.target_yaw - self._world_yaw())
        control.y = float(np.clip(self.y_kp * y_error_world, -1.0, 1.0))
        control.yaw = float(np.clip(self.yaw_kp * yaw_error, -2.0, 2.0))

        if self.rl.running_state == STATE.STATE_POS_GETUP and self.rl.getup_percent >= 1.0:
            control.control_state = STATE.STATE_RL_RUNNING

    def _render(self, text=""):
        self.gl_context.make_current()
        mujoco.mjv_updateScene(
            self.mj_model,
            self.mj_data,
            self.opt,
            None,
            self.cam,
            mujoco.mjtCatBit.mjCAT_ALL.value,
            self.scene,
        )
        self._append_trunk_velocity_viz(self.scene)
        self.depth_camera.draw_frustum_scene(self.scene)
        self.video_recorder.capture_frame(self.scene, self.context, self.mj_data.time, text)

    def _termination_reason(self):
        """按仿真时间判断结束条件，起身期间不累计停滞时间。"""
        sim_time = self.mj_data.time
        x = float(self.trunk_kinematics["pos"][0])
        if x >= self.target_x:
            return "target_reached"
        if sim_time >= self.max_sim_time:
            return "time_limit"
        if self.rl.running_state != STATE.STATE_RL_RUNNING:
            self._progress_x = None
            self._last_forward_progress_time = None
            return None

        if self._progress_x is None or x >= self._progress_x + self.MIN_FORWARD_PROGRESS:
            self._progress_x = x
            self._last_forward_progress_time = sim_time

        if sim_time - self._last_forward_progress_time >= self.stall_time:
            return "stalled"
        return None

    def _close_window(self):
        try:
            self.context.free()
        finally:
            self.gl_context.free()

    def run(self):
        next_render_time = self.mj_data.time
        inference_times = int(1.0 / 50.0 / self.control_dt)
        print(f"{LOGGER.INFO}headless Go1 WMP model=0, parkour, target_x={self.target_x:.2f}m")

        try:
            self.video_recorder.start()
            while True:
                if self._handle_special_state():
                    continue

                self.io.recv()
                self._update_trunk_kinematics()
                self.stop_reason = self._termination_reason()
                if self.stop_reason is not None:
                    x = self.trunk_kinematics["pos"][0]
                    print(f"\n{LOGGER.INFO}Recording stopped: reason={self.stop_reason}, sim_time={self.mj_data.time:.2f}s, x={x:.3f}m")
                    break

                self._update_tracking_command()
                self.rl.step(inference_times)
                self.io.send()
                if not self.paused:
                    mujoco.mj_step(self.mj_model, self.mj_data)
                self._control_counter += 1

                next_render_time = self._run_periodic(self.mj_data.time, next_render_time, self.render_dt, self._render_step)
                self._update_stats()
        finally:
            # 即使某项清理失败，也继续保存视频并释放其他资源。
            with ExitStack() as cleanup:
                cleanup.callback(self._close_window)
                cleanup.callback(self.video_recorder.stop)
                cleanup.callback(self.depth_camera.close)
            print(f"{LOGGER.INFO}Headless recording run stopped")
        return self.stop_reason


# ---------- 命令行入口 ----------

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--target-x", type=float, default=59.0)
    parser.add_argument("--model-path", help="Path to an exported TorchScript model; defaults to the Go1 WMP config.")
    parser.add_argument("--resolution", choices=VIDEO_RESOLUTIONS, default="720p", help="Video resolution (default: 720p).")
    parser.add_argument("--max-sim-time", type=float, default=90.0, help="Maximum simulation duration in seconds (default: 90).")
    parser.add_argument("--stall-time", type=float, default=5.0, help="Stop after this many simulation seconds without 0.2 m forward progress (default: 5).")
    args = parser.parse_args()
    Go1WMPParkour4K(
        target_x=args.target_x,
        model_path=args.model_path,
        resolution=args.resolution,
        max_sim_time=args.max_sim_time,
        stall_time=args.stall_time,
    ).run()


if __name__ == "__main__":
    main()
