# Copyright (c) 2024-2025 zh

import mujoco
import numpy as np
from pathlib import Path
from datetime import datetime
import imageio.v2 as imageio


class MuJoCoVideoRecorder:
    """4K MuJoCo offscreen video recorder."""

    LOGGER = "[Recorder]"

    def __init__(self, model):
        self.width = 3840
        self.height = 2160
        self.fps = 30
        self.output_dir = Path("recordings")
        self._writer = None
        self._path = None
        self._frame = np.empty((self.height, self.width, 3), dtype=np.uint8)
        self._frame_period = 1.0 / float(self.fps)
        self._next_frame_time = None
        self._frame_count = 0

        model.vis.global_.offwidth = self.width
        model.vis.global_.offheight = self.height

        print(f"{self.LOGGER} Press R to start/stop {self.width}x{self.height}@{self.fps}fps recording")

    @property
    def is_recording(self) -> bool:
        return self._writer is not None

    def start(self):
        if self.is_recording:
            return

        self.output_dir.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        self._path = self.output_dir / f"{timestamp}_{self.width}x{self.height}_{self.fps}fps.mp4"
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

    def stop(self):
        if not self.is_recording:
            return

        path = self._path
        frame_count = self._frame_count
        self._writer.close()
        self._writer = None
        self._next_frame_time = None
        self._frame_count = 0
        print(f"{self.LOGGER} recording saved: {path} ({frame_count} frames)")

    def toggle(self):
        if self.is_recording:
            self.stop()
        else:
            self.start()

    def capture_frame(self, scene, context, sim_time: float, overlay_text: str = ""):
        if not self.is_recording:
            return

        if self._next_frame_time is None:
            self._next_frame_time = sim_time

        if sim_time + 1e-9 < self._next_frame_time:
            return

        viewport = mujoco.MjrRect(0, 0, self.width, self.height)

        try:
            mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_OFFSCREEN, context)
            mujoco.mjr_render(viewport, scene, context)
            if overlay_text:
                mujoco.mjr_overlay(mujoco.mjtFont.mjFONT_NORMAL, mujoco.mjtGridPos.mjGRID_TOPLEFT, viewport, overlay_text, "", context)
            mujoco.mjr_readPixels(self._frame, None, viewport, context)
        finally:
            mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_WINDOW, context)

        self._writer.append_data(np.ascontiguousarray(np.flipud(self._frame)))
        self._frame_count += 1
        self._next_frame_time += self._frame_period
