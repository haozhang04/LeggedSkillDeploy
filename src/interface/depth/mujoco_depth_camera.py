# Copyright (c) 2024-2025 zh

import cv2
import mujoco
import numpy as np
import torch


class MuJoCoDepthCamera:
    """Render normalized depth from a fixed MuJoCo camera."""

    def __init__(self, model, data, rl):
        self.model = model
        self.data = data
        self.depth = rl.robot_state.depth
        self.width = self.depth.width
        self.height = self.depth.height
        self.near_clip = self.depth.near_clip
        self.far_clip = self.depth.far_clip
        self.show = True
        self.show_frustum = True
        self.window_name = "MuJoCo depth"
        self.camera_name = "go1/depth_camera"
        self.camera_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, self.camera_name)
        if self.camera_id < 0:
            raise ValueError(f"MuJoCo camera not found: {self.camera_name}")

        self.renderer = mujoco.Renderer(model, height=self.height, width=self.width)
        self.renderer.enable_depth_rendering()

        if self.show:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)

    def update_depth(self):
        self.renderer.update_scene(self.data, camera=self.camera_id)
        depth_m = np.asarray(self.renderer.render(), dtype=np.float32)
        valid = np.isfinite(depth_m) & (depth_m > self.near_clip)
        depth_m = np.where(valid, depth_m, self.far_clip)
        depth_m = np.clip(depth_m, self.near_clip, self.far_clip)
        depth = (depth_m - self.near_clip) / (self.far_clip - self.near_clip) - 0.5
        self.depth.image = torch.from_numpy(depth).unsqueeze(0).unsqueeze(0)
        self.show_depth(depth)

    def show_depth(self, depth):
        if not self.show:
            return
        cv2.imshow(self.window_name, np.clip(depth + 0.5, 0.0, 1.0))
        cv2.waitKey(1)

    def draw_frustum(self, viewer, color=(1.0, 1.0, 0.0, 1.0), line_width=2.0):
        self._draw_frustum(viewer.user_scn, color, line_width, reset_scene=True)

    def draw_frustum_scene(self, scene, color=(1.0, 1.0, 0.0, 1.0), line_width=2.0):
        self._draw_frustum(scene, color, line_width, reset_scene=False)

    def _draw_frustum(self, scene, color, line_width, reset_scene):
        if not self.show_frustum:
            return

        camera_pos = self.data.cam_xpos[self.camera_id]
        camera_rot = self.data.cam_xmat[self.camera_id].reshape(3, 3)
        far = self.far_clip
        half_height = far * np.tan(np.deg2rad(self.model.cam_fovy[self.camera_id]) / 2.0)
        half_width = half_height * self.width / self.height
        local_corners = np.array(
            [
                [-half_width, -half_height, -far],
                [half_width, -half_height, -far],
                [half_width, half_height, -far],
                [-half_width, half_height, -far],
            ]
        )
        world_corners = camera_pos + local_corners @ camera_rot.T
        lines = [(camera_pos, corner) for corner in world_corners]
        lines.extend((world_corners[index], world_corners[(index + 1) % 4]) for index in range(4))

        if reset_scene:
            scene.ngeom = 0
        rgba = np.asarray(color, dtype=np.float32)
        for start, end in lines:
            if scene.ngeom >= scene.maxgeom:
                break
            geom = scene.geoms[scene.ngeom]
            mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_LINE, np.zeros(3), np.zeros(3), np.eye(3).reshape(-1), rgba)
            mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_LINE, line_width, start, end)
            scene.ngeom += 1

    def close(self):
        if self.show:
            try:
                cv2.destroyWindow(self.window_name)
            except cv2.error:
                pass
        if hasattr(self.renderer, "close"):
            self.renderer.close()
