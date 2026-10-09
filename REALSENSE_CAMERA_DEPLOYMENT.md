# RealSense 实机深度相机配置

## 目标

训练时相机输入为：

```text
分辨率：64×64
水平 FOV：58°
深度范围：0～2 m
归一化范围：[-0.5, 0.5]
相机位置：[0.28, 0, 0.12] m
相机俯角：向下 pitch 20° yaw/roll 0°
```

实机相机参数：

```text
分辨率：480×270
fx = fy = 240.210602
ppx = 237.971802
ppy = 133.169891
depth_scale = 0.001 m/unit
畸变系数：[0, 0, 0, 0, 0]
```

## 必须修改的裁剪参数

在 `src/interface/depth/realsense_depth_camera.py` 中设置：

```python
self.crop_top = 1
self.crop_bottom = 3
self.crop_left = 105
self.crop_right = 109
```

裁剪后的区域为：

```text
x: [105, 371)
y: [1, 267)
尺寸：266×266
```

目标 FOV 为 58°，对应的像素尺寸计算如下：

```text
N = 2 × f × tan(FOV / 2)
  = 2 × 240.210602 × tan(58° / 2)
  ≈ 266.3
```

因此取最接近的整数方形区域 `266×266`。

相机光学主点为：

```text
cx = ppx = 237.971802
cy = ppy = 133.169891
```

对于 `N=266`，ROI 中心坐标为 `start + (N-1)/2`，所以：

```text
start_x = round(cx - (N - 1) / 2)
        = round(237.971802 - 132.5) = 105

start_y = round(cy - (N - 1) / 2)
        = round(133.169891 - 132.5) = 1

end_x = start_x + N = 371
end_y = start_y + N = 267
```

因此保留：

```python
depth_m[1:267, 105:371]
```

四边裁剪量为：

```text
crop_left   = start_x     = 105
crop_right  = 480 - end_x = 109
crop_top    = start_y     = 1
crop_bottom = 270 - end_y = 3
```

裁剪区域中心为 `(237.5, 133.5)`，与光学主点仅相差 `(-0.47, +0.33)` 像素。偶数尺寸 ROI 的中心位于两个像素之间，因此无法与主点完全重合。

该区域的水平和垂直 FOV 均约为 58°。不要使用当前的 `left=60, right=46`，它对应约 76° 水平 FOV，并且会把非方形图像拉伸为 64×64。

## 深度图处理顺序

```text
480×270 Z16 深度图
  -> RealSense 滤波
  -> 乘 depth_scale 转为米
  -> 裁剪为 266×266
  -> 最近邻缩放为 64×64
  -> 无效深度设置为 2 m
  -> 深度截断到 [0, 2] m
  -> 归一化到 [-0.5, 0.5]
```
