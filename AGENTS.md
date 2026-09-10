# 项目开发规范

## 语言

- 所有代码注释、文档和回复均使用中文。
- 所有输出的日志信息均使用英文。
- 其它遵循项目已有风格。

## 大功能开发流程

- 开发较大的功能前，必须先创建对应的 Markdown 设计文档。
- 不把多个职责混杂在同一个文件中。
- 先确认模块边界和目录归属，再开始实现代码。

## Git Commit

使用 Conventional Commits：

`<type>(<scope>): <description>`

类型：
- `feat`: 新功能
- `fix`: 修复 bug
- `refactor`: 重构，不改变功能
- `perf`: 性能优化
- `docs`: 文档
- `test`: 测试
- `chore`: 配置、依赖、参数等维护
- `build`: 构建相关
- `ci`: CI/CD
- `revert`: 回滚

规则：
- 根据实际 diff 和修改目的选择 type
- description 使用简短中文

## 文件头注释

每个可独立运行的脚本顶部都应包含版权声明和简洁的模块说明：

```python
# Copyright (c) 2026-2027 zh
"""
内容：
    采集并发布 RealSense 深度图及其网络输入。
    发布原始深度图和预处理后的 64×64 网络输入，按 Ctrl+C 退出。

用法：
    python go1_visual_embedding.py
"""
```

要求：

- `内容` 简洁说明该脚本负责什么。
- 独立运行的脚本应包含 `用法`。
- 作为子模块被导入的文件可以省略 `用法`。
- 不写与当前文件无关的说明。

## 代码风格

- 能写为一行且不影响可读性的代码，尽量保持单行。
- 代码过长、参数较多或嵌套复杂时，拆分为多行。
- 不要为了凑单行而降低可读性。

单行示例：

```python
self.raw_subscription = self.create_subscription(Image, "/camera/depth/image_raw", self._raw_callback, sensor_qos)
self.network_subscription = self.create_subscription(Image, "/camera/depth/network_input", self._network_callback, sensor_qos)
self.get_logger().warning(f"Unsupported network input encoding: {message.encoding}", throttle_duration_sec=2.0)
```

多行示例：

```python
mujoco.mjv_updateScene(
    self.mj_model,
    self.mj_data,
    self.opt,
    None,
    self.cam,
    mujoco.mjtCatBit.mjCAT_ALL.value,
    self.scene,
)
```

## 代码分块

较长代码文件应使用分块注释划分逻辑区域：

```python
# ---------- init ----------

# ---------- callbacks ----------

# ---------- helpers ----------

# ---------- main ----------
```

按实际内容选择合适的分块名称，避免为了分块而添加无意义注释。

## 验收

- 完成后用中文简要说明：修改内容、涉及模块和结果。
