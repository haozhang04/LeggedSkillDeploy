#!/bin/sh

# 赋予权限：chmod +x run_parkour.sh
# 设置开机启动：gnome-session-properties

# 计算项目根目录
DIR="$(cd "$(dirname "$0")/.." && pwd)"

# 激活环境命令
# CON_CMD="source /opt/ros/humble/setup.bash && cd \"$DIR\""
CON_CMD="cd \"$DIR\""

gnome-terminal \
  --tab --title="depth" -e "bash -c \"$CON_CMD; python3 go1_visual_embedding.py; exec bash\"" \
  --tab --title="ctl"   -e "bash -c \"$CON_CMD; python3 src/rl_real_go1.py; exec bash\"" \
  --tab --title="view"  -e "bash -c \"$CON_CMD; python3 go1_view_realsense_depth.py; exec bash\"" \
