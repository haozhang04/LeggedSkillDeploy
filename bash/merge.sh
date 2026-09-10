#!/usr/bin/env bash
set -e

MAIN_BRANCH="lgdeploy"
TARGET_BRANCHES=(
    "lgdeploy_duow"
    "lgdeploy_go1"
    "lgdeploy_go1parkour"
)

# 确保工作区干净
if ! git diff --quiet || ! git diff --cached --quiet; then
    echo "当前有未提交修改，请先 commit 或 stash"
    git status
    exit 1
fi

# 拉取远端最新分支信息
git fetch origin

# 依次更新目标分支
for branch in "${TARGET_BRANCHES[@]}"; do
    echo "======================================"
    echo "Update $branch from origin/$MAIN_BRANCH"
    echo "======================================"

    git checkout "$branch"

    git merge "origin/$MAIN_BRANCH" || {
        echo "分支 $branch 合并冲突，请手动解决："
        echo "  git status"
        echo "  git add -A"
        echo "  git commit"
        echo "然后重新运行脚本，或手动继续剩余分支。"
        exit 1
    }

    git push origin "$branch"
done

echo "全部分支更新完成"