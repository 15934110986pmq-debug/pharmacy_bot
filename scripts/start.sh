#!/usr/bin/env bash
# ============================================================
# start.sh — PharmacyBot 全栈启动（Jetson 端）
# 用法: bash scripts/start.sh
# ============================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

echo "===== PharmacyBot 启动 ====="

# ── 进入 ROS 工作空间 ──────────────────────────────────────
cd "$WORKSPACE_DIR"
source devel/setup.bash

# ── 启动机械臂底层 ──────────────────────────────────────────
echo "[1/3] 启动机械臂底层 (jetarm_bringup)..."
roslaunch jetarm_bringup bringup.launch &
BRINGUP_PID=$!
sleep 5

# ── 启动语音识别 ────────────────────────────────────────────
echo "[2/3] 启动讯飞离线语音识别..."
roslaunch xf_mic_asr_offline voice_control_pharmacy.launch &
ASR_PID=$!

# ── 启动视觉分拣 ────────────────────────────────────────────
echo "[3/3] 启动 6-DOF 视觉分拣应用..."
roslaunch jetarm_6dof_app object_sortting.launch &
SORT_PID=$!

echo ""
echo "===== ✅ PharmacyBot started ====="
echo "  机械臂底层  PID: $BRINGUP_PID"
echo "  语音识别    PID: $ASR_PID"
echo "  视觉分拣    PID: $SORT_PID"
echo ""
echo "停止所有进程: kill $BRINGUP_PID $ASR_PID $SORT_PID"
