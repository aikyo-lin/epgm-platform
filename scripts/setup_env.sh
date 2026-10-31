#!/usr/bin/env bash
# ==============================================================================
# [ZH] EPGM 生产级云端与算法环境一键自动化安装脚本 (Linux/macOS Bash)
# [EN] EPGM Production Cloud & Algorithm One-Click Installer (Linux/macOS Bash)
# ==============================================================================

set -e

echo "=========================================================="
echo " [ZH] 正在初始化 EPGM 全栈环境安装程序..."
echo " [EN] Initializing EPGM fullstack environment installer..."
echo "=========================================================="

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ALGO_DIR="${SCRIPT_DIR}/../../epgm-algorithm"

echo ">>> [1/3] 检查 Python 运行环境..."
python3 --version

echo ">>> [2/3] 安装 epgm-algorithm 核心依赖..."
pip3 install -r "${ALGO_DIR}/requirements.txt"

echo ">>> [3/3] 执行 EPGM 算法仿真基准自测..."
python3 "${ALGO_DIR}/experiments/run_maze_benchmark.py"

echo "=========================================================="
echo " [ZH] EPGM 环境配置与算法自测顺利完成！"
echo " [EN] EPGM Environment setup completed successfully!"
echo "=========================================================="
