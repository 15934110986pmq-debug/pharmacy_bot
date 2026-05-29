#!/usr/bin/env bash
# ============================================================
# start_ai_only.sh — 纯 AI 模式（无 ROS，服务器端测试用）
# 用法: bash scripts/start_ai_only.sh
# 说明: 仅启动 AI Agent 症状诊断，不涉及 ROS 或机械臂
# ============================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

echo "===== PharmacyBot 纯 AI 模式 ====="
echo ""

# ── 设置环境变量 ───────────────────────────────────────────
# 大模型 API 密钥 (可从环境变量读取，或在此填入默认值)
export DEEPSEEK_API_KEY="${DEEPSEEK_API_KEY:-}"
# 可选: 切换 LLM 供应商 (deepseek / openai / ollama)
export LLM_PROVIDER="${LLM_PROVIDER:-deepseek}"

# 设置 PYTHONPATH 确保能导入 ai_agent 模块
export PYTHONPATH="$PROJECT_DIR:$PYTHONPATH"

echo "   LLM_PROVIDER     = $LLM_PROVIDER"
echo "   DEEPSEEK_API_KEY = ${DEEPSEEK_API_KEY:-(未设置)}"
echo "   PYTHONPATH       = $PYTHONPATH"
echo ""

# ── 演示症状诊断 ───────────────────────────────────────────
echo "▶ 运行症状诊断演示 (症状: 头痛发烧两天，有点咳嗽)"
echo ""

python3 << 'PYEOF'
import sys
import json

from ai_agent.symptom_agent import SymptomAgent
from ai_agent.drugs_sample import SAMPLE_DRUGS

# 初始化 Agent
agent = SymptomAgent()

# 加载样本药物数据
agent.ensure_loaded(json.dumps(SAMPLE_DRUGS, ensure_ascii=False))

# 诊断示例
result = agent.diagnose("头痛发烧两天，有点咳嗽")

print("=" * 50)
print("诊断结果 (结构化 JSON):")
print("=" * 50)
print(json.dumps(result, ensure_ascii=False, indent=2))
print("=" * 50)
print("✅ AI Agent 演示完成")
PYEOF

echo ""
echo "===== ✅ AI 模式测试完成 ====="
echo "提示: 如需使用 LLM API，请先设置:"
echo "  export DEEPSEEK_API_KEY=\"your-key-here\""
echo "  export LLM_PROVIDER=\"deepseek\""
echo "然后重新运行本脚本。"
