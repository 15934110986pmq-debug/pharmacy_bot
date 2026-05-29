"""
AI Agent 配置
"""
import os

# Provider 选择: "deepseek" | "qwen" | "custom"
LLM_PROVIDER = os.getenv("PHARMACY_LLM_PROVIDER", "deepseek")

# 药物知识库路径
DRUG_KB_PATH = os.getenv("PHARMACY_DRUG_KB", "./drug_kb_store")
DRUG_DATA_JSON = os.getenv("PHARMACY_DRUG_DATA", "./drugs.json")

# API 重试配置
MAX_RETRIES = int(os.getenv("PHARMACY_API_MAX_RETRIES", "3"))
API_TIMEOUT = int(os.getenv("PHARMACY_API_TIMEOUT", "15"))
