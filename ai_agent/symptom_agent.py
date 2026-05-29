"""
症状诊断Agent — 编排RAG检索 → API调用 → 禁忌检测 → 输出结构化推荐
"""
import json
import logging
from .config import LLM_PROVIDER, DRUG_KB_PATH, DRUG_DATA_JSON
from .llm_client import LLMClient
from .drug_kb import DrugKnowledgeBase

logger = logging.getLogger(__name__)

SYSTEM_PROMPT = """你是一位资深临床药剂师和医学顾问。

根据患者描述的症状和你掌握的药物知识库检索结果，给出专业的药物推荐。

严格要求：
1. 只推荐知识库中存在的药物（drug_id来自检索结果）
2. 优先推荐适应症最匹配的药物
3. 检查禁忌：如有过敏史或交互禁忌，必须在warnings中标注
4. 如信息不足（未提供年龄/过敏史/既往病史），在follow_up中列出需追问的信息
5. 不要编造知识库中不存在的药物

输出严格JSON格式：
{
  "diagnosis": "初步判断（一句话）",
  "recommendations": [
    {
      "drug_id": "DRG-001",
      "drug_name": "阿莫西林胶囊",
      "reason": "推荐理由（症状如何匹配适应症）",
      "dosage": "0.5g，每日3次",
      "precautions": ["注意事项1", "注意事项2"]
    }
  ],
  "warnings": ["禁忌警告1", "缺失信息提示"],
  "follow_up": ["需要追问的信息1", "需要追问的信息2"]
}"""


class SymptomAgent:
    """症状诊断Agent — 全流程编排"""

    def __init__(self, provider=None, kb_path=None):
        if provider is None:
            provider = LLM_PROVIDER
        if kb_path is None:
            kb_path = DRUG_KB_PATH

        self.llm = LLMClient(provider=provider)
        self.kb = DrugKnowledgeBase(kb_path)
        self._loaded = False

    def ensure_loaded(self, drugs_json=None):
        """确保药物知识库已加载"""
        if self._loaded:
            return
        if drugs_json is None:
            drugs_json = DRUG_DATA_JSON
        try:
            self.kb.load_drugs(drugs_json)
            self._loaded = True
        except FileNotFoundError:
            logger.warning(f"Drug data file not found: {drugs_json}. KB will be empty.")
            self._loaded = True

    def diagnose(self, symptom_text: str) -> dict:
        """
        主入口: 症状 → AI诊断 + 药物推荐

        Args:
            symptom_text: 患者症状描述
                e.g. "我头痛两天了，还有点发烧，咳嗽"

        Returns:
            {
                "diagnosis": "初步判断",
                "recommendations": [{drug_id, drug_name, reason, dosage, precautions}],
                "warnings": [],
                "follow_up": [],
                "candidates_retrieved": 5  # RAG检索到的候选药物数量
            }
        """
        self.ensure_loaded()

        # 0. 隐私脱敏
        symptom_text = self._sanitize_input(symptom_text)

        # 1. RAG检索: 症状 → Top-5 匹配药物
        candidates = self.kb.retrieve(symptom_text, k=5)
        logger.info(f"RAG retrieved {len(candidates)} candidate drugs")

        if not candidates:
            return {
                "diagnosis": "无法匹配",
                "recommendations": [],
                "warnings": ["知识库中未找到匹配药物，请扩充药物数据"],
                "follow_up": ["建议详细描述症状"],
                "candidates_retrieved": 0,
            }

        # 2. 构造上下文
        context = json.dumps(candidates, ensure_ascii=False, indent=2)

        # 3. API推理
        logger.info(f"Sending to LLM API ({self.llm.provider}/{self.llm.model})...")
        raw_result = self.llm.chat(SYSTEM_PROMPT, symptom_text, context, json_mode=True)

        # type guard: JSON mode guarantees dict
        if isinstance(raw_result, str):
            logger.error("LLM returned string instead of dict (json_mode should guarantee dict)")
            raw_result = {"diagnosis": "API返回格式异常", "recommendations": [], "warnings": [], "follow_up": []}

        result: dict = raw_result

        # 4. 后处理: 校验 + 补充
        result["candidates_retrieved"] = len(candidates)

        # 确保每个推荐的drug_id在候选列表中
        valid_ids = {c["drug_id"] for c in candidates}
        result["recommendations"] = [
            r
            for r in result.get("recommendations", [])
            if r.get("drug_id") in valid_ids
        ]

        if not result["recommendations"] and candidates:
            # LLM没给出有效推荐 → 回退: 直接返回RAG检索Top-1
            top = candidates[0]
            result["recommendations"] = [
                {
                    "drug_id": top["drug_id"],
                    "drug_name": top["drug_name"],
                    "reason": f"基于症状匹配 (相似度: {top.get('similarity', 'N/A')})",
                    "dosage": top.get("dosage", {}).get("adult", "请遵医嘱"),
                    "precautions": top.get("contraindications", []),
                }
            ]
            result["diagnosis"] = result.get("diagnosis", "规则引擎匹配")
            logger.warning("LLM returned no valid recommendations, fallback to rule engine")

        # 5. 禁忌交叉检测
        result = self._check_interactions(result)

        return result

    def _sanitize_input(self, text: str) -> str:
        """隐私脱敏: 移除可能的PII信息后再发送到云端API"""
        import re
        # 移除电话号码
        text = re.sub(r'1[3-9]\d{9}', '[电话已隐藏]', text)
        # 移除身份证号
        text = re.sub(r'\d{17}[\dXx]', '[证件号已隐藏]', text)
        # 移除邮箱
        text = re.sub(r'[a-zA-Z0-9._%+-]+@[a-zA-Z0-9.-]+\.[a-zA-Z]{2,}', '[邮箱已隐藏]', text)
        return text

    def _check_interactions(self, result: dict) -> dict:
        """检测推荐药物之间的交互禁忌"""
        recs = result.get("recommendations", [])
        for i, r1 in enumerate(recs):
            for j, r2 in enumerate(recs):
                if i >= j:
                    continue
                # TODO: 从知识库加载交互禁忌表进行交叉检查
                pass
        return result
