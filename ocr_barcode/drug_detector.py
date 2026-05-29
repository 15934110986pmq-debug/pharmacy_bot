"""
drug_detector.py — 三通道融合判优模块

功能:
  条码+OCR+颜色 三通道融合判优：
    a) 条码命中 → 直接返回（置信度最高）
    b) 条码无结果 → OCR 识别药品名 → 匹配本地药物知识库
    c) 都无结果 → 返回 None
  method 取值: 'barcode', 'ocr', 'color_fallback', None

集成目标:
  与 ai_agent/drugs_sample.py 中的 SAMPLE_DRUGS 对齐。
  硬编码 5 种药品 SKU 映射表，包括条码前缀匹配和 OCR 关键词匹配。

依赖:
  - barcode_scanner (同目录)
  - ocr_scanner (同目录)
  - opencv-python
  - numpy
"""

from __future__ import annotations

import logging
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np

from .barcode_scanner import scan_barcode, reset_stability
from .ocr_scanner import scan_text

logger = logging.getLogger(__name__)

# =====================================================================
# 本地药品 SKU 映射表
# 与 ai_agent/drugs_sample.py 中的 SAMPLE_DRUGS 对齐
# =====================================================================
DRUG_SKU_TABLE: List[Dict[str, Any]] = [
    {
        "drug_id": "DRG-001",
        "name": "阿莫西林胶囊",
        "generic_name": "Amoxicillin",
        "barcode_patterns": ["690123456789", "123456789012"],  # EAN-13 前缀
        "keywords": ["阿莫西林", "amoxicillin", "Amoxicillin", "阿莫西"],
    },
    {
        "drug_id": "DRG-002",
        "name": "布洛芬缓释胶囊",
        "generic_name": "Ibuprofen",
        "barcode_patterns": ["690987654321", "987654321098"],
        "keywords": ["布洛芬", "ibuprofen", "Ibuprofen", "缓释"],
    },
    {
        "drug_id": "DRG-003",
        "name": "对乙酰氨基酚片",
        "generic_name": "Paracetamol",
        "barcode_patterns": ["690111223344", "111223344556"],
        "keywords": ["对乙酰氨基酚", "paracetamol", "Paracetamol",
                     "乙酰氨基酚", "扑热息痛"],
    },
    {
        "drug_id": "DRG-004",
        "name": "头孢克肟胶囊",
        "generic_name": "Cefixime",
        "barcode_patterns": ["690555666777", "555666777888"],
        "keywords": ["头孢克肟", "cefixime", "Cefixime", "头孢"],
    },
    {
        "drug_id": "DRG-005",
        "name": "复方甘草片",
        "generic_name": "Compound Liquorice",
        "barcode_patterns": ["690999888777", "999888777666"],
        "keywords": ["复方甘草", "甘草片", "compound liquorice",
                     "Compond Liquorice", "甘草"],
    },
]


def _match_by_barcode(barcode_data: str) -> Optional[Dict[str, Any]]:
    """
    根据条码数据匹配药品。

    匹配规则:
      1. 精确匹配 barcode_patterns 中的任意一个
      2. 前缀匹配（条码前 N 位匹配 pattern 前 N 位）
    """
    barcode_clean = barcode_data.strip()
    if not barcode_clean:
        return None

    for drug in DRUG_SKU_TABLE:
        for pattern in drug["barcode_patterns"]:
            # 精确匹配
            if barcode_clean == pattern:
                return drug
            # 前缀匹配（至少 6 位匹配）
            min_len = min(len(barcode_clean), len(pattern))
            if min_len >= 6 and barcode_clean[:min_len] == pattern[:min_len]:
                return drug
    return None


def _match_by_keywords(text: str) -> Optional[Tuple[str, float]]:
    """
    根据 OCR 识别文本匹配药品关键词。
    返回 (drug_id, confidence_score)。

    匹配规则:
      - 逐药品检查关键词是否出现在文本中
      - 得分 = 匹配关键词数 / 该药品总关键词数
      - 取最高分，要求 >= 0.3
    """
    text_lower = text.lower()
    best_id: Optional[str] = None
    best_score: float = 0.0

    for drug in DRUG_SKU_TABLE:
        match_count = 0
        for kw in drug["keywords"]:
            if kw.lower() in text_lower:
                match_count += 1
        if match_count > 0:
            score = match_count / len(drug["keywords"])
            if score > best_score:
                best_score = score
                best_id = drug["drug_id"]

    if best_id is not None and best_score >= 0.3:
        return (best_id, round(best_score, 4))
    return None


def _match_by_color_fallback(
    cv2_image: np.ndarray,
) -> Optional[Tuple[str, float]]:
    """
    颜色回退方案：通过主色调粗略判断药品类别。
    这是一个极简的 fallback，实际精度有限。

    药品与颜色的映射（硬编码，基于常见药盒颜色统计）:
      - 阿莫西林: 红/白 (Red hue range)
      - 布洛芬: 蓝/白 (Blue hue range)
      - 对乙酰氨基酚: 绿/白 (Green hue range)
      - 头孢克肟: 黄/白 (Yellow hue range)
      - 复方甘草: 棕 (Brown hue range)

    返回 (drug_id, confidence)，confidence 固定为 0.3。
    """
    try:
        h, w = cv2_image.shape[:2]
        if h < 10 or w < 10:
            return None

        # 取中心区域主色调
        roi = cv2_image[h // 4:3 * h // 4, w // 4:3 * w // 4]
        if roi.size == 0:
            return None

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        # 计算 Hue 直方图
        hist = cv2.calcHist([hsv], [0], None, [180], [0, 180])
        dominant_hue = int(np.argmax(hist))

        # 粗略色调映射
        # Red: 0-10, 170-180
        # Orange: 11-25
        # Yellow: 26-35
        # Green: 36-85
        # Blue: 86-125
        # Purple: 126-150
        # Brown/Red-brown: 0-10 with low Saturation
        if dominant_hue in range(0, 11) or dominant_hue in range(170, 181):
            # 红 → 阿莫西林
            return ("DRG-001", 0.3)
        elif dominant_hue in range(86, 126):
            # 蓝 → 布洛芬
            return ("DRG-002", 0.3)
        elif dominant_hue in range(36, 86):
            # 绿 → 对乙酰氨基酚
            return ("DRG-003", 0.3)
        elif dominant_hue in range(26, 36):
            # 黄 → 头孢克肟
            return ("DRG-004", 0.3)
        elif dominant_hue in range(11, 26):
            # 橙/棕 → 复方甘草
            return ("DRG-005", 0.3)
        else:
            return None
    except Exception as exc:
        logger.debug("Color fallback failed: %s", exc)
        return None


def _get_drug_info(drug_id: str) -> Optional[Dict[str, Any]]:
    """根据 drug_id 从映射表获取完整药品信息。"""
    for drug in DRUG_SKU_TABLE:
        if drug["drug_id"] == drug_id:
            return drug
    return None


def detect_drug(cv2_image: np.ndarray,
                stability_frames: int = 3,
                use_color_fallback: bool = True,
                reset: bool = False) -> Dict[str, Any]:
    """
    三通道融合判优：条码 → OCR → 颜色回退。

    Args:
        cv2_image: OpenCV BGR numpy 数组图像。
        stability_frames: 条码多帧稳定帧数（默认 3）。
        use_color_fallback: 是否启用颜色回退方案（默认 True）。
        reset: 是否重置条码帧缓存。

    Returns:
        dict: {
            "method": "barcode" | "ocr" | "color_fallback" | None,
            "drug_id": str | None,
            "drug_name": str | None,
            "confidence": float,
            "detail": dict | None,
        }
        全部失败时 method=None, drug_id=None, drug_name=None, confidence=0.0, detail=None。
    """
    # ---- 结果模板 ----
    no_result: Dict[str, Any] = {
        "method": None,
        "drug_id": None,
        "drug_name": None,
        "confidence": 0.0,
        "detail": None,
    }

    if cv2_image is None or cv2_image.size == 0:
        logger.warning("detect_drug: received empty image")
        return no_result

    # ---- 通道 A: 条码识别 ----
    try:
        barcode_results = scan_barcode(cv2_image,
                                       stability_frames=stability_frames,
                                       reset=reset)
    except Exception as exc:
        logger.error("Barcode scan error: %s", exc)
        barcode_results = []

    if barcode_results:
        best_barcode = barcode_results[0]
        matched_drug = _match_by_barcode(best_barcode["data"])
        if matched_drug is not None:
            logger.info("Drug matched via barcode: %s (data=%s)",
                       matched_drug["name"], best_barcode["data"])
            return {
                "method": "barcode",
                "drug_id": matched_drug["drug_id"],
                "drug_name": matched_drug["name"],
                "confidence": best_barcode["quality"],
                "detail": {
                    "barcode_data": best_barcode["data"],
                    "barcode_type": best_barcode["type"],
                    "barcode_quality": best_barcode["quality"],
                },
            }
        else:
            logger.debug("Barcode found but no drug match: %s", best_barcode["data"])

    # ---- 通道 B: OCR 药品名识别 ----
    try:
        ocr_results = scan_text(cv2_image)
    except Exception as exc:
        logger.error("OCR scan error: %s", exc)
        ocr_results = []

    if ocr_results:
        # 合并所有 OCR 文本做关键词匹配
        all_text = " ".join(r["text"] for r in ocr_results)
        logger.debug("OCR combined text: %s", all_text)

        keyword_match = _match_by_keywords(all_text)
        if keyword_match is not None:
            drug_id, match_score = keyword_match
            drug_info = _get_drug_info(drug_id)
            drug_name = drug_info["name"] if drug_info else None
            logger.info("Drug matched via OCR keywords: %s (score=%.4f)",
                       drug_name, match_score)
            return {
                "method": "ocr",
                "drug_id": drug_id,
                "drug_name": drug_name,
                "confidence": match_score,
                "detail": {
                    "ocr_texts": [r["text"] for r in ocr_results],
                    "ocr_confidences": [r["confidence"] for r in ocr_results],
                    "keyword_match_score": match_score,
                },
            }
        else:
            logger.debug("OCR results found but no keyword match")

    # ---- 通道 C: 颜色回退 ----
    if use_color_fallback:
        try:
            color_match = _match_by_color_fallback(cv2_image)
            if color_match is not None:
                drug_id, fallback_conf = color_match
                drug_info = _get_drug_info(drug_id)
                drug_name = drug_info["name"] if drug_info else None
                logger.info("Drug matched via color fallback: %s (conf=%.2f)",
                           drug_name, fallback_conf)
                return {
                    "method": "color_fallback",
                    "drug_id": drug_id,
                    "drug_name": drug_name,
                    "confidence": fallback_conf,
                    "detail": {
                        "method_note": "color-based histogram matching (low confidence)",
                    },
                }
        except Exception as exc:
            logger.debug("Color fallback error: %s", exc)

    # ---- 全部失败 ----
    logger.info("No drug detected (barcode, OCR, color all failed)")
    return no_result


def get_drug_sku_table() -> List[Dict[str, Any]]:
    """返回药品 SKU 映射表（只读副本）。"""
    return list(DRUG_SKU_TABLE)


# ============ 独立测试 ============
def _generate_test_barcode_image() -> np.ndarray:
    """生成条码模拟图像（用于自测）。"""
    img = np.ones((200, 400, 3), dtype=np.uint8) * 255
    cv2.putText(img, "690123456789", (20, 100),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0), 2)
    for i in range(20):
        x = i * 18 + 20
        cv2.rectangle(img, (x, 30), (x + 8, 180),
                      (0, 0, 0) if i % 2 == 0 else (255, 255, 255), -1)
    return img


def _generate_test_ocr_image() -> np.ndarray:
    """生成文字模拟图像（用于自测）。"""
    img = np.ones((150, 500, 3), dtype=np.uint8) * 240
    cv2.putText(img, "阿莫西林胶囊", (30, 80),
                cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 0), 2)
    return img


def main() -> None:
    """独立运行测试"""
    logging.basicConfig(
        level=logging.DEBUG,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )
    logger.info("=== drug_detector self-test ===")

    # 测试 1: 空图像
    result = detect_drug(np.zeros((0, 0, 3), dtype=np.uint8))
    logger.info("Test 1 (empty): %s", result["method"])

    result = detect_drug(None)  # type: ignore
    logger.info("Test 2 (None): %s", result["method"])

    # 测试 2: 模拟条码图像
    barcode_img = _generate_test_barcode_image()
    cv2.imwrite("/tmp/test_drug_barcode.png", barcode_img)
    result = detect_drug(barcode_img, stability_frames=1, reset=True)
    logger.info("Test 3 (barcode image): method=%s drug=%s confidence=%.4f",
               result["method"], result["drug_name"], result["confidence"])

    # 测试 3: 模拟 OCR 图像
    ocr_img = _generate_test_ocr_image()
    cv2.imwrite("/tmp/test_drug_ocr.png", ocr_img)
    result = detect_drug(ocr_img, stability_frames=1, reset=True)
    logger.info("Test 4 (OCR image): method=%s drug=%s confidence=%.4f",
               result["method"], result["drug_name"], result["confidence"])

    # 测试 4: 白色空白图像（应全部失败）
    blank = np.ones((300, 400, 3), dtype=np.uint8) * 255
    result = detect_drug(blank, stability_frames=1, reset=True)
    logger.info("Test 5 (blank): method=%s", result["method"])

    # 测试 5: 药品 SKU 表
    logger.info("Test 6: SKU table has %d drugs", len(get_drug_sku_table()))
    for drug in get_drug_sku_table():
        logger.info("  %s: %s (patterns=%d, keywords=%d)",
                   drug["drug_id"], drug["name"],
                   len(drug["barcode_patterns"]),
                   len(drug["keywords"]))

    logger.info("=== self-test done ===")


if __name__ == "__main__":
    main()
