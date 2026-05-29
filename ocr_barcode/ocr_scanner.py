"""
ocr_scanner.py — PaddleOCR 药品名识别（轻量版）

功能:
  - 使用 PaddleOCR 2.x 轻量版 (ch_ppocr_mobile_v2.0) 识别图像中的文字
  - 过滤置信度 < 0.5 的结果
  - 返回结构化文字检测结果

依赖:
  - paddleocr (paddlepaddle + paddleocr)
  - opencv-python
  - numpy
"""

from __future__ import annotations

import logging
import os
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np

logger = logging.getLogger(__name__)

# ---- 全局 OCR 引擎（懒加载） ----
_OCR_ENGINE: Any = None
_OCR_INITIALIZED: bool = False


def _get_ocr_engine() -> Any:
    """
    懒加载并缓存 PaddleOCR 引擎实例。
    使用 ch_ppocr_mobile_v2.0 轻量版以节省内存。
    """
    global _OCR_ENGINE, _OCR_INITIALIZED

    if _OCR_INITIALIZED:
        return _OCR_ENGINE

    try:
        from paddleocr import PaddleOCR  # type: ignore[import-untyped]

        logger.info("Initializing PaddleOCR (ch_ppocr_mobile_v2.0)...")
        _OCR_ENGINE = PaddleOCR(
            use_angle_cls=True,  # 方向分类器提升竖排文字识别
            lang="ch",
            # 轻量版默认即为 mobile 版，显式指定以确保
            det_model_dir=None,   # 使用默认下载路径
            rec_model_dir=None,
            cls_model_dir=None,
            # 减少并发以提高稳定性
            use_gpu=False,
            gpu_mem=512,
            # 限定检测阈值减少误检
            det_db_thresh=0.3,
            det_db_box_thresh=0.5,
            # 日志级别
            show_log=False,
        )
        _OCR_INITIALIZED = True
        logger.info("PaddleOCR initialized successfully")
    except ImportError:
        logger.warning(
            "paddleocr not installed. OCR calls will return empty results. "
            "Install with: pip install paddlepaddle paddleocr"
        )
        _OCR_ENGINE = None
        _OCR_INITIALIZED = True
    except Exception as exc:
        logger.error("Failed to initialize PaddleOCR: %s", exc)
        _OCR_ENGINE = None
        _OCR_INITIALIZED = True

    return _OCR_ENGINE


def _preprocess_ocr(cv2_image: np.ndarray) -> np.ndarray:
    """
    OCR 预处理：提升文字对比度。
    - 可选：CLAHE 增强局部对比度
    - 保持彩色（PaddleOCR 内部会自己转灰度）
    """
    try:
        lab = cv2.cvtColor(cv2_image, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        l = clahe.apply(l)
        enhanced = cv2.merge([l, a, b])
        result = cv2.cvtColor(enhanced, cv2.COLOR_LAB2BGR)
        return result
    except Exception as exc:
        logger.debug("OCR preprocessing failed, using original: %s", exc)
        return cv2_image


def _paddleocr_result_to_dicts(
    raw_result: Any,
) -> List[Dict[str, Any]]:
    """
    将 PaddleOCR 原始结果转换为统一 dict 列表。
    过滤置信度 < 0.5 的结果。

    PaddleOCR 返回结构:
      [ [[x1,y1],[x2,y2],[x3,y3],[x4,y4]], (text, confidence) ]
    """
    results: List[Dict[str, Any]] = []

    if not raw_result or not isinstance(raw_result, list):
        return results

    for item in raw_result:
        try:
            if not isinstance(item, (list, tuple)) or len(item) < 2:
                continue
            box, text_info = item[0], item[1]

            if not isinstance(text_info, (list, tuple)) or len(text_info) < 2:
                continue

            text = str(text_info[0]).strip()
            confidence = float(text_info[1])

            if confidence < 0.5:
                continue
            if not text:
                continue

            # box: [[x1,y1],[x2,y2],[x3,y3],[x4,y4]]
            box_float = [[float(p[0]), float(p[1])] for p in box] if box else []

            results.append({
                "text": text,
                "confidence": round(confidence, 4),
                "box": box_float,
            })
        except (IndexError, ValueError, TypeError) as exc:
            logger.debug("Skipping malformed OCR result: %s", exc)
            continue

    return results


def scan_text(cv2_image: np.ndarray,
              preprocess: bool = True) -> List[Dict[str, Any]]:
    """
    扫描图像中的文字，使用 PaddleOCR 轻量版。

    Args:
        cv2_image: OpenCV BGR numpy 数组图像。
        preprocess: 是否对图像进行 CLAHE 对比度增强预处理。

    Returns:
        list[dict]: 每个 dict 包含 {text, confidence, box}。
                    空列表表示未检测到文字（或引擎不可用）。
    """
    if cv2_image is None or cv2_image.size == 0:
        logger.warning("scan_text: received empty image")
        return []

    # 检查图像尺寸下限
    h, w = cv2_image.shape[:2]
    if h < 20 or w < 20:
        logger.debug("scan_text: image too small (%dx%d)", w, h)
        return []

    engine = _get_ocr_engine()
    if engine is None:
        logger.warning("PaddleOCR engine unavailable")
        return []

    try:
        # 可选预处理
        img_to_use = _preprocess_ocr(cv2_image) if preprocess else cv2_image

        # PaddleOCR 支持传入 numpy 数组
        raw = engine.ocr(img_to_use, cls=True)

        # PaddleOCR v2.x 返回 list[list[...]]，取第一个（单图）
        if raw and isinstance(raw, list) and len(raw) > 0:
            page_result = raw[0]
        else:
            page_result = raw

        results = _paddleocr_result_to_dicts(page_result)

        # 按置信度降序
        results.sort(key=lambda x: x["confidence"], reverse=True)
        return results

    except Exception as exc:
        logger.error("scan_text: PaddleOCR inference error: %s", exc)
        return []


# ============ 独立测试 ============
def _generate_test_image(text: str = "阿莫西林胶囊") -> np.ndarray:
    """生成测试用简单文字图像。"""
    img = np.ones((100, 400, 3), dtype=np.uint8) * 240
    cv2.putText(img, text, (20, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 2)
    # 加一点噪点模拟真实环境
    noise = np.random.randint(0, 15, img.shape, dtype=np.uint8)
    cv2.add(img, noise, dst=img)
    return img


def main() -> None:
    """独立运行测试"""
    logging.basicConfig(
        level=logging.DEBUG,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )
    logger.info("=== ocr_scanner self-test ===")

    # 测试空输入
    logger.info("Test 1: empty image -> %s", scan_text(np.zeros((0, 0, 3), dtype=np.uint8)))
    logger.info("Test 2: None input -> %s", scan_text(None))  # type: ignore

    # 测试模拟图像
    test_img = _generate_test_image("布洛芬缓释胶囊")
    cv2.imwrite("/tmp/test_ocr.png", test_img)
    logger.info("Test 3: simulated text image saved to /tmp/test_ocr.png")

    result = scan_text(test_img, preprocess=True)
    if result:
        for r in result:
            logger.info("  OCR: '%s' conf=%.4f", r["text"], r["confidence"])
    else:
        logger.info("  (No OCR results — expected if paddleocr not installed)")

    # 测试小图像
    small_img = np.ones((10, 10, 3), dtype=np.uint8) * 128
    logger.info("Test 4: small image -> %s", scan_text(small_img))

    logger.info("=== self-test done ===")


if __name__ == "__main__":
    main()
