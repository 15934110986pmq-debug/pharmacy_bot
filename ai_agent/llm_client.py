"""
多Provider LLM API客户端 — OpenAI SDK兼容
支持: DeepSeek / Qwen (DashScope) / 任意OpenAI兼容API
"""
import os
import json
import time
import logging
from openai import OpenAI

logger = logging.getLogger(__name__)

PROVIDERS = {
    "deepseek": {
        "base_url": "https://api.deepseek.com/v1",
        "model": "deepseek-chat",
        "env_key": "DEEPSEEK_API_KEY",
    },
    "qwen": {
        "base_url": "https://dashscope.aliyuncs.com/compatible-mode/v1",
        "model": "qwen-turbo",
        "env_key": "DASHSCOPE_API_KEY",
    },
    "qwen-plus": {
        "base_url": "https://dashscope.aliyuncs.com/compatible-mode/v1",
        "model": "qwen-plus",
        "env_key": "DASHSCOPE_API_KEY",
    },
}


class LLMClient:
    """LLM API 客户端，默认使用 DeepSeek"""

    def __init__(self, provider="deepseek", max_retries=3, timeout=15):
        if provider not in PROVIDERS:
            raise ValueError(f"Unknown provider: {provider}. Available: {list(PROVIDERS.keys())}")

        self.provider = provider
        self.max_retries = max_retries
        self.timeout = timeout

        cfg = PROVIDERS[provider]
        api_key = os.getenv(cfg["env_key"])
        if not api_key:
            raise RuntimeError(
                f"Missing API key. Set {cfg['env_key']} environment variable.\n"
                f"  export {cfg['env_key']}=your_api_key_here"
            )

        self.client = OpenAI(api_key=api_key, base_url=cfg["base_url"], timeout=timeout)
        self.model = cfg["model"]
        logger.info(f"LLMClient initialized: provider={provider}, model={self.model}")

    def chat(
        self,
        system_prompt: str,
        user_input: str,
        context: str = "",
        json_mode: bool = True,
        temperature: float = 0.3,
        max_tokens: int = 2048,
    ) -> dict | str:
        """
        发送对话请求到云端LLM API

        Args:
            system_prompt: 系统提示词（角色设定）
            user_input: 用户输入（症状描述）
            context: RAG检索到的药物知识上下文
            json_mode: 是否要求JSON结构化输出
            temperature: 采样温度
            max_tokens: 最大输出token数

        Returns:
            dict: JSON模式下的解析结果
            str: 非JSON模式下的原始文本
        """
        messages = [{"role": "system", "content": system_prompt}]

        if context:
            messages.append({"role": "assistant", "content": f"[检索到的药物知识]\n{context}"})

        messages.append({"role": "user", "content": user_input})

        kwargs = {
            "model": self.model,
            "messages": messages,
            "temperature": temperature,
            "max_tokens": max_tokens,
        }
        if json_mode:
            kwargs["response_format"] = {"type": "json_object"}

        for attempt in range(self.max_retries):
            try:
                start = time.monotonic()
                resp = self.client.chat.completions.create(**kwargs)
                elapsed = time.monotonic() - start

                content = resp.choices[0].message.content
                usage = resp.usage

                logger.info(
                    f"API call success: provider={self.provider}, model={self.model}, "
                    f"elapsed={elapsed:.1f}s, "
                    f"tokens_in={usage.prompt_tokens}, tokens_out={usage.completion_tokens}"
                )

                if json_mode:
                    return json.loads(content)
                return content

            except (json.JSONDecodeError,) as e:
                logger.warning(f"JSON parse failed (attempt {attempt+1}/{self.max_retries}): {e}")
                if attempt < self.max_retries - 1:
                    time.sleep(2**attempt)

            except Exception as e:
                logger.error(f"API call failed (attempt {attempt+1}/{self.max_retries}): {e}")
                if attempt < self.max_retries - 1:
                    time.sleep(2**attempt)
                else:
                    raise RuntimeError(f"API call failed after {self.max_retries} attempts") from e

        return {"error": "failed to parse JSON response", "raw": content}
