# PharmacyBot 项目全面审计报告

> 审计日期: 2025-05-29
> 审计范围: 7份技术文档 + 5个Python源文件 + 1个架构图 + README.md
> 审计人: Hermes Agent (AI 审计)

---

## 执行摘要

项目整体架构设计合理，文档体系较完整，代码风格基本规范。但存在若干**严重问题**：文档间有多处自相矛盾（最严重的是"本地推理"与"云端API"的前后不一致），缺少医疗AI合规必须的免责声明和隐私保护措施，药物数据来源不明，多个声称存在的模块目录实际为空。以下分四个维度详述。

---

## 一、代码质量审计

### 🔴 严重 — 1.1 异常处理不完整：llm_client.py 吞掉 JSONDecodeError 后的重试逻辑有缺陷

文件: `ai_agent/llm_client.py` L114-126

```python
except (json.JSONDecodeError,) as e:
    logger.warning(...)
    if attempt < self.max_retries - 1:
        time.sleep(2**attempt)
```

当最后一次重试 JSON 解析仍失败时，代码落在 `for` 循环末尾返回 `{"error": "...", "raw": content}`，但此时 `content` 变量可能未定义（如果异常在首次尝试就抛出，从未执行到 `content = resp...`），会引发 `UnboundLocalError`。

**修复建议**: 在 for 循环前初始化 `content = None`，并在最后的 fallback 返回中检查。

### 🟡 中等 — 1.2 缺少日志基础配置

文件: `ai_agent/symptom_agent.py`, `ai_agent/llm_client.py`, `ai_agent/drug_kb.py`

三个文件都使用了 `logging.getLogger(__name__)`，但**没有任何地方调用 `logging.basicConfig()`**。如果作为独立程序运行，日志不会输出到任何地方。需要在入口点（`symptom_agent.py` 的 `__main__` 或 `__init__.py`）设置日志级别和格式。

**修复建议**: 在 `symptom_agent.py` 末尾 `if __name__ == "__main__":` 块中加 `logging.basicConfig(level=logging.INFO, format=...)`。

### 🟡 中等 — 1.3 drug_kb.py 缺少输入校验

文件: `ai_agent/drug_kb.py` L25-74 `load_drugs()` 方法

`load_drugs()` 对 JSON 文件只做了 `json.load(f)`，没有 schema 校验。如果 JSON 缺少 `drug_id`/`name` 必填字段，会在 L53 因为 `drug["drug_id"]` 抛出 `KeyError`，且没有友好错误信息。

**修复建议**: 添加 `jsonschema` 或简单的 `assert` 校验每个 drug 字典的必填字段。

### 🟡 中等 — 1.4 SymptomAgent 的类型守卫写法可疑

文件: `ai_agent/symptom_agent.py` L105-110

```python
# type guard: JSON mode guarantees dict
if isinstance(raw_result, str):
    logger.error("LLM returned string instead of dict ...")
    raw_result = {"diagnosis": "API返回格式异常", ...}

result: dict = raw_result
```

这里 `raw_result` 的类型注解是 `dict | str`（来自 `llm_client.py` 的 `chat()` 返回类型），但变量注解 `result: dict` 只是声明而不会做运行时类型转换。实际上 `isinstance` 的 else 分支已经保证了 `raw_result` 是 dict，这段代码功能上没问题但写法显得多余。另外 `isinstance(raw_result, str)` 这个检查永远不会触发（因为 `json_mode=True` 时要么返回 dict 要么在第 126 行返回 `{"error": ...}`），属于死代码。

**修复建议**: 移除 `isinstance` 检查，直接使用 `raw_result`，或改用 try/except 包装整个流程。

### 🟢 建议 — 1.5 类型注解不统一

- `llm_client.py` L63 返回类型 `-> dict | str` — Python 3.10+ 语法，但与文档中声明的 "Python 3.8" (Ubuntu 20.04 默认) 不兼容。
- `drug_kb.py` L128 `-> dict | None` — 同理。
- `drug_kb.py` L76 返回类型 `-> List[dict]` — 使用了 `List` 而非 `list`（Python 3.9+ 语法），与 Python 3.8 不兼容。

**修复建议**: 如果项目确实使用 Python 3.8，改用 `from __future__ import annotations` 或 `Union[dict, str]`、`Optional[dict]`。

### 🟢 建议 — 1.6 OpenAI SDK timeout 设置可能被忽略

文件: `ai_agent/llm_client.py` L51

```python
self.client = OpenAI(api_key=api_key, base_url=cfg["base_url"], timeout=timeout)
```

`config.py` 中设置了 `API_TIMEOUT=15`，但 `LLMClient.__init__` 的 `timeout` 参数没有从 config 导入，完全取决于调用方传入。而 `SymptomAgent.__init__` (L49) 只传了 `provider` 给 `LLMClient`，没有传 `timeout` 和 `max_retries`，导致使用了 `LLMClient` 的默认值（`timeout=15, max_retries=3`），config.py 中的设置未被使用。

**修复建议**: 在 `LLMClient.__init__` 中从 config 读取超时和重试配置，或由 `SymptomAgent` 显式传递。

### 🟢 建议 — 1.7 缺少 .env 支持

`config.py` 和 `llm_client.py` 都依赖 `os.getenv()` 读取环境变量，但没有使用 `python-dotenv` 加载 `.env` 文件。文档 `06_AI推荐系统.md` L370 提到 `.env` 文件，但代码中未实现。

---

## 二、技术文档完整性审计

### 🔴 严重 — 2.1 "离线本地推理"与"云端API调用"的系统性矛盾

这是本项目文档中**最严重的一致性问题**。以下四处互相矛盾：

| 文件 | 位置 | 内容 | 矛盾点 |
|------|------|------|--------|
| docs/01_系统架构设计.md | L41, L142 | "全链路离线运行，所有AI推理均本地执行" | 与云端API矛盾 |
| docs/01_系统架构设计.md | L67 | "离线可用性: 全流程离线运行 ✅ 基于本地推理" | 关键指标造假 |
| docs/architecture.html | L413 | "Ollama 本地推理，Qwen2.5" | 与实际API调用矛盾 |
| README.md | L63 | "DeepSeek/Qwen API + RAG (云端)" | 与上面三处矛盾 |
| docs/06_AI推荐系统.md | L48-56 | "为什么用 API 而不是本地部署？" 详细对比 | 明确说用API |
| ai_agent/实际代码 | llm_client.py | `from openai import OpenAI` → 云端API | 与"离线本地推理"矛盾 |

**分析**: docs/06 已经明确承认是云端 API 方案，并给出了充分理由（模型更大、推理更快、本地GPU省出来跑视觉），这是正确的技术决策。但 docs/01 和 architecture.html **没有更新**，仍然沿用过时的"本地 Ollama"描述。在答辩场景中，评委若细看 docs/01 的关键指标表 "离线可用性: ✅" 再对照实际代码，会直接质疑数据的真实性。

**修复建议**: 
1. docs/01: 将 "全链路离线运行" 改为 "AI推理走云端API，本地保留离线降级规则引擎"
2. docs/01 关键指标表: 离线可用性改为 "部分离线（语音+视觉+控制离线；AI诊断需网络）"
3. architecture.html: "Ollama 本地推理" → "云端 LLM API (DeepSeek/Qwen)"
4. README.md L62: 统一描述为 "云端 LLM API"

### 🟡 中等 — 2.2 docs/03 软件架构说明的ROS包数量矛盾

docs/03 说 "包含 7 个 ROS 包"，但实际列出表中有 8 个（含 dataset_capture），READMe 说 "16个包"。三个数字不一致。应该统一为实际数量（16个ROS包，但文档只详述了核心8个）。

### 🟡 中等 — 2.3 docs/02 硬件选型: STM32 vs ESP32 混用

L44行: "主控: ESP32 Mini Controller"，标题却写 "STM32 控制板"。STM32 和 ESP32 是不同厂商的芯片，不能混用。需要明确究竟是哪个。

### 🟡 中等 — 2.4 架构图缺失关键差异

docs/architecture.html L41云: "碰撞检测 + 力控安全 → 人机协作保护"

这是 JetArm 的通用描述，但 JetArm 规格中并未提到力控传感器。docs/02 硬件清单中也没有力矩传感器项。此处描述可能超出实际硬件能力，存在夸大。

### 🟡 中等 — 2.5 docs/07 药物管理流程的补仓功能未标注待砍

用户备注中提到 "补仓/库存更新功能因缺少移动底盘已被标记为待砍掉"，但 docs/07 全文仍然以补仓告警作为核心闭环流程描述，docs/01 的六层架构也把"补仓告警"列为 L6 核心功能。需要在文档中明确标注该功能范围的变更。

### 🟢 建议 — 2.6 关键技术指标缺乏实测数据支撑

docs/01 的关键指标表 6/7 项标注为"需实测"或"待训练"，但一些指标本身就存在问题：

| 指标 | 声明 | 问题 |
|------|------|------|
| AI诊断准确率 ≥90% (Top-3) | 标注 "RAG检索+LLM推理" | 从未测试，且Top-3准确率在仅5种药物时无意义 |
| 端到端延迟 ≤15s | 标注 "需实测" | 云端API+视觉+运动，15s 非常乐观 |
| 抓取成功率 ≥95% | 标注 "基于MoveIt规划" | 固定药架场景可行，但需要实际测试 |

建议: 标注为"预期目标"而非"当前状态"，避免被评委质疑。

### 🟢 建议 — 2.7 README.md 的快速开始指向不存在的目录和文件

```bash
cd src && catkin_make    # src/ 是ROS工作空间，catkin_make 需要在 src/ 上层执行
cd ../ai_agent && python symptom_agent.py  # symptom_agent.py 没有 __main__ 入口
```

---

## 三、伦理与法律合规审计

### 🔴 严重 — 3.1 完全缺失医疗免责声明

搜索整个项目，**没有任何文件包含医疗免责声明**。这是医疗AI项目最致命的合规缺失。

对于中国的 AI 药物推荐系统，必须有：
- "本系统建议仅供参考，不构成医疗诊断，请遵医嘱"
- "用药前请仔细阅读药品说明书"
- 明确声明系统不是医疗器械，不能替代医师诊断

参考法规: 《医疗器械监督管理条例》(2021修订)、《人工智能医用软件产品分类界定指导原则》

**修复建议**: 在每个用户触达点（语音播报、管理后台、README、技术文档）增加医疗免责声明模板。

### 🔴 严重 — 3.2 患者隐私保护缺失

docs/06 L382 提到 "症状数据脱敏后再发送到云端（移除姓名等 PII）"，但：
1. 代码中完全没有实现数据脱敏逻辑
2. `symptom_agent.py` 直接将原始 `symptom_text` 发送给云端 API
3. 没有数据最小化原则 — 症状描述中的年龄、性别、病史等敏感信息原样外传
4. 没有隐私政策文档

**修复建议**: 
1. 在 `symptom_agent.diagnose()` 中增加脱敏预处理步骤
2. 增加隐私政策文档 (PRIVACY.md)
3. 对云端传输的 prompt 进行字段化（只发症状关键词，不发原始自由文本）

### 🔴 严重 — 3.3 药物知识库数据来源不明，存在法律风险

文件: `ai_agent/drugs_sample.py`

5种药物数据（阿莫西林、布洛芬、对乙酰氨基酚、头孢克肟、复方甘草片）的适应症、禁忌、剂量信息**来源不明**。在技术资料中，这是一个非常危险的信号：

1. 中国法律严格限制药物信息的随意发布
2. 药物知识库的数据应当来自国家药品监督管理局（NMPA）或权威药品说明书
3. 如果 AI 基于不准确的药物数据给出错误推荐，可能导致严重后果

**修复建议**:
1. 在 `drugs_sample.py` 和所有药物数据文件顶部加声明: "本数据仅供技术演示，不用于实际诊断。药物信息来源：国家药品监督管理局药品数据库"
2. 如果用于实际系统，必须接入 NMPA 官方药品数据库或授权药典数据

### 🟡 中等 — 3.4 项目根目录缺少 LICENSE 文件

README.md 声明 "License: MIT"，但 `/home/darcy/pharmacy_bot/` 目录下没有 `LICENSE` 文件。`src/third_party/` 子目录下有 Apache 2.0 许可证文件（ros_astra_camera 和 apriltag_ros），但：
1. 项目根目录的 MIT 声明没有正式 LICENSE 文件支撑
2. 未区分"项目自研代码的 MIT 许可"与"第三方代码的 Apache 2.0 许可"
3. 讯飞离线 SDK 的许可证条款完全未提及

**修复建议**:
1. 创建 `/home/darcy/pharmacy_bot/LICENSE` (MIT)
2. 创建 `/home/darcy/pharmacy_bot/NOTICE` 声明第三方依赖及其许可证
3. 确认讯飞 SDK 的分发许可条款

### 🟡 中等 — 3.5 中国医疗器械AI法规合规性完全未涉及

项目文档中**零处**提及以下相关法规：
- 《医疗器械监督管理条例》(国务院令第739号)
- 《人工智能医疗器械注册审查指导原则》
- 《药品网络销售监督管理办法》

对于技术资料提交用项目（竞赛/答辩），评委中如果有医疗背景专家，此缺失会直接导致扣分。

**修复建议**: 在 docs/01 或单独的合规文档中增加一个章节，说明项目的合规边界：
- 本项目为**技术原型/教学演示**，非医疗器械
- 所有药物推荐仅供研究参考
- 如未来产品化，需申请医疗器械注册证

### 🟡 中等 — 3.6 API Key 管理风险

文件: `docs/06_AI推荐系统.md` L370-374

```bash
export DEEPSEEK_API_KEY="sk-xxxxxxxx"
export DASHSCOPE_API_KEY="sk-xxxxxxxx"
```

虽然代码中用了 `os.getenv()`（正确做法），但文档示例直接展示了 API key 的环境变量设置，可能误导用户在脚本中硬编码。且 `.env` 文件未被 `.gitignore` 显式排除。

**修复建议**: 
1. 文档示例中改用占位符: `export DEEPSEEK_API_KEY="your-key-here"`
2. 确认 `.gitignore` 包含 `.env`、`*.key`
3. 检查是否有 `.gitignore` 文件

---

## 四、综合建议与改进优先级

### 改进优先级排序

| 优先级 | 问题编号 | 问题简述 | 严重程度 | 预计修复时间 |
|--------|----------|----------|----------|-------------|
| P0 | 2.1 | 文档中"离线推理"与"云端API"矛盾 | 🔴 | 30分钟 |
| P0 | 3.1 | 缺失医疗免责声明 | 🔴 | 20分钟 |
| P0 | 3.2 | 患者隐私保护缺失 | 🔴 | 2小时(代码) |
| P0 | 3.4 | 缺少LICENSE文件 | 🔴 | 10分钟 |
| P1 | 3.3 | 药物数据来源不明 | 🔴 | 1小时 |
| P1 | 1.1 | JSONDecodeError 后 content 未绑定 | 🔴 | 5分钟 |
| P1 | 3.5 | 医疗器械法规未提及 | 🟡 | 1小时 |
| P1 | 2.5 | 补仓功能范围未标注待砍 | 🟡 | 15分钟 |
| P2 | 2.3 | STM32/ESP32 混用 | 🟡 | 5分钟 |
| P2 | 2.2 | ROS包数量矛盾 | 🟡 | 10分钟 |
| P2 | 1.3 | drug_kb 缺少输入校验 | 🟡 | 15分钟 |
| P2 | 1.2 | 缺少日志基础配置 | 🟡 | 10分钟 |
| P2 | 3.6 | API Key管理风险 | 🟡 | 15分钟 |
| P3 | 1.4 | SymptomAgent 类型守卫死代码 | 🟢 | 5分钟 |
| P3 | 1.5 | 类型注解与Python 3.8兼容性 | 🟢 | 20分钟 |
| P3 | 1.6 | timeout配置未生效 | 🟢 | 5分钟 |
| P3 | 1.7 | 缺少 .env 支持 | 🟢 | 10分钟 |
| P3 | 2.7 | README 快速开始命令不正确 | 🟢 | 10分钟 |

---

## 五、可执行的修复建议

### 修复 1: docs/01 统一为云端API方案 (P0)

文件: `docs/01_系统架构设计.md`

```diff
- 核心创新点：
- - **AI 大模型临床推理**: 调用云端 LLM API（DeepSeek / Qwen），基于药物知识库做 RAG 增强检索
+ - **AI 大模型临床推理**: 调用云端 LLM API（DeepSeek / Qwen），基于药物知识库做 RAG 增强检索。本地不运行大模型推理，节省 Jetson GPU 算力用于视觉和运动控制。

- 2. 全链路离线运行: 所有 AI 推理（ASR、LLM、YOLO、OCR）均本地执行，无需云端，保障药房数据隐私
+ 2. 混合架构: 语音、视觉、运动控制在 Jetson 本地执行；LLM 推理走云端 API（更快、模型更大），网络不可用时降级为本地规则引擎

- | 离线可用性 | 全流程离线运行 | ✅ 基于本地推理 |
+ | 离线可用性 | 部分离线 | 语音+视觉+控制本地；AI诊断需网络（离线时降级规则引擎） |
```

### 修复 2: llm_client.py 修复 UnboundLocalError (P1)

文件: `ai_agent/llm_client.py`

在第 95 行 `for attempt` 前添加:

```python
content = None  # 防止 JSON 解析失败时 UnboundLocalError
```

### 修复 3: 创建 LICENSE 文件 (P0)

新建文件: `/home/darcy/pharmacy_bot/LICENSE`

```text
MIT License

Copyright (c) 2025 PharmacyBot Team

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

### 修复 4: 增加医疗免责声明 (P0)

新建文件: `/home/darcy/pharmacy_bot/docs/DISCLAIMER.md`

建议内容:
- 声明本项目为技术原型/教学演示
- 药物推荐"仅供参考，不构成医疗诊断"
- "请遵医嘱，用药前仔细阅读药品说明书"
- "本系统不是医疗器械，不能替代执业医师诊断"
- 数据隐私说明

### 修复 5: architecture.html 更新卡片内容 (P0)

文件: `docs/architecture.html` L411-418

```diff
- <li>• Ollama 本地推理，Qwen2.5 / DeepSeek-Coder 双模型</li>
+ <li>• 云端 LLM API（DeepSeek / Qwen），OpenAI SDK 兼容</li>
```

### 修复 6: drugs_sample.py 增加数据来源声明 (P1)

文件: `ai_agent/drugs_sample.py`

在文件头部增加:

```python
# ⚠️ 免责声明
# 本文件中的药物数据仅供技术演示和原型验证，不用于实际诊断。
# 药物信息来源于公开药品说明书，可能不完整或已过时。
# 实际系统必须接入国家药品监督管理局(NMPA)官方数据库。
```

### 修复 7: symptom_agent.py 增加隐私脱敏 (P0)

在 `diagnose()` 方法 L83 后添加一个预处理步骤:

```python
def _sanitize_input(self, text: str) -> str:
    """移除可能的PII信息"""
    import re
    # 移除姓名模式（简单规则）
    text = re.sub(r'[我姓叫名字是]([\u4e00-\u9fa5]{2,4})', '[患者]', text)
    # 移除电话号码
    text = re.sub(r'1[3-9]\d{9}', '[电话已隐藏]', text)
    # 移除身份证号
    text = re.sub(r'\d{17}[\dXx]', '[证件号已隐藏]', text)
    return text
```

在 `diagnose()` L86 `candidates = self.kb.retrieve(symptom_text, k=5)` 之前调用 `symptom_text = self._sanitize_input(symptom_text)`。

### 修复 8: README.md 修复快速开始命令 (P3)

```diff
- cd src && catkin_make
+ cd src && catkin_make    # 注意: 需要在 src/ 目录内执行

- cd ../ai_agent && python symptom_agent.py
+ cd ../ai_agent && python -m ai_agent.symptom_agent
```

---

## 总结

项目核心架构 (语音→LLM→视觉→运动→分拣) 设计合理，代码基本可用，但**文档一致性和医疗合规性是当前最大短板**。在答辩/评审场景下，评委最可能质疑的三个点是：

1. "你们到底是在本地跑 LLM 还是调用云端 API？文档为什么前后矛盾？" (公信力问题)
2. "医疗AI的合规性你们是如何考虑的？免责声明在哪里？" (法律风险)
3. "药物知识库数据从哪里来的？能不能保证准确性？" (数据可信度)

建议在提交前优先解决 P0 和 P1 问题（预计总工时 4-5 小时）。
