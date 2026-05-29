# PharmacyBot — AI 药房机械臂系统

> 语音识别 → AI症状诊断与药物推荐 → 视觉定位 → 机械臂分拣 → 补仓告警

## 项目简介

构建面向药房的智能药物分拣与推荐系统。以 NVIDIA Jetson Orin NX 为主控计算平台，集成科大讯飞离线语音识别、Ollama 大语言模型推理、YOLOv8+PaddleOCR+pyzbar 多模态视觉感知和 JetArm 六轴机械臂运动控制，实现全链路自动化。

## 系统架构

```
[患者语音/条码] → [AI Agent] → [药物推荐] → [视觉定位] → [运动规划] → [物理分拣]
```

完整架构图：`docs/architecture.html`（浏览器打开）

## 目录结构

```
pharmacy_bot/
├── docs/                    # 技术文档 (7份)
│   ├── 01_系统架构设计.md
│   ├── 02_硬件选型说明.md
│   ├── 03_软件架构说明.md
│   ├── 04_视觉识别方案.md
│   ├── 05_语音交互方案.md
│   ├── 06_AI推荐系统.md
│   ├── 07_药物管理流程.md
│   └── architecture.html
├── src/                     # ROS 工作空间 (16个包)
│   ├── hiwonder_grasp/      # 抓取轨迹规划
│   ├── hiwonder_imgproc/    # 颜色检测 + YOLOv5目标检测
│   ├── hiwonder_interfaces/ # 自定义ROS消息定义
│   ├── hiwonder_transform/  # 坐标变换(抓取偏航角计算)
│   ├── jetarm_6dof/         # 六轴机械臂控制(含颜色分拣)
│   ├── jetarm_6dof_simulate/# Gazebo仿真(模型+描述文件)
│   ├── jetarm_bringup/      # 启动配置 + 中文语音
│   ├── jetarm_driver/       # ROS机器人驱动控制器
│   ├── jetarm_example/      # 入门示例程序
│   ├── jetarm_peripherals/  # 外设控制(图像同步等)
│   ├── lab_config/          # 实验配置管理(含UI)
│   ├── stepper/             # 滑轨步进电机控制
│   ├── third_party/         # 三方驱动(如Astra相机)
│   ├── vision_utils/        # 视觉工具函数
│   ├── xf_mic_asr_offline/  # 讯飞离线语音识别SDK
│   └── dataset_capture/     # 数据集采集工具(Qt GUI)
├── ai_agent/                # AI大模型智能体 (待开发)
├── ocr_barcode/             # 视觉识别模块 (待开发)
├── models/                  # 模型权重
├── scripts/                 # 部署脚本
└── README.md
```

## 核心技术栈

| 层级 | 技术 |
|------|------|
| 主控 | NVIDIA Jetson Orin NX (32 TOPS) |
| OS | Ubuntu 20.04 + JetPack 5.1 |
| 中间件 | ROS Noetic + MoveIt |
| 语音 | 科大讯飞离线ASR (xf_mic_asr_offline) |
| AI推理 | Ollama + Qwen2.5/DeepSeek-Coder + RAG |
| 视觉 | YOLOv8 TensorRT + PaddleOCR + pyzbar |
| 控制 | STM32 + JetArm 6-DOF 机械臂 |
| 管理 | Flutter Web 管理后台 |

## 快速开始

```bash
# 环境
sudo apt install ros-noetic-desktop-full ros-noetic-moveit
pip install ollama paddleocr pyzbar llama-index chromadb

# 编译 ROS 工作空间
cd src && catkin_make

# 启动机械臂
roslaunch jetarm_bringup bringup.launch

# 启动 AI Agent
cd ../ai_agent && python symptom_agent.py
```

## 资料引用

本项目的机械臂控制、语音识别、视觉基础代码来自 **JXB JetArm 配套资料**：
- 教程资料: `1.教程资料/` (13个课程系列)
- 软件工具: `2.软件工具/`
- 源码资料: `3.源码资料/src.zip` (已解压到本项目 src/)
- 硬件资料: `4.硬件资料/`
- 系统镜像: `5.系统镜像/`
- 拓展资料: `6.拓展资料/`
- 主板文件: `7.主板文件/` (Jetson Orin NX 完整教材)

## License

MIT
