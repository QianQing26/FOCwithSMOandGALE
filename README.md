# FOC with SMO and GALE

🚀 电机控制课设项目 | 基于 STM32 + FOC + 自设计观测器与滤波器结构  
📌 作者：QianQing26   MAI   
📚 本项目在 Workbench 自动生成的 FOC 框架上，集成了滑模观测器（SMO）和多滤波器融合算法（GALE），并提供了完整的仿真、实机测试与文档支持。

---

## 📁 仓库结构

```bash
├── Ytest/                  # 主工程文件
│   ├──MCSDK_v5.Y.4-Full
|   |  └─MotorControl	# mcsdk电机控制文件
│   ├──STM32CubeIDE		# 自己添加的文件（包含SMO、gale、matrix等）
│   └── Src  			# workbench生成的代码 
├── Simulink/               # SMO 仿真模型（Simulink 实现）
├── gale/              # Python 仿真文件（GALE 滤波器 + 信号发生器）
│   ├── gale.py             # GALE 滤波器融合核心
│   ├── welford.py/     # asynchronous dual-core welford
│   └── signalGenerator.py  # 各类速度信号输入生成器
├── Docs/                   # 课设报告（待上传）
└── README.md               # 项目说明文档（本文件）
```

## 🧠 项目背景

本项目旨在实现一个可运行于 STM32 嵌入式平台的矢量控制（FOC）驱动系统，具备如下创新点：

- ✅ 实现滑模观测器（SMO）对电机转速进行估算；
- ✅ 实现 GALE 滤波器对观测结果进行多策略融合；
- ✅ 支持 Kalman、低通、滑动平均、自适应 LMS 滤波器；
- ✅ 引入Asynchronous Dual-Core Welford 算法进行在线均值与方差估计；
- ✅ 支持预测补偿以提升响应性；
- ✅ 提供 Simulink / Python 双平台仿真与实机验证。

------

## ⚙️ 功能简介

### 🎯 滑模观测器（SMO）

- 实时估算电机转速，无需编码器；
- 快速响应，适用于动态控制；
- 噪声较大，通过 GALE 滤波器平滑。

### 🧩 GALE 融合滤波器

- 并行运行多种滤波器（Kalman, LPF, MAF, LMS）；
- 使用异步 Welford 算法进行输出统计；
- 基于高斯似然自适应分配融合权重；
- 可选 Kalman 状态预测补偿，降低滞后。

------

## 🧪 仿真支持

### 🔧 Simulink

- PMSM 模型 + SMO 模块；
- 验证 SMO 对速度输入的估算能力。

### 🧪 Python

- GALE 融合结构仿真；
- 支持多种速度输入（线性、跳变、扫频等）；
- 可视化融合效果对比各子滤波器响应。

------

## 📸 实机测试

- 基于 NUCLEO-G431RB + GMB2804-100T；
- GALE 可对 Luenberger 或 SMO 输出进行滤波；
- 上位机通过串口实时观测滤波效果；
- 实验任务：加减速、突变响应、平稳运行。

------

## ✅ 已完成工作

-  MCSDK 工程搭建与调试
-  滑模观测器嵌入与调试
-  GALE 结构 C 语言部署
-  Python 仿真平台开发
-  实验数据采集与报告撰写

------

## 📌 使用说明

> 若希望在本地运行仿真：

```bash
cd gale
python test.py  # 你可以创建主文件运行不同信号测试
```

> 若希望在 STM32 工程中移植 GALE：

1. 添加 `gale.h`, `filter.h`, `welford.h`,`matrix.h`,`smo.h` 至 `Inc/`；
2. 在 `Src/` 中添加实现文件；
3. 调用 `GALE_Init()`、`GALE_Update(input)`、`GetGALESpeed()` 即可。

------

## 📃 参考与致谢

- [ST Motor Control Workbench](https://www.st.com/en/development-tools/stmc-workbench.html)
- 项目课程组老师与队友协助支持 🙌

------

## 📬 联系方式

如需交流课程设计相关内容或深入探讨滤波结构，请通过 GitHub issues 与我联系！

------

🎓 *本项目作为华中科技大学人工智能与自动化学院2022级计算机控制技术课程设计使用，欢迎参考与学习。*

