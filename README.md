# 🚁 Heterogeneous Co-Simulation Platform
**基于 Unity 与 PyBullet 的空地异构多智能体协同仿真平台**

![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)
![Python](https://img.shields.io/badge/Python-3.8%2B-blue)
![Unity](https://img.shields.io/badge/Unity-2022.3%2B-black)
![PyBullet](https://img.shields.io/badge/Physics-PyBullet-orange)

本项目是一个针对空地异构多智能体（UAV-UGV）系统的高保真协同仿真平台。采用**“计算与渲染解耦”**的数字孪生架构，后端由 Python/PyBullet 提供 240Hz 的严谨刚体动力学解算，前端由 Unity 引擎负责高精度 3D 渲染与实时状态监控，双方通过高频 UDP 协议进行异步双向通信。

> **🎥 核心演示 (System Demonstration)**
> https://github.com/user-attachments/assets/6d7481d7-5f0e-4895-83d3-27e252cfd477


---

## ✨ 核心特性 (Key Features)

- **🖥️ 双引擎解耦架构 (Co-Simulation)**：彻底剥离物理运算与视觉渲染。避免了游戏引擎在复杂空气动力学上的失真，同时赋予了硬核物理仿真极佳的视觉表现力。
- **🕸️ 跨域一致性拓扑控制**：内置 $6 \times 6$ 复杂复拉普拉斯矩阵算法，实现空地跨域智能体的全连通几何编队与动态残差补偿。
- **🛞 严谨的非完整约束动力学**：针对地面 UGV 节点，自主实现了基于航向角误差补偿的双轮差速动力学逆解，杜绝“平移作弊”，还原真实物理运动。
- **🤝 动态握手与自适应生成**：具备高度的工程拓展性，Python 后端可在初始化时自动下发集群规模配置，Unity 前端根据指令全自动实例化对应数量的智能体。

---

## 📸 仿真效果展示 (Gallery)

我们在不同规模和扰动条件下对编队算法进行了充分验证：

| 动态拓扑自适应 (Dynamic Topology) | 复杂地形约束 (Terrain Constraints) |
| :---: | :---: |
| *[在这里放置动图：拖拽单架无人机，编队自动恢复]* <br> `![Demo 1](./docs/demo1.gif)` | *[在这里放置动图：UGV 爬坡与 UAV 高度补偿]* <br> `![Demo 2](./docs/demo2.gif)` |
| **大规模集群编队 (Swarm Control)** | **系统监控面板 (UI Dashboard)** |
| *[在这里放置截图：10+ 智能体的编队展示]* <br> `![Demo 3](./docs/demo3.png)` | *[在这里放置截图：Unity 中的 6-DoF 实时状态监控面板]* <br> `![Demo 4](./docs/demo4.png)` |

---

## ⚙️ 系统架构 (Architecture)

本平台采用了**双环级联闭环控制架构**，宏观位置环进行拓扑误差解算，微观姿态环进行底层 PID 防发散控制。


[Architecture Diagram](./docs/architecture.png)

---

## 🚀 快速开始 (Quick Start)

### 1. 环境依赖准备
- **Python 端**: 推荐使用 Anaconda 创建虚拟环境 (Python >= 3.8)。
- **Unity 端**: 建议使用 Unity Editor `2022.3.x` LTS 或更高版本。

```bash
# 克隆仓库
git clone [https://github.com/YourUsername/Heterogeneous-CoSim-Platform.git](https://github.com/YourUsername/Heterogeneous-CoSim-Platform.git)
cd Heterogeneous-CoSim-Platform

# 安装 Python 物理后端依赖
pip install -r PyBullet_Backend/requirements.txt
```

### 2. 启动数字孪生前端
1. 打开 Unity Hub，点击 `Add project from disk`，选择仓库中的 `3D version_1` 文件夹。
2. 在 Project 面板中打开 `MainScene` 场景。
3. 点击编辑器正上方的 **Play** 按钮，等待前端 UDP 监听服务就绪（默认端口 `5006`）。

### 3. 运行物理后端
提供两种启动方式：

**方式一：一键自动化启动（推荐）**
直接在项目根目录下双击运行 `Run_Simulation.bat` 脚本，系统将自动拉起 Python 环境并建立连接。

**方式二：手动运行指令**
```bash
cd PyBullet_Backend
python main_hetero_sim.py
# 若需运行其他编队算法（如复拉普拉斯矩阵算法）：
# python run_threesimilar.py
```

---

## 📊 数据记录与分析 (Data Logging)

仿真平台内置了数据落盘功能。运行结束后，系统会自动剥离无效的空置步数，将各智能体的真实物理轨迹（$(X, Y, Z)$ 及姿态角）保存至 `drone_trajectories.csv`，可直接用于撰写论文或使用 MATLAB/Python 绘制高精度的轨迹收敛图。

---

## 🎓 致谢 (Acknowledgments)

本项目的基础底层飞控源码部分参考了出色的开源项目 [gym-pybullet-drones](https://github.com/utiasDSL/gym-pybullet-drones)。同时感谢南方科技大学 (SUSTech) 在本项目开发过程中提供的学术指导与支持。

---

**License:** This project is licensed under the MIT License - see the LICENSE file for details.
```

***

### 💡 接下来你可以这样操作：

1. 把之前录制的那些能够体现“物理引擎强大”的录屏，用在线工具（比如 ezgif.com）转成小体积的 `.gif` 文件。
2. 在本地项目文件夹里新建一个 `docs` 文件夹，把动图和架构图放进去。
3. 把这些图片的名字替换到 README 里面对应的 `[]` 位置。
4. 运行 `git add .` -> `git commit -m "Add professional README and media"` -> `git push`。

你的 GitHub 仓库主页将会立刻变得极具学术威慑力！如果你在制作架构图或转录 GIF 的过程中遇到任何问题，随时找我。
