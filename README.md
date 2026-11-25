# Serendipity LLM for Robotics

## 普通桌面回放（VS Code Remote SSH）

项目的开发目录是服务器上的 /home/bj2410/projects/humanoid_manipulation，后续代码和仿真实验直接在这个 VS Code Remote SSH 工作区完成。

在 VS Code 的远程终端运行以下命令。若回放已在运行，直接打开浏览器链接。

```bash
cd /home/bj2410/projects/humanoid_manipulation/sim_workspace
conda activate simulator
OPENBLAS_NUM_THREADS=1 MUJOCO_GL=egl python workspace.py replay   runs/flat-tabletop-adaptive-evaluation/episode_007 --port 8792
```

在 VS Code 的 Ports 面板把 8792 转发到本机，打开 [蓝方块放入黄托盘的回放](http://127.0.0.1:8792/)。这是已有的成功运行记录。重新运行普通桌面任务请使用仿真工作区文档中的 lab-trial，策略权重已放在 sim_workspace/.envs/g1-velocity.onnx。


NYU Abu Dhabi 服务器上的 Go2 与 G1 仿真项目。打开本目录作为 VS Code Remote SSH 工作区。

| 你要做什么 | 从这里开始 |
| --- | --- |
| 跑仿真、回放或启动 benchmark | [仿真工作区](sim_workspace/README.md) |
| 查看 Go2 到 G1 的交接字段和失败状态 | [交接协议](docs/HANDOFF.md) |
| 定位一次失败，找到对应源码并复测 | [排错手册](docs/TROUBLESHOOTING.md) |
| 查阅或补记改动、证据和未解决问题 | [工程笔记](docs/ENGINEERING_LOG.md) |
| 核对原 G1 环境、Dex3-1 控制实验 | [原环境说明](docs/environment_setup_and_visualization.md) · [Dex3-1 开发日志](docs/dex3-1_control_development_log.md) |

- `sim_workspace/robots/go2/` 包含 Go2 模型入口、四腿 IK、RGB-D 相机和搜索定位。
- `sim_workspace/robots/g1/` 包含 G1 模型入口、双腿和手臂 IK、预抓取、模型检查和 Dex3-1 独立查看器。
- `sim_workspace/` 提供共享场景、交接流程、通用 IK、benchmark 入口及 `runs/` 运行产物。
- `third_party/` 存放 Unitree 上游模型。项目级参数和集成逻辑在 `sim_workspace/`。

## 浏览器场景编辑器

使用 [场景编辑器指南](sim_workspace/README.md#浏览器场景编辑器) 在服务器上拖放机器人、添加物体和导入资产。
