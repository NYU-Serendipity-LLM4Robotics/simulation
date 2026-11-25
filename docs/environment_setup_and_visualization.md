# Humanoid Manipulation Pipeline (Unitree G1)

基于 **Unitree G1 + MuJoCo + Python** 的人形机器人抓取与放置（Pick-and-Place）项目。所有模型、代码和仿真都放在 **NYUAD 远程服务器**上运行；Mac 通过 **SSH 端口转发 + 浏览器**实时查看仿真，无需把模型复制到本地，也无需 XQuartz。

> 当前阶段：**已搭建基本环境、获取官方 G1 模型、成功加载模型，并配置 Web Viewer。** IK、可动夹爪、轨迹规划和完整 pick-and-place 尚待开发。以实际服务器文件和运行结果为准。

## 1. 项目目标

从给定的物体位姿出发，完成：

1. 配置 G1 机器人模型、关节限位和末端执行器（夹爪/灵巧手）。
2. 坐标变换、抓取位姿选择和逆运动学（IK）。
3. 考虑关节约束和碰撞的接近轨迹规划。
4. 执行 **Approach → Grasp → Lift → Transfer → Place → Release**。
5. 检验任务是否成功，并测试不可达、碰撞、漏抓、物体掉落等失败情况。

**最终交付：** 可执行的 manipulation pipeline、G1 专用模型/配置，以及抓取与放置测试。

## 2. 服务器与运行环境

| 项目 | 当前配置 |
| --- | --- |
| SSH 用户 | `bj2410` |
| 服务器地址（当前使用） | `10.228.8.244`（需位于授权网络/VPN） |
| 操作系统 | Ubuntu 20.04.6 LTS |
| GPU | 4 × NVIDIA RTX A6000（每张约 48 GB 显存） |
| NVIDIA 驱动 | `550.163.01`；`nvidia-smi` 报告 CUDA 12.4 |
| Conda 安装位置 | `/home/bj2410/miniconda3` |
| 项目环境 | `simulator`（Python 3.10） |
| 项目根目录 | `/home/bj2410/projects/humanoid_manipulation` |

安装环境与项目代码分开存放：

```text
/home/bj2410/
├── miniconda3/
│   └── envs/
│       └── simulator/        # 项目 Python 环境
└── projects/
    ├── humanoid_manipulation/  # 当前项目
    └── <future_project>/      # 其他项目与之并列
```

**磁盘提醒：** 初次检查时，系统盘 `/`（3.6 TB）约剩 133 GB（97% 已用），共享 `/data`（3.5 TB）约剩 39 GB（99% 已用）。这是全服务器剩余空间，不是个人配额；不要未经确认向共享盘下载大型数据集或删除他人的文件。需要时重新运行 `df -h`。

## 3. 当前项目目录

```text
humanoid_manipulation/
├── README.md
├── sim_workspace/robots/g1/
│   └── view_g1.py               # 自己的 Web Viewer 入口
└── third_party/
    └── unitree_mujoco/           # Unitree 官方仓库（稀疏检出）
        └── unitree_robots/
            └── g1/
                ├── scene.xml
                └── ...
```

- `sim_workspace/robots/g1/`：自己编写的仿真、控制和规划代码；后续按需要增加模块。
- `third_party/`：保留上游模型，不直接在其中进行大规模业务修改。
- 后续确实需要定制机器人 MJCF 时，再创建 `models/`，并妥善处理 mesh/include 的相对路径。

## 4. 每次登录服务器

在 **Mac 本地终端**：

```bash
ssh bj2410@10.228.8.244
```

登录后在 **服务器终端**：

```bash
conda activate simulator
cd ~/projects/humanoid_manipulation
python --version
```

预期 Python 主版本为 3.10。可以运行：

```bash
python -m pip show mujoco mjviser viser
```

Web Viewer 不需要 X11 转发，因此这里不用 `ssh -Y`。如果 Conda 没有初始化，可检查 `source ~/.bashrc` 和 `conda env list`。

## 5. G1 官方模型

模型源：[unitreerobotics/unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco)。当前使用的 MJCF 场景：

```text
third_party/unitree_mujoco/unitree_robots/g1/scene.xml
```

最初使用 `git clone --sparse` 遇到了 Git 报错，后来通过 **先克隆但不 checkout，再配置 sparse-checkout** 成功。**仓库已经存在时，不要重复克隆。**

确认文件：

```bash
cd ~/projects/humanoid_manipulation
ls -lah third_party/unitree_mujoco/unitree_robots/g1
```

确认 MuJoCo 能加载：

```bash
python - <<'PY'
from pathlib import Path
import mujoco

path = Path('third_party/unitree_mujoco/unitree_robots/g1/scene.xml')
model = mujoco.MjModel.from_xml_path(str(path))
data = mujoco.MjData(model)
mujoco.mj_forward(model, data)
print('G1 loaded:', model.nbody, 'bodies,', model.njnt, 'joints,', model.nu, 'actuators')
PY
```

**注意：** 能加载 G1 基础场景 ≠ 已经具备可控制的抓取手。实现真实接触抓取前，仍须确认所选 G1 版本、手部关节、执行器和接触几何是否满足任务需求。

## 6. ⭐ 重点：服务器 Web 可视化（mjviser）

### 6.1 为什么使用 Web Viewer？

之前通过 `ssh -Y` 和 XQuartz 已经获得 `DISPLAY=localhost:10.0`，但原生 MuJoCo GUI 启动时出现：

```text
libGL error: No matching fbConfigs or visuals found
libGL error: failed to load driver: swrast
GLX: Failed to create context
ERROR: could not create window
```

原因是远程 X11/GLX 无法建立 MuJoCo 所需的 OpenGL context，**不是 G1 MJCF 加载失败**。现在改用 [mjviser](https://github.com/mujocolab/mjviser)：服务器运行 MuJoCo，Mac 浏览器展示实时 3D 画面，不需要 XQuartz 或本地机器人文件。

使用的是适配 Python 3.10 的 `mjviser==0.0.14`。如需检查或重装依赖：

```bash
conda activate simulator
python -m pip install 'mjviser==0.0.14' 'mujoco>=3.6,<4'
```

### 6.2 服务器端 Viewer 脚本

入口文件：`sim_workspace/robots/g1/view_g1.py`。下面是与当前设置一致的可复现基础版本（如已加入自定义控制器，保留实际项目代码）：

```python
from pathlib import Path

import mujoco
import viser
from mjviser import Viewer

ROOT = Path(__file__).resolve().parents[1]
MODEL_PATH = (
    ROOT / 'third_party' / 'unitree_mujoco'
    / 'unitree_robots' / 'g1' / 'scene.xml'
)

model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
data = mujoco.MjData(model)

if model.nkey > 0:
    mujoco.mj_resetDataKeyframe(model, data, 0)
mujoco.mj_forward(model, data)

print(f'G1 loaded: {model.njnt} joints, {model.nu} actuators')

# 仅监听服务器本机；通过 SSH 端口转发访问，避免公开服务。
server = viser.ViserServer(host='127.0.0.1', port=8765)
Viewer(model, data, server=server).run()
```

如果模型运行后倒下，通常是因为尚未实现站立/平衡控制；并不意味着浏览器可视化失败。可以先用 Viewer 的暂停、重置或关键帧功能查看模型。

### 6.3 启动可视化：需要两个终端

**终端 A：服务器（运行仿真）**

```bash
ssh bj2410@10.228.8.244
conda activate simulator
cd ~/projects/humanoid_manipulation
python sim_workspace/robots/g1/view_g1.py
```

保持这个终端运行。服务监听的是服务器本机的 `127.0.0.1:8765`。

**终端 B：Mac 本地（建立 SSH 隧道）**

```bash
ssh -N -L 127.0.0.1:8765:127.0.0.1:8765 bj2410@10.228.8.244
```

输入密码后，终端不返回提示符是正常现象；保持它运行。

**Mac 浏览器打开：** [http://localhost:8765](http://localhost:8765)

```text
NYUAD server                                     Mac
MuJoCo + G1 + Python
    |
    | 127.0.0.1:8765  <-- SSH 加密端口转发 -->  localhost:8765
    |                                              |
    +------------------------------------------- 浏览器 3D Viewer
```

**重要：** `localhost` 在服务器和 Mac 上分别代表各自的机器；中间由 `ssh -L` 连接。不要把 `ViserServer` 改为 `0.0.0.0` 并公开暴露端口，除非得到管理员批准且配置了访问控制。

### 6.4 Viewer 里能做什么？

- 旋转/缩放视角、查看机器人连杆和关节。
- 暂停、单步、重置仿真；若模型包含 keyframe，可切换关键帧。
- 查看关节/执行器滑块、接触信息和其他可视化设置（具体取决于模型与 Viewer 版本）。
- 以后把 IK、规划和控制代码接入同一个仿真进程，实时观察动作。

**现阶段仅完成可视化基础，并不表示抓取控制器已经实现。**

### 6.5 修改代码后如何看到新效果？

1. 在 Mac 的 **VS Code Remote - SSH** 连接服务器，打开 `/home/bj2410/projects/humanoid_manipulation`。
2. 修改 `sim_workspace/robots/g1/view_g1.py` 或其他 `sim_workspace/robots/g1/` 文件并保存。
3. 回到服务器的终端 A，按 `Ctrl+C` 停止旧 Viewer，再运行 `python sim_workspace/robots/g1/view_g1.py`。
4. Mac 上的终端 B（SSH 隧道）**保持不动**，刷新 `http://localhost:8765`。

**代码并不会仅因为保存就自动热更新。** 模型或 Python 控制逻辑改变时，通常需要重启仿真进程。

### 6.6 使用 tmux，避免 SSH 断线终止仿真

在服务器执行：

```bash
tmux new -s g1viewer
conda activate simulator
cd ~/projects/humanoid_manipulation
python sim_workspace/robots/g1/view_g1.py
```

按 `Ctrl+B`，松开后按 `D`，可暂时离开会话。重新连接服务器后：

```bash
tmux attach -t g1viewer
```

`tmux` 只让服务器程序继续运行；**Mac 的 SSH 隧道仍需保持连接或重新建立**。若提示找不到 `tmux`，先确认服务器有没有安装。

### 6.7 常见问题排查

| 现象 | 检查方法 |
| --- | --- |
| 浏览器打不开 `localhost:8765` | 确认服务器的 `python sim_workspace/robots/g1/view_g1.py` 正在运行，并确认 Mac 的 `ssh -N -L ...` 仍连接 |
| `Address already in use` | 检查是否已有 Viewer 占用端口：`ss -ltnp | grep 8765`；不要重复启动多个实例 |
| Mac 本地端口被占用 | 将 `ssh -L` 命令左侧端口换成如 `18765`，浏览器打开 `http://localhost:18765`；右侧服务器端仍为 `8765` |
| `ModuleNotFoundError` | 执行 `conda activate simulator`，再检查 `python -m pip show mujoco mjviser viser` |
| 找不到 `scene.xml` 或 mesh | 检查 `third_party/unitree_mujoco/unitree_robots/g1/` 是否完整；从项目根目录启动 |
| G1 倒下 | 先暂停仿真/加载有效 keyframe；后续加入稳定控制器 |
| XQuartz/GLX 报错 | 不再走原生 X11 Viewer；使用本节 Web Viewer |
| 浏览器刷新后仍是旧代码 | 停止服务器旧 Python 进程并重新运行脚本 |

## 7. GPU 与计算资源

本项目当前的 MJCF 加载、一般刚体物理仿真和 Web Viewer **不要求专门占用 A6000**。后续需要 GPU 加速训练、CUDA 计算或 GPU 渲染时，必须先遵守实验室预约规定。

检查 GPU：

```bash
nvidia-smi
```

预约确认后，才可以为需要 CUDA 的程序指定例如：

```bash
export CUDA_VISIBLE_DEVICES=2
python train.py
```

`2` 只是示例 GPU 编号，**不能依据之前一次 `nvidia-smi` 的空闲状态认定现在仍可用**。使用结束后应确保进程退出并更新预约状态。

## 8. 研发计划 / 待办

- [x] 安装个人 Miniconda3，创建 `simulator` Python 3.10 环境。
- [x] 建立独立项目目录 `~/projects/humanoid_manipulation`。
- [x] 通过 sparse-checkout 获取 Unitree 官方 G1 MJCF 模型。
- [x] 用 MuJoCo 成功加载 G1 场景。
- [x] 解决原生 Viewer 的远程 GLX 限制，配置浏览器 Web Viewer 和 SSH 隧道。
- [ ] 核对 G1 关节名称、限位、坐标系、执行器类型。
- [ ] 选定具备真实开合/接触功能的夹爪或灵巧手模型。
- [ ] 搭建桌面、可抓取物体和目标放置区域。
- [ ] 实现右臂目标位姿的坐标变换、FK/IK。
- [ ] 实现碰撞检测、预抓取姿态和无碰撞轨迹规划。
- [ ] 完成 Grasp → Lift → Place → Release 状态机与执行器控制。
- [ ] 实现成功判定及不可达、碰撞、漏抓、掉落测试。
- [ ] 输出可重复的测试结果、日志和演示视频。

推荐的第一项开发里程碑：**在 Web Viewer 中，让 G1 右手通过 IK 到达给定的 3D 目标位置**；之后再接入可动手和完整抓取流程。

## 9. 参考资料

- [Unitree 官方 MuJoCo 仓库](https://github.com/unitreerobotics/unitree_mujoco)
- [MuJoCo 文档](https://mujoco.readthedocs.io/)
- [mjviser 浏览器 Viewer](https://github.com/mujocolab/mjviser)
- [mjviser 0.0.14（Python 3.10 兼容版本）](https://pypi.org/project/mjviser/0.0.14/)

> 安全提醒：不要把服务器密码、令牌或 SSH 私钥写进 README，也不要把未经认证的 Web Viewer 直接开放到公网或校园网。
