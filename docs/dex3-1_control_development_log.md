# Humanoid Manipulation — Dex3-1 控制开发与问题排查记录

> **文档定位：** 本文件是项目主 `README.md` 的**后续开发记录**，从 **MuJoCo + G1 + SSH 隧道 + mjviser Web Viewer 已经搭建完成** 之后开始，详述 Dex3-1 模型切换、控制器编写、每次遇到的问题、定位证据、修复位置、最终验证结果和后续注意事项。
>
> **版本：** v2（2026-11-10：新增逐参数中文说明、调整方法、代码位置和调试速查）。
>
> **更新基线：** 2026-11-06 的实际调试记录。这里只记录当时实际执行、验证或观察到的情况，不声称所有未来模型版本都适用。
>
> **核心约定：** 项目中只维护 **一个正式可视化入口 `sim_workspace/robots/g1/view_g1.py`**。不为类似实验反复创建 `view_g1_dex3.py`、`preview_dex3.py` 等功能重复的永久文件。任何实验性代码在验证后合并回这个入口；真正独立的功能变复杂时再抽取模块。

---

## 0. 快速导航：遇到问题先看哪里？

**不知道参数含义、如何改变动作或如何调参？请先看 [§2.5 参数使用手册](#25-参数使用手册每个参数是什么在哪里改改完会怎样重点新增)。**

| 现象 / 需求 | 先检查 `sim_workspace/robots/g1/view_g1.py` 的哪个位置 | 本文对应章节 |
| --- | --- | --- |
| 显示的是没有灵巧手的 G1 | `XML = ROOT / (...)` 模型路径 | §2、§5.1 |
| Dex3-1 手指不动、只有画面 | 最后的 `Viewer(..., step_fn=step)` 和 `def step(...)` | §3、§5.3 |
| 整个 G1 身体跟着晃动 | XML 内存修改：删除非右手关节、移除非右手执行器 | §3.2、§5.2 |
| 手指抽搐/鬼畜、振荡 | `model.opt`、`model.dof_damping`、`kp/kd`、`mujoco.mj_step`、接触检测 | §3.3、§3.7、§5.4–5.7 |
| 只有一个手指关节在动 | `goal` / `moving_joints` 的选择逻辑 | §3.4、§5.3 |
| Index 1 目标在变但实际角度不变 | `kp/kd`、`data.ctrl`、`mj_step` 调用次数、`velocity` 日志 | §3.5–3.6、§5.4–5.5 |
| 报 `IndentationError` / `SyntaxError` | `def step` 里的 `global next_log_time`、循环缩进和函数结束位置 | §3.6、§5.4 |
| 每秒重复打印很多条相同时间 | `next_log_time` 的定时日志条件 | §3.6、§5.5 |
| 开接触后 Index 0 高速抖动 | `Contact Diagnostics`、`geom_bodyid`、`contact/exclude` | §3.7–3.9、§5.6–5.7 |
| `Contacts detected: 10–12`，穿透约 20 mm | `<contact><exclude .../>` 的三个 Body 对 | §3.8–3.9、§5.2 |
| Web 页面打不开 | `ViserServer(host="127.0.0.1", port=8765)`、SSH 隧道、旧进程 | §6 |
| 需要真实抓物体而不只是摆手指 | 不要沿用“锁住全部其他关节 + 无重力”作为最终场景 | §7 |
| **想调动作幅度、速度、PD、阻尼、碰撞或日志** | **对应参数的搜索关键词与修改示例** | **§2.5（新增）** |

**定位方法：** 这里使用代码里的关键字作“位置锚点”，不用行号。因为代码继续修改后行号会变化，但搜索 `def step`、`model.opt`、`contact = root.find("contact")` 等仍然有效。在 VS Code 远程打开文件后使用 `Cmd+F` 查找。

---

## 1. 开发前提与现有项目结构

此前已完成：

- 服务器上安装 Miniconda，创建 `simulator`（Python 3.10）环境。
- 独立项目目录：`~/projects/humanoid_manipulation/`。
- 获取 Unitree G1 基础模型、安装 MuJoCo，并已能用 Python 加载。
- 由于 XQuartz / SSH X11 forwarding 出现 GLX context 错误，改用 **服务器端 `mjviser` + Mac 浏览器 + SSH 本地端口转发** 的方式显示 3D 场景。
- 正式 Web Viewer 入口合并为 `sim_workspace/robots/g1/view_g1.py`；通过 `http://localhost:8765` 查看。

进入 Dex3-1 阶段后，当前涉及的文件约定如下：

```text
/home/bj2410/projects/humanoid_manipulation/
├── README.md                  # 主 README（环境、基础 Viewer）
├── src/
│   └── view_g1.py            # 唯一正式的运行/控制/可视化入口
└── third_party/
    ├── unitree_mujoco/       # 原始 G1 仿真仓库；当前右手实验不从这里加载模型
    └── unitree_ros/          # 带 Dex3-1 的 G1 官方机器人描述
        └── robots/
            └── g1_description/
                ├── g1_29dof_with_hand_rev_1_0.xml
                └── meshes/
```

**请避免：** 修改 `third_party/unitree_ros/...xml` 的原始文件来“修”本次调试。当前处理均在 Python 中读取 XML 后**内存修改**，官方原版保留不变。

当前用于控制与查看的两个进程：

**服务器端：**

```bash
conda activate simulator
cd ~/projects/humanoid_manipulation
python sim_workspace/robots/g1/view_g1.py
```

**Mac 本地终端（SSH 隧道；须能访问服务器内网）：**

```bash
ssh -N -L 127.0.0.1:8765:127.0.0.1:8765 bj2410@10.228.8.244
```

**Mac 浏览器：** <http://localhost:8765>

保存 Python 文件**不会自动热更新**，需要停止服务器旧 Viewer → 重新启动 `python sim_workspace/robots/g1/view_g1.py` → 浏览器刷新；Mac 端 SSH 隧道如果没有断线，无需重启。不要把 Web 服务改成公开监听 `0.0.0.0`。

---

## 2. 为什么改用 `unitree_ros` 的 Dex3-1 模型

### 2.1 原来两个 Unitree 仓库的区别

| 仓库 | 本项目中的作用 |
| --- | --- |
| `third_party/unitree_mujoco/` | 最初获取和验证 **基础 G1** MJCF 仿真场景（`unitree_robots/g1/scene.xml`） |
| `third_party/unitree_ros/` | 使用 `robots/g1_description/g1_29dof_with_hand_rev_1_0.xml`，其中包含 **G1 + 两只 7-DoF 灵巧手** 的 MJCF 描述 |

**重点：** `unitree_ros` 这个仓库名带有 `ros`，但其 MJCF 文件可直接由 Python 的 MuJoCo 加载；本阶段**没有安装或运行 ROS**。不要把两个 XML 生硬拼在一起。

### 2.2 已验证过的模型信息

当时实际执行了：

```python
m = mujoco.MjModel.from_xml_path(
    "third_party/unitree_ros/robots/g1_description/"
    "g1_29dof_with_hand_rev_1_0.xml"
)
print(m.nu)
```

得到 `Total actuators: 43`，右手与左手各存在 7 个手指关节；对应为 **29 个 G1 本体执行器 + 14 个手部执行器**。

当前主要控制右手 7 个关节：

```text
right_hand_thumb_0_joint
right_hand_thumb_1_joint
right_hand_thumb_2_joint
right_hand_index_0_joint
right_hand_index_1_joint
right_hand_middle_0_joint
right_hand_middle_1_joint
```

原模型的 `model.nu=43` 与后来用于控制的内存简化模型的 `model.nu=7` 是**两种不同状态**：后者在本实验中移除了其他关节对应的执行器，不能误以为官方 G1 本来只有 7 个电机。

### 2.3 `ctrlrange=[0,0]` 为什么不是电机坏了？

曾查询右手各 motor，发现：

```text
ctrlrange = [0. 0.]
limited = False
```

只要 `actuator_ctrllimited=False`，`ctrlrange=[0,0]` 就**不是正在生效的控制输入限制**；`data.ctrl` 仍能输出控制信号。真正需要关注模型的 actuator gear、joint actuator force limits、控制器施加的额外限幅及实际的控制效果。不要凭 `ctrlrange` 的两个 0 推断电机不能动。

---

## 2.5 参数使用手册：每个参数是什么、在哪里改、改完会怎样（重点新增）

> **阅读路线：** 如果主要想知道“我应该改哪个数字”，先看 §2.5.1 和 §2.5.10；想明白原理再看中间的小节。文中所有“当前值”都以本次**固定 G1 身体、仅保留 Dex3-1 右手 7 个运动关节、无外部抓取物体**的成功实验为准，不是 Unitree 官方为实体机器人推荐的参数。
>
> **只有一个控制入口：** 所有示例都针对 `sim_workspace/robots/g1/view_g1.py`，不要求增加 Python 文件。每个参数都提供了 **搜索关键词**；在 VS Code 按 `Cmd+F` 搜索即可，**不要依赖行号**。

### 2.5.1 快速查表：我想改变什么，就改哪里？

| 我想实现的效果 | 在 `view_g1.py` 搜索 | 当前值 | 该怎么调整（实验用） | 改动的风险/如何判断 |
| --- | --- | --- | --- | --- |
| 手指弯曲幅度大一点 | `goal = start +` | `0.12 * (hi - lo)` | 先小幅改成 `0.15 * (hi - lo)`，每次只改幅度 | 目标可能接近限位或碰撞；同时看 `actual`、`data.ncon` |
| 手指幅度小一点 | `goal = start +` | `0.12` | 改成 `0.08`（示例） | 动作可能不明显，先放大 Web Viewer 右手区域 |
| 动作更慢、更平滑 | `data.time / 8.0` | 周期 8 s | 例如改成 `/ 12.0` | 时长增大，最大目标速度通常降低；无需改物理时间步 |
| 动作快一点 | `data.time / 8.0` | 周期 8 s | 例如改成 `/ 6.0` | 可能加大跟踪误差与接触冲击，先确保无振荡 |
| 只让食指动 | `goal = start +` | 7 关节都动 | 用 `moving_joints`，其余关节 `goal = start` | 不等于释放其他关节，其他关节仍有保持姿态的 PD 控制 |
| 关节跟踪偏软/滞后 | `kp = 0.3` | 0.3 | 先看 `velocity`、`contact` 和饱和情况；确认无异常后再小步试 `kp` | `kp` 太高可造成振荡；不要作为第一修复手段 |
| 关节快速抖动 | `kd = 0.002`、`dof_damping`、`mj_step`、`Contact Diagnostics` | 0.002 / 0.05 | **先排接触和重复积分**，再单参数小步调试 | 抖动不等于 `kd` 一定太小；本项目曾因 `kd=0.12` 和装配接触出问题 |
| 电机经常打到限幅 | `low, high = -0.15, 0.15` | ±0.15 | **先定位是目标、碰撞、速度还是传动问题**，不要直接放大 | 限幅是额外的测试保护，不是电机真实最大能力 |
| 日志打印太频繁 | `next_log_time += 0.5` | 每 0.5 s | 例如改 `+= 1.0` | 只改变输出频率，**不改变仿真/控制频率** |
| 仿真步更细 | `model.opt.timestep` | `0.001` s | 仅诊断必要时尝试 `0.0005` s | 计算更慢，不保证解决碰撞/控制问题 |
| 启用或禁用重力 | `model.opt.gravity` | 全零 | 当前保持全零；引入真实物体时需另设计支持/初始姿态后再恢复 | 开重力不等于“只让被抓物体受重力”，整个模型都会受影响 |
| 接触突然变成 10–12 | `contact = root.find("contact")` | 排除 3 对装配接触 | 检查排除声明是否在 `MjModel.from_xml_string` **之前**加入 | 不要全局关闭接触，否则不能模拟抓取 |
| Web 页面端口变化 | `port=8765` | 8765 | 服务端和 Mac `ssh -L` 的对应端口一起改 | 两侧不匹配将访问不到 Viewer |

**当前成功实验的“先不要动”组合：** `timestep=0.001`、`integrator=IMPLICITFAST`、`dof_damping=0.05`、`kp=0.3`、`kd=0.002`、测试输入限幅 `±0.15`、手指目标幅度 `0.12`、往返周期 8 s、恢复接触但排除三组已定位的装配碰撞、重力关闭。这是**有条件的实验基线**，不是通用最佳值。

### 2.5.2 Python 变量 / MuJoCo 数组“字典”：哪些是编号，哪些才是物理量？

| 变量或字段 | 具体含义 | 单位/类型 | 需要用户修改吗？ |
| --- | --- | --- | --- |
| `XML` | 带 Dex3-1 的 MJCF 文件路径 | `Path` | 只有切换模型版本才修改 |
| `root` / `tree` | 在内存中解析的 XML 树 | Python 对象 | 通常不直接改；通过 `<joint>` / `<contact>` 等节点操作 |
| `model` | 已编译的 MuJoCo 模型：质量、惯量、关节、执行器、碰撞等 | `MjModel` | 用它读/改仿真参数 |
| `data` | 当前动态状态：`qpos`、`qvel`、`ctrl`、`time`、接触 | `MjData` | 控制过程中读写 |
| `name` | 关节名，如 `right_hand_index_0_joint` | 字符串 | 只在选择关节或换模型时改 |
| `j` / `jid` | 当前编译模型里的关节 ID | 非负整数 | **不要手动写死**；由查询得出 |
| `aid` | 执行器 ID，用作 `data.ctrl[aid]` 下标 | 非负整数 | **不要手动写死**；名称查询得出 |
| `qadr` | 当前关节在 `data.qpos` 中的起始地址 | 整数索引 | 自动从 `model.jnt_qposadr[j]` 读取 |
| `vadr` | 当前关节在 `data.qvel` 中的起始地址 | 整数索引 | 自动从 `model.jnt_dofadr[j]` 读取 |
| `q` / `data.qpos[qadr]` | 关节当前角度；这里都是旋转关节 | rad（弧度） | **读状态**；物理控制时不要每帧直接写它 |
| `dq` / `data.qvel[vadr]` | 关节当前角速度 | rad/s | **读状态**，供阻尼反馈 |
| `data.ctrl[aid]` | 送给第 `aid` 个执行器的控制输入 | 随执行器类型/gear 而定 | 由控制器每步更新 |
| `model.actuator_gear[aid, 0]` | 传动增益；影响控制输入到关节广义力的映射 | 由模型定义 | 本实验验证近似为 1；若改模型须重验 |
| `lo, hi` | 旋转关节允许的角度范围 | rad | 来自 `model.jnt_range[j]`，一般不随意修改 |
| `start` / `goal` | 周期动作两端目标角度 | rad | 可以调，但需落在关节范围内 |
| `phase` | 0 到 1 再回到 0 的平滑插值比例 | 无量纲 | 可以通过周期或轨迹公式修改 |
| `target` | 当前时刻希望关节达到的目标角 | rad | 控制器计算结果，不是固定常数 |
| `torque` | PD 产生的控制量；在当前 `gear≈1` 的 motor 测试中可按关节力矩理解 | 约 Nm | 不宜直接手动设固定值；通过 PD 计算、受限幅约束 |
| `data.time` | MuJoCo 累计的**仿真时间**，不一定等于现实墙钟时间 | s | 不手工修改（除专门的运动学预览） |
| `data.ncon` | 当前检测到的接触点数量 | 整数 | 日志量，不是“碰撞开关” |
| `c.dist` | 两接触几何体表面的有符号接触距离 | m | 观测诊断；负值通常表示穿透 |
| `c.efc_address` | 接触在约束数组中的地址，负值通常表示未激活/未进入约束求解 | 整数 | 诊断用；非直接“接触力”数值 |
| `next_log_time` | 下一次日志输出的仿真时间阈值 | s | 可以更改日志间隔，但别混淆为物理时间步 |

**特别提醒：** 原始 MJCF 有 43 个执行器；当前手部诊断模型通过删掉其他执行器后只有 7 个。`joint_id=37` 是**原始模型**里的索引，简化后不能再沿用数字 37。准确做法始终是通过名称查询。

### 2.5.3 运动范围：`lo`、`hi`、`start`、`goal`、`0.12` 究竟如何决定手指弯曲？

现在的代码：

```python
lo, hi = model.jnt_range[j]
start = lo + 0.25 * (hi - lo)
goal = start + 0.12 * (hi - lo)
```

逐个理解：

- `hi - lo`：这个关节允许转动的总角度范围，而不是一个固定弧度值。
- `0.25`：初始位置选在活动范围的 **25%** 处，避开恰好从最小边界出发。
- `0.12`：本次目标与初始位置之间差 **整个活动范围的 12%**。它不是 0.12 rad，也不是 12°。
- `start + ...`：按关节数值**增大**的方向变化；并不自动等于视觉意义的“握拳”，特别是拇指的关节方向可能与食指不同。
- `start`、`goal` 只是关节目标，`actual` 是否达到目标要看电机控制、约束和接触。

**具体算例（使用本次食指的典型范围作为例子）：** 若 `lo=0`、`hi=1.5708 rad`，则 `start≈0.3927 rad`，`goal≈0.5812 rad`，这就是你之前日志里的约 `0.393→0.581`。改变 `0.12` 到 `0.20`，目标跨度会从约 `0.1885 rad` 变成 `0.3142 rad`。只是**示例**；实际需要根据关节限位和接触逐步验证。

**要增大/缩小动作：** 搜索 `goal = start +`，只改系数 `0.12`。从 `0.12` 到 `0.15` 属于较小的一次测试变化；例如想要减小就试 `0.08`。建议从小变化开始，重启 Viewer，比较每个关节的 `target/actual/velocity/data.ncon`。不要一口气改成 `1.0`。

**只控制两节食指，其他关节保持在 `start`：** 把计算 `goal` 的位置替换为：

```python
moving_joints = {
    "right_hand_index_0_joint",
    "right_hand_index_1_joint",
}
if name in moving_joints:
    goal = start + 0.12 * (hi - lo)
else:
    goal = start
```

这段必须放在构建 `hand` 的循环里、`hand.append(...)` **之前**。注意其它 5 个关节并非没有控制，它们的 `goal=start` 意味着 PD 控制器仍会试图维持它们的静止目标。

**逐指设置不同幅度：** 可以不增加文件，只在 `hand.append(...)` 前使用一个小字典（下面的数值仅作调试示例，不是正式 grasp pose）：

```python
fraction_by_joint = {
    "right_hand_thumb_0_joint": 0.08,
    "right_hand_thumb_1_joint": 0.10,
    "right_hand_thumb_2_joint": 0.10,
    "right_hand_index_0_joint": 0.12,
    "right_hand_index_1_joint": 0.12,
    "right_hand_middle_0_joint": 0.12,
    "right_hand_middle_1_joint": 0.12,
}
goal = start + fraction_by_joint[name] * (hi - lo)
```

对于需要往关节**数值减小**方向运动的情况，可以用负的跨度，但必须确保 `goal` 仍在 `[lo, hi]` 内。对于实际的 `OPEN/PINCH/HOLD`，应先根据指尖几何与接触设计目标，而不是认为每个关节都增加就是闭合。

### 2.5.4 动作速度：8 秒周期与 `phase` 的数学含义

当前代码：

```python
phase = 0.5 - 0.5 * np.cos(2.0 * np.pi * data.time / 8.0)
target = start + phase * (goal - start)
```

这是一个**平滑往复的目标轨迹**：

- `t=0`：`phase=0`，`target=start`。
- `t=2 s`：`phase=0.5`，目标到达中间。
- `t=4 s`：`phase=1`，目标到达 `goal`。
- `t=6 s`：`phase=0.5`，目标回到中间。
- `t=8 s`：`phase=0`，目标回到 `start`，随后重复。

因此 **8 秒是“过去再回来”的总周期**，并不是从打开到闭合的单程时间。想放慢就将 `/ 8.0` 改成 `/ 12.0`；想加快可试 `/ 6.0`。**不要通过调大物理 `timestep` 来让动作看起来更快。** 更快的目标轨迹可能需要控制器产生更大的力矩，实际跟踪误差也可能变大。

`np.pi` 是圆周率；`np.cos` 是余弦；`2π×time/T` 把时间变成周期相位；`0.5−0.5cos(...)` 令首尾目标速度为零，从而避免突然换方向。这里不是在控制真正的张开/闭合状态机，后续抓取时要换成分阶段轨迹。

### 2.5.5 PD 控制：为什么有 `kp`、`kd`，怎么调才能不鬼畜？

核心代码：

```python
q = data.qpos[qadr]
dq = data.qvel[vadr]
kp = 0.3
kd = 0.002
torque = kp * (target - q) - kd * dq
```

公式：`控制输入 = Kp × (目标角度 − 实际角度) − Kd × 当前角速度`。

| 参数 | 当前值 | 物理意义 | 增大时通常怎样 | 减小时通常怎样 |
| --- | --- | --- | --- | --- |
| `kp` | 0.3 | 位置反馈增益；当前 motor / gear≈1 时可理解为 Nm/rad | 对角度误差反应更强，可能减小滞后；也更可能过冲、冲击或振荡 | 较柔和，但可能无法克服负载和约束、跟踪变慢 |
| `kd` | 0.002 | 速度反馈增益；当前 motor / gear≈1 时近似 Nm·s/rad | 更抑制速度，但若对低惯量关节过强，也可能数值振荡或产生很大反向力矩 | 速度反馈更弱，过低可能欠阻尼 |
| `target - q` | 实时变化 | 位置误差 | 不是需要设置的参数 | 需要观察其随时间是否减小 |
| `dq` | 实时测量 | 速度 | 不是参数；异常尖峰意味着须查接触、积分和关节约束 | — |

**为什么曾从 `kd=0.12` 改到 `0.002`？** 当时观察到 Index 1 目标移动、实际几乎不变、控制输入频繁到 ±0.15；同时后来还修复了 `mj_step` 重复执行和装配接触。因此**不能单独归功于 `kd`**，只能说当前组合在本次实验中验证过。

**正确的调参顺序（针对现有模拟实验）：**

1. 先保留当前成功的 `kp=0.3`、`kd=0.002`，确认 `mj_step` 每次只调用一次，`data.ncon` 没有异常装配碰撞。
2. 查看输出的 `target/actual`：如果目标在变，实际角度持续落后，同时控制输入没有饱和、接触正常，才考虑对 `kp` 做**单独的小幅实验**。
3. 如果出现速度反向尖峰、震荡，先看接触对和 `model.opt.timestep`，不要本能地增加 `kd`。
4. 如果目标变化很小，增益调得再激进也未必容易“看出”动作；先确认 `goal-start` 和 Viewer 缩放。
5. 每次只改一个变量，比较改动前后同一目标轨迹下的最大速度、最大/平均绝对误差、力矩饱和次数和接触数。

**不要把这组参数直接复制到真实 Dex3-1 硬件。** 仿真 motor 的 gear、关节惯量、摩擦和硬件控制接口可能不同，这里的参数只经过简化仿真验证。

### 2.5.6 `low/high`、`data.ctrl` 与真正的电机力矩限制

```python
low, high = -0.15, 0.15
if model.jnt_actfrclimited[jid]:
    joint_low, joint_high = model.jnt_actfrcrange[jid]
    low = max(low, joint_low)
    high = min(high, joint_high)
data.ctrl[aid] = np.clip(torque, low, high)
```

- `-0.15` 和 `+0.15` 是**我们自己人为加的诊断限幅**，不是自动读取到的 Unitree 电机允许最大力矩。
- `np.clip` 表示把指令截到这个区间，例如计算得到 `0.30`，实际发出的 `data.ctrl` 是 `0.15`。
- `model.jnt_actfrclimited[jid]` 表示关节是否有**执行器作用在关节上的合力限制**；若有，用 `jnt_actfrcrange` 与测试区间求交集。
- 还可能存在 `actuator_ctrllimited`、`actuator_forcelimited` / `actuator_forcerange` 等模型约束，不能仅凭这一段就宣布满足所有执行器规格。
- 当 `actuator_gear≈1` 且是简单 motor 时，可以把 `data.ctrl` 的数值近似看作对应的关节力矩（Nm）；换执行器类型或 gear 后不能这样直接解释。
- `ctrlrange=[0,0]`、`actuator_ctrllimited=False` 的组合**不表示电机输出为 0**。参见 §2.3。

**若持续出现 `torque=+0.150` 或 `-0.150`：** 这是重要的饱和信号。先判断是否碰撞/关节卡住、`goal` 是否不合理、`kd` 是否导致反向大力矩，再考虑是否需要提高限幅。不要一下子从 0.15 跳到模型最大可用力矩。

### 2.5.7 物理仿真设置：`timestep`、`integrator`、`gravity`、`dof_damping`

当前代码：

```python
model.opt.timestep = 0.001
model.opt.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
model.opt.gravity[:] = 0
for j in range(model.njnt):
    if model.joint(j).name.startswith("right_hand_"):
        model.dof_damping[model.jnt_dofadr[j]] = 0.05
```

| 设置 | 当前值 | 用途 | 修改后可能有什么影响 | 重要提醒 |
| --- | --- | --- | --- | --- |
| `model.opt.timestep` | 0.001 s | 每次 `mj_step` 积分 1 ms，即模拟频率约 1000 步/s（不等于屏幕刷新 1000 fps） | 变小会增加计算量，可能改善数值稳定性；变大可能加剧误差 | 不控制手指动作周期，动作周期在 `phase` 中改 |
| `model.opt.integrator` | `mjINT_IMPLICITFAST` | 数值积分算法，适合本次含阻尼与接触的调试方案 | 更换算法会改变求解结果和性能 | 不保证它对所有机器人模型都是最优 |
| `model.opt.gravity[:]` | `[0,0,0]` | 关闭场景重力，便于纯手部控制隔离测试 | 改成通常的地球重力 `[0,0,-9.81]` 会让所有有质量的运动部件受重力 | **当前不能当成真实抓取验证** |
| `model.dof_damping[...]` | `0.05` | 关节自身的被动黏性阻尼；对于旋转关节按 Nm·s/rad 理解 | 增大通常更抑制运动，也可能增大滞后；减小可能更容易振荡 | 与 PD 的 `kd` **不是同一回事** |
| `model.opt.disableflags` 的 `mjDSBL_CONTACT` 位 | 该位被清除 | 使 MuJoCo 计算接触 | 若被置位，会禁用全局接触 | 要做抓取不能长期禁用接触 |

**最容易混淆的两个数：**

- `timestep=0.001`：**物理每一步多长时间**；每次 `mj_step` 时间增加约 0.001 s。
- `phase` 里的 `8.0`：**完成一次往返目标动作需要多久**；改这个才会令手指动作快/慢。

**为何暂时关闭重力：** 当前模型的 G1 身体被固定，只为验证手指电机控制。恢复重力后必须重新验证目标追踪和物体接触；以后添加圆柱体进行真实抓取时，必须为物体设置质量、摩擦和合适的自由度，并确保其受重力影响，而不是靠零重力“悬浮”保持。

### 2.5.8 碰撞与接触：为什么以前出现 20 mm 穿透？

**三个层次不要混淆：**

1. **启用整个物理引擎的碰撞：** `model.opt.disableflags &= ~int(mujoco.mjtDisableBit.mjDSBL_CONTACT)`，当前已开启。用 `|=` 设置禁用位会关闭所有接触，不适合真实抓取。
2. **只排除特定装配部件对：** XML 内存修改时的 `<contact><exclude body1="..." body2="..."/></contact>`；本次对手腕与三个指根装配对使用该方式。
3. **观察接触信息：** `data.ncon`、`c.dist`、`c.efc_address`、`geom_bodyid`；日志用于判断碰撞发生在哪两个 Body、是否产生穿透。

此前检测到 `right_wrist_yaw_link` 与三个指根 Body 持续重叠约 16–20 mm。**不是把全部碰撞关掉才修好**，而是只针对已诊断的三个相邻装配 Body 对加入排除规则，之后在无外部物体条件下观测到 `Contacts detected: 0`。这些排除项在**内存 XML 编译前**加入，官方原 XML 保持不变。

以后添加圆柱体后，`ncon > 0` **反而可能是需要的**，因为手指需要真实接触物体。不能把“接触数始终必须为 0”当成抓取成功标准。若有大穿透，要看是哪些 Body 对：不应随意排除手指—物体或手指—桌面的真实接触。

### 2.5.9 Viewer 与日志参数：`port`、`host`、`0.5`、`step_fn`

```python
server = viser.ViserServer(host="127.0.0.1", port=8765)
Viewer(model, data, server=server, step_fn=step).run()
```

- `host="127.0.0.1"`：Web 服务只监听服务器的 loopback 接口，不向共享校园网络公开访问入口；不要为了方便直接改为 `0.0.0.0`。
- `port=8765`：服务器监听端口。Mac 上的 SSH 转发需要指向相同的远端端口；例如 `ssh -N -L 127.0.0.1:8765:127.0.0.1:8765 ...`。若改成 `8766`，隧道远端也要对应更改。
- `step_fn=step`：把上面定义的控制回调交给 Viewer；删掉它可能仍有画面，但不会按照自定义 PD 控制更新电机。
- `next_log_time = 0.0` 和 `next_log_time += 0.5`：每隔 **0.5 个仿真秒**打印一次。若想少看日志，改为 `+= 1.0`；不影响实际控制频率。
- `flush=True`：及时刷新输出，不影响机器人动力学。

`mujoco.mj_forward(model, data)` 只计算当前状态对应的前向运动学/派生物理量，**不会像 `mujoco.mj_step` 那样推进仿真时间**。之前的纯运动学预览通过直接写 `qpos` 后 `mj_forward` 显示动作；现在的真实 motor-PD 控制应计算 `data.ctrl` 并用 `mj_step`，每次回调**只调用一次**。

### 2.5.10 问题 → 具体改哪个数 → 先验证什么（以后照着查）

| 出现的现象 | 优先检查 | 建议的定位顺序 / 下一步 | **不要先做的事** |
| --- | --- | --- | --- |
| 手指动作看不明显 | `goal = start + 0.12 * (...)` | 放大右手 → 确定 `target` 变化 → 小幅增大 0.12 | 直接大幅提高 `kp` |
| 只有食指 0 动 | `moving_joints` / `goal` | 看两关节 `target` 有没有变化 → 看 `actual` | 只看画面就判定电机坏了 |
| `target` 变化，`actual` 几乎不动 | `data.ctrl`、`kp/kd`、`contact`、`mj_step` | 看力矩是否饱和、角速度是否尖峰、是否有异常接触，再检查控制回调步进次数 | 直接删掉关节限位 |
| 关节速度突然 ±5 rad/s | `Contact Diagnostics`、`mj_step` 次数、`kd`、`timestep` | 检查 `data.ncon` 和 Body 对 → 检查是否每回调只积分一次 → 再做增益实验 | 先把限幅调很大 |
| `Contacts detected: 10–12` 且 16–20 mm 穿透 | `contact/exclude` | 根据 Body 名称确认是手腕—指根装配对，必要时针对性排除 | 直接关闭全局接触 |
| 关节速度平稳但整体跟踪有滞后 | 8 s 周期、`goal-start`、`kp` | 先放慢目标轨迹 → 确认无饱和/碰撞 → 再小步调 `kp` | 同时改 `kp`、`kd`、阻尼、步长 |
| 想让整个 G1 右臂参与 | 删除非手部关节的 XML 循环 | **这是模型结构变更**：保留右臂 7 DoF、重新定义初始姿态与控制、重新检查碰撞过滤 | 仅改 `moving_joints` 就以为右臂会动 |
| 加入抓取物体后没有接触 | `mjDSBL_CONTACT`、物体 `geom`/`freejoint` 和碰撞设置 | 确认接触开启、物体碰撞体存在、手指能到达物体、检查碰撞过滤 | 以 `ncon=0` 为好消息 |
| 加载模型报错 | `XML`、`compiler.set("meshdir",...)`、XML 树处理 | 核对模型路径和网格路径，检查模型关节是否存在 | 重新下载/重装整台服务器环境 |
| Web 页面不更新 | `Viewer(... step_fn=step)`、旧 Viewer 进程 | 停止旧进程 → 重新运行服务端 → 刷新浏览器；确认 SSH 隧道连接 | 重新安装 XQuartz |

### 2.5.11 推荐：把最常改的数值集中到文件顶部（仍然只有一个 Python 文件）

原来这些数值分散在代码各处，确实不方便。**这是可选的可维护性改进，不是要求你现在必须改动已验证代码**。如果决定统一配置，可以在 `XML = ...` **之后**加入：

```python
# === Experiment tuning: only for fixed-body Dex3 diagnostic ===
SIM_DT = 0.001           # s, physics step duration
JOINT_DAMPING = 0.05    # passive joint damping
START_FRACTION = 0.25   # initial joint position in [lo, hi]
MOTION_FRACTION = 0.12  # motion amplitude as fraction of joint range
CYCLE_SECONDS = 8.0     # s, full there-and-back motion period
KP = 0.3                # PD position gain
KD = 0.002              # PD velocity gain
CTRL_LIMIT = 0.15       # extra symmetric motor-input cap for this test
LOG_INTERVAL = 0.5      # s of simulation time, between diagnostic prints
VIEWER_PORT = 8765      # server web viewer port
```

然后**逐一**将旧代码里相应字面量替换为这些变量（不能只添加上面代码而不替换使用处）：

```python
model.opt.timestep = SIM_DT
model.dof_damping[model.jnt_dofadr[j]] = JOINT_DAMPING

start = lo + START_FRACTION * (hi - lo)
goal = start + MOTION_FRACTION * (hi - lo)

phase = 0.5 - 0.5 * np.cos(2.0 * np.pi * data.time / CYCLE_SECONDS)

kp = KP
kd = KD
low, high = -CTRL_LIMIT, CTRL_LIMIT
next_log_time += LOG_INTERVAL

server = viser.ViserServer(host="127.0.0.1", port=VIEWER_PORT)
```

**修改后务必确认：** 代码里没有遗漏原先散落的 `0.12`、`8.0`、`0.15` 等值；`KP/KD` 等名称与使用处一致；保存后运行 `python -m py_compile sim_workspace/robots/g1/view_g1.py`，停止旧 Viewer 并重启。这样下次再调试，就直接改文件最上面这 10 个值，而不用到处查找。

### 2.5.12 记录一次参数实验的标准模板（避免以后重复调试）

建议在本日志末尾为每次真正改动增加一条简短记录：

```text
日期/实验：
基线模型：G1 + Dex3-1 / fixed-body / 7 joints
本次只改了：例如 MOTION_FRACTION 0.12 → 0.15
其它参数：沿用 §4 当前基线
目标现象：右手弯曲幅度更明显
验证：浏览器看到...；target/actual 误差...；最大速度...；ncon...
结论：通过 / 不通过 / 需复查
回退：0.15 → 0.12
```

有了这样的记录，下次看到手指抖动、卡住或者碰撞，就可以直接知道**最近改了什么、去哪一行恢复**，而不是重新搭建仿真环境。

---

## 3. 完整调试时间线：问题 → 证据 → 修复

### 3.1 一开始出现了多个 Viewer 文件，项目变乱

**现象：** 开发过程中先后创建过 `view_g1.py`、`view_g1_dex3.py`、`test_dex3_control.py`、`preview_dex3.py`；其中部分只是换一个模型或控制模式，功能重复。

**决定：** 验证成功的预览内容最终合并进**一个** `sim_workspace/robots/g1/view_g1.py`。以后不要因为加一个测试动作就额外保留一个永久 Viewer。

**修复位置：** 项目 `src/` 的文件组织；单一入口 `sim_workspace/robots/g1/view_g1.py`。目前更复杂的 IK / 抓取模块将来确实需要时才另建文件。

**验收：** 启动 `python sim_workspace/robots/g1/view_g1.py`，Mac 浏览器仍能看到 G1 + Dex3-1。

### 3.2 第一个控制程序让整个 G1 的身体都动了

**现象：** 原先只想控制右手，实际全身晃动、旋转或漂移。

**原因：** 仅将 `model.opt.gravity[:] = 0` 设为零，并**不会锁住**浮动基座、腿、腰、手臂等关节；`mujoco.mj_step` 仍会推进它们的动力学。

**当时采用的调试修复：**

1. Python `ET.parse(XML)` 读取官方模型为内存中的 ElementTree。
2. 只保留名称以 `right_hand_` 开头的 7 个关节；移除所有其他 `joint/freejoint` 元素。
3. 在 `<actuator>` 中也只保留这 7 个关节对应的 motor。
4. 删除不再匹配维度的 `<keyframe>`，配置绝对 mesh 路径后，再 `mujoco.MjModel.from_xml_string(...)`。

**代码位置：** `XML = ROOT / (...)` 之后、`model = mujoco.MjModel.from_xml_string(...)` 之前（详见 §5.2）。

**结果：** 身体保持固定，只有右手具备动力学自由度。

**局限（非常重要）：** 删除关节不等于开发了正确的人形站立/平衡控制；其他关节被锁在默认/当时的配置，不是物理真实的操作姿态。以后恢复右臂 IK/抓取时要重新设计固定基座模型，**不能直接把这个 7-DoF 简化模型当完整 G1 交付**。

### 3.3 手指一开始剧烈抽搐、控制不稳定

**现象：** 手指高速振荡、看起来“鬼畜”。

**当时的试验组合：**

- 将时间步设置为 `0.001` s：`model.opt.timestep = 0.001`。
- 积分器设为 `mjINT_IMPLICITFAST`。
- 为当前右手每个 DoF 添加 `0.05` 的阻尼：`model.dof_damping[dof_id] = 0.05`。
- 在控制器调试初期**暂时关闭重力与接触**，隔离其他影响。
- 初期仅控制食指一个关节，减小目标幅度、减小输出力矩限幅。

**代码位置：** `model = ...` / `data = ...` 之间的模型选项，及 `def step` 内的控制增益（详见 §5.3–5.4）。

**谨慎解释：** 这些变更不是每一项都做了严格单变量消融，所以不能断言某一个数值单独“治好了鬼畜”；后续还发现了更关键的**接触几何与控制循环结构**问题。

### 3.4 看起来只有一个食指关节在动

**现象：** 明明有两节食指，观察时只有一节明显活动。

**问题 A：** 早期代码里只有 `right_hand_index_0_joint` 被赋予非零的目标变化，其他关节 `goal=start`；所以只有一个关节动是符合程序逻辑的。

**修复位置：** 生成 `hand` 列表的循环中，`start / goal / moving_joints` 代码。选中：

```python
moving_joints = {
    "right_hand_index_0_joint",
    "right_hand_index_1_joint",
}

if name in moving_joints:
    goal = start + 0.30 * (hi - lo)
else:
    goal = start
```

**问题 B：** 即使目标都在变，实际角度也可能没变。必须打印 `target` **和** `actual`，不能仅靠肉眼。

**结果：** 打印后发现 Index 0 能动，而 Index 1 曾在目标从 `0.436` 到 `0.698` rad 时，实际角度始终停留在 `0.436` rad 左右，因此是真实控制跟踪问题，不只是观察角度问题。

### 3.5 Index 1 的 `target` 在变，`actual` 几乎不变；电机正负饱和

**当时日志（代表性数值）：**

```text
Time 2.00 s
Index 0: target=0.629, actual=0.540
Index 1: target=0.698, actual=0.436, torque=-0.150
```

另有时段显示 Index 1 的 torque 在 `+0.150/-0.150` 之间切换。表明目标跟踪很差并出现过人为控制限幅；**不能仅凭这几条断言模型电机本身无力**。

**调试措施：** 保留 `kp=0.3`，将 `kd` 从 `0.12` 降到 `0.002`，并修正下述重复 `mj_step` 问题；保持输出限幅 `[-0.15, 0.15]`。后续两个食指关节恢复正常跟踪。

**代码位置：** `def step` 的：

```python
kp = 0.3
kd = 0.002
torque = kp * (target - q) - kd * dq
low, high = -0.15, 0.15
```

**注意：** 这组数字是无外部负载下调试验证过的参数，不代表 Dex3-1 的最终硬件控制增益，也不能直接拿到真实机器人上执行。

### 3.6 `step()` 缩进错误、重复积分，以及时间日志刷屏

这是实际出现过的**代码逻辑错误**，非常值得以后优先排查。

**错误写法（不要复用）：**

```python
for ... in hand:
    data.ctrl[aid] = ...
    mujoco.mj_step(model, data)  # 错：每更新一个电机就积分一次

mujoco.mj_step(model, data)      # 错：循环结束还额外积分一次
```

7 个关节会使每次控制回调推进 **8 个物理步**，也就破坏了“先写完 7 路电机输入、再一起积分一次”的控制语义。调试时还出现过 `global next_log_time` 的错误缩进（应放在 `def step` 之内、`for` 之前），甚至把日志粘贴到函数外部。

**最终正确顺序：**

```python
def step(model, data):
    global next_log_time
    # ... 计算 phase ...
    for ... in hand:
        # ... 给全部 7 个 data.ctrl[aid] 赋值 ...
        pass
    mujoco.mj_step(model, data)   # 每次回调只调用一次
    if data.time >= next_log_time:
        # ... 打印日志 ...
        next_log_time += 0.5
```

上面的 `pass` 仅示意控制流程，不是要替换为实际控制代码；完整参考见 §8。

**另一个问题：** 早期使用 `int(data.time / dt) % 1000 == 0` 打印，容易在数值时间边界多次触发，导致 `Time: 1.00s` 重复大量出现。

**修复：** 在 `step` 外初始化 `next_log_time = 0.0`，并在函数内部每越过 0.5 秒阈值打印一次，随后 `next_log_time += 0.5`。

**防止低级错误的命令：**

```bash
python -m py_compile sim_workspace/robots/g1/view_g1.py
```

这能检查语法/缩进，但不等于通过了仿真测试。

**验收：** 两个食指关节稳定跟踪；例如在 `t=52.0s` 时，Index 0 的 `target=0.8639/actual=0.8598`，Index 1 的 `target=0.9599/actual=0.9552`，两个误差均小于 `0.005 rad`。

### 3.7 扩展到七关节后，无接触可以稳定；一开接触就又抖动

**无接触时：** 先测试食指两个关节，再让全部 7 个关节周期变化；初步观察误差很小，力矩远小于 `±0.15` 的测试限幅。全部七关节都能追随目标。

**恢复接触：** 在初始化 `model.opt` 时移除“禁用接触”的设置，或确保：

```python
model.opt.disableflags &= ~int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
```

**现象：** 关节位置有时仍看似接近目标，但食指关节角速度出现 `-4.66`、`-5.59`、`+4.83 rad/s` 等大幅变化，表示**位置跟踪不差也可能存在严重高速振荡**。

**教训：** 以后看控制是否稳定，必须同时检查 `target/actual`、`velocity`、`torque` 和 `data.ncon`，不能只看位置误差或肉眼“没掉下来”。

### 3.8 发现固定、重复的接触穿透：约 16–20 mm

最初输出：

```text
Contacts detected: 10–12
geom_87 <-> geom_89 | distance ≈ -0.01995 m
geom_87 <-> geom_95 | distance ≈ -0.01594 m
```

**现象特点：** 同一对几何体的 `dist` 从 `t=0` 开始就接近固定值，即使手指目标变化，最大穿透也几乎不变；这不像是正常抓住外部物体时的动态接触。

**关键改进：** 最初只记录 Geom 数字，但数字对解释没有帮助。后来在 `Contact Diagnostics` 中使用：

```python
b1 = int(model.geom_bodyid[g1])
b2 = int(model.geom_bodyid[g2])
body1 = model.body(b1).name or f"body_{b1}"
body2 = model.body(b2).name or f"body_{b2}"
```

因此确定三对持久接触全部来自 `right_wrist_yaw_link` 与手指根部：

| Geom 对（当时的 ID） | Body 对 | 典型最深穿透 |
| --- | --- | --- |
| `87 ↔ 89` | `right_wrist_yaw_link` ↔ `right_hand_thumb_0_link` | `-0.019952 m`（约 19.95 mm） |
| `87 ↔ 99` | `right_wrist_yaw_link` ↔ `right_hand_index_0_link` | `-0.015944 m`（约 15.94 mm） |
| `87 ↔ 95` | `right_wrist_yaw_link` ↔ `right_hand_middle_0_link` | `-0.015908 m`（约 15.91 mm） |

**重要：** Geom ID 是当前编译模型的内部编号，改动 XML、增加物体后可能改变；**不要在正式控制代码里硬编码 `87/89/95/99`**。应该使用 `Body` 名称和运行时查询。

**诊断判断：** 这三组属于手腕/手指根部的装配处。尤其因为本实验将其他关节固定，此类相邻装配几何重叠在模型中被作为有效碰撞处理，是导致接触振荡的高度可信原因。仅凭日志不能断言“制造了新的几何穿透”，更准确的说法是**固定关节实验改变了相关碰撞过滤/接触行为**。

### 3.9 最终针对性修复：只排除三组装配相邻 Body 的碰撞

**正确做法：** 在将 XML 编译为 MuJoCo 模型**之前**，给内存 XML 添加三个 `<contact><exclude .../></contact>`；这里使用的是 MuJoCo 官方支持的 **Body 对**排除机制（不是 Geom ID）。

**代码位置：** `# Remove incompatible saved poses` 后，`# Make mesh paths absolute` / `MjModel.from_xml_string` 前（见 §5.2）。

```python
contact = root.find("contact")
if contact is None:
    contact = ET.SubElement(root, "contact")

for finger_body in [
    "right_hand_thumb_0_link",
    "right_hand_index_0_link",
    "right_hand_middle_0_link",
]:
    ET.SubElement(
        contact, "exclude",
        body1="right_wrist_yaw_link",
        body2=finger_body,
    )
```

**不要用以下方式掩盖问题：**

- 不要在之后准备真实接触抓取时把 **所有接触**都关掉。
- 不要把三根手指与外部物体之间的碰撞也排除。
- 不要直接删除官方 mesh 或覆盖官方 XML。
- 不要通过盲目增大电机力矩去强行克服 20 mm 的装配穿透。

**最终观察结果（已在 Web Viewer 中验证）：**

```text
Time: 11.00s
Contacts detected: 0
No contacts

right_hand_index_0_joint: target=0.5536, actual=0.5436, velocity=0.0584
right_hand_index_1_joint: target=0.6151, actual=0.6039, velocity=0.0649
```

以及 `t=10.50–11.50s` 时七关节的速度基本平滑、位置误差小、力矩没有饱和。**这说明本次固定身体、无外部物体的测试已消除了那三对异常接触，并恢复了平稳跟踪。**

**不代表：** 正式 G1 模型的所有潜在自碰撞都已排除；更不代表已经验证抓取物体的接触力、摩擦或保持能力。

---

## 4. 当前已验证的实验配置（截至本次记录）

> 本表是配置快照。**不知道某个数值具体是什么意思或怎么改，请回到 §2.5**；此处的数字不是实体硬件认证参数。

| 配置 | 当前设置 | 用途 / 注意事项 |
| --- | --- | --- |
| 模型来源 | `unitree_ros/.../g1_29dof_with_hand_rev_1_0.xml` | 官方带手模型 |
| 实验自由度 | 只保留右手 7 个关节 | 其他关节在**内存模型**中被锁定；非最终全身模型 |
| 模型编译 | `ET.parse` → 删除其他关节和执行器 → `MjModel.from_xml_string` | 不改官方文件 |
| 步长 | `model.opt.timestep = 0.001` | 1 ms |
| 积分器 | `mjINT_IMPLICITFAST` | 调试用 |
| 重力 | `model.opt.gravity[:] = 0` | **仍关闭**；抓取物体实验时要重新评估 |
| 接触 | 开启 | 不再全局禁用 |
| 指定排除对 | 手腕 ↔ 拇指根部、食指根部、中指根部 | 只在本次固定身体调试模型中已验证 |
| 右手关节阻尼 | `model.dof_damping[dof_id] = 0.05` | 调试用，不等于实体电机阻尼 |
| 控制律 | `kp=0.3`, `kd=0.002` | 简单 PD，无接触负载标定 |
| 人为控制限幅 | `[-0.15, +0.15]` | 每个 motor 的测试限幅 |
| 目标 | `start = lo + 0.25*(hi-lo)`，`goal = start + 0.12*(hi-lo)` | 只用于周期性关节验证，不是标准 grasp pose |
| 周期 | 8 秒 | `0.5 - 0.5*cos(2*pi*time/8)` |
| 动力学 | 每次 `step_fn` 回调执行**一次** `mujoco.mj_step` | 注意不要重复调用 |
| 日志 | 每 0.5 秒 | `target`、`actual`、`velocity`、`torque`、`data.ncon`、唯一接触对 |
| Viewer | `viser.ViserServer(host="127.0.0.1", port=8765)`；`Viewer(..., step_fn=step)` | 服务器运行、Mac 浏览器通过 SSH 隧道查看 |

**当前已达到的里程碑：** 固定身体下，右手 7-DoF 的 motor-PD 控制能在 MuJoCo 中稳定执行周期小幅运动；消除了被确认的三组手腕/指根虚假接触。**尚未完成：** 标准的张开/握拳/捏取抓取姿态、外部物体接触抓取、带重力测试、右臂 IK、碰撞感知规划与完整 Pick-and-Place。

---

## 5. 文件定位图：需要改什么时，去代码哪里改？

本节按单一 `sim_workspace/robots/g1/view_g1.py` 的逻辑顺序排列。可在 VS Code 中搜索对应的**锚点**。

### 5.1 模型路径：`XML = ROOT / (...)`

```python
ROOT = Path(__file__).resolve().parents[1]
XML = ROOT / (
    "third_party/unitree_ros/robots/g1_description/"
    "g1_29dof_with_hand_rev_1_0.xml"
)
```

**用途：** 更换机器人版本、切换模型时，先改这里。基础 G1 的 `unitree_mujoco/.../scene.xml` **不是**本次 Dex3-1 控制模型。修改路径后需要重新核对关节名、gear、meshdir、碰撞排除、限位，不能假设都不变。

### 5.2 固定身体 + 排除装配接触：`ET.parse` 到 `MjModel.from_xml_string`

```python
tree = ET.parse(XML)
root = tree.getroot()

hand_names = {
    j.get("name")
    for j in root.iter("joint")
    if (j.get("name") or "").startswith("right_hand_")
}
assert len(hand_names) == 7

for parent in root.iter():
    for child in list(parent):
        if child.tag in ("joint", "freejoint"):
            if child.get("name") not in hand_names:
                parent.remove(child)

actuators = root.find("actuator")
if actuators is None:
    raise RuntimeError("Missing actuator section")
for motor in list(actuators):
    if motor.get("joint") not in hand_names:
        actuators.remove(motor)

keyframes = root.find("keyframe")
if keyframes is not None:
    root.remove(keyframes)

contact = root.find("contact")
if contact is None:
    contact = ET.SubElement(root, "contact")
for finger_body in [
    "right_hand_thumb_0_link",
    "right_hand_index_0_link",
    "right_hand_middle_0_link",
]:
    ET.SubElement(
        contact,
        "exclude",
        body1="right_wrist_yaw_link",
        body2=finger_body,
    )

compiler = root.find("compiler")
if compiler is None:
    compiler = ET.SubElement(root, "compiler")
compiler.set("meshdir", str(XML.parent / "meshes"))

model = mujoco.MjModel.from_xml_string(
    ET.tostring(root, encoding="unicode")
)
```

**适用：** 当前固定 G1 身体、只测试手指。**不适用：** 恢复右臂自由度 / 做真实机械臂轨迹规划时无脑复用；届时必须修改“保留哪些关节”的策略，检查被锁定关节的角度，以及复核 `exclude` 是否仍然需要。

**注意：** 每次启动脚本会重新解析**原始 XML**，因此上述排除项不会在文件上累积；若以后把模型另存为自定义 XML，再添加排除项前要检查是否已有重复声明。

### 5.3 模型物理选项与手指目标：`model.opt` / `hand.append`

```python
model.opt.timestep = 0.001
model.opt.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
model.opt.gravity[:] = 0
model.opt.disableflags &= ~int(mujoco.mjtDisableBit.mjDSBL_CONTACT)

for j in range(model.njnt):
    if model.joint(j).name.startswith("right_hand_"):
        model.dof_damping[model.jnt_dofadr[j]] = 0.05
```

**若手指运动太小/要扩大到 7 关节：** 搜索 `start = ...` 和 `goal = ...`，在生成 `hand` 时修改目标。当前完整七关节版本采用：

```python
start = lo + 0.25 * (hi - lo)
goal = start + 0.12 * (hi - lo)
```

如果只测试食指，就把其他关节的 `goal = start`；不要把 `model.jnt_range` 的上下界直接当电机输入。

### 5.4 PD 控制和积分：`def step(model, data)`

```python
def step(model, data):
    global next_log_time
    phase = 0.5 - 0.5 * np.cos(2.0 * np.pi * data.time / 8.0)

    for name, jid, aid, qadr, vadr, start, goal in hand:
        target = start + phase * (goal - start)
        q = data.qpos[qadr]
        dq = data.qvel[vadr]

        kp = 0.3
        kd = 0.002
        torque = kp * (target - q) - kd * dq

        low, high = -0.15, 0.15
        if model.jnt_actfrclimited[jid]:
            joint_low, joint_high = model.jnt_actfrcrange[jid]
            low = max(low, joint_low)
            high = min(high, joint_high)

        data.ctrl[aid] = np.clip(torque, low, high)

    mujoco.mj_step(model, data)  # 一次且仅一次，必须在 for 循环外
    # ...日志见 5.5 / 5.6...
```

**排查提示：**

- `NameError: next_log_time` → 在函数前初始化 `next_log_time = 0.0`。
- `SyntaxError` / `IndentationError` → `global next_log_time` 必须在 `def step` 内、与 `for` 同级，并在引用变量前；`mj_step` 与 `for` 同级。
- 输出力矩频繁触及 `±0.15` → 先检查数值积分、接触、速度和关节目标，不要只增加力矩限幅。
- 关节编号错乱 → 使用运行时的 `mj_name2id()` 和 `jnt_qposadr / jnt_dofadr`，而不是硬编码 `37` 或 `42` 等原模型索引。

### 5.5 日志定时：`next_log_time`

```python
next_log_time = 0.0
# 在 def step 中，mj_step 之后：
if data.time >= next_log_time:
    print(f"Time: {data.time:.2f}s")
    # ...print target/actual/velocity/torque...
    next_log_time += 0.5
```

**注意：** 上面仅为片段，实际 `if ...` 要缩进在 `step()` 内。若要更详尽的接触诊断，使用下一小节。

### 5.6 接触排查：`Contact Diagnostics`

```python
print("Contacts detected:", data.ncon)
pairs = {}
for i in range(data.ncon):
    c = data.contact[i]
    g1, g2 = int(c.geom1), int(c.geom2)
    key = tuple(sorted((g1, g2)))
    if key not in pairs:
        pairs[key] = {"count": 0, "active": 0, "min_dist": float("inf")}
    pairs[key]["count"] += 1
    pairs[key]["active"] += int(c.efc_address >= 0)
    pairs[key]["min_dist"] = min(pairs[key]["min_dist"], float(c.dist))

for (g1, g2), info in sorted(pairs.items(), key=lambda x: x[1]["min_dist"]):
    b1, b2 = int(model.geom_bodyid[g1]), int(model.geom_bodyid[g2])
    print(
        f"Bodies: {model.body(b1).name or b1} <-> {model.body(b2).name or b2}; "
        f"points={info['count']}; active={info['active']}; "
        f"deepest={info['min_dist']:.6f} m"
    )
```

**理解：**

- `data.ncon` 是当前检测到的总接触数；同一个 Geom 对可以出现多个接触点。
- `c.efc_address >= 0` 表示有对应激活约束的接触；`ncon` 不等于都在产生接触力。
- `c.dist < 0` 通常表示几何穿透；如果刚启动 `t=0` 就持续出现约 `-0.02m`，先检查装配几何与碰撞过滤。
- `geom_87` 等 ID 不是可维护的配置，应打印对应 Body 名称后诊断。
- 当本场景未加入外部物体、且只排除了三对装配接触时，**`Contacts detected: 0` 是预期基线**；这并不能证明抓取场景的接触已经正确。

### 5.7 Viewer 设置：`ViserServer` / `Viewer(...).run()`

```python
server = viser.ViserServer(host="127.0.0.1", port=8765)
Viewer(model, data, server=server, step_fn=step).run()
```

若显示了 G1 但不动：先确认是否传递了 `step_fn=step`，仿真是否处于运行状态，控制目标的 `start` 和 `goal` 是否不同，以及浏览器视角是否放大到右手。

---

## 6. 复现、验证与常见运维错误

### 6.1 每次改完代码的固定流程

在服务器：

```bash
conda activate simulator
cd ~/projects/humanoid_manipulation
python -m py_compile sim_workspace/robots/g1/view_g1.py
# 关闭旧的 Viewer（当前终端 Ctrl+C；tmux 内先 attach）
python sim_workspace/robots/g1/view_g1.py
```

在 Mac：保持 SSH 转发进程运行，浏览器刷新 <http://localhost:8765>。

**不要：** 保存 VS Code 后就直接在旧窗口期待 Python 热更新；也不要同时让两个 Viewer 占用同一个 8765 端口。

### 6.2 端口或进程问题

```bash
# 服务器上查看 8765 端口
ss -ltnp | grep 8765

# 查看可能的 Python Viewer 进程（只用于检查）
pgrep -af 'python.*view_g1'
```

优先停止**你自己的**旧 Viewer，不要随意结束其他用户进程。

### 6.3 远程 Web 可视化问题

| 现象 | 快速处理 |
| --- | --- |
| `localhost:8765` 打不开 | 服务器 `python sim_workspace/robots/g1/view_g1.py` 是否运行？Mac `ssh -N -L ...` 是否存活？ |
| `Address already in use` | 上一 Viewer 没关，先关闭自己的旧进程，再重启 |
| 浏览器显示旧动作 | 服务器端没有重启新代码；Ctrl+C 后重新 `python sim_workspace/robots/g1/view_g1.py` |
| 机器人身体乱动 | 查看是否误换为含 floating base 的原始完整模型，或没有锁定其他关节 |
| 画面有手但不动 | 确认使用 Dex3 XML、目标有变化、`step_fn=step`、Viewer 正在推进 |
| 只有 Index 0 动 | 先查 `moving_joints` / `goal`，再查 `target` 与 `actual` |
| 速度很大、角度差不多 | 检查 `mj_step` 是否重复、`kd`、`data.ncon` 和接触对 |
| 初始 `ncon=10–12`、手腕与指根接触 | 搜索 `<contact><exclude>` 的三对 Body 是否在模型编译**之前**加上 |
| 改完碰撞排除却还检测到旧接触 | 确保新 `model = MjModel.from_xml_string(...)` 已重新编译、Viewer 已重启；打印 `body1/body2` 核对 |

### 6.4 最小验收日志应该长什么样？

**已实际看到：**

```text
Fixed-body Dex3-1 physics test ready
Active joints: 7
Active actuators: 7

Time: 11.00s
right_hand_index_0_joint: target=0.5536, actual=0.5436, velocity=0.0584
right_hand_index_1_joint: target=0.6151, actual=0.6039, velocity=0.0649
Contacts detected: 0
No contacts
```

如果你修改了运行模型、重力、接触或控制目标，数值当然不必完全相同。**验收的是行为：** 7 关节都能跟踪，速度没有高频大幅振荡，当前无外部物体的场景没有意外装配接触，Viewer 正常刷新。

---

## 7. 现在做到了什么？下一阶段做什么？

### 已完成 / 已验证

- [x] G1 + Dex3-1 官方 MJCF 在 MuJoCo 中加载成功（完整模型 43 个执行器，左右手各 7 个关节）。
- [x] 统一入口 `sim_workspace/robots/g1/view_g1.py`，Web Viewer 可在 Mac 浏览器查看远端实际仿真状态。
- [x] 右手 7-DoF 的运动学预览（直接改 `qpos`，已不作为最终物理控制）。
- [x] 当前固定身体测试模型只保留 7 个右手关节与执行器。
- [x] 通过 `data.ctrl` + `mujoco.mj_step` 完成 7-DoF PD 物理关节控制。
- [x] 解决控制回调缩进与重复积分问题。
- [x] 利用日志定位 Index 1 跟踪异常，调小 `kd` 后验证正常跟踪。
- [x] 定位三组手腕—指根装配几何持续接触，并通过 `contact/exclude` 在内存模型中针对性排除。
- [x] 在当前无物体场景中观察到 `Contacts detected: 0`，并恢复平滑的七关节跟踪。

### 尚未实现 / 不应误称已完成

- [ ] 真实抓取手型（Open / Pregrasp / Close / Hold），手指接触协同策略。
- [ ] 加入圆柱体/桌面和正确的物理质量、接触、摩擦参数。
- [ ] 恢复重力后的抓取稳定性与物体保持能力。
- [ ] 固定基座但**保留右臂 7 DoF** 的正式 manipulation 模型，以及初始姿态。
- [ ] 正向/逆运动学、末端目标位姿、坐标变换和避障路径。
- [ ] Lift / Transfer / Place / Release 和成功验证、失败用例。

### 推荐的下一个开发里程碑

**M2：Dex3-1 Contact-Based Grasping（右手先抓住一个圆柱体）。**

实施时仍可使用唯一 Viewer 入口，但**不要仅靠当前 `qpos` 正弦往复或“无重力 + 固定全部 G1 关节”就宣称抓取成功**。需要可碰撞物体、恢复重力、经过验证的手型，以及接触后物体稳定保持的成功判据。之后再将完整 G1 的右臂自由度恢复，实现 IK 与放置动作。

---

## 8. 附录：当前 `view_g1.py` 的单文件参考版本

> 这是根据本次对话中最后已验证的设置整理出的**可复现参考实现**，供代码损坏、覆盖或缩进混乱时恢复。不代表服务器现有文件的逐字节副本，也不是最终抓取控制系统。项目代码仍然**只保留一个 `sim_workspace/robots/g1/view_g1.py`**。以下代码并未在当前会话实际连入你的服务器执行，请以服务器运行测试为准。

```python
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import mujoco
import viser
from mjviser import Viewer

ROOT = Path(__file__).resolve().parents[1]
XML = ROOT / (
    "third_party/unitree_ros/robots/g1_description/"
    "g1_29dof_with_hand_rev_1_0.xml"
)

# 1) Load the official XML into memory; leave the upstream file unchanged.
tree = ET.parse(XML)
root = tree.getroot()
hand_names = {
    j.get("name")
    for j in root.iter("joint")
    if (j.get("name") or "").startswith("right_hand_")
}
assert len(hand_names) == 7, f"Expected 7 right-hand joints, got {len(hand_names)}"

# 2) Fixed-body, hand-only diagnostic model.
for parent in root.iter():
    for child in list(parent):
        if child.tag in ("joint", "freejoint"):
            if child.get("name") not in hand_names:
                parent.remove(child)

actuators = root.find("actuator")
if actuators is None:
    raise RuntimeError("No actuator section in XML")
for motor in list(actuators):
    if motor.get("joint") not in hand_names:
        actuators.remove(motor)

keyframes = root.find("keyframe")
if keyframes is not None:
    root.remove(keyframes)

# 3) Exclude three attachment overlaps exposed in this fixed-body model.
contact = root.find("contact")
if contact is None:
    contact = ET.SubElement(root, "contact")
for finger_body in (
    "right_hand_thumb_0_link",
    "right_hand_index_0_link",
    "right_hand_middle_0_link",
):
    ET.SubElement(
        contact,
        "exclude",
        body1="right_wrist_yaw_link",
        body2=finger_body,
    )

compiler = root.find("compiler")
if compiler is None:
    compiler = ET.SubElement(root, "compiler")
compiler.set("meshdir", str(XML.parent / "meshes"))

model = mujoco.MjModel.from_xml_string(
    ET.tostring(root, encoding="unicode")
)
model.opt.timestep = 0.001
model.opt.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
model.opt.gravity[:] = 0  # ONLY for this diagnostic, NOT real grasping
model.opt.disableflags &= ~int(mujoco.mjtDisableBit.mjDSBL_CONTACT)

for j in range(model.njnt):
    if model.joint(j).name.startswith("right_hand_"):
        model.dof_damping[model.jnt_dofadr[j]] = 0.05

data = mujoco.MjData(model)
hand = []

for j in range(model.njnt):
    name = model.joint(j).name
    if name not in hand_names:
        continue

    aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
    if aid < 0:
        raise RuntimeError(f"Missing hand actuator: {name}")
    gear = float(model.actuator_gear[aid, 0])
    if not np.isclose(gear, 1.0):
        raise RuntimeError(f"Unexpected actuator gear for {name}: {gear}")
    if not model.jnt_limited[j]:
        raise RuntimeError(f"Missing joint limits: {name}")

    qadr = model.jnt_qposadr[j]
    vadr = model.jnt_dofadr[j]
    lo, hi = model.jnt_range[j]
    start = lo + 0.25 * (hi - lo)
    goal = start + 0.12 * (hi - lo)
    data.qpos[qadr] = start
    hand.append((name, j, aid, qadr, vadr, start, goal))

assert len(hand) == 7
mujoco.mj_forward(model, data)

next_log_time = 0.0


def step(model, data):
    global next_log_time

    # Smooth 8-second period: start -> goal -> start.
    phase = 0.5 - 0.5 * np.cos(2.0 * np.pi * data.time / 8.0)

    # Set all motor commands before doing a single physics step.
    for name, jid, aid, qadr, vadr, start, goal in hand:
        target = start + phase * (goal - start)
        q = data.qpos[qadr]
        dq = data.qvel[vadr]
        kp, kd = 0.3, 0.002
        torque = kp * (target - q) - kd * dq
        low, high = -0.15, 0.15
        if model.jnt_actfrclimited[jid]:
            joint_low, joint_high = model.jnt_actfrcrange[jid]
            low = max(low, float(joint_low))
            high = min(high, float(joint_high))
        if low > high:
            raise RuntimeError(f"Invalid actuator force interval for {name}")
        data.ctrl[aid] = np.clip(torque, low, high)

    mujoco.mj_step(model, data)

    if data.time >= next_log_time:
        print(f"\n{'=' * 55}")
        print(f"Time: {data.time:.2f}s")
        print("\n=== Dex3-1 Joint States ===")
        for name, jid, aid, qadr, vadr, start, goal in hand:
            target = start + phase * (goal - start)
            print(
                f"{name}: "
                f"target={target:.4f}, "
                f"actual={data.qpos[qadr]:.4f}, "
                f"velocity={data.qvel[vadr]:.4f}, "
                f"torque={data.ctrl[aid]:.4f}"
            )

        print("\n=== Contact Diagnostics ===")
        print(f"Contacts detected: {data.ncon}")
        pairs = {}
        for i in range(data.ncon):
            c = data.contact[i]
            g1, g2 = int(c.geom1), int(c.geom2)
            key = tuple(sorted((g1, g2)))
            if key not in pairs:
                pairs[key] = {
                    "count": 0,
                    "active": 0,
                    "min_dist": float("inf"),
                }
            pairs[key]["count"] += 1
            pairs[key]["active"] += int(c.efc_address >= 0)
            pairs[key]["min_dist"] = min(
                pairs[key]["min_dist"], float(c.dist)
            )

        if not pairs:
            print("No contacts")
        else:
            for (g1, g2), info in sorted(
                pairs.items(), key=lambda item: item[1]["min_dist"]
            ):
                geom1 = model.geom(g1).name or f"geom_{g1}"
                geom2 = model.geom(g2).name or f"geom_{g2}"
                b1, b2 = int(model.geom_bodyid[g1]), int(model.geom_bodyid[g2])
                body1 = model.body(b1).name or f"body_{b1}"
                body2 = model.body(b2).name or f"body_{b2}"
                print(f"Geom {g1} ({geom1}) <-> Geom {g2} ({geom2})")
                print(f"  Bodies: {body1} <-> {body2}")
                print(f"  Contact points: {info['count']}")
                print(f"  Active points: {info['active']}")
                print(f"  Deepest penetration: {info['min_dist']:.6f} m")

        print("=" * 55, flush=True)
        next_log_time += 0.5


print("Fixed-body Dex3-1 physics test ready")
print("Active joints:", model.njnt)
print("Active actuators:", model.nu)

server = viser.ViserServer(host="127.0.0.1", port=8765)
Viewer(model, data, server=server, step_fn=step).run()
```

---

## 9. 参考资料与维护约定

- [Unitree 官方 `unitree_ros` / G1 description](https://github.com/unitreerobotics/unitree_ros/tree/master/robots/g1_description)：模型版本与手部自由度。
- [MuJoCo XML Reference — `contact/exclude`](https://mujoco.readthedocs.io/en/stable/XMLreference.html#contact-exclude)：指定 **Body 对**忽略碰撞。
- [MuJoCo 官方文档](https://mujoco.readthedocs.io/)：`mj_step`、`mj_forward`、`mjData.contact`、执行器及控制力学。
- [mjviser Web Viewer](https://github.com/mujocolab/mjviser)：浏览器可视化。

**维护原则：**

1. 新的调试发现直接记录到本文件；若已稳定，只在 `sim_workspace/robots/g1/view_g1.py` 保留一份正式实现。
2. 每次改动注明：**症状、复现条件、修改锚点、实际证据、是否解决、是否只是调试设置**。
3. 不将 `t=0` 无负载、无重力的平稳控制夸大为“真实抓取成功”。
4. 不把实验室密码、令牌或 SSH 私钥写进 Markdown；若上传公开 GitHub，先审查私有地址与服务器信息。
5. 未来开始右臂 IK / 真实接触抓取时，应重审固定关节方式、身体姿态、重力、装配碰撞排除和执行器参数，而不是直接沿用手部测试的全部临时配置。
