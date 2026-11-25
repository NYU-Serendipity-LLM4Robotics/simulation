# 工程笔记

跨模块变更、故障根因和验证结果记录在这里。每次追加一条，包含日期和负责人、症状和复现命令、修改位置、验证产物、剩余问题。运行时日志由程序按轮次保存。常见故障入口见 [排错手册](TROUBLESHOOTING.md)。

## 2026-11-23 · 修正固定台架穿透验收

**原因**。旧回放确有接触重叠，逐步复跑实测球—拇指 2.79 mm、球—接收台 7.47 mm。拇指可视网格的凸包与物理碰撞凸包在最差的对应帧给出了相同距离。原仿真有质量、重力、摩擦和执行器，但沿用了 20 ms 的默认接触时间常数，原验收也没有查穿透。

**修复**。在 `sim_workspace/robots/g1/pick_place.py:19–29` 设置明确的材料摩擦、0.50 ms 步长、2.50 ms 接触时间常数和手指 0.85 N·m 扭矩上限。`101–122` 启用右手指间自碰撞和接收盘对机器人的碰撞。为留出手部空间，目标盘的世界 Y 由 -0.244 m 调到 -0.310 m。仿真依然采用固定底座及固定躯干，其余全身自碰撞未纳入。摩擦值经台架仿真试跑选择，实物系数待标定。

**证据**。`sim_workspace/runs/episodes/g1-contact-v2` 两次重复结果一致。仿真逐步的最大穿透为球—手 0.143 mm、球—接收台 0.454 mm、手—接收盘 0.102 mm。超过 0.50 mm 的峰值会写入 `penetration_check`，原本判放置成功的运行会被拒绝为 `excess_penetration`。旧参数用新门槛复跑返回该失败状态，证据在 `sim_workspace/runs/archive/g1-tuning/legacy-penetration-recheck/diagnostic.json`。新版球无手指接触力地在接收台上稳定 2.25 s。浏览器 8767 端口指向新版轨迹。

## 2026-11-23 · 三个运行入口和 G1 动力学验收

**接口**。`workspace.py go2` 只加载 Go2，`team` 加载两台机器人并用版本 1 交接走到 G1 预抓取，`g1-fixed` 在台架上从已知球心走完整个抓放。旧 `run` 命令作为 `team` 的别名。[交接协议](HANDOFF.md) 定义字段、世界坐标和失败状态。

**夹持和释放**。用 Unitree 原始碰撞网格标定 Dex3-1 夹持位形，初版可离台、横移，但球松手后仍会托在手指上或滚离平板。最后在抬升后转腕，释放，再撤离。接收盘斜面只与球碰撞，避免原先的斜面干扰闭合，底板使用滚动摩擦。台架固定底座与躯干，关闭了机器人自碰撞。模型保留球的自由关节、重力和物理接触。没有物体连接约束。物体只在初始条件赋值，之后由 MuJoCo 自由积分。

**证据**。`sim_workspace/runs/episodes/g1-fixed-acceptance` 和 `g1-fixed-repeat` 的报告一致。球峰值举高 9.43 cm，横移及转腕期间最低 9.37 cm，放手后在目标台无手指接触稳定 2.25 s，最终 XY 偏差 2.13 cm。`physics_trace.jsonl` 记录接触力和执行器目标，反例 `g1-fixed-unreachable` 返回 `ik_failed`。同 seed 的 Go2 单跑与双机交接字段一致，双机批量 6/6 到达预抓取。共享场景中的物理抓放和底盘动态平衡仍待接入。

## 2026-11-23 · 分开 Go2 和 G1 模块

**结构**。Go2 模型、相机、四腿关节链、扫描与定位收进 `sim_workspace/robots/go2/scout.py`。G1 模型、站姿、双腿与手臂关节链及预抓取收进 `sim_workspace/robots/g1/manipulation.py`。G1 检查和 Dex3-1 查看器迁入同一目录，移除了空的顶层 `src/`。共享场景和任务交接在 `workspace.py`，通用 IK 和足端轨迹生成在 `kinematics.py`。

**验证**。`sim_workspace/runs/suites/robot-split/` 六轮全部定位和预抓取成功。逐轮比较了与整理日志时的基准报告、所有关节状态和 RGB-D 数组，结果一致。现行源码行号见 [排错手册](TROUBLESHOOTING.md)。

## 2026-11-23 · 整理目录与增加运行日志

**变更**。项目根目录增加导航。两份原有 G1 和 Dex3-1 文档移入 `docs/`，内容经 SHA-256 核对一致。仿真目录中的数据、benchmark 控制台输出和回放日志按用途归到 `sim_workspace/runs/`，早期试验进 `archive/`。

**日志实现**。`sim_workspace/workspace.py:110–135` 在输出目录创建 `run.log`，记录 UTC 时间、输入、解释器、阶段、状态及异常堆栈。`sim_workspace/bench.py:73–92` 将真实 benchmark 命令、标准输出、标准错误及退出码持久化。`report.json` 继续提供任务事件和指标，`qpos.npy` 提供姿态回放。

**验证证据**。`sim_workspace/runs/episodes/maintenance-check/` 包含正常、定位失败注入和 Go2 XML 缺失注入的三份日志。`sim_workspace/runs/suites/maintenance-check/summary.json` 复跑六轮，六轮均生成定位和预抓取。基于当前日志的修复流程见排错手册。

## 2026-11-23 · 接入真实模型的关节级 IK

**问题**。第一轮演示用直接设置根位姿的方式走到交接位置，无法测关节限位、足端轨迹或手部预抓取的可达性。

**处理**。`sim_workspace/kinematics.py:37–89` 实现基于 MuJoCo Jacobian 的阻尼最小二乘，包含限位、收敛检查与失败回退。`sim_workspace/kinematics.py:92–123` 生成共用足端轨迹。机器人关节链和 G1 预抓取现位于各自机器人模块。最初 G1 从伸直腿开始时，足端求解在初始阶段停住。改为 `sim_workspace/robots/g1/manipulation.py:41–47` 的屈膝站姿后，再用保留足姿的轨迹与较高的预抓取点继续求解。

**证据**。`sim_workspace/runs/suites/ik-pose/summary.json` 的三组种子共六轮均达到预抓取。最大足端残差 1.98 mm，最大预抓取残差 0.88 mm 和 0.23°。核对了回放中求解关节的限位及正运动学，并验证不可达回退和手臂穿桌拒绝。

**待接入**。底盘路线仍为预设，双足和四足轨迹用于运动学验证。动态平衡、Dex3-1 夹持与抬放、实机工具坐标标定、VLA 和 LLM 编排仍由对应模块继续开发。

## 2026-11-23 · 双机器人场景与外部 benchmark 初测

最初的 RGB-D 定位数据和报告移入 `sim_workspace/runs/archive/suite-smoke/`。当时使用仿真分割掩码和脚本位姿，六轮有定位报告。参见 [早期结果](baseline-results.md)。独立环境内，ManiSkill 两个官方 G1 任务均 reset 并运行五步随机动作。LIBERO Spatial 的启动命令已检查，实际目录读取需要 LIBERO 环境。

## 新条目模板

- 日期和负责人
- 现象、最后一个有效阶段及复现命令
- 根因与修改位置，写文件、函数和当前行号
- 复测命令、结果及对应日志或报告路径
- 尚未验证的情况和下一步

### G1 固定台架重力回归

旧版球使用 MuJoCo 默认重力，G1 活动部件使用 `gravcomp=1`。`dex3-contact-v3` 显式设定世界 Z 轴重力为 −9.81 m/s²，取消模型级补偿，仍由限力位置伺服跟踪 IK。两块静态台面加装可碰撞支柱。回放界面标明保存轨迹。

有重力时，球 0.20 s 下落 196.69 mm，末速度 −1.962 m/s。停用全部执行器后手腕 0.10 s 下坠 27.85 mm。零重力对照位移均为零。六项测试通过。`sim_workspace/runs/episodes/g1-gravity-v3/report.json` 记录默认抓放通过，球抬升 94.1 mm、水平落点误差 13.2 mm、在盘中稳定 2.25 s、最大几何穿透 0.472 mm，阈值 0.50 mm。

此结果覆盖固定基座抓放。全身平衡、Go2 动力学行走、双机动力学接续和硬件接触参数仍待验证。

### 双机共享房间动力学

加入通用实验室布局。三张四腿桌、两只带边托盘和任务物体均带自由关节，物体通过重力、接触与摩擦运动。Go2 和 G1 都使用浮动基座、足底接触与限幅关节力矩。首次通电试验中 G1 在 0.294 s 失稳，后续以实有足底几何初始化离地间隙，加入限幅的 G1 足踝姿态反馈。手指力矩上限及家具材料参数为仿真假设。

三个物体摆放种子通电站立各 10 s，末态 Go2 倾角 0.61°、G1 倾角 1.27°。断电对照中 Go2 在 0.158 s、G1 在 0.244 s 达到失稳判据。对蓝方块施加 8 N 水平力 160 ms 后，块从桌上掉地，中心高度约 0.045 m。结果位于 sim_workspace/runs/episodes/room-physics-seed-0 等三个种子目录及 room-physics-unpowered，六项既有单测通过。

8767 改为实时 MuJoCo。房间基线覆盖关节力矩站立与可重复接触扰动。双机运动学入口和固定抓放回归各自保留。释放行走时的稳定性、移动抓放和双机动力学交接仍未验证，真机与现场参数待实测。

### 双机房间端到端任务

`team` 主入口改为一次连续的 MuJoCo 物理场景。Go2 依据速度、姿态及足底接触反馈生成摆腿 IK 目标。相机从南侧搜索并由 RGB 和深度拟合球心。G1 在桌边由限幅关节力矩站立，使用全局坐标在线解腕部 IK，通过 Dex3 拇指和食指夹持，抬升、转运、开指并先竖直撤离。

失败证据保留。g 轮在闭指时发现原站姿的膝部碰撞与失稳。k 轮稳定站立，但中指卡在托座浅边，导致抬升只有约 7 mm。l 轮已抬升约 74 mm，但撤离带出球。加宽站姿、使用匹配原模型足底尺寸的鞋底碰撞几何、中指保持张开、提高释放点和竖直撤离后，种子 0 与 1 均通过绿色托盘任务。主报告为 sim_workspace/runs/episodes/team-physics-reference/report.json。种子 0 最大骨盆倾角 1.31°，采样双足接触率 100%，Go2 走了 0.491 m，定位误差 1.73 mm，球在托盘内稳定 2.703 s。最大球体穿透 0.554 mm，发生于落盘，小于 0.75 mm 阈值。

黄色托盘试验在转运阶段因 IK 不可达止步。蓝色方块明确返回 unsupported_grasp_object。G1 初始在桌边，真机标定和自由行走尚待完成。

## 2026-11-24 · 普通桌面双机试验

新增 `workspace.py lab-trial` 和 `bench.py lab-suite`。普通桌面由四腿木桌和两只可移动托盘组成，球、方块和圆柱各有自由关节。桌面上直接摆放物体。球的接触启用滚动阻力，三个工作件的接触参数仍是仿真假设。

- 场景在 [task_room.py 的 lab_layout](../sim_workspace/task_room.py#L130)。托盘初始化时留足边缘和相互间距。一轮调试中两只托盘初始重叠，求解器把绿托盘挤到地上。检查同次数据的 `tray_xyz_m` 可以识别这类故障。
- [lab_trial.py 的 observe](../sim_workspace/lab_trial.py#L19) 用普通 RGB-D 做颜色和形状拟合。发现全在远侧槽位的物体漏检时，用 Go2 轨迹和已保存的 RGB-D 检查桌沿及托盘遮挡、取景角度和颜色掩码。已确认三个漏检都来自同一远侧位置，遮挡和取景的各自影响仍需区分。
- [G1 步态适配](../sim_workspace/robots/g1/locomotion.py#L50) 运行 Unitree ONNX 策略。到位后先发零速度指令，再以实测关节姿态切换到力矩站立反馈。到位失败检查 `g1_walk`，抬手后失衡检查 `g1_stabilize` 和后续采样的足底接触。
- [room_grasp.py 的 follow](../sim_workspace/robots/g1/room_grasp.py#L125) 生成平桌抓取与撤手的位姿 IK。调试中球落盘撞击的几何重叠为 1.001 mm。保留 0.75 mm 验收线并降低释放高度，重复运行降至 0.471 mm。

使用 MuJoCo 3.17.0 在 macOS 执行种子 0、1、2 的九轮组合，绿托盘抓放通过三轮。三轮在 Go2 搜索失败，两轮在向黄托盘转运时 IK 不收敛，一轮在抬升时失去有效抓握。逐轮数据见 `sim_workspace/runs/flat-tabletop-evaluation/summary.json` 和同目录下每轮的 `report.json`、`run.log` 及 `physics_trace.jsonl`。验收检查抬升及空中接触、转运期间的持续夹持、物体在托盘内稳定且脱手、最大几何接触重叠。两种成功工作件的回放位于 `episode_000/replay.mp4` 和 `episode_005/replay.mp4`。

六项既有测试通过。在同一 macOS 环境，将旧托座用例分别用旧提交和当前代码复跑，轨迹文件逐字节相同。两次当地运行都在落盘时超过旧用例的重叠阈值。仓库中旧用例的 Linux 参考运行曾通过。


### 2026-11-24 · Go2 换视角与 G1 换站位

- Go2 遇到漏检，沿桌边已知通道转到另一侧继续拍摄。在 [lab_trial.py 的换角度逻辑](../sim_workspace/lab_trial.py#L185) 写入每次观测点和接受视角。重试视角使用 [连通域、桌面范围和形状拟合](../sim_workspace/lab_trial.py#L19) 排除边缘碎片。三组原漏检均在东北视角定位，中心误差分别为 0.103、0.201 和 1.014 mm。
- G1 在 [换站位循环](../sim_workspace/lab_trial.py#L248) 基于取物和托盘坐标尝试桌边站位与转运腕角。每次移动使用步态策略，双脚站稳后重新评估实际站位。[TabletopReach](../sim_workspace/robots/g1/room_grasp.py#L49) 用独立的 MuJoCo 状态做关节限位下的 IK 检查。[抓取与执行共用的位姿](../sim_workspace/robots/g1/room_grasp.py#L13) 在物体近身且偏向身体正前时改变进手角度，减少与躯干的干涉。两组旧转运失败和一组原漏检的蓝块均落入黄托盘。
- 同一 MuJoCo 3.17.0 环境复跑九轮，逐轮核对了种子、物体、托盘和初始坐标。端到端通过从 3/9 到 6/9，RGB-D 交接从 6/9 到 9/9，黄托盘从 0/3 到 3/3。成功轮最大物体接触重叠为 0.635 mm，验收线仍是 0.75 mm。
- 余下三轮分别是种子 0 紫圆柱释放时碰到黄盘边，重叠 1.320 mm。种子 1 红球第二次换站位后距目标 49 mm，未达 40 mm 的到位范围。种子 2 红球抬起 13.8 mm 后失去夹持。逐轮日志、回放、RGB-D 和 `summary.json` 在 `sim_workspace/runs/flat-tabletop-adaptive-evaluation/`。

## 2026-11-25 · 编辑器预览、颜色与排障日志

- 普通编辑使用[轻量预览](../sim_workspace/scene_preview.py#L10)。在 robosuite 双料箱布局上，编辑视图网格对象从 107 降到 18，同场景服务器端单次位姿更新从 3.16 ms 降到 0.065 ms。浏览器帧率和 SSH 端到端延迟待测。
- 新增和导入物体支持在场景 JSON 保存 rgb，列表显示材质色号。Go2 仿真相机在受控摆放下能区分红、绿、蓝，红色通过现有检测阈值。自定义颜色的整段搜索与抓放流程待测。
- 8797 是当前编辑器入口，服务输出写入 sim_workspace/runs/viewer/scene-editor-8797.log。浏览器保存非法名称得到编号 2c2b05783ea6，服务器记录 save_scene 及 scene_editor.py 第 420 行的 traceback。添加形状记录 action_ok。服务器环境的 6 项现有单元测试通过。
- 服务器普通桌面单轮记录 server-sync-check-1795582263（种子 0，红球到绿托盘）为 placed，包含交接、RGB-D 和物理轨迹。历史的九轮结果为 6/9。服务器目录没有 Git 元数据，无法从该目录核对同步来源。
