# 排错手册

以下行号对应当前 NYUAD 工作区。后续增删代码时，用表里的函数名重新定位，并在提交工程笔记时更新对应行号。在 VS Code 远程终端可以直接打开指定行：

```bash
cd /home/bj2410/projects/humanoid_manipulation/sim_workspace
conda activate simulator
code --goto "$PWD/robots/go2/scout.py:50"
```

## 先找到出错的一轮

- 单轮打开输出目录里的 `run.log`。`WARNING` 是有结果的阶段失败，查看同目录的 `report.json` 和 `ik_status`。`ERROR` 后面保存 Python traceback，最后一条阶段日志说明当时在编译模型、渲染某帧或写结果。
- 批量先看 `summary.json`，再进对应的 `episode_NNN/run.log`。有业务失败时批量继续跑完并返回非零。有未处理异常时进程会退出，已写的 episode 日志仍可查。
- 外部 benchmark 从终端的 `Benchmark log` 找到 `runs/benchmarks/` 中的文件。里面有实际解释器、命令参数、工作目录、合并输出和退出码。`--dry-run` 在终端显示命令。
- 终端已报错但找不到新日志时，先看 Python 导入、CLI 参数和输出目录是否已存在。日志创建在成功解析参数并创建输出目录之后。

```bash
run_dir=runs/suites/maintenance-check/episode_000
tail -n 30 "$run_dir/run.log"
grep -nE 'WARNING|ERROR|Traceback|finish status' "$run_dir/run.log"
python -m json.tool "$run_dir/report.json"
```

## 症状、源码和验证

| 症状 | 看哪里 | 检查或修改 | 修完怎么确认 |
| --- | --- | --- | --- |
| 导入 `mujoco` 失败、参数无效、`FileExistsError` | 终端，`workspace.py:196` 的 `main` 和 `110–135` 的 `logged_episode` | 检查 `which python` 和 `python -c 'import mujoco; print(mujoco.__version__)'`。使用 `simulator` 环境和新的输出目录。CLI 参数在 `workspace.py:196–224`。 | 复跑同 seed，确认日志首行有解释器、MuJoCo 版本和输出目录。 |
| 编译时报模型 XML、mesh、free joint 名称错误 | `run.log` 的 `compiling shared scene`。`robots/go2/scout.py:9` 及 `robots/g1/manipulation.py:9` 的模型路径，`workspace.py:54–75` 的 `scene` 模型合并 | 先确认 `third_party/unitree_mujoco/unitree_robots/go2/go2.xml` 和 G1 XML 及 meshes 在服务器。Go2 sparse checkout 缺文件时，在原 Unitree MuJoCo checkout 恢复 `unitree_robots/g1` 和 `unitree_robots/go2`。有意更换模型时，再修改 相应机器人模块的 `MODEL_XML`、命名映射和关节链。 | 正常跑过 scene，日志进入 `go2_foot_ik`。 |
| EGL 初始化失败，或者没有 RGB-D 文件 | `run.log` 的 `initializing RGB-D renderer` 或 `capturing RGB-D frame=...`。`robots/go2/scout.py:72–88` 渲染及存盘，`32–41` 的 `capture` | 在启动前设置 `MUJOCO_GL=egl`，确认解释器和 GPU 可见。只在可复现为相机或渲染配置问题时改采集代码。 | 同 seed 产生 `frame_000.npz` 到 `frame_013.npz`，并出现 `report` 阶段。 |
| `go2_foot_ik_failed` 或 `g1_foot_ik_failed` | 找 `*_foot_ik`，对照 `samples` 和 `failed_error_m`。`robots/go2/scout.py:26–29` 是 Go2 关节链、`66–71` 是路线。`robots/g1/manipulation.py:13–17` 是 G1 关节链、`50–63` 是站位和路线。`kinematics.py:92–123` 是共用足端轨迹 | 先查足端和关节名称。再看第一处失败的步幅、抬脚高度、站高、足姿约束和初始站姿（`robots/g1/manipulation.py:41–47`）。优先调整生成的目标。`kinematics.py:37–89` 是通用求解与回退。 | 同 seed 的相应事件 `converged: true`，足端残差 ≤ 2 mm，然后跑六轮批量。 |
| `target_not_visible`，或坐标明显漂移 | `robots/go2/scout.py:14–19` 相机外参，`44–59` 的 `intrinsics` 和 `localize`，`83–98` 的 mask、深度和报告 | 检查各帧 `target_mask` 是否有有效像素、深度单位是否为米、`camera_rotation_world` 是否按 right/up/backward 坐标约定使用。对齐最新 `K` 和同一帧的外参再改相机或反投影。当前 mask 来自仿真 geom ID。 | 同 seed 重出 `report.xyz_m`。用归档帧重新计算并查看 `center_error_m`，它包含表面点与物体中心的偏差。 |
| `g1_pregrasp_ik_failed` | `g1_pregrasp_ik` 的 `arm`、`error_m`、`orientation_error_rad`。`robots/g1/manipulation.py:50–94` 的 `approach_with_ik`，目标偏置在 `67`，姿态在 `75–84` | 根据回放先查选臂、站位和目标偏置，再核对 `robots/g1/manipulation.py:20–24` 的腕部关节链及名义掌心偏置。碰撞检查在 `robots/g1/manipulation.py:27–38`。如果目标点或路径穿过障碍物，修正站位或预抓取轨迹。 | 同 seed 到达 `pregrasp_reached`，残差 ≤ 2 mm、角度残差 ≤ 0.025 rad，随后批跑。同一姿态的错误场景应继续被拒绝。 |
| 报告有定位但 G1 未抓起物体 | `report.json` 的 `ik_status` 和 `placement_status`。`workspace.py:151–165` 是交接和写报告 | `team` 的 `placement_status: not_run` 表示共享场景止于预抓取。用 `g1-fixed` 验证物理抓放，再把台架控制接入共享流程。原手部独立实验见 `sim_workspace/robots/g1/view_g1.py:240`。 | 固定台架可复测下表的抓取和放置阶段。 |
| 回放打不开、端口被占用、读不到姿态 | `runs/viewer/` 的控制台日志，`workspace.py:170–193` 的 `replay` | 确认同一输出目录里有 `report.json` 和 `qpos.npy`。远端先请求 `http://127.0.0.1:8766/`，再检查 VS Code Ports 转发。若占用，可用 `--port 8767` 并转发该端口。 | 远端 HTTP 200，浏览器显示 Connected 并播放两台机器人。 |
| Benchmark 找不到解释器、模块或环境 ID | 对照 `runs/benchmarks/` 日志里的 `argv`、`cwd` 和 traceback。`bench.py:56–69` 解析配置，`74–92` 启动子进程 | 默认 ManiSkill 解释器在 `benchmarks.json:10` 和 `20`。用 `--python /绝对路径/bin/python` 指向独立环境。任务 ID 在 `8` 和 `18`。环境创建与动作在 `upstream.py:6–21`。LIBERO 任务目录在 `upstream.py:25–29`。 | 先 dry run，再运行五步随机动作。看 `exit_code=0` 和步数。随机探索返回 `success: false` 表示任务未完成，进程可正常退出。 |
| `handoff.json` 验证失败 | 查看 `handoff.py:16` 的 `validate` 和 `workspace.py:151–162` 的交接 | 核对协议版本、世界坐标系、米、场景 seed 以及失败时的 null 字段。 | `python -m unittest discover -s tests` 六项通过，同 seed 的 Go2 单跑和双机交接 JSON 一致。 |
| `g1-fixed` 的 `ik_failed` 或 `no_opposing_grasp` | 看 `run.log` 的最后一个阶段和 `physics_trace.jsonl`。台架构建在 `robots/g1/pick_place.py:34`，工具偏置与手指闭合值在 `139–142`，接触汇总在 `145–162` | 先用默认球心和目标点复跑。IK 失败时查工具坐标和目标是否可达。夹持失败时看闭合后是否有拇指与食指或中指同时施力，核对源支撑位置和已标定的手指角。 | 默认用例出现 `close` 和 `lift` 阶段，离台时有拇指和对侧手指接触。 |
| `lift_failed`、`dropped_in_transfer` 或 `placement_failed` | 查 `physics_trace.jsonl` 的球高、接触力和支撑面。`robots/g1/pick_place.py:218–247` 是抬升与释放路点，`254–274` 是腕部转角和执行器步进，`292–323` 验证离台及落稳 | 横移时查对向夹持力和相对高度。球跟手后撤时，检查转腕释放角、后撤方向和收球盘滚动摩擦。对照 `runs/episodes/g1-gravity-v3`。 | 报告 `status: placed`，最终阶段无手指力、球在接收台上且稳定至少 0.25 s。 |
| `excess_penetration` 或看见穿模 | 查 `report.json` 的 `evidence.penetration_peaks`，用 stage、time_s 和 geoms 找到对应接触。逐步测量在 `robots/g1/pick_place.py:165–186`，验收在 `314–322` | 先核对可视网格和物理网格、碰撞位掩码。检查 `19–31` 的接触参数、`111–132` 的配置，以及 `218–247` 里的释放与撤手路径。保留证据再改参数。 | 默认台架 `placed` 且峰值 ≤ 0.50 mm，最终阶段球在接收台上、没有手指力。 |
| 球不下落或断电后手臂不动 | 核对 `report.json` 的 `contact_model` 事件里 `gravity_m_s2` 为 `[0, 0, -9.81]` 且 `gravity_compensated_body_count` 为 0。浏览器标注 Recorded trajectory 时显示的是保存的轨迹。 | 在 `sim_workspace` 运行 `OPENBLAS_NUM_THREADS=1 python -m unittest discover -s tests -p test_g1_gravity.py -v`。重力与补偿在 `robots/g1/pick_place.py` 第 42、48 行，仿真步进在第 272 行。 | 有重力时球 0.20 s 下落约 197 mm，断电手腕 0.10 s 下坠约 28 mm。零重力对照两者位移均为零。 |

## 复现及复测

把下面的 seed、物体和目的地换成失败日志首行里的值。每次用新目录，保留失败日志用于对照。

```bash
run_dir="runs/episodes/repro-$(date +%s)"
OPENBLAS_NUM_THREADS=1 MUJOCO_GL=egl python workspace.py run \
  --seed 3 --object 'blue block' --destination 'green tray' --output "$run_dir"

tail -n 15 "$run_dir/run.log"
OPENBLAS_NUM_THREADS=1 MUJOCO_GL=egl python bench.py suite \
  --seeds 0 1 2 --output "runs/suites/recheck-$(date +%s)"
python bench.py run maniskill-g1-apple --dry-run --args --steps 5
```

现成对照在 `runs/episodes/maintenance-check/`。`normal` 是成功样例，`no-target` 注入了定位失败，`missing-asset` 注入了模型缺失。两项故障只在测试进程内注入。研发记录和修复依据见 [工程笔记](ENGINEERING_LOG.md)。

## 实时房间仿真

- 页面连接失败时，查看项目里的 `runs/viewer/room-live.log`。用 VS Code 转发远程端口 8767，并从 `sim_workspace` 启动 `OPENBLAS_NUM_THREADS=1 python workspace.py room-view --port 8767`。
- 机器人倒地时，先检查 Motors enabled，再点 Reset lab。无界面复测用 `workspace.py room-physics --disturbance --output <新目录>`。检查报告的 `robots.*.first_fall_s`、倾角和足部接触，G1 初始足底间隙和力矩反馈分别看 `room.py:55` 和 `room.py:91`。关掉电机后两台机器人应失去支撑。
- 桌面或托盘的接触异常时，检查 `room.py` 中的 `furniture`、`tray` 和材料参数。蓝方块跌落对照通过 `block_final_height_m` 和 `physics_trace.jsonl` 复核，地面静止时方块中心在约 0.045 m。

## 双机物理任务

- `target_not_visible` 时，检查本次 `rgbd/*.npz` 的 RGB、深度、mask 和相机位姿。`team_physics.py:20` 是拟合，`team_physics.py:59` 是搜索与交接。`task_room.py` 的观测位应有直视桌面的视线。
- `g1_lost_balance` 或 `lift_failed` 时，读 `physics_trace.jsonl` 中 `g1_grounded_soles` 和骨盆角，核对 `report.json` 的 `end_finger_force_n` 与 `end_supports`。`task_room.py:99` 是站姿和接地初始化，`robots/g1/room_grasp.py:45` 是腕部 IK，夹持和撤离从第 55 行开始。
- `excess_penetration` 时，按报告里的 `penetration_contact` 对照阶段、仿真时间和几何。托盘求解器设置在 `task_room.py:49`。`ik_failed` 检查事件里的目标和误差，当前黄色托盘存在转运不可达。

## 浏览器场景编辑器

当前入口是 VS Code Ports 转发的 8797。用 SSH 终端在 sim_workspace 中执行下面的命令检查服务、查看最近日志。启动命令见[工作区 README](../sim_workspace/README.md#浏览器场景编辑器)。

```bash
curl -fsS http://127.0.0.1:8797/ -o /dev/null
tail -n 60 runs/viewer/scene-editor-8797.log
grep -n -A 20 'ref=<页面上的日志编号>' runs/viewer/scene-editor-8797.log
```

| 症状 | 查日志和代码 | 修完怎么确认 |
| --- | --- | --- |
| 页面打不开 | 先在 SSH 终端执行上面的 curl。服务器请求失败时查服务启动日志和 [run](../sim_workspace/scene_editor.py#L479)。服务器请求成功时，在 VS Code Ports 确认本地地址转发到远程 8797。 | 日志出现 editor_ready，浏览器看到场景和右侧控件。 |
| 导入 OBJ、STL、GLB 失败 | 查 import_mesh 和 [网格导入](../sim_workspace/scene_editor.py#L126)。检查扩展名、12 MB 上限、三角面数量和网格是否有厚度。 | 导入后物体出现在列表中。 |
| 场景 JSON 导入失败 | 查 import_scene 和 [布局校验](../sim_workspace/scene_editor.py#L75)。核对 format、物体 ID、尺寸及 rgb 的三个 0–255 整数。 | 用编辑器导出的 JSON 复测，确认物体和坐标加载。 |
| 改色后画面或跟踪字段异常 | 查 apply_color、[资产属性更新](../sim_workspace/scene_editor.py#L407)、[MuJoCo 模型生成](../sim_workspace/scene_editor.py#L146)和[轻量预览](../sim_workspace/scene_preview.py#L10)。 | 列表中的色号、保存后的 rgb 和预览一致。实机或仿真相机像素随光照与遮挡变化。 |
| 保存失败 | 查 save_scene、[场景名校验](../sim_workspace/scene_editor.py#L417)和[保存逻辑](../sim_workspace/scene_editor.py#L428)。检查场景名、是否重名和 runs/scenes 写权限。 | 换一个合规名称保存，再从「服务器场景」打开。 |
| 拖动卡住，或播放后无法编辑 | 查 [拖动更新](../sim_workspace/scene_editor.py#L347)和[播放物理](../sim_workspace/scene_editor.py#L444)。播放时先点「返回布置」再编辑。 | 拖动一个物体，松手后核对 XYZ，再验证播放和返回布置。 |

成功操作带有 action_ok，异常带有 action_failed 和 ref。取日志编号在文件中搜索，从 traceback 最后一条项目代码定位实际抛错行。拖动中的高频位置更新只在出错时记录，松手会记录成功事件。进程退出前的循环异常直接写到同一日志的标准错误输出。
