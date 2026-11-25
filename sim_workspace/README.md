# Serendipity LLM for Robotics simulation workspace

This workspace runs Unitree Go2 and G1 with Dex3-1 in one MuJoCo lab. The physical team entry performs Go2 search, RGB-D localization and G1 contact pick-and-place. The fixed G1 fixture and the historical kinematic suite support component regression.

## Run on the NYUAD server

```bash
cd /home/bj2410/projects/humanoid_manipulation/sim_workspace
conda activate simulator
export OPENBLAS_NUM_THREADS=1
export MUJOCO_GL=egl

python workspace.py go2 --seed 0 --output "runs/episodes/go2-$(date +%s)"
python workspace.py g1-fixed --output "runs/episodes/g1-fixed-$(date +%s)"
python workspace.py team --seed 0 --object 'red apple' --destination 'green tray' \
  --output "runs/episodes/team-$(date +%s)"

python bench.py suite --seeds 0 1 2 --output "runs/suites/recheck-$(date +%s)"
python workspace.py replay runs/episodes/team-physics-reference --port 8772
```

`run` is an alias for `team`. The fixed fixture takes optional `--source-xyz X Y Z` and `--destination-xyz X Y Z` in meters. Use the defaults for the calibrated acceptance case. Forward port 8772 in VS Code for the reference team replay. The viewer binds to server loopback. Older team recordings remain replayable. See the [runbook](../docs/TROUBLESHOOTING.md) for source-line references.

Each new run needs an unused output directory and writes a stage-by-stage `run.log`, a `report.json` and a trajectory in `qpos.npy`. Go2 and team additionally write RGB-D and a [validated handoff](../docs/HANDOFF.md). Fixed G1 writes `physics_trace.jsonl` with object position, finger normal forces, support contacts and actuator targets sampled every 20 ms. The control vector follows the report's named actuator order. External benchmark launches print their own log path under `runs/benchmarks/`.

The checked server setup uses Python 3.10.22, MuJoCo 3.17.0, NumPy 2.2.6, viser 1.1.1 and mjviser 0.0.14. Go2 meshes come from the existing Unitree MuJoCo checkout, whose sparse checkout includes `unitree_robots/go2`. G1 and Dex3-1 meshes come from the existing Unitree ROS checkout.

## Physical team run

The default task picks the red sphere from a tabletop cradle and places it in the green tray. Go2 walks under bounded joint torques, using feedback from its body-frame velocity, attitude and foot contacts. The local gait is generated online toward a mapped observation area. The RGB-D estimator selects the red pixels, rejects depth outliers and fits the known sphere radius. The handoff contains a world-frame sphere center and a mapped tray location. G1 starts at the worktable and stands on its two soles while the right arm recomputes world-frame IK and the Dex3 thumb and index generate the grip. The hand opens over the tray and lifts clear before retracting.

The seed 0 reference is in `runs/episodes/team-physics-reference`. It records 0.491 m Go2 displacement before the handoff at 3.641 s, 1.73 mm RGB-D center error against simulator truth, 73.9 mm peak lift, 4.302 s of unsupported opposing finger contact above the lift threshold, and 2.703 s settled in the green tray. G1's peak pelvis tilt was 1.31 degrees and both soles contacted the floor at all 20 ms trace samples. The maximum object contact penetration was 0.554 mm during release, within the 0.75 mm task limit. Seed 1 also passed. The yellow tray trial failed IK in transfer and the blue block reports an unsupported grasp. The RGB detector and the generic lab dimensions are task-specific. The G1 base stays near the worktable.

The episode directory contains `handoff.json`, `rgbd/000.npz` for the accepted reference frame, `physics_trace.jsonl`, `qpos.npy`, `report.json` and `run.log`. In the replay, pause **Play** to scrub to a phase and use **G1 close-up** for the hand and tray. Forward port 8772 from the remote host.

The `team-kinematic` entry and `bench.py suite` reproduce the previous kinematic perception/IK baseline. Go2's physical-room controller is in `robots/go2/trot.py`. G1's balance and contact grasp are in `robots/g1/room_grasp.py`. The geometry, reach layout and contact configuration are in `task_room.py`.

## Live shared lab

Run from this directory with the simulator environment active.

```bash
OPENBLAS_NUM_THREADS=1 python workspace.py room-view --port 8767
OPENBLAS_NUM_THREADS=1 python workspace.py room-physics --disturbance --output runs/episodes/room-check
OPENBLAS_NUM_THREADS=1 python workspace.py room-physics --unpowered --disturbance --output runs/episodes/room-off-check
```

Forward port 8767 in VS Code. Use **Push blue block** to apply an 8 N horizontal force for 160 ms, **Motors enabled** to toggle joint torque, and **Reset lab** to restore the starting scene. Enable the motors before resetting to restart a standing trial. The lab is shared by viewers connected to this process.

In the room, Go2 and G1 have free bases. The three four-legged tables and two rimmed trays have mass and move independently through contacts. The mixed robot setup uses the official Unitree assets, a joint-space stand controller, and G1 ankle attitude feedback. G1's 29 main joint torque bounds come from Unitree's MuJoCo model. The Dex3 hand bounds and furniture friction are simulation assumptions. The apple is represented by a sphere. The room is a generic lab layout with nominal dimensions and materials.

Each headless run saves solver checks, first-fall times and object displacement in `report.json`, time-series positions and contacts in `physics_trace.jsonl`, and poses for `workspace.py replay`. The standing gate covers ten simulated seconds. The unpowered gate expects both robots to lose support. The disturbance gate expects the block to reach floor height. The verified references are `runs/episodes/room-physics-seed-0` through `room-physics-seed-2` and `room-physics-unpowered`.

The live entry covers standing and physics experiments. The `team` entry uses the task layout in the same physics room. The `g1-fixed` entry covers isolated contact grasp tests. The physical task validates Go2 locomotion and G1 standing manipulation for the red sphere and green tray. G1 walking and other objects and destinations require further validation. Matching a specific lab and hardware requires measured geometry, friction and robot response data.

## Fixed G1 acceptance

The 46 mm, 28 g sphere uses a free joint. The `dex3-contact-v3` scene explicitly sets world gravity to (0, 0, −9.81) m/s² and disables body-level gravity compensation. Two static posts join the fixture surfaces to the ground. A fixed base and torso hold the Unitree model while 14 position servos drive the right arm and Dex3-1 finger joints. Six-dimensional IK generates actuator targets, and MuJoCo steps the contacts and object dynamics. The stock finger collision meshes make the grasp. The fixture enables right-finger self-contact and robot contact with the receiver. Broader robot self-collisions stay outside this fixed-base fixture. The receiving plate uses rolling friction. The team physics run uses separate free-base acceptance criteria.

Success requires opposing finger contact during lift, at least 6 cm peak lift and 4 cm minimum clearance during transfer and wrist rotation, then at least 0.25 s of support contact without finger force and an XY error at most 3.5 cm. The maximum admitted signed-distance penetration is 0.50 mm, monitored at every simulation step. See `runs/episodes/g1-gravity-v3` for the verified run and `runs/episodes/g1-fixed-unreachable` for an IK failure.


### Contact model and penetration gate

`robots/g1/pick_place.py:19–31` declares the contact profile. It uses a 0.50 ms physics step, 2.50 ms contact time constant, critical damping, and `solimp = (0.97, 0.995, 0.0004, 0.5, 2.0)`. Sliding friction is 0.95 for the finger geoms, 0.70 for the receiver plate, and 0.55 for the other collidable surfaces. The receiver plate uses rolling friction 0.003, and finger actuator torque is limited to 0.85 N·m. These are simulation parameters tuned for the fixture. Real material coefficients remain unmeasured.

`report.json` records `contact_profile` plus peak signed-distance penetration by contact category, geometry, stage and time. `physics_trace.jsonl` gives per-sample diagnostics. At the 0.50 mm acceptance limit, the original contact profile is rejected for 2.79 mm ball-to-hand and 7.47 mm ball-to-receiver overlap. In the v3 reference run, peaks were 0.095 mm for ball-to-hand, 0.472 mm for ball-to-receiver, and 0.107 mm for hand-to-receiver. The ball lifted 94.1 mm and remained on the receiver for 2.25 seconds. The browser replays saved frames.

Run `OPENBLAS_NUM_THREADS=1 python -m unittest discover -s tests -v` from this directory to check ball free fall and unpowered wrist motion against zero-gravity controls.

## Files and integration points

- `robots/go2/scout.py` owns the Go2 model path, foot chains, camera configuration, recording and localization.
- `robots/g1/manipulation.py` owns the G1 model path, stance, foot and wrist chains, and pregrasp. `robots/g1/pick_place.py` owns the fixed-base physics fixture. `robots/g1/inspect_robot.py` and `robots/g1/view_g1.py` are the standalone model and Dex3-1 tools.
- `workspace.py` builds the shared scene and coordinates the handoff. `kinematics.py` supplies the common solver and foot target generator.

| Owner | Starting point | Next interface to implement |
| --- | --- | --- |
| Yanxi Chen | Pass a resolved target and destination to `workspace.py team`. Consume stage events in `report.json`. | Translate user requests into validated task fields, handle clarification and infeasibility, and advance or retry stages with timeouts. |
| Zihang Shu | Train or evaluate grounding on the saved Go2 RGB-D frames. `robots.go2.scout.localize` accepts a mask and calibrated depth. | Produce masks from model output, use the same world-frame transform, record point error, inference latency, and reduced-frame-rate or quantized variants. |
| Baosheng Jin | Consume the `manipulation_ready` payload. `workspace.scene` returns the full model, data, and indices of both free bases. The G1 joints and actuators use the `g1/` prefix. | Use the fixed fixture to develop contact control and tool calibration. Integrate it with the shared scene after validating frame and object assumptions. |
| Berton Yeh | Replay the handoff and cross-check the model and calibration with the hardware interface. | Validate joint mapping, limits and extrinsics, and run controlled hardware trials from fixed pose through perception-derived handoff. |

Each episode stores `report.json`, `qpos.npy`, and 14 `frame_*.npz` files containing RGB, metric optical-axis depth, MuJoCo geom IDs, oracle target mask, camera intrinsics, and the camera-to-world rotation and position. The camera axes are right, up, backward. World coordinates are in meters. The reported error compares a visible surface point to the object's simulated center, so its average includes geometric offset. The current dataset covers sampled tabletop positions and two named destinations. The Go2 preview solves XYZ for four spherical foot endpoints. The G1 preview solves XYZ and orientation for both feet and for the nominal palm point before grasping. The solver clamps modeled hinge limits and rejects arm samples with modeled penetration deeper than 4 mm. The recorded trajectories are kinematic. Base routes are prescribed and dynamic balance remains pending.

## External benchmarks

```bash
python bench.py list
python bench.py run maniskill-g1-apple --dry-run --args --steps 5
python bench.py run maniskill-g1-apple --args --steps 5
python bench.py run libero-spatial --python /path/to/libero-env/bin/python
```

The two ManiSkill entries exercise the upstream G1 apple placement and box transport environments with random actions for integration smoke tests. The LIBERO entry lists language tasks from the upstream Spatial suite. Policy evaluation can be added by registering its native evaluator command in `benchmarks.json` or by supplying another file with `--registry` before the subcommand. Each entry takes a description, an argument list, and optional Python interpreter and working directory. Use `{python}` and `{workspace}` for the environment interpreter and this directory. Record official suite scores under the upstream embodiment and protocol. The local Go2–G1 sensor suite has a separate protocol.

- Unitree models and setup: https://github.com/unitreerobotics/unitree_mujoco
- ManiSkill official humanoid tasks: https://maniskill.readthedocs.io/en/latest/tasks/humanoid/
- LIBERO evaluation and datasets: https://github.com/Lifelong-Robot-Learning/LIBERO

## Verified on the server

- Shared MuJoCo smoke suite: 6/6 RGB-D handoffs over seeds 0, 1, and 2. Mean surface-to-object-center distance was 3.73 cm and maximum was 4.24 cm. The reports and 84 RGB-D frames are under `runs/archive/suite-smoke/`.
- ManiSkill 3.0.1: both G1 environments reset and stepped five times on CPU using the isolated environment under `.envs/maniskill`. Both returned `success: false` with the random action policy. Logs are in `runs/benchmarks/maniskill-apple.log` and `runs/benchmarks/maniskill-transport.log`.
- LIBERO Spatial: launcher dry run verified. Catalog access requires a separate LIBERO installation.
- Model checkout revisions: Unitree MuJoCo `1eb6642` and Unitree ROS `41dec58`.

## IK verification

The current regular entry point writes schema 2. It stores foot and pregrasp IK events, target and achieved palm poses, pose residuals, and arm joint angles. Pregrasp success requires the position and orientation tolerances to pass. CLI and suite exit nonzero when pregrasp is incomplete. The tool frame uses a nominal palm point 85 mm along wrist-local X. Hardware calibration remains pending.

On the six episodes in `runs/suites/ik-pose`, all six reached the 6D pregrasp. The maximum foot XYZ residual was 1.98 mm. The maximum pregrasp residual was 0.88 mm and 0.23 degrees. These residuals compare forward kinematics to generated goals. They are separate from the RGB-D localization error. Replayed solved joint ranges and FK were verified. An unreachable target rolled back joint state, and a deliberate arm-table penetration was rejected.

## Flat tabletop lab trials

Use `lab-trial` to run the ordinary tabletop with three movable workpieces and two trays. If the first RGB-D survey misses the named object, Go2 follows the mapped aisles and checks new viewing angles. At the new angles, the detector checks connected color regions, tabletop bounds and the expected shape before accepting a handoff.

G1 walks using Unitree's 29-DoF velocity policy. Before closing its hand, it uses pose IK to check the pickup, lift and release poses together. When the current stance fails, it searches candidate stances and transfer wrist angles, walks to a feasible candidate, settles with both soles grounded, and checks the measured pose again. The execution still checks physical fingertip contact, lift, tray support and penetration. The lab layout supplies the tray pose and MuJoCo supplies the camera pose. The workpiece geometries and contact coefficients are simulation estimates.

From `sim_workspace`, install the optional CPU ONNX runtime and download the pinned policy from [Unitree's velocity deployment](https://github.com/unitreerobotics/unitree_rl_lab/tree/4960b84732b0c2ec593dccbfe963fda1bcd7b1e3/deploy/robots/g1_29dof/config/policy/velocity/v0). The loader checks the downloaded file's SHA-256.

```bash
python -m pip install onnxruntime==1.23.2
mkdir -p .envs
curl -fL https://raw.githubusercontent.com/unitreerobotics/unitree_rl_lab/4960b84732b0c2ec593dccbfe963fda1bcd7b1e3/deploy/robots/g1_29dof/config/policy/velocity/v0/exported/policy.onnx \
  -o .envs/g1-velocity.onnx

python bench.py lab-suite --policy .envs/g1-velocity.onnx --seeds 0 1 2 --output runs/lab-suite
python workspace.py lab-trial --policy .envs/g1-velocity.onnx --seed 1 \
  --object 'purple cylinder' --destination 'green tray' --output runs/lab-single
python workspace.py replay runs/lab-suite/episode_000 --port 8772
```

On a headless Linux server, use `MUJOCO_GL=egl`. Each episode has a report, stage log, physics trace, replay trajectory and RGB-D frame when localization succeeds. The suite tests all three objects per seed, with assigned tray names and a shuffled workpiece layout. `summary.json` reports the end-to-end success rate and the exact failure stage of each trial. The suite exits zero when evaluation completes and writes all results.

## 浏览器场景编辑器

选中新增形状或导入网格，用「物体颜色 RGB」输入色值并点击「应用颜色」。物体列表显示色值，面板显示完整物体 ID。场景 JSON 的 objects 条目用 rgb 保存材质基色，范围为 0–255。旧场景缺少该字段时保留蓝灰色。相机观测受到光照与遮挡影响。

摆放场景时使用轻量网格预览。拖动只更新选中实体，松手后可在 XYZ 面板确认坐标。点击「播放物理」切换到完整 MuJoCo 仿真，「返回布置」回到播放前的编辑布局。

在 VS Code 的 Remote SSH 终端，从 sim_workspace 启动。服务器的 simulator 环境已包含 Trimesh 5.2.0。重建环境时可运行 python -m pip install trimesh==5.2.0。

    cd /home/bj2410/projects/humanoid_manipulation/sim_workspace
    conda activate simulator
    mkdir -p runs/viewer
    set -o pipefail
    OPENBLAS_NUM_THREADS=1 MUJOCO_GL=egl python -u workspace.py scene-editor --port 8797 \
      2>&1 | tee -a runs/viewer/scene-editor-8797.log

在 VS Code 的 Ports 面板转发 8797，打开 [场景编辑器](http://127.0.0.1:8797/)。服务器已有 8797 服务时直接打开页面，编辑器进程共享当前布局。在右栏选择物体或点蓝色标记，用鼠标拖动坐标轴。朝向和 XYZ 可在右栏精确输入。可以从普通桌面、空房间或障碍练习场开始。

- 添加资产支持方块、球、圆柱、平台和 OBJ / STL / GLB 网格。先设置最长边，导入网格会缩放到这个尺寸。OBJ / STL 按 Z 向上导入，GLB 从 Y 向上转换为 Z 向上。网格使用统一材质和凸包碰撞。
- 点击「播放物理」运行 MuJoCo 重力和接触，机器人使用站立控制。点击「返回布置」恢复布置时的位置。编辑器物理预览与 lab-trial 自动抓放使用各自的入口。
- 「保存到服务器」将场景写到 runs/scenes。可以从「服务器场景」打开，也可以下载自包含的 JSON，再用「导入场景 JSON」打开。启动时还可传入 --scene runs/scenes/场景名.json。
- 可导入的示例是 [障碍练习场资产](assets/obstacle-course.json)，也可在右栏直接下载。网格上传及场景文件最多 12 MB，每场景最多添加 30 个物体。

页面报错时复制「日志编号」，按[编辑器排错步骤](../docs/TROUBLESHOOTING.md#浏览器场景编辑器)在服务日志中查操作和 traceback。

公开 benchmark 布局和原生资料见 [公开场景资产库](assets/public-scenes/README.md)。
