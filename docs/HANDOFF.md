# Go2 → G1 交接协议，版本 1

`workspace.py go2` 和 `workspace.py team` 都在运行目录写 `handoff.json`。双机流程通过 `handoff.validate` 验证消息，再把坐标交给 G1。

| 字段 | 含义 |
| --- | --- |
| `schema_version` | 整数 `1` |
| `scene_id`、`scene_seed` | `serendipity-table-v1` 和非负种子，用于复现场景 |
| `source_robot` | `go2` |
| `frame_id`、`units` | `world` 和 `m`。右手系，+Z 向上，+X 是 Go2 出发的朝向，+Y 指向出发时 Go2 的左侧 |
| `object_id`、`destination_id` | 场景目标和目的地标识，例如 `red apple`、`green tray` |
| `object_xyz_m` | 三个有限的 `[x,y,z]` 分量。成功时为 RGB-D 回投影的可见表面点，失败时为 `null` |
| `destination_xyz_m` | 托盘指定落点的世界坐标，Z 是托盘上表面高度 |
| `observation` | 成功时含 `position_kind: surface_point`、`capture_index` 和 `method`。帧编号对应同目录的 `frame_NNN.npz`。失败时为 `null` |
| `status` | `localized`、`target_not_visible` 或 `go2_foot_ik_failed` |

接收端在 `status == 'localized'` 时读取对象坐标。本协议版本传递表面点，物体中心、旋转和置信度仍待估计。G1 根据表面点及固定偏置生成预抓取位姿。双机的 G1 步态和预抓取失败写在 `report.json` 的 `status`，分别为 `g1_foot_ik_failed` 和 `g1_pregrasp_ik_failed`。

`g1-fixed` 使用独立的固定底座台架，输入是该台架世界坐标里的已知初始球心和目标球心。`report.json` 给出独立的物理抓放状态。退出码 `0` 表示达到所选模式的成功条件，`1` 表示任务失败，`2` 表示 CLI 参数错误。运行异常会留下 traceback。

```bash
cd /home/bj2410/projects/humanoid_manipulation/sim_workspace
conda activate simulator
python -m json.tool runs/episodes/go2-contract/handoff.json
python -m unittest discover -s tests
```

## 物理房间交接

`scene_id = serendipity-room-team-v1` 时，`observation.position_kind` 是 `object_center`，坐标仍是 MuJoCo 世界系中的米。接受的球心来自 Go2 保存的 RGB-D 和相机标定，仿真真值只用于报告误差。`capture_index` 对应物理任务输出目录里的三位编号 `rgbd/*.npz`。`go2_lost_balance` 和 `unsupported_grasp_object` 的目标坐标与 observation 均为 null。黄色托盘可接收有效交接，但后续仍可能因 G1 IK 不可达失败。

## 普通桌面交接

`scene_id = serendipity-flat-table-v1` 同样使用 `object_center`，Go2 像素和深度结合桌高及目标尺寸先验估计物体中心。目的地是实验室预登记的托盘世界坐标。G1 的行走、站稳、抓放结果在 `report.json` 的 `status` 和 `failure_stage`，帧编号对应 `rgbd/*.npz`。
