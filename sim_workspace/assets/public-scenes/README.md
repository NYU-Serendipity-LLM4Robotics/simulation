# 公开场景资产库

在 VS Code Remote SSH 的 Ports 面板转发 8794，打开 [公开场景编辑器](http://127.0.0.1:8794/)。在右栏「保存与导入」下选择带「公开 /」的服务器场景，再点「打开服务器场景」。可拖动固定家具，物理播放时它会保持当前位置。修改后可另存为自己的场景，或下载可移植 JSON。

## 可直接打开的九份布局

| 文件 | 内容和上游 |
|---|---|
| robosuite-table / robosuite-bins / robosuite-pegs | 用 robosuite Arena 的碰撞盒搭建桌面、双料箱和桌面插销，换成现有 Go2 / G1。双料箱加了物理支撑，附小抓取块。 |
| libero-spatial-bowl-plate | 从 LIBERO Spatial 一项 BDDL 读取桌面区域中心，用简化几何表示碗、盘和盒子。 |
| maniskill-stack-cube | 采用上游 StackCube 的 4 cm 方块尺寸，摆放位置为本项目示例。 |
| playground-stairs | 从 MuJoCo Playground 的 Go1 楼梯场景提取前八级台阶，补齐地面以下的支撑体。 |
| procthor-kitchen / procthor-living-room | 从 ProcTHOR 测试划分各提取一个房间的墙线，门洞位置为简化布局，家具和纹理由原生数据定义。 |
| khronos-props-table | Khronos 的椅子、水瓶和玩具车加到工作台。当前导入器用统一材质及凸包碰撞。 |

所有布局都在服务器编译并运行 0.5 秒 MuJoCo 物理步进，核对了固定物体没有明显偏移。用途是摆场景和物理预览；原始 benchmark 的策略、奖励、传感器与得分协议需在原框架中运行。

## 原生资料与来源

- native/robosuite 保存五种 Arena XML 和对应配置源码。许可证见同目录 LICENSE。来源 https://github.com/ARISE-Initiative/robosuite
- native/libero 保存此版本的 BDDL 和原始初始状态文件，另附 benchmark 任务表。许可证 MIT。演示轨迹和原生渲染资产需由 LIBERO 官方流程安装。来源 https://github.com/Lifelong-Robot-Learning/LIBERO
- native/maniskill 保存 G1 放苹果、搬箱、PickCube、StackCube 和桌面构建源码，另有苹果、碗及桌面上游网格。源码 Apache-2.0，网格按上游资产说明 CC BY-NC 4.0 使用。来源 https://github.com/mani-skill/ManiSkill
- native/playground 保存 RSS 2025 目录下的 Go1/G1 平地和粗糙地形定义、高度图、Go1 楼梯配置。原生 XML 的机器人与传感器引用由上游环境解析。来源 https://github.com/google-deepmind/mujoco_playground
- native/procthor 保存 train、val、test 三个 gzip 户型文件并按上游 Git LFS SHA-256 验证。户型文件引用 AI2-THOR 资产。来源及许可证 https://github.com/allenai/procthor-10k
- meshes/khronos 保存原始 SheenChair、WaterBottle、ToyCar GLB 和各自 LICENSE.md、metadata.json。模型许可 CC0。来源 https://github.com/KhronosGroup/glTF-Sample-Assets

manifest.json 记录下载的 435 个文件、上游固定提交、大小和 SHA-256。可从 sim_workspace 重建目录。

    python assets/public-scenes/fetch_upstream.py
    OPENBLAS_NUM_THREADS=1 python assets/public-scenes/build_layouts.py

手动重启带场景库的编辑器。

    conda activate simulator
    OPENBLAS_NUM_THREADS=1 MUJOCO_GL=egl python workspace.py scene-editor --port 8794 --scene assets/public-scenes/layouts/robosuite-bins.json
