# DualArm_handeye 开发交接

与用户沟通使用简体中文。代码、命令、API 字段和专有名词保留原写法。

## 项目范围与当前状态

项目为左右两台 X5 逐臂做手眼标定。宿主机 Orbbec 驱动发布 ROS 2 图像和内参，PyQt5 GUI 检测棋盘格并读取选中机械臂的位姿，使用 OpenCV Park 求解并导出结果。

- 现场基线已由用户确认。USB 相机和标定 GUI 可正常启动，左右臂可连接，关节和世界坐标点动已验证。设备枚举上报 Gemini 335L。
- 后续现场验证关注棋盘格检测和 PnP、TF0 位姿回读、实际采样，以及使用未参与求解的姿态和独立目标验收精度。
- 运行环境是 Ubuntu 22.04 amd64、ROS 2 Humble、Conda `handeye`（Python 3.10）及 `xapi` 3.4.9（`SDK/xapi-3.4.9-cp310-cp310-linux_x86_64.whl`）。GUI 在同一进程中使用 `rclpy`、`sensor_msgs`、OpenCV、PyQt5 和 `xapi.api`。
- [README.md](README.md) 记录日常操作，[配置文档](docs/配置文档.md) 维护安装、ROS 环境加载和启动命令。自动创建环境、安装依赖、修改 ROS 配置或写入 udev 规则须经用户在当前请求中明确授权。

## 代码入口

| 文件 | 职责 |
| --- | --- |
| `scripts/start_camera.sh` | 用 `SERIAL_NUMBER` 选定 USB 相机，调用厂商 Gemini 330 系列 launch，发布到 `/camera_01`。 |
| `scripts/x5_handeye_calibration.py` | `RosCameraNode` 发现和订阅话题，`MainWindow` 管理选臂、采样与结果展示，`compute_and_save_result` 由 GUI 和 `--recompute-samples` 共用。 |
| `X5/x5_robot.py` | `X5Robot` 串行化 X5 SDK 调用，封装控制器状态、TF、位姿和点动。`diagnose_x5_network` 只读检查系统选路。 |
| `config/x5.example.yaml` | 左右臂地址模板。GUI 自动读取存在的 `config/x5.local.yaml`，也支持用 `--x5-config` 指定文件。 |
| `scripts/diagnose_handeye_samples.py` | 独立离线诊断，针对眼在手外、9×6 内角点、24 mm 方格，查看坐标链和留一法敏感性。 |

## 相机与机械臂边界

- GUI 查找同一命名空间下的 `/camera_01/color/image_raw/compressed`（`sensor_msgs/msg/CompressedImage`）和 `/camera_01/color/camera_info`（`sensor_msgs/msg/CameraInfo`）。两个话题同时可见且类型正确时，才列出 `camera_01`。修改命名时同步检查启动脚本、GUI 发现逻辑和文档。
- 相机驱动由操作者在宿主机管理。GUI 只订阅话题，保留当前终端的 ROS 发现设置。启动驱动前确认 XiaoTop 或其他程序已释放设备。
- ROS 回调在 `RosCameraThread` 中执行，图像和内参经 `FrameBundle` 交给 GUI。修改订阅或采样逻辑时保留相机切换后的帧校验，避免旧回调和旧图像进入当前样本组。
- 默认使用静态检查、ROS 话题检查、只读路由诊断和模拟测试。连接控制器、准备手动模式、使能伺服、点动和现场采样均须经用户在当前请求中明确授权。GUI 连接过程会选择并回读 TF0，遇到报警时会尝试复位。
- GUI 一次只连接一台控制器。切换前关闭伺服、断开当前臂并确认连接释放。运动前核对机械臂和控制器身份，确认工作区无人、速度足够低、实体急停可用、其他客户端已断开。禁止用真实运动验证代码，禁止自动修改现场路由、IP、源地址、工具坐标或标定配置。

## 样本、坐标和结果

- 每会话的数据默认写入 `handeye_samples/<left|right>/<会话>/camera_01/`。原始观测是 `sample_*.json` 和对应 PNG，结果是 `handeye_latest.json` 和 `handeye_latest.npz`。同一组样本必须使用同一相机、机械臂、控制器和 TF0。
- 机器人位姿为 TF0 在 Base 下的 XYZABC，平移用毫米，角度用度。旋转按列向量约定使用 `Rz(C) @ Ry(B) @ Rx(A)`。PnP 提供棋盘格到相机的变换。
- `eye_in_hand`（默认）输出相机到 TF0，结果标识 `camera_to_tcp`。`eye_to_hand` 输出相机到 Base，标识 `camera_to_base`，送入求解器前对机器人位姿取逆。两种结果按 `p_destination = R @ p_source + t` 使用。
- `fixed_transform_validation` 统计每条样本反推出的棋盘格固定变换及其离散度。眼在手上以 Base 为目标坐标系，眼在手外以 TF0 为目标坐标系。现场精度验收另用独立观测。
- 离线重算读取保存的角点，沿用采集时的模式、内角点数和方格尺寸。默认是 9×6 内角点、24 mm 方格。旧样本须有 TF0 标记，缺少臂和控制器标识的一组旧样本保留未知归属。
- `handeye_samples/`、`config/x5.local.yaml`、`SDK/` 和本机 `test/` 已被 Git 忽略。原始数据、现场配置、SDK、运行日志和临时结果留在本机。

## 开发验证

在具备本机测试文件的环境下，可用系统 Python 运行无界面测试。套件覆盖相机发现、压缩图像解码、模拟切臂、TF0 和样本隔离，以及两种模式的合成求解。

```bash
source /opt/ros/humble/setup.bash
QT_QPA_PLATFORM=offscreen PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /usr/bin/python3 -m pytest -q
python3 -m py_compile scripts/*.py X5/*.py
bash -n scripts/start_camera.sh
git diff --check
```

本机最近一次验证中，系统 OpenCV 4.5.4 通过 14 项测试。`handeye` 中的 OpenCV 5.0.0 缺少 `cv2.calibrateHandEye`，两项合成求解测试失败，现场点「求解」前需验证该接口。全新检出需先确认是否已有本机 `test/`，因为该目录随 Git 忽略。
