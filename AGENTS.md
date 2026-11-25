# DualArm_handeye Codex 交接说明

## 项目目标

本项目用于独立完成左右两台 X5 机械臂的手眼标定。已从 LiuZhou Grasp 迁移棋盘格检测、PnP、OpenCV Park 求解、样本保存、误差验证和离线重算。

当前运行方案为 Ubuntu 22.04、ROS 2 Humble、宿主机 Orbbec 驱动和 Conda Python 3.10。Conda 环境名约定为 `handeye`。手眼 GUI 在同一 Python 3.10 进程中加载 ROS 2 Python 模块、NumPy、OpenCV、PyQt5、PyYAML 和 X5 SDK。X5 SDK 直接由 GUI 加载，不使用 Docker 和 HTTP bridge。

## 环境规则

环境由用户手动配置。除非用户在当前请求中明确要求，不要自动创建 Conda 环境、安装 apt 或 Python 依赖、修改 ROS 配置或写入 udev 规则。

环境安装和启动命令以 [docs/配置文档.md](docs/配置文档.md) 为准。每个运行 GUI 的新终端执行：

```bash
cd /home/huo/project/DualArm_handeye
source /opt/ros/humble/setup.bash
source /home/huo/miniconda3/etc/profile.d/conda.sh
conda activate handeye
```

ROS 2 setup 不能省略。它配置 `ROS_DISTRO`、`AMENT_PREFIX_PATH`、ROS Python 模块路径、共享库路径和 DDS 环境。Conda 负责 Python 3.10 和普通 Python 依赖，不能替代 ROS setup。

相机驱动终端只执行：

```bash
source /opt/ros/humble/setup.bash
```

## Python 和 X5 SDK

GUI 必须在同一个 Python 3.10 进程中导入：

```text
numpy
cv2
PyQt5
yaml
rclpy
sensor_msgs
xapi.api
```

使用 `SDK/xapi-3.3.8-cp310-cp310-linux_x86_64.whl`。不要使用 LiuZhou Grasp 的 xapi-3.2.5 wheel，该版本在当前 Ubuntu 22.04 宿主机上需要更高版本 glibc。

环境检查：

```bash
source /opt/ros/humble/setup.bash
source /home/huo/miniconda3/etc/profile.d/conda.sh
conda activate handeye
python - <<'PY'
import cv2, numpy, rclpy, yaml, xapi.api
from PyQt5 import QtWidgets
from sensor_msgs.msg import CameraInfo, CompressedImage
print("python:", __import__("sys").executable)
print("version:", __import__("sys").version.split()[0])
print("opencv:", cv2.__version__)
print("numpy:", numpy.__version__)
print("rclpy:", rclpy.__file__)
print("xapi:", xapi.api.__file__)
print("ROS messages:", CameraInfo.__name__, CompressedImage.__name__)
PY
```

## 相机边界和启动

相机由宿主机 Orbbec ROS 2 驱动发布。GUI 只订阅已存在的话题，不启动、停止、重启或杀死相机驱动。启动相机前确认 XiaoTop 或其他程序已释放设备。

Gemini 330 系列启动示例：

```bash
cd /home/huo/project/DualArm_handeye
source /opt/ros/humble/setup.bash
SERIAL_NUMBER=YOUR_CAMERA_SERIAL \
CAMERA_NAME=camera_01 \
bash scripts/start_camera.sh
```

必须提供：

```text
/<camera_name>/color/image_raw/compressed
/<camera_name>/color/camera_info
```

检查话题：

```bash
source /opt/ros/humble/setup.bash
ros2 topic list -t | grep -E '^/camera_(01|02)/color/(image_raw/compressed|camera_info)$'
ros2 topic info -v /camera_01/color/image_raw/compressed
ros2 topic info -v /camera_01/color/camera_info
```

## GUI 入口

```bash
python scripts/x5_handeye_calibration.py --handeye-mode eye_to_hand
python scripts/x5_handeye_calibration.py --handeye-mode eye_in_hand
```

可选配置文件：

```bash
cp config/x5.example.yaml config/x5.local.yaml
python scripts/x5_handeye_calibration.py \
  --x5-config config/x5.local.yaml \
  --handeye-mode eye_to_hand
```

GUI 支持选择左臂或右臂，一次连接一台控制器，读取并校验 TF0，准备手动控制，低速点动，记录棋盘格、PnP 和机器人位姿，并输出 JSON/NPZ。默认棋盘格为 9×6 个内角点、方格边长 24 mm，可通过 `--pattern-cols`、`--pattern-rows` 和 `--square-size-mm` 调整。

## 机器人安全边界

默认只做静态检查、ROS 话题检查、网络路由诊断和控制器状态读取。

连接机器人、准备手动模式、使能伺服、点动和采样都需要用户在当前请求中明确授权。任何运动前确认工作区无人、速度足够低、急停可用、其他客户端已断开，并核对当前机械臂和控制器身份。

不要为了测试代码自动驱动真实机器人。不要自动修复实际机器人路由，不要自动修改机器人 IP、源 IP、工具坐标或标定配置。

切换机械臂前依次关闭伺服、断开当前机械臂、确认连接释放，再选择另一台机械臂。

## 样本和结果

默认路径：

```text
handeye_samples/<left|right>/<会话>/<相机>/
```

JSON/NPZ 的平移单位为毫米：

```text
eye_in_hand -> camera_to_tcp
eye_to_hand -> camera_to_base
```

样本必须来自同一相机、同一机械臂、同一控制器，并且全部使用 TF0。不要混合左右臂、不同控制器或不同相机的样本。

不要提交原始样本、运行日志、X5 runtime 提取目录和临时结果。`handeye_samples/` 和 `config/x5.local.yaml` 已列入 `.gitignore`。

## 离线重算和测试

```bash
source /opt/ros/humble/setup.bash
source /home/huo/miniconda3/etc/profile.d/conda.sh
conda activate handeye
python scripts/x5_handeye_calibration.py \
  --recompute-samples /path/to/raw-samples \
  --handeye-mode eye_to_hand
```

```bash
QT_QPA_PLATFORM=offscreen \
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
python -m pytest -q
python -m py_compile scripts/*.py X5/*.py test/*.py
bash -n scripts/start_camera.sh
git diff --check
```

此前迁移版本已通过 14 个测试，覆盖相机发现、左右臂切换、TF0 校验、样本隔离、合成手眼求解和两种标定模式。后续修改需保持这些测试通过，并为新增行为补充最小测试。

## 当前状态

已完成：

- 手眼标定 GUI、X5 直连、左右臂选择、样本隔离、离线工具和宿主机相机启动脚本迁移。
- `docs/配置文档.md` 已记录手动 Conda 和 ROS 2 配置。

待完成：

- 用户手动安装和验证 ROS 2 Humble Orbbec 软件包及设备规则。
- 确认相机序列号与 `camera_01`、`camera_02` 的映射。
- 验证真实相机话题、GUI 图像和棋盘格 PnP。
- 在用户明确授权和现场安全确认后，分别验证左右 X5 的只读位姿读取和低速采样。
- 使用未参与求解的姿态和独立目标完成现场精度验收。

## 文档来源和迁移边界

环境命令以 `docs/配置文档.md` 为准。

不要把 LiuZhou Grasp 的 Docker 启动流程、Python 3.12 GUI、X5 HTTP bridge、主 GUI、HOST、ROI、夹爪和抓取任务逻辑迁回本项目。
