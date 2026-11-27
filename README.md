# 双臂手眼标定

Ubuntu 22.04 amd64，ROS 2 Humble，Conda `handeye` Python 3.10。Orbbec 驱动在宿主机发布相机话题，PyQt 界面在同一个 Conda 环境中订阅相机并通过 xapi 3.4.9 直连选中的 X5。

## 安装

Conda 环境、`SDK/xapi-3.4.9-cp310-cp310-linux_x86_64.whl` 安装、相机权限和 X5 网线直连步骤见 [配置文档](docs/配置文档.md)。

## 宿主机相机和 GUI

通过 USB 连接 Gemini 335L，确认 XiaoTop 已释放相机。加载 ROS 2 环境后查询序列号。

```bash
source /opt/ros/humble/setup.bash
ros2 run orbbec_camera list_devices_node
```

找到对应相机输出中的 `serial:`，用后面的值替换 `YOUR_CAMERA_SERIAL`。项目脚本会以 `/camera_01` 启动相机。

```bash
SERIAL_NUMBER=CP253530003C bash scripts/start_camera.sh
```

确认 `/camera_01/color/image_raw/compressed` 和 `/camera_01/color/camera_info` 有发布者，且图像与内参分辨率一致。

```bash
source /opt/ros/humble/setup.bash
source /home/huo/miniconda3/etc/profile.d/conda.sh
conda activate handeye
python scripts/x5_handeye_calibration.py
```

需要预填地址时，将 `config/x5.example.yaml` 复制到 `config/x5.local.yaml` 并填写现场确认的地址，再传入 `--x5-config config/x5.local.yaml`。也可以在界面填入本次使用的地址。源 IP 用于诊断系统选路，留空时使用系统路由。界面独立于相机驱动。

## 标定

1. 选择机械臂、相机和模式。固定外部相机用 `eye_to_hand`，棋盘格固定于末端。腕部相机用 `eye_in_hand`，棋盘格固定于工作区。默认棋盘格为 9×6 内角点、24 mm 方格，可通过 `--pattern-cols`、`--pattern-rows`、`--square-size-mm` 指定。
2. 确认工作区无人、急停可用、控制器已关闭伺服，且其他客户端已释放该臂。点击连接并确认 TF0 回读，准备手动控制后再单独确认使能。
3. 用数字 1–7 或 XYZABC 选轴，方向键“上”、“下”低速点动。等待机械臂停止和图像稳定，确认棋盘完整可见及 PnP 有效后按 W 采样。D 删除最后一条样本，E 求解。Esc 请求 SDK 中止，现场紧急停止使用实体急停。建议采集 10-15 个分布充分的姿态。
4. 关闭伺服并断开后，可选择另一臂。每组数据写入 `handeye_samples/<left|right>/<会话>/<相机>/`。JSON 和 NPZ 的平移单位为毫米，输出分别为相机到 Base 或相机到 TF0（结果标识 `camera_to_tcp`）。用未参与求解的姿态及独立目标做现场精度验证。

## 离线重算

```bash
python scripts/x5_handeye_calibration.py \
  --recompute-samples /path/to/raw-samples --handeye-mode eye_to_hand
```

默认生成到 `handeye_samples/recomputed/` 下的新目录。使用与采样相同的棋盘格参数。旧原始样本须有 TF0 标记，缺失臂标识时结果保留未知归属。独立诊断脚本 `scripts/diagnose_handeye_samples.py` 适用于眼在手外、9×6 内角点、24 mm 方格的数据。

## 测试

```bash
QT_QPA_PLATFORM=offscreen PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q
```
