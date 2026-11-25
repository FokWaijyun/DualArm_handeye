#!/usr/bin/env python3
"""X5 相机手眼标定工具。

工具为选中的一台压缩 ROS 相机记录棋盘格角点、当前 X5 TCP 位姿和相机内参。
它只订阅宿主机已发布的 ROS 相机话题，并在本进程中经 X5 SDK 逐臂操作。
它通过 OpenCV Park 求解器计算眼在手上（相机到 TCP）或眼在手外（相机到 Base）
标定，仅将最终 JSON/NPZ 结果作为长期输出，并支持使用原始样本离线重新计算。
记录样本和机器人操作均需操作员执行；工具绝不会自动移动机器人。
"""

from __future__ import annotations

import argparse
import ipaddress
import json
import os
import sys
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Optional
from uuid import uuid4

import yaml

import cv2
import numpy as np

# 使用与当前 PyQt 库匹配的 Qt 插件，不能强制混用系统 Qt5 插件。
os.environ.pop("QT_QPA_FONTDIR", None)
os.environ.setdefault("QT_QPA_PLATFORM", "xcb")
# Respect the ROS environment established by XiaoTop.  Forcing a domain or
# localhost-only discovery here can isolate this process from the camera
# driver when XiaoTop uses its default ROS settings.

from PyQt5 import QtCore, QtGui, QtWidgets

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from X5.x5_robot import RobotPose, X5Robot, X5RobotError, diagnose_x5_network

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, CompressedImage

    ROS_AVAILABLE = True
except Exception:
    ROS_AVAILABLE = False
    rclpy = None
    Node = object
    CameraInfo = object
    CompressedImage = object
    qos_profile_sensor_data = None


JOINT_KEYS = {
    QtCore.Qt.Key_1: 0,
    QtCore.Qt.Key_2: 1,
    QtCore.Qt.Key_3: 2,
    QtCore.Qt.Key_4: 3,
    QtCore.Qt.Key_5: 4,
    QtCore.Qt.Key_6: 5,
    QtCore.Qt.Key_7: 6,
}
CART_KEYS = {
    QtCore.Qt.Key_X: 0,
    QtCore.Qt.Key_Y: 1,
    QtCore.Qt.Key_Z: 2,
    QtCore.Qt.Key_A: 3,
    QtCore.Qt.Key_B: 4,
    QtCore.Qt.Key_C: 5,
}
COLOR_COMPRESSED_SUFFIX = "/color/image_raw/compressed"
COLOR_INFO_SUFFIX = "/color/camera_info"
COMPRESSED_IMAGE_TYPE = "sensor_msgs/msg/CompressedImage"
CAMERA_INFO_TYPE = "sensor_msgs/msg/CameraInfo"


@dataclass(frozen=True)
class CameraSource:
    """一个相机命名空间对应的压缩彩色和标定话题。"""

    namespace: str
    color_topic: str
    info_topic: str

    @property
    def label(self) -> str:
        return self.namespace.lstrip("/") or "/"


def discover_compressed_color_cameras(topic_types: dict[str, list[str]]) -> list[CameraSource]:
    """返回同时具有 JPEG 彩色流和匹配彩色内参话题的相机。"""
    sources: list[CameraSource] = []
    for color_topic, types in topic_types.items():
        if not color_topic.endswith(COLOR_COMPRESSED_SUFFIX) or COMPRESSED_IMAGE_TYPE not in types:
            continue
        namespace = color_topic[: -len(COLOR_COMPRESSED_SUFFIX)]
        info_topic = namespace + COLOR_INFO_SUFFIX
        if CAMERA_INFO_TYPE in topic_types.get(info_topic, []):
            sources.append(CameraSource(namespace, color_topic, info_topic))
    return sorted(sources, key=lambda source: source.label)


def compressed_color_to_bgr(msg: CompressedImage) -> np.ndarray:
    """解码 compressed_image_transport 彩色数据，供棋盘格检测使用。"""
    data = np.frombuffer(msg.data, dtype=np.uint8)
    image = cv2.imdecode(data, cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError("cv2.imdecode 未返回图像")
    return np.ascontiguousarray(image)


@dataclass
class FrameBundle:
    """选中相机最新的压缩彩色帧和内参。"""

    image_bgr: Optional[np.ndarray] = None
    camera_matrix: Optional[np.ndarray] = None
    dist_coeffs: Optional[np.ndarray] = None
    stamp: float = 0.0
    status: str = "请选择压缩彩色相机"
    camera_namespace: str = ""
    camera_generation: int = 0

    def clear(self, source: CameraSource | None, generation: int) -> None:
        self.image_bgr = None
        self.camera_matrix = None
        self.dist_coeffs = None
        self.stamp = 0.0
        self.camera_namespace = "" if source is None else source.namespace
        self.camera_generation = int(generation)
        self.status = (
            "请选择压缩彩色相机"
            if source is None
            else f"正在订阅 {source.label} 的压缩彩色图像"
        )


@dataclass
class DetectionResult:
    """单帧的棋盘格检测结果、可选 PnP 位姿和图像数据。"""

    found: bool
    image_bgr: np.ndarray
    image_size: tuple[int, int]
    corners: Optional[np.ndarray] = None
    camera_matrix: Optional[np.ndarray] = None
    dist_coeffs: Optional[np.ndarray] = None
    rvec: Optional[np.ndarray] = None
    tvec: Optional[np.ndarray] = None
    reprojection_error: Optional[float] = None
    camera_namespace: str = ""
    camera_generation: int = 0


@dataclass
class CalibrationSample:
    """将棋盘格位姿和机器人位姿关联的一条可持久化标定观测。"""

    index: int
    timestamp: str
    camera_namespace: str
    robot_pose: RobotPose
    joint_values: list[float]
    active_tool_frame_index: int
    image_size: tuple[int, int]
    corners: np.ndarray
    image_path: str
    camera_matrix: Optional[np.ndarray]
    dist_coeffs: Optional[np.ndarray]
    rvec: Optional[np.ndarray]
    tvec: Optional[np.ndarray]
    reprojection_error: Optional[float]
    arm_id: str = ""
    controller_ip: str = ""


class SignalBus(QtCore.QObject):
    """标定相机和机器人工作线程发出的 Qt 安全通知。"""

    log = QtCore.pyqtSignal(str)
    robot_status = QtCore.pyqtSignal(str)
    sample_count = QtCore.pyqtSignal(int)
    result_path = QtCore.pyqtSignal(str)
    camera_sources = QtCore.pyqtSignal(object)


class RosCameraNode(Node):
    def __init__(self, bundle: FrameBundle, lock: threading.Lock):
        super().__init__("x5_handeye_calibration_viewer")
        self.bundle = bundle
        self.lock = lock
        self._source: CameraSource | None = None
        self._subscriptions = []
        self._sources_changed = None
        self._is_source_selected = None
        self._last_sources: tuple[CameraSource, ...] = ()
        self.discovery_timer = self.create_timer(1.0, self._discover_topics)

    def set_sources_callback(self, callback) -> None:
        self._sources_changed = callback

    def set_source_selection_callback(self, callback) -> None:
        """拒绝 UI 已切换到另一相机后才到达的回调。"""
        self._is_source_selected = callback

    def select_camera(self, source: CameraSource | None, generation: int) -> None:
        if self._source == source:
            return
        for subscription in self._subscriptions:
            self.destroy_subscription(subscription)
        self._subscriptions.clear()
        self._source = source
        with self.lock:
            self.bundle.clear(source, generation)
        if source is None:
            return
        self._subscriptions = [
            self.create_subscription(
                CompressedImage,
                source.color_topic,
                lambda msg, expected=source: self._on_compressed_image(msg, expected),
                qos_profile_sensor_data,
            ),
            self.create_subscription(
                CameraInfo,
                source.info_topic,
                lambda msg, expected=source: self._on_info(msg, expected),
                qos_profile_sensor_data,
            ),
        ]
        self.get_logger().info(f"已订阅标定压缩彩色图像：{source.namespace}")

    def _discover_topics(self) -> None:
        sources = tuple(
            discover_compressed_color_cameras(dict(self.get_topic_names_and_types()))
        )
        if sources == self._last_sources:
            return
        self._last_sources = sources
        if self._sources_changed is not None:
            self._sources_changed(list(sources))

    def _set_status(self, text: str) -> None:
        with self.lock:
            self.bundle.status = text

    def _on_compressed_image(self, msg: CompressedImage, source: CameraSource) -> None:
        if source != self._source or not self._source_is_selected(source):
            return
        try:
            image = compressed_color_to_bgr(msg)
        except Exception as exc:
            self.get_logger().warn(f"压缩彩色图像转换失败：{exc}")
            return
        with self.lock:
            self.bundle.image_bgr = image
            self.bundle.stamp = time.time()
            self.bundle.status = f"正在接收 {source.label} 的压缩图像帧"

    def _on_info(self, msg: CameraInfo, source: CameraSource) -> None:
        if source != self._source or not self._source_is_selected(source):
            return
        if len(msg.k) != 9:
            return
        camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        if msg.d:
            dist_coeffs = np.array(msg.d, dtype=np.float64).reshape(-1, 1)
        else:
            dist_coeffs = np.zeros((5, 1), dtype=np.float64)
        with self.lock:
            self.bundle.camera_matrix = camera_matrix
            self.bundle.dist_coeffs = dist_coeffs
            self.bundle.status = f"正在接收 {source.label} 的相机内参"

    def _source_is_selected(self, source: CameraSource) -> bool:
        return self._is_source_selected is None or bool(self._is_source_selected(source))


class RosCameraThread(threading.Thread):
    """在标定 Qt 窗口外独立运行标定 ROS 节点。"""

    def __init__(self, bundle: FrameBundle, lock: threading.Lock):
        super().__init__(daemon=True)
        self.bundle = bundle
        self.lock = lock
        self.node: Optional[RosCameraNode] = None
        self.stop_event = threading.Event()
        self.error: Optional[str] = None
        self._selected_source: CameraSource | None = None
        self._camera_generation = 0
        self._sources_callback = None
        self._source_lock = threading.Lock()

    def set_sources_callback(self, callback) -> None:
        self._sources_callback = callback

    def select_camera(self, source: CameraSource | None, generation: int) -> None:
        with self._source_lock:
            self._selected_source = source
            self._camera_generation = generation

    def _is_selected_source(self, source: CameraSource) -> bool:
        with self._source_lock:
            return source == self._selected_source

    def run(self) -> None:
        if not ROS_AVAILABLE:
            self.error = "未安装可用的 ROS 2 Python 模块"
            return
        try:
            if not rclpy.ok():
                rclpy.init(args=None)
            self.node = RosCameraNode(self.bundle, self.lock)
            self.node.set_sources_callback(self._sources_callback)
            self.node.set_source_selection_callback(self._is_selected_source)
            while rclpy.ok() and not self.stop_event.is_set():
                with self._source_lock:
                    source = self._selected_source
                    generation = self._camera_generation
                self.node.select_camera(source, generation)
                rclpy.spin_once(self.node, timeout_sec=0.05)
        except Exception as exc:
            self.error = str(exc)
        finally:
            if self.node is not None:
                self.node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

    def stop(self) -> None:
        self.stop_event.set()


class ImagePanel(QtWidgets.QLabel):
    def __init__(self) -> None:
        super().__init__()
        self.setText("正在等待 ROS 2 图像...")
        self.setAlignment(QtCore.Qt.AlignCenter)
        self.setMinimumSize(860, 540)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.setStyleSheet("QLabel { background: #101418; color: white; }")
        self._image_rgb: Optional[np.ndarray] = None

    def set_status(self, text: str) -> None:
        self._image_rgb = None
        self.clear()
        self.setText(text)

    def set_image(self, image_rgb: np.ndarray) -> None:
        self._image_rgb = np.ascontiguousarray(image_rgb)
        self._refresh_pixmap()

    def resizeEvent(self, event: QtGui.QResizeEvent) -> None:
        super().resizeEvent(event)
        self._refresh_pixmap()

    def _refresh_pixmap(self) -> None:
        if self._image_rgb is None:
            return
        h, w, channels = self._image_rgb.shape
        qimage = QtGui.QImage(
            self._image_rgb.data,
            w,
            h,
            channels * w,
            QtGui.QImage.Format_RGB888,
        )
        pixmap = QtGui.QPixmap.fromImage(qimage).scaled(
            self.size(),
            QtCore.Qt.KeepAspectRatio,
            QtCore.Qt.SmoothTransformation,
        )
        self.setPixmap(pixmap)


class MainWindow(QtWidgets.QMainWindow):
    """用于选相机、低速 X5 点动和计算标定结果的操作员界面。"""

    CALIBRATION_SPEED_LIMIT_PERCENT = 30

    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__()
        self.args = args
        self.setWindowTitle("X5 手眼标定棋盘格查看器")
        self.resize(1320, 760)
        self.setMinimumSize(1040, 640)

        self.bundle = FrameBundle()
        self.frame_lock = threading.Lock()
        self.samples_lock = threading.Lock()
        self.samples: list[CalibrationSample] = []
        self.latest_detection: Optional[DetectionResult] = None

        self.selected_mode = "joint"
        self.selected_joint_axis = 0
        self.selected_cart_axis = 0
        self.motion_busy = False
        self.record_busy = False
        self.compute_busy = False
        self.robot: X5Robot | None = None
        self.arm_id = ""
        self.arm_addresses = load_arm_addresses(args.x5_config)
        self.connection_busy = False
        self.abort_busy = False
        self.manual_ready = False
        self.servo_enabled = False
        self._last_e1_tcp_pose: RobotPose | None = None
        self._last_camera_status_logged = ""
        self._camera_sources: dict[str, CameraSource] = {}
        self._current_camera_source: CameraSource | None = None
        self._camera_generation = 0

        self.output_root = Path(args.output_dir).expanduser().resolve()
        self.output_dir = self.output_root

        self.signals = SignalBus()
        self.signals.log.connect(self._append_log)
        self.signals.robot_status.connect(self._set_robot_status)
        self.signals.sample_count.connect(self._set_sample_count)
        self.signals.result_path.connect(self._set_result_path)
        self.signals.camera_sources.connect(self._update_camera_sources)

        self._build_ui()
        app = QtWidgets.QApplication.instance()
        if app is not None:
            app.installEventFilter(self)

        self.camera_thread = RosCameraThread(self.bundle, self.frame_lock)
        self.camera_thread.set_sources_callback(self.signals.camera_sources.emit)
        self.camera_thread.start()

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._refresh)
        self.timer.start(33)

        self._log(f"数据目录：{self.output_root}")
        self._log(f"棋盘格内角点：{args.pattern_cols} x {args.pattern_rows}")
        self._log(f"方格边长：{args.square_size_mm:.3f} mm")
        self._log("仅订阅宿主机已发布的 ROS 相机话题；不会管理 Orbbec 进程")
        self._log("标定工具独占当前 X5 控制器会话")

    def _build_ui(self) -> None:
        root = QtWidgets.QWidget()
        self.setCentralWidget(root)
        layout = QtWidgets.QHBoxLayout(root)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(10)

        self.image_panel = ImagePanel()
        layout.addWidget(self.image_panel, 1)

        side = QtWidgets.QWidget()
        side.setFixedWidth(350)
        side_layout = QtWidgets.QVBoxLayout(side)
        side_layout.setAlignment(QtCore.Qt.AlignTop)
        layout.addWidget(side)

        self.camera_label = QtWidgets.QLabel("相机：请选择相机")
        self.board_label = QtWidgets.QLabel("棋盘格：未找到")
        self.sample_label = QtWidgets.QLabel("样本数：0")
        self.selection_label = QtWidgets.QLabel("已选轴：J1")
        self.robot_label = QtWidgets.QLabel("机器人：未连接")
        self.result_label = QtWidgets.QLabel("结果：尚未计算")
        for label in (
            self.camera_label,
            self.board_label,
            self.sample_label,
            self.selection_label,
            self.robot_label,
            self.result_label,
        ):
            label.setWordWrap(True)

        self.joint_step_spin = make_spin(0.01, 30.0, self.args.joint_step_deg, 0.1, " deg")
        self.xyz_step_spin = make_spin(0.01, 100.0, self.args.cart_step_mm, 0.5, " mm")
        self.abc_step_spin = make_spin(0.01, 30.0, self.args.rot_step_deg, 0.1, " deg")
        self.arm_combo = QtWidgets.QComboBox()
        self.arm_combo.addItem("请选择机械臂", "")
        self.arm_combo.addItem("左臂", "left")
        self.arm_combo.addItem("右臂", "right")
        self.arm_combo.currentIndexChanged.connect(self._select_arm)
        self.robot_ip_edit = QtWidgets.QLineEdit()
        self.robot_ip_edit.setPlaceholderText("例如 192.168.1.10")
        self.robot_ip_edit.setToolTip("本次标定连接的 X5 控制器地址；修改后点击连接或诊断即可生效")
        self.robot_source_ip_edit = QtWidgets.QLineEdit()
        self.robot_source_ip_edit.setPlaceholderText("例如 192.168.1.211（可选）")
        self.robot_source_ip_edit.setToolTip("访问控制器时使用的本机源地址；留空由系统路由决定")
        max_manual_speed = self.CALIBRATION_SPEED_LIMIT_PERCENT
        self.robot_speed_spin = make_spin(
            1.0,
            max_manual_speed,
            min(self.args.speed, max_manual_speed),
            1.0,
            " %",
        )
        self.robot_speed_spin.setDecimals(0)
        self.handeye_mode_combo = QtWidgets.QComboBox()
        self.handeye_mode_combo.addItem("眼在手上：相机到 TCP", "eye_in_hand")
        self.handeye_mode_combo.addItem("眼在手外：相机到 Base", "eye_to_hand")
        if self.args.handeye_mode == "eye_to_hand":
            self.handeye_mode_combo.setCurrentIndex(1)
        self.camera_combo = QtWidgets.QComboBox()
        self.camera_combo.addItem("请选择相机", None)
        self.camera_combo.currentIndexChanged.connect(self._select_camera_from_combo)
        self.connect_robot_button = QtWidgets.QPushButton("连接所选臂")
        self.connect_robot_button.clicked.connect(self._connect_robot)
        self.diagnose_robot_button = QtWidgets.QPushButton("诊断 X5")
        self.diagnose_robot_button.clicked.connect(self._diagnose_robot)
        self.inspect_robot_state_button = QtWidgets.QPushButton("检查 X5 状态（只读 10 次）")
        self.inspect_robot_state_button.clicked.connect(self._inspect_robot_state)
        self.prepare_manual_button = QtWidgets.QPushButton("准备手动控制")
        self.prepare_manual_button.clicked.connect(self._prepare_manual)
        self.servo_button = QtWidgets.QPushButton("使能伺服（现场确认后）")
        self.servo_button.clicked.connect(lambda: self._set_servo_enabled(True))
        self.servo_off_button = QtWidgets.QPushButton("关闭伺服")
        self.servo_off_button.clicked.connect(lambda: self._set_servo_enabled(False))
        self.disconnect_robot_button = QtWidgets.QPushButton("断开当前臂")
        self.disconnect_robot_button.clicked.connect(self._disconnect_robot)
        self.apply_speed_button = QtWidgets.QPushButton("应用 X5 速度")
        self.apply_speed_button.clicked.connect(self._apply_robot_speed)

        form = QtWidgets.QFormLayout()
        form.setLabelAlignment(QtCore.Qt.AlignRight)
        form.addRow("机械臂", self.arm_combo)
        form.addRow("X5 控制器 IP", self.robot_ip_edit)
        form.addRow("本机源 IP", self.robot_source_ip_edit)
        form.addRow("X5 速度", self.robot_speed_spin)
        form.addRow("关节步长", self.joint_step_spin)
        form.addRow("XYZ 步长", self.xyz_step_spin)
        form.addRow("ABC 步长", self.abc_step_spin)
        form.addRow("手眼标定模式", self.handeye_mode_combo)
        form.addRow("相机", self.camera_combo)

        robot_buttons = QtWidgets.QHBoxLayout()
        robot_buttons.addWidget(self.connect_robot_button)
        robot_buttons.addWidget(self.diagnose_robot_button)

        robot_buttons_readonly = QtWidgets.QHBoxLayout()
        robot_buttons_readonly.addWidget(self.inspect_robot_state_button)

        robot_buttons2 = QtWidgets.QHBoxLayout()
        robot_buttons2.addWidget(self.prepare_manual_button)
        robot_buttons2.addWidget(self.servo_button)

        robot_buttons3 = QtWidgets.QHBoxLayout()
        robot_buttons3.addWidget(self.apply_speed_button)
        robot_buttons3.addWidget(self.servo_off_button)
        robot_buttons3.addWidget(self.disconnect_robot_button)

        self.log_box = QtWidgets.QPlainTextEdit()
        self.log_box.setReadOnly(True)
        self.log_box.setMinimumHeight(280)

        side_layout.addWidget(self.camera_label)
        side_layout.addWidget(self.board_label)
        side_layout.addWidget(self.sample_label)
        side_layout.addWidget(self.selection_label)
        side_layout.addWidget(self.robot_label)
        side_layout.addSpacing(8)
        side_layout.addLayout(form)
        side_layout.addLayout(robot_buttons)
        side_layout.addLayout(robot_buttons_readonly)
        side_layout.addLayout(robot_buttons2)
        side_layout.addLayout(robot_buttons3)
        side_layout.addSpacing(8)
        side_layout.addWidget(self.result_label)
        side_layout.addSpacing(8)
        side_layout.addWidget(QtWidgets.QLabel("日志"))
        side_layout.addWidget(self.log_box, 1)

    def eventFilter(self, obj: QtCore.QObject, event: QtCore.QEvent) -> bool:
        if (
            event.type() == QtCore.QEvent.KeyPress
            and self.isActiveWindow()
            and isinstance(event, QtGui.QKeyEvent)
        ):
            if self._handle_global_key(event):
                return True
        return super().eventFilter(obj, event)

    def _handle_global_key(self, event: QtGui.QKeyEvent) -> bool:
        if event.modifiers() not in (QtCore.Qt.NoModifier, QtCore.Qt.KeypadModifier):
            return False
        focus = QtWidgets.QApplication.focusWidget()
        if isinstance(focus, (QtWidgets.QAbstractSpinBox, QtWidgets.QComboBox, QtWidgets.QLineEdit)):
            return False
        if isinstance(focus, QtWidgets.QPlainTextEdit) and not focus.isReadOnly():
            return False
        key = event.key()
        if key in JOINT_KEYS:
            self.selected_mode = "joint"
            self.selected_joint_axis = JOINT_KEYS[key]
            self._update_selection_label()
            return True
        if key in CART_KEYS:
            self.selected_mode = "cart"
            self.selected_cart_axis = CART_KEYS[key]
            self._update_selection_label()
            return True
        if key == QtCore.Qt.Key_Up:
            self._jog_selected(+1.0)
            return True
        if key == QtCore.Qt.Key_Down:
            self._jog_selected(-1.0)
            return True
        if key == QtCore.Qt.Key_W:
            self._record_current_sample()
            return True
        if key == QtCore.Qt.Key_D:
            self._delete_latest_sample()
            return True
        if key == QtCore.Qt.Key_E:
            self._compute_calibration()
            return True
        if key == QtCore.Qt.Key_Escape:
            self._abort_motion()
            return True
        return False

    def _refresh(self) -> None:
        idle = not (self.connection_busy or self.motion_busy or self.record_busy or self.compute_busy or self.abort_busy)
        editable = idle and self.robot is None
        for widget in (self.arm_combo, self.robot_ip_edit, self.robot_source_ip_edit,
                       self.connect_robot_button, self.diagnose_robot_button):
            widget.setEnabled(editable)
        self.disconnect_robot_button.setEnabled(idle and self.robot is not None)
        self.camera_combo.setEnabled(idle)
        self.handeye_mode_combo.setEnabled(idle)
        with self.frame_lock:
            frame = None if self.bundle.image_bgr is None else self.bundle.image_bgr.copy()
            camera_matrix = None if self.bundle.camera_matrix is None else self.bundle.camera_matrix.copy()
            dist_coeffs = None if self.bundle.dist_coeffs is None else self.bundle.dist_coeffs.copy()
            stamp = self.bundle.stamp
            status = self.bundle.status
            camera_namespace = self.bundle.camera_namespace
            camera_generation = self.bundle.camera_generation
        if frame is None:
            self.camera_label.setText(f"相机：{status}")
            self.image_panel.set_status(f"相机：{status}")
            if status != self._last_camera_status_logged:
                self._last_camera_status_logged = status
                self._log(f"相机状态：{status}")
            self._show_camera_error()
            return
        if self._last_camera_status_logged != "receiving":
            self._last_camera_status_logged = "receiving"
            self._log(f"已接收相机图像帧：{frame.shape[1]}x{frame.shape[0]}")
        age = time.time() - stamp if stamp else 0.0
        self.camera_label.setText(
            f"相机：{camera_namespace or '--'}，{frame.shape[1]}x{frame.shape[0]}，帧龄 {age:.2f}s"
        )
        detection = detect_chessboard(
            frame,
            pattern_size=(self.args.pattern_cols, self.args.pattern_rows),
            square_size_mm=self.args.square_size_mm,
            camera_matrix=camera_matrix,
            dist_coeffs=dist_coeffs,
        )
        detection.camera_namespace = camera_namespace
        detection.camera_generation = camera_generation
        self.latest_detection = detection
        overlay = draw_detection(
            detection,
            pattern_size=(self.args.pattern_cols, self.args.pattern_rows),
            square_size_mm=self.args.square_size_mm,
        )
        self.image_panel.set_image(cv2.cvtColor(overlay, cv2.COLOR_BGR2RGB))
        if detection.found:
            if detection.rvec is not None:
                err = detection.reprojection_error
                suffix = f"，重投影误差 {err:.3f}px" if err is not None else ""
                self.board_label.setText(f"棋盘格：已找到，PnP 成功{suffix}")
            else:
                self.board_label.setText("棋盘格：已找到，正在等待相机内参")
        else:
            self.board_label.setText("棋盘格：未找到")
        self._show_camera_error()

    def _show_camera_error(self) -> None:
        error = getattr(self.camera_thread, "error", None)
        if error:
            self.camera_label.setText(f"相机：{error}")

    def _update_camera_sources(self, sources: list[CameraSource]) -> None:
        selected = self._current_camera_source.namespace if self._current_camera_source else None
        self._camera_sources = {source.namespace: source for source in sources}
        self.camera_combo.blockSignals(True)
        try:
            self.camera_combo.clear()
            self.camera_combo.addItem("请选择相机", None)
            for source in sources:
                self.camera_combo.addItem(source.label, source.namespace)
            index = self.camera_combo.findData(selected)
            self.camera_combo.setCurrentIndex(index if index >= 0 else 0)
        finally:
            self.camera_combo.blockSignals(False)
        if selected is not None and selected not in self._camera_sources:
            self._set_camera_source(None, reason="所选相机已离线")
        if not sources:
            self._log("未找到压缩彩色相机；正在等待 color/image_raw/compressed 和 color/camera_info")

    def _select_camera_from_combo(self, index: int) -> None:
        if self.record_busy or self.compute_busy or self.connection_busy or self.motion_busy:
            return
        namespace = self.camera_combo.itemData(index)
        self._set_camera_source(self._camera_sources.get(namespace) if namespace else None)

    def _set_camera_source(self, source: CameraSource | None, *, reason: str = "") -> None:
        if self._current_camera_source == source:
            return
        self._current_camera_source = source
        self._camera_generation += 1
        self.camera_thread.select_camera(source, self._camera_generation)
        with self.frame_lock:
            self.bundle.clear(source, self._camera_generation)
        self.latest_detection = None
        self.image_panel.set_status("请选择相机" if source is None else "正在等待压缩彩色图像帧...")
        self._start_sample_group()
        if source is None:
            self.board_label.setText("棋盘格：未找到")
            self._log(reason or "尚未选择相机")

    def _start_sample_group(self) -> None:
        with self.samples_lock:
            self.samples.clear()
        self.output_dir = self.output_root
        self.signals.sample_count.emit(0)
        self.signals.result_path.emit("")
        source = self._current_camera_source
        if source is not None and self.robot is not None:
            session = f"{datetime.now():%Y%m%d_%H%M%S}_{uuid4().hex[:8]}"
            self.output_dir = self.output_root / self.arm_id / session / source.label.replace("/", "_")
            self._log(f"标定数据目录：{self.output_dir}")

    def _select_arm(self, _index: int) -> None:
        if self.robot is not None or self.connection_busy or self.motion_busy or self.record_busy or self.compute_busy or self.abort_busy:
            return
        self.arm_id = self.arm_combo.currentData()
        controller_ip, source_ip = self.arm_addresses.get(self.arm_id, ("", ""))
        self.robot_ip_edit.setText(controller_ip)
        self.robot_source_ip_edit.setText(source_ip)
        self._start_sample_group()

    def _read_endpoint(self) -> tuple[str, str]:
        if self.arm_id not in {"left", "right"}:
            raise X5RobotError("请先选择左臂或右臂")
        try:
            controller_ip = str(ipaddress.IPv4Address(self.robot_ip_edit.text().strip()))
            source_ip = self.robot_source_ip_edit.text().strip()
            return controller_ip, str(ipaddress.IPv4Address(source_ip)) if source_ip else ""
        except ValueError as exc:
            raise X5RobotError(f"请填写有效的 X5 IPv4 地址：{exc}") from exc

    def _connect_robot(self) -> None:
        if self.robot is not None or self.connection_busy or self.motion_busy or self.record_busy or self.compute_busy or self.abort_busy:
            return
        try:
            controller_ip, source_ip = self._read_endpoint()
        except X5RobotError as exc:
            self._log(str(exc))
            return
        self.connection_busy = True
        self.arm_addresses[self.arm_id] = controller_ip, source_ip
        arm_label = self.arm_combo.currentText()
        self.latest_detection = None
        self.signals.robot_status.emit(f"{arm_label}：正在连接 {controller_ip}")

        def worker() -> None:
            robot = None
            try:
                robot = X5Robot(ip=controller_ip, source_ip=source_ip, remote_state=None,
                                auto_discover=False, auto_fix_route=False)
                robot.connect()
                state = robot.current_system_state()
                if state.get("enable"):
                    raise X5RobotError("连接要求控制器伺服已关闭，请现场确认")
                if state.get("alarm"):
                    robot.reset_alarm()
                robot.set_active_tool_frame(0)
                index, pose = robot.get_active_tool_frame()
                if index != 0:
                    raise X5RobotError(f"标定要求 TF0，实际 TF{index}")
                self.robot = robot
                self.manual_ready = False
                self.servo_enabled = False
                self._last_e1_tcp_pose = None
                self._start_sample_group()
                self.signals.robot_status.emit(f"{arm_label} {robot.ip}：已连接，TF0")
                self.signals.log.emit(f"TF0 定义（相对法兰）={format_pose(pose)}")
            except Exception as exc:
                try:
                    if robot is not None:
                        robot.disconnect()
                except Exception as cleanup_error:
                    self.signals.log.emit(f"连接清理失败：{cleanup_error}")
                self.signals.robot_status.emit(f"{arm_label} {controller_ip}：连接失败")
                self.signals.log.emit(str(exc))
            finally:
                self.connection_busy = False

        threading.Thread(target=worker, daemon=True).start()

    def _diagnose_robot(self) -> None:
        try:
            controller_ip, source_ip = self._read_endpoint()
        except X5RobotError as exc:
            self._log(str(exc))
            return

        def action() -> None:
            result = diagnose_x5_network(controller_ip, source_ip=source_ip, ports=())
            self.signals.log.emit(result.summary())

        self._run_robot_action(action)

    def _inspect_robot_state(self) -> None:
        """只读采集十次 X5 状态，用于确认 TF 和位姿接口的稳定性。"""
        def action() -> None:
            robot = self._require_robot()
            observations = []
            self.signals.log.emit("开始只读检查 X5 状态：连续读取 10 次，不保存标定样本、不运动机器人")
            for index in range(10):
                tf_index, tf_pose = robot.get_active_tool_frame()
                joints, manual_pose, _speed = robot.current_manual_state()
                tcp_pose = robot.current_tcp_pose(world=True)
                direct_joints = robot.current_joint()
                observations.append((tf_index, tf_pose, joints, manual_pose, tcp_pose, direct_joints))
                pose_delta = pose_distance_mm(manual_pose, tcp_pose)
                joints_delta = max_abs_difference(joints, direct_joints)
                self.signals.log.emit(
                    f"只读 {index + 1:02d}/10：TF{tf_index}；"
                    f"get_wpoint={format_pose(manual_pose)}；"
                    f"current_tcp_pose={format_pose(tcp_pose)}；"
                    f"E1={joints[6]:.4f}°；接口位姿差={pose_delta:.6f} mm；"
                    f"关节差={joints_delta:.6f}°"
                )
                if index < 9:
                    time.sleep(0.2)

            tf_indices = {item[0] for item in observations}
            tf0_pose = observations[0][1]
            manual_poses = [item[3] for item in observations]
            joints = [item[2] for item in observations]
            pose_spread = max_pose_spread_mm(manual_poses)
            joint_spread = max_joint_spread_deg(joints)
            if tf_indices != {0}:
                raise X5RobotError(f"只读检查失败：期望始终为 TF0，实际出现 {sorted(tf_indices)}")
            self.signals.log.emit(
                "只读检查完成：TF 始终为 TF0；"
                f"TF0 定义={format_pose(tf0_pose)}；"
                f"10 次 get_wpoint 最大平移波动={pose_spread:.6f} mm；"
                f"最大关节波动={joint_spread:.6f}°。"
            )

        self._run_robot_action(action)

    def _disconnect_robot(self) -> None:
        def action() -> None:
            robot = self.robot
            if robot is None:
                return
            if not robot.connected:
                raise X5RobotError("连接状态未知，请现场确认控制器状态")
            if robot.current_system_state().get("enable"):
                raise X5RobotError("请先关闭当前臂伺服")
            robot.disconnect()
            self.robot = None
            self.manual_ready = False
            self.servo_enabled = False
            self.signals.robot_status.emit("机器人：已断开")

        self._run_robot_action(action)

    def _prepare_manual(self) -> None:
        speed = int(self.robot_speed_spin.value())

        def action() -> None:
            robot = self._require_robot()
            self.manual_ready = False
            robot.prepare_manual(speed=speed)
            self.manual_ready = True
            self.signals.log.emit(f"手动控制已准备，速度 {speed}%")
            self.signals.log.emit("手动控制已准备；请确认工作区安全后再单独使能伺服")
        self._run_robot_action(action)

    def _set_servo_enabled(self, enabled: bool) -> None:
        if enabled:
            if not self.manual_ready or self.motion_busy or self.connection_busy:
                self._log("请先准备手动控制并等待当前操作结束")
                return
            if QtWidgets.QMessageBox.question(
                self, "确认使能伺服", "确认工作区无人、速度足够低、急停可用？",
                QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No, QtWidgets.QMessageBox.No,
            ) != QtWidgets.QMessageBox.Yes:
                return

        def action() -> None:
            robot = self._require_robot()
            self.servo_enabled = False
            robot.set_servo_enabled(enabled)
            self.servo_enabled = enabled
            self.signals.log.emit("伺服已使能" if enabled else "伺服已关闭")

        self._run_robot_action(action)

    def _apply_robot_speed(self) -> None:
        """应用标定界面设置的手动速度。"""
        speed = min(int(self.robot_speed_spin.value()), self.CALIBRATION_SPEED_LIMIT_PERCENT)

        def action() -> None:
            robot = self._require_robot()
            robot.set_speed(speed)
            self.signals.robot_status.emit(f"机器人：速度已设为 {speed}%")
            self.signals.log.emit(f"X5 手动速度已设为 {speed}%")

        self._run_robot_action(action)

    def _require_robot(self) -> X5Robot:
        if self.robot is None or not self.robot.connected:
            raise X5RobotError("请先连接所选机械臂")
        return self.robot

    def _run_robot_action(self, action) -> None:
        if self.motion_busy or self.connection_busy or self.record_busy or self.compute_busy or self.abort_busy:
            return
        self.motion_busy = True

        def worker() -> None:
            try:
                action()
            except X5RobotError as exc:
                self.signals.log.emit(str(exc))
            finally:
                self.motion_busy = False

        threading.Thread(target=worker, daemon=True).start()

    def _jog_selected(self, sign: float) -> None:
        if not self.manual_ready or not self.servo_enabled or self.abort_busy:
            self._log("点动前请准备手动控制并使能伺服")
            return
        if self.selected_mode == "joint":
            axis = self.selected_joint_axis
            delta = sign * self.joint_step_spin.value()

            def action() -> None:
                robot = self._require_robot()
                robot.jog_joint_rel(axis, delta)
                pose = robot.current_tcp_pose(world=True)
                joint_name = "E1 (J7)" if axis == 6 else f"J{axis + 1}"
                if axis == 6 and self._last_e1_tcp_pose is not None:
                    displacement = np.linalg.norm(np.array(pose.as_list()[:3]) - np.array(self._last_e1_tcp_pose.as_list()[:3]))
                    self.signals.log.emit(f"E1 后 TCP 平移变化：{displacement:.3f} mm")
                if axis == 6:
                    self._last_e1_tcp_pose = pose
                self.signals.log.emit(f"{joint_name} 点动 {delta:+.3f} 度；TCP {format_pose(pose)}")

            self._run_robot_action(action)
        else:
            axis = self.selected_cart_axis
            delta = sign * (self.abc_step_spin.value() if axis >= 3 else self.xyz_step_spin.value())

            def action() -> None:
                robot = self._require_robot()
                robot.jog_cartesian_rel(axis, delta, frame_type=self.args.cart_frame_type)
                pose = robot.current_tcp_pose(world=True)
                self.signals.log.emit(f"{'XYZABC'[axis]} 点动 {delta:+.3f}；TCP {format_pose(pose)}")

            self._run_robot_action(action)

    def _abort_motion(self) -> None:
        robot = self.robot
        if robot is None or self.connection_busy or self.abort_busy:
            return
        self.abort_busy = True

        def worker() -> None:
            try:
                robot.abort()
                self.signals.log.emit("已请求中止运动")
            except X5RobotError as exc:
                self.signals.log.emit(str(exc))
            finally:
                self.abort_busy = False

        threading.Thread(target=worker, daemon=True).start()

    def _record_current_sample(self) -> None:
        if self.connection_busy or self.motion_busy or self.compute_busy or self.abort_busy:
            return
        if self.record_busy:
            self._log("已跳过记录：上一条样本仍在保存")
            return
        detection = self.latest_detection
        if detection is None or not detection.found or detection.corners is None:
            self._log("无法记录：未检测到棋盘格")
            return
        source = self._current_camera_source
        if source is None or detection.camera_generation != self._camera_generation:
            self._log("无法记录：请选择相机并等待当前图像帧")
            return
        output_dir = self.output_dir
        arm_id = self.arm_id
        self.record_busy = True
        snapshot = DetectionResult(
            found=True,
            image_bgr=detection.image_bgr.copy(),
            image_size=detection.image_size,
            corners=detection.corners.copy(),
            camera_matrix=None if detection.camera_matrix is None else detection.camera_matrix.copy(),
            dist_coeffs=None if detection.dist_coeffs is None else detection.dist_coeffs.copy(),
            rvec=None if detection.rvec is None else detection.rvec.copy(),
            tvec=None if detection.tvec is None else detection.tvec.copy(),
            reprojection_error=detection.reprojection_error,
            camera_namespace=detection.camera_namespace,
            camera_generation=detection.camera_generation,
        )

        def worker() -> None:
            try:
                robot = self._require_robot()
                joints, pose, _speed = robot.current_manual_state()
                active_tf_index, _active_tf_pose = robot.get_active_tool_frame()
                if int(active_tf_index) != 0:
                    raise X5RobotError(
                        f"已拒绝记录样本：标定要求 TF0，当前实际为 TF{active_tf_index}"
                    )
                if (
                    self._current_camera_source != source
                    or snapshot.camera_generation != self._camera_generation
                ):
                    self.signals.log.emit("已忽略相机切换前采集的样本")
                    return
                with self.samples_lock:
                    index = len(self.samples)
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
                image_path = output_dir / f"sample_{index:03d}_{timestamp}.png"
                output_dir.mkdir(parents=True, exist_ok=True)
                if not cv2.imwrite(str(image_path), snapshot.image_bgr):
                    raise X5RobotError(f"无法保存图像：{image_path}")
                sample = CalibrationSample(
                    index=index,
                    timestamp=timestamp,
                    arm_id=arm_id,
                    controller_ip=robot.ip,
                    camera_namespace=snapshot.camera_namespace,
                    robot_pose=pose,
                    joint_values=joints,
                    active_tool_frame_index=int(active_tf_index),
                    image_size=snapshot.image_size,
                    corners=snapshot.corners,
                    image_path=str(image_path),
                    camera_matrix=snapshot.camera_matrix,
                    dist_coeffs=snapshot.dist_coeffs,
                    rvec=snapshot.rvec,
                    tvec=snapshot.tvec,
                    reprojection_error=snapshot.reprojection_error,
                )
                write_sample_json(sample, output_dir / f"sample_{index:03d}_{timestamp}.json")
                with self.samples_lock:
                    if (
                        self._current_camera_source != source
                        or snapshot.camera_generation != self._camera_generation
                    ):
                        self.signals.log.emit("已忽略相机切换前采集的样本")
                        return
                    self.samples.append(sample)
                    count = len(self.samples)
                self.signals.sample_count.emit(count)
                self.signals.log.emit(f"已记录样本 {index:03d}：{format_pose(pose)}")
            except (OSError, X5RobotError, cv2.error) as exc:
                self.signals.log.emit(str(exc))
            finally:
                self.record_busy = False

        threading.Thread(target=worker, daemon=True).start()

    def _delete_latest_sample(self) -> None:
        """删除本次会话中最后保存的原始标定样本及其图像。"""
        if self.record_busy:
            self._log("无法删除：上一条样本仍在保存")
            return
        if self.compute_busy:
            self._log("无法删除：标定正在计算，请完成后再删除样本")
            return
        with self.samples_lock:
            if not self.samples:
                self._log("无法删除：没有已保存的标定样本")
                return
            sample = self.samples[-1]

        image_path = Path(sample.image_path)
        json_path = image_path.with_suffix(".json")
        try:
            image_path.unlink(missing_ok=True)
            json_path.unlink(missing_ok=True)
        except OSError as exc:
            self._log(f"删除样本 {sample.index:03d} 失败：{exc}")
            return

        with self.samples_lock:
            # 记录和删除均由 GUI 线程触发；该检查仅防止未来改动造成误删。
            if not self.samples or self.samples[-1] is not sample:
                self._log("删除样本失败：最新样本已发生变化")
                return
            self.samples.pop()
            count = len(self.samples)
        self.signals.sample_count.emit(count)
        self._log(
            f"已删除最新样本 {sample.index:03d}：{image_path.name}、{json_path.name}"
        )

    def _compute_calibration(self) -> None:
        if self.record_busy or self.connection_busy or self.motion_busy:
            return
        if self.compute_busy:
            self._log("已跳过计算：标定正在运行")
            return
        with self.samples_lock:
            samples = list(self.samples)
        if len(samples) < 3:
            self._log("计算手眼标定前至少需要 3 条有效样本")
            return
        handeye_mode = self.handeye_mode_combo.currentData()
        namespaces = {sample.camera_namespace for sample in samples}
        if len(namespaces) != 1 or not self._current_camera_source:
            self._log("无法标定：样本必须全部属于同一台已选相机")
            return
        output_dir = self.output_dir
        self.compute_busy = True
        self._log(f"正在计算 {handeye_mode}；将覆盖最新结果文件")

        def worker() -> None:
            try:
                result_path = compute_and_save_result(
                    samples=samples,
                    output_dir=output_dir,
                    pattern_size=(self.args.pattern_cols, self.args.pattern_rows),
                    square_size_mm=self.args.square_size_mm,
                    handeye_mode=handeye_mode,
                )
                self.signals.result_path.emit(str(result_path))
                payload = json.loads(result_path.read_text(encoding="utf-8"))
                self.signals.log.emit(format_handeye_error_summary(payload))
                self.signals.log.emit(f"标定结果已保存：{result_path}")
            except Exception as exc:
                self.signals.log.emit(f"标定失败：{exc}")
            finally:
                self.compute_busy = False

        threading.Thread(target=worker, daemon=True).start()

    def _update_selection_label(self) -> None:
        if self.selected_mode == "joint":
            label = "E1 (J7)" if self.selected_joint_axis == 6 else f"J{self.selected_joint_axis + 1}"
            self.selection_label.setText(f"已选轴：{label}")
        else:
            self.selection_label.setText(f"已选轴：{'XYZABC'[self.selected_cart_axis]}")

    def _set_robot_status(self, text: str) -> None:
        self.robot_label.setText(text)

    def _set_sample_count(self, count: int) -> None:
        self.sample_label.setText(f"样本数：{count}")

    def _set_result_path(self, path: str) -> None:
        if not path:
            self.result_label.setText("结果：尚未计算")
            return
        try:
            payload = json.loads(Path(path).read_text(encoding="utf-8"))
            position = payload["handeye_position_error_mm"]
            rotation = payload["handeye_rotation_error_deg"]
            self.result_label.setText(
                "结果：反向校验\n"
                f"位置 RMS/最大值：{position['rms']:.3f}/{position['max']:.3f} mm\n"
                f"旋转 RMS/最大值：{rotation['rms']:.3f}/{rotation['max']:.3f} 度\n"
                f"已保存：{path}"
            )
        except (KeyError, OSError, ValueError, TypeError):
            self.result_label.setText(f"结果：{path}")

    def _log(self, message: str) -> None:
        self.signals.log.emit(message)

    def _append_log(self, message: str) -> None:
        self.log_box.appendPlainText(f"[{datetime.now().strftime('%H:%M:%S')}] {message}")

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:
        if self.robot is not None or self.connection_busy or self.motion_busy or self.record_busy or self.compute_busy or self.abort_busy:
            self._log("请等待当前操作结束，并在关闭伺服、断开机械臂后退出")
            event.ignore()
            return
        self.camera_thread.stop()
        super().closeEvent(event)


def detect_chessboard(
    image_bgr: np.ndarray,
    pattern_size: tuple[int, int],
    square_size_mm: float,
    camera_matrix: Optional[np.ndarray],
    dist_coeffs: Optional[np.ndarray],
) -> DetectionResult:
    """查找并细化棋盘格角点；仅在已知内参时估计 PnP 位姿。"""

    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    found = False
    corners = None
    if hasattr(cv2, "findChessboardCornersSB"):
        try:
            found, sb_corners = cv2.findChessboardCornersSB(
                gray,
                pattern_size,
                flags=cv2.CALIB_CB_NORMALIZE_IMAGE,
            )
            if found:
                corners = sb_corners.astype(np.float32)
        except cv2.error:
            found = False
            corners = None
    if not found:
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
        found, raw_corners = cv2.findChessboardCorners(gray, pattern_size, flags)
        if found:
            criteria = (
                cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                30,
                0.001,
            )
            corners = cv2.cornerSubPix(gray, raw_corners, (11, 11), (-1, -1), criteria)

    rvec = None
    tvec = None
    error = None
    if found and corners is not None and camera_matrix is not None:
        dist = np.zeros((5, 1), dtype=np.float64) if dist_coeffs is None else dist_coeffs
        object_points = make_object_points(pattern_size, square_size_mm)
        ok, rvec, tvec = cv2.solvePnP(object_points, corners, camera_matrix, dist)
        if ok:
            error = reprojection_error(object_points, corners, rvec, tvec, camera_matrix, dist)
        else:
            rvec = None
            tvec = None

    h, w = image_bgr.shape[:2]
    return DetectionResult(
        found=bool(found),
        image_bgr=image_bgr,
        image_size=(w, h),
        corners=corners,
        camera_matrix=camera_matrix,
        dist_coeffs=dist_coeffs,
        rvec=rvec,
        tvec=tvec,
        reprojection_error=error,
    )


def draw_detection(
    detection: DetectionResult,
    pattern_size: tuple[int, int],
    square_size_mm: float,
) -> np.ndarray:
    image = detection.image_bgr.copy()
    if detection.found and detection.corners is not None:
        cv2.drawChessboardCorners(image, pattern_size, detection.corners, True)
        if detection.rvec is not None and detection.tvec is not None and detection.camera_matrix is not None:
            dist = np.zeros((5, 1), dtype=np.float64) if detection.dist_coeffs is None else detection.dist_coeffs
            axis_len = square_size_mm * min(pattern_size) * 0.7
            draw_axes(image, detection.camera_matrix, dist, detection.rvec, detection.tvec, axis_len)
    return image


def draw_axes(
    image: np.ndarray,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    rvec: np.ndarray,
    tvec: np.ndarray,
    axis_len: float,
) -> None:
    axis = np.float32(
        [
            [0.0, 0.0, 0.0],
            [axis_len, 0.0, 0.0],
            [0.0, axis_len, 0.0],
            [0.0, 0.0, -axis_len],
        ]
    )
    projected, _ = cv2.projectPoints(axis, rvec, tvec, camera_matrix, dist_coeffs)
    pts = projected.reshape(-1, 2).astype(int)
    origin = tuple(pts[0])
    cv2.circle(image, origin, 5, (0, 0, 255), -1)
    cv2.line(image, origin, tuple(pts[1]), (255, 0, 0), 3)
    cv2.line(image, origin, tuple(pts[2]), (0, 255, 0), 3)
    cv2.line(image, origin, tuple(pts[3]), (0, 0, 255), 3)


def compute_and_save_result(
    samples: list[CalibrationSample],
    output_dir: Path,
    pattern_size: tuple[int, int],
    square_size_mm: float,
    handeye_mode: str,
) -> Path:
    """计算单相机手眼标定，并覆盖该目录的最终结果文件。"""

    camera_namespaces = {sample.camera_namespace for sample in samples}
    if len(camera_namespaces) > 1:
        raise ValueError("标定样本必须全部属于同一台相机")
    tool_frame_indices = {sample.active_tool_frame_index for sample in samples}
    if tool_frame_indices != {0}:
        actual = ", ".join(f"TF{index}" for index in sorted(tool_frame_indices))
        raise ValueError(f"标定样本必须全部使用 TF0，实际包含：{actual}")
    identities = {(sample.arm_id, sample.controller_ip) for sample in samples}
    if len(identities) != 1:
        raise ValueError("标定样本必须全部属于同一机械臂和控制器")
    arm_id, controller_ip = next(iter(identities))
    if (arm_id, controller_ip) != ("", "") and (arm_id not in {"left", "right"} or not controller_ip):
        raise ValueError("无效的机械臂或控制器标识")
    camera_namespace = next(iter(camera_namespaces), "")
    object_points = make_object_points(pattern_size, square_size_mm)
    camera_matrix = first_available_matrix(samples)
    dist_coeffs = first_available_dist(samples)
    rvecs: list[np.ndarray] = []
    tvecs: list[np.ndarray] = []

    if camera_matrix is None:
        objpoints = [object_points.copy() for _ in samples]
        imgpoints = [sample.corners.astype(np.float32) for sample in samples]
        ret, camera_matrix, dist_coeffs, cal_rvecs, cal_tvecs = cv2.calibrateCamera(
            objpoints,
            imgpoints,
            samples[0].image_size,
            None,
            None,
        )
        if not ret:
            raise RuntimeError("cv2.calibrateCamera 执行失败")
        rvecs = [rvec.astype(np.float64) for rvec in cal_rvecs]
        tvecs = [tvec.astype(np.float64) for tvec in cal_tvecs]
    else:
        if dist_coeffs is None:
            dist_coeffs = np.zeros((5, 1), dtype=np.float64)
        for sample in samples:
            # 从保存的角点重新计算 PnP，确保修正棋盘格尺寸后始终更新
            # 手眼标定使用的平移尺度。
            ok, rvec, tvec = cv2.solvePnP(
                object_points,
                sample.corners.astype(np.float32),
                camera_matrix,
                dist_coeffs,
            )
            if not ok:
                raise RuntimeError(f"样本 {sample.index} 的 solvePnP 执行失败")
            rvecs.append(rvec)
            tvecs.append(tvec)

    R_target2cam: list[np.ndarray] = []
    t_target2cam: list[np.ndarray] = []
    reprojection_errors: list[float] = []
    for sample, rvec, tvec in zip(samples, rvecs, tvecs):
        R, _ = cv2.Rodrigues(rvec)
        R_target2cam.append(R.astype(np.float64))
        t_target2cam.append(tvec.reshape(3, 1).astype(np.float64))
        reprojection_errors.append(
            reprojection_error(object_points, sample.corners, rvec, tvec, camera_matrix, dist_coeffs)
        )

    R_gripper2base: list[np.ndarray] = []
    t_gripper2base: list[np.ndarray] = []
    for sample in samples:
        R, t = robot_pose_to_rt(sample.robot_pose)
        if handeye_mode == "eye_to_hand":
            R, t = invert_rt(R, t)
        R_gripper2base.append(R.astype(np.float64))
        t_gripper2base.append(t.reshape(3, 1).astype(np.float64))

    R_result, t_result = cv2.calibrateHandEye(
        R_gripper2base,
        t_gripper2base,
        R_target2cam,
        t_target2cam,
        method=cv2.CALIB_HAND_EYE_PARK,
    )
    T_result = rt_to_matrix(R_result, t_result)
    if not np.isfinite(T_result).all():
        raise ValueError("手眼求解结果无效，请检查样本的姿态变化")
    validation = calculate_handeye_validation(
        R_gripper2base=R_gripper2base,
        t_gripper2base=t_gripper2base,
        R_target2cam=R_target2cam,
        t_target2cam=t_target2cam,
        R_result=R_result,
        t_result=t_result,
        handeye_mode=handeye_mode,
        sample_indices=[sample.index for sample in samples],
    )
    output_dir.mkdir(parents=True, exist_ok=True)
    json_path = output_dir / "handeye_latest.json"
    npz_path = output_dir / "handeye_latest.npz"
    target = "camera_to_tcp" if handeye_mode == "eye_in_hand" else "camera_to_base"
    payload = {
        "mode": handeye_mode,
        "target": target,
        "camera_namespace": camera_namespace,
        "arm_id": arm_id,
        "controller_ip": controller_ip,
        "updated_at": datetime.now().isoformat(timespec="seconds"),
        "solver": "OpenCV CALIB_HAND_EYE_PARK",
        "sample_count": len(samples),
        "active_tool_frame_index": 0,
        "pattern_inner_corners": list(pattern_size),
        "square_size_mm": square_size_mm,
        "camera_matrix": camera_matrix.tolist(),
        "dist_coeffs": dist_coeffs.reshape(-1).tolist(),
        "mean_reprojection_error_px": float(np.mean(reprojection_errors)),
        "per_sample_reprojection_error_px": [float(error) for error in reprojection_errors],
        "handeye_position_error_mm": validation["position_error_mm"],
        "handeye_rotation_error_deg": validation["rotation_error_deg"],
        "per_sample_handeye_error": validation["per_sample"],
        "fixed_transform_validation": validation["fixed_transform_validation"],
        "rotation_matrix": R_result.tolist(),
        "translation_vector_mm": t_result.reshape(3).tolist(),
        "transform_matrix": T_result.tolist(),
        "samples": [
            {
                "index": sample.index,
                "timestamp": sample.timestamp,
                "image_path": sample.image_path,
                "robot_pose_xyzabc_mm_deg": sample.robot_pose.as_list(),
                "joint_values_deg": sample.joint_values,
                "active_tool_frame_index": sample.active_tool_frame_index,
            }
            for sample in samples
        ],
    }
    json_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    np.savez(
        npz_path,
        transform_matrix=T_result,
        rotation_matrix=R_result,
        translation_vector=t_result,
        camera_matrix=camera_matrix,
        dist_coeffs=dist_coeffs,
        R_target2cam=np.array(R_target2cam),
        t_target2cam=np.array(t_target2cam),
        R_gripper2base=np.array(R_gripper2base),
        t_gripper2base=np.array(t_gripper2base),
        per_sample_position_error_mm=np.array(
            [item["position_error_mm"] for item in validation["per_sample"]]
        ),
        per_sample_rotation_error_deg=np.array(
            [item["rotation_error_deg"] for item in validation["per_sample"]]
        ),
        mode=np.array(handeye_mode),
        target=np.array(target),
        arm_id=np.array(arm_id),
        controller_ip=np.array(controller_ip),
        camera_namespace=np.array(camera_namespace),
    )
    return json_path


def calculate_handeye_validation(
    R_gripper2base: list[np.ndarray],
    t_gripper2base: list[np.ndarray],
    R_target2cam: list[np.ndarray],
    t_target2cam: list[np.ndarray],
    R_result: np.ndarray,
    t_result: np.ndarray,
    handeye_mode: str,
    sample_indices: list[int],
) -> dict[str, object]:
    """将每条观测反推到固定变换，并统计其离散程度。"""
    transforms: list[np.ndarray] = []
    T_result = rt_to_matrix(R_result, t_result)
    if not np.isfinite(T_result).all():
        raise ValueError("手眼求解结果无效，请检查样本的姿态变化")
    for R_gripper, t_gripper, R_target, t_target in zip(
        R_gripper2base,
        t_gripper2base,
        R_target2cam,
        t_target2cam,
    ):
        T_gripper = rt_to_matrix(R_gripper, t_gripper)
        T_target = rt_to_matrix(R_target, t_target)
        if handeye_mode == "eye_in_hand":
            # 对所有采集位姿而言，target-to-base 应保持一致。
            transforms.append(T_gripper @ T_result @ T_target)
        elif handeye_mode == "eye_to_hand":
            # 眼在手外输入为 base-to-gripper 和 camera-to-base，因此其乘积是
            # 固定的 target-to-gripper 变换。
            transforms.append(T_gripper @ T_result @ T_target)
        else:
            raise ValueError(f"不支持的手眼标定模式：{handeye_mode}")

    mean_transform = average_transform(transforms)
    position_errors = [
        float(np.linalg.norm(transform[:3, 3] - mean_transform[:3, 3]))
        for transform in transforms
    ]
    rotation_errors = [
        rotation_distance_deg(mean_transform[:3, :3], transform[:3, :3])
        for transform in transforms
    ]
    per_sample = [
        {
            "sample_index": sample_index,
            "position_error_mm": position_error,
            "rotation_error_deg": rotation_error,
        }
        for sample_index, position_error, rotation_error in zip(
            sample_indices,
            position_errors,
            rotation_errors,
        )
    ]
    return {
        "position_error_mm": error_statistics(position_errors),
        "rotation_error_deg": error_statistics(rotation_errors),
        "per_sample": per_sample,
        "fixed_transform_validation": {
            "source_frame": "target",
            "destination_frame": "TF0" if handeye_mode == "eye_to_hand" else "base",
            "matrix_convention": "p_destination = R @ p_target + t",
            "translation_unit": "mm",
            "mean_method": "arithmetic mean translation; SVD projection of mean rotation onto SO(3)",
            "mean_transform_matrix": mean_transform.tolist(),
            "mean_translation_vector_mm": mean_transform[:3, 3].tolist(),
            "per_sample": [
                {
                    **errors,
                    "transform_matrix": transform.tolist(),
                    "translation_vector_mm": transform[:3, 3].tolist(),
                    "translation_deviation_from_mean_mm": (
                        transform[:3, 3] - mean_transform[:3, 3]
                    ).tolist(),
                }
                for errors, transform in zip(per_sample, transforms)
            ],
        },
    }


def average_transform(transforms: list[np.ndarray]) -> np.ndarray:
    if not transforms:
        raise ValueError("没有可用于校验的变换矩阵")
    rotation_sum = np.sum([transform[:3, :3] for transform in transforms], axis=0)
    U, _, Vt = np.linalg.svd(rotation_sum)
    R_mean = U @ np.diag([1.0, 1.0, np.linalg.det(U @ Vt)]) @ Vt
    t_mean = np.mean([transform[:3, 3] for transform in transforms], axis=0)
    return rt_to_matrix(R_mean, t_mean)


def rotation_distance_deg(R_reference: np.ndarray, R_observed: np.ndarray) -> float:
    cosine = (np.trace(R_reference.T @ R_observed) - 1.0) / 2.0
    return float(np.degrees(np.arccos(np.clip(cosine, -1.0, 1.0))))


def error_statistics(errors: list[float]) -> dict[str, float]:
    values = np.asarray(errors, dtype=np.float64)
    return {
        "mean": float(np.mean(values)),
        "rms": float(np.sqrt(np.mean(np.square(values)))),
        "max": float(np.max(values)),
    }


def format_handeye_error_summary(payload: dict[str, object]) -> str:
    position = payload["handeye_position_error_mm"]
    rotation = payload["handeye_rotation_error_deg"]
    return (
        "反向校验误差："
        f"位置 平均值/RMS/最大值={position['mean']:.3f}/{position['rms']:.3f}/{position['max']:.3f} mm；"
        f"旋转 平均值/RMS/最大值={rotation['mean']:.3f}/{rotation['rms']:.3f}/{rotation['max']:.3f} 度"
    )


def write_sample_json(sample: CalibrationSample, path: Path) -> None:
    """持久化一条原始观测，以便后续离线重新计算标定。"""

    payload = {
        "index": sample.index,
        "timestamp": sample.timestamp,
        "camera_namespace": sample.camera_namespace,
        "arm_id": sample.arm_id,
        "controller_ip": sample.controller_ip,
        "image_path": sample.image_path,
        "image_size": list(sample.image_size),
        "robot_pose_xyzabc_mm_deg": sample.robot_pose.as_list(),
        "joint_values_deg": sample.joint_values,
        "active_tool_frame_index": sample.active_tool_frame_index,
        "corners": sample.corners.reshape(-1, 2).tolist(),
        "camera_matrix": None if sample.camera_matrix is None else sample.camera_matrix.tolist(),
        "dist_coeffs": None if sample.dist_coeffs is None else sample.dist_coeffs.reshape(-1).tolist(),
        "rvec": None if sample.rvec is None else sample.rvec.reshape(-1).tolist(),
        "tvec": None if sample.tvec is None else sample.tvec.reshape(-1).tolist(),
        "reprojection_error_px": sample.reprojection_error,
    }
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def load_saved_samples(directory: Path) -> list[CalibrationSample]:
    """读取一台相机的原始观测，并要求满足 OpenCV 手眼计算的最少数据量。"""

    samples: list[CalibrationSample] = []
    for path in sorted(directory.glob("sample_*.json")):
        data = json.loads(path.read_text(encoding="utf-8"))
        pose = data.get("robot_pose_xyzabc_mm_deg")
        corners = data.get("corners")
        active_tool_frame_index = data.get("active_tool_frame_index")
        if (
            not isinstance(pose, list)
            or len(pose) != 6
            or not corners
            or not isinstance(active_tool_frame_index, int)
        ):
            raise ValueError(f"无效的标定样本：{path}")
        camera_matrix = data.get("camera_matrix")
        dist_coeffs = data.get("dist_coeffs")
        rvec = data.get("rvec")
        tvec = data.get("tvec")
        samples.append(
            CalibrationSample(
                index=int(data["index"]),
                timestamp=str(data["timestamp"]),
                camera_namespace=str(data.get("camera_namespace", "")),
                arm_id=str(data.get("arm_id", "")),
                controller_ip=str(data.get("controller_ip", "")),
                robot_pose=RobotPose(*[float(value) for value in pose]),
                joint_values=[float(value) for value in data.get("joint_values_deg", [])],
                active_tool_frame_index=active_tool_frame_index,
                image_size=tuple(int(value) for value in data["image_size"]),
                corners=np.asarray(corners, dtype=np.float32).reshape(-1, 1, 2),
                image_path=str(data.get("image_path", "")),
                camera_matrix=None if camera_matrix is None else np.asarray(camera_matrix, dtype=np.float64),
                dist_coeffs=None if dist_coeffs is None else np.asarray(dist_coeffs, dtype=np.float64).reshape(-1, 1),
                rvec=None if rvec is None else np.asarray(rvec, dtype=np.float64).reshape(3, 1),
                tvec=None if tvec is None else np.asarray(tvec, dtype=np.float64).reshape(3, 1),
                reprojection_error=(
                    None
                    if data.get("reprojection_error_px") is None
                    else float(data["reprojection_error_px"])
                ),
            )
        )
    samples.sort(key=lambda sample: sample.index)
    if len(samples) < 3:
        raise ValueError(f"目录中至少需要 3 条已保存样本：{directory}")
    return samples


def make_object_points(pattern_size: tuple[int, int], square_size_mm: float) -> np.ndarray:
    cols, rows = pattern_size
    objp = np.zeros((cols * rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp *= float(square_size_mm)
    return objp


def reprojection_error(
    object_points: np.ndarray,
    image_points: np.ndarray,
    rvec: np.ndarray,
    tvec: np.ndarray,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
) -> float:
    projected, _ = cv2.projectPoints(object_points, rvec, tvec, camera_matrix, dist_coeffs)
    return float(cv2.norm(image_points, projected, cv2.NORM_L2) / len(projected))


def robot_pose_to_rt(pose: RobotPose) -> tuple[np.ndarray, np.ndarray]:
    """将 X5 XYZABC（毫米/角度）转换为列向量齐次 RT 对。"""

    R = euler_abc_to_rotation(pose.a, pose.b, pose.c)
    t = np.array([pose.x, pose.y, pose.z], dtype=np.float64).reshape(3, 1)
    return R, t


def euler_abc_to_rotation(a_deg: float, b_deg: float, c_deg: float) -> np.ndarray:
    """应用 X5 外旋 XYZ 约定：Rz(C) @ Ry(B) @ Rx(A)。"""

    a, b, c = np.radians([a_deg, b_deg, c_deg])
    ca, sa = np.cos(a), np.sin(a)
    cb, sb = np.cos(b), np.sin(b)
    cc, sc = np.cos(c), np.sin(c)
    Rx = np.array([[1, 0, 0], [0, ca, -sa], [0, sa, ca]], dtype=np.float64)
    Ry = np.array([[cb, 0, sb], [0, 1, 0], [-sb, 0, cb]], dtype=np.float64)
    Rz = np.array([[cc, -sc, 0], [sc, cc, 0], [0, 0, 1]], dtype=np.float64)
    # 对列向量 base_T_tcp 位姿而言，get_wpoint() 的 ABC 是外旋 XYZ，
    # 等价于 Rz(C) @ Ry(B) @ Rx(A)。
    return Rz @ Ry @ Rx


def rt_to_matrix(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = t.reshape(3)
    return T


def invert_rt(R: np.ndarray, t: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    R_inv = R.T
    t_inv = -R_inv @ t.reshape(3, 1)
    return R_inv, t_inv


def first_available_matrix(samples: list[CalibrationSample]) -> Optional[np.ndarray]:
    for sample in samples:
        if sample.camera_matrix is not None:
            return sample.camera_matrix.astype(np.float64)
    return None


def first_available_dist(samples: list[CalibrationSample]) -> Optional[np.ndarray]:
    for sample in samples:
        if sample.dist_coeffs is not None:
            return sample.dist_coeffs.astype(np.float64).reshape(-1, 1)
    return None


def format_pose(pose: RobotPose) -> str:
    values = pose.as_list()
    return (
        f"XYZ=({values[0]:.2f},{values[1]:.2f},{values[2]:.2f}) "
        f"ABC=({values[3]:.2f},{values[4]:.2f},{values[5]:.2f})"
    )


def pose_distance_mm(first: RobotPose, second: RobotPose) -> float:
    """返回两个 X5 位姿原点之间的直线距离，不混入 ABC 角度。"""
    return float(np.linalg.norm(np.asarray(first.as_list()[:3]) - np.asarray(second.as_list()[:3])))


def max_pose_spread_mm(poses: list[RobotPose]) -> float:
    """计算一批静止状态读取的最大 TCP 平移波动。"""
    if len(poses) < 2:
        return 0.0
    positions = np.asarray([pose.as_list()[:3] for pose in poses], dtype=np.float64)
    return float(np.max(np.linalg.norm(positions[:, None, :] - positions[None, :, :], axis=2)))


def max_abs_difference(first: list[float], second: list[float]) -> float:
    """返回等长数值序列的最大绝对差。"""
    if len(first) != len(second):
        raise ValueError(f"数值长度不一致：{len(first)} != {len(second)}")
    if not first:
        return 0.0
    return float(np.max(np.abs(np.asarray(first, dtype=np.float64) - np.asarray(second, dtype=np.float64))))


def max_joint_spread_deg(samples: list[list[float]]) -> float:
    """计算一批静止状态读取中任一关节的最大波动。"""
    if len(samples) < 2:
        return 0.0
    values = np.asarray(samples, dtype=np.float64)
    if values.ndim != 2 or values.shape[1] != 7:
        raise ValueError("关节样本必须均含 J1-J6 和 E1 共 7 个值")
    return float(np.max(np.max(values, axis=0) - np.min(values, axis=0)))


def make_spin(low: float, high: float, value: float, step: float, suffix: str) -> QtWidgets.QDoubleSpinBox:
    spin = QtWidgets.QDoubleSpinBox()
    spin.setRange(low, high)
    spin.setValue(value)
    spin.setSingleStep(step)
    spin.setSuffix(suffix)
    return spin


def load_arm_addresses(path: str | None) -> dict[str, tuple[str, str]]:
    if path is None:
        return {}
    data = yaml.safe_load(Path(path).expanduser().read_text(encoding="utf-8"))
    if not isinstance(data, dict) or not isinstance(data.get("arms"), dict):
        raise ValueError("X5 配置需要 arms.left 和 arms.right 映射")
    addresses = {}
    for arm_id, values in data["arms"].items():
        if arm_id not in {"left", "right"} or not isinstance(values, dict):
            raise ValueError(f"无效的机械臂配置：{arm_id}")
        addresses[arm_id] = tuple(str(values.get(key) or "") for key in ("controller_ip", "source_ip"))
    return addresses


def parse_args() -> argparse.Namespace:
    default_output = REPO_ROOT / "handeye_samples"
    local_config = REPO_ROOT / "config" / "x5.local.yaml"
    parser = argparse.ArgumentParser(description="X5 机器人手眼标定图形界面")
    parser.add_argument("--speed", type=int, default=5)
    parser.add_argument(
        "--x5-config",
        default=str(local_config) if local_config.exists() else None,
        help="可选：从 YAML 读取初始地址；未传时自动读取 config/x5.local.yaml（若存在）",
    )
    parser.add_argument("--cart-frame-type", type=int, default=1, help="X5 jogl_rel 坐标系类型：1 世界，2 工具，3 用户")
    parser.add_argument("--joint-step-deg", type=float, default=1.0)
    parser.add_argument("--cart-step-mm", type=float, default=2.0)
    parser.add_argument("--rot-step-deg", type=float, default=1.0)
    parser.add_argument("--pattern-cols", type=int, default=9)
    parser.add_argument("--pattern-rows", type=int, default=6)
    parser.add_argument("--square-size-mm", type=float, default=24.0)
    parser.add_argument(
        "--handeye-mode",
        choices=("eye_in_hand", "eye_to_hand"),
        default="eye_in_hand",
        help="眼在手上输出 camera_to_tcp；眼在手外输出 camera_to_base",
    )
    parser.add_argument("--output-dir")
    parser.add_argument(
        "--recompute-samples",
        help="离线重新计算已保存的样本目录，不启动图形界面或连接机器人",
    )
    args = parser.parse_args()
    if args.output_dir is None:
        if args.recompute_samples:
            default_output = default_output / "recomputed" / f"{datetime.now():%Y%m%d_%H%M%S}_{uuid4().hex[:8]}"
        args.output_dir = str(default_output)
    if not ROS_AVAILABLE and not args.recompute_samples:
        parser.error("ROS 2 压缩相机数据源需要 rclpy 和 sensor_msgs")
    return args


def main() -> int:
    args = parse_args()
    if args.recompute_samples:
        source_dir = Path(args.recompute_samples).expanduser().resolve()
        output_dir = Path(args.output_dir).expanduser().resolve()
        samples = load_saved_samples(source_dir)
        result_path = compute_and_save_result(
            samples=samples,
            output_dir=output_dir,
            pattern_size=(args.pattern_cols, args.pattern_rows),
            square_size_mm=args.square_size_mm,
            handeye_mode=args.handeye_mode,
        )
        payload = json.loads(result_path.read_text(encoding="utf-8"))
        print(f"标定结果已保存：{result_path}")
        print(format_handeye_error_summary(payload))
        return 0
    app = QtWidgets.QApplication(sys.argv)
    try:
        window = MainWindow(args)
    except (OSError, ValueError, yaml.YAMLError) as exc:
        print(f"无法读取标定配置：{exc}", file=sys.stderr)
        return 2
    window.show()
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
