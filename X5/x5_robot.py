"""围绕原厂 X5 ``xapi`` SDK 的安全、线程串行化适配层。

在 Python 3.10 环境加载已安装的 X5 SDK。封装层通过一把
可重入锁串行化原生调用，校验工具坐标系和数字输出的控制器回读，并为 X5 USB
以太网提供 Linux 路由诊断。它不是仿真层：运动机器人或切换输出的方法会影响
真实硬件，只能在操作员确认过的流程中调用。
"""

from __future__ import annotations

import importlib
import ipaddress
import json
import math
import os
import shlex
import socket
import shutil
import subprocess
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional


DEFAULT_IP = "192.168.1.6"
XAPI_RUNTIME_DIR = Path(os.environ.get("X5_XAPI_RUNTIME_DIR", "/tmp/dualarm_handeye_xapi"))
XAPI_BIN_DIR = XAPI_RUNTIME_DIR / "bin"
DEFAULT_X5_SOURCE_IP = ""
ROUTE_LOOKUP_UDP_PORT = 9
DEFAULT_TCP_PROBE_PORTS: tuple[int, ...] = ()
_LOADED_XAPI_SOURCE = ""


class X5RobotError(RuntimeError):
    """X5 SDK 或控制器报告操作失败时抛出。"""


@dataclass(frozen=True)
class RobotPose:
    """X5 Base/TF 坐标系中的笛卡尔位姿：XYZ 为毫米，ABC 为角度。"""

    x: float
    y: float
    z: float
    a: float
    b: float
    c: float

    @classmethod
    def from_x5_point(cls, point: Any) -> "RobotPose":
        pose = point.pose if hasattr(point, "pose") else point
        return cls(
            x=float(pose.x),
            y=float(pose.y),
            z=float(pose.z),
            a=float(pose.a),
            b=float(pose.b),
            c=float(pose.c),
        )

    def as_list(self) -> list[float]:
        return [self.x, self.y, self.z, self.a, self.b, self.c]


@dataclass(frozen=True)
class CartesianPointState:
    """控制器 Point 快照；Pose.e1 是 R1，而不是 Joint.e1/J7。"""

    pose: RobotPose
    r1_deg: float
    external_axes_deg: tuple[float, float]
    cfg: tuple[int, int, int, int]

    def to_mapping(self) -> dict[str, Any]:
        return {
            "pose": self.pose.as_list(),
            "r1_deg": self.r1_deg,
            "external_axes_deg": list(self.external_axes_deg),
            "cfg": list(self.cfg),
        }


@dataclass(frozen=True)
class X5TcpProbe:
    """SDK 连接前一次可选 TCP 连通性探测的结果。"""

    port: int
    ok: bool
    elapsed_ms: int
    error: str = ""

    def summary(self) -> str:
        if self.ok:
            return f"tcp/{self.port} open ({self.elapsed_ms} ms)"
        return f"tcp/{self.port} closed: {self.error or 'unreachable'} ({self.elapsed_ms} ms)"


@dataclass(frozen=True)
class X5NetworkDiagnostics:
    """一个候选 X5 IP 的路由、源地址与可选 TCP 诊断信息。"""

    ip: str
    local_ip: Optional[str]
    same_24_subnet: Optional[bool]
    probes: tuple[X5TcpProbe, ...]
    expected_source_ip: Optional[str] = None
    source_matches: Optional[bool] = None
    route_fix_command: str = ""
    adapter_name: Optional[str] = None
    notes: tuple[str, ...] = ()

    @property
    def tcp_reachable(self) -> bool:
        return any(probe.ok for probe in self.probes)

    def summary(self) -> str:
        source = self.local_ip or "未知本地源地址"
        expected = ""
        if self.expected_source_ip:
            expected = f"，期望源地址 {self.expected_source_ip}"
            if self.source_matches is True:
                expected += "，路由正常"
            elif self.source_matches is False:
                expected += "，路由不匹配"
        subnet = ""
        if self.same_24_subnet is True:
            subnet = "，同一 /24 子网"
        elif self.same_24_subnet is False:
            subnet = "，不在同一 /24 子网"
        probes = "；".join(probe.summary() for probe in self.probes) or "未启用 TCP 探测"
        notes = f" 说明：{'；'.join(self.notes)}" if self.notes else ""
        return f"{self.ip} 经由 {source}{expected}{subnet}：{probes}。{notes}"


@dataclass(frozen=True)
class X5RouteFixResult:
    """检查或应用 X5 源地址路由规则的结果。"""

    ok: bool
    ip: str
    source_ip: Optional[str]
    interface: Optional[str]
    command: str
    message: str

    def summary(self) -> str:
        command = f" 命令：{self.command}" if self.command else ""
        return f"{self.message}{command}"


@dataclass(frozen=True)
class X5DiscoveryResult:
    """原厂发现接口返回的控制器候选 IP 与 SDK 错误。"""

    ips: tuple[str, ...]
    errors: tuple[str, ...]
    sdk: str


def _load_xapi() -> Any:
    _prepare_xapi_runtime()
    try:
        module = importlib.import_module("xapi.api")
    except (ImportError, OSError) as exc:
        raise X5RobotError(f"无法加载 X5 SDK：{exc}") from exc
    global _LOADED_XAPI_SOURCE
    _LOADED_XAPI_SOURCE = "Python 3.10 环境"
    return module


def _prepare_xapi_runtime() -> None:
    XAPI_BIN_DIR.mkdir(parents=True, exist_ok=True)
    netstat = XAPI_BIN_DIR / "netstat"
    netstat.write_text(
        "#!/usr/bin/env bash\n"
        "if command -v /usr/bin/ss >/dev/null 2>&1; then\n"
        "  echo 'Proto Recv-Q Send-Q Local Address           Foreign Address         State'\n"
        "  /usr/bin/ss -H -uan 2>/dev/null | awk '{local=$4; gsub(/\\[|\\]/, \"\", local); n=split(local, parts, \":\"); port=parts[n]; if (port ~ /^[0-9]+$/) print \"udp 0 0 0.0.0.0:\"port\" 0.0.0.0:* -\"}'\n"
        "  exit 0\n"
        "fi\n"
        "exit 0\n",
        encoding="utf-8",
    )
    netstat.chmod(0o755)
    path_parts = os.environ.get("PATH", "").split(os.pathsep)
    bin_dir = str(XAPI_BIN_DIR)
    if bin_dir not in path_parts:
        os.environ["PATH"] = bin_dir + os.pathsep + os.environ.get("PATH", "")


def _rob_exception_message(exc: Exception) -> str:
    code = getattr(exc, "error_code", None)
    symbol = getattr(exc, "error_symbol", None)
    message = getattr(exc, "error_message", None)
    if code is not None or symbol is not None or message is not None:
        return f"code={code}, symbol={symbol}, message={message}"
    return str(exc)


def _env_bool(name: str, default: bool) -> bool:
    value = os.environ.get(name)
    if value is None:
        return default
    return value.strip().lower() not in {"0", "false", "no", "off"}


def xapi_runtime_summary(x5: Optional[Any] = None) -> str:
    module = x5
    if module is None:
        module = _load_xapi()
    version = getattr(module, "VERSION", None) or getattr(module, "__version__", None) or "未知"
    path = getattr(module, "__file__", "未知路径")
    source = _LOADED_XAPI_SOURCE or "未知来源"
    return f"xapi {version}，来源：{source}（{path}）"


def diagnose_x5_network(
    ip: str,
    ports: Optional[tuple[int, ...]] = None,
    timeout: float = 0.75,
    source_ip: Optional[str] = None,
) -> X5NetworkDiagnostics:
    """检查 SDK 将使用的 Linux 路由；本函数绝不修改路由。"""

    target = ip.strip() or DEFAULT_IP
    probe_ports = ports if ports is not None else _tcp_probe_ports_from_env()
    expected_source_ip = _normalize_optional_ip(source_ip) or _expected_x5_source_ip()
    route_info = _route_info_for(target)
    local_ip = route_info.source_ip or _local_ipv4_for(target, ROUTE_LOOKUP_UDP_PORT)
    adapter_name = route_info.dev
    source_adapter = _adapter_for_ip(expected_source_ip) if expected_source_ip else None
    if expected_source_ip and not adapter_name:
        adapter_name = source_adapter
    same_24 = _same_24_subnet(local_ip, target) if local_ip else None
    source_matches = None
    if expected_source_ip and local_ip:
        source_matches = local_ip == expected_source_ip
    notes: list[str] = []
    if local_ip is None:
        notes.append("无法推断出站本地 IPv4 地址")
    elif same_24 is False and target.startswith("192.168.1."):
        notes.append("X5 默认 NET0 需要主机配置如 192.168.1.x/24 的网卡")
    if expected_source_ip and source_matches is False:
        notes.append(
            f"路由不匹配：X5 应使用源地址 {expected_source_ip}，"
            f"但 Linux 选择了 {local_ip}"
        )
    if expected_source_ip and source_matches is not True and not source_adapter:
        notes.append(f"找不到拥有 {expected_source_ip} 的本地网卡")

    probes = tuple(_probe_tcp(target, port, timeout) for port in probe_ports)
    if probes and not any(probe.ok for probe in probes):
        notes.append("SDK 连接前无法访问配置的可选 TCP 探测端口")
    route_fix_command = ""
    if expected_source_ip and source_matches is not True:
        route_fix_command = _route_fix_command(target, expected_source_ip, source_adapter)
        if route_fix_command:
            notes.append(f"可用以下命令修复路由：{route_fix_command}")
    return X5NetworkDiagnostics(
        ip=target,
        local_ip=local_ip,
        same_24_subnet=same_24,
        probes=probes,
        expected_source_ip=expected_source_ip,
        source_matches=source_matches,
        route_fix_command=route_fix_command,
        adapter_name=adapter_name,
        notes=tuple(notes),
    )


def ensure_x5_route(
    ip: str = DEFAULT_IP,
    source_ip: Optional[str] = None,
    apply: bool = False,
) -> X5RouteFixResult:
    """可选地将 X5 路由固定到配置的 USB 网卡源地址。"""

    target = ip.strip() or DEFAULT_IP
    expected_source_ip = _normalize_optional_ip(source_ip) or _expected_x5_source_ip()
    if not expected_source_ip:
        return X5RouteFixResult(
            ok=True,
            ip=target,
            source_ip=None,
            interface=None,
            command="",
            message="未配置 X5 源 IP；跳过固定路由",
        )

    before = diagnose_x5_network(target, ports=(), source_ip=expected_source_ip)
    interface = _adapter_for_ip(expected_source_ip)
    command = _route_fix_command(target, expected_source_ip, interface)
    if before.source_matches is True:
        return X5RouteFixResult(
            ok=True,
            ip=target,
            source_ip=expected_source_ip,
            interface=interface,
            command=command,
            message=f"X5 路由正常：{target} 已使用源地址 {expected_source_ip}",
        )
    if not interface:
        return X5RouteFixResult(
            ok=False,
            ip=target,
            source_ip=expected_source_ip,
            interface=None,
            command="",
            message=(
                f"X5 路由不匹配且没有网卡拥有 {expected_source_ip}。"
                "请在连接前检查 USB 以太网 IP。"
            ),
        )
    if not apply:
        return X5RouteFixResult(
            ok=False,
            ip=target,
            source_ip=expected_source_ip,
            interface=interface,
            command=command,
            message=(
                f"X5 路由不匹配：{target} 当前使用 "
                f"{before.local_ip or '未知源地址'}，期望为 {expected_source_ip}"
            ),
        )

    cmd = _route_fix_argv(target, expected_source_ip, interface)
    try:
        result = subprocess.run(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=5,
            check=False,
        )
    except Exception as exc:
        return X5RouteFixResult(
            ok=False,
            ip=target,
            source_ip=expected_source_ip,
            interface=interface,
            command=command,
            message=f"执行 X5 路由修复失败：{exc}",
        )

    after = diagnose_x5_network(target, ports=(), source_ip=expected_source_ip)
    if result.returncode == 0 and after.source_matches is True:
        return X5RouteFixResult(
            ok=True,
            ip=target,
            source_ip=expected_source_ip,
            interface=interface,
            command=command,
            message=f"X5 路由已修复：{target} 当前使用源地址 {expected_source_ip}",
        )
    detail = (result.stderr or result.stdout or "").strip()
    if result.returncode != 0:
        detail = f"路由命令退出码为 {result.returncode}：{detail}".strip()
    elif after.source_matches is not True:
        detail = f"路由仍使用 {after.local_ip or '未知源地址'}"
    return X5RouteFixResult(
        ok=False,
        ip=target,
        source_ip=expected_source_ip,
        interface=interface,
        command=command,
        message=f"X5 路由修复未生效。{detail}".strip(),
    )


def discover_x5_ips(x5: Optional[Any] = None) -> X5DiscoveryResult:
    errors: list[str] = []
    try:
        module = x5 if x5 is not None else _load_xapi()
    except X5RobotError as exc:
        return X5DiscoveryResult(ips=(), errors=(str(exc),), sdk="xapi 不可用")

    discovered: list[str] = []
    func = getattr(module, "get_ip_list", None)
    if func is not None:
        try:
            discovered.extend(_extract_ips(func()))
        except Exception as exc:
            errors.append(f"get_ip_list: {_rob_exception_message(exc)}")

    func = getattr(module, "get_ip_list_with_adapter", None)
    if func is not None:
        try:
            discovered.extend(_extract_adapter_controller_ips(func()))
        except Exception as exc:
            errors.append(f"get_ip_list_with_adapter: {_rob_exception_message(exc)}")

    for name in ("get_ip_list_by_adapter",):
        func = getattr(module, name, None)
        if func is None:
            continue
        try:
            discovered.extend(_extract_ips(func()))
        except Exception as exc:
            errors.append(f"{name}: {_rob_exception_message(exc)}")

    env_ips = os.environ.get("X5_ROBOT_IPS", "")
    if env_ips:
        discovered.extend(part.strip() for part in env_ips.replace(";", ",").split(","))

    return X5DiscoveryResult(
        ips=tuple(_dedupe(discovered)),
        errors=tuple(errors),
        sdk=xapi_runtime_summary(module),
    )


def _tcp_probe_ports_from_env() -> tuple[int, ...]:
    value = os.environ.get("X5_TCP_PORTS", "")
    ports: list[int] = []
    for part in value.replace(";", ",").split(","):
        part = part.strip()
        if not part:
            continue
        try:
            port = int(part)
        except ValueError:
            continue
        if 1 <= port <= 65535:
            ports.append(port)
    return tuple(ports)


@dataclass(frozen=True)
class _RouteInfo:
    source_ip: Optional[str] = None
    dev: Optional[str] = None


def _expected_x5_source_ip() -> Optional[str]:
    return _normalize_optional_ip(os.environ.get("X5_SOURCE_IP", DEFAULT_X5_SOURCE_IP))


def _normalize_optional_ip(value: Optional[str]) -> Optional[str]:
    if value is None:
        return None
    value = str(value).strip()
    if not value or value.lower() in {"0", "false", "none", "off", "auto"}:
        return None
    try:
        ipaddress.ip_address(value)
    except ValueError:
        return None
    return value


def _route_info_for(target_ip: str) -> _RouteInfo:
    try:
        result = subprocess.run(
            ["ip", "-4", "route", "get", target_ip],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            timeout=2,
            check=False,
        )
    except Exception:
        return _RouteInfo()
    if result.returncode != 0:
        return _RouteInfo()
    words = result.stdout.strip().split()
    source_ip = _word_after(words, "src")
    dev = _word_after(words, "dev")
    if source_ip and not _normalize_optional_ip(source_ip):
        source_ip = None
    return _RouteInfo(source_ip=source_ip, dev=dev)


def _word_after(words: list[str], marker: str) -> Optional[str]:
    try:
        index = words.index(marker)
    except ValueError:
        return None
    if index + 1 >= len(words):
        return None
    return words[index + 1]


def _adapter_for_ip(source_ip: str) -> Optional[str]:
    try:
        result = subprocess.run(
            ["ip", "-json", "-4", "addr"],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            timeout=2,
            check=False,
        )
    except Exception:
        return None
    if result.returncode != 0:
        return None
    try:
        interfaces = json.loads(result.stdout)
    except json.JSONDecodeError:
        return None
    for interface in interfaces:
        ifname = interface.get("ifname")
        for addr in interface.get("addr_info", []):
            if addr.get("family") == "inet" and addr.get("local") == source_ip:
                return str(ifname) if ifname else None
    return None


def _route_fix_argv(target_ip: str, source_ip: str, interface: Optional[str]) -> list[str]:
    command = ["ip", "route", "replace", f"{target_ip}/32"]
    if interface:
        command.extend(["dev", interface])
    command.extend(["src", source_ip, "scope", "link"])
    return command


def _route_fix_command(target_ip: str, source_ip: str, interface: Optional[str]) -> str:
    if not interface:
        return ""
    return "sudo " + " ".join(shlex.quote(part) for part in _route_fix_argv(target_ip, source_ip, interface))


def _probe_tcp(ip: str, port: int, timeout: float) -> X5TcpProbe:
    started = time.monotonic()
    try:
        with socket.create_connection((ip, int(port)), timeout=max(0.1, float(timeout))):
            elapsed = int((time.monotonic() - started) * 1000)
            return X5TcpProbe(port=int(port), ok=True, elapsed_ms=elapsed)
    except OSError as exc:
        elapsed = int((time.monotonic() - started) * 1000)
        return X5TcpProbe(port=int(port), ok=False, elapsed_ms=elapsed, error=str(exc))


def _local_ipv4_for(target_ip: str, target_port: int) -> Optional[str]:
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.connect((target_ip, int(target_port)))
            local_ip = sock.getsockname()[0]
            if local_ip and not local_ip.startswith("127."):
                return local_ip
    except OSError:
        pass
    try:
        for value in socket.gethostbyname_ex(socket.gethostname())[2]:
            if value and not value.startswith("127."):
                return value
    except OSError:
        return None
    return None


def _same_24_subnet(local_ip: Optional[str], target_ip: str) -> Optional[bool]:
    if not local_ip:
        return None
    try:
        network = ipaddress.ip_network(f"{target_ip}/24", strict=False)
        return ipaddress.ip_address(local_ip) in network
    except ValueError:
        return None


def _extract_ips(data: Any) -> list[str]:
    ips: list[str] = []
    if isinstance(data, str):
        return [data]
    if isinstance(data, dict):
        for key, value in data.items():
            if _looks_like_ip(key):
                ips.append(str(key))
            ips.extend(_extract_ips(value))
        return ips
    if isinstance(data, (list, tuple, set)):
        for item in data:
            ips.extend(_extract_ips(item))
        return ips
    ip = getattr(data, "ip", None)
    if ip:
        ips.append(str(ip))
    return ips


def _extract_adapter_controller_ips(data: Any) -> list[str]:
    if not isinstance(data, dict):
        return _extract_ips(data)
    ips: list[str] = []
    for value in data.values():
        ips.extend(_extract_ips(value))
    return ips


def _looks_like_ip(value: Any) -> bool:
    text = str(value)
    parts = text.split(".")
    if len(parts) != 4:
        return False
    try:
        return all(0 <= int(part) <= 255 for part in parts)
    except ValueError:
        return False


def _dedupe(values: list[str]) -> list[str]:
    seen: set[str] = set()
    result: list[str] = []
    for value in values:
        value = str(value).strip()
        if value and value not in seen:
            seen.add(value)
            result.append(value)
    return result


class X5Robot:
    """原厂 xapi 模块的线程安全窄封装。"""

    def __init__(
        self,
        ip: str = DEFAULT_IP,
        speed: int = 5,
        remote_state: Optional[bool] = False,
        link_timeout: int = 2,
        retry_num: int = 5,
        move_timeout_ms: int = 10000,
        auto_discover: bool = True,
        preflight_tcp: bool = False,
        tcp_probe_timeout: float = 0.75,
        source_ip: Optional[str] = None,
        auto_fix_route: bool = True,
    ) -> None:
        self.ip = ip
        self.speed = self._clamp_speed(speed)
        self.remote_state = remote_state
        self.link_timeout = max(2, int(link_timeout))
        self.retry_num = max(1, int(retry_num))
        self.move_timeout_ms = int(move_timeout_ms)
        self.auto_discover = bool(auto_discover)
        self.preflight_tcp = bool(preflight_tcp)
        self.tcp_probe_timeout = max(0.1, float(tcp_probe_timeout))
        self.source_ip = _normalize_optional_ip(source_ip) or _expected_x5_source_ip()
        self.auto_fix_route = bool(auto_fix_route)
        self._x5: Any = None
        self._handle: Optional[int] = None
        self._lock = threading.RLock()
        self.version = ""
        self.discovery_errors: list[str] = []
        self.network_diagnostics: list[str] = []

    @property
    def connected(self) -> bool:
        return self._handle is not None

    def connect(self) -> None:
        with self._lock:
            if self._handle is not None:
                return
            self._x5 = _load_xapi()
            candidates = [self.ip]
            if self.auto_discover:
                candidates.extend(self._discover_controller_ips())
            candidates = self._dedupe(candidates)
            errors: list[str] = []
            self.network_diagnostics = []
            try:
                if hasattr(self._x5, "enable_debug_output"):
                    self._x5.enable_debug_output(False)
                for ip in candidates:
                    route_result = ensure_x5_route(
                        ip,
                        source_ip=self.source_ip,
                        apply=self.auto_fix_route,
                    )
                    if route_result.source_ip:
                        self.network_diagnostics.append(route_result.summary())
                    if self.preflight_tcp:
                        diag = diagnose_x5_network(
                            ip,
                            timeout=min(float(self.link_timeout), self.tcp_probe_timeout),
                            source_ip=self.source_ip,
                        )
                        self.network_diagnostics.append(diag.summary())
                        if not diag.tcp_reachable:
                            errors.append(f"{ip}: {diag.summary()}")
                            continue
                    try:
                        self._handle = self._x5.connect(
                            ip,
                            link_timeout=self.link_timeout,
                            retry_num=self.retry_num,
                        )
                        self.ip = ip
                        self.version = self._x5.get_version(self._handle)
                        return
                    except Exception as exc:
                        self._handle = None
                        errors.append(f"{ip}: {_rob_exception_message(exc)}")
            except Exception as exc:
                self._handle = None
                self._shutdown_logger()
                raise X5RobotError(
                    f"连接 X5 控制器 {self.ip} 失败："
                    f"{_rob_exception_message(exc)}"
                ) from exc
            recent_errors = self._recent_sdk_errors()
            self._shutdown_logger()
            raise X5RobotError(
                "连接 X5 控制器失败。已尝试 "
                f"{', '.join(candidates)}。"
                f"当前使用 {xapi_runtime_summary(self._x5)}。"
                f"{self._format_network_diagnostics()} "
                f"{self._format_discovery_errors()} "
                f"错误：{'；'.join(errors)}。"
                f"{recent_errors} "
                "如果出现 XERR_SYS_COM_INIT，说明 X5 SDK 命令通道未初始化。"
                "UDP 发现仍可能正常，但 TCP 连接可能被路由到错误网卡或被控制器阻止。"
                f"请确保 Linux 将 {DEFAULT_IP} 路由至 USB 以太网源地址 "
                f"{self.source_ip or DEFAULT_X5_SOURCE_IP}；随后检查控制器通信/API 状态、"
                "是否已有其他客户端连接，以及 SDK/固件兼容性。"
            )

    def prepare(self, enable_servo: bool = True) -> None:
        """将控制器准备为低速手动键盘点动状态。"""
        with self._lock:
            self._require_connection()
            x5 = self._x5
            handle = self._handle
            try:
                state = x5.get_system_state(handle)
                if getattr(state, "alarm", 0):
                    x5.reset(handle)
                    self._wait_cmd_send_done()
                    time.sleep(0.2)
                if self.remote_state is not None:
                    state = self._set_remote_state_locked(bool(self.remote_state), state=state)
                if hasattr(x5, "set_system_mode"):
                    mode = getattr(x5, "SYSTEM_MODE_MANUAL", 3)
                    if getattr(state, "mode", None) != mode:
                        x5.set_system_mode(handle, mode)
                        self._wait_cmd_send_done()
                if enable_servo and hasattr(x5, "enable_servo"):
                    x5.enable_servo(handle, True)
                    self._wait_cmd_send_done()
                self.set_speed(self.speed)
            except Exception as exc:
                raise X5RobotError(
                    f"准备 X5 控制器失败：{_rob_exception_message(exc)}"
                ) from exc

    def prepare_manual(self, *, enable_servo: bool = False, speed: int | None = None) -> None:
        """Prepare local/manual mode without implicitly enabling the servo.

        The legacy ``prepare`` method intentionally keeps its historical
        behavior for the original single-arm workflow.  Right-arm integration
        uses this explicit method so connection and servo enable remain two
        separate operator actions.
        """
        with self._lock:
            self._require_connection()
            try:
                state = self._x5.get_system_state(self._handle)
                if getattr(state, "alarm", 0):
                    raise X5RobotError(
                        f"X5 当前存在报警（alarm={getattr(state, 'alarm', '未知')}），"
                        f"无法准备手动控制；报警详情：{self._format_system_alarm_info_locked()}"
                    )
                self._set_remote_state_locked(False, state=state)
                if hasattr(self._x5, "set_system_mode"):
                    mode = getattr(self._x5, "SYSTEM_MODE_MANUAL", 3)
                    state = self._x5.get_system_state(self._handle)
                    if int(getattr(state, "mode", -1)) != int(mode):
                        self._x5.set_system_mode(self._handle, mode)
                        self._wait_cmd_send_done()
                        self._wait_for_system_state_locked("mode", int(mode), "手动")
                if speed is not None:
                    self.set_speed(int(speed))
            except X5RobotError:
                raise
            except Exception as exc:
                raise X5RobotError(
                    f"准备 X5 手动控制失败：{_rob_exception_message(exc)}"
                ) from exc

    def set_servo_enabled(self, enabled: bool) -> None:
        """Explicitly enable/disable servo power after operator confirmation."""
        with self._lock:
            self._require_connection()
            if not hasattr(self._x5, "enable_servo"):
                raise X5RobotError("X5 SDK 未提供 enable_servo")
            try:
                self._x5.enable_servo(self._handle, bool(enabled))
                self._wait_cmd_send_done()
                self._wait_for_system_state_locked("enable", int(bool(enabled)), "伺服")
            except X5RobotError:
                raise
            except Exception as exc:
                raise X5RobotError(
                    f"设置 X5 伺服 {'使能' if enabled else '关闭'}失败：{_rob_exception_message(exc)}"
                ) from exc

    def current_system_state(self) -> dict[str, Any]:
        """Return a stable, serializable snapshot of controller state."""
        with self._lock:
            self._require_connection()
            if not hasattr(self._x5, "get_system_state"):
                raise X5RobotError("X5 SDK 未提供 get_system_state")
            try:
                state = self._x5.get_system_state(self._handle)
                result: dict[str, Any] = {
                    "alarm": int(getattr(state, "alarm", 0)),
                    "run": int(getattr(state, "run", -1)),
                    "remote": bool(getattr(state, "remote", False)),
                    "mode": int(getattr(state, "mode", -1)),
                    "enable": bool(getattr(state, "enable", False)),
                }
                if result["alarm"]:
                    result["alarm_report"] = self._format_system_alarm_info_locked()
                else:
                    result["alarm_report"] = ""
                return result
            except Exception as exc:
                raise X5RobotError(
                    f"读取 X5 控制器状态失败：{_rob_exception_message(exc)}"
                ) from exc

    def reset_alarm(self) -> dict[str, Any]:
        """Clear controller alarms when present and verify the result."""
        with self._lock:
            self._require_connection()
            if not hasattr(self._x5, "reset"):
                raise X5RobotError("X5 SDK 未提供 reset")
            try:
                self._x5.reset(self._handle)
                self._wait_cmd_send_done()
                time.sleep(0.2)
                state = self._x5.get_system_state(self._handle)
                if int(getattr(state, "alarm", 0)):
                    detail = self._format_system_alarm_info_locked()
                    raise X5RobotError(
                        f"X5 复位报警后仍存在报警（alarm={getattr(state, 'alarm', '未知')}）；报警详情：{detail}"
                    )
                # 与 handle_dual_arm.py 保持一致：复位后先恢复 AutoCMD
                # 并重新上使能，单独 reset 在部分控制器上不会清理通信状态。
                self.set_auto_cmd_mode()
                time.sleep(1.0)
                state = self._x5.get_system_state(self._handle)
                if int(getattr(state, "alarm", 0)) or not bool(getattr(state, "enable", False)):
                    raise X5RobotError(
                        "X5 报警已清除但仍未达到可运动状态："
                        f"mode={getattr(state, 'mode', '未知')}，"
                        f"enable={getattr(state, 'enable', '未知')}，"
                        f"remote={getattr(state, 'remote', '未知')}"
                    )
                result = {
                    "alarm": int(getattr(state, "alarm", 0)),
                    "run": int(getattr(state, "run", -1)),
                    "remote": bool(getattr(state, "remote", False)),
                    "mode": int(getattr(state, "mode", -1)),
                    "enable": bool(getattr(state, "enable", False)),
                }
                return result
            except X5RobotError:
                raise
            except Exception as exc:
                raise X5RobotError(
                    f"X5 复位报警失败：{_rob_exception_message(exc)}"
                ) from exc

    def set_remote_state(self, is_remote: bool) -> None:
        """手动点动前设置并校验控制器远程/本地状态。"""
        with self._lock:
            self._require_connection()
            try:
                self._set_remote_state_locked(bool(is_remote))
            except X5RobotError:
                raise
            except Exception as exc:
                raise X5RobotError(
                    f"设置 X5 远程状态为 {int(bool(is_remote))} 失败："
                    f"{_rob_exception_message(exc)}"
                ) from exc

    def set_auto_cmd_mode(self) -> int:
        """切换至自动命令模式并确认伺服使能，可安全下发自动运动。"""

        with self._lock:
            self._require_system_mode_interfaces_locked()
            # X5 的上使能命令只能在本地状态下执行。自动抓取沿用 GUI
            # 手动控制的本地状态，防止控制器静默拒绝上使能。
            self._set_remote_state_locked(False)
            self._require_stopped_and_alarm_free_locked("切换自动 CMD")
            mode, mode_changed = self._set_system_mode(
                "SYSTEM_MODE_AUTOCMD", 100, "自动 CMD"
            )
            if mode_changed:
                # 原厂时序要求模式切换完成后等待控制器内部状态稳定。
                time.sleep(1.0)
            # 模式转换会使先前的 enable 状态失效。即使刚读到 enable=1，
            # 也要按原厂自动 CMD 时序重新上使能并留出稳定时间。
            self._ensure_servo_enabled_locked(force=mode_changed, expected_mode=mode)
            return mode

    def set_manual_mode(self) -> int:
        """切换至手动模式，供本地点动和人工操作使用。"""

        with self._lock:
            self._require_system_mode_interfaces_locked()
            self._set_remote_state_locked(False)
            self._require_stopped_and_alarm_free_locked("切换手动")
            mode, mode_changed = self._set_system_mode("SYSTEM_MODE_MANUAL", 3, "手动")
            if mode_changed:
                time.sleep(1.0)
            self._ensure_servo_enabled_locked(force=mode_changed, expected_mode=mode)
            return mode

    def ensure_auto_cmd_motion_ready(self) -> None:
        """确认自动 CMD 的完整前置状态，供自动模式启动时调用一次。"""

        with self._lock:
            self._require_connection()
            if not hasattr(self._x5, "get_system_state"):
                raise X5RobotError("X5 SDK 未提供 get_system_state；无法确认自动运动状态")
            state = self._x5.get_system_state(self._handle)
            mode = int(getattr(self._x5, "SYSTEM_MODE_AUTOCMD", 100))
            if (
                int(getattr(state, "mode", -1)) == mode
                and bool(getattr(state, "enable", False))
                and not bool(getattr(state, "remote", False))
                and not bool(getattr(state, "alarm", False))
            ):
                return
            alarm_detail = ""
            if bool(getattr(state, "alarm", False)):
                alarm_detail = f"；报警详情：{self._format_system_alarm_info_locked()}"
            raise X5RobotError(
                "X5 自动 CMD 未就绪："
                f"mode={getattr(state, 'mode', '未知')}（期望 {mode}），"
                f"enable={getattr(state, 'enable', '未知')}（期望 1），"
                f"remote={getattr(state, 'remote', '未知')}（期望 0），"
                f"alarm={getattr(state, 'alarm', '未知')}（期望 0）"
                f"{alarm_detail}"
            )

    def get_active_system_alarm_report(self) -> str | None:
        """只读检查当前是否报警；报警时返回包含详细报警码的报告。

        自动模式的低频监控使用此接口。它不改变控制器状态，且仅当
        ``SystemState.alarm`` 非零时才读取报警队列。
        """

        with self._lock:
            self._require_connection()
            if not hasattr(self._x5, "get_system_state"):
                raise X5RobotError("X5 SDK 未提供 get_system_state；无法监控自动模式报警")
            try:
                state = self._x5.get_system_state(self._handle)
            except Exception as exc:
                raise X5RobotError(
                    f"读取 X5 自动模式状态失败：{_rob_exception_message(exc)}"
                ) from exc
            if not bool(getattr(state, "alarm", False)):
                return None
            return (
                "X5 控制器报警："
                f"mode={getattr(state, 'mode', '未知')}，"
                f"enable={getattr(state, 'enable', '未知')}，"
                f"remote={getattr(state, 'remote', '未知')}，"
                f"alarm={getattr(state, 'alarm', '未知')}；"
                f"报警详情：{self._format_system_alarm_info_locked()}"
            )

    def get_system_alarm_report(self) -> str:
        """只读获取控制器当前的详细报警报告。

        ``get_system_alarm_info`` 会访问控制器数据库，因此自动流程仅在已经
        检测到 ``SystemState.alarm`` 非零时调用它。本公开方法供人工诊断使用。
        """

        with self._lock:
            self._require_connection()
            return self._format_system_alarm_info_locked()

    def _require_system_mode_interfaces_locked(self) -> None:
        if not hasattr(self._x5, "get_system_state") or not hasattr(self._x5, "set_system_mode"):
            raise X5RobotError("X5 SDK 未提供 get_system_state/set_system_mode；无法切换系统模式")

    def _set_system_mode(
        self, sdk_constant: str, fallback: int, label: str
    ) -> tuple[int, bool]:
        """按需切换控制器系统模式，并等待 SDK 确认命令已发送。"""

        with self._lock:
            self._require_connection()
            x5 = self._x5
            handle = self._handle
            self._require_system_mode_interfaces_locked()
            mode = int(getattr(x5, sdk_constant, fallback))
            try:
                state = x5.get_system_state(handle)
                mode_changed = getattr(state, "mode", None) != mode
                if mode_changed:
                    x5.set_system_mode(handle, mode)
                    self._wait_cmd_send_done()
                self._wait_for_system_state_locked("mode", mode, label)
                return mode, mode_changed
            except Exception as exc:
                if isinstance(exc, X5RobotError):
                    raise
                raise X5RobotError(
                    f"切换 X5 至{label}模式失败：{_rob_exception_message(exc)}"
                ) from exc

    def _ensure_servo_enabled_locked(
        self, *, force: bool = False, expected_mode: Optional[int] = None
    ) -> None:
        """在已持有 ``_lock`` 时使能并确认伺服状态。"""

        x5 = self._x5
        handle = self._handle
        if not hasattr(x5, "enable_servo"):
            raise X5RobotError("X5 SDK 未提供 enable_servo；无法确认自动模式伺服使能")
        try:
            state = x5.get_system_state(handle)
            if bool(getattr(state, "remote", False)):
                raise X5RobotError("X5 当前为远程状态，无法上使能；请切换至本地状态")
            if force or not bool(getattr(state, "enable", False)):
                x5.enable_servo(handle, True)
                self._wait_cmd_send_done()
                # 原厂自动 CMD 示例要求上使能后等待 1 秒；只等待命令发送
                # 完成并不等于伺服电源已稳定。
                time.sleep(1.0)
            self._wait_for_system_state_locked("enable", 1, "伺服上使能")
            state = x5.get_system_state(handle)
            if bool(getattr(state, "remote", False)):
                raise X5RobotError("X5 上使能后切换为远程状态，自动运动不可执行")
            if expected_mode is not None and int(getattr(state, "mode", -1)) != int(expected_mode):
                raise X5RobotError(
                    "X5 上使能后模式发生变化："
                    f"期望 mode={int(expected_mode)}，实际 mode={getattr(state, 'mode', '未知')}"
                )
        except Exception as exc:
            if isinstance(exc, X5RobotError):
                raise
            raise X5RobotError(
                f"启用 X5 自动模式伺服失败：{_rob_exception_message(exc)}"
            ) from exc

    def _wait_for_system_state_locked(self, field: str, expected: int, label: str) -> Any:
        """在已持有 ``_lock`` 时等待控制器状态字段切换完成。"""

        state = None
        for attempt in range(10):
            state = self._x5.get_system_state(self._handle)
            if int(getattr(state, field, -1)) == int(expected):
                return state
            if attempt < 9:
                time.sleep(0.1)
        actual = getattr(state, field, "未知")
        raise X5RobotError(
            f"X5 {label}状态未确认：期望 {field}={int(expected)}，实际 {field}={actual}"
        )

    def _require_stopped_and_alarm_free_locked(self, action: str) -> None:
        """切换系统模式前确认机械臂已停止且无报警。"""

        state = self._x5.get_system_state(self._handle)
        if getattr(state, "alarm", 0):
            raise X5RobotError(
                f"X5 当前存在报警（alarm={getattr(state, 'alarm', '未知')}），无法{action}；"
                f"报警详情：{self._format_system_alarm_info_locked()}"
            )
        if int(getattr(state, "run", 3)) != 3:
            raise X5RobotError(
                f"X5 当前未停止（run={getattr(state, 'run', '未知')}），无法{action}"
            )

    def _format_system_alarm_info_locked(self) -> str:
        """格式化 SDK 报警队列；调用方必须已持有 ``_lock``。"""

        if not hasattr(self._x5, "get_system_alarm_info"):
            return "X5 SDK 未提供 get_system_alarm_info"
        try:
            records = self._x5.get_system_alarm_info(self._handle)
        except Exception as exc:
            return f"读取失败：{_rob_exception_message(exc)}"
        if not records:
            return "控制器未返回详细报警"

        if isinstance(records, dict):
            entries = [records]
        elif isinstance(records, (list, tuple)):
            entries = list(records)
        else:
            return str(records)

        formatted: list[str] = []
        for index, record in enumerate(entries[:8], start=1):
            if isinstance(record, dict):
                fields = [
                    ("level", "级别"),
                    ("type", "类型"),
                    ("code", "代码"),
                    ("content", "内容"),
                    ("reason", "原因"),
                    ("solution", "处理"),
                ]
                details = [
                    f"{label}={record[key]}"
                    for key, label in fields
                    if record.get(key) not in (None, "")
                ]
                formatted.append(f"#{index} " + ("，".join(details) or str(record)))
            else:
                formatted.append(f"#{index} {record}")
        if len(entries) > len(formatted):
            formatted.append(f"其余 {len(entries) - len(formatted)} 条未展开")
        return "；".join(formatted)

    def _set_remote_state_locked(self, is_remote: bool, state: Any = None) -> Any:
        x5 = self._x5
        handle = self._handle
        if not hasattr(x5, "set_remote"):
            raise X5RobotError("X5 SDK 未提供 set_remote；无法校验本地手动状态")
        current = state if state is not None else x5.get_system_state(handle)
        if bool(getattr(current, "remote", False)) != bool(is_remote):
            x5.set_remote(handle, bool(is_remote))
            self._wait_cmd_send_done()

        # 控制器异步切换模式；在 get_system_state 确认所需远程/本地状态前，
        # 不得发送手动控制指令。
        for _ in range(10):
            current = x5.get_system_state(handle)
            if bool(getattr(current, "remote", False)) == bool(is_remote):
                return current
            time.sleep(0.1)
        expected = "远程" if is_remote else "本地"
        actual = int(bool(getattr(current, "remote", False)))
        raise X5RobotError(
            f"X5 控制器未切换至{expected}状态；remote={actual}。"
            "请将控制器远程/本地模式设为本地后重新连接。"
        )

    def set_speed(self, speed: int) -> None:
        with self._lock:
            self._require_connection()
            self.speed = self._clamp_speed(speed)
            try:
                if hasattr(self._x5, "set_speed"):
                    self._x5.set_speed(self._handle, self.speed)
                    self._wait_cmd_send_done()
            except Exception as exc:
                raise X5RobotError(
                    f"设置 X5 速度 {self.speed}% 失败：{_rob_exception_message(exc)}"
                ) from exc

    def get_speed(self) -> int:
        with self._lock:
            self._require_connection()
            try:
                if hasattr(self._x5, "get_speed"):
                    self.speed = self._clamp_speed(self._x5.get_speed(self._handle))
                return self.speed
            except Exception as exc:
                raise X5RobotError(f"读取 X5 速度失败：{_rob_exception_message(exc)}") from exc

    def get_digital_output(self, index: int) -> bool:
        """读取一个控制器数字输出（DO0-DO247）。"""
        with self._lock:
            self._require_connection()
            do_index = self._check_digital_output_index(index)
            if not hasattr(self._x5, "get_do"):
                raise X5RobotError("X5 SDK 未提供 get_do")
            try:
                return bool(int(self._x5.get_do(self._handle, do_index)))
            except Exception as exc:
                raise X5RobotError(
                    f"读取 X5 DO{do_index} 失败：{_rob_exception_message(exc)}"
                ) from exc

    def get_digital_input(self, index: int) -> bool:
        """读取一个控制器数字输入（DI0-DI247）。"""
        with self._lock:
            self._require_connection()
            di_index = self._check_digital_input_index(index)
            if not hasattr(self._x5, "get_di"):
                raise X5RobotError("X5 SDK 未提供 get_di")
            try:
                return bool(int(self._x5.get_di(self._handle, di_index)))
            except Exception as exc:
                raise X5RobotError(
                    f"读取 X5 DI{di_index} 失败：{_rob_exception_message(exc)}"
                ) from exc

    def set_digital_output(self, index: int, enabled: bool) -> bool:
        """设置一个数字输出并返回已校验的控制器状态。"""
        with self._lock:
            self._require_connection()
            do_index = self._check_digital_output_index(index)
            if not hasattr(self._x5, "set_do") or not hasattr(self._x5, "get_do"):
                raise X5RobotError("X5 SDK 未提供 set_do/get_do")
            expected = bool(enabled)
            try:
                self._x5.set_do(self._handle, do_index, int(expected))
                if hasattr(self._x5, "wait_set_do_done"):
                    self._x5.wait_set_do_done(self._handle, self.move_timeout_ms)
                else:
                    self._wait_cmd_send_done()
                actual = bool(int(self._x5.get_do(self._handle, do_index)))
            except Exception as exc:
                raise X5RobotError(
                    f"设置 X5 DO{do_index}={int(expected)} 失败：{_rob_exception_message(exc)}"
                ) from exc
            if actual != expected:
                raise X5RobotError(
                    f"X5 DO{do_index} 校验失败：期望={int(expected)}，实际={int(actual)}"
                )
            return actual

    def get_tool_frame(self, index: int) -> RobotPose:
        """读取一个控制器工具坐标系（TF0-TF16）。"""
        with self._lock:
            self._require_connection()
            tf_index = self._check_tool_frame_index(index, allow_zero=True)
            if not hasattr(self._x5, "get_tf"):
                raise X5RobotError("X5 SDK 未提供 get_tf")
            try:
                return RobotPose.from_x5_point(self._x5.get_tf(self._handle, tf_index))
            except Exception as exc:
                raise X5RobotError(
                    f"读取 X5 TF{tf_index} 失败：{_rob_exception_message(exc)}"
                ) from exc

    def get_active_tool_frame(self) -> tuple[int, RobotPose]:
        """读取当前活动工具坐标系编号及其位姿。"""
        with self._lock:
            self._require_connection()
            if not hasattr(self._x5, "get_tfno"):
                raise X5RobotError("X5 SDK 未提供 get_tfno")
            try:
                tf_index = self._check_tool_frame_index(
                    int(self._x5.get_tfno(self._handle)), allow_zero=True
                )
                pose = RobotPose.from_x5_point(self._x5.get_tf(self._handle, tf_index))
                return tf_index, pose
            except Exception as exc:
                if isinstance(exc, X5RobotError):
                    raise
                raise X5RobotError(
                    f"读取当前 X5 工具坐标系失败：{_rob_exception_message(exc)}"
                ) from exc

    def set_active_tool_frame(self, index: int) -> int:
        """激活 TF0-TF16 并校验选中的坐标系编号。"""
        with self._lock:
            self._require_connection()
            tf_index = self._check_tool_frame_index(index, allow_zero=True)
            if not hasattr(self._x5, "set_tfno") or not hasattr(self._x5, "get_tfno"):
                raise X5RobotError("X5 SDK 未提供 set_tfno/get_tfno")
            try:
                self._x5.set_tfno(self._handle, tf_index)
                self._wait_cmd_send_done()
                # 部分控制器会在 get_tfno 刷新前确认指令，因此短暂等待选中的
                # 坐标系编号出现。
                actual = int(self._x5.get_tfno(self._handle))
                deadline = time.monotonic() + 1.0
                while actual != tf_index and time.monotonic() < deadline:
                    time.sleep(0.05)
                    actual = int(self._x5.get_tfno(self._handle))
            except Exception as exc:
                raise X5RobotError(
                    f"激活 X5 TF{tf_index} 失败：{_rob_exception_message(exc)}"
                ) from exc
            if actual != tf_index:
                raise X5RobotError(
                    f"X5 工具坐标系校验失败：期望 TF{tf_index}，实际 TF{actual}"
                )
            return actual

    def set_tool_frame(
        self,
        index: int,
        pose: RobotPose,
        *,
        activate: bool = True,
    ) -> RobotPose:
        """写入 TF1-TF16，可选择立即激活，并返回控制器回读值。"""
        with self._lock:
            self._require_connection()
            tf_index = self._check_tool_frame_index(index, allow_zero=False)
            if not isinstance(pose, RobotPose):
                raise X5RobotError("工具坐标系位姿必须是 RobotPose")
            if not hasattr(self._x5, "Pose") or not hasattr(self._x5, "set_tf"):
                raise X5RobotError("X5 SDK 未提供 Pose/set_tf")
            try:
                sdk_pose = self._x5.Pose(*pose.as_list(), 0.0, 0.0, 0.0)
                self._x5.set_tf(self._handle, tf_index, sdk_pose)
                self._wait_cmd_send_done()
                readback = RobotPose.from_x5_point(self._x5.get_tf(self._handle, tf_index))
            except Exception as exc:
                raise X5RobotError(
                    f"设置 X5 TF{tf_index} 失败：{_rob_exception_message(exc)}"
                ) from exc
            if activate:
                # 控制器确认前，不得将请求的 TF 报告为已激活。
                self.set_active_tool_frame(tf_index)
            return readback

    def disconnect(self, servo_off: bool = False) -> None:
        """关闭 SDK 会话；除非明确请求，否则保持伺服使能状态不变。"""

        with self._lock:
            if self._handle is None or self._x5 is None:
                return
            try:
                if servo_off and hasattr(self._x5, "enable_servo"):
                    self._x5.enable_servo(self._handle, False)
                    self._wait_cmd_send_done()
                self._x5.disconnect(self._handle)
            finally:
                self._handle = None

    def current_tcp_pose(self, world: bool = True) -> RobotPose:
        """读取当前 TCP 的 Base 坐标（或控制器坐标）。"""

        with self._lock:
            self._require_connection()
            try:
                if world:
                    point = self._x5.get_wpoint(self._handle)
                else:
                    point = self._x5.get_cpoint(self._handle)
                return RobotPose.from_x5_point(point)
            except Exception as exc:
                raise X5RobotError(
                    f"读取 TCP 位姿失败：{_rob_exception_message(exc)}"
                ) from exc

    @staticmethod
    def _point_state(point: Any) -> CartesianPointState:
        pose = point.pose if hasattr(point, "pose") else point
        cfg = getattr(point, "cfg", None)
        try:
            cfg_values = tuple(int(value) for value in cfg)
        except (TypeError, ValueError):
            raise X5RobotError("X5 SDK Point 未提供有效 cfg")
        if len(cfg_values) != 4:
            raise X5RobotError("X5 SDK Point cfg 必须包含四个值")
        try:
            extras = tuple(float(getattr(pose, name)) for name in ("e1", "e2", "e3"))
        except (AttributeError, TypeError, ValueError) as exc:
            raise X5RobotError("X5 SDK Point 未提供 R1/E2/E3") from exc
        if not all(math.isfinite(value) for value in extras):
            raise X5RobotError("X5 SDK Point 的 R1/E2/E3 不是有限数")
        return CartesianPointState(
            pose=RobotPose.from_x5_point(point),
            r1_deg=extras[0],
            external_axes_deg=(extras[1], extras[2]),
            cfg=cfg_values,
        )

    def current_cartesian_point(self, world: bool = True) -> dict[str, Any]:
        """读取完整 Point；返回可跨 HTTP bridge 传输的明确字段。"""
        with self._lock:
            self._require_connection()
            try:
                point = self._x5.get_wpoint(self._handle) if world else self._x5.get_cpoint(self._handle)
                return self._point_state(point).to_mapping()
            except X5RobotError:
                raise
            except Exception as exc:
                raise X5RobotError(f"读取完整 Point 失败：{_rob_exception_message(exc)}") from exc

    def current_joint(self) -> list[float]:
        """读取当前 J1-J6 及七轴机 E1 角度值。"""

        with self._lock:
            self._require_connection()
            try:
                joint = self._x5.get_cjoint(self._handle)
                values = [float(getattr(joint, name, 0.0)) for name in ("j1", "j2", "j3", "j4", "j5", "j6", "e1")]
                if hasattr(joint, "e2") or hasattr(joint, "e3"):
                    values.extend([float(getattr(joint, "e2", 0.0)), float(getattr(joint, "e3", 0.0))])
                return values
            except Exception as exc:
                raise X5RobotError(
                    f"读取关节位姿失败：{_rob_exception_message(exc)}"
                ) from exc

    def current_manual_state(self) -> tuple[list[float], RobotPose, int]:
        """在一次 SDK 加锁中读取关节、世界/Base TCP 位姿和全局速度。"""
        with self._lock:
            self._require_connection()
            try:
                joint = self._x5.get_cjoint(self._handle)
                joints = [
                    float(getattr(joint, name, 0.0))
                    for name in ("j1", "j2", "j3", "j4", "j5", "j6", "e1")
                ]
                point = self._x5.get_wpoint(self._handle)
                pose = RobotPose.from_x5_point(point)
                if hasattr(self._x5, "get_speed"):
                    self.speed = self._clamp_speed(self._x5.get_speed(self._handle))
                return joints, pose, self.speed
            except Exception as exc:
                raise X5RobotError(
                    f"读取 X5 手动状态失败：{_rob_exception_message(exc)}"
                ) from exc

    def jog_joint_rel(self, axis: int, delta_deg: float, wait: bool = True) -> None:
        """让一个关节做小幅相对运动；调用方必须选择安全速度。"""

        with self._lock:
            self._require_connection()
            self._check_joint_axis(axis)
            try:
                self._x5.jogj_rel(self._handle, axis, float(delta_deg))
                self._wait_cmd_send_done()
                if wait:
                    self._wait_move_done()
            except Exception as exc:
                raise X5RobotError(
                    f"关节 J{axis + 1} 点动失败：{_rob_exception_message(exc)}"
                ) from exc

    def jog_joint_start(self, axis: int, positive: bool) -> None:
        """启动 SDK 连续手动关节点动；调用 abort() 停止。"""
        with self._lock:
            self._require_connection()
            self._check_joint_axis(axis)
            try:
                direction = 0 if bool(positive) else 1
                self._x5.jogj(self._handle, axis, direction)
                self._wait_cmd_send_done()
            except Exception as exc:
                raise X5RobotError(
                    f"关节 J{axis + 1} 连续点动失败：{_rob_exception_message(exc)}"
                ) from exc

    def jog_cartesian_rel(
        self,
        axis: int,
        delta: float,
        frame_type: int = 1,
        wait: bool = True,
    ) -> None:
        """在选定 SDK 坐标系中让一个笛卡尔轴做相对运动。"""

        with self._lock:
            self._require_connection()
            self._check_axis(axis)
            try:
                self._x5.jogl_rel(self._handle, axis, float(delta), type=int(frame_type))
                self._wait_cmd_send_done()
                if wait:
                    self._wait_move_done()
            except Exception as exc:
                raise X5RobotError(
                    f"笛卡尔轴 {'XYZABC'[axis]} 点动失败：{_rob_exception_message(exc)}"
                ) from exc

    def jog_cartesian_start(self, axis: int, positive: bool, frame_type: int = 1) -> None:
        """在指定坐标系中启动 SDK 连续笛卡尔点动。"""
        with self._lock:
            self._require_connection()
            self._check_axis(axis)
            try:
                direction = 0 if bool(positive) else 1
                self._x5.jogl(self._handle, axis, direction, type=int(frame_type))
                self._wait_cmd_send_done()
            except Exception as exc:
                raise X5RobotError(
                    f"笛卡尔轴 {'XYZABC'[axis]} 连续点动失败：{_rob_exception_message(exc)}"
                ) from exc

    @staticmethod
    def _external_axes_target(values):
        try:
            result = [float(v) for v in values]
            if len(result) == 3 and all(math.isfinite(v) for v in result):
                return result
        except (TypeError, ValueError):
            pass
        raise X5RobotError("运动缺少明确的 E1/E2/E3 目标，拒绝下发")

    def move_joint_abs(self, joints_deg: list[float] | tuple[float, ...], *, joint_external_axes_deg=None, external_axes_deg=None, wait: bool = False) -> None:
        """提交角度单位的绝对关节 MovJ 目标。"""
        values = self._validate_six_finite_values(joints_deg, "关节目标")
        with self._lock:
            self._require_connection()
            if not hasattr(self._x5, "Joint") or not hasattr(self._x5, "movj"):
                raise X5RobotError("X5 SDK 未提供 Joint/movj")
            try:
                current = self._x5.get_cjoint(self._handle)
                extra = self._external_axes_target(
                    joint_external_axes_deg if joint_external_axes_deg is not None else external_axes_deg
                )
                target = self._x5.Joint(*values, *extra)
                self._x5.movj(self._handle, target)
                self._wait_cmd_send_done()
            except Exception as exc:
                raise X5RobotError(
                    f"绝对关节 MovJ 失败：{_rob_exception_message(exc)}"
                ) from exc
        if wait:
            self.wait_for_move_done()

    def move_cartesian_abs(
        self,
        pose: RobotPose | list[float] | tuple[float, ...],
        *,
        linear: bool,
        r1_deg=None,
        point_external_axes_deg=None,
        cfg=None,
        external_axes_deg=None,
        expected_tf: Optional[int] = None,
        wait: bool = False,
        validate_ik: bool = False,
    ) -> int:
        """提交绝对 Base TCP 目标，并保留当前逆解构型。"""
        values = pose.as_list() if isinstance(pose, RobotPose) else pose
        values = self._validate_six_finite_values(values, "笛卡尔目标")
        with self._lock:
            self._require_connection()
            if not all(hasattr(self._x5, name) for name in ("Pose", "Point", "get_tfno")):
                raise X5RobotError("X5 SDK 未提供 Pose/Point/get_tfno")
            method_name = "movl" if linear else "movj"
            if not hasattr(self._x5, method_name):
                raise X5RobotError(f"X5 SDK 未提供 {method_name}")
            try:
                active_tf = self._check_tool_frame_index(
                    int(self._x5.get_tfno(self._handle)), allow_zero=True
                )
                if expected_tf is not None and active_tf != int(expected_tf):
                    raise X5RobotError(
                        f"活动 TF 已从 TF{int(expected_tf)} 变为 TF{active_tf}；"
                        "运动前请重新读取当前数值"
                    )
                try:
                    legacy = [float(v) for v in external_axes_deg] if external_axes_deg is not None else None
                    r1 = float(r1_deg if r1_deg is not None else legacy[0])
                    point_extra = [float(v) for v in (point_external_axes_deg if point_external_axes_deg is not None else legacy[1:])]
                    cfg_values = tuple(int(v) for v in (cfg if cfg is not None else getattr(self._x5.get_wpoint(self._handle), "cfg", ())))
                except (TypeError, ValueError):
                    raise X5RobotError("笛卡尔目标缺少明确的 R1、外置轴或 cfg")
                if not math.isfinite(r1) or len(point_extra) != 2 or not all(math.isfinite(v) for v in point_extra):
                    raise X5RobotError("笛卡尔目标缺少明确的 R1、外置轴或 cfg")
                if len(cfg_values) != 4:
                    raise X5RobotError("笛卡尔目标 cfg 必须包含四个值")
                extra = [r1, *point_extra]
                target_pose = self._x5.Pose(*values, *extra)
                target = self._x5.Point(target_pose, 0, active_tf, cfg_values)
                if validate_ik:
                    if not hasattr(self._x5, "cnvrt_j"):
                        raise X5RobotError("SDK 不支持 cnvrt_j，未执行运动")
                    try:
                        reference = self._x5.get_cjoint(self._handle)
                        self._x5.cnvrt_j(self._handle, target, 1, reference)
                    except Exception as exc:
                        raise X5RobotError(f"目标逆解检查失败，未下发运动：{_rob_exception_message(exc)}") from exc
                getattr(self._x5, method_name)(self._handle, target)
                self._wait_cmd_send_done()
            except X5RobotError:
                raise
            except Exception as exc:
                raise X5RobotError(
                    f"绝对笛卡尔 {method_name.upper()} 失败：{_rob_exception_message(exc)}"
                ) from exc
        if wait:
            self.wait_for_move_done()
            state = self.current_system_state() if hasattr(self._x5, "get_system_state") else {}
            if state.get("alarm"):
                raise X5RobotError(
                    "运动等待返回但控制器出现报警：" + (self.get_active_system_alarm_report() or "未提供报警详情")
                )
        return active_tf

    def move_cartesian_via_abs(
        self,
        via_pose: RobotPose | list[float] | tuple[float, ...],
        target_pose: RobotPose | list[float] | tuple[float, ...],
        *,
        expected_tf: Optional[int] = None,
        cnt: int,
        r1_deg=None,
        point_external_axes_deg=None,
        cfg=None,
        external_axes_deg=None,
        speed: Optional[int] = None,
        wait: bool = False,
    ) -> int:
        """以连续 MovL 经由一个 CNT 中转点到最终 Fine 目标。

        两条运动命令在同一次封装锁内提交，避免其他 SDK 调用插入两段之间。
        ``via_pose`` 使用 ``cnt`` 连续掠过，``target_pose`` 使用 Fine（``cnt=-1``）
        精确到位；仅在最终目标上等待运动完成。
        """

        via_values = via_pose.as_list() if isinstance(via_pose, RobotPose) else via_pose
        target_values = target_pose.as_list() if isinstance(target_pose, RobotPose) else target_pose
        via_values = self._validate_six_finite_values(via_values, "CNT 中转笛卡尔目标")
        target_values = self._validate_six_finite_values(target_values, "CNT 终点笛卡尔目标")
        try:
            cnt_value = int(cnt)
        except (TypeError, ValueError) as exc:
            raise X5RobotError("CNT 必须为 1-100 的整数") from exc
        if not 1 <= cnt_value <= 100:
            raise X5RobotError(f"CNT 超出范围：{cnt_value}；期望为 1-100")
        point_speed = self._clamp_speed(self.speed if speed is None else speed)

        with self._lock:
            self._require_connection()
            x5 = self._x5
            required = ("Pose", "Point", "get_tfno", "get_wpoint", "MovPointAdd", "movl")
            if not all(callable(getattr(x5, name, None)) for name in required):
                raise X5RobotError("X5 SDK 未提供 CNT 连续 MovL 所需的 Pose/Point/MovPointAdd/movl 接口")
            try:
                active_tf = self._check_tool_frame_index(
                    int(x5.get_tfno(self._handle)), allow_zero=True
                )
                if expected_tf is not None and active_tf != int(expected_tf):
                    raise X5RobotError(
                        f"活动 TF 已从 TF{int(expected_tf)} 变为 TF{active_tf}；"
                        "运动前请重新读取当前数值"
                    )
                try:
                    legacy = [float(v) for v in external_axes_deg] if external_axes_deg is not None else None
                    extra = [
                        float(r1_deg if r1_deg is not None else legacy[0]),
                        *[float(v) for v in (point_external_axes_deg if point_external_axes_deg is not None else legacy[1:])],
                    ]
                    cfg_values = tuple(int(v) for v in (cfg if cfg is not None else getattr(x5.get_wpoint(self._handle), "cfg", ())))
                except (TypeError, ValueError):
                    raise X5RobotError("CNT 笛卡尔目标缺少明确的 R1、外置轴或 cfg")
                if len(extra) != 3 or not all(math.isfinite(v) for v in extra) or len(cfg_values) != 4:
                    raise X5RobotError("CNT 笛卡尔目标缺少明确的 R1、外置轴或 cfg")

                def build_point(values: list[float]) -> Any:
                    pose = x5.Pose(*values, *extra)
                    return x5.Point(pose, 0, active_tf, cfg_values)

                via_add = x5.MovPointAdd(
                    vel=point_speed,
                    cnt=cnt_value,
                    acc=point_speed,
                    dec=point_speed,
                )
                target_add = x5.MovPointAdd(
                    vel=point_speed,
                    cnt=-1,
                    acc=point_speed,
                    dec=point_speed,
                )
                via_point = build_point(via_values)
                target_point = build_point(target_values)
                x5.movl(self._handle, via_point, via_add)
                self._wait_cmd_send_done()
                x5.movl(self._handle, target_point, target_add)
                self._wait_cmd_send_done()
            except X5RobotError:
                raise
            except Exception as exc:
                raise X5RobotError(
                    "CNT 连续 MovL 失败："
                    f"{_rob_exception_message(exc)}"
                ) from exc
        if wait:
            self.wait_for_move_done()
            state = self.current_system_state() if hasattr(self._x5, "get_system_state") else {}
            if state.get("alarm"):
                raise X5RobotError(
                    "运动等待返回但控制器出现报警：" + (self.get_active_system_alarm_report() or "未提供报警详情")
                )
        return active_tf

    def wait_for_move_done(
        self,
        *,
        timeout_ms: Optional[int] = None,
        cleared_is_done: bool = False,
    ) -> None:
        """等待运动完成时不占用封装锁，以便另一线程可调用 abort()。"""
        with self._lock:
            self._require_connection()
            x5 = self._x5
            handle = self._handle
            # XAPI 用 -1 表示无限等待。自动流程保留该值，避免长距离 MovL
            # 仅因耗时较长而失败。
            timeout = self.move_timeout_ms if timeout_ms is None else int(timeout_ms)
            if timeout < -1:
                timeout = -1

        # XAPI 3.1.11 的 Python 接口暴露 get_cur_move_state，但原生库缺少
        # XGetCurMovState；优先使用可靠的 SDK 等待接口。
        if hasattr(x5, "wait_move_done"):
            try:
                x5.wait_move_done(handle, timeout)
                return
            except Exception as exc:
                raise X5RobotError(
                    f"等待 X5 运动完成失败：{_rob_exception_message(exc)}"
                ) from exc

        if not hasattr(x5, "get_cur_move_state"):
            return

        deadline = None if timeout < 0 else time.monotonic() + timeout / 1000.0
        while True:
            try:
                state = int(x5.get_cur_move_state(handle))
            except Exception as exc:
                raise X5RobotError(
                    f"读取 X5 运动状态失败：{_rob_exception_message(exc)}"
                ) from exc
            if state == 1:
                return
            if state == 2:
                if cleared_is_done:
                    return
                raise X5RobotError("X5 运动在完成前已被清除")
            if deadline is not None and time.monotonic() >= deadline:
                raise X5RobotError(f"X5 运动未在 {timeout} ms 内完成")
            time.sleep(0.05)

    def abort(self) -> None:
        """请求控制器立即停止当前运动。"""

        with self._lock:
            self._require_connection()
            try:
                self._x5.abort(self._handle)
                self._wait_cmd_send_done()
            except Exception as exc:
                raise X5RobotError(f"中止运动失败：{_rob_exception_message(exc)}") from exc

    def _wait_cmd_send_done(self) -> None:
        if hasattr(self._x5, "wait_cmd_send_done"):
            self._x5.wait_cmd_send_done(self._handle)

    def _wait_move_done(self) -> None:
        if hasattr(self._x5, "wait_move_done"):
            self._x5.wait_move_done(self._handle, self.move_timeout_ms)

    def _require_connection(self) -> None:
        if self._handle is None or self._x5 is None:
            raise X5RobotError("X5 控制器未连接")

    @staticmethod
    def _check_axis(axis: int) -> None:
        if axis < 0 or axis > 5:
            raise X5RobotError(f"轴索引超出范围：{axis}")

    @staticmethod
    def _check_joint_axis(axis: int) -> None:
        if axis < 0 or axis > 6:
            raise X5RobotError(f"关节轴索引超出范围：{axis}")

    @staticmethod
    def _validate_six_finite_values(
        values: list[float] | tuple[float, ...], label: str
    ) -> list[float]:
        if len(values) != 6:
            raise X5RobotError(f"{label}必须恰好包含六个数值")
        try:
            converted = [float(value) for value in values]
        except (TypeError, ValueError) as exc:
            raise X5RobotError(f"{label}包含非数值") from exc
        if not all(math.isfinite(value) for value in converted):
            raise X5RobotError(f"{label}只能包含有限数值")
        return converted

    @staticmethod
    def _check_digital_output_index(index: int) -> int:
        do_index = int(index)
        if do_index < 0 or do_index > 247:
            raise X5RobotError(f"数字输出索引超出范围：DO{do_index}；期望为 DO0-DO247")
        return do_index

    @staticmethod
    def _check_digital_input_index(index: int) -> int:
        di_index = int(index)
        if di_index < 0 or di_index > 247:
            raise X5RobotError(f"数字输入索引超出范围：DI{di_index}；期望为 DI0-DI247")
        return di_index

    @staticmethod
    def _check_tool_frame_index(index: int, *, allow_zero: bool) -> int:
        tf_index = int(index)
        minimum = 0 if allow_zero else 1
        if tf_index < minimum or tf_index > 16:
            expected = "TF0-TF16" if allow_zero else "TF1-TF16"
            raise X5RobotError(
                f"工具坐标系索引超出范围：TF{tf_index}；期望为 {expected}"
            )
        return tf_index

    @staticmethod
    def _clamp_speed(speed: int) -> int:
        return max(1, min(100, int(speed)))

    @staticmethod
    def _dedupe(values: list[str]) -> list[str]:
        return _dedupe(values)

    def _discover_controller_ips(self) -> list[str]:
        if self._x5 is None:
            return []
        result = discover_x5_ips(self._x5)
        self.discovery_errors = list(result.errors)
        return list(result.ips)

    def _format_network_diagnostics(self) -> str:
        if not self.network_diagnostics:
            return ""
        return f"网络诊断：{' | '.join(self.network_diagnostics)}。"

    def _format_discovery_errors(self) -> str:
        if not self.discovery_errors:
            return ""
        return f"发现错误：{'；'.join(self.discovery_errors)}。"

    def _extract_ips(self, data: Any) -> list[str]:
        return _extract_ips(data)

    @staticmethod
    def _looks_like_ip(value: Any) -> bool:
        return _looks_like_ip(value)

    def _recent_sdk_errors(self) -> str:
        if self._x5 is None:
            return ""
        func = getattr(self._x5, "logger_get_recent_errors", None)
        if func is None:
            return ""
        try:
            errors = func()
        except Exception:
            return ""
        if not errors:
            return ""
        return f"最近 SDK 错误：{errors}。"

    def _shutdown_logger(self) -> None:
        if self._x5 is None:
            return
        func = getattr(self._x5, "logger_shutdown", None)
        if func is None:
            return
        try:
            func()
        except Exception:
            pass

    def __enter__(self) -> "X5Robot":
        self.connect()
        return self

    def __exit__(self, *_exc: object) -> None:
        self.disconnect()
