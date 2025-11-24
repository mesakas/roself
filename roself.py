#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
roself.py - 统一支持 ROS1 / ROS2 的交互式 TUI 话题浏览器（无 Bag 播放功能版）
- 列表页：↑↓选择、←→翻页、/筛选、Enter进入、q退出；显示 HZ（仅采样“当前页可见”topic），显示 ROS 版本与运行时
- 详情页：树形浏览（Enter进入子消息/数组，ESC返回），显示 Name | Type | Value 与当前 topic Hz，显示 ROS 版本与运行时
- 图表页：实时数值曲线（bars/blocks/envelope/line/braille 切换），右下角显示高精度当前值；Space 暂停
- 日志窗：F2 切换底部/右侧；F3/F4 调整大小；PgUp/PgDn/Home/End 滚动；o 开关拦截；l 清空

注意：本版本 **不再包含 Bag 播放功能**，bag 播放请用你的 C++ 播放器。
"""

import os
import sys
import time
import argparse
import threading
import traceback
from dataclasses import dataclass
from collections import deque
from typing import Any, Callable, Dict, List, Optional, Tuple

import curses
import curses.ascii
import importlib.util

# ====================== 日志捕获到 Pane ======================
class LogBuffer:
    def __init__(self, max_lines=5000):
        self.lines = deque(maxlen=max_lines)
        self.lock = threading.Lock()
        self.scroll = 0
        self.capture_enabled = True

    def append(self, text: str):
        if not text:
            return
        with self.lock:
            for ln in text.splitlines():
                self.lines.append(ln)

    def clear(self):
        with self.lock:
            self.lines.clear()
            self.scroll = 0

    def toggle_capture(self):
        with self.lock:
            self.capture_enabled = not self.capture_enabled
            return self.capture_enabled

    def get_view(self, height: int) -> List[str]:
        with self.lock:
            n = len(self.lines)
            if n == 0:
                return []
            start = max(0, n - height - self.scroll)
            end = max(0, n - self.scroll)
            return list(list(self.lines)[start:end])

    def scroll_up(self, n: int = 5):
        with self.lock:
            self.scroll = min(len(self.lines), self.scroll + n)

    def scroll_down(self, n: int = 5):
        with self.lock:
            self.scroll = max(0, self.scroll - n)

    def scroll_home(self):
        with self.lock:
            self.scroll = len(self.lines)

    def scroll_end(self):
        with self.lock:
            self.scroll = 0


GLOBAL_LOG = LogBuffer()


class _StreamToLog:
    def __init__(self, name):
        self.name = name
        self.buf = ""

    def write(self, s):
        if not isinstance(s, str):
            s = str(s)
        self.buf += s
        while "\n" in self.buf:
            line, self.buf = self.buf.split("\n", 1)
            if GLOBAL_LOG.capture_enabled:
                GLOBAL_LOG.append(line)

    def flush(self):
        if self.buf and GLOBAL_LOG.capture_enabled:
            GLOBAL_LOG.append(self.buf)
        self.buf = ""


_ORIG_STDOUT = sys.stdout
_ORIG_STDERR = sys.stderr
sys.stdout = _StreamToLog("stdout")
sys.stderr = _StreamToLog("stderr")

# ====================== UI 安全绘制工具 ======================
def safe_addstr(win, y, x, text, attr=0):
    try:
        if text is None:
            text = ""
        H, W = win.getmaxyx()
        if H <= 0 or W <= 0 or y < 0 or y >= H or x < 0 or x >= W:
            return
        maxlen = max(0, W - x)
        if maxlen <= 0:
            return
        win.addnstr(y, x, text, maxlen, attr)
    except curses.error:
        pass


def safe_hline(win, y, x, ch, n):
    try:
        H, W = win.getmaxyx()
        if H <= 0 or W <= 0 or y < 0 or y >= H or x < 0 or x >= W:
            return
        n = max(0, min(n, W - x))
        if n <= 0:
            return
        win.hline(y, x, ch, n)
    except curses.error:
        pass


# ====================== ROS 兼容层（按需加载） ======================
MsgTypeName = str


@dataclass
class TopicInfo:
    name: str
    type: MsgTypeName


class RosAPI:
    is_ros2: bool = False

    def init_node(self, name: str):
        raise NotImplementedError

    def shutdown(self):
        raise NotImplementedError

    def list_topics(self) -> List[TopicInfo]:
        raise NotImplementedError

    def resolve_type(self, topic: str) -> Optional[MsgTypeName]:
        raise NotImplementedError

    def get_message_class(self, type_name: MsgTypeName):
        raise NotImplementedError

    def create_publisher(self, topic: str, type_name: MsgTypeName):
        raise NotImplementedError

    def create_subscriber(
        self, topic: str, type_name: MsgTypeName, cb: Callable[[Any], None]
    ):
        raise NotImplementedError

    def runtime_hint(self) -> str:
        return ""

    def is_ros_message(self, obj: Any) -> bool:
        raise NotImplementedError

    def fields_and_types(self, msg_obj: Any) -> List[Tuple[str, str]]:
        raise NotImplementedError


# ---- ROS1 实现 ----
class _Ros1(RosAPI):
    is_ros2 = False

    def __init__(self):
        import rospy, rostopic, roslib, rosgraph

        self._rospy = rospy
        self._rostopic = rostopic
        self._roslib = roslib
        self._rosgraph = rosgraph

    def init_node(self, name: str):
        if not self._rospy.core.is_initialized():
            try:
                self._rospy.init_node(name, anonymous=True, disable_signals=True)
            except Exception as e:
                GLOBAL_LOG.append(f"[WARN] rospy.init_node failed: {e}")

    def shutdown(self):
        try:
            self._rospy.signal_shutdown("bye")
        except Exception:
            pass

    def list_topics(self) -> List[TopicInfo]:
        import socket

        old = socket.getdefaulttimeout()
        try:
            socket.setdefaulttimeout(0.5)
            master = self._rosgraph.Master("/ros_tui_watch")
            lst = master.getTopicTypes()
            lst.sort(key=lambda x: x[0])
            return [TopicInfo(n, t) for n, t in lst]
        except Exception as e:
            GLOBAL_LOG.append(f"[WARN] master not reachable: {e}")
            return []
        finally:
            socket.setdefaulttimeout(old)

    def resolve_type(self, topic: str) -> Optional[MsgTypeName]:
        t, _, _ = self._rostopic.get_topic_type(topic, blocking=False)
        return t

    def get_message_class(self, type_name: MsgTypeName):
        return self._roslib.message.get_message_class(type_name)

    def create_publisher(self, topic: str, type_name: MsgTypeName):
        cls = self.get_message_class(type_name)
        return self._rospy.Publisher(topic, cls, queue_size=10)

    class _SubWrap:
        def __init__(self, sub):
            self.sub = sub

        def unregister(self):
            try:
                self.sub.unregister()
            except Exception:
                pass

    def create_subscriber(self, topic: str, type_name: MsgTypeName, cb):
        cls = self.get_message_class(type_name)
        sub = self._rospy.Subscriber(topic, cls, cb, queue_size=5)
        return _Ros1._SubWrap(sub)

    def runtime_hint(self) -> str:
        return os.environ.get("ROS_MASTER_URI", "(unset)")

    def is_ros_message(self, obj: Any) -> bool:
        return hasattr(obj, "__slots__") and hasattr(obj, "_slot_types")

    def fields_and_types(self, msg_obj: Any) -> List[Tuple[str, str]]:
        slots = getattr(msg_obj, "__slots__", [])
        types = getattr(msg_obj, "_slot_types", [])
        return list(zip(slots, types))


# ---- ROS2 实现 ----
class _Ros2(RosAPI):
    is_ros2 = True

    def __init__(self):
        import rclpy
        from rclpy.node import Node
        from rclpy.executors import MultiThreadedExecutor
        from rosidl_runtime_py.utilities import get_message
        from rclpy.qos import QoSProfile

        self._rclpy = rclpy
        self._Node = Node
        self._executor = None
        self._node: Optional[Node] = None
        self._spin_thread: Optional[threading.Thread] = None
        self._get_message = get_message
        self._QoSProfile = QoSProfile

    def init_node(self, name: str):
        if not self._rclpy.ok():
            self._rclpy.init()
        if self._node is None:
            self._node = self._Node(name)
            self._executor = self._rclpy.executors.MultiThreadedExecutor()
            self._executor.add_node(self._node)

            def _spin():
                try:
                    self._executor.spin()
                except Exception:
                    pass

            self._spin_thread = threading.Thread(target=_spin, daemon=True)
            self._spin_thread.start()

    def shutdown(self):
        try:
            if self._executor and self._node:
                self._executor.remove_node(self._node)
            if self._node:
                self._node.destroy_node()
            if self._rclpy.ok():
                self._rclpy.shutdown()
        except Exception:
            pass

    def list_topics(self) -> List[TopicInfo]:
        if not self._node:
            return []
        lst = self._node.get_topic_names_and_types()
        out: List[TopicInfo] = []
        for name, types in lst:
            for t in types:
                out.append(TopicInfo(name, t))
        out.sort(key=lambda x: x.name)
        return out

    def resolve_type(self, topic: str) -> Optional[MsgTypeName]:
        if not self._node:
            return None
        for name, types in self._node.get_topic_names_and_types():
            if name == topic and types:
                return types[0]
        return None

    def get_message_class(self, type_name: MsgTypeName):
        return self._get_message(type_name)

    def create_publisher(self, topic: str, type_name: MsgTypeName):
        cls = self.get_message_class(type_name)
        qos = self._QoSProfile(depth=10)
        return self._node.create_publisher(cls, topic, qos)

    class _SubWrap:
        def __init__(self, node, sub):
            self._node = node
            self.sub = sub

        def unregister(self):
            try:
                self._node.destroy_subscription(self.sub)
            except Exception:
                pass

    def create_subscriber(self, topic: str, type_name: MsgTypeName, cb):
        cls = self.get_message_class(type_name)
        qos = self._QoSProfile(depth=10)
        sub = self._node.create_subscription(cls, topic, cb, qos)
        return _Ros2._SubWrap(self._node, sub)

    def runtime_hint(self) -> str:
        rmw = os.environ.get("RMW_IMPLEMENTATION", "?")
        dom = os.environ.get("ROS_DOMAIN_ID", "?")
        return f"rmw={rmw} dom={dom}"

    def is_ros_message(self, obj: Any) -> bool:
        return hasattr(obj, "__class__") and hasattr(obj, "__slots__")

    def fields_and_types(self, msg_obj: Any) -> List[Tuple[str, str]]:
        if hasattr(msg_obj, "get_fields_and_field_types"):
            return list(msg_obj.get_fields_and_field_types().items())
        out = []
        for s in getattr(msg_obj, "__slots__", []):
            out.append((s, ""))
        return out


# ------- 运行时选择与全局句柄 -------

COMPAT: Optional[RosAPI] = None


def _module_exists(name: str) -> bool:
    return importlib.util.find_spec(name) is not None


def _detect_runtime_from_env() -> str:
    # 1) 显式变量优先
    v = os.environ.get("ROS_VERSION", "").strip()
    if v == "1":
        return "ros1"
    if v == "2":
        return "ros2"

    # 2) 典型环境变量
    if os.environ.get("ROS_MASTER_URI"):
        return "ros1"
    if os.environ.get("RMW_IMPLEMENTATION") or os.environ.get("ROS_DOMAIN_ID"):
        return "ros2"

    # 3) 已安装模块
    has_ros1 = (
        _module_exists("rospy")
        and _module_exists("rostopic")
        and _module_exists("roslib")
        and _module_exists("rosgraph")
    )
    has_ros2 = _module_exists("rclpy")
    if has_ros1 and not has_ros2:
        return "ros1"
    if has_ros2 and not has_ros1:
        return "ros2"
    if has_ros1 and has_ros2:
        # 若两者同在，默认优先 ROS1
        return "ros1"

    GLOBAL_LOG.append(
        "[WARN] Neither ROS1 nor ROS2 detected explicitly; default to ROS1"
    )
    return "ros1"


def set_runtime(choice: str):
    """choice: 'ros1' or 'ros2'"""
    global COMPAT
    if choice == "ros2":
        COMPAT = _Ros2()
    else:
        COMPAT = _Ros1()


def ensure_ros_node():
    if COMPAT:
        COMPAT.init_node("ros_tui_watch")


def runtime_hint() -> str:
    return COMPAT.runtime_hint() if COMPAT else ""


def ros_version_str() -> str:
    if COMPAT is None:
        return "ROS (?)"
    ver = "2" if COMPAT.is_ros2 else "1"
    distro = os.environ.get("ROS_DISTRO", "").strip()
    return f"ROS{ver}{f' ({distro})' if distro else ''}"


def is_ros_message(obj: Any) -> bool:
    return COMPAT.is_ros_message(obj) if COMPAT else False


def strip_array_suffix(type_str: str) -> Tuple[str, Optional[str]]:
    s = (type_str or "").strip()
    if "[" in s and s.endswith("]"):
        base = s[: s.index("[")]
        arr = s[s.index("[") :]
        return base, arr
    return s, None


def preview_value(v: Any, max_len: int = 64) -> str:
    try:
        if is_ros_message(v):
            return "<msg>"
        if isinstance(v, (list, tuple)):
            return f"<seq:{len(v)}>"
        if isinstance(v, bytes):
            text = f"<bytes:{len(v)}>"
        else:
            text = str(v)
        if len(text) > max_len:
            return text[: max_len - 3] + "..."
        return text
    except Exception:
        return "<unprintable>"


# ====================== 日志窗 Pane ======================
class LogPane:
    def __init__(self):
        self.at_bottom = True
        self.min_lines = 3
        self.fixed_lines = 3
        self.min_cols = 16
        self.fixed_cols = 24

    def toggle_side(self):
        self.at_bottom = not self.at_bottom

    def inc(self):
        if self.at_bottom:
            self.fixed_lines = min(20, self.fixed_lines + 1)
        else:
            self.fixed_cols = min(80, self.fixed_cols + 2)

    def dec(self):
        if self.at_bottom:
            self.fixed_lines = max(self.min_lines, self.fixed_lines - 1)
        else:
            self.fixed_cols = max(self.min_cols, self.fixed_cols - 2)

    def layout(self, H, W):
        if self.at_bottom:
            log_h = min(max(self.min_lines, self.fixed_lines), max(1, H - 1))
            main_h = max(1, H - log_h)
            return (0, 0, main_h, W), (main_h, 0, log_h, W)
        else:
            log_w = min(max(self.min_cols, self.fixed_cols), max(1, W - 1))
            main_w = max(1, W - log_w)
            return (0, 0, H, main_w), (0, main_w, H, log_w)

    def draw(self, stdscr, rect):
        y, x, h, w = rect
        try:
            win = stdscr.derwin(h, w, y, x)
            win.erase()
            pos = "BOTTOM" if self.at_bottom else "RIGHT"
            title = (
                f" Logs [{pos}]  F2切换  F3增大  F4减小  PgUp/PgDn滚动  "
                f"o拦截={GLOBAL_LOG.capture_enabled}  l清空 "
            )
            win.addnstr(0, 0, title.ljust(w - 1), w - 1, curses.A_REVERSE)
            view_h = max(1, h - 1)
            lines = GLOBAL_LOG.get_view(view_h)
            row = 1
            for ln in lines[-view_h:]:
                for i in range(0, len(ln), max(1, w - 1)):
                    if row >= h:
                        break
                    win.addnstr(row, 0, ln[i : i + max(1, w - 1)], w - 1)
                    row += 1
                if row >= h:
                    break
            win.noutrefresh()
        except Exception:
            pass


# ====================== Hz 监控（仅采样当前页可见） ======================
class TopicHzMonitor:
    def __init__(self, maxlen: int = 200):
        ensure_ros_node()
        self.maxlen = maxlen
        self.lock = threading.Lock()
        self.arrivals: Dict[str, deque] = {}
        self.subs: Dict[str, Any] = {}

    def _cb_for(self, topic: str):
        dq = self.arrivals.setdefault(topic, deque(maxlen=self.maxlen))

        def _cb(_msg):
            dq.append(time.monotonic())

        return _cb

    def set_active(self, topic_types: List[Tuple[str, str]]):
        want = {t for t, _ in topic_types}
        with self.lock:
            for t in list(self.subs.keys()):
                if t not in want:
                    try:
                        self.subs[t].unregister()
                    except Exception:
                        pass
                    self.subs.pop(t, None)
            for t, ty in topic_types:
                if t in self.subs:
                    continue
                try:
                    self.subs[t] = COMPAT.create_subscriber(t, ty, self._cb_for(t))
                except Exception as e:
                    GLOBAL_LOG.append(f"[WARN] subscribe {t} failed: {e}")

    def get_hz(self, topic: str) -> float:
        dq = self.arrivals.get(topic)
        if not dq or len(dq) < 2:
            return 0.0
        span = dq[-1] - dq[0]
        if span <= 0:
            return 0.0
        return (len(dq) - 1) / span


# ====================== 列表页 ======================
class TopicListUI:
    def __init__(self, win):
        self.win = win
        self.filter_text = ""
        self.all_topics: List[Tuple[str, str]] = []
        self.filtered: List[Tuple[str, str]] = []
        self.sel_index = 0
        self.page = 0
        self.filter_mode = False
        self.page_size = 1
        self.last_refresh_time = 0.0
        self.refresh_interval = 1.5
        self.hzmon = TopicHzMonitor()
        self._last_visible_key = None

    def refresh_topics(self, force=False):
        now = time.time()
        if force or (now - self.last_refresh_time) >= self.refresh_interval:
            topics = [(ti.name, ti.type) for ti in COMPAT.list_topics()]
            self.all_topics = topics
            self.apply_filter()
            self.last_refresh_time = now

    def apply_filter(self):
        if not self.filter_text:
            self.filtered = list(self.all_topics)
        else:
            ft = self.filter_text.lower()
            self.filtered = [
                (t, ty) for (t, ty) in self.all_topics if ft in t.lower()
            ]
        if self.sel_index >= len(self.filtered):
            self.sel_index = max(0, len(self.filtered) - 1)
        self.fix_page()

    def fix_page(self):
        if self.page_size <= 0:
            self.page_size = 1
        self.page = self.sel_index // self.page_size

    def handle_key(self, ch) -> Optional[Tuple[str, str]]:
        # 进入/处理 filter 模式
        if ch == ord("/"):
            self.filter_mode = True
            return None
        if self.filter_mode:
            if ch in (ord("q"), ord("Q")):  # 用 q 返回/退出过滤
                self.filter_mode = False
                if self.filter_text:
                    self.filter_text = ""
                    self.apply_filter()
                return None
            elif ch in (curses.KEY_BACKSPACE, 127, curses.ascii.DEL):
                if self.filter_text:
                    self.filter_text = self.filter_text[:-1]
                    self.apply_filter()
                return None
            elif 32 <= ch <= 126:
                self.filter_text += chr(ch)
                self.apply_filter()
                return None

        # 普通列表导航
        if ch in (curses.KEY_UP, ord("k")) and self.sel_index > 0:
            self.sel_index -= 1
            self.fix_page()
        elif (
            ch in (curses.KEY_DOWN, ord("j"))
            and self.sel_index + 1 < len(self.filtered)
        ):
            self.sel_index += 1
            self.fix_page()
        elif ch == curses.KEY_LEFT and self.page > 0:
            self.page -= 1
            self.sel_index = self.page * self.page_size
        elif ch == curses.KEY_RIGHT:
            max_page = max(0, (len(self.filtered) - 1) // self.page_size)
            if self.page < max_page:
                self.page += 1
                self.sel_index = min(
                    len(self.filtered) - 1, self.page * self.page_size
                )
        elif ch in (10, 13, curses.KEY_ENTER):
            if 0 <= self.sel_index < len(self.filtered):
                return self.filtered[self.sel_index]

        return None

    def draw(self):
        self.win.erase()
        H, W = self.win.getmaxyx()
        header = (
            "ROS Topic Browser  |  ↑ ↓ 移动  ← → 翻页  /搜索  "
            "Enter查看  q退出 "
        )
        self.win.addnstr(0, 0, header, W - 1, curses.A_REVERSE)
        self.page_size = max(1, H - 4)

        total = len(self.filtered)
        max_page = max(0, (total - 1) // self.page_size)
        self.page = min(self.page, max_page)
        start = self.page * self.page_size
        end = min(total, start + self.page_size)

        if self.filter_mode:
            filter_line = f"Filter (/ active, q to cancel): {self.filter_text}"
        else:
            filter_line = (
                f"Press / to filter   |   {ros_version_str()}   |   Runtime: {runtime_hint()}"
            )
        self.win.addnstr(1, 0, filter_line, W - 1)

        name_w = max(20, min(48, W - 22 - 10))
        type_w = max(12, min(22, W - name_w - 10))
        hz_w = 8
        self.win.addnstr(
            2,
            0,
            "TOPIC".ljust(name_w)
            + "TYPE".ljust(type_w)
            + "HZ".rjust(hz_w),
            W - 1,
            curses.A_BOLD,
        )

        visible = self.filtered[start:end]
        vis_key = tuple(visible)
        if vis_key != self._last_visible_key:
            self.hzmon.set_active(visible)
            self._last_visible_key = vis_key

        row = 3
        for i in range(start, end):
            t, ty = self.filtered[i]
            attr = curses.A_REVERSE if i == self.sel_index else curses.A_NORMAL
            hz = self.hzmon.get_hz(t)
            hz_str = f"{hz:6.2f}" if hz > 0 else "  -   "
            line = ("> " if i == self.sel_index else "  ") + t
            self.win.addnstr(
                row,
                0,
                line.ljust(name_w) + ty.ljust(type_w) + hz_str.rjust(hz_w),
                W - 1,
                attr,
            )
            row += 1
            if row >= H - 1:
                break

        footer = f"Total: {total}  Page: {self.page+1}/{max_page+1}"
        self.win.addnstr(H - 1, 0, footer.ljust(W - 1), W - 1, curses.A_REVERSE)
        self.win.noutrefresh()


# ====================== 详情页 ======================
class TopicViewUI:
    def __init__(self, win, topic: str, type_name: str):
        self.win = win
        self.topic = topic
        self.type_name = type_name
        self.sub = None
        self.msg_lock = threading.Lock()
        self.last_msg = None
        self.arrivals = deque(maxlen=200)
        self.hz = 0.0
        self.path = []  # [("slot", name, type_str) | ("idx", index, elem_type)]
        self.sel_index = 0
        self.page = 0
        self.page_size = 1

    def _make_leaf_reader(self, leaf_name: str):
        is_index = leaf_name.startswith("[") and leaf_name.endswith("]")
        idx = int(leaf_name[1:-1]) if is_index else None

        def reader():
            with self.msg_lock:
                msg = self.last_msg
                if msg is None:
                    return None
                obj, _ = self._navigate(msg)
                try:
                    v = obj[idx] if is_index else getattr(obj, leaf_name)
                except Exception:
                    return None
            if isinstance(v, bool):
                return 1.0 if v else 0.0
            if isinstance(v, (int, float)):
                return float(v)
            return None

        return reader

    def start_sub(self):
        ensure_ros_node()
        if not self.type_name:
            self.type_name = COMPAT.resolve_type(self.topic) or ""
        if not self.type_name:
            raise RuntimeError(
                f"Cannot resolve message type for {self.topic}"
            )

        def cb(msg):
            now = time.monotonic()
            self.arrivals.append(now)
            if len(self.arrivals) >= 2:
                span = self.arrivals[-1] - self.arrivals[0]
                if span > 0:
                    self.hz = (len(self.arrivals) - 1) / span
            with self.msg_lock:
                self.last_msg = msg

        self.sub = COMPAT.create_subscriber(self.topic, self.type_name, cb)

    def stop_sub(self):
        if self.sub is not None:
            try:
                self.sub.unregister()
            except Exception:
                pass
            self.sub = None

    def _navigate(self, msg):
        obj = msg
        tstr = self.type_name
        for kind, key, elem_t in self.path:
            if kind == "slot":
                obj = getattr(obj, key)
                tstr = elem_t
            elif kind == "idx":
                obj = obj[key]
                tstr = elem_t
        return obj, tstr

    def _children(self, obj, type_str) -> List[Tuple[str, str, Any, bool]]:
        out = []
        if is_ros_message(obj):
            for s, t in COMPAT.fields_and_types(obj):
                v = getattr(obj, s)
                is_cont = is_ros_message(v) or isinstance(v, (list, tuple))
                out.append((s, t, v, is_cont))
        elif isinstance(obj, (list, tuple)):
            base, arr = strip_array_suffix(type_str or "")
            elem_type = base if arr is not None else ""
            for i, v in enumerate(obj):
                vt = elem_type or (v.__class__.__name__)
                is_cont = is_ros_message(v) or isinstance(v, (list, tuple))
                out.append((f"[{i}]", vt, v, is_cont))
        return out

    def _fix_page(self):
        if self.page_size <= 0:
            self.page_size = 1
        self.page = self.sel_index // self.page_size

    def _total_children(self) -> int:
        with self.msg_lock:
            msg = self.last_msg
        if msg is None:
            return 0
        cur_obj, cur_t = self._navigate(msg)
        return len(self._children(cur_obj, cur_t))

    def handle_key(self, ch) -> Any:
        if ch in (ord("q"), ord("Q")):  # q 返回列表
            if self.path:
                self.path.pop()
                self.sel_index = 0
                self.page = 0
            else:
                return True
        elif ch in (curses.KEY_UP, ord("k")):
            self.sel_index = max(0, self.sel_index - 1)
            self._fix_page()
        elif ch in (curses.KEY_DOWN, ord("j")):
            self.sel_index = min(
                self.sel_index + 1, self._total_children() - 1
            )
            self._fix_page()
        elif ch == curses.KEY_LEFT:
            if self.page > 0:
                self.page -= 1
                self.sel_index = self.page * self.page_size
        elif ch == curses.KEY_RIGHT:
            max_page = max(
                0, (self._total_children() - 1) // self.page_size
            )
            if self.page < max_page:
                self.page += 1
                self.sel_index = min(
                    self._total_children() - 1, self.page * self.page_size
                )
        elif ch in (10, 13, curses.KEY_ENTER):
            with self.msg_lock:
                msg = self.last_msg
            if msg is None:
                return False
            cur_obj, cur_t = self._navigate(msg)
            items = self._children(cur_obj, cur_t)
            if 0 <= self.sel_index < len(items):
                name, tstr, val, is_cont = items[self.sel_index]
                if is_cont:
                    if name.startswith("[") and name.endswith("]"):
                        idx = int(name[1:-1])
                        self.path.append(("idx", idx, tstr))
                    else:
                        self.path.append(("slot", name, tstr))
                    self.sel_index = 0
                    self.page = 0
                else:
                    title = (
                        f"{self.topic}  {self.type_name}  |  Path: "
                        + (
                            "/"
                            + "/".join(
                                [
                                    f"{k}:{v}"
                                    if kind == "slot"
                                    else f"[{k}]:{v}"
                                    for (kind, k, v) in [
                                        (p[0], p[1], p[2])
                                        for p in self.path
                                    ]
                                ]
                            )
                            or "/"
                        )
                    )
                    reader = self._make_leaf_reader(name)
                    test_v = reader()
                    if test_v is None:
                        GLOBAL_LOG.append(
                            f"[INFO] '{name}' 不是数值（或暂无值），图表页会等待样本..."
                        )
                    return ("chart", title, reader)
        return False

    def draw(self):
        self.win.erase()
        H, W = self.win.getmaxyx()
        if H < 4 or W < 20:
            safe_addstr(
                self.win, 0, 0, "Terminal too small; enlarge this pane."
            )
            self.win.noutrefresh()
            return

        path_str = "/" + "/".join(
            [
                f"{k}:{v}" if kind == "slot" else f"[{k}]:{v}"
                for (kind, k, v) in [
                    (p[0], p[1], p[2]) for p in self.path
                ]
            ]
        )
        header = (
            f"{self.topic}  ({self.type_name})"
            f"  |  Path: {path_str if path_str != '/' else '/'}"
            f"  |  Hz: {self.hz:.2f}"
            f"  |  {ros_version_str()}"
            f"  |  Runtime: {runtime_hint()}"
        )
        safe_addstr(self.win, 0, 0, header, curses.A_REVERSE)

        name_w = max(12, int(W * 0.28))
        type_w = max(14, int(W * 0.30))
        val_w = max(10, W - 2 - name_w - type_w)
        safe_addstr(
            self.win,
            1,
            0,
            f"{'Name'.ljust(name_w)}{'Type'.ljust(type_w)}Value",
            curses.A_BOLD,
        )
        safe_hline(self.win, 2, 0, ord("-"), W - 1)

        with self.msg_lock:
            msg = self.last_msg
        if msg is None:
            safe_addstr(self.win, 3, 0, "(waiting for message...)")
            self.win.noutrefresh()
            return

        cur_obj, cur_t = self._navigate(msg)
        items = self._children(cur_obj, cur_t)
        avail_rows = max(1, H - 4)
        self.page_size = avail_rows
        total = len(items)
        max_page = max(0, (total - 1) // self.page_size)
        self.page = min(self.page, max_page)
        start = self.page * self.page_size
        end = min(total, start + self.page_size)

        row = 3
        for i in range(start, end):
            name, tstr, val, is_cont = items[i]
            val_txt = preview_value(val, max_len=val_w)
            mark = "▶ " if is_cont else "  "
            line_name = (mark + name).ljust(name_w)
            line_type = (tstr or "").ljust(type_w)
            attr = curses.A_REVERSE if i == self.sel_index else curses.A_NORMAL
            safe_addstr(
                self.win, row, 0, line_name + line_type + val_txt, attr
            )
            row += 1
            if row >= H - 1:
                break

        footer = f"Items: {total}  Page: {self.page+1}/{max_page+1}   Enter进入(若可)  q返回"
        safe_addstr(self.win, H - 1, 0, footer, curses.A_REVERSE)
        self.win.noutrefresh()


# ====================== 实时曲线 ======================
def lab_len(s: str) -> int:
    try:
        return len(s)
    except Exception:
        return 0


class ChartViewUI:
    def __init__(
        self, win, title: str, read_value_fn, time_window: float = 10.0, max_points: int = 20000
    ):
        self.win = win
        self.title = title
        self.read_value = read_value_fn
        self.time_window = max(1.0, float(time_window))
        self.max_points = max_points

        self.paused = False
        self.show_grid = True

        self.samples = deque()
        self.last_sample_t = 0.0

        self.use_smoothing = False
        self.smooth_alpha = 0.35
        self.last_smooth = None

        self.lock_y = False
        self.locked_vmin = None
        self.locked_vmax = None
        self.vmargin_ratio = 0.08

        self.mode = "bars"
        self.last_raw = None
        self.last_raw_ts = 0.0

    def handle_key(self, ch) -> bool:
        if ch in (ord("q"), ord("Q")):
            return True
        elif ch in (ord(" "),):
            self.paused = not self.paused
        elif ch in (ord("+"),):
            self.time_window = min(300.0, self.time_window * 1.25)
        elif ch in (ord("-"),):
            self.time_window = max(1.0, self.time_window / 1.25)
        elif ch in (ord("r"), ord("R")):
            self.samples.clear()
            self.last_smooth = None
        elif ch in (ord("g"), ord("G")):
            self.show_grid = not self.show_grid
        elif ch in (ord("s"), ord("S")):
            self.use_smoothing = not self.use_smoothing
            self.last_smooth = None
        elif ch in (ord("y"), ord("Y")):
            self.lock_y = not self.lock_y
            if self.lock_y:
                vmin, vmax = self._current_vrange()
                if vmin is not None:
                    self.locked_vmin, self.locked_vmax = vmin, vmax
            else:
                self.locked_vmin = self.locked_vmax = None
        elif ch in (ord("m"), ord("M")):
            order = ["bars", "blocks", "envelope", "line", "braille"]
            self.mode = order[(order.index(self.mode) + 1) % len(order)]
        return False

    def _sample(self):
        if self.paused:
            return
        now = time.monotonic()
        if now - self.last_sample_t < 0.02:
            return
        self.last_sample_t = now
        v = self.read_value()
        if v is None:
            return
        try:
            v = float(v)
        except Exception:
            return

        self.last_raw = v
        self.last_raw_ts = now

        if self.use_smoothing:
            self.last_smooth = (
                v if self.last_smooth is None else self.smooth_alpha * v
                + (1 - self.smooth_alpha) * self.last_smooth
            )
            v_plot = self.last_smooth
        else:
            v_plot = v

        self.samples.append((now, v_plot))
        cutoff = now - self.time_window
        while self.samples and self.samples[0][0] < cutoff:
            self.samples.popleft()
        while len(self.samples) > self.max_points:
            self.samples.popleft()

    def _current_vrange(self):
        if not self.samples:
            return (None, None)
        vs = [v for _, v in self.samples]
        vmin = min(vs)
        vmax = max(vs)
        if vmin == vmax:
            pad = 1.0 if vmax == 0 else abs(vmax) * 0.1 + 1e-9
            vmin -= pad
            vmax += pad
        vr = vmax - vmin
        return (vmin - vr * self.vmargin_ratio, vmax + vr * self.vmargin_ratio)

    def _time_to_col(self, t, start_t, width):
        col = int((t - start_t) / self.time_window * (width - 1))
        return max(0, min(width - 1, col))

    def _val_to_row(self, v, vmin, vmax, top, axis_y, plot_h):
        r = axis_y - 1 - int(
            (v - vmin) / (vmax - vmin) * (plot_h - 1)
        )
        return max(top + 1, min(axis_y - 1, r))

    def _draw_envelope(
        self, top, axis_y, plot_x0, plot_w, plot_h, vmin, vmax
    ):
        now = self.samples[-1][0]
        start_t = now - self.time_window
        buckets = [{"min": None, "max": None, "last": None} for _ in range(plot_w)]
        for (t, v) in self.samples:
            if t < start_t:
                continue
            c = self._time_to_col(t, start_t, plot_w)
            b = buckets[c]
            b["min"] = v if b["min"] is None else min(b["min"], v)
            b["max"] = v if b["max"] is None else max(b["max"], v)
            b["last"] = v
        for i, b in enumerate(buckets):
            if b["last"] is None:
                continue
            x = plot_x0 + i
            lo = self._val_to_row(
                b["min"], vmin, vmax, top, axis_y, plot_h
            )
            hi = self._val_to_row(
                b["max"], vmin, vmax, top, axis_y, plot_h
            )
            if hi > lo:
                lo, hi = hi, lo
            for rr in range(hi, lo + 1):
                safe_addstr(self.win, rr, x, "│")
            safe_addstr(
                self.win,
                self._val_to_row(
                    b["last"], vmin, vmax, top, axis_y, plot_h
                ),
                x,
                "•",
            )

    def _draw_line(
        self, top, axis_y, plot_x0, plot_w, plot_h, vmin, vmax
    ):
        now = self.samples[-1][0]
        start_t = now - self.time_window
        last_row = None
        last_x = None
        for (t, v) in self.samples:
            if t < start_t:
                continue
            c = self._time_to_col(t, start_t, plot_w)
            x = plot_x0 + c
            y = self._val_to_row(v, vmin, vmax, top, axis_y, plot_h)
            safe_addstr(self.win, y, x, "•")
            if last_row is not None and x > last_x:
                step = 1 if y < last_row else -1
                for rr in range(last_row, y, -step):
                    safe_addstr(self.win, rr, x - 1, "│")
            last_row, last_x = y, x

    def _draw_braille(
        self, top, axis_y, plot_x0, plot_w, plot_h, vmin, vmax
    ):
        vW = plot_w * 2
        vH = plot_h * 4

        def vrow_from_val(val):
            ratio = (val - vmin) / (vmax - vmin + 1e-18)
            ratio = max(0.0, min(1.0, ratio))
            return int((1.0 - ratio) * (vH - 1))

        def vcol_from_time(t, start_t):
            ratio = (t - start_t) / (self.time_window + 1e-18)
            ratio = max(0.0, min(1.0, ratio))
            return int(ratio * (vW - 1))

        now = self.samples[-1][0]
        start_t = now - self.time_window
        pts = []
        for (t, v) in self.samples:
            if t < start_t:
                continue
            vc = vcol_from_time(t, start_t)
            vr = vrow_from_val(v)
            pts.append((vc, vr))
        if not pts:
            return

        on_pixels = set()

        def plot(vx, vy):
            if 0 <= vx < vW and 0 <= vy < vH:
                on_pixels.add((vx, vy))

        def line(x0, y0, x1, y1):
            dx = abs(x1 - x0)
            dy = -abs(y1 - y0)
            sx = 1 if x0 < x1 else -1
            sy = 1 if y0 < y1 else -1
            err = dx + dy
            x, y = x0, y0
            while True:
                plot(x, y)
                if x == x1 and y == y1:
                    break
                e2 = 2 * err
                if e2 >= dy:
                    err += dy
                    x += sx
                if e2 <= dx:
                    err += dx
                    y += sy

        px, py = pts[0]
        plot(px, py)
        for (cx, cy) in pts[1:]:
            line(px, py, cx, cy)
            px, py = cx, cy

        DOT_BITS = {
            (0, 0): 0x01,
            (0, 1): 0x02,
            (0, 2): 0x04,
            (0, 3): 0x40,
            (1, 0): 0x08,
            (1, 1): 0x10,
            (1, 2): 0x20,
            (1, 3): 0x80,
        }
        BRAILLE_BASE = 0x2800

        for cell_y in range(plot_h):
            for cell_x in range(plot_w):
                vx0 = cell_x * 2
                vy0 = cell_y * 4
                mask = 0
                for sx in range(2):
                    for sy in range(4):
                        if (vx0 + sx, vy0 + sy) in on_pixels:
                            mask |= DOT_BITS[(sx, sy)]
                ch = chr(BRAILLE_BASE + mask) if mask else " "
                term_y = (plot_h - 1 - cell_y) + (top + 1)
                term_x = plot_x0 + cell_x
                if (
                    0 < term_y < axis_y
                    and plot_x0 <= term_x < plot_x0 + plot_w
                ):
                    safe_addstr(self.win, term_y, term_x, ch)

    def _draw_blocks(
        self, top, axis_y, plot_x0, plot_w, plot_h, vmin, vmax
    ):
        now = self.samples[-1][0]
        start_t = now - self.time_window
        buckets = [None] * plot_w
        for (t, v) in self.samples:
            if t < start_t:
                continue
            ratio = (t - start_t) / (self.time_window + 1e-12)
            c = int(ratio * (plot_w - 1))
            c = max(0, min(plot_w - 1, c))
            buckets[c] = v if buckets[c] is None else v
        blocks = [" ", "▁", "▂", "▃", "▄", "▅", "▆", "▇", "█"]
        for i, v in enumerate(buckets):
            x = plot_x0 + i
            if v is None:
                continue
            level = (v - vmin) / (vmax - vmin + 1e-18)
            level = 0.0 if level < 0 else (1.0 if level > 1 else level)
            idx = int(round(level * 8))
            idx = max(1, min(8, idx))
            ch = blocks[idx]
            y = axis_y - 1
            safe_addstr(self.win, y, x, ch)

    def _draw_bars(
        self, top, axis_y, plot_x0, plot_w, plot_h, vmin, vmax
    ):
        if vmax <= vmin:
            return
        now = self.samples[-1][0]
        start_t = now - self.time_window
        buckets = [None] * plot_w
        for (t, v) in self.samples:
            if t < start_t:
                continue
            col = int(
                (t - start_t) / self.time_window * (plot_w - 1) + 1e-9
            )
            col = max(0, min(plot_w - 1, col))
            buckets[col] = v
        total_half_rows = (axis_y - (top + 1)) * 2
        for i, v in enumerate(buckets):
            if v is None:
                continue
            level = (v - vmin) / (vmax - vmin)
            level = 0.0 if level < 0 else (1.0 if level > 1 else level)
            fill_half = int(round(level * total_half_rows))
            x = plot_x0 + i
            full_rows = fill_half // 2
            half_top = (fill_half % 2) == 1
            for r in range(full_rows):
                y = (axis_y - 1) - r
                if y <= top:
                    break
                safe_addstr(self.win, y, x, "█")
            if half_top:
                y = (axis_y - 1) - full_rows
                if y > top:
                    safe_addstr(self.win, y, x, "▄")

    def _stats(self):
        if not self.samples:
            return None
        vs = [v for _, v in self.samples]
        return dict(cur=vs[-1], min=min(vs), max=max(vs), avg=sum(vs) / len(vs))

    def draw(self):
        self._sample()
        self.win.erase()
        H, W = self.win.getmaxyx()
        if H < 8 or W < 32:
            safe_addstr(
                self.win, 0, 0, "Terminal too small; enlarge this pane."
            )
            self.win.noutrefresh()
            return

        header = (
            f"{self.title}  |  window={self.time_window:.1f}s  "
            f"{'PAUSED' if self.paused else 'LIVE'}  |  "
            f"mode:{self.mode}  smooth:{'on' if self.use_smoothing else 'off'}  "
            f"Y:{'lock' if self.lock_y else 'auto'}  "
            f"+/-缩放 空格暂停 g网格 r清空 s平滑 y锁Y m模式 q返回"
        )
        safe_addstr(self.win, 0, 0, header[: W - 1], curses.A_REVERSE)

        top = 1
        bottom = H - 3
        left_pad = 10
        plot_x0 = left_pad
        plot_w = max(10, W - plot_x0 - 1)
        plot_h = max(4, bottom - top)
        axis_y = bottom

        safe_hline(self.win, top, 0, ord("─"), W - 1)
        safe_hline(self.win, axis_y, 0, ord("─"), W - 1)
        for yy in range(top + 1, axis_y):
            safe_addstr(self.win, yy, 0, "│")

        if not self.samples:
            safe_addstr(
                self.win, top + 1, plot_x0, "(waiting for numeric samples...)"
            )
            self.win.noutrefresh()
            return

        if self.lock_y and self.locked_vmin is not None:
            vmin, vmax = self.locked_vmin, self.locked_vmax
        else:
            vmin, vmax = self._current_vrange()

        yr = (vmax - vmin) if (vmin is not None) else 1.0
        ticks = [vmax, vmin + yr * 0.5, vmin]
        tick_rows = [top + 1, top + 1 + plot_h // 2, axis_y - 1]
        for row, tv in zip(tick_rows, ticks):
            if 0 <= row < H:
                label = f"{tv:.3g}"
                safe_addstr(
                    self.win, row, 1, label.rjust(left_pad - 2)
                )
                if self.show_grid:
                    for xx in range(
                        plot_x0, min(W - 1, plot_x0 + plot_w)
                    ):
                        safe_addstr(self.win, row, xx, "┈")

        if vmin < 0 < vmax:
            zero_row = axis_y - 1 - int(
                (0 - vmin) / (vmax - vmin) * (plot_h - 1)
            )
            zero_row = max(top + 1, min(axis_y - 1, zero_row))
            for xx in range(plot_x0, min(W - 1, plot_x0 + plot_w)):
                safe_addstr(self.win, zero_row, xx, "╌")

        safe_addstr(self.win, axis_y, plot_x0, f"{-self.time_window:.0f}s")
        rlab = "0s"
        safe_addstr(
            self.win,
            axis_y,
            min(W - 1 - lab_len(rlab), plot_x0 + plot_w - lab_len(rlab)),
            rlab,
        )

        if self.mode == "bars":
            self._draw_bars(
                top, axis_y, plot_x0, plot_w, plot_h, vmin, vmax
            )
        elif self.mode == "blocks":
            self._draw_blocks(
                top, axis_y, plot_x0, plot_w, plot_h, vmin, vmax
            )
        elif self.mode == "envelope":
            self._draw_envelope(
                top, axis_y, plot_x0, plot_w, plot_h, vmin, vmax
            )
        elif self.mode == "line":
            self._draw_line(
                top, axis_y, plot_x0, plot_w, plot_h, vmin, vmax
            )
        else:
            self._draw_braille(
                top, axis_y, plot_x0, plot_w, plot_h, vmin, vmax
            )

        st = self._stats()
        if st:
            info = (
                f"cur={st['cur']:.4g}  min={st['min']:.4g}  "
                f"max={st['max']:.4g}  avg={st['avg']:.4g}"
            )
            safe_addstr(self.win, axis_y + 1, 0, info[: W - 1])

        if self.last_raw is not None:
            cur_str = f"cur_raw={format(self.last_raw, '.17g')}"
            y = axis_y + 1
            x = max(0, W - 1 - len(cur_str))
            safe_addstr(self.win, y, x, cur_str)

        self.win.noutrefresh()


# ====================== 主循环 ======================
def main_curses(stdscr):
    try:
        curses.curs_set(0)
    except curses.error:
        pass
    stdscr.nodelay(True)
    stdscr.keypad(True)

    logpane = LogPane()
    chart_ui: Optional[ChartViewUI] = None
    list_ui: Optional[TopicListUI] = None
    view_ui: Optional[TopicViewUI] = None

    def draw_all():
        nonlocal list_ui
        stdscr.erase()
        H, W = stdscr.getmaxyx()
        main_rect, log_rect = logpane.layout(H, W)
        main_win = stdscr.derwin(
            main_rect[2], main_rect[3], main_rect[0], main_rect[1]
        )

        if chart_ui is not None:
            chart_ui.win = main_win
            chart_ui.draw()
        elif view_ui is not None:
            view_ui.win = main_win
            view_ui.draw()
        else:
            if list_ui is None:
                list_ui = TopicListUI(main_win)
                list_ui.refresh_topics(force=True)
            else:
                list_ui.win = main_win
            list_ui.draw()

        logpane.draw(stdscr, log_rect)
        curses.doupdate()

    fps = 30.0
    interval = 1.0 / fps
    last_draw = 0.0
    last_topic_refresh = 0.0

    while True:
        try:
            ch = stdscr.getch()

            if ch == curses.KEY_RESIZE:
                stdscr.erase()
                try:
                    new_h, new_w = stdscr.getmaxyx()
                    if hasattr(curses, "is_term_resized") and curses.is_term_resized(
                        new_h, new_w
                    ):
                        try:
                            curses.resizeterm(new_h, new_w)
                        except curses.error:
                            pass
                except Exception:
                    pass
                last_draw = 0
                continue

            # 日志快捷键是否启用（在列表过滤输入时，不抢 / q 等）
            allow_log_hotkeys = True
            if isinstance(list_ui, TopicListUI) and list_ui.filter_mode:
                allow_log_hotkeys = False

            if allow_log_hotkeys:
                if ch == curses.KEY_F2:
                    logpane.toggle_side()
                elif ch == curses.KEY_F3:
                    logpane.inc()
                elif ch == curses.KEY_F4:
                    logpane.dec()
                elif ch == curses.KEY_PPAGE:
                    GLOBAL_LOG.scroll_up(10)
                elif ch == curses.KEY_NPAGE:
                    GLOBAL_LOG.scroll_down(10)
                elif ch == curses.KEY_HOME:
                    GLOBAL_LOG.scroll_home()
                elif ch == curses.KEY_END:
                    GLOBAL_LOG.scroll_end()
                elif ch in (ord("o"), ord("O")):
                    on = GLOBAL_LOG.toggle_capture()
                    GLOBAL_LOG.append(f"[LOG] output capture -> {on}")
                elif ch in (ord("l"), ord("L")):
                    GLOBAL_LOG.clear()

            # 页面逻辑
            if chart_ui is not None:
                if ch != -1 and chart_ui.handle_key(ch):
                    chart_ui = None
            elif view_ui is not None:
                ret = None
                if ch != -1:
                    ret = view_ui.handle_key(ch)
                if ret is True:
                    view_ui.stop_sub()
                    view_ui = None
                elif (
                    isinstance(ret, tuple)
                    and len(ret) == 3
                    and ret[0] == "chart"
                ):
                    _, title, reader = ret
                    chart_ui = ChartViewUI(None, title, reader)
            else:
                # TopicList 模式
                if ch in (ord("q"), ord("Q")):
                    break
                if list_ui is not None and ch != -1:
                    sel = list_ui.handle_key(ch)
                    if sel is not None:
                        topic, ttype = sel
                        view_ui = TopicViewUI(None, topic, ttype)
                        try:
                            view_ui.start_sub()
                        except Exception as e:
                            if view_ui:
                                view_ui.stop_sub()
                            view_ui = None
                            GLOBAL_LOG.append(f"[ERR] 订阅失败: {e}")

                now = time.time()
                if list_ui and (now - last_topic_refresh > 1.2):
                    list_ui.refresh_topics()
                    last_topic_refresh = now

            now = time.time()
            if now - last_draw >= interval:
                draw_all()
                last_draw = now

            time.sleep(0.005)

        except KeyboardInterrupt:
            break
        except SystemExit:
            break
        except Exception:
            GLOBAL_LOG.append(
                "[EXC] " + "".join(traceback.format_exc())
            )
            time.sleep(0.05)


# ====================== 程序入口 ======================
def main():
    parser = argparse.ArgumentParser(
        description="ROS1/ROS2 交互式 Topic 浏览 + 数值曲线 TUI（无 Bag 播放功能版）"
    )
    parser.add_argument(
        "--ros",
        choices=["auto", "1", "2"],
        default="auto",
        help="强制选择 ROS 版本（默认 auto）",
    )
    args = parser.parse_args()

    # 选择运行时
    if args.ros == "1":
        choice = "ros1"
    elif args.ros == "2":
        choice = "ros2"
    else:
        choice = _detect_runtime_from_env()

    # 初始化运行时
    set_runtime(choice)
    GLOBAL_LOG.append(
        f"[INFO] Runtime selected: {ros_version_str()}  |  hint: {runtime_hint()}"
    )

    curses.wrapper(main_curses)


if __name__ == "__main__":
    main()
