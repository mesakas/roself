#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import curses
import json
import os
import signal
import subprocess
import sys
import tempfile
import time
from datetime import datetime
from argparse import ArgumentParser

# 新增：用于检测 roscore 的依赖
import xmlrpc.client
from urllib.parse import urlparse

SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
CONFIG_DEFAULT = os.path.join(SCRIPT_DIR, "record_config.json")
LAST_USED_FILE = os.path.expanduser("~/.rosbag_profiles_last.json")
DEFAULT_OUT_DIR = os.path.expanduser("~/bags")

# ---------- roscore 检测相关，全局缓存，避免每帧都去连 XMLRPC ----------

_last_ros_status = ""
_last_ros_check_ts = 0.0
_ROS_CHECK_INTERVAL = 2.0  # 秒


def get_roscore_status():
    """
    检测当前 ROS master 状态，并返回一行可显示的字符串。
    使用 ROS_MASTER_URI 环境变量，并通过 XMLRPC 调用 getSystemState 测试连通性。
    结果做缓存，避免频繁请求。
    """
    global _last_ros_status, _last_ros_check_ts

    now = time.time()
    if now - _last_ros_check_ts < _ROS_CHECK_INTERVAL and _last_ros_status:
        return _last_ros_status

    _last_ros_check_ts = now

    uri = os.environ.get("ROS_MASTER_URI")
    if not uri:
        _last_ros_status = "ROS master: 未设置 ROS_MASTER_URI"
        return _last_ros_status

    try:
        parsed = urlparse(uri)
    except Exception:
        parsed = None

    host = parsed.hostname if parsed else "?"
    port = parsed.port if parsed and parsed.port else None
    host_str = f"{host}:{port}" if port else host

    try:
        proxy = xmlrpc.client.ServerProxy(uri)
        # 调用 getSystemState 只是测试连通性，不关心结果内容
        proxy.getSystemState("/rosbag_record_tui")
        status = "已连接"
    except Exception as e:
        status = f"不可达({type(e).__name__})"

    _last_ros_status = f"ROS master: {status}  URI={uri}  Host={host_str}"
    return _last_ros_status


class Profile:
    def __init__(self, key, description, topic_list):
        self.key = key
        self.description = description
        self.topic_list = topic_list


def load_profiles(config_path):
    if not os.path.isfile(config_path):
        raise FileNotFoundError(f"找不到配置文件: {config_path}")
    with open(config_path, "r", encoding="utf-8") as f:
        data = json.load(f)

    profiles = []
    for key, val in data.items():
        desc = val.get("description", "")
        topics = val.get("topic_list", [])
        if not isinstance(topics, list):
            raise ValueError(f"配置 {key} 的 topic_list 必须是数组")
        profiles.append(Profile(key, desc, topics))
    if not profiles:
        raise ValueError("配置文件中没有任何配置项")
    return profiles


def load_last_used():
    if not os.path.isfile(LAST_USED_FILE):
        return None
    try:
        with open(LAST_USED_FILE, "r", encoding="utf-8") as f:
            data = json.load(f)
        return data.get("last_profile")
    except Exception:
        return None


def save_last_used(profile_key):
    try:
        with open(LAST_USED_FILE, "w", encoding="utf-8") as f:
            json.dump({"last_profile": profile_key}, f)
    except Exception:
        pass


def ensure_out_dir(path):
    os.makedirs(path, exist_ok=True)


def run_editor_and_get_text(initial_text=""):
    editor = os.environ.get("EDITOR", "vim")
    with tempfile.NamedTemporaryFile(delete=False, suffix=".note", mode="w", encoding="utf-8") as tmp:
        tmp_path = tmp.name
        if initial_text:
            tmp.write(initial_text)
            tmp.flush()

    try:
        subprocess.call([editor, tmp_path])
    except FileNotFoundError:
        # 如果连 vim 都没有，就直接使用 nano 再尝试
        try:
            subprocess.call(["nano", tmp_path])
        except FileNotFoundError:
            return ""  # 实在没有编辑器，直接放弃

    try:
        with open(tmp_path, "r", encoding="utf-8") as f:
            content = f.read()
    finally:
        try:
            os.remove(tmp_path)
        except OSError:
            pass

    return content.strip()


def append_note(note_file, start_time_ts, note_text):
    if not note_text.strip():
        return
    now = time.time()
    rel_sec = now - start_time_ts
    human_time = datetime.fromtimestamp(now).strftime("%Y-%m-%d %H:%M:%S")
    rel_str = f"{rel_sec:.1f}s"

    line_header = f"[{human_time} | +{rel_str}]\n"
    with open(note_file, "a", encoding="utf-8") as f:
        f.write(line_header)
        f.write(note_text.strip() + "\n\n")


def start_rosbag_record(profile, out_dir):
    ts_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    base_name = f"{ts_str}_{profile.key}"

    ensure_out_dir(out_dir)
    bag_path = os.path.join(out_dir, base_name + ".bag")
    note_path = os.path.join(out_dir, base_name + ".txt")

    cmd = ["rosbag", "record", "-O", bag_path] + profile.topic_list

    try:
        proc = subprocess.Popen(cmd)
    except FileNotFoundError:
        raise RuntimeError("无法找到 rosbag 命令，请检查 ROS 是否安装并 source 了环境。")

    return proc, bag_path, note_path, time.time()


# ==================== UI 绘制函数（美化版） ====================

def _draw_title_bar(stdscr, text):
    """顶栏：居中标题 + 颜色"""
    h, w = stdscr.getmaxyx()
    title = f" {text} "
    x = max(0, (w - len(title)) // 2)
    stdscr.attron(curses.color_pair(1) | curses.A_BOLD)
    stdscr.addnstr(0, 0, " " * (w - 1), w - 1)  # 清整行
    stdscr.addnstr(0, x, title, w - x - 1)
    stdscr.attroff(curses.color_pair(1) | curses.A_BOLD)


def _draw_status_bar(stdscr, lines):
    """底栏：两行提示信息，反色显示"""
    h, w = stdscr.getmaxyx()
    line1 = lines[0] if len(lines) > 0 else ""
    line2 = lines[1] if len(lines) > 1 else ""

    stdscr.attron(curses.color_pair(3))
    stdscr.addnstr(h - 2, 0, " " * (w - 1), w - 1)
    stdscr.addnstr(h - 1, 0, " " * (w - 1), w - 1)
    stdscr.addnstr(h - 2, 0, line1[:w - 1], w - 1)
    stdscr.addnstr(h - 1, 0, line2[:w - 1], w - 1)
    stdscr.attroff(curses.color_pair(3))


def draw_menu(stdscr, profiles, selected_index, page_size, info_line,
              last_profile_key=None, ros_status=""):
    stdscr.clear()
    h, w = stdscr.getmaxyx()

    # 顶栏标题
    _draw_title_bar(stdscr, "ROSBag 录制工具 - 配置选择")

    # ROS 状态行
    row = 1
    if ros_status:
        stdscr.attron(curses.A_DIM)
        stdscr.addnstr(row, 1, ros_status, w - 2)
        stdscr.attroff(curses.A_DIM)
        row += 1

    # 上次配置提示
    if last_profile_key:
        last_text = f"上次录制配置: {last_profile_key}（已高亮）"
        stdscr.addnstr(row, 1, last_text, w - 2)
        row += 1

    # 分隔线
    stdscr.hline(row, 0, ord('-'), w)
    row += 1

    # 列表头
    header = "键名 / 描述"
    stdscr.addnstr(row, 2, header, w - 3)
    row += 1
    stdscr.hline(row, 0, ord('-'), w)
    row += 1

    total = len(profiles)
    page = selected_index // page_size
    start_idx = page * page_size
    end_idx = min(start_idx + page_size, total)

    # 留出底部两行做 status bar，再预留一行 info
    max_list_rows = h - row - 3
    for idx in range(start_idx, end_idx):
        if idx - start_idx >= max_list_rows:
            break
        p = profiles[idx]
        cur_row = row + (idx - start_idx)

        is_sel = (idx == selected_index)
        prefix = "→ " if is_sel else "  "
        line = f"{prefix}{p.key}: {p.description}"

        if is_sel:
            stdscr.attron(curses.color_pair(2) | curses.A_BOLD)
            stdscr.addnstr(cur_row, 0, " " * (w - 1), w - 1)
            stdscr.addnstr(cur_row, 1, line[:w - 2], w - 2)
            stdscr.attroff(curses.color_pair(2) | curses.A_BOLD)
        else:
            stdscr.addnstr(cur_row, 1, line[:w - 2], w - 2)

    # 页信息 + info_line
    page_info = f"第 {page+1} / {((total-1)//page_size)+1} 页，共 {total} 个配置"
    stdscr.addnstr(h - 4, 1, page_info[:w - 2], w - 2)

    if info_line:
        stdscr.addnstr(h - 3, 1, info_line[:w - 2], w - 2)

    # 底栏操作提示
    help1 = "↑↓:选择配置  ←→:翻页  Enter:开始录制  q:退出"
    help2 = "配置来自 record_config.json  |  建议先检查 ROS_MASTER_URI 与 roscore 状态"
    _draw_status_bar(stdscr, [help1, help2])

    stdscr.refresh()


def draw_recording(stdscr, profile, bag_path, elapsed, info_line, ros_status=""):
    stdscr.clear()
    h, w = stdscr.getmaxyx()

    # 顶栏标题
    _draw_title_bar(stdscr, "ROSBag 录制中")

    row = 1
    if ros_status:
        stdscr.attron(curses.A_DIM)
        stdscr.addnstr(row, 1, ros_status, w - 2)
        stdscr.attroff(curses.A_DIM)
        row += 1

    stdscr.hline(row, 0, ord('-'), w)
    row += 1

    stdscr.addnstr(row, 1, f"配置: {profile.key}", w - 2)
    row += 1
    stdscr.addnstr(row, 1, f"描述: {profile.description}", w - 2)
    row += 1
    stdscr.addnstr(row, 1, f"bag 文件: {bag_path}", w - 2)
    row += 2

    stdscr.addnstr(row, 1, f"已录制时间: {elapsed:.1f}s", w - 2)
    row += 2

    stdscr.addnstr(row, 1, "Topic 列表:", w - 2)
    row += 1

    max_topic_rows = h - row - 4
    for i, t in enumerate(profile.topic_list):
        if i >= max_topic_rows:
            stdscr.addnstr(row, 3, "...(更多 Topic 已省略)", w - 4)
            row += 1
            break
        stdscr.addnstr(row, 3, f"- {t}", w - 4)
        row += 1

    if info_line:
        stdscr.addnstr(h - 3, 1, info_line[:w - 2], w - 2)

    help1 = "q:停止录制并返回菜单  a:记笔记（外部编辑器）"
    help2 = "笔记会写入同名 .txt 文件，可在 roself / bag 播放器中联动查看"
    _draw_status_bar(stdscr, [help1, help2])

    stdscr.refresh()


# ==================== 主循环 ====================

def main_curses(stdscr, profiles, out_dir):
    curses.curs_set(0)
    stdscr.nodelay(False)
    stdscr.keypad(True)

    # 初始化颜色
    curses.start_color()
    try:
        curses.use_default_colors()
    except Exception:
        pass
    # 标题栏
    curses.init_pair(1, curses.COLOR_BLACK, curses.COLOR_CYAN)
    # 选中项
    curses.init_pair(2, curses.COLOR_YELLOW, -1)
    # 底部状态栏
    curses.init_pair(3, curses.COLOR_BLACK, curses.COLOR_WHITE)

    page_size = 10
    last_profile_key = load_last_used()

    # 找到上一次使用的 index
    selected_index = 0
    if last_profile_key:
        for i, p in enumerate(profiles):
            if p.key == last_profile_key:
                selected_index = i
                break

    mode = "menu"  # "menu" or "recording"
    info_line = ""

    # recording 状态
    proc = None
    cur_profile = None
    bag_path = None
    note_path = None
    start_time_ts = None

    while True:
        # 每一轮刷新前都更新 roscore 状态（内部有 2s 缓存）
        ros_status = get_roscore_status()

        if mode == "menu":
            draw_menu(stdscr, profiles, selected_index, page_size,
                      info_line, last_profile_key, ros_status)
            info_line = ""

            ch = stdscr.getch()

            if ch in (ord('q'), ord('Q')):
                break

            elif ch == curses.KEY_UP:
                if selected_index > 0:
                    selected_index -= 1

            elif ch == curses.KEY_DOWN:
                if selected_index < len(profiles) - 1:
                    selected_index += 1

            elif ch == curses.KEY_LEFT:
                # 上一页
                selected_index = max(0, selected_index - page_size)

            elif ch == curses.KEY_RIGHT:
                # 下一页
                selected_index = min(len(profiles) - 1, selected_index + page_size)

            elif ch in (curses.KEY_ENTER, 10, 13):
                cur_profile = profiles[selected_index]
                try:
                    proc, bag_path, note_path, start_time_ts = start_rosbag_record(cur_profile, out_dir)
                except Exception as e:
                    info_line = f"启动 rosbag 失败: {e}"
                    cur_profile = None
                    proc = None
                    continue

                save_last_used(cur_profile.key)
                last_profile_key = cur_profile.key
                mode = "recording"
                stdscr.nodelay(True)  # 录制时用非阻塞 getch

        elif mode == "recording":
            # 检查进程是否还活着
            if proc.poll() is not None:
                # rosbag 已停止
                info_line = f"录制结束，bag: {bag_path}"
                mode = "menu"
                stdscr.nodelay(False)
                proc = None
                cur_profile = None
                continue

            elapsed = time.time() - start_time_ts
            draw_recording(stdscr, cur_profile, bag_path, elapsed, info_line, ros_status)
            info_line = ""

            ch = stdscr.getch()
            if ch == -1:
                time.sleep(0.05)
                continue

            if ch in (ord('q'), ord('Q')):
                # 优先发送 SIGINT，模拟 Ctrl+C 以正确关闭 bag
                try:
                    proc.send_signal(signal.SIGINT)
                except Exception:
                    try:
                        proc.terminate()
                    except Exception:
                        pass
                try:
                    proc.wait(timeout=5)
                except Exception:
                    pass
                info_line = f"录制结束，bag: {bag_path}"
                mode = "menu"
                stdscr.nodelay(False)
                proc = None
                cur_profile = None

            elif ch in (ord('a'), ord('A')):
                # 记笔记
                curses.endwin()  # 临时退出 curses，打开编辑器
                try:
                    note_text = run_editor_and_get_text()
                    if note_text:
                        append_note(note_path, start_time_ts, note_text)
                        info_line = "笔记已保存。"
                    else:
                        info_line = "未输入内容，笔记未保存。"
                finally:
                    # 重新初始化 curses
                    stdscr = curses.initscr()
                    curses.curs_set(0)
                    stdscr.nodelay(True)
                    stdscr.keypad(True)
                    # 重新初始化颜色
                    curses.start_color()
                    try:
                        curses.use_default_colors()
                    except Exception:
                        pass
                    curses.init_pair(1, curses.COLOR_BLACK, curses.COLOR_CYAN)
                    curses.init_pair(2, curses.COLOR_YELLOW, -1)
                    curses.init_pair(3, curses.COLOR_BLACK, curses.COLOR_WHITE)


def main():
    parser = ArgumentParser(description="基于配置文件的 rosbag 录制小工具")
    parser.add_argument(
        "-c", "--config",
        default=CONFIG_DEFAULT,
        help=f"配置文件路径（默认: {CONFIG_DEFAULT}）"
    )
    parser.add_argument(
        "-o", "--outdir",
        default=DEFAULT_OUT_DIR,
        help=f"bag 和 txt 输出目录（默认: {DEFAULT_OUT_DIR}）"
    )
    args = parser.parse_args()

    try:
        profiles = load_profiles(args.config)
    except Exception as e:
        print(f"[FATAL] 加载配置失败: {e}")
        sys.exit(1)

    try:
        curses.wrapper(main_curses, profiles, args.outdir)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
