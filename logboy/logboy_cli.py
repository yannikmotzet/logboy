import argparse
import yaml
import os
import sys
import tty
import termios
import threading
import time
from datetime import datetime
from logboy.logboy_controller import LogboyController
from logboy.logboy_stats import TopicSnapshot


# ── Terminal Helpers ─────────────────────────────────────────────────────────

class RawModeStream:
    """Wraps a stream to replace \n with \r\n for raw terminal mode."""
    def __init__(self, stream):
        self._stream = stream

    def write(self, data):
        self._stream.write(data.replace('\n', '\r\n'))

    def flush(self):
        self._stream.flush()

    def __getattr__(self, attr):
        return getattr(self._stream, attr)


def clear():
    os.system("clear")


def fmt_first_seen(ts: float | None) -> str:
    if ts is None:
        return "   —"
    return datetime.fromtimestamp(ts).strftime("%H:%M:%S")


def fps_color(s: TopicSnapshot) -> str:
    if s.first_seen is None: return "\033[90m"   # gray  – no message yet
    if s.age > 5:            return "\033[90m"   # gray  – dead
    if s.fps < 1:            return "\033[91m"   # red
    return "\033[92m"                            # green


# ── Monitor Render ────────────────────────────────────────────────────────────

def render(snapshots: list[TopicSnapshot], is_paused: bool, start_time: float):
    R = "\033[0m"
    elapsed = time.monotonic() - start_time
    h, rem = divmod(int(elapsed), 3600)
    m, s   = divmod(rem, 60)

    status = "\033[93m⏸  PAUSED\033[0m" if is_paused else "\033[92m⏺  RECORDING\033[0m"

    clear()
    print(f"\n\033[1;34m  ══ Logboy ══\033[0m  {status}   "
          f"\033[90melapsed {h:02d}:{m:02d}:{s:02d}"
          f"   [SPACE] pause/resume   [Ctrl+C] stop\033[0m\n")

    col_w = 44
    print(f"  {'TOPIC':<{col_w}} {'EXP FPS':>8} {'FPS':>7} {'DROPS':>7} {'MSGS':>7} {'AGE':>8}")
    print("  " + "─" * 86)

    for s in sorted(snapshots, key=lambda x: x.name):
        fc = fps_color(s)
        dc = "\033[91m" if s.drops > 0 else "\033[92m"
        exp_str = f"{s.expected_fps:>7.1f}" if s.expected_fps else "      —"

        if not s.first_seen:
            age_str = f"\033[90m{'—':>7}{R}"
            fps_str = f"\033[90m{'—':>7}{R}"
        else:
            fps_str = f"{fc}{s.fps:>7.1f}{R}"
            if s.age > 5:
                age_color = "\033[91m"   # rot  – topic tot
            elif s.expected_fps > 0 and s.age > 1.0 / s.expected_fps * 3:
                age_color = "\033[93m"   # gelb – deutlich überfällig
            else:
                age_color = "\033[92m"   # grün
            age_str = f"{age_color}{s.age:>7.1f}s{R}"

        print(
            f"  \033[36m{s.name:<{col_w}}{R}"
            f" \033[90m{exp_str}{R}"
            f" {fps_str}"
            f" {dc}{s.drops:>7}{R}"
            f" {s.total_msgs:>7}"
            f" {age_str}"
        )

    print("  " + "─" * 86)
    print(f"  \033[90m{len(snapshots)} topic(s)\033[0m\n")


# ── Monitor Loop ──────────────────────────────────────────────────────────────

def monitor_loop(controller: LogboyController,
                 get_paused,
                 stop_event: threading.Event,
                 start_time: float,
                 refresh: float = 1.0):
    while not stop_event.is_set():
        render(controller.get_stats(), get_paused(), start_time)
        time.sleep(refresh)


# ── Keypress Handler ──────────────────────────────────────────────────────────

def read_keypresses(pause_callback, stop_event):
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    old_stdout, old_stderr = sys.stdout, sys.stderr
    try:
        tty.setcbreak(fd)
        sys.stdout = RawModeStream(old_stdout)
        sys.stderr = RawModeStream(old_stderr)
        while not stop_event.is_set():
            ch = sys.stdin.read(1)
            if ch == ' ':
                pause_callback()
            elif ch == '\x03':  # Ctrl+C in raw mode
                stop_event.set()
                break
    finally:
        sys.stdout, sys.stderr = old_stdout, old_stderr
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


# ── Config ────────────────────────────────────────────────────────────────────

def load_config(config_file: str) -> dict:
    with open(config_file, 'r') as f:
        return yaml.safe_load(f)


def validate_config(config: dict):
    required = ["storage_path", "robot_name", "topics", "ros_storage_plugin"]
    for key in required:
        if key not in config:
            raise ValueError(f"Missing required config key: '{key}'")


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Logboy CLI tool")
    subparsers = parser.add_subparsers(dest="command", required=True)
    subparsers.add_parser('record', help="Start recording")
    parser.add_argument('--config',   type=str, required=True, help="Path to config YAML")
    parser.add_argument('--refresh',  type=float, default=1.0, help="Monitor refresh rate in seconds")
    args = parser.parse_args()

    config = load_config(args.config)
    validate_config(config)

    controller = LogboyController()
    controller.configure_recorder(config)  # path validation happens here

    if args.command == "record":
        controller.start_recording()

        is_paused = False
        stop_event = threading.Event()
        start_time = time.monotonic()

        def toggle_pause():
            nonlocal is_paused
            if is_paused:
                controller.resume_recording()
            else:
                controller.pause_recording()
            is_paused = not is_paused

        monitor_thread = threading.Thread(
            target=monitor_loop,
            args=(controller, lambda: is_paused, stop_event, start_time, args.refresh),
            daemon=True,
        )
        key_thread = threading.Thread(
            target=read_keypresses,
            args=(toggle_pause, stop_event),
            daemon=True,
        )

        monitor_thread.start()
        key_thread.start()

        try:
            stop_event.wait()
        except KeyboardInterrupt:
            pass
        finally:
            controller.stop_recording()
            controller.shutdown()
            clear()
            print("Recording stopped.")


if __name__ == "__main__":
    main()