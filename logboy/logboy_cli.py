import typer
import yaml
import sys
import os
import tty
import termios
import threading
import time
from pathlib import Path
from datetime import datetime
from rich.live import Live
from rich.table import Table
from rich.text import Text
from rich.console import Console
from rich import box
from logboy.logboy_controller import LogboyController
from logboy.logboy_stats import TopicSnapshot

app = typer.Typer(invoke_without_command=True, add_completion=False)
console = Console()


@app.callback()
def callback(ctx: typer.Context):
    if ctx.invoked_subcommand is None:
        typer.echo(ctx.get_help())


# ── TUI Helpers ───────────────────────────────────────────────────────────────

def bag_size(bag_path: str | None) -> str:
    if not bag_path or not os.path.isdir(bag_path):
        return ""
    total = sum(e.stat().st_size for e in os.scandir(bag_path) if e.is_file())
    for unit in ("B", "KB", "MB", "GB"):
        if total < 1024:
            return f"{total:.1f} {unit}"
        total /= 1024
    return f"{total:.1f} TB"


def fps_style(s: TopicSnapshot) -> str:
    if s.first_seen is None: return "dim"
    if s.age > 5:            return "dim"
    if s.fps < 1:            return "red"
    return "green"


def fmt_first_seen(ts: float | None) -> str:
    if ts is None:
        return "—"
    return datetime.fromtimestamp(ts).strftime("%H:%M:%S")


# ── Monitor Render ────────────────────────────────────────────────────────────

def build_table(snapshots: list[TopicSnapshot], is_paused: bool, elapsed: float, is_recording: bool = True, bag_path: str | None = None) -> Table:
    h, rem = divmod(int(elapsed), 3600)
    m, s = divmod(rem, 60)

    if not is_recording:
        status = "[cyan]👁  MONITORING[/cyan]"
        hint = "\\[Ctrl+C] stop"
    elif is_paused:
        status = "[yellow]⏸  PAUSED[/yellow]"
        hint = "\\[SPACE] pause/resume   \\[Ctrl+C] stop"
    else:
        status = "[green]⏺  RECORDING[/green]"
        hint = "\\[SPACE] pause/resume   \\[Ctrl+C] stop"

    if bag_path:
        rec_info = (
            f"\n[dim]  name  [/dim]{os.path.basename(bag_path)}"
            f"[dim]   path  [/dim]{os.path.dirname(bag_path)}"
        )
    else:
        rec_info = ""

    title = (
        f"[bold blue]══ Logboy ══[/bold blue]  {status}{rec_info}\n"
        f"[dim]  elapsed  [/dim]{h:02d}:{m:02d}:{s:02d}"
        f"[dim]   size  [/dim]{bag_size(bag_path) if bag_path else '—'}"
        f"[dim]   {hint}[/dim]"
    )

    table = Table(title=title, box=box.SIMPLE, show_footer=True, title_justify="left")
    table.add_column("TOPIC", style="cyan", footer=f"[dim]{len(snapshots)} topic(s)[/dim]")
    table.add_column("EXP FPS", justify="right", style="dim")
    table.add_column("FPS", justify="right")
    table.add_column("DROPS", justify="right")
    table.add_column("MSGS", justify="right")
    table.add_column("AGE", justify="right")

    for s in sorted(snapshots, key=lambda x: x.name):
        exp_str = f"{s.expected_fps:.1f}" if s.expected_fps else "—"

        if not s.first_seen:
            fps_str = Text("—", style="dim")
            age_str = Text("—", style="dim")
        else:
            fps_str = Text(f"{s.fps:.1f}", style=fps_style(s))
            if s.age > 5:
                age_style = "red"
            elif s.expected_fps > 0 and s.age > 1.0 / s.expected_fps * 3:
                age_style = "yellow"
            else:
                age_style = "green"
            age_str = Text(f"{s.age:.1f}s", style=age_style)

        drops_str = Text(str(s.drops), style="red" if s.drops > 0 else "green")
        table.add_row(s.name, exp_str, fps_str, drops_str, str(s.total_msgs), age_str)

    return table


# ── Monitor Loop ──────────────────────────────────────────────────────────────

def monitor_loop(controller: LogboyController,
                 get_paused,
                 stop_event: threading.Event,
                 get_elapsed,
                 live: Live,
                 refresh: float = 1.0,
                 is_recording: bool = True):
    while not stop_event.is_set():
        if is_recording:
            controller.check_max_duration()
            if controller.get_bag_path() is None and not stop_event.is_set():
                stop_event.set()
                break
        live.update(build_table(controller.get_stats(), get_paused(), get_elapsed(), is_recording, controller.get_bag_path()))
        time.sleep(refresh)


# ── Keypress Handler ──────────────────────────────────────────────────────────

def read_keypresses(pause_callback, stop_event):
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        while not stop_event.is_set():
            ch = sys.stdin.read(1)
            if ch == ' ':
                pause_callback()
            elif ch == '\x03':  # Ctrl+C in raw mode
                stop_event.set()
                break
    finally:
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


# ── Commands ──────────────────────────────────────────────────────────────────

@app.command()
def record(
    config:  Path = typer.Option(None,    "-c", "--config",  help="Path to config YAML (optional if --storage and --robot are provided)", exists=True, file_okay=True, dir_okay=False),
    storage: Path = typer.Option(None,          "--storage", help="Storage path for the bag (required if no --config)"),
    robot:   str  = typer.Option(None,          "--robot",   help="Robot name used in the bag filename (required if no --config)"),
    plugin:  str  = typer.Option("mcap",        "--plugin",  help="ROS2 storage plugin"),
    refresh: float = typer.Option(1.0,          help="Monitor refresh rate in seconds"),
):
    """Start recording."""
    if config:
        cfg = load_config(str(config))
        validate_config(cfg)
    else:
        if not storage or not robot:
            raise typer.BadParameter("--storage and --robot are required when --config is not provided.")
        cfg = {
            "storage_path":      str(storage),
            "robot_name":        robot,
            "topics":            None,
            "ros_storage_plugin": plugin,
        }

    # Suppress ROS2 logger output (writes directly to fd 2, bypassing sys.stderr)
    devnull_fd = os.open(os.devnull, os.O_WRONLY)
    saved_stderr_fd = os.dup(2)
    os.dup2(devnull_fd, 2)
    os.close(devnull_fd)

    controller = LogboyController()
    controller.configure_recorder(cfg)
    controller.start_recording()

    is_paused = False
    stop_event = threading.Event()

    # Save terminal state before entering raw mode
    stdin_fd = sys.stdin.fileno()
    old_term_settings = termios.tcgetattr(stdin_fd)

    try:
        with Live(console=console, screen=True, refresh_per_second=4) as live:
            def toggle_pause():
                nonlocal is_paused
                if is_paused:
                    controller.resume_recording()
                else:
                    controller.pause_recording()
                is_paused = not is_paused
                live.update(build_table(controller.get_stats(), is_paused, controller.get_elapsed() or 0.0, True, controller.get_bag_path()))

            monitor_thread = threading.Thread(
                target=monitor_loop,
                args=(controller, lambda: is_paused, stop_event, lambda: controller.get_elapsed() or 0.0, live, refresh),
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
                stop_event.set()
                final_bag_path = controller.get_bag_path()
                final_stats = controller.get_stats()
                final_elapsed = controller.get_elapsed() or 0.0
                controller.stop_recording()
                controller.shutdown()
    finally:
        termios.tcsetattr(stdin_fd, termios.TCSADRAIN, old_term_settings)
        os.dup2(saved_stderr_fd, 2)
        os.close(saved_stderr_fd)

    console.print(build_table(final_stats, False, final_elapsed, True, final_bag_path))


@app.command()
def monitor(
    config: Path = typer.Option(None, "-c", "--config", help="Path to config YAML", exists=True, file_okay=True, dir_okay=False),
    refresh: float = typer.Option(1.0, help="Monitor refresh rate in seconds"),
):
    """Monitor topics without recording. Without --config all topics are discovered automatically."""
    # Suppress ROS2 logger output (writes directly to fd 2, bypassing sys.stderr)
    devnull_fd = os.open(os.devnull, os.O_WRONLY)
    saved_stderr_fd = os.dup(2)
    os.dup2(devnull_fd, 2)
    os.close(devnull_fd)

    controller = LogboyController()
    if config:
        cfg = load_config(str(config))
        validate_config(cfg)
        controller.configure_recorder(cfg)
    else:
        controller.configure_monitor()

    stop_event = threading.Event()
    start_time = time.monotonic()

    stdin_fd = sys.stdin.fileno()
    old_term_settings = termios.tcgetattr(stdin_fd)

    try:
        tty.setcbreak(stdin_fd)
        with Live(console=console, screen=True, refresh_per_second=4) as live:
            monitor_thread = threading.Thread(
                target=monitor_loop,
                args=(controller, lambda: False, stop_event, lambda: time.monotonic() - start_time, live, refresh, False),
                daemon=True,
            )
            monitor_thread.start()

            try:
                stop_event.wait()
            except KeyboardInterrupt:
                pass
            finally:
                stop_event.set()
                final_stats = controller.get_stats()
                final_elapsed = time.monotonic() - start_time
                controller.shutdown()
    finally:
        termios.tcsetattr(stdin_fd, termios.TCSADRAIN, old_term_settings)
        os.dup2(saved_stderr_fd, 2)
        os.close(saved_stderr_fd)

    console.print(build_table(final_stats, False, final_elapsed, False))


def main():
    app()


if __name__ == "__main__":
    main()
