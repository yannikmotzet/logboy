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

def build_table(snapshots: list[TopicSnapshot], is_paused: bool, start_time: float, is_recording: bool = True, bag_path: str | None = None) -> Table:
    elapsed = time.monotonic() - start_time
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
                 start_time: float,
                 live: Live,
                 refresh: float = 1.0,
                 is_recording: bool = True):
    while not stop_event.is_set():
        live.update(build_table(controller.get_stats(), get_paused(), start_time, is_recording, controller.get_bag_path()))
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
    config: Path = typer.Option(..., "-c", "--config", help="Path to config YAML", exists=True, file_okay=True, dir_okay=False),
    refresh: float = typer.Option(1.0, help="Monitor refresh rate in seconds"),
):
    """Start recording."""
    cfg = load_config(str(config))
    validate_config(cfg)

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
    start_time = time.monotonic()

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
                live.update(build_table(controller.get_stats(), is_paused, start_time, True, controller.get_bag_path()))

            monitor_thread = threading.Thread(
                target=monitor_loop,
                args=(controller, lambda: is_paused, stop_event, start_time, live, refresh),
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
                controller.stop_recording()
                controller.shutdown()
    finally:
        termios.tcsetattr(stdin_fd, termios.TCSADRAIN, old_term_settings)
        os.dup2(saved_stderr_fd, 2)
        os.close(saved_stderr_fd)

    elapsed = time.monotonic() - start_time
    h, rem = divmod(int(elapsed), 3600)
    m, s = divmod(rem, 60)

    summary = Table(box=box.SIMPLE, show_header=False, title="[bold]Recording stopped[/bold]", title_justify="left")
    summary.add_column(style="dim", justify="right")
    summary.add_column()
    summary.add_row("name",     os.path.basename(final_bag_path) if final_bag_path else "—")
    summary.add_row("path",     os.path.dirname(final_bag_path)  if final_bag_path else "—")
    summary.add_row("duration", f"{h:02d}:{m:02d}:{s:02d}")
    summary.add_row("size",     bag_size(final_bag_path)         if final_bag_path else "—")
    console.print(summary)

    topics_table = Table(box=box.SIMPLE, title_justify="left")
    topics_table.add_column("TOPIC", style="cyan")
    topics_table.add_column("EXP FPS", justify="right", style="dim")
    topics_table.add_column("FPS",   justify="right")
    topics_table.add_column("MSGS",  justify="right")
    topics_table.add_column("DROPS", justify="right")
    for s in sorted(final_stats, key=lambda x: x.name):
        exp_str = f"{s.expected_fps:.1f}" if s.expected_fps else "—"
        fps_str = Text(f"{s.fps:.1f}", style=fps_style(s))
        drops_str = Text(str(s.drops), style="red" if s.drops > 0 else "green")
        topics_table.add_row(s.name, exp_str, fps_str, str(s.total_msgs), drops_str)
    console.print(topics_table)


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
                args=(controller, lambda: False, stop_event, start_time, live, refresh, False),
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
                controller.shutdown()
    finally:
        termios.tcsetattr(stdin_fd, termios.TCSADRAIN, old_term_settings)
        os.dup2(saved_stderr_fd, 2)
        os.close(saved_stderr_fd)

    topics_table = Table(box=box.SIMPLE, title="[bold]Monitor stopped[/bold]", title_justify="left")
    topics_table.add_column("TOPIC", style="cyan")
    topics_table.add_column("EXP FPS", justify="right", style="dim")
    topics_table.add_column("FPS",   justify="right")
    topics_table.add_column("MSGS",  justify="right")
    topics_table.add_column("DROPS", justify="right")
    for s in sorted(final_stats, key=lambda x: x.name):
        exp_str = f"{s.expected_fps:.1f}" if s.expected_fps else "—"
        fps_str = Text(f"{s.fps:.1f}", style=fps_style(s))
        drops_str = Text(str(s.drops), style="red" if s.drops > 0 else "green")
        topics_table.add_row(s.name, exp_str, fps_str, str(s.total_msgs), drops_str)
    console.print(topics_table)


def main():
    app()


if __name__ == "__main__":
    main()
