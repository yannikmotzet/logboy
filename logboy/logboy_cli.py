import argparse
import yaml
import os
import sys
import tty
import termios
import threading
from logboy.logboy_controller import LogboyController

class RawModeStream:
    """Wraps a stream to replace \\n with \\r\\n for raw terminal mode."""
    def __init__(self, stream):
        self._stream = stream

    def write(self, data):
        self._stream.write(data.replace('\n', '\r\n'))

    def flush(self):
        self._stream.flush()

    def __getattr__(self, attr):
        return getattr(self._stream, attr)

def load_config(config_file):
    with open(config_file, 'r') as file:
        return yaml.safe_load(file)

def read_keypresses(pause_callback, stop_event):
    """Read keypresses in raw mode without requiring Enter."""
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


def main():
    parser = argparse.ArgumentParser(description="Logboy CLI tool")
    subparsers = parser.add_subparsers(dest="command", required=True)
    subparsers.add_parser('record', help="Start recording")
    parser.add_argument(
        '--config',
        type=str,
        required=True,
        help="Path to the configuration file"
    )

    args = parser.parse_args()

    config_file = args.config
    recorder_config = load_config(config_file)

    required_keys = ["storage_path", "robot_name", "topics", "ros_storage_plugin"]
    for key in required_keys:
        if key not in recorder_config:
            raise ValueError(f"Missing required configuration key: {key}")

    if recorder_config["storage_path"].startswith("~"):
        recorder_config["storage_path"] = os.path.expanduser(recorder_config["storage_path"])

    if not os.path.exists(recorder_config["storage_path"]):
        raise ValueError(f"Storage path does not exist: {recorder_config['storage_path']}")

    logboy_controller = LogboyController()
    logboy_controller.configure_recorder(recorder_config)

    if args.command == "record":
        logboy_controller.start_recording()
        # print("Starting recording. Press [SPACE] to pause/resume, [Ctrl+C] to stop.\r")

        is_paused = False
        stop_event = threading.Event()

        def toggle_pause():
            nonlocal is_paused
            if is_paused:
                logboy_controller.resume_recording()
            else:
                logboy_controller.pause_recording()
            is_paused = not is_paused

        key_thread = threading.Thread(target=read_keypresses, args=(toggle_pause, stop_event), daemon=True)
        key_thread.start()

        stop_event.wait()  # block until Ctrl+C sets the event
        logboy_controller.stop_recording()
        logboy_controller.shutdown()


if __name__ == "__main__":
    main()