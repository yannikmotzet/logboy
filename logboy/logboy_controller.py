import rclpy
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
import threading
import time
import os
import re
import hashlib
import yaml
from dataclasses import dataclass, field
from logboy.logboy_node import LogboyNode


# ── Recording info ─────────────────────────────────────────────────────────────

_BAG_NAME_RE = re.compile(r'^(.+)_(\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2})$')


@dataclass
class RecordingInfo:
    name: str
    path: str
    start_time: float | None
    end_time: float | None
    duration: float | None
    robot: str | None
    location: str
    size: int
    files: list[dict] = field(default_factory=list)
    topics: list[dict] = field(default_factory=list)


def _ns_to_s(ns: int) -> float:
    return ns / 1_000_000_000


def _file_md5(path: str) -> str:
    h = hashlib.md5()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(8 * 1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def _dir_size(path: str) -> int:
    return sum(e.stat().st_size for e in os.scandir(path) if e.is_file())


def _parse_bag_metadata(bag_dir: str) -> dict | None:
    meta_path = os.path.join(bag_dir, 'metadata.yaml')
    if not os.path.isfile(meta_path):
        return None
    try:
        with open(meta_path) as f:
            data = yaml.safe_load(f)
        return data.get('rosbag2_bagfile_information') or {}
    except Exception:
        return None


def list_recordings(storage_path: str, compute_md5: bool = True) -> list[RecordingInfo]:
    """Scan *storage_path* and return a RecordingInfo for every bag directory found,
    sorted newest-first.  Pass compute_md5=False to skip (potentially slow) MD5
    hashing of large MCAP files."""
    storage_path = os.path.expanduser(storage_path)
    if not os.path.isdir(storage_path):
        return []

    recordings: list[RecordingInfo] = []

    for entry in os.scandir(storage_path):
        if not entry.is_dir():
            continue

        name = entry.name
        bag_dir = entry.path

        # Skip directories without a metadata.yaml (not a bag)
        meta = _parse_bag_metadata(bag_dir)
        if meta is None:
            continue

        # Parse robot name from logboy's naming convention: {robot}_{YYYY-MM-DD_HH-MM-SS}
        m = _BAG_NAME_RE.match(name)
        robot = m.group(1) if m else None

        # Bag-level times
        bag_start_ns = (meta.get('starting_time') or {}).get('nanoseconds_since_epoch')
        bag_dur_ns   = (meta.get('duration')       or {}).get('nanoseconds')
        bag_start = _ns_to_s(bag_start_ns) if bag_start_ns is not None else None
        bag_dur   = _ns_to_s(bag_dur_ns)   if bag_dur_ns   is not None else None
        bag_end   = (bag_start + bag_dur)   if (bag_start is not None and bag_dur is not None) else None

        # Files list
        files: list[dict] = []
        for f in meta.get('files') or []:
            rel_path = f.get('path', '')
            abs_path = os.path.join(bag_dir, rel_path)
            f_start_ns = (f.get('starting_time') or {}).get('nanoseconds_since_epoch')
            f_dur_ns   = (f.get('duration')       or {}).get('nanoseconds')
            f_start = _ns_to_s(f_start_ns) if f_start_ns is not None else None
            f_dur   = _ns_to_s(f_dur_ns)   if f_dur_ns   is not None else None
            f_end   = (f_start + f_dur)    if (f_start is not None and f_dur is not None) else None
            f_size  = os.path.getsize(abs_path) if os.path.isfile(abs_path) else 0
            files.append({
                'path':       rel_path,
                'start_time': f_start,
                'end_time':   f_end,
                'duration':   f_dur,
                'md5sum':     _file_md5(abs_path) if (compute_md5 and os.path.isfile(abs_path)) else None,
                'size':       f_size,
            })

        # Topics list
        topics: list[dict] = []
        for t in meta.get('topics_with_message_count') or []:
            tm    = t.get('topic_metadata') or {}
            count = t.get('message_count', 0)
            freq  = (count / bag_dur) if (bag_dur and bag_dur > 0) else None
            topics.append({
                'name':      tm.get('name'),
                'type':      tm.get('type'),
                'count':     count,
                'frequency': freq,
            })

        recordings.append(RecordingInfo(
            name=name,
            path=bag_dir,
            start_time=bag_start,
            end_time=bag_end,
            duration=bag_dur,
            robot=robot,
            location=storage_path,
            size=_dir_size(bag_dir),
            files=files,
            topics=topics,
        ))

    recordings.sort(key=lambda r: r.start_time or 0.0, reverse=True)
    return recordings


# ── Controller ─────────────────────────────────────────────────────────────────

class LogboyController:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = LogboyNode()
        self._record_start_time: float | None = None
        self._paused_duration: float = 0.0
        self._pause_start_time: float | None = None
        self._max_duration: float | None = None
        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self.node)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()

    def shutdown(self):
        self._executor.shutdown()
        self._spin_thread.join()
        if rclpy.ok():
            rclpy.shutdown()

    def configure_monitor(self):
        self.node.configure_monitor()

    def configure_recorder(self, config):
        self.node.configure_recorder(config)

    def set_topics(self, topics):
        self.node.set_rec_topics(topics)

    def set_max_duration(self, seconds: float | None):
        self._max_duration = seconds

    def get_max_duration(self) -> float | None:
        return self._max_duration

    def check_max_duration(self):
        if self._max_duration is None:
            return
        elapsed = self.get_elapsed()
        if elapsed is not None and elapsed >= self._max_duration:
            self.stop_recording()

    def start_recording(self):
        self.node.start_recording()
        self._record_start_time = time.monotonic()
        self._paused_duration = 0.0
        self._pause_start_time = None

    def stop_recording(self):
        self.node.stop_recording()
        self._record_start_time = None
        self._paused_duration = 0.0
        self._pause_start_time = None

    def pause_recording(self):
        self.node.pause_recording()
        self._pause_start_time = time.monotonic()

    def resume_recording(self):
        self.node.resume_recording()
        if self._pause_start_time is not None:
            self._paused_duration += time.monotonic() - self._pause_start_time
            self._pause_start_time = None

    def get_elapsed(self) -> float | None:
        if self._record_start_time is None:
            return None
        elapsed = time.monotonic() - self._record_start_time - self._paused_duration
        if self._pause_start_time is not None:
            elapsed -= time.monotonic() - self._pause_start_time
        return elapsed

    def get_topics(self):
        return self.node.get_rec_topics()
    
    def get_stats(self):
        return self.node.get_stats()

    def get_bag_path(self):
        return self.node.get_bag_path()

    def is_paused(self) -> bool:
        return self.node.is_paused()