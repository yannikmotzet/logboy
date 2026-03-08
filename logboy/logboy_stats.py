"""
Model: TopicStats
Holds FPS and drop statistics for a single topic.
Completely thread-safe, no ROS import.
"""

import threading
import time
from collections import deque
from dataclasses import dataclass, field


@dataclass
class TopicSnapshot:
    """Immutable snapshot - safe to pass to the view."""
    name: str
    msg_type: str
    fps: float
    drops: int
    total_msgs: int
    age: float          # seconds since the last message
    is_recording: bool  # True if Logboy is currently recording
    first_seen: float | None  # unix timestamp of first message
    expected_fps: float


class TopicStats:
    """
    Thread-safe model for a single topic.
    Populated by LogboyNode, read-only by the view (via snapshot()).
    """

    def __init__(self, name: str, msg_type: str,
                 expected_fps: float = 0.0,
                 window: float = 5.0):
        self.name = name
        self.msg_type = msg_type
        self.expected_fps = expected_fps  # 0 → Drop detection disabled
        self.window = window

        self._lock = threading.Lock()
        self._timestamps: deque[float] = deque()
        self._drops = 0
        self._total_msgs = 0
        self._last_recv = 0.0
        self._is_recording = False
        self._first_seen: float | None = None  # unix timestamp

    # ── Called from ROS callback ────────────────
    def record(self) -> None:
        now = time.monotonic()
        with self._lock:
            if self._first_seen is None:
                self._first_seen = time.time()  # real timestamp for first seen
            self._total_msgs += 1

            if self.expected_fps > 0 and self._last_recv > 0:
                gap = now - self._last_recv
                interval = 1.0 / self.expected_fps
                if gap > interval * 1.8:
                    self._drops += max(int(gap / interval) - 1, 1)

            self._last_recv = now
            self._timestamps.append(now)
            self._trim(now)

    def set_recording(self, flag: bool) -> None:
        with self._lock:
            self._is_recording = flag

    def reset_drops(self) -> None:
        with self._lock:
            self._drops = 0

    # ── Called by the view ──────────────────────
    def snapshot(self) -> TopicSnapshot:
        now = time.monotonic()
        with self._lock:
            self._trim(now)
            n = len(self._timestamps)
            if n >= 2:
                fps = (n - 1) / (self._timestamps[-1] - self._timestamps[0])
            elif n == 1 and (now - self._timestamps[0]) < self.window:
                fps = 1.0 / self.window
            else:
                fps = 0.0

            age = (now - self._last_recv) if self._last_recv else float("inf")

            # Pending drops: Include silence since the last message,
            # but do not write to _drops (record() would count them again)
            pending_drops = 0
            if self.expected_fps > 0 and self._last_recv > 0:
                interval = 1.0 / self.expected_fps
                if age > interval * 1.8:
                    pending_drops = int(age / interval) - 1

            return TopicSnapshot(
                name=self.name,
                msg_type=self.msg_type,
                fps=round(fps, 2),
                drops=self._drops + pending_drops,
                total_msgs=self._total_msgs,
                age=round(age, 2),
                is_recording=self._is_recording,
                first_seen=self._first_seen,
                expected_fps=self.expected_fps,
            )

    # ── Internal ────────────────────────────────
    def _trim(self, now: float) -> None:
        cutoff = now - self.window
        while self._timestamps and self._timestamps[0] < cutoff:
            self._timestamps.popleft()