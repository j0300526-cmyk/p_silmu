"""ROS-independent command freshness and motion limits."""
import math
from dataclasses import dataclass


@dataclass
class InputGate:
    timeout: float = 0.35
    joint_timeout: float = 0.5
    enabled: bool = False
    enable_at: float = -math.inf
    pose_at: float = -math.inf
    joints_at: float = -math.inf
    stopped: bool = False

    def set_enabled(self, value, now):
        # An explicit stop requires a false->true operator rearm.
        if not value:
            self.stopped = False
        self.enabled = value
        self.enable_at = now

    def ready(self, now):
        return (self.enabled and not self.stopped
                and 0 <= now - self.enable_at <= self.timeout
                and 0 <= now - self.pose_at <= self.timeout
                and 0 <= now - self.joints_at <= self.joint_timeout)

    def stop(self):
        self.stopped = True


def stamp_is_fresh(stamp_seconds, ros_now, timeout):
    return (math.isfinite(stamp_seconds) and math.isfinite(ros_now)
            and -0.05 <= ros_now - stamp_seconds <= timeout)


def limited_step(current, target, max_step):
    if len(current) != len(target) or max_step <= 0:
        raise ValueError('Invalid step')
    delta = [b - a for a, b in zip(current, target)]
    scale = min(1.0, max_step / max(max(abs(d) for d in delta), 1e-12))
    return [a + scale * d for a, d in zip(current, delta)]


def smooth_duration(current, target, min_duration, max_velocity, max_acceleration):
    """Quintic, zero endpoint velocity/acceleration bounds for a single segment."""
    if min(min_duration, max_velocity, max_acceleration) <= 0:
        raise ValueError('Motion limits must be positive')
    distance = max(abs(b - a) for a, b in zip(current, target))
    return max(min_duration, 1.875 * distance / max_velocity,
               math.sqrt(5.774 * distance / max_acceleration))
