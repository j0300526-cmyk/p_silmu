"""ROS-independent filtering, gesture debounce, and relative clutch retargeting.

1 Euro: Casiez et al., CHI 2012. Relative pause/reindexing and fine resolution
are adaptations inspired by OPEN TEACH (CoRL 2024 / PMLR 2025), not VR tracking.
"""
from collections import deque
import math
from statistics import median


class OneEuroFilter:
    def __init__(self, min_cutoff=1.0, beta=0.8, derivative_cutoff=1.0, max_gap=0.35):
        values = (min_cutoff, beta, derivative_cutoff, max_gap)
        if not all(math.isfinite(v) for v in values) or min(
                min_cutoff, derivative_cutoff, max_gap) <= 0 or beta < 0:
            raise ValueError('Invalid 1 Euro filter parameters')
        self.min_cutoff = min_cutoff
        self.beta = beta
        self.derivative_cutoff = derivative_cutoff
        self.max_gap = max_gap
        self.reset()

    def reset(self):
        self.raw = self.filtered = self.timestamp = None
        self.derivative = 0.0

    @staticmethod
    def alpha(cutoff, dt):
        return 1.0 / (1.0 + 1.0 / (2.0 * math.pi * cutoff * dt))

    def update(self, value, timestamp):
        if not math.isfinite(value) or not math.isfinite(timestamp):
            raise ValueError('Filter requires finite input and timestamp')
        if self.timestamp is not None and timestamp <= self.timestamp:
            raise ValueError('Filter timestamps must increase')
        if self.timestamp is None or timestamp - self.timestamp > self.max_gap:
            self.raw = self.filtered = value
            self.timestamp = timestamp
            self.derivative = 0.0
            return value
        dt = timestamp - self.timestamp
        derivative = (value - self.raw) / dt
        alpha_d = self.alpha(self.derivative_cutoff, dt)
        self.derivative += alpha_d * (derivative - self.derivative)
        cutoff = self.min_cutoff + self.beta * abs(self.derivative)
        self.filtered += self.alpha(cutoff, dt) * (value - self.filtered)
        self.raw, self.timestamp = value, timestamp
        return self.filtered


class VectorFilter:
    def __init__(self, kind='one_euro', window=5, **one_euro_parameters):
        if kind not in ('one_euro', 'moving_average') or window < 1:
            raise ValueError('Unknown filter or invalid window')
        self.kind = kind
        self.filters = [OneEuroFilter(**one_euro_parameters) for _ in range(3)]
        self.histories = [deque(maxlen=window) for _ in range(3)]

    def reset(self):
        for filt, history in zip(self.filters, self.histories):
            filt.reset()
            history.clear()

    def update(self, values, timestamp):
        if len(values) != 3 or not all(math.isfinite(v) for v in (*values, timestamp)):
            raise ValueError('Invalid three-axis input')
        if self.kind == 'one_euro':
            return [filt.update(value, timestamp) for filt, value in zip(self.filters, values)]
        result = []
        for history, value in zip(self.histories, values):
            history.append(value)
            result.append(sum(history) / len(history))
        return result


class DwellHysteresis:
    """Close below low, open above high, requiring a continuous dwell."""
    def __init__(self, low=0.22, high=0.30, dwell=0.08):
        if not all(math.isfinite(v) for v in (low, high, dwell)) or not 0 < low < high or dwell < 0:
            raise ValueError('Invalid gesture thresholds')
        self.low, self.high, self.dwell = low, high, dwell
        self.reset()

    def reset(self):
        self.state = False
        self.candidate = None
        self.since = None
        self.last_time = None

    def update(self, value, timestamp):
        if not math.isfinite(value) or not math.isfinite(timestamp):
            self.reset()
            return False
        if self.last_time is not None and timestamp < self.last_time:
            raise ValueError('Gesture time must not go backwards')
        self.last_time = timestamp
        desired = True if value < self.low else False if value > self.high else self.state
        if desired == self.state:
            self.candidate = None
        elif desired != self.candidate:
            self.candidate, self.since = desired, timestamp
        if self.candidate is not None and timestamp - self.since >= self.dwell:
            self.state, self.candidate = self.candidate, None
        return self.state


class ControlHandSelector:
    """Lock a handedness label until explicit rearming; never transfer on loss."""
    def __init__(self, preferred='auto'):
        if preferred not in ('auto', 'left', 'right'):
            raise ValueError('Control hand must be auto, left or right')
        self.preferred = preferred
        self.reset()

    def reset(self):
        self.selected = None

    def update(self, available, armed):
        if not armed:
            self.reset()
            return None
        if self.selected is None:
            choices = ('Right', 'Left') if self.preferred == 'auto' else (self.preferred.title(),)
            self.selected = next((label for label in choices if label in available), None)
        return self.selected if self.selected in available else None


def hand_features(points, width, height, min_palm_pixels=20.0):
    """Return normalized [log apparent size, screen x, screen-up y] and pinch ratio.

    Apparent palm size is a monocular proxy, not metric depth. No camera-to-world
    extrinsic calibration or 3D reconstruction is implied.
    """
    if (len(points) != 21 or min(width, height, min_palm_pixels) <= 0
            or not all(len(point) == 2 and all(math.isfinite(v) for v in point) for point in points)):
        raise ValueError('Invalid landmarks')
    if any(not (-.2 * width <= x <= 1.2 * width and -.2 * height <= y <= 1.2 * height)
           for x, y in points):
        raise ValueError('Landmarks outside observation bounds')
    palm = math.dist(points[0], points[9])
    if palm < min_palm_pixels:
        raise ValueError('Palm too small for stable input')
    wrist = points[0]
    features = [math.log(palm / width), wrist[0] / width, -wrist[1] / height]
    pinch = math.dist(points[4], points[8]) / palm
    return features, pinch


class RelativeRetargeter:
    def __init__(self, gains=(0.12, 0.4, 0.3), calibration_frames=5, calibration_tolerance=0.05,
                 max_speed=0.06, max_lead=0.04,
                 lower=(0.05, -0.25, 0.05), upper=(0.35, 0.25, 0.30)):
        if (len(gains) != 3 or len(lower) != 3 or len(upper) != 3
                or not all(math.isfinite(v) for v in (
                    *gains, *lower, *upper, calibration_tolerance, max_speed, max_lead))
                or min(*gains, max_speed, max_lead, calibration_tolerance) <= 0
                or calibration_frames < 1 or any(a >= b for a, b in zip(lower, upper))):
            raise ValueError('Invalid retargeting configuration')
        self.gains, self.lower, self.upper = gains, lower, upper
        self.frames, self.tolerance = calibration_frames, calibration_tolerance
        self.max_speed, self.max_lead = max_speed, max_lead
        self.samples = deque(maxlen=calibration_frames)
        self.reset()

    def reset(self):
        self.samples.clear()
        self.anchor_hand = self.anchor_robot = self.target = self.timestamp = None
        self.resolution = 1.0

    @property
    def calibrated(self):
        return self.anchor_hand is not None

    def update(self, features, robot, timestamp, resolution=1.0):
        if (len(features) != 3 or len(robot) != 3
                or not all(math.isfinite(v) for v in (*features, *robot, timestamp, resolution))
                or not 0 < resolution <= 1):
            raise ValueError('Invalid retargeting observation')
        if self.timestamp is not None and timestamp <= self.timestamp:
            raise ValueError('Retargeting timestamps must increase')
        if not self.calibrated:
            self.samples.append(tuple(features))
            self.timestamp = timestamp
            if len(self.samples) < self.frames or any(
                    max(s[i] for s in self.samples) - min(s[i] for s in self.samples) > self.tolerance
                    for i in range(3)):
                return None
            self.anchor_hand = [median(s[i] for s in self.samples) for i in range(3)]
            self.anchor_robot = list(robot)
            self.target = list(robot)
            self.resolution = resolution
            return list(self.target)
        dt = min(0.1, timestamp - self.timestamp)
        self.timestamp = timestamp
        if resolution != self.resolution:
            # Reindex both ends when changing precision so the target never jumps.
            self.anchor_hand, self.anchor_robot = list(features), list(robot)
            self.target, self.resolution = list(robot), resolution
            return list(self.target)
        desired = [p + resolution * gain * (value - anchor)
                   for p, gain, value, anchor in zip(
                       self.anchor_robot, self.gains, features, self.anchor_hand)]
        desired = [max(min(lo, anchor), min(max(hi, anchor), value))
                   for value, anchor, lo, hi in zip(desired, self.anchor_robot, self.lower, self.upper)]
        lead = [value - current for value, current in zip(desired, robot)]
        lead_norm = math.sqrt(sum(v*v for v in lead))
        desired = [current + value * min(1.0, self.max_lead / max(lead_norm, 1e-12))
                   for current, value in zip(robot, lead)]
        delta = [value - prior for value, prior in zip(desired, self.target)]
        norm = math.sqrt(sum(v*v for v in delta))
        scale = min(1.0, self.max_speed * resolution * dt / max(norm, 1e-12))
        self.target = [prior + scale * change for prior, change in zip(self.target, delta)]
        if math.dist(self.target, robot) > self.max_lead + 1e-9:
            # Discontinuous robot feedback cannot be reconciled with both speed and lead bounds.
            # Recalibrate instead of issuing a jump or silently weakening either bound.
            self.reset()
            return None
        return list(self.target)
