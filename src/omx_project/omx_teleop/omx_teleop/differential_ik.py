"""Position-only differential IK with a small, exact box-constrained convex QP.

Inspired by constrained-control/CBF literature; this is a kinematic adaptation,
not a reproduction of a torque-level controller or a whole-robot safety proof.
The four/five-variable solver uses at most 3**5 active sets, without NumPy/SciPy.
"""
from dataclasses import dataclass
from itertools import product
import math


def solve_linear(matrix, rhs):
    n = len(rhs)
    a = [list(row) + [float(b)] for row, b in zip(matrix, rhs)]
    for column in range(n):
        pivot = max(range(column, n), key=lambda i: abs(a[i][column]))
        if abs(a[pivot][column]) < 1e-14:
            raise ValueError('Singular linear system')
        a[column], a[pivot] = a[pivot], a[column]
        divisor = a[column][column]
        a[column] = [v / divisor for v in a[column]]
        for row in range(n):
            if row != column:
                factor = a[row][column]
                a[row] = [v - factor * w for v, w in zip(a[row], a[column])]
    return [row[-1] for row in a]


def box_qp(hessian, gradient, lower, upper):
    """Minimize .5*x'H*x + g'x on a box. H must be symmetric positive definite."""
    n = len(gradient)
    if (not 1 <= n <= 5 or len(hessian) != n or any(len(row) != n for row in hessian)
            or len(lower) != n or len(upper) != n):
        raise ValueError('QP supports one to five variables')
    values = [*gradient, *lower, *upper, *(v for row in hessian for v in row)]
    if not all(math.isfinite(v) for v in values):
        raise ValueError('Non-finite QP input')
    if any(a > b for a, b in zip(lower, upper)):
        return None
    # Cholesky verifies the precondition needed for KKT sufficiency.
    factor = [[0.0] * n for _ in range(n)]
    for i in range(n):
        for j in range(i + 1):
            if abs(hessian[i][j] - hessian[j][i]) > 1e-10:
                raise ValueError('QP Hessian is not symmetric')
            value = hessian[i][j] - sum(factor[i][k] * factor[j][k] for k in range(j))
            if i == j:
                if value <= 0:
                    raise ValueError('QP Hessian is not positive definite')
                factor[i][j] = math.sqrt(value)
            else:
                factor[i][j] = value / factor[j][j]
    # Try the unconstrained optimum first; typical interior steps need one solve.
    patterns = [(0,) * n]
    patterns.extend(p for p in product((-1, 0, 1), repeat=n) if any(p))
    for pattern in patterns:
        free = [i for i, state in enumerate(pattern) if state == 0]
        fixed = [i for i, state in enumerate(pattern) if state != 0]
        x = [lower[i] if state < 0 else upper[i] if state > 0 else 0.0
             for i, state in enumerate(pattern)]
        if free:
            rhs = [-gradient[i] - sum(hessian[i][j] * x[j] for j in fixed) for i in free]
            reduced = [[hessian[i][j] for j in free] for i in free]
            solution = solve_linear(reduced, rhs)
            for i, value in zip(free, solution):
                x[i] = value
        if any(v < lo - 1e-10 or v > hi + 1e-10 for v, lo, hi in zip(x, lower, upper)):
            continue
        derivative = [gradient[i] + sum(hessian[i][j] * x[j] for j in range(n))
                      for i in range(n)]
        if all((abs(d) <= 1e-8 if state == 0 else d >= -1e-8 if state < 0 else d <= 1e-8)
               for d, state in zip(derivative, pattern)):
            return [max(lo, min(hi, v)) for v, lo, hi in zip(x, lower, upper)]
    raise ArithmeticError('No KKT point found for a feasible positive-definite box QP')


def smallest_singular_value(jacobian):
    """Jacobi eigenvalue iteration on the symmetric 3x3 J*J' matrix."""
    a = [[sum(x * y for x, y in zip(row, other)) for other in jacobian]
         for row in jacobian]
    for _ in range(24):
        p, q = max(((0, 1), (0, 2), (1, 2)), key=lambda ij: abs(a[ij[0]][ij[1]]))
        if abs(a[p][q]) < 1e-15:
            break
        theta = .5 * math.atan2(2 * a[p][q], a[q][q] - a[p][p])
        c, s = math.cos(theta), math.sin(theta)
        app, aqq, apq = a[p][p], a[q][q], a[p][q]
        for k in range(3):
            if k not in (p, q):
                apk, aqk = a[p][k], a[q][k]
                a[p][k] = a[k][p] = c * apk - s * aqk
                a[q][k] = a[k][q] = s * apk + c * aqk
        a[p][p] = c*c*app - 2*s*c*apq + s*s*aqq
        a[q][q] = s*s*app + 2*s*c*apq + c*c*aqq
        a[p][q] = a[q][p] = 0.0
    return math.sqrt(max(0.0, min(a[i][i] for i in range(3))))


@dataclass(frozen=True)
class DifferentialIKConfig:
    gain: float = 2.0
    max_cartesian_speed: float = 0.06
    max_velocity: float = 0.4
    max_acceleration: float = 0.8
    joint_margin: float = 0.08
    barrier_gain: float = 2.0
    damping_min: float = 0.01
    damping_max: float = 0.06
    singularity_threshold: float = 0.04
    temporal_weight: float = 0.0002
    posture_weight: float = 0.0
    posture_gain: float = 0.2

    def __post_init__(self):
        if not all(math.isfinite(v) for v in vars(self).values()):
            raise ValueError('Non-finite differential IK setting')
        if any(getattr(self, name) <= 0 for name in (
                'gain', 'max_cartesian_speed', 'max_velocity', 'max_acceleration',
                'barrier_gain', 'damping_min', 'singularity_threshold')):
            raise ValueError('Differential IK limits must be positive')
        if (self.joint_margin < 0 or self.damping_max < self.damping_min
                or min(self.temporal_weight, self.posture_weight, self.posture_gain) < 0):
            raise ValueError('Invalid regularization or margin')


@dataclass
class VelocityResult:
    velocity: list
    feasible: bool
    sigma_min: float = 0.0
    damping: float = 0.0
    tracking_error: float = 0.0
    active_bounds: int = 0
    reason: str = ''


class DifferentialIK:
    def __init__(self, model, config=None):
        self.model = model
        self.config = config or DifferentialIKConfig()
        if any(hi - lo <= 2 * self.config.joint_margin for lo, hi in model.limits):
            raise ValueError('Joint margin leaves no admissible interval')

    def objective(self, q, target, previous):
        cfg = self.config
        n = len(q)
        current = self.model.forward(q)[:3]
        error = [t - p for t, p in zip(target, current)]
        speed = [cfg.gain * value for value in error]
        norm = math.sqrt(sum(v*v for v in speed))
        speed = [v * min(1.0, cfg.max_cartesian_speed / max(norm, 1e-12)) for v in speed]
        jacobian = self.model.position_jacobian(q)
        sigma = smallest_singular_value(jacobian)
        damping = cfg.damping_min + (cfg.damping_max - cfg.damping_min) * max(
            0.0, 1 - (sigma / cfg.singularity_threshold) ** 2)
        posture = [cfg.posture_gain * ((lo + hi) / 2 - value)
                   for value, (lo, hi) in zip(q, self.model.limits)]
        diagonal = damping ** 2 + cfg.temporal_weight + cfg.posture_weight
        hessian = [[sum(row[i] * row[j] for row in jacobian) + (diagonal if i == j else 0.0)
                    for j in range(n)] for i in range(n)]
        gradient = [-sum(row[i] * value for row, value in zip(jacobian, speed))
                    - cfg.temporal_weight * previous[i] - cfg.posture_weight * posture[i]
                    for i in range(n)]
        return hessian, gradient, sigma, damping, math.sqrt(sum(v*v for v in error))

    def bounds(self, q, previous, dt):
        cfg = self.config
        lower, upper = [], []
        for value, prior, (lo, hi) in zip(q, previous, self.model.limits):
            lo, hi = lo + cfg.joint_margin, hi - cfg.joint_margin
            # Continuous-time CBF-style bounds AND discrete Euler step bounds.
            lower.append(max(-cfg.max_velocity, prior - cfg.max_acceleration * dt,
                             -cfg.barrier_gain * (value - lo), (lo - value) / dt))
            upper.append(min(cfg.max_velocity, prior + cfg.max_acceleration * dt,
                             cfg.barrier_gain * (hi - value), (hi - value) / dt))
        return lower, upper

    def step(self, q, target, previous, dt):
        n = len(self.model.limits)
        if (len(q) != n or len(target) != 3 or len(previous) != n
                or not all(math.isfinite(v) for v in (*q, *target, *previous, dt))
                or not 0 < dt <= 0.1):
            return VelocityResult([0.0] * n, False, reason='Invalid state or control period')
        h, g, sigma, damping, error = self.objective(q, target, previous)
        lower, upper = self.bounds(q, previous, dt)
        solution = box_qp(h, g, lower, upper)
        if solution is None:
            return VelocityResult([0.0] * n, False, sigma, damping, error,
                                  reason='Joint/velocity/acceleration constraints conflict')
        active = sum(abs(v - lo) < 1e-8 or abs(v - hi) < 1e-8
                     for v, lo, hi in zip(solution, lower, upper))
        return VelocityResult(solution, True, sigma, damping, error, active)
