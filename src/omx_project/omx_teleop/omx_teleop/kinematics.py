"""Analytical 4-axis IK using the geometry and limits of the loaded URDF."""
from dataclasses import dataclass
import math
import xml.etree.ElementTree as ET

JOINTS = ('joint1', 'joint2', 'joint3', 'joint4')


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class ArmGeometry:
    base_x: float
    shoulder_z: float
    upper_x: float
    upper_z: float
    forearm: float
    tool: float
    limits: tuple

    @property
    def joint_names(self):
        return JOINTS

    @classmethod
    def from_urdf(cls, xml):
        root = ET.fromstring(xml)
        joints = {j.attrib['name']: j for j in root.findall('joint')}

        def origin(name):
            node = joints[name].find('origin')
            xyz = tuple(float(v) for v in node.get('xyz', '0 0 0').split())
            rpy = tuple(float(v) for v in node.get('rpy', '0 0 0').split())
            if len(xyz) != 3 or len(rpy) != 3 or any(abs(v) > 1e-9 for v in rpy):
                raise ValueError('IK requires unrotated OMX joint origins')
            return xyz

        o1, o2, o3, o4, ot = [origin(n) for n in (*JOINTS, 'end_effector_joint')]
        expected_axes = ((0, 0, 1), (0, 1, 0), (0, 1, 0), (0, 1, 0))
        for name, axis in zip(JOINTS, expected_axes):
            actual = tuple(float(v) for v in joints[name].find('axis').get('xyz').split())
            if actual != axis:
                raise ValueError('Unsupported joint axis: ' + name)
        if any(abs(v) > 1e-9 for v in (o1[1], o2[0], o2[1], o3[1],
                                        o4[1], o4[2], ot[1], ot[2])):
            raise ValueError('Unsupported OMX chain geometry')
        limits = tuple((float(joints[n].find('limit').get('lower')),
                        float(joints[n].find('limit').get('upper'))) for n in JOINTS)
        values = (*o1, *o2, *o3, *o4, *ot, *(v for pair in limits for v in pair))
        if not all(math.isfinite(v) for v in values):
            raise ValueError('Non-finite geometry')
        if o3[2] <= 0 or o4[0] <= 0 or ot[0] <= 0 or any(lo >= hi for lo, hi in limits):
            raise ValueError('Invalid geometry or limits')
        return cls(o1[0], o1[2] + o2[2], o3[0], o3[2], o4[0], ot[0], limits)

    def forward(self, q):
        """Return x,y,z in link1 and downward-positive tool pitch."""
        q1, q2, q3, q4 = q
        pitch = q2 + q3 + q4
        radial = (self.upper_x * math.cos(q2) + self.upper_z * math.sin(q2)
                  + self.forearm * math.cos(q2 + q3) + self.tool * math.cos(pitch))
        z = (self.shoulder_z - self.upper_x * math.sin(q2)
             + self.upper_z * math.cos(q2) - self.forearm * math.sin(q2 + q3)
             - self.tool * math.sin(pitch))
        return (self.base_x + radial * math.cos(q1), radial * math.sin(q1), z, pitch)

    def solutions(self, x, y, z, pitch):
        if not all(math.isfinite(v) for v in (x, y, z, pitch)):
            return []
        x = x - self.base_x
        rho = math.hypot(x, y)
        yaw = math.atan2(y, x)
        upper = math.hypot(self.upper_x, self.upper_z)
        delta = math.atan2(self.upper_x, self.upper_z)
        result = []
        for q1, radial in ((yaw, rho), (wrap(yaw + math.pi), -rho)):
            dr = radial - self.tool * math.cos(pitch)
            dz = z + self.tool * math.sin(pitch) - self.shoulder_z
            cosine = (dr * dr + dz * dz - upper * upper - self.forearm ** 2) / (
                2 * upper * self.forearm)
            if abs(cosine) > 1 + 1e-10:
                continue
            for gamma in (math.acos(max(-1, min(1, cosine))),
                          -math.acos(max(-1, min(1, cosine)))):
                alpha = math.atan2(dr, dz) - math.atan2(
                    self.forearm * math.sin(gamma), upper + self.forearm * math.cos(gamma))
                q2 = wrap(alpha - delta)
                q3 = wrap(gamma - math.pi / 2 + delta)
                q4 = wrap(pitch - q2 - q3)
                q = (q1, q2, q3, q4)
                if all(lo - 1e-9 <= v <= hi + 1e-9 for v, (lo, hi) in zip(q, self.limits)):
                    q = tuple(max(lo, min(hi, v)) for v, (lo, hi) in zip(q, self.limits))
                    check = self.forward(q)
                    if math.dist(check[:3], (x + self.base_x, y, z)) < 1e-7:
                        result.append(q)
        return result

    def position_jacobian(self, q):
        """Analytic 3x4 translational Jacobian, expressed in link1."""
        yaw, shoulder, elbow, wrist = q
        pitch = shoulder + elbow + wrist
        radial = (self.upper_x * math.cos(shoulder) + self.upper_z * math.sin(shoulder)
                  + self.forearm * math.cos(shoulder + elbow) + self.tool * math.cos(pitch))
        dr = [
            -self.upper_x * math.sin(shoulder) + self.upper_z * math.cos(shoulder)
            - self.forearm * math.sin(shoulder + elbow) - self.tool * math.sin(pitch),
            -self.forearm * math.sin(shoulder + elbow) - self.tool * math.sin(pitch),
            -self.tool * math.sin(pitch),
        ]
        dz = [
            -self.upper_x * math.cos(shoulder) - self.upper_z * math.sin(shoulder)
            - self.forearm * math.cos(shoulder + elbow) - self.tool * math.cos(pitch),
            -self.forearm * math.cos(shoulder + elbow) - self.tool * math.cos(pitch),
            -self.tool * math.cos(pitch),
        ]
        return [[-radial * math.sin(yaw), *(v * math.cos(yaw) for v in dr)],
                [radial * math.cos(yaw), *(v * math.sin(yaw) for v in dr)],
                [0.0, *dz]]

    def solve_position(self, xyz, seed, pitches):
        candidates = [q for pitch in (sum(seed[1:]), *pitches)
                      for q in self.solutions(*xyz, pitch)]
        return min(candidates, key=lambda q: sum((a - b) ** 2 for a, b in zip(q, seed)),
                   default=None)


def load_arm_model(xml):
    """Select only the explicitly supported geometries; no silent four/five-axis remap."""
    root = ET.fromstring(xml)
    if root.get('name') == 'omx_f':
        from .omx_f_kinematics import OMXFGeometry
        return OMXFGeometry.from_urdf(xml)
    if root.get('name') not in (None, 'open_manipulator_x'):
        raise ValueError('Unsupported robot model: ' + str(root.get('name')))
    return ArmGeometry.from_urdf(xml)
