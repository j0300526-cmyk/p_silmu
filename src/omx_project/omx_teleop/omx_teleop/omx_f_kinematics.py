"""Five-axis OMX-F geometry, including the roll joint and offset TCP."""
from dataclasses import dataclass
import math
import xml.etree.ElementTree as ET

from .kinematics import ArmGeometry, JOINTS, wrap


@dataclass(frozen=True)
class OMXFGeometry(ArmGeometry):
    tool_y: float

    @property
    def joint_names(self):
        return (*JOINTS, 'joint5')

    @classmethod
    def from_urdf(cls, xml):
        root = ET.fromstring(xml)
        joints = {joint.get('name'): joint for joint in root.findall('joint')}
        names = (*JOINTS, 'joint5', 'end_effector_joint')
        origins = []
        for name in names:
            origin = joints[name].find('origin')
            xyz = tuple(float(v) for v in origin.get('xyz', '0 0 0').split())
            rpy = tuple(float(v) for v in origin.get('rpy', '0 0 0').split())
            if len(xyz) != 3 or len(rpy) != 3 or any(abs(v) > 1e-9 for v in rpy):
                raise ValueError('Unsupported OMX-F origin')
            origins.append(xyz)
        for name, expected in zip(names[:5], ((0, 0, 1), (0, 1, 0), (0, 1, 0), (0, 1, 0), (1, 0, 0))):
            actual = tuple(float(v) for v in joints[name].find('axis').get('xyz').split())
            if actual != expected:
                raise ValueError('Unsupported OMX-F joint axis: ' + name)
        a, b, c, d, e, tcp = origins
        if any(abs(v) > 1e-9 for v in (a[1], b[0], b[1], c[1], d[1], d[2], e[1], e[2], tcp[2])):
            raise ValueError('Unsupported OMX-F chain offsets')
        limits = tuple((float(joints[n].find('limit').get('lower')),
                        float(joints[n].find('limit').get('upper'))) for n in names[:5])
        if (not all(math.isfinite(v) for row in (*origins, *limits) for v in row)
                or min(c[2], d[0], e[0], tcp[0]) <= 0 or any(lo >= hi for lo, hi in limits)):
            raise ValueError('Invalid OMX-F geometry')
        return cls(a[0], a[2] + b[2], c[0], c[2], d[0], e[0] + tcp[0], limits, tcp[1])

    def forward(self, q):
        yaw, shoulder, elbow, wrist, roll = q
        pitch = shoulder + elbow + wrist
        lateral = self.tool_y * math.cos(roll)
        vertical = self.tool_y * math.sin(roll)
        radial = (self.upper_x * math.cos(shoulder) + self.upper_z * math.sin(shoulder)
                  + self.forearm * math.cos(shoulder + elbow)
                  + self.tool * math.cos(pitch) + vertical * math.sin(pitch))
        z = (self.shoulder_z - self.upper_x * math.sin(shoulder)
             + self.upper_z * math.cos(shoulder) - self.forearm * math.sin(shoulder + elbow)
             - self.tool * math.sin(pitch) + vertical * math.cos(pitch))
        return (self.base_x + math.cos(yaw)*radial - math.sin(yaw)*lateral,
                math.sin(yaw)*radial + math.cos(yaw)*lateral, z, pitch)

    def position_jacobian(self, q):
        yaw, shoulder, elbow, wrist, roll = q
        pitch = shoulder + elbow + wrist
        cy, sy = math.cos(yaw), math.sin(yaw)
        lateral, vertical = self.tool_y * math.cos(roll), self.tool_y * math.sin(roll)
        radial = (self.upper_x * math.cos(shoulder) + self.upper_z * math.sin(shoulder)
                  + self.forearm * math.cos(shoulder + elbow)
                  + self.tool * math.cos(pitch) + vertical * math.sin(pitch))
        dr_tool = -self.tool*math.sin(pitch) + vertical*math.cos(pitch)
        dz_tool = -self.tool*math.cos(pitch) - vertical*math.sin(pitch)
        dr = [-self.upper_x*math.sin(shoulder) + self.upper_z*math.cos(shoulder)
              - self.forearm*math.sin(shoulder+elbow) + dr_tool,
              -self.forearm*math.sin(shoulder+elbow) + dr_tool, dr_tool]
        dz = [-self.upper_x*math.cos(shoulder) - self.upper_z*math.sin(shoulder)
              - self.forearm*math.cos(shoulder+elbow) + dz_tool,
              -self.forearm*math.cos(shoulder+elbow) + dz_tool, dz_tool]
        roll_r = lateral*math.sin(pitch)
        roll_y = -self.tool_y*math.sin(roll)
        roll_z = lateral*math.cos(pitch)
        return [[-sy*radial-cy*lateral, *(cy*v for v in dr), cy*roll_r-sy*roll_y],
                [cy*radial-sy*lateral, *(sy*v for v in dr), sy*roll_r+cy*roll_y],
                [0.0, *dz, roll_z]]

    def solutions(self, x, y, z, pitch, roll=0.0):
        if not all(math.isfinite(v) for v in (x, y, z, pitch, roll)):
            return []
        if not self.limits[4][0] <= roll <= self.limits[4][1]:
            return []
        dx = x-self.base_x
        lateral = self.tool_y*math.cos(roll)
        vertical = self.tool_y*math.sin(roll)
        radial2 = dx*dx+y*y-lateral*lateral
        if radial2 < -1e-10:
            return []
        upper = math.hypot(self.upper_x, self.upper_z)
        delta = math.atan2(self.upper_x, self.upper_z)
        candidates = []
        for radial in (math.sqrt(max(0, radial2)), -math.sqrt(max(0, radial2))):
            yaw = math.atan2(y, dx) - math.atan2(lateral, radial)
            dr = radial-self.tool*math.cos(pitch)-vertical*math.sin(pitch)
            dz = z+self.tool*math.sin(pitch)-vertical*math.cos(pitch)-self.shoulder_z
            cosine = (dr*dr+dz*dz-upper*upper-self.forearm*self.forearm)/(2*upper*self.forearm)
            if abs(cosine) > 1+1e-10:
                continue
            for gamma in (math.acos(max(-1,min(1,cosine))), -math.acos(max(-1,min(1,cosine)))):
                alpha = math.atan2(dr,dz)-math.atan2(
                    self.forearm*math.sin(gamma), upper+self.forearm*math.cos(gamma))
                q2 = wrap(alpha-delta)
                q3 = wrap(gamma-math.pi/2+delta)
                q4 = wrap(pitch-q2-q3)
                for winding in (-1,0,1):
                    q = (wrap(yaw)+winding*2*math.pi, q2, q3, q4, roll)
                    if all(lo-1e-9 <= v <= hi+1e-9 for v,(lo,hi) in zip(q,self.limits)):
                        if math.dist(self.forward(q)[:3],(x,y,z)) < 1e-7:
                            candidates.append(q)
        return candidates

    def solve_position(self, xyz, seed, pitches):
        candidates = [q for pitch in (sum(seed[1:4]), *pitches)
                      for q in self.solutions(*xyz, pitch, roll=seed[4])]
        return min(candidates, key=lambda q: sum((a-b)**2 for a,b in zip(q,seed)), default=None)
