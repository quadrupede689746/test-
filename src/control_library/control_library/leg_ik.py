"""Geometric inverse kinematics for the four OpenQuadruped legs."""

import math


class LegIKModel:
    def __init__(self, upper, lower, off0, off1):
        """Create a leg model using link lengths and hip offsets in millimetres."""
        self.upper = float(upper)
        self.lower = float(lower)
        self.off0 = float(off0)
        self.off1 = float(off1)
        if min(self.upper, self.lower, self.off0, self.off1) <= 0.0:
            raise ValueError('All leg dimensions must be positive')

    @staticmethod
    def _safe_asin(value, label):
        # Permit only tiny floating-point excursions outside [-1, 1].
        if value < -1.0 - 1e-8 or value > 1.0 + 1e-8:
            raise ValueError(f'{label} is outside the reachable workspace')
        return math.asin(max(-1.0, min(1.0, value)))

    @staticmethod
    def _safe_acos(value, label):
        if value < -1.0 - 1e-8 or value > 1.0 + 1e-8:
            raise ValueError(f'{label} is outside the reachable workspace')
        return math.acos(max(-1.0, min(1.0, value)))

    def ja_from_htf_vecs(self, htf_vecs):
        """Return [hip, upper-leg, lower-leg] angles for each hip-to-foot vector.

        Raises ValueError if a requested vector is outside the leg workspace.
        The caller can then reject the entire pose instead of publishing a
        partially filled JointAngles message.
        """
        joint_angles = []
        for index, vector in enumerate(htf_vecs):
            if len(vector) != 3:
                raise ValueError(f'Leg {index} vector must contain x, y and z')
            x, y, z = (float(component) for component in vector)
            if not all(math.isfinite(value) for value in (x, y, z)):
                raise ValueError(f'Leg {index} vector contains a non-finite value')

            h1 = math.hypot(self.off0, self.off1)
            h2 = math.hypot(z, y)
            if h2 <= 1e-9:
                raise ValueError(f'Leg {index} hip-to-foot vector is singular')

            alpha_0 = math.atan2(y, z)
            alpha_1 = math.atan(self.off1 / self.off0)
            alpha_2 = math.atan(self.off0 / self.off1)
            alpha_3_arg = h1 * math.sin(alpha_2 + math.radians(90.0)) / h2
            alpha_3 = self._safe_asin(alpha_3_arg, f'Leg {index} hip geometry')
            alpha_4 = math.pi - (alpha_3 + alpha_2 + math.radians(90.0))
            alpha_5 = alpha_1 - alpha_4
            theta_h = alpha_0 - alpha_5

            sin_alpha_3 = math.sin(alpha_3)
            if abs(sin_alpha_3) <= 1e-9:
                raise ValueError(f'Leg {index} hip geometry is singular')
            r0 = h1 * math.sin(alpha_4) / sin_alpha_3
            h = math.hypot(r0, x)
            if h <= 1e-9:
                raise ValueError(f'Leg {index} upper-leg geometry is singular')

            phi = self._safe_asin(x / h, f'Leg {index} sagittal geometry')
            theta_s_arg = (h * h + self.upper * self.upper - self.lower * self.lower) / (2.0 * h * self.upper)
            theta_s = self._safe_acos(theta_s_arg, f'Leg {index} upper-leg angle') - phi
            theta_w_arg = (self.lower * self.lower + self.upper * self.upper - h * h) / (2.0 * self.lower * self.upper)
            theta_w = self._safe_acos(theta_w_arg, f'Leg {index} lower-leg angle')
            joint_angles.append([theta_h, theta_s, theta_w])

        return joint_angles
