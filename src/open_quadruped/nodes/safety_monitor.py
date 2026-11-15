"""Stateless-input safety state classifier, independent of ROS."""
from enum import Enum

class SafetyState(str, Enum):
    OK='OK'; WARNING='WARNING'; FALL='FALL'

class SafetyMonitor:
    def __init__(self, warning_angle=.25, fall_angle=.65, effort_limit=4.0, tracking_warning=.25):
        self.warning_angle=warning_angle; self.fall_angle=fall_angle
        self.effort_limit=effort_limit; self.tracking_warning=tracking_warning

    def evaluate(self, roll=0., pitch=0., angular_speed=0., contacts=None,
                 tracking_error=0., efforts=None):
        if max(abs(roll),abs(pitch))>=self.fall_angle or (contacts is not None and sum(contacts.values())<2):
            return SafetyState.FALL
        if (max(abs(roll),abs(pitch))>=self.warning_angle or abs(angular_speed)>1.5
            or (contacts is not None and not all(contacts.values()))
            or abs(tracking_error)>=self.tracking_warning
            or (efforts is not None and any(abs(v)>=self.effort_limit for v in efforts))):
            return SafetyState.WARNING
        return SafetyState.OK
