import time
from utils import network, helpers
import pygame

# Throttle in percentage
INT_THROTTLE = 20
MAX_THROTTLE = 100
MIN_THROTTLE = 0


class GamepadController:
    """
    Controller Scheme

    ------------------------------------------------------------
    |        Left Thumbstick             Right Trigger         |
    |                                                          |
    |           + PITCH                   + THROTTLE           |
    |                                                          |
    |     - ROLL        + ROLL                                 |
    |                                                          |
    |           - PITCH                   - THROTTLE           |
    |                                                          |
    ------------------------------------------------------------
    """

    def __init__(self, send_callback):
        self.send_callback = send_callback
        self.throttle = INT_THROTTLE
        # [ROLL, PITCH, YAW]
        self.rotation = [0, 0, 0]

        # YAW Rotation factor, currently this is just a flag since we don't have a compass meter yet
        self.rotation_angle = 90
        self.max_angle = 25  # Max angle -/+
        self.cycle_speed = 0.05

        self.AXIS = {
            'ROLL': 0,
            'PITCH': 1,
            'YAW': 2,
            'THROTTLE': 5,  # PS5 trigger
        }

        pygame.init()
        pygame.joystick.init()
        self.joystick = pygame.joystick.Joystick(0)

    def loop(self):
        pygame.event.pump()

        roll_axis = self.joystick.get_axis(self.AXIS['ROLL'])
        if abs(roll_axis):
            self.rotation[0] = int(self.max_angle * roll_axis)
        else:
            self.rotation[0] = 0

        pitch_axis = self.joystick.get_axis(self.AXIS['PITCH']) * -1
        if abs(pitch_axis):
            self.rotation[1] = int(self.max_angle * pitch_axis)
        else:
            self.rotation[1] = 0

        yaw_axis = self.joystick.get_axis(self.AXIS['YAW'])

        if abs(yaw_axis):
            self.rotation[2] = self.rotation_angle * (-1 if yaw_axis < 0 else 1)
        else:
            self.rotation[2] = 0

        # This normalizes -1.0 to 1.0 range to be 0, 1.0 range * 100 is percentage
        throttle_axis = (self.joystick.get_axis(self.AXIS['THROTTLE']) * -1 + 1) / 2.0
        self.throttle = helpers.clamp(round(throttle_axis * 100, 2), MIN_THROTTLE, MAX_THROTTLE)

        self.send_callback(network.NetworkEvent.CONTROL, helpers.encode_control(self.throttle, *self.rotation))
        print(f'{self.throttle}, {self.rotation[0]}, {self.rotation[1]}, {self.rotation[2]}')

    def run(self):
        """
        Actually transform raw inputs into throttle and target angle
        should run every like 50ms
        """
        while True:
            self.loop()
            time.sleep(self.cycle_speed)
