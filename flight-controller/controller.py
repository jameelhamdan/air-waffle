import utime
import config
from utils import network, helpers


def ease_in(value: int, target: int):
    dist = target - value
    return value + (dist * 0.7)


class FixedWingController:
    """
    Singleton for Flight controller
    """

    # Update cycles in MS
    cycle_speed = 50
    proportional_gain = 5.0  # TODO: auto calibrate or something
    int_throttle = 800
    max_throttle = 1800
    min_throttle = 600

    # Time duration before resetting controls to idle
    time_before_reset = 250  # ms

    offset_angle = -45

    def __init__(self, sensor, *motors):
        """
        sensor: a Mpu instance,
        motors: Motor instances (Main Throttle, Right Servo, Left Servo)
        """

        self.sensor = sensor
        self.motors = motors
        self.motor_main = motors[0]
        self.motor_right = motors[1]
        self.motor_left = motors[2]

        # THROTTLE
        self.throttle = self.int_throttle
        self.TARGET_ROLL_ANGLE = 0
        self.TARGET_PITCH_ANGLE = 0
        self.TARGET_YAW_ANGLE = 0

        self.update_timestamp = None
        self.set_rotation()

    def arm_motors(self):
        """
        Arm all motors at once, this will shorten arming time
        """
        for motor in self.motors:
            motor.arm(0)
        utime.sleep(1)

    def set_rotation(self, roll: float = 0, pitch: float = 0, yaw: float = 0):
        """
        Set Target Rotation
        """
        roll, pitch, yaw = helpers.rotate_on_z([roll, pitch, yaw], self.offset_angle)

        self.update_timestamp = int(utime.ticks_ms())
        self.TARGET_ROLL_ANGLE, self.TARGET_PITCH_ANGLE, self.TARGET_YAW_ANGLE = pitch, pitch, yaw

    @classmethod
    def throttle_pct_pwm(cls, percentage: float) -> int:
        """ Get PWM Throttle from percentage (0.0, 100.0) """
        percentage /= 100.0
        diff = cls.max_throttle - cls.min_throttle
        return int(cls.min_throttle + diff * percentage)

    def set_throttle(self, throttle: int):
        """
        Set target Throttle
        """
        self.update_timestamp = int(utime.ticks_ms())
        self.throttle = throttle

    @property
    def angles(self):
        """
        Return current rotation from sensor
        """
        # TODO: Implement MPU with functional z axis
        if not config.Mpu.enable:
            return [0, 0, 0]

        r, p, y = self.sensor.angles
        return [r, p, 0]

    def control(self):
        """
        Balance and Accelerate towards target rotation and throttle
        """
        rotation_vector = helpers.rotate_on_z(self.angles, self.offset_angle)

        # Convert x y vector to left and right evelon angles

        self.motor_left.pwm(self.throttle)
        self.motor_right.pwm(self.throttle)

        print("XYZ ROTATION: ", str(self.sensor.angles))

    def idle(self):
        self.set_throttle(self.throttle_pct_pwm(50))
        self.set_rotation()

    def halt(self):
        for motor in self.motors:
            motor.halt()

    def run_event(self, event, data):
        if event == network.NetworkEvent.STOP:
            self.set_throttle(self.min_throttle)
            self.set_rotation()

            for motor in self.motors:
                motor.halt(snooze=0)
            return

        elif event == network.NetworkEvent.CONTROL:
            throttle_pct, roll, pitch, yaw = helpers.decode_control(data)
            self.set_throttle(self.throttle_pct_pwm(throttle_pct))
            self.set_rotation(roll, pitch, yaw)
            return
        elif event in [network.NetworkEvent.CONNECTED]:
            print('\nCONNECTED TO CLIENT\n')
            # NOOP
            return

        raise Exception(f'EVENT {event} UNKNOWN')

    def loop(self):
        if config.Mpu.enable:
            self.sensor.loop()
        if self.update_timestamp and self.update_timestamp + self.time_before_reset > int(utime.ticks_ms()):
            self.idle()

        self.control()

    def run(self):
        while True:
            self.loop()
            print("controller loop")
            utime.sleep_ms(self.cycle_speed)
