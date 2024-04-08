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
    int_throttle = 30
    max_throttle = 100
    min_throttle = 0

    # Time duration before resetting controls to idle
    time_before_reset = 250  # ms

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
        # ROLL, PITCH, YAW
        self.target = [0, 0, 0]

        self.update_timestamp = None
        self.set_rotation(*self.target)

    def arm_motors(self):
        """
        Arm all motors at once, this will shorten arming time
        """
        for motor in self.motors:
            motor.arm(0)
        utime.sleep(1)

    def set_rotation(self, roll, pitch, yaw):
        """
        Set Target Rotation
        """
        self.update_timestamp = int(utime.ticks_ms())
        self.target = [roll, pitch, yaw]

    def set_throttle(self, throttle: int):
        """
        Set target Throttle Percentage
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
        # Convert x y vector to left and right evelons angles
        self.motor_left.pwm(self.target[0] * 1.8)
        self.motor_right.pwm(self.target[1] * 1.8)
        self.motor_main.pwm(self.throttle)

    def halt(self):
        for motor in self.motors:
            motor.halt()

    def run_event(self, event, data):
        if event == network.NetworkEvent.STOP:
            self.set_throttle(0)
            self.set_rotation(0, 0, 0)

            for motor in self.motors:
                motor.halt(snooze=0)
            return

        elif event == network.NetworkEvent.CONTROL:
            throttle_pct, roll, pitch, yaw = helpers.decode_control(data)
            self.set_throttle(throttle_pct)
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

        self.control()

    def run(self):
        while True:
            self.loop()
            utime.sleep_ms(self.cycle_speed)
