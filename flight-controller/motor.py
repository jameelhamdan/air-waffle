import time
from utils import helpers
from machine import Pin, PWM


class BaseMotor:
    _pwm: PWM

    def __init__(self, config, code=None):
        self.motor_config = config
        self.throttle = 0
        self.code = code
        self._pwm = None

    def halt(self, snooze=1) -> None:
        pass

    def calibrate(self) -> None:
        pass

    def arm(self, snooze: int = 4) -> None:
        pass


class EscMotor(BaseMotor):
    # MAX ESC SPEED
    MAX_THROTTLE = 2400
    # MIN ESC SPEED
    MIN_THROTTLE = 650

    @classmethod
    def throttle_pct_pwm(cls, pct: float) -> int:
        """ Get PWM Throttle from percentage (0.0, 100.0) """
        pct /= 100.0
        diff = cls.MAX_THROTTLE - cls.MIN_THROTTLE
        return int(cls.MIN_THROTTLE + diff * pct)

    def pwm(self, pct: int, calibrated: bool = True, snooze=0):
        throttle = self.throttle_pct_pwm(pct)

        if calibrated:
            throttle += self.motor_config.calibration

        self.throttle = helpers.clamp(throttle, self.MIN_THROTTLE, self.MAX_THROTTLE)

        if self.motor_config.enable:
            if not self._pwm:
                self._pwm = PWM(Pin(self.motor_config.pin), freq=50)
            self._pwm.duty_u16(self.throttle)

        if snooze:
            time.sleep(snooze)
        return

    def calibrate(self) -> None:
        """
        This trains the ESC on the full scale (max - min range) of the controller / pulse generator.
        This only needs to be done when changing controllers, transmitters, etc. not upon every power-on.
        NB: if already calibrated, full throttle will be applied (briefly)!  Disconnect propellers, etc.
        """
        self.pwm(throttle=self.MAX_THROTTLE)
        self.pwm(throttle=self.MAX_THROTTLE, snooze=2)  # Official docs: "about 2 seconds".
        self.pwm(throttle=self.MIN_THROTTLE, snooze=4)  # Time eno

    def arm(self, snooze: int = 4) -> None:
        """
        Arms the ESC. Required upon every power cycle.
        """

        # Time enough for the cell count, etc. beeps to play.
        self.pwm(throttle=self.MIN_THROTTLE, snooze=snooze)

    def halt(self, snooze=1) -> None:
        """
        Switch off the GPIO, and un-arm the ESC.
        Ensure this runs, even on unclean shutdown.
        """
        self.pwm(throttle=self.MIN_THROTTLE, snooze=snooze)  # This 1 sec seems to *hasten* shutdown.
        self.pwm(0)


class ServoMotor(BaseMotor):
    def goto(self, pct: int, calibrated: bool = True):
        angle = pct * 1.8

        if calibrated:
            angle += self.motor_config.calibration

        if angle < -180:
            angle = -180
        if angle > 180:
            angle = 180

        def servo_map(x, in_min, in_max, out_min, out_max):
            return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

        value = round(servo_map(angle, -180, 180, 0, 1024))

        min_val = 2500
        max_val = 7500

        if value < 0:
            value = 0
        if value > 1024:
            value = 1024
        delta = max_val - min_val
        target = int(min_val + ((value / 1024) * delta))
        self.throttle = target
        self._pwm.duty_u16(self.throttle)

    def pwm(self, pct: int, calibrated: bool = True, snooze=0):
        if self.motor_config.enable:
            if not self._pwm:
                self._pwm = PWM(Pin(self.motor_config.pin), freq=50)
            self.goto(pct, calibrated)

        if snooze:
            time.sleep(snooze)
