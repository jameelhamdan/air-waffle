CAMERA_ENABLED = False

DEFAULT_HOST = 'pico1'
DEFAULT_PORT = 7777
DEFAULT_WIFI_SSID = 'Noor'
DEFAULT_WIFI_PASSWORD = '0597380000'


class Mpu:
    enable = True
    id = 1
    SDA = 2
    SCL = 3


class MotorType:
    BRUSHLESS = 'BRUSHLESS'
    SERVO = 'SERVO'


class MotorConfig:
    type: MotorType
    enable = True
    pin = 0
    calibration = 0

    def __init__(self, motor_type, pin: int = 0, calibration: float = 0.0, enable: bool = True):
        self.type = motor_type
        self.pin = pin
        self.calibration = calibration
        self.enable = enable


class Motors:
    MAIN = MotorConfig(MotorType.BRUSHLESS, 6, 0, False)
    RIGHT = MotorConfig(MotorType.SERVO, 8, 0)
    LEFT = MotorConfig(MotorType.SERVO, 9, 0)
