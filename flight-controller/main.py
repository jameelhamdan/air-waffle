#!/usr/bin/python3
"""
FLIGHT CONTROLLER ON RP2040 to control the motors and sensors

SHOULD RUN ON PI STARTUP and beep
MOTOR GPIO MAPPING - CLOCK WISE


       12           10               11
----- left -----|| main ||-------- right ----
---------------------------------------------
               ||||||||||
                ||||||||
                 ||||||
                 |||||
                 |||||
                 |||||
                 |||||

10 is the main thurst brushless motor

11 and 12 are the left and right evelons

Y is vertical or YAW
"""
from utils import telemetry
import machine
import network
import sensor
import controller
import radio
import motor
import utime
import config
import rp2
import _thread


def listen_server_func(event, data):
    """
    Mapping events to controller actions
    """
    if CONTROLLER:
        CONTROLLER.run_event(event, data)
    else:
        print('CONTROLLER NOT INITIALIZED YET')


def connect_to_wifi():
    rp2.country('AU')

    wlan = network.WLAN(network.STA_IF)
    wlan.config(hostname=config.DEFAULT_HOST)
    wlan.active(True)
    wlan.connect(config.DEFAULT_WIFI_SSID, config.DEFAULT_WIFI_PASSWORD)

    while wlan.isconnected() == False:
        print('Waiting for wifi...')
        utime.sleep_ms(500)
    print("Connected to", wlan.ifconfig()[0])


def setup_server():
    # TODO Replace network.Server with serial input to communicate between controller and computer
    global SERVER_THREAD, CONTROLLER

    print('CONNECTING TO NETWORK')
    connect_to_wifi()

    print('STARTING SERVER...')
    server_handler = radio.ServerConnection(target_func=listen_server_func, host=config.DEFAULT_HOST, port=int(config.DEFAULT_PORT))

    print('STARTING TELEMETRY...')
    telemetry_handler = telemetry.Telemetry(server_handler.send, controller=CONTROLLER)

    server_handler.connect()

    return server_handler, telemetry_handler


def setup_controller():
    global CONTROLLER

    print('STARTING FLIGHT CONTROLLER...')
    CONTROLLER = controller.FixedWingController(SENSOR, main=MOTOR_MAIN, right=MOTOR_RIGHT, left=MOTOR_LEFT)
    print('ARMING MOTORS...')
    CONTROLLER.arm_motors()
    print('FINISHED ARMING!')


def main():
    """
    MAIN PROGRAM LOOP
    launch server, controller and hardware
    """
    print('INITIALIZING DRONE...')
    setup_controller()
    server_handler, telemetry_handler = setup_server()

    print('\n...\nREADY TO FLY!')

    def run_controller_loop():
        while True:
            CONTROLLER.loop()
            utime.sleep_ms(CONTROLLER.cycle_speed)

    _thread.start_new_thread(run_controller_loop, ())

    while True:
        server_handler.loop()
        telemetry_handler.loop()
        utime.sleep_ms(telemetry_handler.cycle_speed)


if __name__ == "__main__":
    led = machine.Pin("LED", machine.Pin.OUT)
    led.on()
    # These are here since they actually do something on init
    MOTOR_MAIN = motor.EscMotor(config.Motors.MAIN, code='M')
    MOTOR_RIGHT = motor.ServoMotor(config.Motors.RIGHT, code='R')
    MOTOR_LEFT = motor.ServoMotor(config.Motors.LEFT, code='L')

    if config.Mpu.enable:
        SENSOR = sensor.Mpu(flip=True, invert_y=True)
    else:
        SENSOR = None

    # Function exists as to not pollute the global namespace
    try:
        main()
    except KeyboardInterrupt:
        CONTROLLER.halt()
        print('\nStopping flight controller')


