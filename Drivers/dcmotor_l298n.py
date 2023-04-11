import RPi.GPIO as gp
from time import sleep
class L298N:
    def __init__(self, in1=None, in2=None, en1=None, in3=None, in4=None, en2=None):
        # in1 = 24
        # in2 = 23
        # en1 = 25

        # in3 = 24
        # in4 = 23
        # en2 = 25

        self.motor_1 = {"in1": in1, "in2": in2, "en": en1}  # [in1, in2, en1]
        self.motor_2 = {"in1": in3, "in2": in4, "en": en2}  # [in3, in4, en2]
        self.pwm_1 = None
        self.pwm_2 = None
        self.speeds = {1: 50, 2: 50}

    def setup_motor(self, motor_num):
        motor = getattr(self, f"motor_{motor_num}")
        in1, in2, en = motor["in1"], motor["in2"], motor["en"]
        if all([in1, in2, en]):
            gp.setmode(gp.BCM)
            gp.setup(in1, gp.OUT)
            gp.setup(in2, gp.OUT)
            gp.setup(en, gp.OUT)
            gp.output(in1, gp.LOW)
            gp.output(in2, gp.LOW)
            setattr(self, f"pwm_{motor_num}", gp.PWM(en, 1000))
        else:
            print(f"Pins not given for motor {motor}")

    def forward(self, motor_num):
        motor = getattr(self, f"motor_{motor_num}")
        in1, in2 = motor["in1"], motor["in2"]
        gp.output(in1, gp.HIGH)
        gp.output(in2, gp.LOW)
        print(f"forward motor {motor_num}")

    def backward(self, motor_num):
        motor = getattr(self, f"motor_{motor_num}")
        in1, in2 = motor["in1"], motor["in2"]
        gp.output(in1, gp.LOW)
        gp.output(in2, gp.HIGH)
        print(f"backward motor {motor_num}")

    def start_motor(self, motor_num):
        pwm = getattr(self, f"pwm_{motor_num}")
        speed = self.speeds[motor_num]
        if pwm:
            pwm.start(speed)
        else:
            print(f"Motor {motor_num} not active")

    def stop_motor(self, motor_num):
        pwm = getattr(self, f"pwm_{motor_num}")
        if pwm:
            pwm.stop()
        else:
            print(f"Motor {motor_num} not active")

    def change_speed(self, motor_num, speed):
        self.speeds[motor_num] = speed
        pwm = getattr(self, f"pwm_{motor_num}")
        if pwm:
            pwm.ChangeDutyCycle(speed)
        else:
            print(f"Motor {motor_num} not active")

    def quit(self):
        gp.cleanup()
