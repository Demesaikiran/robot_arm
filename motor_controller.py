from typing import Dict, List
import time
from sc08a import SC08A
from dcmotor_l298n import L298N


class Driver:
    def __init__(self, dcmotor_pins: List[int], servo_port: str, servo_pin: int):
        self.l298n = L298N(*dcmotor_pins)
        self.servo = SC08A(servo_port, 9600)
        self.servo.init_all_motors()
        self.pin = servo_pin
        self.l298n.setup_motor(1)
        self.l298n.start_motor(1)

    def set_pos_speed(self, pos, speed):
        old_pos = self.servo.get_pos(self.pin)
        func = "forward" if (pos - old_pos) > 0 else "backward"
        self.servo.set_pos_speed(self.pin, pos, speed)
        self.l298n.start_motor(1)
        self.l298n.change_speed(1, speed)
        if func == "forward":
            self.l298n.forward(1)
            print("Going forward")
        else:
            self.l298n.backward(1)
            print("Going backward")
        while self.servo.get_pos(self.pin) != pos:
            time.sleep(.1)
        self.l298n.stop_motor(1)
        print("Stopping")