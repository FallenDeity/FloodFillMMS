import keyboard

from ..sensors import mdd10a


class Driver:
    left_motor = mdd10a.Motor(direction_pin=23, pwm_pin=24)
    right_motor = mdd10a.Motor(direction_pin=18, pwm_pin=25)

    def _set_speed(self, speed: int) -> None:
        self.left_motor.set_speed(speed)
        self.right_motor.set_speed(speed)

    def run(self):
        while True:
            if keyboard.is_pressed("w"):
                self._set_speed(100)
                self.left_motor.set_direction(True)
                self.right_motor.set_direction(True)
            elif keyboard.is_pressed("s"):
                self._set_speed(100)
                self.left_motor.set_direction(False)
                self.right_motor.set_direction(False)
            elif keyboard.is_pressed("a"):
                self._set_speed(100)
                self.left_motor.set_direction(False)
                self.right_motor.set_direction(True)
            elif keyboard.is_pressed("d"):
                self._set_speed(100)
                self.left_motor.set_direction(True)
                self.right_motor.set_direction(False)
            else:
                self.left_motor.stop()
                self.right_motor.stop()


if __name__ == "__main__":
    driver = Driver()
    driver.run()
