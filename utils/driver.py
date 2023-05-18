import typing as t

from ..sensors import hcsr04, l29x, mpu6050
from . import Drive


class Driver(Drive):
    left_motor = l29x.Motor(direction_pin=20, pwm_pin=16, enable_pin=21)
    right_motor = l29x.Motor(direction_pin=12, pwm_pin=25, enable_pin=24)
    ultrasonic_front = hcsr04.HCSR04(trigger_pin=4, echo_pin=17)
    ultrasonic_right = hcsr04.HCSR04(trigger_pin=4, echo_pin=27)
    ultrasonic_back = hcsr04.HCSR04(trigger_pin=4, echo_pin=18)
    ultrasonic_left = hcsr04.HCSR04(trigger_pin=4, echo_pin=23)
    imu = mpu6050.MPU6050()
    _wall_threshold = 5
    _tile_size = 25
    _error_threshold = 0.5

    def __init__(self, width: int, height: int) -> None:
        self.width = width
        self.height = height

    def _set_speed(self, speed: int) -> None:
        self.left_motor.set_speed(speed)
        self.right_motor.set_speed(speed)

    def _turn(self, left: bool) -> None:
        self._set_speed(100)
        yaw = 0
        while yaw < 90:
            gyro, accel = self.imu.get_gyro_data(), self.imu.get_accel_data()
            yaw = (yaw + mpu6050.calculate_yaw_angle(gyro, accel)) % 360
            self.left_motor.set_direction(not left)
            self.right_motor.set_direction(left)
        self.left_motor.stop()
        self.right_motor.stop()

    def _center(self) -> None:
        self._set_speed(100)
        while True:
            left = self.ultrasonic_left.get_distance()
            right = self.ultrasonic_right.get_distance()
            if abs(left - right) <= self._error_threshold:
                break
            self.left_motor.set_direction(left > right)
            self.right_motor.set_direction(left < right)

    def turn_left(self) -> None:
        self._turn(True)

    def turn_right(self) -> None:
        self._turn(False)

    def move_forward(self, distance: t.Optional[int] = None) -> None:
        front = self.ultrasonic_front.get_distance()
        back = self.ultrasonic_back.get_distance()
        nearer = min(front, back)
        sensor = self.ultrasonic_front if front == nearer else self.ultrasonic_back
        self._set_speed(100)
        distance = distance or self._tile_size
        while abs(sensor.get_distance() - nearer) < distance:
            self.left_motor.set_direction(True)
            self.right_motor.set_direction(True)
        self.left_motor.stop()
        self.right_motor.stop()
        # uncomment when inside maze
        """
        front = self.ultrasonic_front.get_distance()
        if front < self._wall_threshold:
            self._set_speed(100)
            while self.ultrasonic_front.get_distance() < self._wall_threshold:
                self.left_motor.set_direction(False)
                self.right_motor.set_direction(False)
        if abs(self.ultrasonic_left.get_distance() - self.ultrasonic_right.get_distance()) > self._error_threshold:
            self._center()
        """

    def run(self):
        while True:
            choice = input("Enter command: ").lower()
            if choice == "w":
                self.move_forward()
            elif choice == "s":
                self.turn_left()
                self.turn_left()
                self.move_forward()
            elif choice == "a":
                self.turn_left()
            elif choice == "d":
                self.turn_right()
            else:
                break

    def ack_reset(self) -> None:
        raise NotImplementedError

    @property
    def maze_width(self) -> int:
        return self.width

    @property
    def maze_height(self) -> int:
        return self.height

    @property
    def wall_front(self) -> bool:
        return self.ultrasonic_front.get_distance() <= self._wall_threshold

    @property
    def wall_right(self) -> bool:
        return self.ultrasonic_right.get_distance() <= self._wall_threshold

    @property
    def wall_left(self) -> bool:
        return self.ultrasonic_left.get_distance() <= self._wall_threshold


if __name__ == "__main__":
    driver = Driver(width=8, height=8)
    driver.run()
