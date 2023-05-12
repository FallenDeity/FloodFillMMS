import dataclasses

import RPi.GPIO as GPIO


@dataclasses.dataclass(kw_only=True)
class Motor:
    direction_pin: int
    pwm_pin: int

    def __post_init__(self) -> None:
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.direction_pin, GPIO.OUT)
        GPIO.setup(self.pwm_pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.pwm_pin, 100)
        self.pwm.start(0)

    def set_speed(self, speed: int) -> None:
        """
        Set motor speed

        Parameters
        ----------
        speed : int
            speed to set
        """
        self.pwm.ChangeDutyCycle(speed)

    def set_direction(self, direction: bool) -> None:
        """
        Set motor direction

        Parameters
        ----------
        direction : bool
            direction to set
        """
        GPIO.output(self.direction_pin, direction)

    def stop(self) -> None:
        """
        Stop motor
        """
        self.pwm.ChangeDutyCycle(0)

    def cleanup(self) -> None:
        """
        Cleanup GPIO
        """
        self.stop()
        GPIO.cleanup()
