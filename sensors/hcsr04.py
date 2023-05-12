from __future__ import annotations

import dataclasses
import time

import RPi.GPIO as GPIO


@dataclasses.dataclass(kw_only=True, frozen=True, slots=True)
class HCSR04:
    trigger_pin: int
    echo_pin: int

    def __post_init__(self) -> None:
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.trigger_pin, GPIO.OUT)
        GPIO.setup(self.echo_pin, GPIO.IN)

    def get_distance(self) -> float:
        """
        Get distance in cm

        Returns
        -------
        float
            distance in cm
        """
        GPIO.output(self.trigger_pin, True)
        time.sleep(1e-5)
        GPIO.output(self.trigger_pin, False)
        start_time = time.perf_counter()
        stop_time = time.perf_counter()
        while GPIO.input(self.echo_pin) == 0:
            start_time = time.perf_counter()
        while GPIO.input(self.echo_pin) == 1:
            stop_time = time.perf_counter()
        elapsed_time = stop_time - start_time
        distance = (elapsed_time * 34300) / 2
        return distance

    @staticmethod
    def cleanup() -> None:
        """
        Cleanup GPIO
        """
        GPIO.cleanup()
