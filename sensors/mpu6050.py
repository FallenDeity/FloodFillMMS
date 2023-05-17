from __future__ import annotations

import dataclasses
import enum
import math
import time
import typing as t

import smbus
from pyquaternion import Quaternion

__all__: t.Tuple[str, ...] = (
    "MPU6050",
    "REGISTERS",
    "SCALES",
    "Accelerometer",
    "Gyroscope",
)


class REGISTERS(enum.IntEnum):
    """
    MPU6050 registers
    """

    PWR_MGMT_1 = 0x6B
    PWR_MGMT_2 = 0x6C

    ACCEL_XOUT0 = 0x3B
    ACCEL_YOUT0 = 0x3D
    ACCEL_ZOUT0 = 0x3F

    TEMP_OUT0 = 0x41

    GYRO_XOUT0 = 0x43
    GYRO_YOUT0 = 0x45
    GYRO_ZOUT0 = 0x47

    SMPLRT_DIV = 0x19

    ACCEL_CONFIG = 0x1C
    GYRO_CONFIG = 0x1B
    CONFIG = 0x1A


class SCALES(enum.IntEnum):
    """
    MPU6050 scales
    """

    GRAVIY_MS2 = 9.80665  # m/s^2

    ACCEL_SCALE_MODIFIER_2G = 16384  # LSB/g
    ACCEL_SCALE_MODIFIER_4G = 8192  # LSB/g
    ACCEL_SCALE_MODIFIER_8G = 4096  # LSB/g
    ACCEL_SCALE_MODIFIER_16G = 2048  # LSB/g

    GYRO_SCALE_MODIFIER_250DEG = 131  # LSB/deg/s
    GYRO_SCALE_MODIFIER_500DEG = 65.5  # LSB/deg/s
    GYRO_SCALE_MODIFIER_1000DEG = 32.8  # LSB/deg/s
    GYRO_SCALE_MODIFIER_2000DEG = 16.4  # LSB/deg/s


class RANGES(enum.IntEnum):
    """
    MPU6050 ranges
    """

    ACCEL_RANGE_2G = 0x00
    ACCEL_RANGE_4G = 0x08
    ACCEL_RANGE_8G = 0x10
    ACCEL_RANGE_16G = 0x18

    GYRO_RANGE_250DEG = 0x00
    GYRO_RANGE_500DEG = 0x08
    GYRO_RANGE_1000DEG = 0x10
    GYRO_RANGE_2000DEG = 0x18

    FILTER_250HZ = 0x00
    FILTER_184HZ = 0x01
    FILTER_92HZ = 0x02
    FILTER_41HZ = 0x03
    FILTER_20HZ = 0x04
    FILTER_10HZ = 0x05
    FILTER_5HZ = 0x06


@dataclasses.dataclass
class Accelerometer:
    """
    Accelerometer data

    Attributes
    ----------
    x : float
        x-axis data in m/s^2
    y : float
        y-axis data in m/s^2
    z : float
        z-axis data in m/s^2
    """

    x: float
    y: float
    z: float

    def __add__(self, other: Accelerometer) -> Accelerometer:
        return Accelerometer(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other: Accelerometer) -> Accelerometer:
        return Accelerometer(self.x - other.x, self.y - other.y, self.z - other.z)

    def __truediv__(self, other: float) -> Accelerometer:
        return Accelerometer(self.x / other, self.y / other, self.z / other)

    def to_g(self) -> tuple[float, float, float]:
        """
        Convert to g

        Returns
        -------
        tuple[float, float, float]
            accelerometer data in g
        """
        x, y, z = (v / float(SCALES.GRAVIY_MS2) for v in (self.x, self.y, self.z))
        return x, y, z


@dataclasses.dataclass
class Gyroscope:
    """
    Gyroscope data

    Attributes
    ----------
    x : float
        x-axis data in deg/s
    y : float
        y-axis data in deg/s
    z : float
        z-axis data in deg/s
    """

    x: float
    y: float
    z: float

    def __truediv__(self, other: float) -> Gyroscope:
        return Gyroscope(self.x / other, self.y / other, self.z / other)

    def __sub__(self, other: Gyroscope) -> Gyroscope:
        return Gyroscope(self.x - other.x, self.y - other.y, self.z - other.z)

    def __add__(self, other: Gyroscope) -> Gyroscope:
        return Gyroscope(self.x + other.x, self.y + other.y, self.z + other.z)

    def to_rad(self) -> tuple[float, float, float]:
        """
        Convert to radians

        Returns
        -------
        tuple[float, float, float]
            gyroscope data in radians
        """
        x, y, z = (v * (180 / math.pi) for v in (self.x, self.y, self.z))
        return x, y, z


class MPU6050:
    __slots__: tuple[str, ...] = (
        "address",
        "bus_number",
        "bus",
        "_accelerometer_calibration",
        "_gyroscope_calibration",
        "_calibrated",
    )

    def __init__(self, address: int = 0x68, bus_number: int = 1) -> None:
        self.address = address
        self.bus_number = bus_number
        self.bus = smbus.SMBus(self.bus_number)
        self._accelerometer_calibration = Accelerometer(0, 0, 0)
        self._gyroscope_calibration = Gyroscope(0, 0, 0)
        self._calibrated = False
        self._init_mpu6050()
        self._calibrate()

    def _calibrate(self, iterations: int = 200) -> None:
        print("Calibrating MPU6050...")
        for _ in range(iterations):
            self._accelerometer_calibration += self.get_accel_data()
            self._gyroscope_calibration += self.get_gyro_data()
            time.sleep(1e-2)
        self._accelerometer_calibration /= iterations
        self._gyroscope_calibration /= iterations
        self._accelerometer_calibration.z -= SCALES.GRAVIY_MS2
        self._calibrated = True
        print("Calibration complete")

    def _init_mpu6050(self) -> None:
        """
        Initialize MPU6050
        """
        self.bus.write_byte_data(self.address, REGISTERS.PWR_MGMT_1, 0x00)
        self.bus.write_byte_data(self.address, REGISTERS.SMPLRT_DIV, 0x07)
        self.bus.write_byte_data(self.address, REGISTERS.CONFIG, 0x06)
        self.bus.write_byte_data(self.address, REGISTERS.GYRO_CONFIG, 0x18)
        self.bus.write_byte_data(self.address, REGISTERS.ACCEL_CONFIG, 0x01)

    def read_raw_data(self, register: REGISTERS) -> int:
        """
        Read raw data from register

        Parameters
        ----------
        register : REGISTERS
            register to read from

        Returns
        -------
        int
            raw data
        """
        high = int(self.bus.read_byte_data(self.address, int(register)))  # type: ignore
        low = int(self.bus.read_byte_data(self.address, int(register) + 1))  # type: ignore
        value = (high << 8) + low
        return value if value < 32768 else value - 65536

    def get_temp(self) -> float:
        """
        Get temperature in degrees Celsius

        :return: temperature in degrees Celsius
        """
        temp = self.read_raw_data(REGISTERS.TEMP_OUT0)
        return temp / 340 + 36.53

    def set_accel_range(self, accel_range: RANGES = RANGES.ACCEL_RANGE_2G) -> None:
        """
        Set accelerometer range

        Parameters
        ----------
        accel_range : RANGES
            accelerometer range, by default RANGES.ACCEL_RANGE_2G
        """
        self.bus.write_byte_data(self.address, REGISTERS.ACCEL_CONFIG, int(RANGES.ACCEL_RANGE_2G))
        self.bus.write_byte_data(self.address, REGISTERS.ACCEL_CONFIG, int(accel_range))

    def set_gyro_range(self, gyro_range: RANGES = RANGES.GYRO_RANGE_250DEG) -> None:
        """
        Set gyroscope range

        Parameters
        ----------
        gyro_range : RANGES
            gyroscope range, by default RANGES.GYRO_RANGE_250DEG
        """
        self.bus.write_byte_data(self.address, REGISTERS.GYRO_CONFIG, int(RANGES.GYRO_RANGE_250DEG))
        self.bus.write_byte_data(self.address, REGISTERS.GYRO_CONFIG, int(gyro_range))

    def set_filter_range(self, filter_range: RANGES = RANGES.FILTER_250HZ) -> None:
        """
        Set filter range

        Parameters
        ----------
        filter_range : RANGES, optional
            filter range, by default RANGES.FILTER_250HZ
        """
        ext_sync_set = int(self.bus.read_byte_data(self.address, REGISTERS.CONFIG) & 0b00111000)  # type: ignore
        self.bus.write_byte_data(self.address, REGISTERS.CONFIG, ext_sync_set | int(filter_range))

    def get_accel_range(self) -> RANGES:
        """
        Get accelerometer range

        Returns
        -------
        RANGES
            accelerometer range
        """
        return RANGES(self.bus.read_byte_data(self.address, REGISTERS.ACCEL_CONFIG))

    def get_gyro_range(self) -> RANGES:
        """
        Get gyroscope range

        Returns
        -------
        RANGES
            gyroscope range
        """
        return RANGES(self.bus.read_byte_data(self.address, REGISTERS.GYRO_CONFIG))

    def get_accel_data(self) -> Accelerometer:
        """
        Get accelerometer data returns Accelerometer object

        Returns
        -------
        Accelerometer
            accelerometer data
        """
        args = (
            REGISTERS.ACCEL_XOUT0,
            REGISTERS.ACCEL_YOUT0,
            REGISTERS.ACCEL_ZOUT0,
        )
        x, y, z = (self.read_raw_data(arg) for arg in args)
        accel_range = self.get_accel_range()
        mapping = {
            RANGES.ACCEL_RANGE_2G: SCALES.ACCEL_SCALE_MODIFIER_2G,
            RANGES.ACCEL_RANGE_4G: SCALES.ACCEL_SCALE_MODIFIER_4G,
            RANGES.ACCEL_RANGE_8G: SCALES.ACCEL_SCALE_MODIFIER_8G,
            RANGES.ACCEL_RANGE_16G: SCALES.ACCEL_SCALE_MODIFIER_16G,
        }
        accel_scale = mapping.get(accel_range, SCALES.ACCEL_SCALE_MODIFIER_2G)
        x, y, z = ((value * SCALES.GRAVIY_MS2) / accel_scale for value in (x, y, z))
        if self._calibrated:
            return Accelerometer(x=x, y=y, z=z) - self._accelerometer_calibration
        return Accelerometer(x=x, y=y, z=z)

    def get_gyro_data(self) -> Gyroscope:
        """
        Get gyroscope data returns Gyroscope object

        Returns
        -------
        Gyroscope
            gyroscope data
        """
        args = (
            REGISTERS.GYRO_XOUT0,
            REGISTERS.GYRO_YOUT0,
            REGISTERS.GYRO_ZOUT0,
        )
        x, y, z = (self.read_raw_data(arg) for arg in args)
        gyro_range = self.get_gyro_range()
        mapping = {
            RANGES.GYRO_RANGE_250DEG: SCALES.GYRO_SCALE_MODIFIER_250DEG,
            RANGES.GYRO_RANGE_500DEG: SCALES.GYRO_SCALE_MODIFIER_500DEG,
            RANGES.GYRO_RANGE_1000DEG: SCALES.GYRO_SCALE_MODIFIER_1000DEG,
            RANGES.GYRO_RANGE_2000DEG: SCALES.GYRO_SCALE_MODIFIER_2000DEG,
        }
        gyro_scale = mapping.get(gyro_range, SCALES.GYRO_SCALE_MODIFIER_250DEG)
        x, y, z = (value / gyro_scale for value in (x, y, z))
        if self._calibrated:
            return Gyroscope(x=x, y=y, z=z) - self._gyroscope_calibration
        return Gyroscope(x=x, y=y, z=z)


def calculate_yaw_angle(gyro: Gyroscope, acc: Accelerometer, dt: float) -> float:
    """
    Calculate yaw angle

    Parameters
    ----------
    gyro : Gyroscope
        gyroscope data
    acc : Accelerometer
        accelerometer data
    dt : float
        time interval

    Returns
    -------
    float
        yaw angle
    """
    accel_quat = Quaternion(axis=(acc.x, acc.y, acc.z), angle=0.0)
    gyro_quat = Quaternion(axis=(0, 0, 1), angle=gyro.z * dt)
    return (gyro_quat * accel_quat).yaw_pitch_roll[0]


if __name__ == "__main__":
    mpu = MPU6050()
    yaw = 0.0
    dt = 0.01
    while True:
        yaw += calculate_yaw_angle(mpu.get_gyro_data(), mpu.get_accel_data(), dt)
        print(yaw % 360)
        time.sleep(dt)
