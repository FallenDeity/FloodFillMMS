from __future__ import annotations

import dataclasses
import enum
import math
import time
import typing as t

import smbus

__all__: t.Tuple[str, ...] = (
    "MPU6050",
    "REGISTERS",
    "SCALES",
    "Accelerometer",
    "Gyroscope",
    "KalmanFilter",
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


class KalmanFilter:
    q_angle: float = 0.001
    q_bias: float = 0.003
    r_measure: float = 0.03

    def __init__(self, angle: float = 0.0, bias: float = 0.0):
        self.angle = angle
        self.bias = bias
        self.rate = 0.0
        self.P = [[0.0, 0.0], [0.0, 0.0]]

    def update(self, new_angle: float, new_rate: float, dt: float) -> float:
        self.rate = new_rate - self.bias
        self.angle += dt * self.rate
        self.P[0][0] += dt * (dt * self.P[1][1] - self.P[0][1] - self.P[1][0] + self.q_angle)
        self.P[0][1] -= dt * self.P[1][1]
        self.P[1][0] -= dt * self.P[1][1]
        self.P[1][1] += self.q_bias * dt
        y = new_angle - self.angle
        s = self.P[0][0] + self.r_measure
        k = [self.P[0][0] / s, self.P[1][0] / s]
        self.angle += k[0] * y
        self.bias += k[1] * y
        p00_temp, p01_temp = self.P[0][0], self.P[0][1]
        self.P[0][0] -= k[0] * p00_temp
        self.P[0][1] -= k[0] * p01_temp
        self.P[1][0] -= k[1] * p00_temp
        self.P[1][1] -= k[1] * p01_temp
        return self.angle


@dataclasses.dataclass(kw_only=True, frozen=True, slots=True)
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

    @property
    def magnitude(self) -> float:
        """
        Magnitude of the accelerometer data

        Returns
        -------
        float
            magnitude of the accelerometer data
        """
        return math.sqrt(self.x**2 + self.y**2 + self.z**2)

    @property
    def pitch(self) -> float:
        """
        Pitch angle in degrees

        Returns
        -------
        float
            pitch angle in degrees
        """
        return math.degrees(math.atan2(self.x, math.sqrt(self.y**2 + self.z**2)))

    @property
    def roll(self) -> float:
        """
        Roll angle in degrees

        Returns
        -------
        float
            roll angle in degrees
        """
        return math.degrees(math.atan2(self.y, math.sqrt(self.x**2 + self.z**2)))

    @property
    def yaw(self) -> float:
        """
        Yaw angle in degrees

        Returns
        -------
        float
            yaw angle in degrees
        """
        return math.degrees(math.atan2(self.z, math.sqrt(self.x**2 + self.y**2)))

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


@dataclasses.dataclass(kw_only=True, frozen=True, slots=True)
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


class MPU6050:
    __slots__: tuple[str, ...] = (
        "address",
        "bus_number",
        "bus",
    )

    def __init__(self, address: int = 0x68, bus_number: int = 1) -> None:
        self.address = address
        self.bus_number = bus_number
        self.bus = smbus.SMBus(self.bus_number)
        self._init_mpu6050()

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
        return Gyroscope(x=x, y=y, z=z)


if __name__ == "__main__":
    kalman_x = KalmanFilter()
    kalman_y = KalmanFilter()
    mpu = MPU6050()
    accel = mpu.get_accel_data()
    kalman_x.angle = accel.roll
    kalman_y.angle = accel.pitch
    timer = time.time()
    while True:
        accel = mpu.get_accel_data()
        gyro = mpu.get_gyro_data()
        dt = time.time() - timer
        timer = time.time()
        roll = kalman_x.update(accel.roll, gyro.x, dt)
        pitch = kalman_y.update(accel.pitch, gyro.y, dt)
        print(f"roll: {roll:.2f} pitch: {pitch:.2f}")
        time.sleep(1e-3)
