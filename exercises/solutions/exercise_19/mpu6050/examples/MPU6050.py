# MPU6050.py
import struct
from machine import I2C, Pin
from utime import sleep_ms

class MPU6050:
    def __init__(self, addr=0x68, i2c=None, debug=False):
        self.addr = addr
        self.debug = debug
        # Initialize I2C if not provided (adjust pins as needed for your ESP32)
        if i2c is None:
            self.i2c = I2C(1, scl=Pin(22), sda=Pin(21), freq=400000)
        else:
            self.i2c = i2c

        # Wake up MPU6050 (it starts in sleep mode)
        self.i2c.writeto_mem(self.addr, 0x6B, b'\x00')
        
        # Initialize software offsets (these are used by calibration routines)
        self._ax_offset = 0
        self._ay_offset = 0
        self._az_offset = 0
        self._gx_offset = 0
        self._gy_offset = 0
        self._gz_offset = 0

    def testConnection(self):
        # The WHO_AM_I register (0x75) should return 0x68 if connected correctly
        try:
            who_am_i = self.i2c.readfrom_mem(self.addr, 0x75, 1)[0]
            if self.debug:
                print("WHO_AM_I: 0x{:02x}".format(who_am_i))
            return who_am_i == 0x68
        except Exception as e:
            if self.debug:
                print("Error reading WHO_AM_I:", e)
            return False

    # Getter methods for accelerometer offsets
    def getXAccelOffset(self):
        return self._ax_offset

    def getYAccelOffset(self):
        return self._ay_offset

    def getZAccelOffset(self):
        return self._az_offset

    # Getter methods for gyroscope offsets
    def getXGyroOffset(self):
        return self._gx_offset

    def getYGyroOffset(self):
        return self._gy_offset

    def getZGyroOffset(self):
        return self._gz_offset

    # Setter methods for offsets
    def setXAccelOffset(self, offset):
        self._ax_offset = offset

    def setYAccelOffset(self, offset):
        self._ay_offset = offset

    def setZAccelOffset(self, offset):
        self._az_offset = offset

    def setXGyroOffset(self, offset):
        self._gx_offset = offset

    def setYGyroOffset(self, offset):
        self._gy_offset = offset

    def setZGyroOffset(self, offset):
        self._gz_offset = offset

    def getMotion6(self):
        # Read 14 bytes from ACCEL_XOUT_H (0x3B) to GYRO_ZOUT_H (0x48)
        data = self.i2c.readfrom_mem(self.addr, 0x3B, 14)
        # Unpack the data (big-endian, 7 short integers)
        ax, ay, az, temp, gx, gy, gz = struct.unpack('>hhhhhhh', data)
        # Subtract stored offsets from the raw data
        ax -= self._ax_offset
        ay -= self._ay_offset
        az -= self._az_offset
        gx -= self._gx_offset
        gy -= self._gy_offset
        gz -= self._gz_offset
        return (ax, ay, az, gx, gy, gz)

    def CalibrateAccel(self, iterations):
        # A simple calibration: average several readings to determine offsets.
        sum_ax = sum_ay = sum_az = 0
        for i in range(iterations):
            ax, ay, az, _, _, _ = self.getMotion6()
            sum_ax += ax
            sum_ay += ay
            sum_az += az
            sleep_ms(5)
        avg_ax = sum_ax // iterations
        avg_ay = sum_ay // iterations
        avg_az = sum_az // iterations
        # Assuming the sensor is stationary and flat, ax and ay should be 0, az ~16384 (1g)
        self._ax_offset = avg_ax
        self._ay_offset = avg_ay
        self._az_offset = avg_az - 16384
        if self.debug:
            print("Calibrated Accel Offsets: X={}, Y={}, Z={}".format(self._ax_offset, self._ay_offset, self._az_offset))

    def CalibrateGyro(self, iterations):
        # A simple calibration: average several readings for the gyroscope offsets.
        sum_gx = sum_gy = sum_gz = 0
        for i in range(iterations):
            _, _, _, gx, gy, gz = self.getMotion6()
            sum_gx += gx
            sum_gy += gy
            sum_gz += gz
            sleep_ms(5)
        self._gx_offset = sum_gx // iterations
        self._gy_offset = sum_gy // iterations
        self._gz_offset = sum_gz // iterations
        if self.debug:
            print("Calibrated Gyro Offsets: X={}, Y={}, Z={}".format(self._gx_offset, self._gy_offset, self._gz_offset))

    def PrintActiveOffsets(self):
        print("Accel Offsets: X={}, Y={}, Z={}".format(self._ax_offset, self._ay_offset, self._az_offset))
        print("Gyro Offsets:  X={}, Y={}, Z={}".format(self._gx_offset, self._gy_offset, self._gz_offset))
