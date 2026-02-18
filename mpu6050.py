import smbus2
import time

class MPU6050:
    def __init__(self, bus_number=1, address=0x68):
        self.bus = smbus2.SMBus(bus_number)
        self.address = address
        self.PWR_MGMT_1 = 0x6B
        
        # Wake up the MPU-6050 (It starts in sleep mode)
        self.bus.write_byte_data(self.address, self.PWR_MGMT_1, 0)

    def _read_raw_data(self, addr):
        # Read two bytes (high and low) and combine them
        high = self.bus.read_byte_data(self.address, addr)
        low = self.bus.read_byte_data(self.address, addr + 1)
        value = (high << 8) | low
        
        # Convert to signed integer (handling negative values)
        if value > 32768:
            value = value - 65536
        return value

    def get_data(self):
        # --- Read Raw Data ---
        # Accelerometer registers start at 0x3B
        acc_x = self._read_raw_data(0x3B)
        acc_y = self._read_raw_data(0x3D)
        acc_z = self._read_raw_data(0x3F)

        # Gyroscope registers start at 0x43
        gyro_x = self._read_raw_data(0x43)
        gyro_y = self._read_raw_data(0x45)
        gyro_z = self._read_raw_data(0x47)

        # --- Convert to Standard Units ---
        # Default scale: Accel / 16384.0 (for +/- 2g)
        # Default scale: Gyro / 131.0 (for +/- 250 deg/s)
        data = {
            'Ax': acc_x / 16384.0,
            'Ay': acc_y / 16384.0,
            'Az': acc_z / 16384.0,
            'Gx': gyro_x / 131.0,
            'Gy': gyro_y / 131.0,
            'Gz': gyro_z / 131.0
        }
        return data






