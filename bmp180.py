import smbus2
import time

# --- BMP180 Driver Class ---
class BMP180:
    def __init__(self, bus_number=1, address=0x77):
        self.bus = smbus2.SMBus(bus_number)
        self.address = address
        self.cal = {}
        self._load_calibration()

    def _read_word(self, reg):
        # Reads a signed 16-bit integer (2 bytes)
        msb = self.bus.read_byte_data(self.address, reg)
        lsb = self.bus.read_byte_data(self.address, reg + 1)
        value = (msb << 8) + lsb
        if value >= 0x8000:
            value = -((65535 - value) + 1)
        return value

    def _read_unsigned_word(self, reg):
        # Reads an unsigned 16-bit integer
        msb = self.bus.read_byte_data(self.address, reg)
        lsb = self.bus.read_byte_data(self.address, reg + 1)
        return (msb << 8) + lsb

    def _load_calibration(self):
        # Load factory calibration coefficients from the sensor's EEPROM
        try:
            self.cal['AC1'] = self._read_word(0xAA)
            self.cal['AC2'] = self._read_word(0xAC)
            self.cal['AC3'] = self._read_word(0xAE)
            self.cal['AC4'] = self._read_unsigned_word(0xB0)
            self.cal['AC5'] = self._read_unsigned_word(0xB2)
            self.cal['AC6'] = self._read_unsigned_word(0xB4)
            self.cal['B1']  = self._read_word(0xB6)
            self.cal['B2']  = self._read_word(0xB8)
            self.cal['MB']  = self._read_word(0xBA)
            self.cal['MC']  = self._read_word(0xBC)
            self.cal['MD']  = self._read_word(0xBE)
        except OSError:
            print("Error: Could not read calibration data. Check wiring!")

    def read_all(self, sea_level_pressure=101325):
        # 1. Read Raw Temperature
        self.bus.write_byte_data(self.address, 0xF4, 0x2E)
        time.sleep(0.005)
        UT = self._read_unsigned_word(0xF6)

        # 2. Calculate True Temperature
        X1 = ((UT - self.cal['AC6']) * self.cal['AC5']) >> 15
        X2 = (self.cal['MC'] << 11) // (X1 + self.cal['MD'])
        B5 = X1 + X2
        temperature = ((B5 + 8) >> 4) / 10.0

        # 3. Read Raw Pressure (Standard Mode)
        self.bus.write_byte_data(self.address, 0xF4, 0x34 + (0 << 6))
        time.sleep(0.005)
        MSB = self.bus.read_byte_data(self.address, 0xF6)
        LSB = self.bus.read_byte_data(self.address, 0xF7)
        XLSB = self.bus.read_byte_data(self.address, 0xF8)
        UP = ((MSB << 16) + (LSB << 8) + XLSB) >> (8 - 0)

        # 4. Calculate True Pressure
        B6 = B5 - 4000
        X1 = (self.cal['B2'] * (B6 * B6 >> 12)) >> 11
        X2 = (self.cal['AC2'] * B6) >> 11
        X3 = X1 + X2
        B3 = (((self.cal['AC1'] * 4 + X3) << 0) + 2) >> 2

        X1 = (self.cal['AC3'] * B6) >> 13
        X2 = (self.cal['B1'] * ((B6 * B6) >> 12)) >> 16
        X3 = ((X1 + X2) + 2) >> 2
        B4 = (self.cal['AC4'] * (X3 + 32768)) >> 15

        B7 = (UP - B3) * (50000 >> 0)

        if B7 < 0x80000000:
            p = (B7 * 2) // B4
        else:
            p = (B7 // B4) * 2

        X1 = (p >> 8) * (p >> 8)
        X1 = (X1 * 3038) >> 16
        X2 = (-7357 * p) >> 16
        pressure = p + ((X1 + X2 + 3791) >> 4)

        # 5. Calculate Altitude
        # Formula: Altitude = 44330 * (1 - (p / p0)^(1/5.255))
        altitude = 44330.0 * (1.0 - pow(pressure / sea_level_pressure, 0.1903))

        return temperature, pressure, altitude

# --- Main Execution Block ---
if __name__ == "__main__":
    print("Initializing BMP180 Sensor...")
    
    try:
        # Create sensor instance
        sensor = BMP180()
        
        # Header for the data table
        print("-" * 60)
        print(f"{'Temperature':<15} | {'Pressure (hPa)':<18} | {'Altitude':<15}")
        print("-" * 60)

        while True:
            # Read all three values
            temp, pressure, alt = sensor.read_all()

            # Format and print (Pressure converted to hPa by dividing by 100)
            print(f"{temp:<15.2f} | {pressure/100.0:<18.2f} | {alt:<15.2f} m")
            
            # Wait 1 second before next reading
            time.sleep(1)

    except KeyboardInterrupt:
        print("\nProgram stopped by user.")
    except Exception as e:
        print(f"\nAn error occurred: {e}")
