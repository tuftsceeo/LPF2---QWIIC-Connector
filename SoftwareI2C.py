
from hub import pins as Pin
from hub import uart
import time
import struct

class SoftwareI2C:
    SW_I2C_WAIT_TIME = 40  # Microseconds to wait, adjust based on your needs
    def __init__(self, scl_pin, sda_pin):
        self.en =  Pin.init(0, Pin.EN, Pin.OUT)
        self.en.value(1)
        self.scl = Pin.init(0, scl_pin, Pin.OUT)
        self.sda = Pin.init(0, sda_pin, Pin.OUT)
        self.scl.value(1)
        self.sda.value(1)

    def scl_high(self):
        self.scl.value(1)

    def scl_low(self):
        self.scl.value(0)

    def sda_high(self):
        self.sda.value(1)

    def sda_low(self):
        self.sda.value(0)

    def sda_input(self):
        self.sda = Pin.init(0, Pin.RX, Pin.IN)

    def sda_output(self):
        self.sda = Pin.init(0, Pin.RX, Pin.OUT)

    def delay_us(self, us):
        time.sleep_us(us)

    def start(self):
        self.sda_high()
        self.scl_high()
        self.delay_us(self.SW_I2C_WAIT_TIME)
        self.sda_low()
        self.delay_us(self.SW_I2C_WAIT_TIME)
        self.scl_low()
        self.delay_us(self.SW_I2C_WAIT_TIME * 2)

    def stop(self):
        self.sda_low()
        self.scl_high()
        self.delay_us(self.SW_I2C_WAIT_TIME)
        self.sda_high()
        self.delay_us(self.SW_I2C_WAIT_TIME)

    def check_ack(self):
        self.sda_input()
        self.scl_high()
        ack = not self.sda.value()
        self.scl_low()
        self.sda_output()
        self.delay_us(self.SW_I2C_WAIT_TIME)
        return ack

    def write_byte(self, byte):
        self.scl_low()
        for i in range(8):
            self.sda.value((byte >> (7 - i)) & 1)
            self.delay_us(self.SW_I2C_WAIT_TIME)
            self.scl_high()
            self.delay_us(self.SW_I2C_WAIT_TIME)
            self.scl_low()
        return self.check_ack()

    def read_byte(self, ack=True):
        self.sda_input()
        byte = 0
        for i in range(8):
            self.scl_high()
            byte = (byte << 1) | self.sda.value()
            self.scl_low()
        self.sda_output()
        if ack:
            self.sda_low()
        else:
            self.sda_high()
        self.scl_high()
        self.delay_us(self.SW_I2C_WAIT_TIME)
        self.scl_low()
        self.sda_high()
        return byte

    def scan(self):
        found_devices = []
        for address in range(0x00, 0x78):  # Valid I2C addresses
            self.start()
            #print("add")
            #print(address)
            if self.write_byte(address << 1):  # Shift address for write mode
                #print("found")
                found_devices.append(address)
            self.stop()
        return found_devices

    def writeto(i2c, address, data):
        """
        Write data to an I2C device.
        :param i2c: SoftwareI2C object instance.
        :param address: 7-bit I2C device address.
        :param data: Bytearray or list of data to write.
        """
        i2c.start()
        if i2c.write_byte(address << 1):  # Shift address for write mode and send
            for byte in data:
                if not i2c.write_byte(byte):
                    print("Error: No ACK received after data byte.")
                    break
        else:
            print("Error: No ACK received for address.")
        i2c.stop()

    def readfrom(i2c, address, num_bytes):
        """
        Read data from an I2C device.
        :param i2c: SoftwareI2C object instance.
        :param address: 7-bit I2C device address.
        :param num_bytes: Number of bytes to read.
        :return: Data read as a bytearray.
        """
        data = bytearray()
        i2c.start()
        if i2c.write_byte((address << 1) | 1):  # Shift address for read mode and send
            for i in range(num_bytes):
                ack = i < num_bytes - 1  # ACK all but the last byte
                data.append(i2c.read_byte(ack))
        else:
            print("Error: No ACK received for address.")
        i2c.stop()
        return data

    def writeto_mem(i2c, device_addr, register_addr, data):
        """
        Write data to a specific register of an I2C device.
        :param i2c: SoftwareI2C object instance, the custom I2C implementation.
        :param device_addr: The 7-bit address of the I2C device.
        :param register_addr: The register address within the I2C device where data will be written.
        :param data: The data to write (as a bytes object or list of bytes).
        """
        i2c.start()  # Start I2C communication
        # Send the device address in write mode
        if not i2c.write_byte(device_addr << 1):
            print("Error: No ACK received for device address.")
            i2c.stop()
            return
        # Send the register address
        if not i2c.write_byte(register_addr):
            print("Error: No ACK received for register address.")
            i2c.stop()
            return
        # Write the data bytes
        for byte in data:
            if not i2c.write_byte(byte):
                print("Error: No ACK received after data byte.")
                break
        i2c.stop()  # Stop I2C communication

    def readfrom_mem(i2c, address, register, num_bytes):
        """
        Read data from a specific register of an I2C device.
        :param i2c: The SoftwareI2C object instance.
        :param address: The 7-bit address of the I2C device.
        :param register: The register address within the device from which to read.
        :param num_bytes: The number of bytes to read from the register.
        :return: A bytearray containing the data read from the device.
        """
        # Start the I2C communication and send the device address in write mode
        i2c.start()
        if not i2c.write_byte(address << 1):  # Shift address for write mode
            print("Error: Device not acknowledging write mode.")
            i2c.stop()
            return
        # Write the register address to read from
        if not i2c.write_byte(register):
            print("Error: Device not acknowledging register address.")
            i2c.stop()
            return
        # Repeated start to switch to read mode
        i2c.start()
        if not i2c.write_byte((address << 1) | 1):  # Shift address for read mode
            print("Error: Device not acknowledging read mode.")
            i2c.stop()
            return
        # Read the specified number of bytes
        data = bytearray()
        for i in range(num_bytes):
            ack = i < num_bytes - 1  # ACK for all but the last byte
            byte = i2c.read_byte(ack)
            data.append(byte)
        # Stop the I2C communication
        i2c.stop()
        return data
