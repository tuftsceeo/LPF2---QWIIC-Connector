import SoftwareI2C
import pins as Pin
i2c = SoftwareI2C(scl_pin=Pin.TX, sda_pin=Pin.RX)

while(1)
    print(i2c.scan())

print("Scanning I2C bus...", i2c.scan())
