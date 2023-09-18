import machine
from time import sleep
import ustruct
sda=machine.Pin(16)
scl=machine.Pin(17)
i2c=machine.I2C(0,sda=sda, scl=scl, freq=400000)

print('Scan i2c bus...')
devices = i2c.scan()

if len(devices) == 0:
    print("No i2c device !")
else:
    print('i2c devices found:',len(devices))

for device in devices:
    print("Decimal address: ",device," | Hexa address: ",hex(device))

# I2C address of the LSM6DS33
lsm6ds33_address = 0x6B # Use this slave addr for gyroscope and accelerometer

# Acc X,Y,Z axes enabled
CTRL9_XL_Reg = 0x18 # Reg ADDR
CTRL9_XL_val = 0x38 # 
i2c.writeto_mem(lsm6ds33_address, CTRL9_XL_Reg, bytes([CTRL9_XL_val]))


# Acc = 416Hz (High-Performance mode)
CTRL1_XL_Reg = 0x10 # Reg ADDR
CTRL1_XL_val = 0x60
i2c.writeto_mem(lsm6ds33_address, CTRL1_XL_Reg, bytes([CTRL1_XL_val]))

# Acc Data Ready interrupt on INT1
INT1_CTRL_REG = 0x0D # Reg ADDR
INT1_CTRL_val = 0x01
i2c.writeto_mem(lsm6ds33_address, INT1_CTRL_REG, bytes([INT1_CTRL_val]))


STATUS_REG = 0x1E
SENSITIVITY_2G = 1.0 / 256  # (g/LSB)
EARTH_GRAVITY = 9.80665     # Earth's gravity in [m/s^2]
while True:
    val = i2c.readfrom_mem(lsm6ds33_address, STATUS_REG, 1) # Read 1 byte
    # Extract the 0th bit
    bit_0 = (val[0] >> 0) & 1
    if bit_0: # If new sets of data available
        data = i2c.readfrom_mem(lsm6ds33_address, 0x28, 6) # Output data, each axis takes up 16 bits or 2 bytes
        print("Accel data:" + str(data))
    sleep(1)
    
