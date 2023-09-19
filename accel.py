import machine
from time import sleep
import struct
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


def calibration(i2c, lsm6ds33_address, scale_factor):
    raw_data = i2c.readfrom_mem(lsm6ds33_address, 0x28, 6) # Output data, each axis takes up 16 bits or 2 bytes
    x_raw, y_raw, z_raw = struct.unpack('<hhh', raw_data) # Unpack binary data form
    x_acc = round(x_raw * scale_factor, 1) # Due to the direction chip is facing and the gravity
    y_acc = round(y_raw * scale_factor, 1)# Make the data into unit of g
    z_acc = round(z_raw * scale_factor, 1)
    return [x_acc, y_acc, z_acc]


scale_factor = (2.0 * 2) / (2 ** 15)  # g/LSB
cal_axis = calibration(i2c, lsm6ds33_address, scale_factor)
x_cal = cal_axis[0]
y_cal = cal_axis[1]
z_cal = cal_axis[2]
x_dis = 0
y_dis = 0
z_dis = 0
while True:
    raw_data = i2c.readfrom_mem(lsm6ds33_address, 0x28, 6) # Output data, each axis takes up 16 bits or 2 bytes
    x_raw, y_raw, z_raw = struct.unpack('<hhh', raw_data) # Unpack binary data form
    # Gives us true acceleration
    x_acc = round(x_raw * scale_factor, 1) - x_cal 
    y_acc = round(y_raw * scale_factor, 1) - y_cal
    z_acc = round(z_raw * scale_factor, 1) - z_cal
    x_dis += round(0.5 * (x_acc * 9.81), 2) # t^2 doesn't matter because we refresh at t = 1
    y_dis += round(0.5 * (y_acc * 9.81), 2)
    z_dis += round(0.5 * (z_acc * 9.81), 2)
    print("X: " + str(x_dis) + " meters")
    print("Y: " + str(y_dis) + " meters")
    print("Z: " + str(z_dis) + " meters")
        # We find distance using d = 1/2at^2
    sleep(1)
    
# Turn raw data into some unit
# Deal with noise, sensor offset, and gravity to make the measurement as accurate as possible
