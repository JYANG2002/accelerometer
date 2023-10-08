import machine
from time import sleep
import struct
import math

def byteUnpack(low, high):
    low = int.from_bytes(low, 'big')
    high = int.from_bytes(high, 'big')
    num = 256 * high + low # Since 2^8 is 256
    if num >= 32768: # Negatives due to two's comp
        num = num - 65536
    return num
def readAccelerometer(i2c):
    acc_x = byteUnpack(i2c.readfrom_mem(0x6B, 0x28, 1), i2c.readfrom_mem(0x6B, 0x29, 1)) # Read a byte from OUTX_L_XL & OUTX_H_XL
    acc_y = byteUnpack(i2c.readfrom_mem(0x6B, 0x2A, 1), i2c.readfrom_mem(0x6B, 0x2B, 1)) # Read a byte from OUTY_L_XL & OUTY_H_XL
    acc_z = byteUnpack(i2c.readfrom_mem(0x6B, 0x2C, 1), i2c.readfrom_mem(0x6B, 0x2D, 1)) # Read a byte from OUTZ_L_XL & OUTZ_H_XL
    # Convert to decimal to understand value
    scale_factor = (2) / (2 ** 15)  # g/LSB
    g = 9.81 # gravity
    acc_x *= scale_factor * g
    acc_y *= scale_factor * g
    acc_z *= scale_factor * g
    return [acc_x, acc_y, acc_z]

def calibration(i2c): # Takes 12 seconds to calibrate
    x = []
    y = []
    z = []
    for i in range(0, 1000): # Take in 100 data
        data = readAccelerometer(i2c)
        x.append(data[0])
        y.append(data[1])
        z.append(data[2])
        sleep(0.01) # Give time to read since it's 416 Hz so it stabilizes
    avg_x = sum(x) / 1000
    avg_y = sum(y) / 1000
    avg_z = sum(z) / 1000
    return [avg_x, avg_y, avg_z]
    
sda=machine.Pin(16)
scl=machine.Pin(17)
i2c=machine.I2C(0,sda=sda, scl=scl, freq=400000)
# I2C address of the LSM6DS33
lsm6ds33_address = 0x6B # Use this slave addr for gyroscope and accelerometer

# Acc X,Y,Z axes enabled
CTRL9_XL_Reg = 0x18 # Reg ADDR
CTRL9_XL_val = 0x38 # 
i2c.writeto_mem(lsm6ds33_address, CTRL9_XL_Reg, bytes([CTRL9_XL_val]))


# Acc = 208 Hz (low-Performance mode)
CTRL1_XL_Reg = 0x10 # Reg ADDR
CTRL1_XL_val = 0x30
i2c.writeto_mem(lsm6ds33_address, CTRL1_XL_Reg, bytes([CTRL1_XL_val]))

# Acc Data Ready interrupt on INT1
INT1_CTRL_REG = 0x0D # Reg ADDR
INT1_CTRL_val = 0x01
i2c.writeto_mem(lsm6ds33_address, INT1_CTRL_REG, bytes([INT1_CTRL_val]))

# Apply filter
# i2c.writeto_mem(lsm6ds33_address, 0x17, bytes([0x01]))  # 0x02 corresponds to 50Hz LPF, when using CTRL8_XL register

print("Calibrating")
calibrated_data = calibration(i2c)


x = 0
y = 0
z = 0
omega_c = 0.15 * 8 # just a starting point, change later
T = 0.1 # duration
# Set all params to 0
x_km1 = 0
xu_km1 = 0
y_km1 = 0
yu_km1 = 0
z_km1 = 0
zu_km1 = 0

# Useful info for calculating distance
smooth_bit = 0 # After it experiences acceleration it requires some time for filter to smooth out
# x axis
sign_bit_x = 0 # 0 positive, 1 negative
start_x = 0 # If there are changes in the y axis
accel_x = [] # Grab the acceleration it experiences
distance_x = 0
count_x = 0 # used for time intervals
# y axis
sign_bit_y = 0 # 0 positive, 1 negative
start_y = 0 # If there are changes in the y axis
accel_y = [] # Grab the acceleration it experiences
distance_y = 0
count_y = 0 # used for time intervals
#z axis
sign_bit_z = 0 # 0 positive, 1 negative
start_z = 0 # If there are changes in the y axis
accel_z = [] # Grab the acceleration it experiences
distance_z = 0
count_z = 0 # used for time intervals


while True:
    data = readAccelerometer(i2c)
    # Round to hundredths
    result = [round(a - b, 2) for a, b in zip(data, calibrated_data)] # Subtract val with calibration
    x = (1-omega_c*T)*x_km1 + omega_c*T*xu_km1; # Our acceleration for the axis
    y = (1-omega_c*T)*y_km1 + omega_c*T*yu_km1; 
    z = (1-omega_c*T)*z_km1 + omega_c*T*zu_km1;
    
    # set previous unfiltered val
    xu_km1 = result[0]
    yu_km1 = result[1]
    zu_km1 = result[2]
    # set previous filtered val
    x_km1 = x
    y_km1 = y
    z_km1 = z
    
    if abs(z) < 0.2 and abs(y) < 0.2 and abs(z) < 0.2: # If data is smooth we can accurately calculate distance
        smooth_bit = 1
        
    if smooth_bit:
        # For direction on X
        if abs(x) > 0.2 and start_x == 0:
            if x < 0: # If negative
                sign_bit_x = 1
            accel_x.append(x) # Grab current acceleration to do double integral
            start_x = 1  
            
        if start_x: # If we are counting
            count_x += 1
            accel_x.append(x)
            if abs(x) < 0.2 or (x < 0 and sign_bit_x == 0) or (x > 0 and sign_bit_x == 1): # if body stop or acceleration is suddenly opposite
                avg_x = sum(accel_x) / len(accel_x) 
                # d = 1/2at^2
                distance_cal = 1/2*avg_x * (count_x * 0.1)**2 * 6# Multipled by 6 based on data collected and make it as close to meters
                distance_x += distance_cal
                print("Time: " + str(count_x * 0.1))
                # reset
                sign_bit_x = 0
                start_x = 0
                count_x = 0
                smooth_bit = 0
                accel_x.clear()
                print("Distance calculated X: " + str(distance_cal))
                print("Distance X: " + str(distance_x))
                sleep(5)
        
        # For direction on Y
        if abs(y) > 0.2 and start_y == 0:
            if y < 0: # If negative
                sign_bit_y = 1
            accel_y.append(y) # Grab current acceleration to do double integral
            start_y = 1  
            
        if start_y: # If we are counting
            count_y += 1
            accel_y.append(y)
            if abs(y) < 0.2 or (y < 0 and sign_bit_y == 0) or (y > 0 and sign_bit_y == 1): # if body stop or acceleration is suddenly opposite
                avg_y = sum(accel_y) / len(accel_y) 
                # d = 1/2at^2
                distance_cal = 1/2*avg_y * (count_y * 0.1)**2 * 6# Multipled by 6 based on data collected and make it as close to meters
                distance_y += distance_cal
                print("Time: " + str(count_y * 0.1))
                # reset
                sign_bit_y = 0
                start_y = 0
                count_y = 0
                smooth_bit = 0
                accel_y.clear()
                print("Distance calculated Y: " + str(distance_cal))
                print("Distance Y: " + str(distance_y))
                sleep(5)

        # For direction on Z
        if abs(z) > 0.2 and start_z == 0:
            if z < 0: # If negative
                sign_bit_z = 1
            accel_z.append(z) # Grab current acceleration to do double integral
            start_z = 1  
            
        if start_z: # If we are counting
            count_z += 1
            accel_z.append(z)
            if abs(z) < 0.2 or (z < 0 and sign_bit_z == 0) or (z > 0 and sign_bit_z == 1): # if body stop or acceleration is suddenly opposite
                avg_z = sum(accel_z) / len(accel_z) 
                # d = 1/2at^2
                distance_cal = 1/2*avg_z * (count_z * 0.1)**2 * 6# Multipled by 6 based on data collected and make it as close to meters
                distance_z += distance_cal
                print("Time: " + str(count_z * 0.1))
                # reset
                sign_bit_z = 0
                start_z = 0
                count_z = 0
                smooth_bit = 0
                accel_z.clear()
                print("Distance calculated Z: " + str(distance_cal))
                print("Distance Z: " + str(distance_z))
                sleep(5)



    print("X:", "{:.2f}".format(x))
    # Print result
    '''
    print("X:", "{:.2f}".format(x), \
          "| Y:", "{:.2f}".format(y), \
          "| Z:", "{:.2f}".format(z))
    '''
    sleep(0.1)
