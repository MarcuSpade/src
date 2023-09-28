from rangefinders_i2c.ReadSensor import ReadSensor
from smbus2 import SMBus, i2c_msg
import time


SMBUS_ADDRESS = 1
I2C_ADDRESS = 0x82
MAX_COUNT = 100

dirsend_flag = 0

bus = SMBus()
bus.open(SMBUS_ADDRESS)

count = 0

def SensorWrite(datbuf):
    try:
        bus.write_word_data(I2C_ADDRESS, 0x00, int.from_bytes([0x0e, datbuf], byteorder='big'))
    except IOError:
        print('i2c address does not exist or rangefinder not connected')
        pass

print("Address: ")
I2C_ADDRESS = int(input())

measured_time = time.time()

while count < MAX_COUNT:
    try:
        bus.write_byte(I2C_ADDRESS, 0x00)
        distance = bus.read_byte(I2C_ADDRESS) << 8 | bus.read_byte(I2C_ADDRESS)
    except IOError:
        print('i2c address does not exist or rangefinder not connected')
        continue
    print("Distance = ", distance, " mm")
    time.sleep(300.0/1000.0)
    if distance > 110 and dirsend_flag == 0:
        dirsend_flag = 1
        SensorWrite(dirsend_flag)
        print("dir=", dirsend_flag)
        time.sleep(30.0/1000.0)
    elif distance < 90 and dirsend_flag == 1:
        dirsend_flag = 0
        SensorWrite(dirsend_flag)
        print("dir=", dirsend_flag)
        time.sleep(30.0/1000.0)
    if distance < 10:
        continue
    count += 1

measured_time = time.time() - measured_time

bus.close()

print("Results:\n--- Freq = ", MAX_COUNT/measured_time, "\n---Measure Time = ", measured_time/MAX_COUNT)

