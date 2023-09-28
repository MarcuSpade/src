from rangefinders_i2c.SensorManagment import SensorManagment

managment = SensorManagment('/dev/ttyUSB0', 9600)

command = input()
# print(command)

while command != "exit":
    managment.serialSend(command)
    command = input()