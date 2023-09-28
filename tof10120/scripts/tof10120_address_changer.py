from rangefinders_i2c.SensorManagment import SensorManagment

# Se tiver mais de um sensor conectado simultaneamente:
# print(SensorManagment.GetUsbDeviceList())

managment = SensorManagment('/dev/ttyUSB0', 9600)

print("Digite o endereço do TOF10120 (1 ~ 254): ")

address = int(input())
# print(address)

managment.setAddress(address)
