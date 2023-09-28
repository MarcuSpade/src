from rangefinders_i2c.SensorManagment import SensorManagment

# Se tiver mais de um sensor conectado simultaneamente:
# print(SensorManagment.GetUsbDeviceList())

managment = SensorManagment('/dev/ttyUSB0', 9600)

print("Digite o modo de envio do TOF10120 (0 = envio ativo (UART); 1 = leitura passiva (UART/I2C) default=0 envio ativo): ")

mode = int(input())
# print(address)

managment.setSendMode(mode)
