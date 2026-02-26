import serial, time

ser = serial.Serial('/dev/ttyUSB0', baudrate=19200, timeout=1)
time.sleep(0.1)

# Arrêter le streaming
ser.write(b'@@A\r')
time.sleep(0.5)
ser.reset_input_buffer()

# Vérifier
ser.write(b'A\r')
time.sleep(0.3)
print("Poll:", ser.read_all())

# Changer setpoint
ser.write(b'AS2.000\r')
time.sleep(0.3)
print("Setpoint:", ser.read_all())

ser.close()