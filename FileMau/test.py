import serial

# Open serial port (adjust 'COM3' and baudrate as needed)
ser = serial.Serial('COM4', 115200, timeout=1)

# Write data
ser.write('#1P1500T1000\r\n'.encode('ascii'))

ser.close()