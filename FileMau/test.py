import serial
import time

# Open serial port (adjust 'COM3' and baudrate as needed)
ser = serial.Serial('COM5', 115200, timeout=1)

# Write data
# run joint 1 to position 1000 with a travel time of 1000 ms
ser.write('#1P1500T1000\r\n'.encode('ascii'))
time.sleep(1)  # wait for the command to be processed
# run joint 2 to position 2000 with a travel time of 500 ms
ser.write('#2P1300T500\r\n'.encode('ascii'))
time.sleep(0.5)  # wait for the command to be processed

ser.close()