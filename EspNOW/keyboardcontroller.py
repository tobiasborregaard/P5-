import serial
import time
import keyboard  # use 'pip install keyboard' to install this module

# setup the serial connection. Replace 'COMx' with your ESP32 serial port.
ser = serial.Serial('COM5', 9600)
time.sleep(2)  # wait for the serial connection to initialize

while True:
    try:
        if keyboard.is_pressed('a'):  # if key 'a' is pressed
            ser.write(b'a')  # send 'a' to the ESP32
            time.sleep(0.1)
        elif keyboard.is_pressed('d'):  # if key 'd' is pressed
            ser.write(b'd')  # send 'd' to the ESP32
            time.sleep(0.1)
        elif keyboard.is_pressed('w'):  # if key 'w' is pressed
            ser.write(b'w')  # send 'w' to the ESP32
            time.sleep(0.1)
    except:
        break  # if user pressed a key other than the specified key, break the loop

ser.close()  # close the serial connection
