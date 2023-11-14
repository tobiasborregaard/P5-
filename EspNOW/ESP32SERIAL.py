import serial
import time
import csv
from datetime import datetime

# Configure the serial port and baud rate
SERIAL_PORT = 'COM6'  # Update with your serial port name
BAUD_RATE = 115200            # Update with your baud rate
CSV_FILE = 'angle_speed_data.csv'

def main():
    try:
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
            print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud.")
            time.sleep(2)  # Give some time for the device to initialize
            ser.reset_input_buffer()  # Clear the input buffer
            ser.reset_output_buffer()  # Clear the output buffer
            Packetnumber = 0
            # Open CSV file
            with open(CSV_FILE, mode='a', newline='') as file:
                writer = csv.writer(file)
                # Write header if the file is empty
                if file.tell() == 0:
                    writer.writerow(['Angle', 'Speed','Packetnumber'])

                # Continuously read data and write to CSV
                while True:
                     # Read a line from the serial port
                    line = ser.readline().decode('utf-8').rstrip()
                    
                    if ',' in line:
                                try:
                                    angle, speed = line.split(',')
                                    angle = float(angle.strip())
                                    speed = float(speed.strip())
                                    print(f"Angle: {angle}°, Speed: {speed} km/h")
                                    # Write to CSV
                                    writer.writerow([ angle, speed, Packetnumber])
                                    Packetnumber += 1
                                except ValueError:
                                    print("Invalid data format received:", line)
                                    continue
                    else:
                                continue

                  
    except serial.SerialException as e:
        print(f"Error: {e}")


if __name__ == '__main__':
    main()
