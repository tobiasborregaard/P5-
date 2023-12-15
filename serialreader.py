import serial
import time
import csv
import keyboard  # Make sure to install this library using pip

def create_output_filename():
    timestamp = time.strftime("%Y%m%d-%H%M%S")
    return f'output_{timestamp}.csv'   

def main():
    # Serial port configuration
    serial_port = 'COM3'
    baud_rate = 230400

    # Output file name with automatic increment
    output_file = create_output_filename()

    # Open serial port
    ser = serial.Serial(serial_port, baud_rate)
    ser.flushInput()

    # Open output file
    with open(output_file, 'w', newline='') as csvfile:     
        writer = csv.writer(csvfile, delimiter=',')
        writer.writerow(["Rel Angle"])

        # Read serial port and write to CSV
        while True:
            try:
                # Check for spacebar press
                if keyboard.is_pressed('space'):
                    print("Spacebar pressed, stopping...")
                    break

                ser_bytes = ser.readline()
                decoded_bytes = ser_bytes[0:len(ser_bytes)-2].decode("utf-8")
                print(decoded_bytes)
                writer.writerow([decoded_bytes])
     
            except KeyboardInterrupt:
                print("Keyboard Interrupt")
                break
            except Exception as e:
                print(f"Error: {e}")
                break

    # Close serial port
    ser.close()

if __name__ == "__main__":
    main()
