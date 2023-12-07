import csv
from bluetooth import *

def find_esp32(name):
    nearby_devices = discover_devices(lookup_names=True)
    for addr, device_name in nearby_devices:
        if device_name == name:
            print(f"Found ESP32 device with name {name}: {addr}")
            return addr
    return None

def connect_to_esp32(addr):
    sock = BluetoothSocket(RFCOMM)
    try:
        sock.connect((addr, 1))
    except Exception as e:
        print(f"Could not connect to the device: {e}")
        return None
    return sock

def extract_values(data):
    try:
        # Split the received data by the comma and strip whitespace
        angle_str, speed_str = [x.strip() for x in data.split(',')]
        # Convert the string values to the appropriate type, such as float
        angle = float(angle_str)
        speed = float(speed_str)
        return angle, speed
    except ValueError as e:
        # If there is an error converting to float, or if the data is not as expected
        print(f"Error processing data '{data}': {e}")
        return None, None

def main():
    esp32_name = "Der fuhrer"  # Replace with your ESP32's Bluetooth name
    esp32_addr = find_esp32(esp32_name)

    if not esp32_addr:
        print(f"No ESP32 with name {esp32_name} found.")
        return

    print(f"Connecting to ESP32 at {esp32_addr}...")
    sock = connect_to_esp32(esp32_addr)

    if sock:
        print("Connected.")
    try:
        with open('data.csv', 'a', newline='') as file:
            csv_writer = csv.writer(file)
            if file.tell() == 0:
                csv_writer.writerow(["Angle", "Speed"])

            buffer = ""
            while True:
                data = sock.recv(1024).decode("utf-8")  # Buffer size of 1024 bytes
                buffer += data  # Store the incoming data in the buffer
                if '\n' in buffer:
                    # Complete message is received, process it
                    lines = buffer.split('\n')
                    for line in lines[:-1]:  # The last line may be incomplete
                        try:
                            angle, speed = extract_values(line)
                            if angle is not None and speed is not None:
                                csv_writer.writerow([angle, speed])
                                file.flush()  # Flush data to file immediately
                                print(f"Angle: {angle} Speed: {speed}")
                            else:
                                print("Invalid data received:", line)
                        except Exception as e:
                            print(f"Error while writing data to CSV: {e}")
                    buffer = lines[-1]  # Keep the last part (incomplete message) in the buffer
    except KeyboardInterrupt:
        print("Program terminated by user.")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        sock.close()
        print("Disconnected.")

if __name__ == "__main__":
    main()
