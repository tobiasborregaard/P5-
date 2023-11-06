import serial
import time
from bluetooth import *

def find_esp32():
    nearby_devices = discover_devices(lookup_names=True)
    print("Found {} devices.".format(len(nearby_devices)))

    for addr, name in nearby_devices:
        print("  {} - {}".format(addr, name))
        if name == "Der fuhrer":  # Ensure your ESP32's Bluetooth Name matches this
            return addr

    return None


def main():
    esp32_addr = find_esp32()

    if esp32_addr is None:
        print("No ESP32 found.")
        return

    print("Connecting to {}...".format(esp32_addr))

    # Establish a Bluetooth socket connection
    sock = BluetoothSocket(RFCOMM)
    sock.connect((esp32_addr, 1))

    try:
        while True:
            data = sock.recv(1024)  # Adjust buffer size as needed
            print("Received: {}".format(data.decode("utf-8")))
            time.sleep(1)

    except KeyboardInterrupt:
        print("Disconnected.")

    finally:
        sock.close()


if __name__ == "__main__":
    main()
