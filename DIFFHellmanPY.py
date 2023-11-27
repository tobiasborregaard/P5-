def crc32_bitwise(data):
    # Polynomial for CRC-32 (Ethernet, ZIP, etc.)
    poly = 0x104C11DB7
    # Initial value of the CRC
    crc = 0xFFFFFFFF

    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ poly
            else:
                crc >>= 1

    # Finalize the CRC-32 value by inverting all the bits
    crc ^= 0xFFFFFFFF
    return crc

# Example data
data = b"Hello, world!"
# Convert data to a list of byte values
byte_data = [b for b in data]

# Perform CRC32 calculation
crc_value = crc32_bitwise(byte_data)
print(f"CRC32: {crc_value:#010x}")  # Display as hex
