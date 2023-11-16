import serial
import time
import keyboard
import json

# Set up the serial connection (adjust the COM port and baud rate according to your setup)
arduino = serial.Serial('COM4', 115200, timeout=1)
time.sleep(2)  # Wait for the connection to initialize

# Define the grid
grid = {}
for num in range(1, 13):   # 1-12 for vertical positions
    for char in 'ABCDE':   # A-E for horizontal positions
        grid[f"{num}{char}"] = None

# Initialize the current position
current_pos = "1A"


# Function to update the grid with new distance measurements
def update_grid(position, distance):
    try:
        distance_val = float(distance)
        if distance_val > 120 or distance_val == -1:
            grid[position] = None
        else:
            grid[position] = distance_val
    except ValueError:
        # If the distance is not a valid float, set it to None
        grid[position] = None


# Function to move to the next position in the grid
def next_position(current):
    num, char = int(current[:-1]), current[-1]
    if char == 'E':
        if num == 12:
            return "1A"  # Wrap around to the start
        else:
            return f"{num+1}A"
    else:
        return f"{num}{chr(ord(char) + 1)}"

# Function to save the grid to a JSON file
def save_to_json(grid_data):
    with open('datagrid.json', 'w') as file:
        json.dump(grid_data, file, indent=4)
    print("Grid data saved to datagrid.json")

try:
    while True:
        # Check for incoming data
        if arduino.in_waiting > 0:
            distance = arduino.readline().decode('utf-8').rstrip()

        # Handle left key press - save distance
        if keyboard.is_pressed('left'):
            update_grid(current_pos, distance)
            print(f"Saved {distance} cm at {current_pos}")
            current_pos = next_psosition(current_pos)
            time.sleep(2)  # Prevent multiple detections

        # Handle right key press - move to next position
        elif keyboard.is_pressed('right'):
            current_pos = next_position(current_pos)
            print(f"Moved to {current_pos}")
            time.sleep(2)  # Prevent multiple detections

        # Handle 's' key press - save to JSON
        elif keyboard.is_pressed('s'):
            save_to_json(grid)
            time.sleep(0.2)  # Prevent multiple detections
        print(f"Distance:  {distance}   Position: {current_pos} Grid: {grid[current_pos]}" )

        
except KeyboardInterrupt:
    print("Program stopped. Saving last state to JSON...")
    save_to_json(grid)  # Save the last state before exiting

# Close the serial connection
arduino.close()
