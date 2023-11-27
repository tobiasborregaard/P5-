
# pip install pyserial keyboard matplotlib numpy 

import serial
import time
import keyboard
import json
import matplotlib.pyplot as plt
import numpy as np

# Set up the serial connection
arduino = serial.Serial('COM4', 115200, timeout=1)
time.sleep(2)

# Define the grid for angles with sub-dictionaries for points
grid = {f"{degree}d": {str(point): None for point in range(1, 13)} for degree in range(0, 60, 5)}



# Initialize the current angle and point
current_angle = 0
current_point = 1
inisialize = False

def update_grid(angle, point, distance):
    try:
        distance_val = float(distance)
        if distance_val > 120 or distance_val == -1:
            grid[f"{angle}d"][str(point)] = None
        else:
            grid[f"{angle}d"][str(point)] = distance_val
    except ValueError:
        grid[f"{angle}d"][str(point)] = None

def next_measurement(angle, point):
    new_point = point + 1
    new_angle = angle
    if new_point > 12:
        new_point = 1
        new_angle = (angle + 5) % 60
    return new_angle, new_point

def save_to_json(grid_data):
    with open('datagrid.json', 'w') as file:
        json.dump(grid_data, file, indent=4)
    print("Grid data saved to datagrid.json")

def plot_live_polar(grid_data):
    plt.ion()  # Enable interactive mode
    plt.clf()  # Clear the current figure

    # Create a polar plot
    ax = plt.subplot(111, polar=True)

    # Plot each point in the grid
    for angle_str, points in grid_data.items():
        # Convert angle to radians and adjust for the desired orientation
        angle_rad = np.deg2rad(int(angle_str[:-1]) - 20)
        for point, distance in points.items():
            if distance is not None:
                ax.plot([angle_rad], [distance], 'ro')  # Plot the point

    # Set the theta direction to clockwise
    ax.set_theta_direction(-1)

    # Set the zero angle at the top
    ax.set_theta_zero_location('N')

    # Draw the plot
    plt.draw()
    plt.pause(0.001)  # Pause briefly to update the plot

try:
    plt.figure()
    while True:
        if inisialize == False:
            plot_live_polar(grid)  # Update the plot
            inisialize = True
            
        if arduino.in_waiting > 0:
            distance = arduino.readline().decode('utf-8').rstrip()

        if keyboard.is_pressed('left'):
            #wait for accept
            state = False
            while (state == False):
                print(f"Press up to accept {distance}, down to try again ")
                if keyboard.is_pressed('up'):
                    update_grid(current_angle, current_point, distance)
                    print(f"Saved {distance} cm at {current_angle} degrees, point {current_point}")
                    current_angle, current_point = next_measurement(current_angle, current_point)
                    plot_live_polar(grid)  # Update the plot'
                    state = True

                elif keyboard.is_pressed('down'):
                    state = True
                
            time.sleep(0.8) 
                    

        elif keyboard.is_pressed('right'):
            current_angle, current_point = next_measurement(current_angle, current_point)
            print(f"Moved to {current_angle} degrees, point {current_point}")
            plot_live_polar(grid)  # Update the plot
            time.sleep(0.5)

        elif keyboard.is_pressed('s'):
            save_to_json(grid)
            time.sleep(0.2)

        print(f"Distance: {distance}   Angle: {current_angle} Point: {current_point} Grid: {grid[f'{current_angle}d'][str(current_point)]}")

except KeyboardInterrupt:
    print("Program stopped. Saving last state to JSON...")
    save_to_json(grid)
    plt.ioff()  # Turn off interactive mode

arduino.close()
