import matplotlib
# Choose the backend based on your environment
# Uncomment one of the following lines based on what you have installed

# For Qt5 backend
matplotlib.use('Qt5Agg')

# For Tkinter backend (if Qt5 is problematic)
# matplotlib.use('TkAgg')

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import serial
import re
from collections import deque

# Configure the serial port
try:
    ser = serial.Serial('/dev/ttyACM0', 2000000)  # Adjust COM port and baud rate as necessary
except serial.SerialException as e:
    print(f"Error: Could not open serial port. {e}")
    exit()

# Initialize plot
fig, ax = plt.subplots()
x_data = {2: deque(maxlen=1000), 3: deque(maxlen=1000), 4: deque(maxlen=1000), 5: deque(maxlen=1000), 12: deque(maxlen=1000)}  # Data for each timer
y_data = {2: deque(maxlen=1000), 3: deque(maxlen=1000), 4: deque(maxlen=1000), 5: deque(maxlen=1000), 12: deque(maxlen=1000)}  # Data for each timer

# Create line objects for each timer
lines = {
    2: ax.plot([], [], label='Timer 2')[0],
    3: ax.plot([], [], label='Timer 3')[0],
    4: ax.plot([], [], label='Timer 4')[0],
    5: ax.plot([], [], label='Timer 5')[0],
    12: ax.plot([], [], label='Timer 12')[0],
}

def update_plot(frame):
    data_to_process = []
    try:
        while ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').strip()
            data_to_process.append(line)
    except serial.SerialException as e:
        print(f"Error reading from serial port: {e}")

    for line in data_to_process:
        match = re.match(r'\[(\d+)\]:(\d+\.\d+)', line)
        if match:
            timer_id = int(match.group(1))
            try:
                duty_cycle = float(match.group(2))
            except ValueError:
                print(f"Warning: Failed to convert duty cycle '{match.group(2)}' to float.")
                continue

            if timer_id in x_data:
                if len(x_data[timer_id]) == 0:
                    new_time_index = 1
                else:
                    new_time_index = x_data[timer_id][-1] + 1

                x_data[timer_id].append(new_time_index)
                y_data[timer_id].append(duty_cycle)

                # Update the data for the respective line
                lines[timer_id].set_data(x_data[timer_id], y_data[timer_id])

                # Set the limits of the axes
                ax.relim()
                ax.autoscale_view()

    # Redraw the plot
    ax.legend()
    fig.canvas.draw()
    fig.canvas.flush_events()

    return list(lines.values())

# Create animation and assign it to a variable
anim = animation.FuncAnimation(fig, update_plot, interval=20, cache_frame_data=False)

# Set up the plot labels and title
ax.set_ylim(-0.1, 15)
ax.set_xlabel('Time Index')
ax.set_ylabel('Duty Cycle [%]')
ax.set_title('Real-time Duty Cycle Plot')
ax.legend()  # Show legend

# Add grid to the plot
ax.grid(True)  # Enable the grid

# Show plot
plt.show()  # Allow real-time updates

