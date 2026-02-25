import matplotlib.pyplot as plt
import matplotlib.animation as animation
import serial
from collections import deque

active_timers = [2, 3, 4, 5]

# Configure the serial port
try:
    ser = serial.Serial('/dev/ttyACM0', 2000000)  # Adjust COM port and baud rate as necessary
except serial.SerialException as e:
    print(f"Error: Could not open serial port. {e}")
    exit()

# Initialize plot
fig, ax = plt.subplots()
x_data = {2: deque(maxlen=1000), 3: deque(maxlen=1000), 4: deque(maxlen=1000), 5: deque(maxlen=1000)}  # Data for each timer
y_data = {2: deque(maxlen=1000), 3: deque(maxlen=1000), 4: deque(maxlen=1000), 5: deque(maxlen=1000)}  # Data for each timer

# Create line objects for each timer with increased line width
lines = {
    2: ax.plot([], [], label='Emergency Trigger', linewidth=4)[0],
    3: ax.plot([], [], label='Emergency Steer', linewidth=4)[0],
    4: ax.plot([], [], label='Joystick Gas/Brake', linewidth=4)[0],
    5: ax.plot([], [], label='Joystick Steer', linewidth=4)[0],
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
        
        if line:
        
            data_to_use = line.split()

            try:
                
                data_to_use = [float(t_on) for t_on in data_to_use]

            except ValueError:

                break
            
            if len(data_to_use)!=4:
                
                break

            for t_on in data_to_use:

                if t_on<900 or t_on>2100:

                    break

            print(f"Valori: {data_to_use}")

            for timer_id in active_timers:
                
                if len(x_data[timer_id]) == 0:
                    new_time_index = 1
                else:
                    new_time_index = x_data[timer_id][-1] + 1

                x_data[timer_id].append(new_time_index)
                y_data[timer_id].append(data_to_use[timer_id-2])
    
                # Update the data for the respective line
                lines[timer_id].set_data(x_data[timer_id], y_data[timer_id])

                # Set the limits of the axes
                ax.relim()
                ax.autoscale_view()
        

    # Redraw the plot
    ax.legend(fontsize=14)  # Increase legend font size
    fig.canvas.draw()
    fig.canvas.flush_events()

    return list(lines.values())

# Create animation and assign it to a variable
anim = animation.FuncAnimation(fig, update_plot, interval=20, cache_frame_data=False)

# Set up the plot labels and title with increased font size
ax.set_ylim(900, 2100)
ax.set_xlabel('Time Index', fontsize=16)
ax.set_ylabel(r'$T_{\mathrm{ON}} \, [\mu\mathrm{sec}]$', fontsize=25)
ax.set_title(r'Real-time $T_{\mathrm{ON}}$ Plot', fontsize=25)
ax.legend(fontsize=14)  # Increase legend font size

# Add grid to the plot
ax.grid(True)  # Enable the grid

# Increase tick label size
ax.tick_params(axis='both', which='major', labelsize=14)

# Show plot
plt.show()  # Allow real-time updates
