import serial
import time
import re
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.ticker import MultipleLocator

# ------------------- Serial Setup -------------------
ser = serial.Serial("COM18", 115200, timeout=1)
time.sleep(2)
ser.reset_input_buffer()

# ------------------- Data Buffers -------------------
raw_roll_data = []
roll_data = []
raw_pitch_data = []
pitch_data = []
time_data = []
start_time = time.time()

# Show fixed 10s window
TIME_WINDOW = 10

# Regex to extract "Roll: raw, filtered | Pitch: raw, filtered"
pattern = re.compile(r"Roll:\s*(-?\d+\.\d+),\s*(-?\d+\.\d+)\s*\|\s*Pitch:\s*(-?\d+\.\d+),\s*(-?\d+\.\d+)")

# ------------------- Matplotlib Setup -------------------
fig, ax = plt.subplots(figsize=(9, 6))

# Lines for plotting
line_raw_roll,   = ax.plot([], [], label="Raw Roll",   color="cyan", linewidth=1, linestyle="--")
line_roll,       = ax.plot([], [], label="Filtered Roll", color="blue", linewidth=1.5)
line_raw_pitch,  = ax.plot([], [], label="Raw Pitch",  color="orange", linewidth=1, linestyle="--")
line_pitch,      = ax.plot([], [], label="Filtered Pitch", color="red", linewidth=1.5)

ax.set_xlabel("Time (s)")
ax.set_ylabel("Angle (deg)")
ax.set_title("Real-Time Roll & Pitch (Raw vs Filtered)")
ax.legend(loc="upper right")
ax.grid(True)

# Fix x-axis: 0 to 10 sec with ticks every 1 sec
ax.set_xlim(0, TIME_WINDOW)
ax.xaxis.set_major_locator(MultipleLocator(1))
ax.set_ylim(-180, 180)  # fixed y-axis for smooth scrolling

# ------------------- Update Function -------------------
def update(frame):
    while ser.in_waiting:  # process all available lines in buffer
        line = ser.readline().decode(errors="ignore").strip()
        match = pattern.match(line)

        if match:
            raw_roll = float(match.group(1))
            roll = float(match.group(2))
            raw_pitch = float(match.group(3))
            pitch = float(match.group(4))
            t = time.time() - start_time

            raw_roll_data.append(raw_roll)
            roll_data.append(roll)
            raw_pitch_data.append(raw_pitch)
            pitch_data.append(pitch)
            time_data.append(t)

            # remove old points outside the TIME_WINDOW
            while time_data and (t - time_data[0]) > TIME_WINDOW:
                raw_roll_data.pop(0)
                roll_data.pop(0)
                raw_pitch_data.pop(0)
                pitch_data.pop(0)
                time_data.pop(0)

            # Shift time axis so it always shows 0–10 sec window
            shifted_time = [ti - (t - TIME_WINDOW) for ti in time_data]

            line_raw_roll.set_data(shifted_time, raw_roll_data)
            line_roll.set_data(shifted_time, roll_data)
            line_raw_pitch.set_data(shifted_time, raw_pitch_data)
            line_pitch.set_data(shifted_time, pitch_data)

    return line_raw_roll, line_roll, line_raw_pitch, line_pitch

# ------------------- Animate -------------------
ani = animation.FuncAnimation(fig, update, interval=5, blit=True)

plt.tight_layout()
plt.show()
