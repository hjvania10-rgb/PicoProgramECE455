import serial
import matplotlib.pyplot as plt

# Change this to your Pico's COM port
PORT = "COM5"

BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=1)

times = []
voltages = []

print("Waiting for waveform...")

collecting = False

while True:
    line = ser.readline().decode(errors="ignore").strip()

    if not line:
        continue

    if line == "DATA_START":
        times = []
        voltages = []
        collecting = True
        print("Receiving samples...")
        continue

    if line == "DATA_END":
        print(f"Received {len(times)} samples")
        break

    if collecting:
        try:
            time_us, voltage_mv = line.split(",")

            times.append(float(time_us) / 1_000_000.0)
            voltages.append(float(voltage_mv))

        except ValueError:
            pass

ser.close()


# ------------------------------------------------------------
# Plot
# ------------------------------------------------------------

plt.figure()

plt.plot(times, voltages)

plt.xlabel("Time (s)")
plt.ylabel("Voltage (mV)")
plt.title("ADS1256 Waveform")

plt.grid(True)

plt.show()