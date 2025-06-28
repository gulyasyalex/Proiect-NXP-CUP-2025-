import time
import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation

def main():
    port = "COM4"
    baud_rate = 230400
    ser = serial.Serial(port, baud_rate, timeout=0.1)
    time.sleep(2)
    print(f"Connected to {port} at {baud_rate} baud.")

    ser.write(b"0;220;0.0015;0.0001;0.00001\n")

    fig, ax = plt.subplots()
    fig.suptitle("Real-Time Speed from Teensy")

    time_data = []
    speed_data = []
    start_time = time.time()
    last_toggle_time = start_time
    toggle_state = True  # Start with 220 active

    (line,) = ax.plot([], [], "b-", label="Speed")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Speed (units)")
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 400)
    ax.legend(loc="upper left")
    plt.tight_layout()

    f = open("savedData.txt", "w")

    def update(frame):
        nonlocal last_toggle_time, toggle_state
        current_time = time.time()

        # Toggle every 3 seconds
        if current_time - last_toggle_time >= 3:
            if toggle_state:
                ser.write(b"0;0;0.0015;0.0001;0.00001\n")
                print("Sent: 0;0;0.0015;0.0001;0.00001")
            else:
                ser.write(b"0;100;0.0015;0.0001;0.00001\n")
                print("Sent: 0;100;0.0015;0.0001;0.00001")
            toggle_state = not toggle_state
            last_toggle_time = current_time

        raw_line = ser.read(ser.in_waiting).decode(errors="replace")
        if raw_line:
            print(raw_line)
            parts = raw_line.split(";")
            if len(parts) >= 2:
                try:
                    actual_speed = float(parts[0])
                except ValueError:
                    actual_speed = None
            else:
                actual_speed = None

            if actual_speed is not None:
                plot_time = current_time - start_time
                time_data.append(plot_time)
                speed_data.append(actual_speed)
                if actual_speed != 0:
                    f.write(f"{plot_time},{actual_speed}\n")
                ax.set_xlim(max(0, plot_time - 10), plot_time + 1)
                line.set_xdata(time_data)
                line.set_ydata(speed_data)

        return (line,)

    ani = animation.FuncAnimation(
        fig,
        update,
        interval=50,
        blit=False
    )

    try:
        plt.show()
    finally:
        ser.close()
        print("Serial port closed.")
        f.close()
        print("File saved.")

if __name__ == "__main__":
    main()
