import serial
import time 
import csv
import pandas as pd
import matplotlib.pyplot as plt

PORT = "____" #change this for the port the arduino is plugged into
BAUDRATE = 9600
OUTPUT_CSV = "motor_data.csv"

READ_DURATION = 60 # duration of measurement

def read_serial_to_csv():
    print(f"Opening serialport {PORT} at {BAUDRATE}")
    ser = serial.Serial(PORT, BAUDRATE, timeout=1)
    time.sleep(2) #wait for arduino to reset

    start_time = time.time() 
    lines = []

    print("reading data from arduino...")
    print(f"(will read for {READ_DURATION} seconds)")

    try:
        while time.time() - start_time < READ_DURATION:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if not line: 
                continue

            print(line) 

            if line.startswith("PWM"):
                lines.append(line)
            else:
                parts = line.split(",")
                if len(parts) == 3:
                    lines.append(line)

    except KeyboardInterrupt:
        print("stopped by user")

    finally:
        ser.close()
        print("Serial port closed")
    if not lines:
        print("no data received")
        return None
    
    if not lines[0].startswith("PWM"):
        lines.insert(0, "PWM,direction,wheel_RPM")

    with open(OUTPUT_CSV, "w", newline="") as f:
        writer = csv.writer(f)
        for i in lines:
            writer.writerow(i.split(","))

    print(f"\nSaved {len(lines)-1} data rows to {OUTPUT_CSV}")
    return OUTPUT_CSV


# load csv and plot
def plot_data(csv_file):
    print(f"Loading data from {csv_file}...")
    df = pd.read_csv(csv_file)

    # Ensure correct types
    df["PWM"] = df["PWM"].astype(int)
    df["direction"] = df["direction"].astype(int)
    df["wheel_RPM"] = df["wheel_RPM"].astype(float)

    # Separate forward (+1) and backward (-1)
    forward = df[df["direction"] > 0]
    backward = df[df["direction"] < 0]

    plt.figure()

    if not forward.empty:
        plt.scatter(forward["PWM"], forward["wheel_RPM"], label="Forward (+1)")
        plt.plot(forward["PWM"], forward["wheel_RPM"])

    if not backward.empty:
        plt.scatter(backward["PWM"], backward["wheel_RPM"], label="Backward (-1)")
        plt.plot(backward["PWM"], backward["wheel_RPM"])

    plt.xlabel("PWM value")
    plt.ylabel("Wheel RPM")
    plt.title("Output–response curve: PWM vs wheel RPM")
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.show()


# main

if __name__ == "__main__":
    csv_path = read_serial_to_csv()
    if csv_path is not None:
        plot_data(csv_path)
