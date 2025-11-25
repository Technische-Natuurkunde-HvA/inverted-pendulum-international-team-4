import serial
import time
import csv

# Set the serial port and baud rate (make sure these match your Arduino setup)
serial_port = 'COM15'  # Update with the correct port for your system (e.g., 'COM3' for Windows or '/dev/ttyUSB0' for Linux)
baud_rate = 9600       # This should match the baud rate in your Arduino code

# Open the serial port
try:
    ser = serial.Serial(serial_port, baud_rate, timeout=1)
    print(f"Connected to {serial_port} at {baud_rate} baud")
except serial.SerialException as e:
    print(f"Error connecting to serial port: {e}")
    exit()

# File to store the data (you can choose a .csv or .txt file)
output_file = "motor_data.csv"  # Change this to .txt if preferred

# Prepare CSV writer if saving as CSV
with open(output_file, mode='w', newline='') as file:
    csv_writer = csv.writer(file)
    csv_writer.writerow(['Timestamp', 'Output', 'Frequency (Hz)', 'RPM'])  # CSV headers

    print("Start reading data... Press Ctrl+C to stop.")

    try:
        while True:
            # Read data from the serial port
            line = ser.readline().decode('utf-8').strip()  # Read a line from the serial port and decode

            if line:  # If line is not empty
                # Example line format: Output: -246  Frequency: 0.00 Hz  RPM: 0.00
                try:
                    # Parse the line to extract output, frequency, and RPM using string splitting
                    parts = line.split("  ")
                    
                    # Split each part further to get the actual value
                    output = parts[0].split(":")[1].strip()  # Get the value after "Output:"
                    frequency = parts[1].split(":")[1].strip()  # Get the value after "Frequency:"
                    rpm = parts[2].split(":")[1].strip()  # Get the value after "RPM:"

                    # Get the current timestamp
                    timestamp = time.strftime('%Y-%m-%d %H:%M:%S')

                    # Write data to CSV
                    csv_writer.writerow([timestamp, output, frequency, rpm])

                    print(f"Logged: {timestamp} - Output: {output} Frequency: {frequency} RPM: {rpm}")

                except Exception as e:
                    print(f"Error parsing line: {e}")
            time.sleep(0.1)  # Small delay to prevent overloading the serial buffer

    except KeyboardInterrupt:
        print("Data collection stopped by user.")

    finally:
        ser.close()  # Close the serial connection
        print(f"Serial connection closed. Data saved to {output_file}")