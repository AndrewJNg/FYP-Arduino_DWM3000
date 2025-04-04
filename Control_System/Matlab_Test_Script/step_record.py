import serial
import csv
import time

# Replace 'COM3' with your Arduino port (e.g., 'COM4' on Windows or '/dev/ttyUSB0' on Linux)
arduino_port = 'COM5'
baud_rate = 115200  # Ensure this matches your Arduino's serial settings
csv_file = 'Motor_Step_Response_Matlab\TT_Speed_stepping_AS5600_1m.csv'

# Open serial connection
ser = serial.Serial(arduino_port, baud_rate, timeout=1)
time.sleep(2)  # Wait for connection to establish

# Open CSV file for writing
with open(csv_file, 'w', newline='') as file:
    writer = csv.writer(file)
    # Write header
    writer.writerow(["time", "leftEncoderCount", "rightEncoderCount"])

    print("Recording data... Press Ctrl+C to stop.")
    try:
        while True:
            # Read line from serial, decode to string, and strip any whitespace
            line = ser.readline().decode('utf-8').strip()
            if line:
                # Split the line into separate values based on spaces
                values = line.split()
                if len(values) == 3:
                    # Parse values and write to CSV
                    
                    writer.writerow(values)
                    print(values)  # Optional: Print data to console
    except KeyboardInterrupt:
        print("Recording stopped.")
    finally:
        ser.close()