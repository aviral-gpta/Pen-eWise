import serial

# Open serial connection to ESP32
ser = serial.Serial('COM3', 115200)  # Replace 'COMx' with the correct serial port

# Open file to save data
with open('sensor_data_50.txt', 'w') as file:
    while True:
        # Read data from serial port
        data = ser.readline().decode().strip()
        # print(data)  # Optional: Print received data to console
        
        # Write data to file
        file.write(data + '\n')
