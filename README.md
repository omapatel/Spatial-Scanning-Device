# Spatial Scanning Device

#### Scanning device able to gather and visualize Time-of-Flight data

# Overview
The room which ToF data was to be gathered from.
![IMG_4876](https://github.com/user-attachments/assets/bf5baee0-e80e-475e-bf38-6514a760c6cc) \
\
The device itself, featuring the MSP432E401Y microcontroller, VL53L1X ToF sensor attached to a stepper motor using a 3D-printed mount. The device is connected to a computer/laptop which runs a python script in the background, collected the ToF data for Open3D (python library) visualization.
![IMG_4929](https://github.com/user-attachments/assets/6a0f4ea1-46b0-43cc-9557-2e141a12a6c3) \
\

The resulting Open3D visual from the ToF data.
![2](https://github.com/user-attachments/assets/e89c351b-6fa7-4d6b-9ccc-1e08153232bb)\
\

Block diagram showing how different components are connected.
![image](https://github.com/user-attachments/assets/315e261d-8b3d-4cae-b51a-adda9c50a2fd)\
\

The circuit schematic showcasing the connections in greater detail.
![image](https://github.com/user-attachments/assets/066b712c-41ee-4dc6-9162-0fa9b78cb8c6)\
\

# Description
The device is designed for capturing spatial information within indoor environments. It combines the capabilities of the VL53L1X Time-of-Flight (ToF) sensor (mounted to the 28BYJ-48 Stepper Motor) and the MSP432E401Y microcontroller to create discretized room scans and visualize them using Python.

The VL53L1X Time-of-Flight (ToF) measures 4.9 mm x 2.5 mm x 1.56 mm. The ToF sensor emits a 940 nm invisible laser (Class 1). Its ranging capability extends up to 4 meters, making it ideal for room-scale applications. With a maximum sampling rate of 50 Hz, the ToF sensor captures real-time distance data efficiently. Communication with the microcontroller occurs via the I2C interface, supporting speeds of up to 400 kHz.

The MSP432E401Y microcontroller links the peripherals to collect and transmit data. Operating at a clock speed of 60 MHz, it handles data processing and control utilizing 32-bit registers. The microcontroller is powered through a micro-USB connected to a computer; this serves a double purpose as the Python script accesses the UART port linked to the microcontroller. For communication with the computer, the device employs 8N1 UART at a baud rate of 115200. The Python script runs on the computer and interfaces with the device. It saves distance measurements obtained from the ToF sensor. Using the Open3D Python library, the script visualizes the collected data points, creating a discretized representation of the scanned room. An assumption made is that each xy-slice is exactly 10 cm apart, no matter how much the user actually moves.

# Technologies
I used the following to create this program:
- VL53L1X Time-of-Flight (ToF) sensor
- ULN2003 driver board --> 28BYJ-48 Stepper Motor
- MSP432E401Y microcontroller
- Python
- Open3D library

# Learnings
This project deepened my understanding of integrating sensor data with microcontroller systems for real-time applications. I gained experience working with the VL53L1X Time-of-Flight sensor, learning how to calibrate and optimize its performance for indoor environments. Utilizing I2C communication allowed me to transmit data between the sensor and the MSP432E401Y microcontroller. I also enhanced my skills in UART communication to efficiently handle data flow between the microcontroller and Python scripts. Visualizing the captured data using Open3D furthered my knowledge of point cloud processing and data representation, helping me transform raw distance measurements into meaningful spatial information.

# Authorship
Key helper files (such as the ToF api and I2C protocol) were provided from the course instructors or from the ToF product website. The main program loop (both in C and Python) scripts were developed by myself. You can check out more of my work in my portfolio:

#### [Visit my portfolio](https://portfolio-ompatel.netlify.app/)
