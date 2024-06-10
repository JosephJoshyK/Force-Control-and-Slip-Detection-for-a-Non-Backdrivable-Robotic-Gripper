# FSR Setup

## Overview

This folder contains the necessary files and instructions to set up the Force Sensitive Resistor (FSR) sensors for the robotic gripper. The setup includes uploading code to an ESP32 microcontroller, connecting the FSR to the ADC port, and calibrating the sensor using data collected from the ESP32 and Galil bridge.

## Contents

- `serial_calib.ino`: Arduino code to upload to the ESP32 for reading FSR data.
- `galil_bridge.cpp`: Code to interface with the Galil controller and collect data in parallel with the ESP32.
- `calibration_data.xlsx`: Excel file used to store and process the calibration data.
- `fsr_calibration.md`: Detailed instructions for the calibration process.

## Hardware Setup

1. **Connect the FSR to ESP32:**
    - Connect the FSR sensor to the 7th ADC port on the ESP32.

2. **Upload the `serial_calib.ino` Code:**
    - Open the `serial_calib.ino` file in the Arduino IDE.
    - Select the correct board (ESP32) and port.
    - Upload the code to the ESP32.

## Calibration Process

1. **Run the `serial_calib.ino` Code:**
    - Ensure the ESP32 is connected to your system via a serial connection.
    - The ESP32 will start reading data from the FSR and send it over serial communication.

2. **Run the `galil_bridge.cpp` Code:**
    - Compile and run the `galil_bridge.cpp` file on your system.
    - This code will interface with the Galil controller and collect data in parallel with the ESP32.

3. **Collect Data:**
    - The data from both the ESP32 and the Galil bridge will be collected and stored in an Excel file (`calibration_data.xlsx`).
    - The Excel file will have multiple sheets:
        - Sheet 1: Data from the ESP32.
        - Sheet 2: Data from the Galil bridge.
        - Sheet 3: Averaged values and derived calibration equations.

4. **Process Data:**
    - Use the data in the `calibration_data.xlsx` file to derive the calibration equations for the FSR.
    - The final sheet in the Excel file will contain the averaged values and the derived equations.

## Calibration Equations

After processing the data, you will obtain calibration equations for the FSR. These equations will be used in the main application to convert raw sensor readings into meaningful force measurements.

Example calibration equation:
\[ \text{Force (N)} = a \times \text{FSR Reading} + b \]

## Example Calibration Equations

Upload the processed `calibration_data.xlsx` file to the repository and document the derived calibration equations here.

Example:
```markdown
## Calibration Equations

Based on the calibration data, the following equations were derived for the FSR:

\[ \text{Force (N)} = 0.005 \times \text{FSR Reading} + 0.2 \]

These equations are used in the main application to convert the raw FSR readings into force measurements.

