# Fall Detection Device using STM32F03C8T6 and ADXL345 sensor.

### Project Overview
This project utilizes the STM32F103C8T6 microcontroller and the ADXL345 accelerometer sensor to detect falls. The system provides real-time status updates and alerts on a 16x02 LCD display and uses LEDs for visual indications of the system's state and alerts.

### Features
Fall Detection: Automatically detects a fall using the ADXL345 accelerometer.

Status Display: Real-time system status displayed on a 16x02 LCD.

LED Indicators:
- Red LED flashes at 2 Hz to signal a fall.
- Green LED blinks at 1 Hz to indicate the system is active.
System Controls:
- SW1: Toggle the system operation state (active/inactive).
- SW2: Reset the system to normal state.
### Hardware Components Used
![image](https://github.com/VuTuanAnh-1368/FallDetectionSystem_STM32_ADXL345/assets/92041804/320859c2-960b-4686-93ab-26254740985a)

### Inputs
- ADXL345 Accelerometer: Detects fall based on sudden changes in acceleration.
- SW1: Toggles the operational state of the system.
- SW2: Resets the system to its initial state.
### Outputs
- Green LED: Blinks at a frequency of 1Hz when the system is operational; off when inactive.
- Red LED: Flashes at a frequency of 2Hz when a fall is detected; off when the system is in normal state.
- LCD Display: Shows 0 - Normal, 1 - Fall Detected based on the system's current status.
### Software Configuration
Download and Install Keil uVision 5:

Navigate to the Keil downloads page and download the MDK-ARM version that includes Keil uVision 5.
Install the software by following the on-screen instructions.

Create project -> Select STMicroelectronics > STM32F103C8T6 from the device database and confirm with OK.
### Usage
How to use the system:

1. Ensure the system is powered on.
2. SW1 toggles the active monitoring of the user.
3. SW2 resets any alert and returns the system to normal monitoring.
4. Observe the system status via the LCD and LED indicators.
