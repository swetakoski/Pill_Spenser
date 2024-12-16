Pill Dispenser System

Overview
This project implements a Pill Dispenser System designed to manage and dispense pills efficiently and securely. The system is built on a Raspberry Pi Pico microcontroller and integrates various components, such as a stepper motor, LEDs, sensors, and a LoRa communication module for remote connectivity.
The system supports the following core functionalities:
1.	Calibration of the dispenser mechanism.
2.	Automated dispensing of pills based on sensor feedback.
3.	Error handling and user interaction via buttons and LEDs.
4.	LoRaWAN communication for network connectivity.
________________________________________
Features
•	Idle Mode: Waits for user input via buttons for calibration or dispensing.
•	Motor Calibration: Automatically determines the step count required for a full motor revolution and aligns the pill compartments with the dispenser.
•	Pill Dispensing: Detects pill drops using a piezoelectric sensor and manages errors if pills are not dispensed correctly.
•	LoRaWAN Integration: Configures and communicates with a LoRa module for remote status reporting.
•	Error Notifications: Indicates errors with LED blinking patterns.
________________________________________
Hardware Requirements
Components:
1.	Raspberry Pi Pico (microcontroller)
2.	Stepper Motor (with four GPIO pins for control)
3.	Optofork Sensor (for alignment detection)
4.	Piezoelectric Sensor (for pill-drop detection)
5.	Buttons:
o	SW_0 (Dispense button)
o	SW_2 (Calibration button)
6.	LED (for visual notifications)
7.	LoRa Module (UART communication)
Pin Assignments:
Component	GPIO Pin
Dispense Button	9
Calibration Button	7
LED	21
Stepper Motor IN1	2
Stepper Motor IN2	3
Stepper Motor IN3	6
Stepper Motor IN4	13
Optofork Sensor	28
Piezoelectric Sensor	27
UART TX (LoRa)	4
UART RX (LoRa)	5
________________________________________
Software Setup
Prerequisites:
•	Raspberry Pi Pico SDK
•	GCC toolchain for ARM-based devices
•	CMake for project build
Compilation:
1.	Clone the repository containing the source code.
2.	Navigate to the project directory:
cd <project-directory>
3.	Build the project using CMake:
mkdir build
cd build
cmake ..
make
4.	Flash the compiled binary to the Raspberry Pi Pico using a USB connection.
Libraries Used:
•	pico/stdlib.h for GPIO and system functions.
•	hardware/gpio.h for GPIO control.
•	hardware/uart.h for UART communication.
________________________________________
Functional Details
States:
The dispenser operates in the following states:
1.	STATE_CHECK_LORA_CONNECTION:
o	Sends AT commands to verify connectivity with the LoRa module.
o	Proceeds to the next state upon successful response.
2.	STATE_CONNECT_LORA_TO_NETWORK:
o	Configures the LoRa module for OTAA (Over-the-Air Activation).
o	Sends a test message upon successful connection.
3.	STATE_WAIT:
o	Enters idle mode where the system awaits user input.
o	Indicates the current system status using LEDs.
4.	STATE_CALIBRATION:
o	Rotates the stepper motor to calibrate the system.
o	Calculates the number of steps per revolution for precise dispensing.
5.	STATE_DISPENSE_PILLS:
o	Dispenses pills by rotating the stepper motor to specific compartments.
o	Detects pill drops via the piezoelectric sensor.
________________________________________
Usage Instructions
1.	Power On: Connect the Raspberry Pi Pico to a power source.
2.	Idle Mode:
o	The system enters idle mode by default.
o	The LED blinks to indicate the system's readiness.
3.	Calibration:
o	Press the calibration button (SW_2) to start the motor calibration process.
o	Wait for the system to complete calibration before dispensing pills.
4.	Dispensing Pills:
o	Press the dispense button (SW_0) to initiate pill dispensing.
o	The system will attempt to dispense one pill per compartment, verifying each drop using the piezoelectric sensor.
5.	LoRaWAN Connection:
o	Ensure the LoRa module is connected.
o	The system will automatically attempt to join a LoRa network and send test messages.
________________________________________
Troubleshooting
LED Patterns:
•	Blinking LED: Indicates system is uncalibrated or waiting for action.
•	Steady LED: Indicates the system is calibrated and ready to dispense pills.
•	Error Blinks: LED blinks five times to indicate a pill-dispensing error.
Common Issues:
1.	Motor Not Moving:
o	Check the wiring and ensure the motor is connected to the correct GPIO pins.
2.	LoRa Connection Failure:
o	Verify the LoRa module's baud rate and connections.
o	Check for correct AT command responses.
3.	No Pill Detected:
o	Ensure the piezoelectric sensor is functioning and properly connected.
________________________________________
Future Improvements
•	Add a real-time clock (RTC) for timed dispensing functionality.
•	Enhance error recovery mechanisms for failed pill drops.
•	Integrate a display module for user feedback and system status.
•	Implement secure data transmission for LoRaWAN communication.
________________________________________
License
This project is licensed under the MIT License. Feel free to use, modify, and distribute it as needed.

