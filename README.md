# Smart Cleaning Robot

This is a smart cleaning robot that uses ultrasonic sensors to detect objects and navigate around them. It also has a pump that can be used to spray water for wet cleaning. The robot is controlled via the Arduino Serial Monitor interface that allows the user to select between dry and wet cleaning modes.

## Features

* Dry and wet cleaning modes
* Ultrasonic sensors for object detection and crack detection
* Water pump system with level monitoring
* LCD display interface for mode selection and status
* Dust level monitoring
* Intelligent obstacle avoidance navigation
* Real-time sensor data display

## Hardware Requirements

* Arduino Nano
* HC-SR04 Ultrasonic Sensor
* 5v mini water pump
* L293D_MOTOR_DRIVER
* LCD screen 16x2 I2C
* Battery 12v
* Motor drivers
* Water Lever Sensor
* Motor and Wheels
* Dust Sensor
* Jumper Wires


## Software Requirements

* Arduino IDE

## Installation

Step 1. Install the Arduino IDE </br>
Step 2. Connect the hardware components according to the following steps:
   - Connect 4 HC-SR04 ultrasonic sensors to pins 2-5, 7-8, and A0-A1
   - Connect the L293D motor driver and motors to pins 6, 9, 10, and 11
   - Connect the 5V water pump to pin 13
   - Connect the LCD screen via I2C (SDA/SCL pins)
   - Connect the water level sensor to pin A3
   - Connect the dust sensor to pin A2
   - Power the system with a 12V battery

Step 3. Install required libraries:
- NewPing
- LiquidCrystal_I2C
- Wire
</br>
Step 4. Upload the code to the Arduino Nano </br>
Step 5. Test the system by selecting a cleaning mode via serial monitor

## Usage

1. Power on the robot and wait for the "Choose Mode" message on the LCD
2. Open the Arduino Serial Monitor (baud rate 9600)
3. Send '1' for dry cleaning mode or '2' for wet cleaning mode
4. The LCD will display the selected mode and the robot will begin cleaning
5. In dry cleaning mode:
   - Robot navigates using ultrasonic sensors to avoid obstacles
   - Dust sensor monitors air quality
   - LCD displays sensor readings and status
6. In wet cleaning mode:
   - All dry cleaning features plus water spraying
   - Pump activates every 10 seconds if water level is sufficient
   - LCD indicates "Spraying Water" or "Dry Cleaning" based on water level
7. The robot will automatically:
   - Stop and display warning if high dust levels detected
   - Navigate around obstacles and edges
   - Alert when cracks are detected in the floor
8. To stop cleaning, reset or power off the robot


## Diagram

<img src="https://raw.githubusercontent.com/anvng/SmartCleaning/refs/heads/master/documents/img/diagram.png" alt="Figure 1. Model application development direction" 
/> <br> Figure 1. Model application development direction

## Contributing

This project is open source and welcomes contributions. To contribute, simply fork the repository and submit a pull request with your changes.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.
