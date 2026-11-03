# SmartDrive
### Intelligent Driver Safety & Smart Vehicle Control System

**Developed by Team ZX**

SmartDrive is an ESP32-based smart vehicle safety prototype designed to improve road safety through alcohol detection, intelligent speed limiting, ultrasonic obstacle detection and real-time vehicle monitoring.

The system uses two ESP32 controllers communicating over UART. A web-based dashboard provides the driver with vehicle status, route selection, speed information and safety alerts.

---

## 1. Key Features

- **Alcohol Detection:** MQ-2 sensor-based alcohol detection with sensor warm-up and clean-air baseline measurement.
- **Automatic Vehicle Lock:** Prevents the vehicle from entering driving mode when alcohol is detected.
- **Intelligent Speed Limiter:** Limits vehicle speed according to the selected route's speed limit.
- **Obstacle Detection:** Ultrasonic sensor monitors the distance ahead.
- **Automatic Braking:** Reduces speed when an obstacle approaches and stops the motors at the configured braking distance.
- **Real-Time Dashboard:** Displays speed, speed limits, obstacle warnings and trip progress.
- **OLED Display:** Shows vehicle speed, requested speed and system status.
- **Route Selection:** Offers five predefined routes with individual speed limits.
- **Wireless Connectivity:** ESP32 A hosts a Wi-Fi-based dashboard accessible through a browser.
- **Dual ESP32 Architecture:** Separates vehicle control and safety sensing for modular operation.

## 2. System Architecture

SmartDrive uses two ESP32 boards.

### ESP32 A – Main Controller

Responsibilities:
- Hosts the Wi-Fi web server.
- Serves the SmartDrive dashboard.
- Handles route selection and trip management.
- Reads the rotary encoder for speed requests.
- Reads the physical start button.
- Calculates vehicle speed and applies speed limits.
- Controls acceleration and deceleration logic.
- Displays vehicle information on the OLED.
- Communicates with ESP32 B through UART.

### ESP32 B – Safety and Motor Controller

Responsibilities:
- Controls DC motors through the TB6612 motor driver.
- Reads ultrasonic distance measurements.
- Performs automatic obstacle braking.
- Reads MQ-2 alcohol sensor values.
- Measures clean-air baseline and calculates the detection threshold.
- Receives motor PWM commands from ESP32 A.
- Sends sensor readings and safety status back to ESP32 A.
- Stops the motors if communication with ESP32 A is lost.

### Communication Protocol

Both controllers communicate using UART at 115200 baud.

**ESP32 A → ESP32 B**

`S,<PWM_VALUE>`

Sends motor speed commands.

**ESP32 B → ESP32 A**

`D,<DISTANCE>,<GAS_VALUE>,<GAS_THRESHOLD>,<BRAKE>,<ALCOHOL>,<READY>`

Sends obstacle distance, alcohol sensor readings and safety status.

## 3. Hardware Components

| Component | Purpose |
|---|---|
| ESP32 × 2 | Main and safety controllers |
| TB6612FNG | DC motor driver |
| DC Motors | Vehicle movement |
| MQ-2 Sensor | Alcohol/gas sensing |
| HC-SR04 Ultrasonic Sensor | Obstacle detection |
| OLED Display (SSD1306) | Vehicle information |
| Rotary Encoder | Speed input |
| Push Button | Vehicle start control |
| Battery / Power Supply | System power |

## 4. Pin Configuration

### ESP32 A – Main Controller

| Component | ESP32 Pin |
|---|---|
| Encoder CLK | GPIO 32 |
| Encoder DT | GPIO 33 |
| Encoder Switch | GPIO 25 |
| Start Button | GPIO 27 |
| UART RX2 | GPIO 16 |
| UART TX2 | GPIO 17 |
| OLED SDA | GPIO 21 |
| OLED SCL | GPIO 22 |

### ESP32 B – Motor and Safety Controller

| Component | ESP32 Pin |
|---|---|
| TB6612 PWMA | GPIO 26 |
| TB6612 AIN1 | GPIO 13 |
| TB6612 AIN2 | GPIO 14 |
| TB6612 PWMB | GPIO 25 |
| TB6612 BIN1 | GPIO 12 |
| TB6612 BIN2 | GPIO 27 |
| TB6612 STBY | GPIO 33 |
| Ultrasonic TRIG | GPIO 5 |
| Ultrasonic ECHO | GPIO 18 |
| MQ-2 Analog Output | GPIO 34 |
| UART RX2 | GPIO 16 |
| UART TX2 | GPIO 17 |

**UART Connections:**
- ESP32 A TX2 → ESP32 B RX2
- ESP32 B TX2 → ESP32 A RX2
- Connect both ESP32 boards to a common GND.

Use appropriate voltage levels and power supplies. Do not connect a 5V ultrasonic ECHO signal directly to an ESP32 GPIO.

## 5. Software Structure

```text
SmartDrive/
│
├── README.md
│
├── software web/
│   └── screen.html
│
└── firmware/
    ├── esp_aaa.ino
    └── esp_bbb.ino
```

- `screen.html` – Standalone web interface demonstration.
- `esp_aaa.ino` – Main ESP32 firmware, including its integrated hardware-connected web dashboard.
- `esp_bbb.ino` – Secondary ESP32 firmware for motor control and safety sensing.

## 6. Working Principle

1. The driver initiates the system using the dashboard or physical start button.
2. ESP32 A communicates with ESP32 B and initiates the alcohol-check process.
3. ESP32 B performs MQ-2 sensor warm-up and clean-air baseline measurement.
4. The alcohol test runs after the sensor becomes ready.
5. If alcohol is detected, driving remains locked.
6. If the test passes, the driver can select one of the predefined routes.
7. The rotary encoder controls the requested speed.
8. ESP32 A applies the selected route's speed limit.
9. ESP32 B continuously monitors the distance ahead.
10. If an obstacle is detected, the system applies collision warning or automatic braking.
11. Vehicle information is continuously updated on the dashboard and OLED display.

## 7. Safety Mechanisms

- Alcohol detection before enabling driving.
- Route-based speed limitation.
- Obstacle warning and automatic braking.
- Communication timeout protection.
- Motor stop when the controller link is lost.
- Speed clamping to prevent exceeding configured limits.

## 8. Installation and Setup

**Requirements:**
- Arduino IDE
- ESP32 board package
- Two ESP32 development boards
- Required Arduino libraries:
  - Adafruit GFX Library
  - Adafruit SSD1306
- Compatible ESP32 Arduino Core

**Setup:**

1. Configure the Wi-Fi hotspot credentials in ESP32 A firmware.
2. Upload `esp_aaa.ino` to ESP32 A.
3. Upload `esp_bbb.ino` to ESP32 B.
4. Connect both controllers through UART.
5. Connect the motors, sensors and OLED according to the pin configuration.
6. Power both controllers.
7. Connect your laptop or phone to the same Wi-Fi hotspot.
8. Open the IP address printed by ESP32 A in a browser.

## 9. Project Limitations

- MQ-2 readings are prototype-level gas measurements and are not a certified blood-alcohol or breath-alcohol measurement.
- Route positions are calculated using predefined geographic coordinates rather than live GPS hardware.
- The standalone `screen.html` is a demonstration interface; the dashboard embedded in ESP32 A is the hardware-connected version.
- The prototype's displayed photo and emergency-contact messages should not be interpreted as confirmation of actual camera capture or SMS delivery.

## 10. Future Improvements

- GPS hardware integration for live positioning.
- GSM-based emergency notifications.
- Camera integration for driver monitoring.
- Improved alcohol sensing and calibration.
- Cloud-based vehicle monitoring.
- Real-world navigation and dynamic speed-limit updates.

---

**Team ZX | SmartDrive**

*Building safer roads through intelligent embedded systems.*