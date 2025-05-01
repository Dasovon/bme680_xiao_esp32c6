# BME680 XIAO ESP32C6

This repository contains code and documentation for interfacing the BME680 environmental sensor with the Seeed Studio XIAO ESP32C6 microcontroller. The project demonstrates how to read temperature, humidity, pressure, and gas resistance data using the I2C protocol.

## Features
- Reads environmental data (temperature, humidity, pressure, gas resistance) from the BME680 sensor.
- Utilizes the Seeed Studio XIAO ESP32C6 for compact and efficient processing.
- Implements I2C communication for sensor interfacing.
- Example code for easy integration into IoT or environmental monitoring projects.

## Hardware Requirements
- Seeed Studio XIAO ESP32C6
- BME680 Environmental Sensor
- Jumper wires
- (Optional) Breadboard for prototyping

## Software Requirements
- Arduino IDE or PlatformIO
- [Adafruit BME680 Library](https://github.com/adafruit/Adafruit_BME680)
- [Wire Library](https://www.arduino.cc/en/Reference/Wire) (included with Arduino IDE)
- ESP32 Core for Arduino (install via Boards Manager)

## Installation
1. **Clone the Repository**:
   ```bash
   git clone https://github.com/Dasovon/bme680_xiao_esp32c6.git
   ```
2. **Install Dependencies**:
   - Open Arduino IDE or PlatformIO.
   - Install the Adafruit BME680 library via the Library Manager.
   - Ensure the ESP32 Core is installed (see [Seeed Studio's guide](https://wiki.seeedstudio.com/xiao_esp32c6_getting_started/)).
3. **Connect Hardware**:
   - Connect the BME680 to the XIAO ESP32C6 via I2C:
     - SDA (BME680) → Pin D4 (XIAO ESP32C6)
     - SCL (BME680) → Pin D5 (XIAO ESP32C6)
     - VCC (BME680) → 3.3V (XIAO ESP32C6)
     - GND (BME680) → GND (XIAO ESP32C6)
4. **Upload Code**:
   - Open the `bme680_xiao.ino` sketch in the `src` folder.
   - Select the XIAO ESP32C6 board in the Arduino IDE.
   - Upload the code to the microcontroller.

## Usage
- After uploading the code, open the Serial Monitor (baud rate: 115200) to view sensor readings.
- The sketch will output temperature (°C), humidity (%), pressure (hPa), and gas resistance (KOhms) at regular intervals.
- Modify the code in `src/bme680_xiao.ino` to adjust the sampling rate or add custom functionality.

## Folder Structure
```
bme680_xiao_esp32c6/
├── src/
│   └── bme680_xiao.ino    # Main Arduino sketch
├── docs/
│   └── wiring_diagram.png # Wiring diagram for BME680 and XIAO ESP32C6
├── LICENSE                # License file
└── README.md              # This file
```

## Contributing
Contributions are welcome! Please follow these steps:
1. Fork the repository.
2. Create a new branch (`git checkout -b feature-branch`).
3. Make your changes and commit (`git commit -m "Add feature"`).
4. Push to the branch (`git push origin feature-branch`).
5. Open a Pull Request.

## License
This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

## Acknowledgments
- [Adafruit](https://www.adafruit.com/) for the BME680 library.
- [Seeed Studio](https://www.seeedstudio.com/) for the XIAO ESP32C6 documentation.