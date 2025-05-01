# BME680 XIAO ESP32C6

This repository contains code and documentation for interfacing the BME680 environmental sensor with the Seeed Studio XIAO ESP32C6 microcontroller using the ESP-IDF framework. The project demonstrates how to read temperature, humidity, pressure, and gas resistance data via the I2C protocol.

## Features
- Reads environmental data (temperature, humidity, pressure, gas resistance) from the BME680 sensor.
- Utilizes the Seeed Studio XIAO ESP32C6 for compact and efficient processing.
- Implements I2C communication using ESP-IDF drivers.
- Example code for integration into IoT or environmental monitoring projects.

## Hardware Requirements
- Seeed Studio XIAO ESP32C6
- BME680 Environmental Sensor
- Jumper wires
- (Optional) Breadboard for prototyping

## Software Requirements
- [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c6/get-started/index.html) (v5.0 or later)
- [Visual Studio Code](https://code.visualstudio.com/) with the [ESP-IDF Extension](https://marketplace.visualstudio.com/items?itemName=espressif.esp-idf-extension)
- [Bosch BME68x Library](https://github.com/boschsensortec/Bosch-BME68x-Library.git)

## Installation
1. **Clone the Repository**:
   ```bash
   git clone https://github.com/Dasovon/bme680_xiao_esp32c6.git
   ```
2. **Set Up ESP-IDF**:
   - Follow the [ESP-IDF Getting Started Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c6/get-started/index.html) to install ESP-IDF and configure the environment.
   - Ensure the ESP-IDF extension is installed in VS Code.
3. **Install Bosch BME68x Library**:
   - Clone the Bosch BME68x Library into the `components` folder:
     ```bash
     cd bme680_xiao_esp32c6/components
     git clone https://github.com/boschsensortec/Bosch-BME68x-Library.git
     ```
4. **Connect Hardware**:
   - Connect the BME680 to the XIAO ESP32C6 via I2C:
     - SDA (BME680) → Pin 22 (XIAO ESP32C6)
     - SCL (BME680) → Pin 23 (XIAO ESP32C6)
     - VCC (BME680) → 3.3V (XIAO ESP32C6)
     - GND (BME680) → GND (XIAO ESP32C6)
5. **Build and Flash**:
   - Open the project in VS Code.
   - Use the ESP-IDF extension to set the target to `esp32c6` (`idf.py set-target esp32c6`).
   - Build the project (`idf.py build`).
   - Flash the code to the XIAO ESP32C6 (`idf.py -p PORT flash`).
   - Monitor output (`idf.py monitor`).

## Usage
- After flashing, use the ESP-IDF monitor (baud rate: 115200) to view sensor readings.
- The code outputs temperature (°C), humidity (%), pressure (hPa), and gas resistance (KOhms) at regular intervals.
- Modify the code in `main/main.c` to adjust the sampling rate or add custom functionality.

## Folder Structure
```
bme680_xiao_esp32c6/
├── components/
│   └── bme68x/ # From the Bosch BME68x Library
├── main/
│   ├── main.c                # Main application code
│   └── CMakeLists.txt        # Main component build configuration
├── CMakeLists.txt            # Project build configuration
├── sdkconfig                 # ESP-IDF configuration
├── LICENSE                   # License file
└── README.md                 # This file
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
- [Bosch Sensortec](https://github.com/boschsensortec) for the BME68x Library.
- [Seeed Studio](https://www.seeedstudio.com/) for the XIAO ESP32C6 documentation.
- [Espressif Systems](https://www.espressif.com/) for the ESP-IDF framework.