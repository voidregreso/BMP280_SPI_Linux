# BMP280 Sensor Data Reader

This C program reads temperature and pressure data from a BMP280 sensor using software SPI communication protocol on Linux embedded devices. It calculates the water boiling point and altitude based on the pressure reading. 

## How it Works

The program communicates with the BMP280 sensor over SPI protocol to read temperature and pressure data. It utilizes the provided calibration coefficients to compensate for sensor errors. The temperature and pressure readings are then used to calculate the water boiling point and altitude using established formulas. The program runs a background thread to continuously poll the sensor at a predefined interval.

## Implementation Details

The program consists of several functions:
- `read8`, `read16`, `readS16`, `read16_LE`, `readS16_LE`: These functions read 8-bit and 16-bit values from the BMP280 registers over SPI.
- `readCoefficients`: Reads the factory-set calibration coefficients from the sensor.
- `readTemperature`: Reads the temperature data from the sensor and compensates it using calibration coefficients.
- `readPressure`: Reads the pressure data from the sensor and compensates it using calibration coefficients.
- `waterBoilingPoint`: Calculates the boiling point of water at a given pressure using the Magnus formula.
- `getAltitude`: Calculates the altitude based on the current pressure and a reference sea level pressure.

## Sensor Wiring

Ensure correct wiring of the BMP280 sensor to your embedded device:
- **VCC**: Connect to 3.3V
- **GND**: Connect to GND
- **SDA**: Connect to SPIx_MOSI (Master Out Slave In). Replace `x` with your actual SPI channel.
- **SDO**: Connect to SPIx_MISO (Master In Slave Out). Replace `x` with your actual SPI channel.
- **CSB**: Connect to SPIx_CSy (Chip Select). Replace `x` with your actual SPI channel and `y` with your actual chip number.
- **SCL**: Connect to SPIx_SCLK (Serial Clock). Replace `x` with your actual SPI channel.

## Compilation

To compile the program, ensure you have the necessary dependencies and follow these steps:

1. Clone the repository.
2. Navigate to the repository directory.
3. Compile the program using the following command:
    ```
    xxxxxx-gcc -I ./linux-4.18/include/ BMP280_app.c -o BMP280_app -lm -pthread
    ```
    Replace `xxxxxx-gcc` with your compiler command, and make sure to provide the correct path to the kernel source tree's include directory (`./linux-4.18/include/`). 
    Ensure to link the math library `-lm` and the pthread library `-pthread`.

## Dependencies

- Linux SPI device driver
- SPI kernel module
- pthread library
- math library

## Usage

After compiling the program, run the executable. The program will continuously output temperature, pressure, water boiling point, and altitude readings until interrupted (e.g., by pressing Ctrl+C).

## Contributing

Contributions are welcome! Please feel free to open issues or submit pull requests.

## License

This project is licensed under the [MIT License](LICENSE).
