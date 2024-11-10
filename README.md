# mmc5603-library-espressif

## Introduction

This library allows you to use the MMC5603 magnetometer sensor with ESP32 microcontrollers under the ESP-IDF framework. It provides essential functionalities for reading magnetic field data, configuring the sensor, and managing temperature. Whether you're developing motion detection, navigation applications, or any other project requiring precise magnetic field measurements, this library offers the necessary tools to easily integrate the MMC5603 into your ESP32 project.

## Features

- **Magnetic Field Data Reading**: Measure magnetic fields on the X, Y, and Z axes.
- **Advanced Sensor Configuration**: Adjust settings such as power mode and automatic calibration.
- **Temperature Management**: Obtain the sensor's internal temperature for precise calibration.
- **Flexible Operating Modes**: Enable continuous mode or configure single measurements as needed.
- **Extended Support**: Compatible with multiple ESP32 variants, including ESP32-S3.

## Prerequisites

- **ESP-IDF**: Version 5.3.1 or higher.
- **ESP32 Microcontroller**: Confirmed compatibility with ESP32-S3.
- **MMC5603 Sensor**: Ensure your sensor is properly connected (3.5V to 5V for the Adafruit model).

## Installation

1. **Clone the GitHub Repository**:

    ```bash
    git clone https://github.com/Bertrand-selvasystems/mmc5603-library-espressif.git
    ```

2. **Integrate the Library into Your ESP-IDF Project**:

    Place the `mmc5603-library-espressif` folder into the `components` directory of your ESP-IDF project.

    ```bash
    mkdir -p your_project/components
    cp -r mmc5603-library-espressif your_project/components/
    ```

3. **Configure `CMakeLists.txt`**:

    Ensure that your `CMakeLists.txt` file includes the MMC5603 library.

    ```cmake
    cmake_minimum_required(VERSION 3.5)

    include($ENV{IDF_PATH}/tools/cmake/project.cmake)
    project(your_project)

    set(EXTRA_COMPONENT_DIRS components/mmc5603-library-espressif)
    ```

## Configuration

Before using the library, configure the I2C pins and other necessary parameters in your code or via `menuconfig` if applicable.

### Example I2C Pin Configuration

```c
#define I2C_MASTER_SCL_IO           16    /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           17    /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              I2C_NUM_0 /*!< I2C port number for master device */
#define I2C_MASTER_FREQ_HZ          100000 /*!< I2C master clock frequency */
```

## Function Documentation
### esp_err_t mmc5603_init()

Description: Initializes the MMC5603 sensor by configuring the initial parameters for reading magnetic field data.

Arguments: None
Return: esp_err_t
* ESP_OK on success
* Error code otherwise
Details: Prepares the sensor for measurements by enabling necessary initial settings.

### esp_err_t mmc5603_read_single_measurement(float *x, float *y, float *z, bool autoset)

Description: Reads a single magnetic field measurement for the X, Y, and Z axes.

Arguments:
* float *x: Pointer to store the X-axis magnetic field value.
* float *y: Pointer to store the Y-axis magnetic field value.
* float *z: Pointer to store the Z-axis magnetic field value.
* bool autoset: Enable or disable automatic SET mode.

Return: esp_err_t
* ESP_OK if the read was successful
* ESP_FAIL otherwise

Details: Performs a magnetic field value read for each axis and stores them in the provided pointers.

### float mmc5603_read_temperature()

Description: Reads the current temperature measured by the sensor.

Arguments: None

Return: float Temperature value in degrees Celsius

Details: Retrieves the internal temperature of the sensor, useful for thermal calibration.

### esp_err_t mmc5603_do_set()

Description: Performs a SET operation on the sensor.

Arguments: None

Return: esp_err_t
* ESP_OK on success
* Error code otherwise

Details: Applies a SET operation to enhance measurement accuracy.

### esp_err_t mmc5603_do_reset()

Description: Performs a RESET operation on the sensor.

Arguments: None

Return: esp_err_t
* ESP_OK on success
* Error code otherwise

Details: Resets the sensor to restore default settings.

### esp_err_t mmc5603_get_offset(float *offset_x, float *offset_y, float *offset_z)

Description: Calculates and retrieves offsets for each axis X, Y, and Z.

Arguments:
* float *offset_x: Pointer to store the X-axis offset.
* float *offset_y: Pointer to store the Y-axis offset.
* float *offset_z: Pointer to store the Z-axis offset.

Return: esp_err_t
* ESP_OK on success
* Error code otherwise

Details: Calculates offsets to correct biases in magnetic field measurements.

### esp_err_t mmc5603_self_test()

Description: Executes a self-test of the sensor to verify its proper functioning.

Arguments: None

Return: esp_err_t
* ESP_OK on success
* Error code otherwise

Details: Requests the sensor to perform a self-test sequence, verifying the validity of its data. The auto_st_en bit resets after the operation.

### void mmc5603_get_informations()

Description: Displays information from the sensor's configuration registers.

Arguments: None

Return: None

Details: Reads multiple configuration registers from the MMC5603 device and displays their values along with interpretations of specific bits in each register. In case of a read error, an error message is displayed for each affected register.

### esp_err_t mmc5603_reset()

Description: Performs a software reset on the sensor.

Arguments: None

Return: esp_err_t
* ESP_OK on success
* Error code otherwise

Details: Resets the sensor by clearing current configurations and restoring default settings.

### esp_err_t mmc5603_set_auto_sr(bool enable)

Description: Enables or disables the sensor's automatic SET/RESET function.

Arguments:
* bool enable: true to enable, false to disable

Return: esp_err_t
* ESP_OK on success
* Error code otherwise

Details: Enables or disables the automatic SET/RESET functionality to improve measurement accuracy.

### esp_err_t mmc5603_set_odr(uint8_t frequency_hz)

Description: Sets the acquisition frequency (ODR - Output Data Rate).

Arguments:
* uint8_t frequency_hz: Frequency in Hertz (0 for 1000 Hz)

Return: esp_err_t
* ESP_OK if the operation was successful
* Error code otherwise

Details: Configures the sensor's data output frequency.

### esp_err_t mmc5603_set_hpower(bool enable)

Description: Enables or disables high-power mode (required for 1000 Hz).

Arguments:
* bool enable: true to enable, false to disable

Return: esp_err_t
* ESP_OK on success
* Error code otherwise

Details: Enables or disables high-power mode for high-frequency measurements.

### esp_err_t mmc5603_enable_continuous_mode(uint8_t acquisition_frequency_hz)

Description: Enables continuous mode with a specified acquisition frequency.

Arguments:
* uint8_t acquisition_frequency_hz: Acquisition frequency in Hertz

Return: esp_err_t
* ESP_OK on success
* Error code otherwise

Details: Configures the sensor for continuous measurements with the specified acquisition frequency.

### esp_err_t mmc5603_disable_continuous_mode()

Description: Disables the sensor's continuous mode.

Arguments: None

Return: esp_err_t
* ESP_OK if the operation was successful
* Error code otherwise

Details: Stops continuous measurements, allowing for single-point readings.

### esp_err_t mmc5603_set_periodic_set(uint8_t prd_set_value)

Description: Sets the Prd_set configuration for the MMC5603 sensor.

Arguments:
* uint8_t prd_set_value: Desired value for Prd_set (0-7)

Return: esp_err_t
* ESP_OK on success
* Error code otherwise
Details: Configures the Prd_set bits to define the number of measurements taken before an automatic SET operation is executed. This feature only works in continuous mode with both En_prd_set and Auto_SR enabled.
Prd_set values and corresponding measurement intervals:
0b000: Execute SET after every 1 sample
0b001: Execute SET after every 25 samples
0b010: Execute SET after every 75 samples
0b011: Execute SET after every 100 samples
0b100: Execute SET after every 250 samples
0b101: Execute SET after every 500 samples
0b110: Execute SET after every 1000 samples
0b111: Execute SET after every 2000 samples

Required Conditions:
* The sensor must be in continuous mode.
* En_prd_set (bit 3) and Auto_SR (automatic SET/RESET) must both be enabled.

### esp_err_t mmc5603_periodic_set_enable(bool enable)

Description: Enables or disables periodic SET based on Prd_set.

Arguments:
* bool enable: true to enable periodic SET; false to disable periodic SET

Return: esp_err_t
* ESP_OK if the operation succeeded
* Error code otherwise

Details: Enables or disables the periodic SET functionality, requiring the sensor to be in continuous mode, Auto_SR enabled, and Prd_set configured with the number of samples before each SET cycle.

### esp_err_t mmc5603_set_permanent_set_mode(bool enable)

Description: Enables or disables the sensor's permanent SET function.

Arguments:
* bool enable: true to enable, false to disable

Return: esp_err_t
* ESP_OK on success
* Error code otherwise

Details: Sets the St_enp bit in the CTRL1 register to apply a continuous positive current for permanent SET. The function respects other static configuration settings, such as the bw value (bandwidth) on bits 0 and 1.

### esp_err_t mmc5603_disable_periodic_set()

Description: Disables the periodic SET function.

Arguments: None

Return: esp_err_t
* ESP_OK if the operation was successful
* Error code otherwise

Details: Disables the periodic SET by clearing the En_prd_set bit (bit 3) in the CTRL2 register while preserving other settings, including Prd_set, continuous mode, and high-power mode.

### esp_err_t mmc5603_set_permanent_reset_mode(bool enable)

Description: Performs a permanent RESET operation by enabling or disabling St_enm while preserving the bw value.

Arguments:
* bool enable: true to enable permanent RESET, false to disable

Return: esp_err_t
* ESP_OK on success
* Error code otherwise

Details: Activates or deactivates the St_enm bit in the CTRL1 register to apply a constant opposing current for a permanent RESET, while preserving other configurations such as the bandwidth bw.

### esp_err_t mmc5603_set_bandwidth(uint8_t bw_value)

Description: Sets the bandwidth (BW) for acquisition duration while preserving other settings in the CTRL1 register.

Arguments:
* uint8_t bw_value: Bandwidth value (0-3) corresponding to different acquisition durations.

Return: esp_err_t
* ESP_OK if the operation was successful
* Error code otherwise

Details: Configures the bandwidth for acquisition duration, which controls the measurement time:
* 0: 6.6 ms measurement time
* 1: 3.5 ms
* 2: 2 ms
* 3: 1.2 ms

## Usage Example

Here's an example illustrating how to use this library with an ESP32.
```c
#include "mmc5603.h"

void app_main() {
    // Initialize the MMC5603 sensor
    esp_err_t ret = mmc5603_init();
    if (ret != ESP_OK) {
        printf("Error initializing MMC5603 sensor\n");
        return;
    }
    
    float x, y, z;
    ret = mmc5603_read_single_measurement(&x, &y, &z, true);
    if (ret == ESP_OK) {
        printf("Magnetic Data - X: %.2f, Y: %.2f, Z: %.2f\n", x, y, z);
    } else {
        printf("Error reading magnetic data\n");
    }

    float temperature = mmc5603_read_temperature();
    printf("Temperature: %.2f°C\n", temperature);

    // Enable low-power mode
    ret = mmc5603_set_power_mode(true);
    if (ret != ESP_OK) {
        printf("Error enabling low-power mode\n");
    }
    
    // Perform a self-test
    ret = mmc5603_self_test();
    if (ret != ESP_OK) {
        printf("Error performing sensor self-test\n");
    }

    // Set bandwidth
    ret = mmc5603_set_bandwidth(2);
    if (ret != ESP_OK) {
        printf("Error setting bandwidth\n");
    }

    // Enable continuous mode with an acquisition frequency of 100 Hz
    ret = mmc5603_enable_continuous_mode(100);
    if (ret != ESP_OK) {
        printf("Error enabling continuous mode\n");
    }

    // Get configuration information
    mmc5603_get_informations();
}
```

## Troubleshooting

### Reading Issues:
* Verify I2C connections and ensure the communication frequency is correct (<= 400000 Hz).
* Ensure the sensor is properly powered.
* Make sure you have a 2.2 kΩ pull-up resistor between 3.3V and SDA/SCL or set pushup option in I2C module.

## Support

For any technical questions or bug reports, please contact me at Bertrand.selva@tutamail.com

## License

This project is licensed under the MIT License. For more details, see the LICENSE file.
