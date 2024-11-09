#ifndef MMC5603_H
#define MMC5603_H

/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////// INCLUDE LIBRARIES
/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "esp_log.h"
#include "init_i2c.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////// CONSTANT DEFINITIONS
/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
#define I2C_MASTER_NUM I2C_NUM_0
#define TAG "MMC5603"

// I2C Address for the sensor
#define MMC5603NJ_ADDR 0x30

// Register Definitions
#define PRODUCT_ID_REG 0x39
#define EXPECTED_PRODUCT_ID 0x10

// Configuration register adress
#define CTRL0_REG 0x1B    ///< Internal control register 0
#define CTRL1_REG 0x1C    ///< Internal control register 1
#define CTRL2_REG 0x1D    ///< Internal control register 2
#define STATUS_REG 0x18   ///< Status register
#define OUT_X_L 0x00      ///< X-axis output register (low byte)
#define TEMP_OUT 0x09     ///< Temperature output register
#define ODR_REG 0x1A      ///< Output Data Rate (ODR) register
#define RESET_REG 0x1B    ///< Reset register
#define SELF_TEST_X 0x27  ///< Self-test register for X-axis
#define SELF_TEST_Y 0x28  ///< Self-test register for Y-axis
#define SELF_TEST_Z 0x29  ///< Self-test register for Z-axis
#define ST_X_TH 0x1E      ///< Self-test threshold for X-axis
#define ST_Y_TH 0x1F      ///< Self-test threshold for Y-axis
#define ST_Z_TH 0x20      ///< Self-test threshold for Z-axis

#define XOUT0 0x00  ///< Most significant byte of the X-axis
#define XOUT1 0x01  ///< Intermediate byte of the X-axis
#define XOUT2 0x06  ///< Least significant byte of the X-axis
#define YOUT0 0x02  ///< Most significant byte of the Y-axis
#define YOUT1 0x03  ///< Intermediate byte of the Y-axis
#define YOUT2 0x07  ///< Least significant byte of the Y-axis
#define ZOUT0 0x04  ///< Most significant byte of the Z-axis
#define ZOUT1 0x05  ///< Intermediate byte of the Z-axis
#define ZOUT2 0x08  ///< Least significant byte of the Z-axis

/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////// EXTERN VARIABLES
/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
extern float mmc5603_offset_x, mmc5603_offset_y, mmc5603_offset_z;

/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////// FUNCTION PROTOTYPES
/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Initialize the MMC5603 sensor.
 * @return esp_err_t ESP_OK on success, error code otherwise.
 */
esp_err_t mmc5603_init();

/**
 * @brief Read a single magnetic field measurement for X, Y, and Z axes.
 * @param x Pointer to store X-axis value.
 * @param y Pointer to store Y-axis value.
 * @param z Pointer to store Z-axis value.
 * @param autoset Enable or disable automatic SET mode.
 * @return true if read successful, false otherwise.
 */
esp_err_t mmc5603_read_single_measurement(float *x, float *y, float *z);

/**
 * @brief Read the temperature from the sensor.
 * @return Measured temperature in degrees Celsius.
 */
float mmc5603_read_temperature();

/**
 * @brief Perform a SET operation on the sensor.
 * @return esp_err_t ESP_OK on success, error code otherwise.
 */
esp_err_t mmc5603_do_set();

/**
 * @brief Perform a RESET operation on the sensor.
 * @return esp_err_t ESP_OK on success, error code otherwise.
 */
esp_err_t mmc5603_do_reset();

/**
 * @brief Calculate and retrieve offsets for each axis X, Y, and Z.
 * @param offset_x Pointer to store X-axis offset.
 * @param offset_y Pointer to store Y-axis offset.
 * @param offset_z Pointer to store Z-axis offset.
 * @return esp_err_t ESP_OK on success, error code otherwise.
 */
esp_err_t mmc5603_get_offset(float *offset_x, float *offset_y, float *offset_z);

/**
 * @brief Execute the sensor's self-test.
 * @return esp_err_t ESP_OK on success, error code otherwise.
 */
esp_err_t mmc5603_self_test();

/**
 * @brief Display information from configuration registers.
 */
void mmc5603_get_informations();

/**
 * @brief Perform a software reset on the sensor.
 * @return esp_err_t ESP_OK on success, error code otherwise.
 */
esp_err_t mmc5603_reset();

/**
 * @brief Enable or disable the automatic SET/RESET function.
 * @param enable true to enable, false to disable.
 * @return esp_err_t ESP_OK on success, error code otherwise.
 */
esp_err_t mmc5603_set_auto_sr(bool enable);

/**
 * @brief Set the acquisition frequency (ODR - Output Data Rate).
 *
 * @param frequency_hz Frequency in Hertz (0 for 1000 Hz).
 * @return esp_err_t ESP_OK on success, error code otherwise.
 *
 */
esp_err_t mmc5603_set_odr(uint8_t frequency_hz);

/**
 * @brief Enable or disable high-power mode (required for 1000 Hz).
 * @param enable true to enable, false to disable.
 * @return esp_err_t ESP_OK on success, error code otherwise.
 */
esp_err_t mmc5603_set_hpower(bool enable);

/**
 * @brief Enable continuous mode with specified acquisition frequency and bandwidth setting.
 * @param acquisition_frequency_hz Acquisition frequency in Hertz.
 * @return esp_err_t ESP_OK on success, error code otherwise.
 */
esp_err_t mmc5603_enable_continuous_mode(uint8_t acquisition_frequency_hz);

/**
 * @brief Disable the sensor's continuous mode.
 * @return esp_err_t ESP_OK on success, error code otherwise.
 */
esp_err_t mmc5603_disable_continuous_mode();

/**
 * @brief Set the Prd_set configuration for the MMC5603 sensor.
 *
 * This function configures the `Prd_set` bits to define the number of measurements
 * taken before an automatic SET operation is executed. This feature only works in
 * continuous mode with both `En_prd_set` and `Auto_SR` enabled.
 *
 * - `Prd_set` values and corresponding measurement intervals:
 *     - 0b000: Execute SET after every 1 sample
 *     - 0b001: Execute SET after every 25 samples
 *     - 0b010: Execute SET after every 75 samples
 *     - 0b011: Execute SET after every 100 samples
 *     - 0b100: Execute SET after every 250 samples
 *     - 0b101: Execute SET after every 500 samples
 *     - 0b110: Execute SET after every 1000 samples
 *     - 0b111: Execute SET after every 2000 samples
 *
 * Note: For this feature to function, the following conditions must be met:
 * 1. The sensor must be in continuous mode.
 * 2. `En_prd_set` (bit 3) and `Auto_SR` (automatic SET/RESET) must both be enabled.
 *
 * @param prd_set_value The desired value for Prd_set (0-7).
 * @return esp_err_t ESP_OK on success, or an error code otherwise.
 */
esp_err_t mmc5603_set_periodic_set(uint8_t prd_set_value);

/**
 * @brief Enables or disables periodic SET based on Prd_set.
 *
 * For this function to be effective, the sensor must be in continuous mode,
 * `Auto_SR` must be enabled, and `Prd_set` must be configured with the number
 * of samples before each SET cycle.
 *
 * @param enable If true, enables periodic SET; if false, disables periodic SET.
 * @return esp_err_t ESP_OK if the operation succeeded, an error otherwise.
 */
esp_err_t mmc5603_periodic_set_enable(bool enable);

/**
 * @brief Enable or disable the permanent SET function on the sensor.
 *
 * This function sets the `St_enp` bit in CTRL1 register to apply a continuous
 * positive current for permanent SET. The function respects other static configuration
 * settings, such as the `bw` value (bandwidth) on bits 0 and 1.
 *
 * @param enable True to enable the permanent SET function, false to disable it.
 * @return esp_err_t ESP_OK if successful, or an error code indicating failure.
 */
esp_err_t mmc5603_set_permanent_set_mode(bool enable);

/**
 * @brief Disable the periodic set function.
 *
 * This function disables the periodic set by clearing the En_prd_set bit (bit 3) in the CTRL2 register,
 * while preserving other settings, including Prd_set, continuous mode, and high power mode.
 *
 * @return esp_err_t Returns ESP_OK if the operation is successful, or an error code otherwise.
 */
esp_err_t mmc5603_disable_periodic_set();

/**
 * @brief Perform a permanent RESET operation by enabling or disabling St_enm with preserved BW value.
 *
 * Cette fonction active ou désactive le bit `St_enm` dans le registre CTRL1 pour appliquer
 * un courant constant de sens opposé pour un RESET permanent, en préservant les autres
 * configurations comme la largeur de bande `bw`.
 *
 * @param enable True pour activer le RESET permanent, false pour le désactiver.
 * @return esp_err_t ESP_OK si succès, ou un code d'erreur indiquant l'échec.
 */
esp_err_t mmc5603_set_permanent_reset_mode(bool enable);

/**
 * @brief Set the bandwidth (BW) for acquisition duration while preserving other settings in CTRL1 register.
 *
 * This function configures the bandwidth for the acquisition duration, which controls the measurement time:
 * - 0: 6.6 ms measurement time
 * - 1: 3.5 ms
 * - 2: 2 ms
 * - 3: 1.2 ms
 *
 * @param bw_value The bandwidth value (0-3) corresponding to different acquisition durations.
 * @return esp_err_t Returns ESP_OK if the operation is successful, or an error code otherwise.
 */
esp_err_t mmc5603_set_bandwidth(uint8_t bw_value);

/**
 * @brief Reads and displays detailed configuration register information of the MMC5603 sensor.
 *
 * This function reads multiple configuration registers from the MMC5603 device and
 * displays their values along with the interpretation of specific bits in each register.
 * In case of a read error, an error message is displayed for each affected register.
 */
void mmc5603_get_informations();

#endif  // MMC5603_H
