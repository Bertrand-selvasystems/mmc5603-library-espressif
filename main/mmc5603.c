/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////// DEFINITION DES LIBRAIRIES
/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
#include "mmc5603.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////// DEFINITION DES VARIABLES STATIC
/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
// Variables d'état pour stocker l'état interne
// static bool continuous_mode_enabled = true; // A ENLEVER
// static bool auto_set_reset_enabled = false; // A ENLEVER
// Variables pour stocker l'état des bits non self clearing, qu'il faut conserver au moment de l'écriture
static bool bit_auto_sr_en = false;  // function of auto set reset
static bool bit_En_prd_set = false;  // function of periodic set, bit_auto_sr_en must be on
static bool bit_Cmm_en = false;      // function of continuous mode if ODR != 0 and Cmm_freq_en has been set to on before
static bool bit_hpower = false;      // function high alimentation must be on before 1000 Hz
static bool bit_St_enp = false;      // function permanent SET
static bool bit_St_enm = false;      // function permanent RESET
static uint8_t bw = 0;
static uint8_t Prd_set = 0;

/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////// DEFINITION DES VARIABLES EXTERN
/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
float mmc5603_offset_x = 0;
float mmc5603_offset_y = 0;
float mmc5603_offset_z = 0;

/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////// DEFINITION DES FONCTIONS
/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Read data from a specified register.
 * @param reg Register address.
 * @param data Pointer to store the read data.
 * @param len Number of bytes to read.
 * @return esp_err_t ESP_OK on success, error code otherwise.
 */
/* Read data from a register with specific sequences */
esp_err_t i2c_read_register(uint8_t reg, uint8_t *data, size_t len) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (MMC5603NJ_ADDR << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, reg, true);

  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (MMC5603NJ_ADDR << 1) | I2C_MASTER_READ, true);
  if (len > 1) {
    i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
  }
  i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
  i2c_master_stop(cmd);
  esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to read register 0x%02X: %s", reg, esp_err_to_name(ret));
  }
  return ret;
}

/**
 * @brief Write data to a specified register.
 * @param reg Register address.
 * @param value Value to write.
 * @return esp_err_t ESP_OK on success, error code otherwise.
 */
/* Write data to a register with strict compliance */
static esp_err_t i2c_write_register(uint8_t reg, uint8_t value) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  esp_err_t ret;
  i2c_master_start(cmd);
  ret = i2c_master_write_byte(cmd, (MMC5603NJ_ADDR << 1) | I2C_MASTER_WRITE, true);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to write I2C address: %s", esp_err_to_name(ret));
    i2c_cmd_link_delete(cmd);
    return ret;
  }
  ret = i2c_master_write_byte(cmd, reg, true);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to write register address (0x%02X): %s", reg, esp_err_to_name(ret));
    i2c_cmd_link_delete(cmd);
    return ret;
  }
  ret = i2c_master_write_byte(cmd, value, true);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to write value (0x%02X) to register: %s", value, esp_err_to_name(ret));
    i2c_cmd_link_delete(cmd);
    return ret;
  }
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to execute I2C command: %s", esp_err_to_name(ret));
  }
  vTaskDelay(pdMS_TO_TICKS(5));  // Ensure register stability
  return ret;
}

/* Perform a single magnetic field measurement */
esp_err_t mmc5603_read_single_measurement(float *x, float *y, float *z) {
  const float LSB_TO_NT = 2.86;  // Facteur de conversion en nT par LSB
  uint8_t status;
  uint8_t xout0, xout1, xout2;
  uint8_t yout0, yout1, yout2;
  uint8_t zout0, zout1, zout2;
  int32_t x_raw, y_raw, z_raw;
  esp_err_t ret;

  // Démarrer une mesure magnétique si le mode continu n'est pas activé
  if (!bit_Cmm_en) {
    // Définir le bit 0 (Take_meas_M) et le bit 5 (Auto_SR_en) si activé
    uint8_t measurement_command = (bit_auto_sr_en ? (1 << 5) : 0) | (1 << 0);
    // Bit 5 : Auto_SR_en, Bit 0 : Take_meas_M
    ret = i2c_write_register(CTRL0_REG, measurement_command);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Failed to start magnetic field measurement: %s", esp_err_to_name(ret));
      return ret;
    }

    // Attendre que la mesure soit terminée (bit 6 de STATUS_REG)
    while (1) {
      ret = i2c_read_register(STATUS_REG, &status, 1);
      if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read status register");
        return ret;
      }
      if (status & (1 << 6)) {  // Vérifier si Meas_m_done est défini
        break;
      }
      vTaskDelay(pdMS_TO_TICKS(1));  // Petite pause pour éviter un polling excessif
    }

    // Lire les registres X, Y, Z
    ret = i2c_read_register(XOUT0, &xout0, 1);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Failed to read XOUT0");
      return ret;
    }
    ret = i2c_read_register(XOUT1, &xout1, 1);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Failed to read XOUT1");
      return ret;
    }
    ret = i2c_read_register(XOUT2, &xout2, 1);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Failed to read XOUT2");
      return ret;
    }
    ret = i2c_read_register(YOUT0, &yout0, 1);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Failed to read YOUT0");
      return ret;
    }
    ret = i2c_read_register(YOUT1, &yout1, 1);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Failed to read YOUT1");
      return ret;
    }
    ret = i2c_read_register(YOUT2, &yout2, 1);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Failed to read YOUT2");
      return ret;
    }
    ret = i2c_read_register(ZOUT0, &zout0, 1);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Failed to read ZOUT0");
      return ret;
    }
    ret = i2c_read_register(ZOUT1, &zout1, 1);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Failed to read ZOUT1");
      return ret;
    }
    ret = i2c_read_register(ZOUT2, &zout2, 1);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Failed to read ZOUT2");
      return ret;
    }
  } else {
    i2c_read_register(XOUT0, &xout0, 1);
    i2c_read_register(XOUT1, &xout1, 1);
    i2c_read_register(XOUT2, &xout2, 1);
    i2c_read_register(YOUT0, &yout0, 1);
    i2c_read_register(YOUT1, &yout1, 1);
    i2c_read_register(YOUT2, &yout2, 1);
    i2c_read_register(ZOUT0, &zout0, 1);
    i2c_read_register(ZOUT1, &zout1, 1);
    i2c_read_register(ZOUT2, &zout2, 1);
  }

  // Combiner les valeurs brutes de X, Y, Z en utilisant les bits de poids faible
  x_raw = ((int32_t)xout0 << 12) | ((int32_t)xout1 << 4) | ((int32_t)(xout2 & 0x0F));
  y_raw = ((int32_t)yout0 << 12) | ((int32_t)yout1 << 4) | ((int32_t)(yout2 & 0x0F));
  z_raw = ((int32_t)zout0 << 12) | ((int32_t)zout1 << 4) | ((int32_t)(zout2 & 0x0F));

  // Centrer les valeurs autour de zéro
  x_raw -= 524288;
  y_raw -= 524288;
  z_raw -= 524288;

  // Convertir les valeurs en nanoTesla
  *x = x_raw * LSB_TO_NT;
  *y = y_raw * LSB_TO_NT;
  *z = z_raw * LSB_TO_NT;

  // Optionnel : Débogage
  // ESP_LOGI(TAG, "Raw Values - X: %ld, Y: %ld, Z: %ld", x_raw, y_raw, z_raw);
  // ESP_LOGI(TAG, "Scaled Values (nT) - X: %.2f, Y: %.2f, Z: %.2f", *x, *y, *z);

  return ESP_OK;
}

/* Read temperature from the sensor */
float mmc5603_read_temperature() {
  esp_err_t ret;
  uint8_t status;
  if (bit_Cmm_en) {
    return NAN;
  }  // Read temperature not available in continuous mode
  // Démarrer une mesure temperature si le mode continu n'est pas activé
  // Définir le bit 1 (Take_meas_T) et le bit 5 (Auto_SR_en) si activé
  uint8_t measurement_command = (bit_auto_sr_en ? (1 << 5) : 0) | (1 << 1);
  // Bit 5 : Auto_SR_en, Bit 1 : Take_meas_T
  ret = i2c_write_register(CTRL0_REG, measurement_command);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to start temperature measurement: %s", esp_err_to_name(ret));
    return NAN;
  }
  // Attendre que la mesure soit terminée (bit 6 de STATUS_REG)
  while (1) {
    ret = i2c_read_register(STATUS_REG, &status, 1);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Failed to read status register");
      return NAN;
    }
    if (status & (1 << 7)) {  // Vérifier si Meas_t_done est défini
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(1));  // Petite pause pour éviter un polling excessif
  }
  uint8_t temp_raw;
  // Read the raw temperature data
  if (i2c_read_register(TEMP_OUT, &temp_raw, 1) != ESP_OK) {
    ESP_LOGE(TAG, "Failed to read temperature");
    return NAN;
  }
  // Convert raw data to temperature in °C
  return temp_raw * 0.8f - 75.0f;
}

/* Perform a SET operation on the sensor */
esp_err_t mmc5603_do_set() {
  uint8_t measurement_command = (bit_auto_sr_en ? (1 << 5) : 0) | (1 << 3);
  esp_err_t ret = i2c_write_register(CTRL0_REG, measurement_command);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to perform SET operation: %s", esp_err_to_name(ret));
  }
  return ret;
}

/* Perform a RESET operation on the sensor */
esp_err_t mmc5603_do_reset() {
  uint8_t measurement_command = (bit_auto_sr_en ? (1 << 5) : 0) | (1 << 4);
  esp_err_t ret = i2c_write_register(CTRL0_REG, measurement_command);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to perform RESET operation: %s", esp_err_to_name(ret));
  }
  return ret;
}

/* Calculate and set the measurement offset for each axis */
esp_err_t mmc5603_get_offset(float *offset_x, float *offset_y, float *offset_z) {
  // Désactiver Auto_SR avant le calcul de l'offset
  if (mmc5603_set_auto_sr(false) != ESP_OK) {
    ESP_LOGE(TAG, "Failed to disable Auto_SR for offset calculation");
    return ESP_FAIL;
  }
  float x1, x2, y1, y2, z1, z2;
  // Effectuer le SET
  if (mmc5603_do_set() != ESP_OK) {
    ESP_LOGE(TAG, "Failed to perform SET operation for offset calculation");
    return ESP_FAIL;
  }
  vTaskDelay(pdMS_TO_TICKS(3));
  // Prendre 5 mesures à vide après le SET
  for (int i = 0; i < 5; i++) {
    if (mmc5603_read_single_measurement(&x1, &y1, &z1) != ESP_OK) {
      ESP_LOGE(TAG, "Failed to read dummy measurement after SET");
      return ESP_FAIL;
    }
  }
  // Garder la 6ème mesure pour le calcul
  if (mmc5603_read_single_measurement(&x1, &y1, &z1) != ESP_OK) {
    ESP_LOGE(TAG, "Failed to read measurement after SET");
    return ESP_FAIL;
  }
  // Effectuer le RESET
  if (mmc5603_do_reset() != ESP_OK) {
    ESP_LOGE(TAG, "Failed to perform RESET operation for offset calculation");
    return ESP_FAIL;
  }
  vTaskDelay(pdMS_TO_TICKS(3));
  // Prendre 5 mesures à vide après le RESET
  for (int i = 0; i < 5; i++) {
    if (mmc5603_read_single_measurement(&x2, &y2, &z2) != ESP_OK) {
      ESP_LOGE(TAG, "Failed to read dummy measurement after RESET");
      return ESP_FAIL;
    }
  }
  // Garder la 6ème mesure pour le calcul
  if (mmc5603_read_single_measurement(&x2, &y2, &z2) != ESP_OK) {
    ESP_LOGE(TAG, "Failed to read measurement after RESET");
    return ESP_FAIL;
  }
  // Calculer les offsets en moyenne des deux mesures finales
  *offset_x = (x1 + x2) / 2;
  *offset_y = (y1 + y2) / 2;
  *offset_z = (z1 + z2) / 2;
  ESP_LOGI(TAG, "Offsets calculated - X: %.2f, Y: %.2f, Z: %.2f", *offset_x, *offset_y, *offset_z);
  // Effectuer un dernier SET pour revenir à l'état initial
  if (mmc5603_do_set() != ESP_OK) {
    ESP_LOGE(TAG, "Failed to perform final SET operation");
    return ESP_FAIL;
  }
  vTaskDelay(pdMS_TO_TICKS(2));  // Petite pause entre chaque mesure
  return ESP_OK;
}

/* Initialize the MMC5603 sensor */
esp_err_t mmc5603_init() {
  uint8_t product_id;
  esp_err_t ret = i2c_read_register(PRODUCT_ID_REG, &product_id, 1);
  if (ret == ESP_OK) {
    if (product_id == EXPECTED_PRODUCT_ID) {
      ESP_LOGI(TAG, "Product ID verified: 0x%02X", product_id);
      ESP_LOGI(TAG, "MMC5603 sensor initialization complete");
      return ESP_OK;
    } else {
      ESP_LOGE(TAG, "Unexpected Product ID: 0x%02X (expected 0x%02X)", product_id, EXPECTED_PRODUCT_ID);
      return ESP_ERR_INVALID_RESPONSE;
    }
  } else {
    ESP_LOGE(TAG, "Failed to read Product ID: %s", esp_err_to_name(ret));
    return ret;
  }
}

/* Enable or disable the automatic self-test function.
 */
esp_err_t mmc5603_self_test() {
  uint8_t st_x, st_y, st_z;
  uint8_t threshold_x, threshold_y, threshold_z;
  // Lire les valeurs de self-test pour chaque axe
  if (i2c_read_register(SELF_TEST_X, &st_x, 1) != ESP_OK ||
      i2c_read_register(SELF_TEST_Y, &st_y, 1) != ESP_OK ||
      i2c_read_register(SELF_TEST_Z, &st_z, 1) != ESP_OK) {
    ESP_LOGE(TAG, "Failed to read self-test signal registers");
    return ESP_FAIL;
  }
  // Calculer les seuils de self-test (80% des valeurs lues)
  threshold_x = (uint8_t)(st_x * 0.8);
  threshold_y = (uint8_t)(st_y * 0.8);
  threshold_z = (uint8_t)(st_z * 0.8);
  // Écrire les seuils dans les registres 1EH, 1FH et 20H
  if (i2c_write_register(ST_X_TH, threshold_x) != ESP_OK ||
      i2c_write_register(ST_Y_TH, threshold_y) != ESP_OK ||
      i2c_write_register(ST_Z_TH, threshold_z) != ESP_OK) {
    ESP_LOGE(TAG, "Failed to write self-test thresholds");
    return ESP_FAIL;
  }
  // Activer le bit Auto_st_en (bit 6) pour lancer l'auto-test
  esp_err_t ret = i2c_write_register(CTRL0_REG, (1 << 6));  // Auto_st_en activé temporairement
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initiate automatic self-test: %s", esp_err_to_name(ret));
    return ret;
  }
  // Attendre que l'auto-test soit terminé
  vTaskDelay(pdMS_TO_TICKS(5));  // Ajuster si nécessaire
  // Lire le registre Status pour vérifier le résultat de l'auto-test
  uint8_t sat_sensor;
  ret = i2c_read_register(STATUS_REG, &sat_sensor, 1);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to read status register for self-test result: %s", esp_err_to_name(ret));
    return ret;
  }
  // Vérifier le bit Sat_sensor (bit 3) pour déterminer le succès de l'auto-test
  if ((sat_sensor & (1 << 3)) == 0) {
    ESP_LOGI(TAG, "Self-test passed");
    return ESP_OK;
  } else {
    ESP_LOGE(TAG, "Self-test failed");
    return ESP_FAIL;
  }
}

/* Reset the sensor */
esp_err_t mmc5603_reset() {
  esp_err_t ret = i2c_write_register(CTRL1_REG, (1 << 7));
  if (ret == ESP_OK) {
    ESP_LOGI(TAG, "Sensor successfully reset");
  } else {
    ESP_LOGE(TAG, "Failed to reset sensor: %s", esp_err_to_name(ret));
  }
  return ret;
}

/* Enable or disable the Auto_SR_en function for automatic set/reset */
esp_err_t mmc5603_set_auto_sr(bool enable) {
  uint8_t data = enable ? (1 << 5) : 0x00;
  esp_err_t ret = i2c_write_register(CTRL0_REG, data);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to update Internal Control 0 register: %s", esp_err_to_name(ret));
  } else {
    ESP_LOGI(TAG, "Auto_SR_en has been %s successfully", enable ? "enabled" : "disabled");
    bit_auto_sr_en = enable;
  }
  return ret;
}

/* Set the output data rate (ODR) of the sensor */
esp_err_t mmc5603_set_odr(uint8_t frequency_hz) {
  if (frequency_hz == 0) {
    ESP_LOGE(TAG, "ODR frequency cannot be 0. Choose a value between 1 and 255 Hz.");
    return ESP_ERR_INVALID_ARG;
  }
  esp_err_t ret = i2c_write_register(ODR_REG, frequency_hz);
  if (ret == ESP_OK) {
    ESP_LOGI(TAG, "ODR set to %d Hz%s", frequency_hz, (frequency_hz == 255 && bit_hpower) ? " (1000 Hz with hpower)" : "");
  } else {
    ESP_LOGE(TAG, "Failed to update ODR register: %s", esp_err_to_name(ret));
  }
  return ret;
}

/* Enable or disable the high power (hpower) bit */
esp_err_t mmc5603_set_hpower(bool enable) {
  uint8_t data = (Prd_set & 0x07) |                 // Bits 0-2 : Prd_set (période pour set automatique)
                 (bit_En_prd_set ? (1 << 3) : 0) |  // Bit 3 : En_prd_set (set périodique activé ou non)
                 (bit_Cmm_en ? (1 << 4) : 0) |      // Bit 4 : Cmm_en (mode continu activé ou non)
                 (enable ? (1 << 7) : 0);           // Bit 7 : hpower (activer ou désactiver haute puissance)

  esp_err_t ret = i2c_write_register(CTRL2_REG, data);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to update CTRL2 register: %s", esp_err_to_name(ret));
  } else {
    ESP_LOGI(TAG, "hpower set to %d", enable ? 1 : 0);
  }
  return ret;
}

/* Disable continuous measurement mode */
esp_err_t mmc5603_disable_continuous_mode() {
  bit_Cmm_en = false;
  uint8_t data = (Prd_set & 0x07) |                 // Bits 0-2 : Prd_set (période pour set automatique)
                 (bit_En_prd_set ? (1 << 3) : 0) |  // Bit 3 : En_prd_set (set périodique activé ou non)
                 (bit_Cmm_en ? (1 << 4) : 0) |      // Bit 4 : Cmm_en (mode continu activé ou non)
                 (bit_hpower ? (1 << 7) : 0);       // Bit 7 : hpower (activer ou désactiver haute puissance)
  if (i2c_write_register(CTRL2_REG, data) != ESP_OK) {
    ESP_LOGE(TAG, "Failed to disable continuous mode in CTRL2 register");
    return ESP_FAIL;
  }
  ESP_LOGI(TAG, "Continuous mode disabled successfully");
  return ESP_OK;
}

/* Enable continuous measurement mode, si 0 on passe à 1000 Hz */
esp_err_t mmc5603_enable_continuous_mode(uint8_t acquisition_frequency_hz) {
  esp_err_t ret;
  // Si fréquence d'acquisition est 0, on active le mode 1000 Hz avec hpower
  if (acquisition_frequency_hz == 0) {
    acquisition_frequency_hz = 255;  // Valeur de 255 pour indiquer 1000 Hz
    bit_hpower = true;               // Active le mode haute puissance
    ESP_LOGI(TAG, "Setting ODR to 1000 Hz with hpower enabled");
  } else {
    bit_hpower = false;  // Désactiver hpower pour les autres fréquences
    ESP_LOGI(TAG, "Setting acquisition frequency to %d Hz", acquisition_frequency_hz);
  }
  // Écrire la fréquence d'acquisition (ODR) dans le registre ODR_REG
  ret = i2c_write_register(ODR_REG, acquisition_frequency_hz);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to write acquisition frequency to ODR register");
    return ret;
  }
  // Commander le calcul de la période de mesure du mode ODR
  uint8_t measurement_command = (bit_auto_sr_en ? (1 << 5) : 0) | (1 << 7);  // Préserve bit_auto_sr_en et active Cmm_freq_en
  ret = i2c_write_register(CTRL0_REG, measurement_command);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to enable Cmm_freq_en: %s", esp_err_to_name(ret));
    return ret;
  }
  vTaskDelay(pdMS_TO_TICKS(1));  // Petite pause
  // Activer le mode continu avec les configurations finales
  bit_Cmm_en = true;
  uint8_t data = (Prd_set & 0x07) |                 // Bits 0-2 : Prd_set (période pour set automatique)
                 (bit_En_prd_set ? (1 << 3) : 0) |  // Bit 3 : En_prd_set (set périodique activé ou non)
                 (bit_Cmm_en ? (1 << 4) : 0) |      // Bit 4 : Cmm_en (mode continu activé)
                 (bit_hpower ? (1 << 7) : 0);       // Bit 7 : hpower (activer ou désactiver haute puissance)
  // Écrire la configuration finale dans CTRL2_REG
  ret = i2c_write_register(CTRL2_REG, data);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to enable continuous mode in CTRL2 register");
    return ret;
  }
  ESP_LOGI(TAG, "Continuous mode enabled with ODR = %d Hz", acquisition_frequency_hz == 255 ? 1000 : acquisition_frequency_hz);
  return ESP_OK;
}

/* Disable the periodic set function. */
esp_err_t mmc5603_disable_periodic_set() {
  // Mettre à jour la variable statique pour indiquer que le set périodique est désactivé
  bit_En_prd_set = false;
  // Construire la valeur à écrire dans CTRL2_REG en conservant les autres paramètres non self-clearing
  uint8_t data = (Prd_set & 0x07) |                 // Bits 0-2 : Prd_set (période pour set automatique)
                 (bit_En_prd_set ? (1 << 3) : 0) |  // Bit 3 : En_prd_set, désactivé
                 (bit_Cmm_en ? (1 << 4) : 0) |      // Bit 4 : Cmm_en, si le mode continu est activé
                 (bit_hpower ? (1 << 7) : 0);       // Bit 7 : hpower, si la haute puissance est activée

  // Écrire la configuration dans CTRL2_REG
  esp_err_t ret = i2c_write_register(CTRL2_REG, data);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to disable periodic set in CTRL2 register: %s", esp_err_to_name(ret));
    return ret;
  }
  ESP_LOGI(TAG, "Periodic set function disabled successfully.");
  return ESP_OK;
}

/* Set periodic set function with specified prd_set value */
esp_err_t mmc5603_set_periodic_set(uint8_t prd_set_value) {
  // Limiter la valeur de prd_set_value à la plage 0-7 (3 bits)
  if (prd_set_value > 7) {
    ESP_LOGE(TAG, "Invalid prd_set_value: must be between 0 and 7");
    return ESP_ERR_INVALID_ARG;
  }
  if (!bit_auto_sr_en) {
    ESP_LOGE(TAG, "Auto SR must be activated before enable periodic set");
    return ESP_ERR_INVALID_STATE;
  }
  if (!bit_Cmm_en) {
    ESP_LOGE(TAG, "Continuous mode must be activated before enable periodic set");
    return ESP_ERR_INVALID_STATE;
  }
  // Mettre à jour les variables statiques pour conserver l'état des bits non self-clearing
  Prd_set = prd_set_value;  // Bits 0-2 pour la valeur de prd_set
  bit_En_prd_set = true;    // Activer En_prd_set si prd_set_value est non nul
  // Construire la valeur à écrire dans CTRL2_REG
  uint8_t data = (Prd_set & 0x07) |            // Bits 0-2 : prd_set (période pour set automatique)
                 (1 << 3) |                    // Bit 3 : En_prd_set set périodique en cours d'activation
                 (1 << 4) |                    // Bit 4 : Cmm_en (mode continu déjà activé)
                 (bit_hpower ? (1 << 7) : 0);  // Bit 7 : hpower (activer ou désactiver haute puissance)
  // Écrire la configuration finale dans CTRL2_REG
  esp_err_t ret = i2c_write_register(CTRL2_REG, data);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to write CTRL2 register: %s", esp_err_to_name(ret));
    return ret;
  } else {
    ESP_LOGI(TAG, "Prd_set configured to execute SET after every %s samples.",
             (prd_set_value == 0) ? "1" : (prd_set_value == 1) ? "25"
                                      : (prd_set_value == 2)   ? "75"
                                      : (prd_set_value == 3)   ? "100"
                                      : (prd_set_value == 4)   ? "250"
                                      : (prd_set_value == 5)   ? "500"
                                      : (prd_set_value == 6)   ? "1000"
                                                               : "2000");
  }
  return ESP_OK;
}

/* Perform a permanent SET operation by enabling St_enp with preserved BW value */
esp_err_t mmc5603_set_permanent_set_mode(bool enable) {
  // Construire la valeur à écrire en gardant les bits non self-clearing
  uint8_t data = (bw & 0x03) |                      // Bits 0-1 : BW (largeur de bande)
                 (enable ? (1 << 5) : 0) |          // Bit 5 : St_enp (SET permanent activé ou désactivé)
                 (bit_En_prd_set ? (1 << 3) : 0) |  // Bit 3 : En_prd_set, si activé
                 (bit_Cmm_en ? (1 << 4) : 0);       // Bit 4 : Cmm_en, si le mode continu est activé
  // Écrire la configuration dans CTRL1_REG
  esp_err_t ret = i2c_write_register(CTRL1_REG, data);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to update CTRL1 register for permanent SET: %s", esp_err_to_name(ret));
    return ret;
  }
  ESP_LOGI(TAG, "Permanent SET %s successfully", enable ? "enabled" : "disabled");
  return ESP_OK;
}

/* Perform a permanent RESET operation by enabling St_enm with preserved BW value */
esp_err_t mmc5603_set_permanent_reset_mode(bool enable) {
  // Construire la valeur à écrire en gardant les bits non self-clearing
  uint8_t data = (bw & 0x03) |                      // Bits 0-1 : BW (largeur de bande)
                 (enable ? (1 << 6) : 0) |          // Bit 5 : St_enp (SET permanent activé ou désactivé)
                 (bit_En_prd_set ? (1 << 3) : 0) |  // Bit 3 : En_prd_set, si activé
                 (bit_Cmm_en ? (1 << 4) : 0);       // Bit 4 : Cmm_en, si le mode continu est activé
  // Écrire la configuration dans CTRL1_REG
  esp_err_t ret = i2c_write_register(CTRL1_REG, data);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to update CTRL1 register for permanent SET: %s", esp_err_to_name(ret));
    return ret;
  }
  ESP_LOGI(TAG, "Permanent RESET %s successfully", enable ? "enabled" : "disabled");
  return ESP_OK;
}

/* Set the bandwidth (BW) for acquisition duration while preserving other settings in CTRL1 register. */
esp_err_t mmc5603_set_bandwidth(uint8_t bw_value) {
  if (bw_value > 3) {
    ESP_LOGE(TAG, "Invalid BW value. Must be between 0 and 3.");
    return ESP_ERR_INVALID_ARG;
  }
  // Update static variable to preserve BW setting
  bw = bw_value & 0x03;
  // Construct the CTRL1 register value with preserved settings
  uint8_t ctrl1_value = (bw & 0x03) |                  // Bits 0-1 : BW (bandwidth setting)
                        (bit_St_enp ? (1 << 5) : 0) |  // Bit 5 : St_enp, if permanent SET is enabled
                        (bit_St_enm ? (1 << 6) : 0);   // Bit 6 : St_enm, if permanent RESET is enabled
  // Write the updated value to CTRL1 register
  esp_err_t ret = i2c_write_register(CTRL1_REG, ctrl1_value);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set BW in CTRL1 register: %s", esp_err_to_name(ret));
    return ret;
  }
  ESP_LOGI(TAG, "Bandwidth set to %s ms measurement time.",
           (bw_value == 0)   ? "6.6"
           : (bw_value == 1) ? "3.5"
           : (bw_value == 2) ? "2.0"
                             : "1.2");
  return ESP_OK;
}

// Fonction pour lire et afficher les informations détaillées des registres de configuration
void mmc5603_get_informations() {
  uint8_t data;
  // Lecture et affichage du registre Status1
  if (i2c_read_register(STATUS_REG, &data, 1) == ESP_OK) {
    printf("=== Registre Status1 ===\n");
    printf("Registre Status1 (0x%02X): 0x%02X\n", STATUS_REG, data);
    printf("  Meas_t_done: %d - %s\n", (data >> 7) & 0x01, ((data >> 7) & 0x01) ? "Mesure de température prête" : "Pas de mesure de température prête");
    printf("  Meas_m_done: %d - %s\n", (data >> 6) & 0x01, ((data >> 6) & 0x01) ? "Mesure de champ magnétique prête" : "Pas de mesure de champ magnétique prête");
    printf("  Sat_sensor: %d - %s\n", (data >> 5) & 0x01, ((data >> 5) & 0x01) ? "Échec du self-test (saturation)" : "Self-test réussi");
    printf("  OTP_read_done: %d - %s\n", (data >> 4) & 0x01, ((data >> 4) & 0x01) ? "Lecture OTP terminée OK" : "Lecture OTP non terminée erreur; à 1 si OK");
    printf("  ST_Fail: %d - %s\n", (data >> 3) & 0x01, ((data >> 3) & 0x01) ? "Échec du self-test" : "Self-test réussi ; doit être à 0");
    printf("  Mdt_flag_int: %d - %s\n", (data >> 2) & 0x01, ((data >> 2) & 0x01) ? "Flag MDT activé" : "Flag MDT désactivé ; doit être à 0");
    printf("  Meas_t_done_int: %d - %s\n", (data >> 1) & 0x01, ((data >> 1) & 0x01) ? "Interruption mesure température activée" : "Interruption mesure température désactivée ; doit être à 0");
    printf("  Meas_m_done_int: %d - %s\n", data & 0x01, (data & 0x01) ? "Interruption mesure magnétique activée" : "Interruption mesure magnétique désactivée ; doit être à 0");
  } else {
    printf("Erreur lors de la lecture du registre Status1.\n");
  }

  // Affichage des informations des bits de configuration
  printf("\n=== Informations de configuration ===\n");
  printf("  Auto_SR_en: %d - %s\n", bit_auto_sr_en, bit_auto_sr_en ? "Auto set/reset activé" : "Auto set/reset désactivé");
  printf("  En_prd_set: %d - %s\n", bit_En_prd_set, bit_En_prd_set ? "Set périodique activé" : "Set périodique désactivé");
  printf("  Cmm_en: %d - %s\n", bit_Cmm_en, bit_Cmm_en ? "Mode continu activé" : "Mode continu désactivé");
  printf("  Hpower: %d - %s\n", bit_hpower, bit_hpower ? "Haute puissance activée" : "Haute puissance désactivée");
  printf("  St_enp (SET permanent): %d - %s\n", bit_St_enp, bit_St_enp ? "SET permanent activé" : "SET permanent désactivé");
  printf("  St_enm (RESET permanent): %d - %s\n", bit_St_enm, bit_St_enm ? "RESET permanent activé" : "RESET permanent désactivé");

  // Affichage des informations de configuration de la bande passante
  printf("  BW (Bande passante): %d - Temps de mesure: %s ms\n", bw,
         (bw == 0)   ? "6.6"
         : (bw == 1) ? "3.5"
         : (bw == 2) ? "2.0"
                     : "1.2");

  // Affichage des informations de configuration de la fréquence de set périodique
  printf("  Prd_set: %d - Fréquence de SET périodique après tous les %s échantillons\n", Prd_set,
         (Prd_set == 0)   ? "1"
         : (Prd_set == 1) ? "25"
         : (Prd_set == 2) ? "75"
         : (Prd_set == 3) ? "100"
         : (Prd_set == 4) ? "250"
         : (Prd_set == 5) ? "500"
         : (Prd_set == 6) ? "1000"
                          : "2000");
}