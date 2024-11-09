#ifndef INITI2C_H  // Vérifie si TACTILE_H n'est pas défini
#define INITI2C_H  // Définit TACTILE_H
/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////// DEFINITION DES LIBRAIRIES
/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////// DEFINITION DES CONSTANTES
/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
#define I2C_SDA_PIN GPIO_NUM_46
#define I2C_SCL_PIN GPIO_NUM_9         // Pin SCL (à adapter)
#define I2C_PORT I2C_NUM_0             // Le numéro du port I2C
#define I2C_MASTER_FREQ_HZ 200000      // 400kHz max
#define I2C_MASTER_TX_BUF_DISABLE 128  // Taille du tampon TX
#define I2C_MASTER_RX_BUF_DISABLE 128  // Taille du tampon RX
#define I2C_MASTER_TIMEOUT_MS 1000     // Timeout pour les opérations I2C

/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////// PROTOCOLE DES FONCTIONS
/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
esp_err_t i2c_master_init(void);

#endif  // Fin de la vérification de TACTILE_H