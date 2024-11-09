/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <inttypes.h>
#include <stdio.h>

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mmc5603.h"

void app_main(void) {
  esp_err_t ret;
  ret = i2c_master_init();
  if (ret != ESP_OK) {
    ESP_LOGE("I2C", "Échec de l'initialisation du port I2C: %s", esp_err_to_name(ret));
  } else {
    ESP_LOGI("I2C", "I2C initialisé avec succès");
  }

  ret = mmc5603_init();
  mmc5603_reset();
  mmc5603_self_test();
  float x,
      y, z;
  float temp;

  mmc5603_set_bandwidth(3);
  mmc5603_disable_continuous_mode();

  mmc5603_get_informations();
  mmc5603_do_set();
  mmc5603_read_single_measurement(&x, &y, &z);
  printf("X, Y et Z : %f %f %f\n", x, y, z);
  mmc5603_read_single_measurement(&x, &y, &z);
  printf("X, Y et Z : %f %f %f\n", x, y, z);
  mmc5603_read_single_measurement(&x, &y, &z);
  printf("X, Y et Z : %f %f %f\n", x, y, z);
  mmc5603_read_single_measurement(&x, &y, &z);
  printf("X, Y et Z : %f %f %f\n", x, y, z);
  mmc5603_read_single_measurement(&x, &y, &z);
  printf("X, Y et Z : %f %f %f\n", x, y, z);
  mmc5603_read_single_measurement(&x, &y, &z);
  printf("X, Y et Z : %f %f %f\n", x, y, z);
  float temperature = mmc5603_read_temperature();
  printf("temperature : %f\n", temperature);

  mmc5603_set_auto_sr(true);
  mmc5603_enable_continuous_mode(0);
  mmc5603_set_periodic_set(6);
  int64_t start_time = esp_timer_get_time();
  int count = 0;
  while (count < 100) {
    mmc5603_read_single_measurement(&x, &y, &z);
    printf("X, Y et Z : %f %f %f\n", x, y, z);
    count++;
  }
  int64_t end_time = esp_timer_get_time();
  printf("Temps total pour 100 acquisitions : %lld ms\n", (end_time - start_time) / 1000);

  mmc5603_do_reset();
  mmc5603_disable_periodic_set();
  start_time = esp_timer_get_time();
  count = 0;
  while (count < 100) {
    mmc5603_read_single_measurement(&x, &y, &z);
    printf("X, Y et Z : %f %f %f\n", x, y, z);
    count++;
  }
  end_time = esp_timer_get_time();
  printf("Temps total pour 100 acquisitions : %lld ms\n", (end_time - start_time) / 1000);

  mmc5603_disable_continuous_mode();
  temperature = mmc5603_read_temperature();
  printf("temperature : %f\n", temperature);
}
