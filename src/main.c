/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <math.h>
#include <stdio.h>

#include "driver/ledc.h"
#include "esp_chip_info.h"
#include "esp_err.h"
#include "esp_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

// H-Bride Configuration
#define H_BRIDGE_IN_LEFT GPIO_NUM_1
#define H_BRIDGE_IN_RIGHT GPIO_NUM_42
#define H_BRIDGE_OUT_LEFT GPIO_NUM_41
#define H_BRIDGE_OUT_RIGHT GPIO_NUM_2

#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEFT_LEDC_CHANNEL LEDC_CHANNEL_0
#define RIGHT_LEDC_CHANNEL LEDC_CHANNEL_1
#define LEDC_DUTY_RES LEDC_TIMER_10_BIT
#define LEDC_DUTY \
  (pow(2, LEDC_DUTY_RES) * 0.8)  // Set duty percent. (2 ** 13) * %
#define LEDC_FREQUENCY (20000)   // Set frequency to 20 kHz

static void print_chip_info(void) {
  /* Print chip information */
  esp_chip_info_t chip_info;
  uint32_t flash_size;
  esp_chip_info(&chip_info);
  printf("This is %s chip with %d CPU core(s), WiFi%s%s, ", CONFIG_IDF_TARGET,
         chip_info.cores, (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
         (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

  unsigned major_rev = chip_info.revision / 100;
  unsigned minor_rev = chip_info.revision % 100;
  printf("silicon revision v%d.%d, ", major_rev, minor_rev);
  if (esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
    printf("Get flash size failed");
    return;
  }

  printf(
      "%uMB %s flash\n", flash_size / (1024 * 1024),
      (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

  printf("Minimum free heap size: %d bytes\n",
         esp_get_minimum_free_heap_size());
}

static void ledc_init(void) {
  // Prepare and then apply the LEDC PWM timer configuration
  ledc_timer_config_t ledc_timer = {.speed_mode = LEDC_MODE,
                                    .duty_resolution = LEDC_DUTY_RES,
                                    .timer_num = LEDC_TIMER,
                                    .freq_hz = LEDC_FREQUENCY,
                                    .clk_cfg = LEDC_AUTO_CLK};
  ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

  // Prepare and then apply the LEDC PWM channel configurations
  ledc_channel_config_t left_ledc_channel = {.speed_mode = LEDC_MODE,
                                             .channel = LEFT_LEDC_CHANNEL,
                                             .timer_sel = LEDC_TIMER,
                                             .intr_type = LEDC_INTR_DISABLE,
                                             .gpio_num = H_BRIDGE_OUT_LEFT,
                                             .duty = 0,
                                             .hpoint = 0};
  ledc_channel_config_t right_ledc_channel = {.speed_mode = LEDC_MODE,
                                              .channel = RIGHT_LEDC_CHANNEL,
                                              .timer_sel = LEDC_TIMER,
                                              .intr_type = LEDC_INTR_DISABLE,
                                              .gpio_num = H_BRIDGE_OUT_RIGHT,
                                              .duty = 0,
                                              .hpoint = 0};
  ESP_ERROR_CHECK(ledc_channel_config(&left_ledc_channel));
  ESP_ERROR_CHECK(ledc_channel_config(&right_ledc_channel));
}

void turn_off_bridge(void) {
  // First, turn off both directions
  printf("Turning off both directions\n");
  ESP_ERROR_CHECK(gpio_set_level(H_BRIDGE_IN_LEFT, 0));
  ESP_ERROR_CHECK(gpio_set_level(H_BRIDGE_IN_RIGHT, 0));

  // Then, turn off the PWM signals
  printf("Turning off the PWM signals\n");
  ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEFT_LEDC_CHANNEL, 0));
  ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, RIGHT_LEDC_CHANNEL, 0));

  ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEFT_LEDC_CHANNEL));
  ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, RIGHT_LEDC_CHANNEL));
}

enum HBridgeDirection { LEFT, RIGHT };

void set_direction(enum HBridgeDirection direction) {
  // Turn off the bridge
  turn_off_bridge();

  // Wait for the PWM signals to turn off
  vTaskDelay(2 / portTICK_PERIOD_MS);

  // Finally, turn on the desired direction
  printf("Turning on the desired direction\n");
  switch (direction) {
    case LEFT:
      ESP_ERROR_CHECK(gpio_set_level(H_BRIDGE_IN_RIGHT, 1));
      ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEFT_LEDC_CHANNEL, LEDC_DUTY));
      ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEFT_LEDC_CHANNEL));
      printf("Right in on, ledc left out on\n");
      break;
    case RIGHT:
      ESP_ERROR_CHECK(gpio_set_level(H_BRIDGE_IN_LEFT, 1));
      ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, RIGHT_LEDC_CHANNEL, LEDC_DUTY));
      ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, RIGHT_LEDC_CHANNEL));
      printf("Left in on, ledc right out on\n");
      break;
  }
}

void app_main(void) {
  print_chip_info();
  ledc_init();

  int remaining = 5;
  enum HBridgeDirection direction = LEFT;

  // Configure H-Bridge pins
  ESP_ERROR_CHECK(gpio_set_direction(H_BRIDGE_IN_LEFT, GPIO_MODE_OUTPUT));
  ESP_ERROR_CHECK(gpio_set_direction(H_BRIDGE_IN_RIGHT, GPIO_MODE_OUTPUT));

  // Set initial direction
  set_direction(direction);

  // Toggle direction every 10 seconds
  for (;;) {
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    remaining--;
    printf("Switching to %s in %d seconds\n",
           direction == LEFT ? "left" : "right", remaining);

    if (remaining == 0) {
      remaining = 5;
      direction = direction == LEFT ? RIGHT : LEFT;

      set_direction(direction);
    }

    // Only keep the magnet on for 1 second
    if (remaining == 3) {
      printf("Turning off the bridge\n");
      turn_off_bridge();
    }
  }
}