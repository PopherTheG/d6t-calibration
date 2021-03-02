#include <stdio.h>
#include <time.h>
#include <sys/time.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "driver/i2c.h"
#include "esp_sntp.h"
#include "esp_attr.h"
#include "esp_event.h"

#include "ssd1306.h"
#include "d6t44l.h"
#include "vl53l3cx.h"

#define TAG "main-app"
#define I2C_MASTER_PORT I2C_NUM_0
#define I2C_SDA 27
#define I2C_SCL 14

typedef enum
{
  STATE_READY,
  STATE_SCAN,
  STATE_ERROR,
  STATE_AUTH_SUCCESS,
  STATE_AUTH_FAIL,
  STATE_TEMP_RESULT,
} state_t;

static state_t state = STATE_ERROR;

static void vl53l3cx_event_handler(vl53l3cx_event_t *evt)
{
    switch (evt->id)
    {
    case TOF_EVT_THRESHOLD_INSIDE:
        // state = STATE_SCAN;
        // gpio_set_level(LED_BLUE, 1);
        // scanner_app_trigger();
        break;

    case TOF_EVT_THRESHOLD_OUTSIDE:
        D6T44L_reset();
        // scanner_app_sleep();
        // state = STATE_READY;
        break;

    default:
        // ESP_LOGE(TAG, "Unknow TOF event.");
        break;
    }
}

static void d6t44lc_event_handler(d6t44l_event_t *evt)
{
  int i;
  switch (evt->id)
  {
  case TEMP_EVT_DATA_READY:
    // gpio_set_level(LED_BLUE, 0);
    D6T44L_update_temp();
    // telemetry_notify_log(device_type);
    // double temp = 0.0;
    // D6T44l_get_data(&temp);
    // ESP_LOGI(TAG, "Temperature: %04.1f", temp);
    break;

  case TEMP_EVT_ERROR:
    state = STATE_ERROR;
    break;

  case TEMP_EVT_TEMP_PASS:
    // gpio_set_level(LED_GREEN, 1);
    // gpio_set_level(BUZZER_IO, 1);
    // vTaskDelay(250 / portTICK_RATE_MS);
    // gpio_set_level(BUZZER_IO, 0);
    break;
  case TEMP_EVT_RESET:
    // gpio_set_level(LED_BLUE, 1);
    // gpio_set_level(LED_GREEN, 0);
    // gpio_set_level(LED_RED, 0);
    state = STATE_READY;
    break;

  case TEMP_EVT_TEMP_FAIL:
    // gpio_set_level(LED_RED, 1);
    // gpio_set_level(BUZZER_IO, 1);
    // vTaskDelay(250 / portTICK_RATE_MS);
    // gpio_set_level(BUZZER_IO, 0);
    // vTaskDelay(250 / portTICK_RATE_MS);
    // gpio_set_level(BUZZER_IO, 1);
    // vTaskDelay(250 / portTICK_RATE_MS);
    // gpio_set_level(BUZZER_IO, 0);
    // vTaskDelay(250 / portTICK_RATE_MS);
    // gpio_set_level(BUZZER_IO, 1);
    // vTaskDelay(250 / portTICK_RATE_MS);
    // gpio_set_level(BUZZER_IO, 0);
    // vTaskDelay(250 / portTICK_RATE_MS);
    // gpio_set_level(BUZZER_IO, 1);
    // vTaskDelay(250 / portTICK_RATE_MS);
    // gpio_set_level(BUZZER_IO, 0);
    break;

  default:
    break;
  }
}

static esp_err_t i2c_master_driver_initialize(void)
{
  i2c_config_t i2c_master_config = {
      .mode = I2C_MODE_MASTER,
      .sda_io_num = I2C_SDA,
      .scl_io_num = I2C_SCL,
      .sda_pullup_en = GPIO_PULLUP_ENABLE,
      .scl_pullup_en = GPIO_PULLUP_ENABLE,
      .master.clk_speed = 400000};

  return i2c_param_config(I2C_MASTER_PORT, &i2c_master_config);
}

static uint8_t i2c_slave_knock(uint8_t i2c_port, uint8_t slave_addr)
{
  esp_err_t err;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (slave_addr << 1) | I2C_MASTER_WRITE, 1);
  i2c_master_stop(cmd);
  err = i2c_master_cmd_begin(i2c_port, cmd, 50 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);
  return err == ESP_OK;
}

static void i2c_scan(void *pdata)
{
  while (1)
  {
    ESP_LOGI(TAG, "Scanning I2C.");
    i2c_driver_install(I2C_MASTER_PORT, I2C_MODE_MASTER, 0, 0, 0);
    i2c_master_driver_initialize();

    uint8_t slave_count = 0;
    uint8_t address;

    printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\r\n");
    for (int i = 0; i < 128; i += 16)
    {
      printf("%02x: ", i);
      for (int j = 0; j < 16; j++)
      {
        address = i + j;
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, 1);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_PORT, cmd, 50 / portTICK_RATE_MS);
        i2c_cmd_link_delete(cmd);
        if (ret == ESP_OK)
        {
          printf("%02x ", address);
          slave_count++;
        }
        else if (ret == ESP_ERR_TIMEOUT)
        {
          printf("UU ");
        }
        else
        {
          printf("-- ");
        }
      }
      printf("\r\n");
    }

    ESP_LOGI(TAG, "Slave Count %d", slave_count);
    i2c_driver_delete(I2C_MASTER_PORT);
    vTaskDelay(3000 / portTICK_RATE_MS);
  }

#if 0
    for (int slave_addr = 0; slave_addr < 127; slave_addr++)
    {
        if (i2c_slave_knock(I2C_MASTER_PORT, slave_addr))
        {
            // Count the number of slaves
            ESP_LOGI(TAG, "Slave_%d_addr %02X", slave_count, slave_addr);
            // Increment count
            slave_count++;
        }
    }
#endif
}

static void system_init(void)
{
  ESP_LOGI(TAG, "System Init");
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
  {
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }

  ESP_ERROR_CHECK(err);
  // ESP_ERROR_CHECK(nvs_flash_init_partition("cfg_part"));

  setenv("TZ", "PST-8", 1);
  tzset();

  i2c_config_t i2c_master_config = {
      .mode = I2C_MODE_MASTER,
      .sda_io_num = I2C_SDA,
      .scl_io_num = I2C_SCL,
      .sda_pullup_en = GPIO_PULLUP_ENABLE,
      .scl_pullup_en = GPIO_PULLUP_ENABLE,
      .master = {
          .clk_speed = 100000,
      }};

  i2c_param_config(I2C_MASTER_PORT, &i2c_master_config);

  err = i2c_driver_install(I2C_MASTER_PORT, I2C_MODE_MASTER, 0, 0, 0);
}

void app_main()
{
  ESP_LOGI(TAG, "Start!");

  system_init();

  // io_init();
  // i2c_scan();

  // esp_efuse_mac_get_default(chipId);
  // sprintf(serial, "%d%d%d%d%d%d", chipId[0], chipId[1], chipId[2], chipId[3], chipId[4], chipId[5]);
  // ESP_LOGI(TAG, "%s", serial);

  // ESP_LOGI(TAG, "Long serial: %lld", strtoll(serial, NULL, 0));
  // initialise_wifi(smart_wifi_cb);

  // telemetry_init();
  if (D6T44L_init(d6t44lc_event_handler) == 1)
  {
    D6T44L_app_run();
  }

#if 0
    char bluetooth_name[23] = {0};
    sprintf(bluetooth_name, "Xeleqt_%.*s", 6, serial);
    // printf("%s", bluetooth_name);
    bluetooth_register_gatt_handler(gatts_event_handler);
    bluetooth_init(bluetooth_name);

    if (scanner_app_init(scanner_event_handler) == SCANNER_STATUS_OK)
    {
        scanner_app_start();
    }
    else
    {
        ESP_LOGE(TAG, "Unable to start scanner!");
    }

    ssd1306_init();

    if (D6T44L_init(d6t44lc_event_handler) == 1)
    {
        D6T44L_app_run();
    }
#endif

  if (init_vl53l3cx(vl53l3cx_event_handler) == VL53LX_ERROR_NONE)
  {
      vl53l3cx_start_app();
  }

  // xTaskCreate(system_info_task, "sys-info", 2048, NULL, 1, NULL);
  // xTaskCreate(i2c_scan, "i2c-scan", 1024 * 2, NULL, 5, NULL);

  // uint64_t id = strtoull(serial, NULL, 0);
  // telemetry_start(&id);
}