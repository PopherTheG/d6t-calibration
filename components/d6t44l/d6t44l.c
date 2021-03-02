#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>

#include "esp_log.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "d6t44l.h"
#include "ssd1306.h"

#define TAG "d6t44l-app"
#define DEBUG
// #define CALIBRATE

#define MUTEX_TIMEOUT_MS 50

#define TOF_MUTEX_LOCK() xSemaphoreTake(tof_mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS))
#define TOF_MUTEX_UNLOCK() xSemaphoreGive(tof_mutex)

// #define Txl_1 32.3 //DC1 = Txl_1
// #define Txh 38.4
// double data_1[16] =
//     {31.72, 31.63, 31.62, 31.61,
//      31.51, 31.54, 31.61, 31.52,
//      31.54, 31.75, 31.74, 31.72,
//      32.51, 32.57, 32.51, 31.72};
// double data_2[16] =
//     {38.24, 38.13, 38.12, 38.21,
//      38.01, 38.02, 38.11, 38.04,
//      38.04, 38.06, 38.24, 37.33,
//      38.13, 37.91, 38.01, 38.32};
// #define Txl_3 32.7
// #define PTAT_1 25.5
// #define PTAT_3 35.3
// double data_3[16] =
//     {31.42, 31.43, 31.32, 31.31,
//      31.21, 31.24, 31.41, 31.32,
//      31.34, 31.45, 31.24, 31.32,
//      31.51, 31.47, 31.41, 31.32};
// double data_calib_3[16] = {0};

/* Calibration at distance of 150mm */
// #define Txl_1 32 //DC1 = Txl_1
// #define Txh 38
// double data_1[16] =
//     {30.33, 30.33, 30.33, 30.33,
//      30.33, 30.33, 30.33, 30.33,
//      30.33, 30.33, 30.33, 30.33,
//      30.33, 30.33, 30.33, 30.33};
// double data_2[16] =
//     {36.38, 36.38, 36.38, 36.38,
//      36.38, 36.38, 36.38, 36.38,
//      36.38, 36.38, 36.38, 36.38,
//      36.38, 36.38, 36.38, 36.38};
// #define Txl_3 32
// #define PTAT_1 19.5
// #define PTAT_3 28.4
// double data_3[16] =
//     {30.57, 30.57, 30.57, 30.57,
//      30.57, 30.57, 30.57, 30.57,
//      30.57, 30.57, 30.57, 30.57,
//      30.57, 30.57, 30.57, 30.57};
// double data_calib_3[16] = {0};

/* Calibration at distance of 200mm */
// #define Txl_1 32 //DC1 = Txl_1
// #define Txh 38
// double data_1[16] =
//     {29.44, 29.44, 29.44, 29.44,
//      29.44, 29.44, 29.44, 29.44,
//      29.44, 29.44, 29.44, 29.44,
//      29.44, 29.44, 29.44, 29.44};
// double data_2[16] =
//     {35.1, 35.1, 35.1, 35.1,
//      35.1, 35.1, 35.1, 35.1,
//      35.1, 35.1, 35.1, 35.1,
//      35.1, 35.1, 35.1, 35.1};
// #define Txl_3 32
// #define PTAT_1 19.5
// #define PTAT_3 28.4
// double data_3[16] =
//     {30.11, 30.11, 30.11, 30.11,
//      30.11, 30.11, 30.11, 30.11,
//      30.11, 30.11, 30.11, 30.11,
//      30.11, 30.11, 30.11, 30.11};
// double data_calib_3[16] = {0};

/* Calibration at distance of 250mm */
// #define Txl_1 32 //DC1 = Txl_1
// #define Txh 38
// double data_1[16] =
//     {27.75, 27.75, 27.75, 27.75,
//      27.75, 27.75, 27.75, 27.75,
//      27.75, 27.75, 27.75, 27.75,
//      27.75, 27.75, 27.75, 27.75};
// double data_2[16] =
//     {32.11, 32.11, 32.11, 32.11,
//      32.11, 32.11, 32.11, 32.11,
//      32.11, 32.11, 32.11, 32.11,
//      32.11, 32.11, 32.11, 32.11};
// #define Txl_3 32
// #define PTAT_1 19.5
// #define PTAT_3 28.4
// double data_3[16] =
//     {29.48, 29.48, 29.48, 29.48,
//      29.48, 29.48, 29.48, 29.48,
//      29.48, 29.48, 29.48, 29.48,
//      29.48, 29.48, 29.48, 29.48};
// double data_calib_3[16] = {0};

/* Calibration at distance of 300mm */
// #define Txl_1 32 //DC1 = Txl_1
// #define Txh 38
// double data_1[16] =
//     {25.81, 25.81, 25.81, 25.81,
//      25.81, 25.81, 25.81, 25.81,
//      25.81, 25.81, 25.81, 25.81,
//      25.81, 25.81, 25.81, 25.81};
// double data_2[16] =
//     {31.4, 31.4, 31.4, 31.4,
//      31.4, 31.4, 31.4, 31.4,
//      31.4, 31.4, 31.4, 31.4,
//      31.4, 31.4, 31.4, 31.4};
// #define Txl_3 32
// #define PTAT_1 19.5
// #define PTAT_3 28.3
// double data_3[16] =
//     {28.87, 28.87, 28.87, 28.87,
//      28.87, 28.87, 28.87, 28.87,
//      28.87, 28.87, 28.87, 28.87,
//      28.87, 28.87, 28.87, 28.87};
// double data_calib_3[16] = {0};

/* Calibration at distance of 350mm */
#define Txl_1 32 //DC1 = Txl_1
#define Txh 38
double data_1[16] =
    {24.79, 24.79, 24.79, 24.79,
     24.79, 24.79, 24.79, 24.79,
     24.79, 24.79, 24.79, 24.79,
     24.79, 24.79, 24.79, 24.79};
double data_2[16] =
    {29.6, 29.6, 29.6, 29.6,
     29.6, 29.6, 29.6, 29.6,
     29.6, 29.6, 29.6, 29.6,
     29.6, 29.6, 29.6, 29.6};
#define Txl_3 32
#define PTAT_1 19.5
#define PTAT_3 28.3
double data_3[16] =
    {28.48, 28.48, 28.48, 28.48,
     28.48, 28.48, 28.48, 28.48,
     28.48, 28.48, 28.48, 28.48,
     28.48, 28.48, 28.48, 28.48};
double data_calib_3[16] = {0};

/* Calibration at distance of 400mm */
// #define Txl_1 32 //DC1 = Txl_1
// #define Txh 38
// double data_1[16] =
//     {24.34, 24.34, 24.34, 24.34,
//      24.34, 24.34, 24.34, 24.34,
//      24.34, 24.34, 24.34, 24.34,
//      24.34, 24.34, 24.34, 24.34};
// double data_2[16] =
//     {29.26, 29.26, 29.26, 29.26,
//      29.26, 29.26, 29.26, 29.26,
//      29.26, 29.26, 29.26, 29.26,
//      29.26, 29.26, 29.26, 29.26};
// #define Txl_3 32
// #define PTAT_1 19.5
// #define PTAT_3 28.3
// double data_3[16] =
//     {28.07, 28.07, 28.07, 28.07,
//      28.07, 28.07, 28.07, 28.07,
//      28.07, 28.07, 28.07, 28.07,
//      28.07, 28.07, 28.07, 28.07};
// double data_calib_3[16] = {0};


uint8_t rbuf[N_READ];
double pix_data[N_PIXEL];

static uint8_t run_flag = 0;
static uint8_t start_sampling = 0;
static double current_temp = 0.0;
static SemaphoreHandle_t tof_mutex;
static d6t44l_app_cb user_callback = NULL;
static uint8_t temp_present;
static double temp[SAMPLING] = {0};

static void updateScreen(double temp)
{
    char str[10];

    sprintf(str, "%04.1fC", temp);
    ssd1306_fill(Black);
    ssd1306_set_cursor(15, 38);

    ssd1306_write_string(str, Font_16x26, White);
    ssd1306_update_screen();
}

static int scan_device(void)
{
    esp_err_t err;
    i2c_cmd_handle_t cmd;
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, WRITE_ADDR, 1);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 100);
    i2c_cmd_link_delete(cmd);

    if (err == ESP_OK)
        return 1;

    return 0;
}

uint8_t calc_crc(uint8_t data)
{
    int index;
    uint8_t temp;
    for (index = 0; index < 8; index++)
    {
        temp = data;
        data <<= 1;
        if (temp & 0x80)
        {
            data ^= 0x07;
        }
    }
    return data;
}

bool D6T_checkPEC(uint8_t buf[], int n)
{
    int i;
    uint8_t crc = calc_crc((D6T_ADDR << 1) | 1); // I2C Read address (8bit)
    for (i = 0; i < n; i++)
    {
        crc = calc_crc(buf[i] ^ crc);
    }
    bool ret = crc != buf[n];
    if (ret)
    {
        ESP_LOGE(__FUNCTION__, "PEC check Failed: %02X(cal)-%02x(get)\n", crc, buf[n]);
    }
    return ret;
}

int16_t conv8us_s16_le(uint8_t *buf, int n)
{
    int ret;
    ret = buf[n];
    ret += buf[n + 1] << 8;
    return (int16_t)ret; // and convert negative.
}

void initialSetting(void)
{
    /*** add ambient temperature calibration  ***********************/
    for (int i = 0; i < N_PIXEL; i++)
    {
        data_calib_3[i] = (data_3[i] - data_1[i]) * ((double)Txh - (double)Txl_1) / (data_2[i] - data_1[i]) + (double)Txl_1;
    }
    /***************************************************************/
}

static int i2c_write(void)
{
    esp_err_t err;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, WRITE_ADDR, 1);
    i2c_master_write_byte(cmd, D6T_CMD, 1);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 100 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return err;
}

static int i2c_read(uint8_t *val, size_t len)
{
    esp_err_t err;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, READ_ADDR, 1);
    i2c_master_read(cmd, val, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 100 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return err;
}

static void d6t_app_task(void)
{
    int16_t ptat = 0;
    int8_t count = 0;
    int16_t itemp = 0;
    initialSetting();

    while (run_flag)
    {

        memset(rbuf, 0, N_READ);
        int i, j;
        int ret;

        ret = i2c_write();
        vTaskDelay(1000 / portTICK_RATE_MS);
        ret = i2c_read(rbuf, N_READ);
        if (ret == 0)
        {
            if (!D6T_checkPEC(rbuf, N_READ - 1))
            {
                ptat = conv8us_s16_le(rbuf, 0);
#ifdef DEBUG
                ESP_LOGI(TAG, "PTAT: %4.1f [degC], Temperature: ", ptat / 10.0);
#endif
                double hTemp = 0.0;
                for (i = 0, j = 2; i < N_PIXEL; i++, j += 2)
                {
                    itemp = (float)conv8us_s16_le(rbuf, j);
#ifdef DEBUG
                    printf("%4.1f", itemp / 10.0);
                    if ((i % N_ROW) == N_ROW - 1)
                        printf(",\n");
                    else
                        printf(", ");

#endif
#ifdef CALIBRATE
                    pix_data[i] = (((double)itemp) / 10.0 - data_1[i]) * ((double)Txh - (double)Txl_1) / (data_2[i] - data_1[i]) + (double)Txl_1;
                    pix_data[i] = pix_data[i] - (((double)ptat) / 10.0 - (double)PTAT_1) * (data_calib_3[i] - (double)Txl_3) / ((double)PTAT_3 - (double)PTAT_1);
#else
                    /*** add ambient temperature calibration  ***********************/
                    /*if (distance <= 5) 
                    {

                    }*/
                    pix_data[i] = (((double)itemp) / 10.0 - data_1[i]) * ((double)Txh - (double)Txl_1) / (data_2[i] - data_1[i]) + (double)Txl_1;
                    pix_data[i] = pix_data[i] - (((double)ptat) / 10.0 - (double)PTAT_1) * (data_calib_3[i] - (double)Txl_3) / ((double)PTAT_3 - (double)PTAT_1);
                    /***************************************************************/

                    // pix_data[i] = (itemp / 10.0);
#endif
                    if (pix_data[i] > hTemp)
                    {
                        hTemp = pix_data[i];
                    }
                }

                // printf("temp[%d] = %4.1f compensated value %f\n", count, hTemp, dist_compensator);
                printf("hTemp: %4.1f\n", hTemp);

                if (start_sampling == 1)
                {
                    // printf("temp[%d] = %4.1f\n", count, hTemp);
                    temp[count] = hTemp;
                    count += 1;
                }
            }
        }
        else
        {
            temp_present = 0;
        }

        if (count == SAMPLING)
        {
            d6t44l_event_t event = {
                .id = TEMP_EVT_DATA_READY};

            user_callback(&event);
            start_sampling = 0;
            count = 0;
        }
    }
    vTaskDelete(NULL);
}

void D6T44l_get_data(double *dTemp)
{
    if (temp_present)
    {
        TOF_MUTEX_LOCK();
        *dTemp = current_temp;
        TOF_MUTEX_UNLOCK();
    }
}

void D6T44L_update_temp(void)
{
    TOF_MUTEX_LOCK();
    // gpio_set_level(LED_BLUE, 0);

    int i;
    double htemp = 0.0;
    for (i = 0; i < SAMPLING; i++)
    {
        if (temp[i] > htemp)
        {
            htemp = temp[i];
        }
    }
    // printf("\n");
    htemp += 2.0;
    current_temp = htemp;
    temp_present = 1;
    updateScreen(htemp);

    if (htemp >= TEMPERATURE_THRESHOLD)
    {
        // EVT_TEMP_FAIL
        d6t44l_event_t event = {
            .id = TEMP_EVT_TEMP_FAIL};
        user_callback(&event);
    }
    else
    {
        //EVT_TEMP_PASS
        d6t44l_event_t event = {
            .id = TEMP_EVT_TEMP_PASS};
        user_callback(&event);
    }
    TOF_MUTEX_UNLOCK();
}

void D6T44L_reset(void)
{
    updateScreen(0.0);

    memset(temp, 0, sizeof(temp));

    d6t44l_event_t event = {0};
    event.id = TEMP_EVT_RESET;

    user_callback(&event);
}

void D6T44L_start_sampling(void)
{
    start_sampling = 1;
}

void D6T44L_app_run(void)
{
    ESP_LOGI(TAG, "D6T44L start");
    run_flag = 1;
    xTaskCreate(d6t_app_task, "d6t-task", 2048, NULL, 5, NULL);
}

void D6T44L_app_stop(void)
{
    run_flag = 0;
    updateScreen(0.0);
}

int D6T44L_init(d6t44l_app_cb app_cb)
{
    int i;
    tof_mutex = xSemaphoreCreateMutex();
    int err = scan_device();

    if (err != 1)
    {
        ESP_LOGE(__FUNCTION__, "Device Not found.");
        return err;
    }

    user_callback = app_cb;

    for (i = 0; i < N_PIXEL; i++)
    {
        data_calib_3[i] = (data_3[i] - data_1[i]) * ((double)Txh - (double)Txl_1) / (data_2[i] - data_1[i]) + (double)Txl_1;
    }

    temp_present = 0;
    D6T44L_reset();
    return err;
}