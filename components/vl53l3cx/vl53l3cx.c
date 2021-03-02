#include <string.h>
#include <stdlib.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "vl53lx_api.h"
#include "vl53l3cx.h"

#define TAG "vl53l3cx-app"

VL53LX_Dev_t dev;
VL53LX_DEV Dev = &dev;

static uint8_t run_flag = 0;
static vl53l3cx_app_cb user_callbacks = NULL;
static uint8_t isInside = 0;

static void vl53l3cx_app_task(void *arg)
{
    VL53LX_MultiRangingData_t MultiRangingData;
    VL53LX_MultiRangingData_t *pMultiRangingData = &MultiRangingData;
    uint8_t NewDataReady = 0;
    int no_of_object_found = 0;

    VL53LX_Error status = VL53LX_ERROR_NONE;

    do
    {
        status = VL53LX_GetMeasurementDataReady(Dev, &NewDataReady);
        vTaskDelay(10 / portTICK_RATE_MS);
        if ((!status) && (NewDataReady != 0))
        {
            status = VL53LX_GetMultiRangingData(Dev, pMultiRangingData);
            no_of_object_found = pMultiRangingData->NumberOfObjectsFound;
            if (no_of_object_found > 0)
            {
                if (pMultiRangingData->RangeData[0].RangeStatus == VL53LX_RANGESTATUS_RANGE_VALID)
                {
                    printf("D=%5dmm \n", pMultiRangingData->RangeData[0].RangeMilliMeter);
                    if (pMultiRangingData->RangeData[0].RangeMilliMeter < DISTANCE_THRESHOLD && isInside == 0)
                    {
                        isInside = 1;
                        printf("D=%5dmm \n", pMultiRangingData->RangeData[0].RangeMilliMeter);
                        vl53l3cx_event_t event = {0};
                        event.id = TOF_EVT_THRESHOLD_INSIDE;
                        user_callbacks(&event);
                    }
                    else if (pMultiRangingData->RangeData[0].RangeMilliMeter >= DISTANCE_THRESHOLD && isInside == 1)
                    {
                        isInside = 0;
                        vl53l3cx_event_t event = {0};
                        event.id = TOF_EVT_THRESHOLD_OUTSIDE;
                        user_callbacks(&event);
                    }
#ifdef DEBUG
                    printf("status=%d, D=%5dmm, Signal=%2.2f Mcps, Ambient=%2.2f Mcps \n",
                           pMultiRangingData->RangeData[0].RangeStatus,
                           pMultiRangingData->RangeData[0].RangeMilliMeter,
                           pMultiRangingData->RangeData[0].SignalRateRtnMegaCps / 65536.0,
                           pMultiRangingData->RangeData[0].AmbientRateRtnMegaCps / 65536.0);
#endif
                }
            }
            else
            {
                if (isInside == 1)
                {
                    isInside = 0;
                    vl53l3cx_event_t event = {0};
                    event.id = TOF_EVT_THRESHOLD_OUTSIDE;
                    user_callbacks(&event);
                }
            }

            if (status == 0)
            {
                status = VL53LX_ClearInterruptAndStartMeasurement(Dev);
            }
        }
        vTaskDelay(250 / portTICK_RATE_MS);
    } while (run_flag);
    VL53LX_StopMeasurement(Dev);
    vTaskDelete(NULL);
}

void vl53l3cx_reset(void)
{
    isInside = 0;
}

void vl53l3cx_start_app(void)
{
    ESP_LOGI(TAG, "Starting Task");
    run_flag = 1;
    xTaskCreate(vl53l3cx_app_task, "app-task", 2048, NULL, 5, NULL);
}

void vl53l3cx_stop_app(void)
{
    run_flag = 0;
}

VL53LX_Error init_vl53l3cx(vl53l3cx_app_cb app_cb)
{
    // gpio_set_direction(GPIO_NUM_32, GPIO_MODE_OUTPUT);
    // gpio_set_level(GPIO_NUM_32, 0);
    Dev->i2c_slave_address = 0x29;
    // gpio_set_level(GPIO_NUM_32, 1);

    user_callbacks = app_cb;

    VL53LX_Error status = VL53LX_ERROR_NONE;

    // VL53LX_SetDeviceAddress();
    // ESP_LOGI(TAG, "Old address: %x", Dev->i2c_slave_address);

    status = VL53LX_WaitDeviceBooted(Dev);
    ESP_LOGI(TAG, "VL53LX_WaitDeviceBooted %d", status);

    status = VL53LX_DataInit(Dev);
    ESP_LOGI(TAG, "VL53LX_DataInit %d", status);

    vTaskDelay(10 / portTICK_RATE_MS);

    status = VL53LX_StartMeasurement(Dev);
    ESP_LOGI(TAG, "VL53LX_StartMeasurement %d", status);

    return status;
}