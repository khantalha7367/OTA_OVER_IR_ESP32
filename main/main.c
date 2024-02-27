/*
This is just a general main.c file which can be used to write any related code using
the libraries designed for OTA updates over IR on NEC Protocol
*/



#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_rx.h"
#include "ir_nec_encoder.h"
#include <string.h>
#include <inttypes.h>
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_app_format.h"
#include "esp_http_client.h"
#include "esp_flash_partitions.h"
#include "esp_partition.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "protocol_examples_common.h"
#include "errno.h"
#include "native_ota_example.h"
#include "nec.h"





void app_main(void)
{
    ota_init();
    rmt_init();
    while(1)
    {
        rmt_task();

        ota_task();

        vTaskDelay(1000/portTICK_PERIOD_MS);
        
    }
}