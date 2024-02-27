#pragma once

#include <stdint.h>
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


//#ifdef __cplusplus
//extern "C" {
//#endif

void ota_init(void);

void rmt_init(void);

void ota_task(void);

void rmt_task(void);

void __attribute__((noreturn)) task_fatal_error(void);

void print_sha256 (const uint8_t *image_hash, const char *label);

bool nec_parse_frame(rmt_symbol_word_t *rmt_nec_symbols);

bool nec_parse_frame_repeat(rmt_symbol_word_t *rmt_nec_symbols);

void parse_nec_frame(rmt_symbol_word_t *rmt_nec_symbols, size_t symbol_num);



//}