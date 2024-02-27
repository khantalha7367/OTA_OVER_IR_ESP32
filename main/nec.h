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


bool nec_check_in_range(uint32_t signal_duration, uint32_t spec_duration);

bool nec_parse_logic0(rmt_symbol_word_t *rmt_nec_symbols);

bool nec_parse_logic1(rmt_symbol_word_t *rmt_nec_symbols);

bool rmt_rx_done_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_data);

bool diagnostic(void);

void infinite_loop(void);

void __attribute__((noreturn)) task_fatal_error(void);