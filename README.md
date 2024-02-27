| Supported Targets | ESP32 | ESP32-C2 | ESP32-C3 | ESP32-S2 | ESP32-S3 |
| ----------------- | ----- | -------- | -------- | -------- | -------- |

# Library for OTA update over IR with a sample main.c file

This library was created using two examples in the ESP-IDF framework, namely the `ir_nec_transceiver` 
example and the `native_ota_example`. An extra library by the filename `nec.c` has also been created, and some functions transferred there for the sole purpose of making the code clean. 

The main components to understand the code are the `s_nec_code_command` variable, `ota_write_data` 
variable, the `nec_parse_frame` function, and the `ota_write_example` function.

In this code, all components related to HTTP protocols have been removed.

In the `rmt_example_task()`, we check the received data and parse it using the `nec_parse_frame()` function. In this function, the global variable `s_nec_code_command` variable stores the data received.

In the `ota_example_task`, a variable named `ota_write_data` is created and assigned memory dynamically. Afterward, we do `ota_write_data = s_nec_code_command` and then give up the variable to `esp_ota_write()` API after the `esp_ota_begin()` API.

When the data is transferred to the `ota_write_data`, it is checked to see if the firmware update received is the same as original or different along with any corruption of the firmware image.

The `esp_ota_write()` performs the write operation of writing the update to the boot partition, after which it is validated using the 
`esp_ota_end()` API.

Some of the functions that are apparently missing can be found in the `nec.c` file.

**IMPORTANT NOTE:**
Since this is a copy of the original `native_ota_example` project, when you try to compile this project using `idf.py build`, you might get an error. This happens if you have already compiled the original example once. To go around this, use the `idf.py fullclean` and then use the `idf.py build`. Furthermore, this project is a work in progress and may or may not work. Developers are invited to improve this project as they see fit.
