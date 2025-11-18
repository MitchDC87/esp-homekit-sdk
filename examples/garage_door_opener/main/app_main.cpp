/*
 * ESPRESSIF MIT License
 *
 * Copyright (c) 2018 <ESPRESSIF SYSTEMS (SHANGHAI) PTE LTD>
 *
 * Permission is hereby granted for use on ESPRESSIF SYSTEMS products only, in which case,
 * it is free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */
/* HomeKit Smart Outlet Example
*/
#include <stdio.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <esp_log.h>
#include <driver/gpio.h>
#include "driver/uart.h"
#include "hciemulator.h"

#ifdef __cplusplus
extern "C"
{
#endif

#include <hap.h>

#include <hap_apple_servs.h>
#include <hap_apple_chars.h>

#include <app_wifi.h>
#include <app_hap_setup_payload.h>

#ifdef __cplusplus
}
#endif

#ifndef max
#define max(a, b) ((a) > (b) ? (a) : (b))
#endif

static const char *TAG = "HAP garage door";
static hap_char_t* currentDoorState;
static hap_char_t* targetDoorState;

#define SMART_OUTLET_TASK_PRIORITY  1
#define SMART_OUTLET_TASK_STACKSIZE 4 * 1024
#define SMART_OUTLET_TASK_NAME      "garage_door_opener"

//UART
static const int RX_BUF_SIZE = 256;

#define TXD_PIN 4
#define RXD_PIN 3

static void uart_init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = 57600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_EVEN,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

// Hörmann HCP2 based on modbus rtu @57.6kB 8E1
HCIEmulator emulator;

volatile unsigned long lastCall = 0;
volatile unsigned long maxPeriod = 0;

void modBusPolling(void *parameter)
{
    while (true)
    {
        if (lastCall > 0)
        {
            maxPeriod = max(micros() - lastCall, maxPeriod);
        }
        lastCall = micros();
        emulator.poll();
        vTaskDelay(1);
    }
    vTaskDelete(NULL);
}

TaskHandle_t modBusTask;

// switch GPIO4 und GPIO2 sync to the lamp
void onStatusChanged(const SHCIState &state)
{
    hap_val_t doorState;
    hap_val_t targDoorState;
    const hap_val_t* activeTargDoorState;
    // see https://ucexperiment.wordpress.com/2016/12/18/yunshan-esp8266-250v-15a-acdc-network-wifi-relay-module/
    // Setting GPIO4 high, causes the relay to close the NO contact with
    if (state.valid)
    {
        ESP_LOGI(TAG, "Garage door state changed %u", state.doorState);
        switch (state.doorState)
        {
        case 0:
            /* code */
            doorState.u = 4;
            break;
        case DOOR_OPEN_POSITION:
            /* code */
            doorState.u = 0;
            break;
        case DOOR_CLOSE_POSITION:
            /* code */
            doorState.u = 1;
            break;
        case DOOR_HALF_POSITION:
            /* code */
            doorState.u = 4;
            break;
        case DOOR_MOVE_CLOSEPOSITION:
            /* code */
            doorState.u = 3;
            break;
        case DOOR_MOVE_OPENPOSITION:
            /* code */
            doorState.u = 2;
            break;
        default:
            break;
        }

        activeTargDoorState = hap_char_get_val(targetDoorState);

        if (state.doorState == DOOR_MOVE_CLOSEPOSITION && activeTargDoorState->u != 1)
        {
            targDoorState.u = 1;
            hap_char_update_val(targetDoorState, &targDoorState);
            ESP_LOGI(TAG, "Setting target door state to closing");
        }
        else if (state.doorState == DOOR_MOVE_OPENPOSITION && activeTargDoorState->u != 0)
        {
            targDoorState.u = 0;
            hap_char_update_val(targetDoorState, &targDoorState);
            ESP_LOGI(TAG, "Setting target door state to opening");
        }

        hap_char_update_val(currentDoorState, &doorState);
    }
}

/* Mandatory identify routine for the accessory.
 * In a real accessory, something like LED blink should be implemented
 * got visual identification
 */
static int outlet_identify(hap_acc_t *ha)
{
    ESP_LOGI(TAG, "Accessory identified");
    return HAP_SUCCESS;
}

/* A dummy callback for handling a write on the "On" characteristic of Outlet.
 * In an actual accessory, this should control the hardware
 */
static int garage_door_write(hap_write_data_t write_data[], int count,
        void *serv_priv, void *write_priv)
{
    int i, ret = HAP_SUCCESS;
    hap_write_data_t *write;
    for (i = 0; i < count; i++) {
        write = &write_data[i];
        if (!strcmp(hap_char_get_type_uuid(write->hc), HAP_CHAR_UUID_TARGET_DOOR_STATE))
        {
            ESP_LOGI(TAG, "Received Write. garage door state %lu", write->val.u);
            /* TODO: Control Actual Hardware */
            if (write->val.u == 0)
            {
                emulator.openDoor();
            }
            else if (write->val.u == 1)
            {
                emulator.closeDoor();
            }
            //hap_char_update_val(write->hc, &(write->val));
            *(write->status) = HAP_STATUS_SUCCESS;
        }
        else
        {
            *(write->status) = HAP_STATUS_RES_ABSENT;
        }
    }
    return ret;
}

static int garage_door_read(hap_char_t *hc, hap_status_t *status_code,
                            void *serv_priv, void *read_priv)
{
    int ret = HAP_SUCCESS;
    hap_val_t doorState;

    if (!strcmp(hap_char_get_type_uuid(hc), HAP_CHAR_UUID_CURRENT_DOOR_STATE) && emulator.getState().valid)
    {
        ESP_LOGI(TAG, "Getting garage door state %u", emulator.getState().doorState);
        switch (emulator.getState().doorState)
        {
        case 0:
            /* code */
            doorState.u = 4;
            break;
        case DOOR_OPEN_POSITION:
            /* code */
            doorState.u = 0;
            break;
        case DOOR_CLOSE_POSITION:
            /* code */
            doorState.u = 1;
            break;
        case DOOR_HALF_POSITION:
            /* code */
            doorState.u = 4;
            break;
        case DOOR_MOVE_CLOSEPOSITION:
            /* code */
            doorState.u = 3;
            break;
        case DOOR_MOVE_OPENPOSITION:
            /* code */
            doorState.u = 2;
            break;
        default:
            break;
        }
        hap_char_update_val(hc, &doorState);
    }

    *status_code = HAP_STATUS_SUCCESS;
    return ret;                         
}

/*The main thread for handling the Smart Outlet Accessory */
static void garage_door_thread_entry(void *p)
{
    hap_acc_t *accessory;
    hap_serv_t *service;

    /* Initialize the HAP core */
    hap_init(HAP_TRANSPORT_WIFI);

    /* Initialise the mandatory parameters for Accessory which will be added as
     * the mandatory services internally
     */
    hap_acc_cfg_t cfg = {
        .name = "Esp-Garagedoor",
        .model = "EspGaragedoor01",
        .manufacturer = "Espressif",
        .serial_num = "001122334455",
        .fw_rev = "0.9.0",
        .hw_rev = NULL,
        .pv = "1.1.0",
        .cid = HAP_CID_GARAGE_DOOR_OPENER,
        .identify_routine = outlet_identify
    };
    /* Create accessory object */
    accessory = hap_acc_create(&cfg);

    /* Add a dummy Product Data */
    uint8_t product_data[] = {'E','S','P','3','2','H','A','P'};
    hap_acc_add_product_data(accessory, product_data, sizeof(product_data));

    /* Add Wi-Fi Transport service required for HAP Spec R16 */
    hap_acc_add_wifi_transport_service(accessory, 0);

    /* Create the Outlet Service. Include the "name" since this is a user visible service  */
    service = hap_serv_garage_door_opener_create(0, 0, false);
    hap_serv_add_char(service, hap_char_name_create("Garage door opener"));

    currentDoorState = hap_serv_get_char_by_uuid(service, HAP_CHAR_UUID_CURRENT_DOOR_STATE);
    targetDoorState = hap_serv_get_char_by_uuid(service, HAP_CHAR_UUID_TARGET_DOOR_STATE);

    /* Set the write callback for the service */
    hap_serv_set_write_cb(service, garage_door_write);

    /* Set the read callback for the service */
    //hap_serv_set_read_cb(service, garage_door_read);

    /* Add the Outlet Service to the Accessory Object */
    hap_acc_add_serv(accessory, service);

    /* Add the Accessory to the HomeKit Database */
    hap_add_accessory(accessory);

    /* For production accessories, the setup code shouldn't be programmed on to
     * the device. Instead, the setup info, derived from the setup code must
     * be used. Use the factory_nvs_gen utility to generate this data and then
     * flash it into the factory NVS partition.
     *
     * By default, the setup ID and setup info will be read from the factory_nvs
     * Flash partition and so, is not required to set here explicitly.
     *
     * However, for testing purpose, this can be overridden by using hap_set_setup_code()
     * and hap_set_setup_id() APIs, as has been done here.
     */
#ifdef CONFIG_EXAMPLE_USE_HARDCODED_SETUP_CODE
        /* Unique Setup code of the format xxx-xx-xxx. Default: 111-22-333 */
        hap_set_setup_code(CONFIG_EXAMPLE_SETUP_CODE);
    /* Unique four character Setup Id. Default: ES32 */
    hap_set_setup_id(CONFIG_EXAMPLE_SETUP_ID);
#ifdef CONFIG_APP_WIFI_USE_WAC_PROVISIONING
    app_hap_setup_payload(CONFIG_EXAMPLE_SETUP_CODE, CONFIG_EXAMPLE_SETUP_ID, true, cfg.cid);
#else
    app_hap_setup_payload(CONFIG_EXAMPLE_SETUP_CODE, CONFIG_EXAMPLE_SETUP_ID, false, cfg.cid);
#endif
#endif

    /* Enable Hardware MFi authentication (applicable only for MFi variant of SDK) */
    hap_enable_mfi_auth(HAP_MFI_AUTH_HW);

    /* Initialize Wi-Fi */
    app_wifi_init();

    /* After all the initializations are done, start the HAP core */
    hap_start();
    /* Start Wi-Fi */
    app_wifi_start(portMAX_DELAY);

    emulator.onStatusChanged(onStatusChanged);

    /* The task ends here. The read/write callbacks will be invoked by the HAP Framework */
    vTaskDelete(NULL);
}

extern "C" void app_main()
{
    // Initialize UART
    uart_init();
    /* Create the application thread */
    xTaskCreate(garage_door_thread_entry, SMART_OUTLET_TASK_NAME, SMART_OUTLET_TASK_STACKSIZE,
                NULL, SMART_OUTLET_TASK_PRIORITY, NULL);

    xTaskCreatePinnedToCore(
        modBusPolling, /* Function to implement the task */
        "ModBusTask",  /* Name of the task */
        10000,         /* Stack size in words */
        NULL,          /* Task input parameter */
        // 1,  /* Priority of the task */
        configMAX_PRIORITIES - 1,
        &modBusTask, /* Task handle. */
        0);          /* Core where the task should run */
}

