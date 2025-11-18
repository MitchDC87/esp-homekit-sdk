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

/* HomeKit Shelly Dimmer Example
*/

#include <stdio.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include "esp_websocket_client.h"
#include "cJSON.h"

#include <hap.h>
#include <hap_apple_servs.h>
#include <hap_apple_chars.h>
#include <hap_fw_upgrade.h>

#include <iot_button.h>

#include <app_wifi.h>
#include <app_hap_setup_payload.h>

/* Comment out the below line to disable Firmware Upgrades */
#define CONFIG_FIRMWARE_SERVICE

static const char *TAG = "HAP lightbulb";

#define LIGHTBULB_TASK_PRIORITY  1
#define LIGHTBULB_TASK_STACKSIZE 4 * 1024
#define LIGHTBULB_TASK_NAME      "hap_lightbulb"

/* Reset network credentials if button is pressed for more than 3 seconds and then released */
#define RESET_NETWORK_BUTTON_TIMEOUT        3

/* Reset to factory if button is pressed and held for more than 10 seconds */
#define RESET_TO_FACTORY_BUTTON_TIMEOUT     10

/* The button "Boot" will be used as the Reset button for the example */
#define RESET_GPIO  GPIO_NUM_0

static hap_char_t *char_on;
static hap_char_t *char_brightness;
static hap_char_t *button0_state;
static hap_char_t *button1_state;

/* Websocket client with shelly dimmer */
static esp_websocket_client_handle_t shelly_websocket;

static char *build_shelly_light_command(bool on, int brightness);

/**
 * @brief The network reset button callback handler.
 * Useful for testing the Wi-Fi re-configuration feature of WAC2
 */
static void reset_network_handler(void *arg)
{
    hap_reset_network();
}
/**
 * @brief The factory reset button callback handler.
 */
static void reset_to_factory_handler(void* arg)
{
    hap_reset_to_factory();
}

static void hap_event_handler(hap_event_t event, void *data)
{
    switch (event)
    {
    case HAP_EVENT_CTRL_CONNECTED:
        ESP_LOGI(TAG, "HomeKit controller connected");
        esp_websocket_client_start(shelly_websocket);
        break;

    case HAP_EVENT_CTRL_DISCONNECTED:
        ESP_LOGI(TAG, "HomeKit controller disconnected");
        esp_websocket_client_stop(shelly_websocket);
        break;

    case HAP_EVENT_PAIRING_ABORTED:
        ESP_LOGW(TAG, "Pairing aborted");
        break;

    default:
        ESP_LOGI(TAG, "Other HAP event: %d", event);
        break;
    }
}

/**
 * The Reset button  GPIO initialisation function.
 * Same button will be used for resetting Wi-Fi network as well as for reset to factory based on
 * the time for which the button is pressed.
 */
static void reset_key_init(uint32_t key_gpio_pin)
{
    button_handle_t handle = iot_button_create(key_gpio_pin, BUTTON_ACTIVE_LOW);
    iot_button_add_on_release_cb(handle, RESET_NETWORK_BUTTON_TIMEOUT, reset_network_handler, NULL);
    iot_button_add_on_press_cb(handle, RESET_TO_FACTORY_BUTTON_TIMEOUT, reset_to_factory_handler, NULL);
}

/* Mandatory identify routine for the accessory.
 * In a real accessory, something like LED blink should be implemented
 * got visual identification
 */
static int light_identify(hap_acc_t *ha)
{
    ESP_LOGI(TAG, "Accessory identified");
    return HAP_SUCCESS;
}

/* Mandatory identify routine for the accessory (bridge)
 * In a real accessory, something like LED blink should be implemented
 * got visual identification
 */
static int bridge_identify(hap_acc_t *ha)
{
    ESP_LOGI(TAG, "Bridge identified");
    return HAP_SUCCESS;
}

/* Callback for handling writes on the Light Bulb Service
 */
static int lightbulb_write(hap_write_data_t write_data[], int count,
        void *serv_priv, void *write_priv)
{
    int i, ret = HAP_SUCCESS;
    hap_write_data_t *write;
    for (i = 0; i < count; i++) {
        write = &write_data[i];
        /* Setting a default error value */
        *(write->status) = HAP_STATUS_VAL_INVALID;
        if (!strcmp(hap_char_get_type_uuid(write->hc), HAP_CHAR_UUID_ON)) {
            ESP_LOGI(TAG, "Received Write for Light %s", write->val.b ? "On" : "Off");
            char *cmd = build_shelly_light_command(write->val.b, -1);
            esp_websocket_client_send_text(shelly_websocket, cmd, strlen(cmd), portMAX_DELAY);
            *(write->status) = HAP_STATUS_SUCCESS;
        } else if (!strcmp(hap_char_get_type_uuid(write->hc), HAP_CHAR_UUID_BRIGHTNESS)) {
            ESP_LOGI(TAG, "Received Write for Light Brightness %d", write->val.i);
            bool on = true;
            if (write->val.i == 0)
            {
                on = false;
            }
            char *cmd = build_shelly_light_command(on, write->val.i);
            esp_websocket_client_send_text(shelly_websocket, cmd, strlen(cmd), portMAX_DELAY);
            *(write->status) = HAP_STATUS_SUCCESS;
        } else if (!strcmp(hap_char_get_type_uuid(write->hc), HAP_CHAR_UUID_HUE)) {
            ESP_LOGI(TAG, "Received Write for Light Hue %f", write->val.f);
            *(write->status) = HAP_STATUS_SUCCESS;
        } else if (!strcmp(hap_char_get_type_uuid(write->hc), HAP_CHAR_UUID_SATURATION)) {
            ESP_LOGI(TAG, "Received Write for Light Saturation %f", write->val.f);
            *(write->status) = HAP_STATUS_SUCCESS;
        } else {
            *(write->status) = HAP_STATUS_RES_ABSENT;
        }
        /* If the characteristic write was successful, update it in hap core
         */
        if (*(write->status) == HAP_STATUS_SUCCESS) {
            hap_char_update_val(write->hc, &(write->val));
        } else {
            /* Else, set the return value appropriately to report error */
            ret = HAP_FAIL;
        }
    }
    return ret;
}

/*The main thread for handling the Bridge Accessory */
static void lightbulb_thread_entry(void *p)
{
    hap_acc_t *accessory;
    hap_serv_t *service;

    /* Initialize the HAP core */
    hap_init(HAP_TRANSPORT_WIFI);

    /* Initialise the mandatory parameters for Accessory which will be added as
     * the mandatory services internally
     */
    hap_acc_cfg_t cfg = {
        .name = "Dimmer-Bridge",
        .manufacturer = "Espressif",
        .model = "DimmerBridge01",
        .serial_num = "001122334455",
        .fw_rev = "0.9.0",
        .hw_rev = NULL,
        .pv = "1.1.0",
        .identify_routine = bridge_identify,
        .cid = HAP_CID_BRIDGE,
    };
    /* Create accessory object */
    accessory = hap_acc_create(&cfg);

    /* Add a dummy Product Data */
    uint8_t product_data[] = {'E', 'S', 'P', '3', '2', 'H', 'A', 'P'};
    hap_acc_add_product_data(accessory, product_data, sizeof(product_data));

    /* Add Wi-Fi Transport service required for HAP Spec R16 */
    hap_acc_add_wifi_transport_service(accessory, 0);

    /* Add the Accessory to the HomeKit Database */
    hap_add_accessory(accessory);

    /* Create and add the Accessory to the Bridge object*/

    hap_acc_cfg_t bridge_cfg_dimmer = {
        .name = "Dimmer",
        .manufacturer = "Espressif",
        .model = "EspDimmer01",
        .serial_num = "abcdefg",
        .fw_rev = "0.9.0",
        .hw_rev = NULL,
        .pv = "1.1.0",
        .identify_routine = light_identify,
        .cid = HAP_CID_BRIDGE,
    };
    /* Create accessory object */
    accessory = hap_acc_create(&bridge_cfg_dimmer);

    /* Create the Fan Service. Include the "name" since this is a user visible service  */
    service = hap_serv_lightbulb_create(true);

    /* Add the optional characteristic to the Light Bulb Service */
    hap_serv_add_char(service, hap_char_name_create("Dimmer"));
    hap_serv_add_char(service, hap_char_brightness_create(50));
    // ret |= hap_serv_add_char(service, hap_char_hue_create(180));
    // ret |= hap_serv_add_char(service, hap_char_saturation_create(100));

    char_on = hap_serv_get_char_by_uuid(service, HAP_CHAR_UUID_ON);
    char_brightness = hap_serv_get_char_by_uuid(service, HAP_CHAR_UUID_BRIGHTNESS);

    /* Set the Accessory name as the Private data for the service,
     * so that the correct accessory can be identified in the
     * write callback
     */
    int *ptr = (int *)malloc(sizeof(int));
    *ptr = 0;
    hap_serv_set_priv(service, ptr);

    /* Set the write callback for the service */
    hap_serv_set_write_cb(service, lightbulb_write);

    /* Add the Fan Service to the Accessory Object */
    hap_acc_add_serv(accessory, service);

    /* Add the Accessory to the HomeKit Database */
    hap_add_bridged_accessory(accessory, hap_get_unique_aid("Dimmer"));

    // Button 0 config
    hap_acc_cfg_t bridge_cfg_button0 = {
        .name = "Button0",
        .manufacturer = "Espressif",
        .model = "EspButton01",
        .serial_num = "abcdefg",
        .fw_rev = "0.9.0",
        .hw_rev = NULL,
        .pv = "1.1.0",
        .identify_routine = light_identify,
        .cid = HAP_CID_BRIDGE,
    };

    /* Create accessory object */
    accessory = hap_acc_create(&bridge_cfg_button0);

    /* Create the Outlet Service. Include the "name" since this is a user visible service  */
    service = hap_serv_stateless_programmable_switch_create(0);
    hap_serv_add_char(service, hap_char_name_create("Button0"));

    /* Get pointer to the outlet in use characteristic which we need to monitor for state changes */
    button0_state = hap_serv_get_char_by_uuid(service, HAP_CHAR_UUID_PROGRAMMABLE_SWITCH_EVENT);

    /* Add the Fan Service to the Accessory Object */
    hap_acc_add_serv(accessory, service);

    /* Add the Accessory to the HomeKit Database */
    hap_add_bridged_accessory(accessory, hap_get_unique_aid("Button0"));

    // Button 0 config
    hap_acc_cfg_t bridge_cfg_button1 = {
        .name = "Button1",
        .manufacturer = "Espressif",
        .model = "EspButton01",
        .serial_num = "abcdefg",
        .fw_rev = "0.9.0",
        .hw_rev = NULL,
        .pv = "1.1.0",
        .identify_routine = light_identify,
        .cid = HAP_CID_BRIDGE,
    };

    /* Create accessory object */
    accessory = hap_acc_create(&bridge_cfg_button1);

    /* Create the Outlet Service. Include the "name" since this is a user visible service  */
    service = hap_serv_stateless_programmable_switch_create(0);
    hap_serv_add_char(service, hap_char_name_create("Button1"));

    /* Get pointer to the outlet in use characteristic which we need to monitor for state changes */
    button1_state = hap_serv_get_char_by_uuid(service, HAP_CHAR_UUID_PROGRAMMABLE_SWITCH_EVENT);

    /* Add the Fan Service to the Accessory Object */
    hap_acc_add_serv(accessory, service);

    /* Add the Accessory to the HomeKit Database */
    hap_add_bridged_accessory(accessory, hap_get_unique_aid("Button1"));

    /* Register a common button for reset Wi-Fi network and reset to factory.
     */
    reset_key_init(RESET_GPIO);

    /* TODO: Do the actual hardware initialization here */

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

    /* The task ends here. The read/write callbacks will be invoked by the HAP Framework */
    vTaskDelete(NULL);
}

static void handle_incoming_message(const char *data, int len)
{
    hap_val_t val_on;
    hap_val_t val_brightness;

    hap_val_t val_button = {
        .i = 0,
    };

    cJSON *root = cJSON_ParseWithLength(data, len);
    if (!root)
    {
        ESP_LOGW(TAG, "Failed to parse JSON");
        return;
    }

    cJSON *method = cJSON_GetObjectItem(root, "method");
    if (!method || !method->valuestring)
    {
        cJSON_Delete(root);
        return;
    }

    // -----------------------------
    // Handle NotifyStatus messages
    // -----------------------------
    if (strcmp(method->valuestring, "NotifyStatus") == 0)
    {
        cJSON *params = cJSON_GetObjectItem(root, "params");
        if (params)
        {
            cJSON *light0 = cJSON_GetObjectItem(params, "light:0");
            if (light0)
            {
                // Brightness update
                cJSON *brightness = cJSON_GetObjectItem(light0, "brightness");
                if (brightness && cJSON_IsNumber(brightness))
                {
                    ESP_LOGI(TAG, "Brightness changed: %d%%", brightness->valueint);
                    val_brightness.i = brightness->valueint;
                    hap_char_update_val(char_brightness, &val_brightness);
                }

                // On/off update
                cJSON *on = cJSON_GetObjectItem(light0, "output");
                if (on && cJSON_IsBool(on))
                {
                    int isOn = cJSON_IsTrue(on);
                    ESP_LOGI(TAG, "On changed: %d", isOn);
                    val_on.b = isOn;
                    hap_char_update_val(char_on, &val_on);
                }
            }
        }
    }

    // -----------------------------
    // Handle NotifyEvent messages
    // -----------------------------
    else if (strcmp(method->valuestring, "NotifyEvent") == 0)
    {
        cJSON *params = cJSON_GetObjectItem(root, "params");
        if (params)
        {
            cJSON *events = cJSON_GetObjectItem(params, "events");
            if (cJSON_IsArray(events))
            {
                int event_count = cJSON_GetArraySize(events);
                for (int i = 0; i < event_count; i++)
                {
                    cJSON *event_obj = cJSON_GetArrayItem(events, i);
                    if (event_obj)
                    {
                        cJSON *event_field = cJSON_GetObjectItem(event_obj, "event");
                        cJSON *component_field = cJSON_GetObjectItem(event_obj, "component");
                        if (cJSON_IsString(event_field))
                        {
                            const char *event_value = event_field->valuestring;
                            const char *component_str = component_field->valuestring;
                            // Extract the input number from "input:X"
                            int input_number = -1;
                            if (strncmp(component_str, "input:", 6) == 0)
                            {
                                input_number = atoi(component_str + 6);
                            }

                            ESP_LOGI(TAG, "Event: %s from input %d", event_value, input_number);

                            // Example: Handle "triple_push"
                            if (strcmp(event_value, "triple_push") == 0)
                            {
                                ESP_LOGI(TAG, "Triple push detected");
                                val_button.i = 2;
                                if (input_number == 0)
                                {
                                    hap_char_update_val(button0_state, &val_button);
                                }
                                else if (input_number == 1)
                                {
                                    hap_char_update_val(button1_state, &val_button);
                                }
                                
                            }
                            else if (strcmp(event_value, "single_push") == 0)
                            {
                                ESP_LOGI(TAG, "Single push detected");
                                val_button.i = 0;
                                if (input_number == 0)
                                {
                                    hap_char_update_val(button0_state, &val_button);
                                }
                                else if (input_number == 1)
                                {
                                    hap_char_update_val(button1_state, &val_button);
                                }
                            }
                            else if (strcmp(event_value, "double_push") == 0)
                            {
                                ESP_LOGI(TAG, "Double push detected");
                                val_button.i = 1;
                                if (input_number == 0)
                                {
                                    hap_char_update_val(button0_state, &val_button);
                                }
                                else if (input_number == 1)
                                {
                                    hap_char_update_val(button1_state, &val_button);
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    // Clean up
    cJSON_Delete(root);
}

// Function to build JSON command
char *build_shelly_light_command(bool on, int brightness)
{
    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "id", 1);
    cJSON_AddStringToObject(root, "src", "esp32-client");
    cJSON_AddStringToObject(root, "method", "Light.Set");

    cJSON *params = cJSON_CreateObject();
    cJSON_AddNumberToObject(params, "id", 0);
    cJSON_AddBoolToObject(params, "on", on);
    if (brightness > 0)
    {
        cJSON_AddNumberToObject(params, "brightness", brightness);
    }
    cJSON_AddItemToObject(root, "params", params);

    char *json_str = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return json_str; // Remember to free after sending
}

/* WebSocket event handler */
static void websocket_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_websocket_event_data_t *data = (esp_websocket_event_data_t *)event_data;

    switch (event_id)
    {
    case WEBSOCKET_EVENT_CONNECTED:
        ESP_LOGI(TAG, "WebSocket Connected");

        // Send initial message with src to receive notifications
        {
            const char *init_msg = "{\"id\":1,\"src\":\"esp32-client\",\"method\":\"Light.GetStatus\",\"params\":{\"id\":0}}";
            esp_websocket_client_send_text(shelly_websocket, init_msg, strlen(init_msg), portMAX_DELAY);
        }
        break;

    case WEBSOCKET_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "WebSocket Disconnected");
        break;

    case WEBSOCKET_EVENT_DATA:
        ESP_LOGI(TAG, "Received data (%d bytes)", data->data_len);
        handle_incoming_message(data->data_ptr, data->data_len);
        break;

    case WEBSOCKET_EVENT_ERROR:
        ESP_LOGE(TAG, "WebSocket Error");
        break;
    }
}

void app_main()
{
    esp_websocket_client_config_t websocket_cfg = {
        .uri = "ws://shellydimmerg3-b08184f22bf4.local/rpc", // <--- change to your Shelly IP
    };

    shelly_websocket = esp_websocket_client_init(&websocket_cfg);

    esp_websocket_register_events(shelly_websocket, WEBSOCKET_EVENT_ANY, websocket_event_handler, (void *)shelly_websocket);

    hap_register_event_handler(hap_event_handler);

    xTaskCreate(lightbulb_thread_entry, LIGHTBULB_TASK_NAME, LIGHTBULB_TASK_STACKSIZE,
            NULL, LIGHTBULB_TASK_PRIORITY, NULL);
}
