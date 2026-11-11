#include "mesh_tasks.h"

#include <string.h>
#include <stdbool.h>

#include "esp_log.h"
#include "esp_mesh.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#include "mesh_messages.h"
#include "config.h"

#define BUTTON_PIN GPIO_NUM_14

static const char *TAG_RX = "MESH_RX";
static const char *TAG_TX = "MESH_TX";

static bool local_alarm_active = false;

void mesh_tx_task(void *arg)
{

    const TickType_t heartbeat_interval = pdMS_TO_TICKS(1000);
    TickType_t last_heartbeat = xTaskGetTickCount() - heartbeat_interval;
    int previous_button_level = 0;

    while (true)
    {
        const int button_level = gpio_get_level(BUTTON_PIN);
        const bool alarm_pressed = (previous_button_level == 1 && button_level == 0);

        if (button_level != previous_button_level)
        {
            ESP_LOGI("BUTTON", "GPIO14 läge = %d", button_level);
            previous_button_level = button_level;
        }

        if (alarm_pressed)
        {
            local_alarm_active = true;
             ESP_LOGW(TAG_TX, "BRANDLARM AKTIVERAT! Nod ID: %d",
             MESH_NODE_ID);
        }

        const TickType_t now = xTaskGetTickCount();
        const bool heartbeat_due = (now - last_heartbeat) >= heartbeat_interval;

        // Kontrollera att noden är ansluten till nätverket innan sändning görs
        bool is_connected = esp_mesh_is_root() || (esp_mesh_get_layer() > 0);

        if (is_connected && (alarm_pressed || (!esp_mesh_is_root() && heartbeat_due)))
        {
            mesh_message_t msg = {
                .node_id = MESH_NODE_ID,
                .msg_type = alarm_pressed ? MSG_ALARM_ON : MSG_HEARTBEAT,
                .timestamp = esp_mesh_get_tsf_time(),
                .alarm_active = local_alarm_active,
                .person_count = 0
            };

            mesh_data_t data = {
                .data = (uint8_t *)&msg,
                .size = sizeof(msg),
                .proto = MESH_PROTO_BIN,
                .tos = MESH_TOS_P2P
            };

            esp_err_t err;

            if (esp_mesh_is_root())
            {
                // Om root skickar (eget knapptryck): sänd nedåt i trädet till alla barn
                err = esp_mesh_send(NULL, &data, MESH_DATA_TODS, NULL, 0);
            }
            else
            {
                // Vanlig nod: skicka uppåt mot root
                err = esp_mesh_send(NULL, &data, 0, NULL, 0);
            }

            if (err == ESP_OK)
            {
                ESP_LOGI(TAG_TX, "Skickat: nod=%lu typ=%d larm=%d",
                         (unsigned long)msg.node_id, msg.msg_type, msg.alarm_active);
            }
            else
            {
                ESP_LOGE(TAG_TX, "Sändning misslyckades: %s", esp_err_to_name(err));
            }

            if (!alarm_pressed)
            {
                last_heartbeat = now;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20)); // Avstudsning
    }
}

void mesh_rx_task(void *arg)
{
    ESP_LOGI(TAG_RX, "RX task startad");

    mesh_addr_t from;
    mesh_data_t data = {0};
    mesh_message_t msg;

    data.data = (uint8_t *)&msg;

    while (true)
    {
        data.size = sizeof(msg);

        esp_err_t err = esp_mesh_recv(
            &from,
            &data,
            portMAX_DELAY,
            NULL,
            NULL,
            0);

        if (err == ESP_OK)
        {
            if (data.size != sizeof(msg))
            {
                ESP_LOGW(TAG_RX, "Felaktig storlek: mottog %u, förväntade %u",
                         (unsigned)data.size, (unsigned)sizeof(msg));
                continue;
            }

            int64_t rx_time = esp_mesh_get_tsf_time();
            int64_t latency_us = rx_time - msg.timestamp;

            switch (msg.msg_type)
            {
                case MSG_HEARTBEAT:
                    ESP_LOGI(TAG_RX, "Heartbeat från nod %lu (latens: %lld us) personantal %d",
                             (unsigned long)msg.node_id, latency_us,msg.person_count);
                    break;

                case MSG_ALARM_ON:
                    local_alarm_active = true;
                    ESP_LOGW(TAG_RX, "🚨 BRANDLARM MOTTAGET från nod %lu!", (unsigned long)msg.node_id);

                    // Om noden är Root: sprid vidare larmet nedåt till resten av nätverket
                    if (esp_mesh_is_root())
                    {
                        mesh_data_t fwd_data = {
                            .data = (uint8_t *)&msg,
                            .size = sizeof(msg),
                            .proto = MESH_PROTO_BIN,
                            .tos = MESH_TOS_P2P
                        };

                        esp_err_t fwd_err = esp_mesh_send(NULL, &fwd_data, MESH_DATA_TODS, NULL, 0);
                        if (fwd_err == ESP_OK)
                        {
                            ESP_LOGI(TAG_RX, "Root vidarebefordrade larmet till alla noder.");
                        }
                        else
                        {
                            ESP_LOGE(TAG_RX, "Vidarebefordran misslyckades: %s", esp_err_to_name(fwd_err));
                        }
                    }
                    break;

                case MSG_ALARM_OFF:
                    local_alarm_active = false;
                    ESP_LOGI(TAG_RX, "Larm återställt från nod %lu", (unsigned long)msg.node_id);
                    break;

                default:
                    ESP_LOGW(TAG_RX, "Okänd meddelandetyp %d", (int)msg.msg_type);
                    break;
            }
        }
        else
        {
            ESP_LOGE(TAG_RX, "Mottagning misslyckades: %s", esp_err_to_name(err));
        }
    }
}