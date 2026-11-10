#include "mesh_tasks.h"

#include <string.h>

#include "esp_log.h"
#include "esp_mesh.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mesh_messages.h"
#include "config.h"
#include "driver/gpio.h"

#define BUTTON_PIN GPIO_NUM_14



static const char *TAG_RX = "MESH_RX";
/**
 * @brief FreeRTOS-uppgift för sändning av mesh-meddelanden.
 *
 * Funktionen körs som en separat task och skickar med jämna
 * mellanrum meddelanden till mesh-nätverket. I den aktuella
 * implementationen används den främst för heartbeat-meddelanden
 * som innehåller nod-ID och tidsstämpel.
 *
 * @param arg Pekare till eventuella uppstartsparametrar.
 */
void mesh_tx_task(void *arg)
{
    const TickType_t heartbeat_interval = pdMS_TO_TICKS(5000);
    TickType_t last_heartbeat = xTaskGetTickCount() - heartbeat_interval;
    int previous_button_level = 1;

    while (true)
    {
        const int button_level = gpio_get_level(BUTTON_PIN);
        const bool alarm_pressed = previous_button_level == 1 && button_level == 0;
        previous_button_level = button_level;


        if (button_level != previous_button_level)
        {
            ESP_LOGI("BUTTON", "GPIO14 = %d", button_level);
        }
        
        const TickType_t now = xTaskGetTickCount();
        const bool heartbeat_due = (now - last_heartbeat) >= heartbeat_interval;

        if (!esp_mesh_is_root() && (alarm_pressed || heartbeat_due))
        {
            mesh_message_t msg = {
                .node_id = MESH_NODE_ID,
                .msg_type = alarm_pressed ? MSG_ALARM_ON : MSG_HEARTBEAT,
                .timestamp = esp_mesh_get_tsf_time()
            };

            if (!alarm_pressed)
            {
                last_heartbeat = now;
            }

            mesh_data_t data = {
                .data = (uint8_t *)&msg,
                .size = sizeof(msg),
                .proto = MESH_PROTO_BIN,
                .tos = MESH_TOS_P2P
            };

            esp_err_t err = esp_mesh_send(NULL, &data, 0, NULL, 0);

            if (err == ESP_OK)
            {
                ESP_LOGI("MESH_TX", "Sent node=%d type=%d timestamp=%lld",
                         msg.node_id, msg.msg_type, msg.timestamp);
            }
            else
            {
                ESP_LOGE("MESH_TX", "Send failed: %s", esp_err_to_name(err));
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}


/**
 * @brief FreeRTOS-uppgift för mottagning av mesh-meddelanden.
 *
 * Funktionen väntar på inkommande meddelanden från mesh-nätverket.
 * När ett meddelande tas emot kontrolleras meddelandetyp,
 * avsändarnod och tidsstämpel. Mottagningstiden används för att
 * beräkna latensen mellan sändare och mottagare.
 *
 * @param arg Pekare till eventuella uppstartsparametrar.
 */
void mesh_rx_task(void *arg)
{
    ESP_LOGI("MESH_RX", "RX task started");

    mesh_addr_t from;
    mesh_data_t data = {0};

   mesh_message_t msg;

   data.data = (uint8_t *)&msg;
   ESP_LOGI(TAG_RX,
         "I am root = %d",
         esp_mesh_is_root());

    while (true)
    {
        data.size = sizeof(msg);
        ESP_LOGI(TAG_RX, "Waiting for packet...");

        esp_err_t err = esp_mesh_recv(
            &from,
            &data,
            portMAX_DELAY,
            NULL,
            NULL,
            0);
          

       if (err == ESP_OK)
{
      ESP_LOGI(TAG_RX,
         "Received packet size=%u",
         (unsigned)data.size);

    if (data.size != sizeof(msg))
    {
        ESP_LOGW(TAG_RX, "Fel meddelandelängd: %u", (unsigned)data.size);
        continue;
    }
            int64_t rx_time = esp_mesh_get_tsf_time();
            int64_t latency_us = rx_time - msg.timestamp;

    switch (msg.msg_type)
    {
        case MSG_HEARTBEAT:
            ESP_LOGI(TAG_RX, "Heartbeat från nod %u med latens %lld us",
                     (unsigned)msg.node_id , latency_us);
            break;

        case MSG_ALARM_ON:
            ESP_LOGW(TAG_RX, "LARM från nod %u",
                     (unsigned)msg.node_id);
            break;

        case MSG_ALARM_OFF:
            ESP_LOGI(TAG_RX, "Larm återställt från nod %u",
                     (unsigned)msg.node_id);
            break;

        case MSG_ACK:
            ESP_LOGI(TAG_RX, "ACK från nod %u",
                     (unsigned)msg.node_id);
            break;

        default:
            ESP_LOGW(TAG_RX, "Okänd meddelandetyp %d",
                     (int)msg.msg_type);
            break;
    }
}
        else
        {
            ESP_LOGE(TAG_RX, "Mottagning misslyckades: %s",
                     esp_err_to_name(err));
        }
    }
}