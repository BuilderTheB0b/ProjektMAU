#include "mesh_tasks.h"

#include <stdbool.h>
#include <string.h>

#include "config.h"
#include "mesh_messages.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_mesh.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define BUTTON_PIN GPIO_NUM_14
#define DEBOUNCE_MS 50
#define HEARTBEAT_MS 1000
#define ROUTING_TABLE_CAPACITY 50

static const char *TAG = "MESH_ALARM";
static bool local_alarm_active = false;

/* Root sends a separate P2P packet to every address in its routing table.
   MESH_DATA_TODS is NOT a broadcast flag. */
static void root_fanout(const mesh_message_t *message)
{
    mesh_addr_t routes[ROUTING_TABLE_CAPACITY];
    int route_count = 0;
    esp_err_t err = esp_mesh_get_routing_table(
        routes, sizeof(routes), &route_count);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Routing table: %s", esp_err_to_name(err));
        return;
    }

    mesh_addr_t self = {0};
    esp_mesh_get_id(&self);
    mesh_data_t data = {
        .data = (uint8_t *)message,
        .size = sizeof(*message),
        .proto = MESH_PROTO_BIN,
        .tos = MESH_TOS_P2P
    };

    for (int i = 0; i < route_count; ++i) {
        if (memcmp(routes[i].addr, self.addr, sizeof(self.addr)) == 0) {
            continue;
        }
        err = esp_mesh_send(&routes[i], &data, 0, NULL, 0);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Alarm till route %d misslyckades: %s",
                     i, esp_err_to_name(err));
        }
    }
    ESP_LOGI(TAG, "Root skickade alarm till routingtabellen (%d poster)",
             route_count);
}

static void send_to_root(const mesh_message_t *message)
{
    mesh_addr_t root = {0};
    esp_err_t err = esp_mesh_get_root_addr(&root);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Ingen root-adress: %s", esp_err_to_name(err));
        return;
    }

    mesh_data_t data = {
        .data = (uint8_t *)message,
        .size = sizeof(*message),
        .proto = MESH_PROTO_BIN,
        .tos = MESH_TOS_P2P
    };
    err = esp_mesh_send(&root, &data, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Sändning till root misslyckades: %s",
                 esp_err_to_name(err));
    }
}

static void send_message(message_type_t type)
{
    mesh_message_t message = {
        .node_id = MESH_NODE_ID,
        .msg_type = type,
        .timestamp = esp_mesh_get_tsf_time(),
        .alarm_active = local_alarm_active,
        .person_count = 0
    };

    if (esp_mesh_is_root()) {
        if (type == MSG_ALARM_ON) {
            root_fanout(&message);
        }
    } else if (esp_mesh_get_layer() > 0) {
        send_to_root(&message);
    } else {
        ESP_LOGW(TAG, "Ingen mesh-anslutning; lokalt larm kvarstår");
    }
}

void mesh_tx_task(void *arg)
{
    (void)arg;
    const TickType_t debounce = pdMS_TO_TICKS(DEBOUNCE_MS);
    const TickType_t heartbeat = pdMS_TO_TICKS(HEARTBEAT_MS);
    int stable_level = gpio_get_level(BUTTON_PIN);
    int last_sample = stable_level;
    TickType_t changed_at = xTaskGetTickCount();
    TickType_t last_heartbeat = xTaskGetTickCount();

    while (true) {
        TickType_t now = xTaskGetTickCount();
        int sample = gpio_get_level(BUTTON_PIN);
        if (sample != last_sample) {
            last_sample = sample;
            changed_at = now;
        }

        if ((now - changed_at) >= debounce && sample != stable_level) {
            stable_level = sample;
            if (stable_level == 0) {
                local_alarm_active = true;
                ESP_LOGW(TAG, "BRANDLARM AKTIVERAT lokalt på nod %d", MESH_NODE_ID);
                send_message(MSG_ALARM_ON);
            }
        }

        if ((now - last_heartbeat) >= heartbeat) {
            last_heartbeat = now;
            if (!esp_mesh_is_root() && esp_mesh_get_layer() > 0) {
                send_message(MSG_HEARTBEAT);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void mesh_rx_task(void *arg)
{
    (void)arg;
    mesh_addr_t from = {0};
    mesh_message_t message;
    mesh_data_t data = {
        .data = (uint8_t *)&message,
        .size = sizeof(message)
    };

    while (true) {
        data.size = sizeof(message);
        esp_err_t err = esp_mesh_recv(
            &from, &data, portMAX_DELAY, NULL, NULL, 0);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Mottagning: %s", esp_err_to_name(err));
            continue;
        }
        if (data.size != sizeof(message)) {
            ESP_LOGW(TAG, "Ogiltig meddelandestorlek: %u", (unsigned)data.size);
            continue;
        }

        switch (message.msg_type) {
        case MSG_HEARTBEAT:
            ESP_LOGI(TAG, "Heartbeat fran nod %lu, personer=%u",
                     (unsigned long)message.node_id,
                     (unsigned)message.person_count);
            break;
        case MSG_ALARM_ON:
            local_alarm_active = true;
            ESP_LOGW(TAG, "BRANDLARM MOTTAGET fran nod %lu",
                     (unsigned long)message.node_id);
            if (esp_mesh_is_root()) {
                root_fanout(&message);
            }
            break;
        case MSG_ALARM_OFF:
            ESP_LOGW(TAG, "MSG_ALARM_OFF ignoreras i detta test; larmet ar latched");
            break;
        case MSG_ACK:
            ESP_LOGI(TAG, "ACK fran nod %lu", (unsigned long)message.node_id);
            break;
        default:
            ESP_LOGW(TAG, "Okand meddelandetyp: %d", (int)message.msg_type);
            break;
        }
    }
}
