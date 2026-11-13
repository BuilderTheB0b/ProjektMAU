
#include "mesh_tasks.h"

#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#include "esp_log.h"
#include "esp_mesh.h"
#include "esp_wifi.h"
#include "esp_mac.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#include "mesh_messages.h"
#include "config.h"
#include "evacuation.h"

#define BUTTON_PIN GPIO_NUM_14
#define MAX_NODE_IDS 256
#define MAX_MESH_NODES 10

static const char *TAG_RX = "MESH_RX";
static const char *TAG_TX = "MESH_TX";

// Larmstatus för den lokala noden
static bool local_alarm_active = false;

// Registrerar vilka noder som redan skickat brandlarm
static bool received_alarm_from_node[MAX_NODE_IDS] = {false};

static evac_node_state_t node_states[EVAC_NUM_NODES] = {0};

static void calculate_evacuation_route(void)
{
    evacuation_result_t result =
        evacuation_calculate_route(MESH_NODE_ID, node_states);

    if (result.route_found)
    {
        ESP_LOGI("DIJKSTRA",
                 "Nod %d | Nästa nod: %d | Utgång: %d | Kostnad: %.1f",
                 MESH_NODE_ID,
                 result.next_node,
                 result.target_exit,
                 result.total_cost);
    }
    else
    {
        ESP_LOGW("DIJKSTRA",
                 "Nod %d | Ingen säker väg hittades!",
                 MESH_NODE_ID);
    }
}


// ----------------------------------------------------
// Vidarebefordra brandlarm till övriga mesh-noder
// ----------------------------------------------------

static void forward_alarm_to_mesh(const mesh_message_t *msg,
                                  const mesh_addr_t *sender)
{
    if (!esp_mesh_is_root())
    {
        return;
    }

    mesh_addr_t routing_table[MAX_MESH_NODES];
    int routing_table_size = 0;

    esp_err_t err = esp_mesh_get_routing_table(
        routing_table,
        sizeof(routing_table),
        &routing_table_size
    );

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG_RX,
                 "Kunde inte hämta routingtabellen: %s",
                 esp_err_to_name(err));
        return;
    }

    // Hämta Roots egen MAC-adress
    mesh_addr_t root_mac = {0};

    err = esp_wifi_get_mac(WIFI_IF_STA, root_mac.addr);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG_RX,
                 "Kunde inte hämta Root MAC: %s",
                 esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG_RX,
             "Routingtabell innehåller %d noder",
             routing_table_size);

    mesh_data_t data = {
        .data = (uint8_t *)msg,
        .size = sizeof(*msg),
        .proto = MESH_PROTO_BIN,
        .tos = MESH_TOS_P2P
    };

    for (int i = 0; i < routing_table_size; i++)
    {
        mesh_addr_t *destination = &routing_table[i];

        // Skicka inte till Root själv
        if (memcmp(destination->addr,
                   root_mac.addr, 6) == 0)
        {
            continue;
        }

        // Skicka inte tillbaka till avsändaren
        if (sender != NULL &&
            memcmp(destination->addr,
                   sender->addr, 6) == 0)
        {
            continue;
        }

        err = esp_mesh_send(
            destination,
            &data,
            MESH_DATA_P2P,
            NULL,
            0
        );

        if (err == ESP_OK)
        {
            ESP_LOGI(TAG_RX,
                     "Larm skickat till " MACSTR,
                     MAC2STR(destination->addr));
        }
        else
        {
            ESP_LOGE(TAG_RX,
                     "Kunde inte skicka larm till " MACSTR
                     ": %s",
                     MAC2STR(destination->addr),
                     esp_err_to_name(err));
        }
    }
}


// ----------------------------------------------------
// TX-task: knapphantering och heartbeat
// ----------------------------------------------------

void mesh_tx_task(void *arg)
{
    const TickType_t heartbeat_interval =
        pdMS_TO_TICKS(5000);

    const TickType_t debounce_time =
        pdMS_TO_TICKS(50);

    TickType_t last_heartbeat =
        xTaskGetTickCount() - heartbeat_interval;

    TickType_t last_button_change =
        xTaskGetTickCount();

    int last_raw_button_level =
        gpio_get_level(BUTTON_PIN);

    int stable_button_level =
        last_raw_button_level;

    while (true)
    {
        const TickType_t now =
            xTaskGetTickCount();

        const int button_level =
            gpio_get_level(BUTTON_PIN);

        bool alarm_pressed = false;

        // Debounce
        if (button_level != last_raw_button_level)
        {
            last_button_change = now;
            last_raw_button_level = button_level;
        }

        if ((now - last_button_change) >= debounce_time &&
            button_level != stable_button_level)
        {
            stable_button_level = button_level;

            ESP_LOGI("BUTTON",
                     "GPIO14 stabilt läge = %d",
                     stable_button_level);

            if (stable_button_level == 0 &&
                !local_alarm_active)
            {
                alarm_pressed = true;
            }
        }

        // Aktivera lokalt brandlarm
        if (alarm_pressed)
        {
            local_alarm_active = true;

            received_alarm_from_node[MESH_NODE_ID] = true;

            if (MESH_NODE_ID < EVAC_NUM_NODES)
            {
                node_states[MESH_NODE_ID].status = EVAC_NODE_FIRE;
                calculate_evacuation_route();
            }

            ESP_LOGW(TAG_TX,
                     "BRANDLARM AKTIVERAT! Nod ID: %d",
                     MESH_NODE_ID);
        }

        const bool heartbeat_due =
            (now - last_heartbeat) >= heartbeat_interval;

        bool is_connected =
            esp_mesh_is_root() ||
            (esp_mesh_get_layer() > 0);

        if (is_connected &&
            (alarm_pressed ||
             (!esp_mesh_is_root() && heartbeat_due)))
        {
            mesh_message_t msg = {
                .node_id = MESH_NODE_ID,
                .msg_type = alarm_pressed
                    ? MSG_ALARM_ON
                    : MSG_HEARTBEAT,
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

            esp_err_t err = ESP_OK;

            if (esp_mesh_is_root())
            {
                // Root skickar till övriga noder
                forward_alarm_to_mesh(&msg, NULL);
            }
            else
            {
                // Child skickar uppåt till Root
                err = esp_mesh_send(
                    NULL,
                    &data,
                    0,
                    NULL,
                    0
                );
            }

            if (err == ESP_OK)
            {
                ESP_LOGI(TAG_TX,
                         "Skickat: nod=%lu typ=%d larm=%d",
                         (unsigned long)msg.node_id,
                         msg.msg_type,
                         msg.alarm_active);
            }
            else
            {
                ESP_LOGE(TAG_TX,
                         "Sändning misslyckades: %s",
                         esp_err_to_name(err));
            }

            if (!alarm_pressed)
            {
                last_heartbeat = now;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}


// ----------------------------------------------------
// RX-task: ta emot meddelanden
// ----------------------------------------------------

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
            0
        );

        if (err == ESP_OK)
        {
            if (data.size != sizeof(msg))
            {
                ESP_LOGW(TAG_RX,
                         "Felaktig storlek: mottog %u, förväntade %u",
                         (unsigned)data.size,
                         (unsigned)sizeof(msg));
                continue;
            }

            int64_t rx_time =
                esp_mesh_get_tsf_time();

            int64_t latency_us =
                rx_time - msg.timestamp;

            switch (msg.msg_type)
            {
                case MSG_HEARTBEAT:
                {
                    ESP_LOGI(TAG_RX,
                             "Heartbeat från nod %lu "
                             "(latens: %lld us) "
                             "personantal %d",
                             (unsigned long)msg.node_id,
                             (long long)latency_us,
                             msg.person_count);
                    break;
                }

                case MSG_ALARM_ON:
                {
                    uint32_t sender_id = msg.node_id;

                    if (sender_id >= MAX_NODE_IDS)
                    {
                        ESP_LOGW(TAG_RX,
                                 "Ogiltigt nod-ID: %lu",
                                 (unsigned long)sender_id);
                        break;
                    }

                    // Ignorera redan registrerade larm
                    if (received_alarm_from_node[sender_id])
                    {
                        break;
                    }

                    received_alarm_from_node[sender_id] = true;

                    if (sender_id < EVAC_NUM_NODES)
                    {
                        node_states[sender_id].status = EVAC_NODE_FIRE;
                    }

                    ESP_LOGW(TAG_RX,
                        "BRANDLARM MOTTAGET från nod %lu!",
                        (unsigned long)sender_id);

                        if (sender_id < EVAC_NUM_NODES)
                        {
                            ESP_LOGW("DIJKSTRA", "Startar ruttberäkning...");
                            calculate_evacuation_route();
                        }

                        if (esp_mesh_is_root())
                        {
                            forward_alarm_to_mesh(&msg, &from);
                        }

                            break;
                        }

                case MSG_ALARM_OFF:
                {
                    uint32_t sender_id = msg.node_id;

                    if (sender_id < MAX_NODE_IDS)
                    {
                        received_alarm_from_node[sender_id] = false;
                    }

                    if (sender_id < EVAC_NUM_NODES)
                    {
                        node_states[sender_id].status = EVAC_NODE_NORMAL;
                        calculate_evacuation_route();
                    }

                    ESP_LOGI(TAG_RX,
                             "Larm återställt från nod %lu",
                             (unsigned long)sender_id);
                    break;
                }

                default:
                {
                    ESP_LOGW(TAG_RX,
                             "Okänd meddelandetyp %d",
                             (int)msg.msg_type);
                    break;
                }
            }
        }
        else
        {
            ESP_LOGE(TAG_RX,
                     "Mottagning misslyckades: %s",
                     esp_err_to_name(err));
        }
    }
}