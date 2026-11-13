#include <string.h>

#include "driver/gpio.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mesh.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_wifi_default.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "evacuation.h"

#include "mesh_tasks.h"

#define BUTTON_PIN GPIO_NUM_14

#ifndef MESH_NODE_ID
#error "Ange MESH_NODE_ID i PlatformIO build_flags"
#endif

static const char *TAG = "MESH_NO_ROUTER";
static const uint8_t MESH_ID[] = {0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC};

static void mesh_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    switch (event_id) {
    case MESH_EVENT_STARTED:
        ESP_LOGI(TAG, "Mesh startad");
        break;

    case MESH_EVENT_PARENT_CONNECTED: {
        const mesh_event_connected_t *event = event_data;
        ESP_LOGI(TAG, "Ansluten till parent, Layer=%d", event->self_layer);
        break;
    }

    case MESH_EVENT_PARENT_DISCONNECTED: {
        const mesh_event_disconnected_t *event = event_data;
        ESP_LOGW(TAG, "Tappade parent, reason=%d", event->reason);
        break;
    }

    case MESH_EVENT_NO_PARENT_FOUND:
        ESP_LOGW(TAG, "Hittade ingen parent");
        break;

    case MESH_EVENT_CHILD_CONNECTED:
        ESP_LOGI(TAG, "En barnnod anslöt");
        break;

    default:
        break;
    }
}



static void test_dijkstra(void)
{
    // Skapa information om alla 18 noder.
    // Alla börjar som status normal med 0 personer.
    evac_node_state_t states[EVAC_NUM_NODES] = {0};

    // Simulera brand i nod 1.
    //states[1].status = EVAC_NODE_FIRE;

    // Simulera 20 personer i nod 6.
    //states[6].people_count = 20;

    // Beräkna väg från nod 0.
    evacuation_result_t result =
        evacuation_calculate_route(0, states);

    if (result.route_found) {
        ESP_LOGI("DIJKSTRA",
                 "Nästa nod: %d | Utgång: %d | Kostnad: %.1f",
                 result.next_node,
                 result.target_exit,
                 result.total_cost);
    } else {
        ESP_LOGW("DIJKSTRA", "Ingen väg hittades!");
    }
}



void app_main(void)
{
    esp_log_level_set("mesh", ESP_LOG_INFO);
    esp_log_level_set("wifi", ESP_LOG_WARN);
    esp_log_level_set("wifi_init", ESP_LOG_WARN);
    esp_log_level_set("phy_init", ESP_LOG_WARN);


    test_dijkstra();

    const gpio_config_t button_config = {
        .pin_bit_mask = 1ULL << BUTTON_PIN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&button_config));

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_t *mesh_sta_netif = NULL;
    ESP_ERROR_CHECK(
        esp_netif_create_default_wifi_mesh_netifs(&mesh_sta_netif, NULL)
    );

    wifi_init_config_t wifi_config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_config));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_ERROR_CHECK(esp_mesh_init());


    // Ställ in rollen innan mesh-konfigurationen valideras.
ESP_ERROR_CHECK(esp_mesh_fix_root(true));

#if MESH_NODE_ID == 1
    ESP_LOGI(TAG, "Nod %d startar som ROOT", MESH_NODE_ID);
    ESP_ERROR_CHECK(esp_mesh_set_type(MESH_ROOT));
#else
    ESP_LOGI(TAG, "Nod %d startar som CHILD", MESH_NODE_ID);
    // Child-rollen tilldelas vid anslutning
#endif



    ESP_ERROR_CHECK(esp_event_handler_register(
        MESH_EVENT, ESP_EVENT_ANY_ID, mesh_event_handler, NULL
    ));

    ESP_ERROR_CHECK(esp_mesh_set_topology(MESH_TOPO_TREE));
    ESP_ERROR_CHECK(esp_mesh_set_max_layer(6));
    ESP_ERROR_CHECK(esp_mesh_set_ap_authmode(WIFI_AUTH_WPA2_PSK));

    mesh_cfg_t config = MESH_INIT_CONFIG_DEFAULT();
    memcpy(config.mesh_id.addr, MESH_ID, sizeof(MESH_ID));

    config.channel = 6;
    config.mesh_ap.max_connection = 6;
    strcpy((char *)config.mesh_ap.password, "evakuering123");

    // Routerfältet lämnas tomt för routerlös drift.
    ESP_ERROR_CHECK(esp_mesh_set_config(&config));
    ESP_ERROR_CHECK(esp_mesh_start());

    ESP_LOGI(TAG, "Startar RX- och TX-taskar");
    xTaskCreate(mesh_rx_task, "mesh_rx", 4096, NULL, 5, NULL);
    xTaskCreate(mesh_tx_task, "mesh_tx", 4096, NULL, 5, NULL);

    while (true) {
        const int layer = esp_mesh_get_layer();

        ESP_LOGI(TAG,
            "Nod=%d | Layer=%d | Root=%d | ParentConnected=%d",
            MESH_NODE_ID,
            layer,
            esp_mesh_is_root(),
            esp_mesh_get_parent_bssid(NULL) == ESP_OK);

        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}