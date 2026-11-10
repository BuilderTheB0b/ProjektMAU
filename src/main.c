#include <string.h>
#include "esp_log.h"
#include "esp_mesh.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "mesh_tasks.h"
#include "driver/gpio.h"

#define BUTTON_PIN GPIO_NUM_14

static const char *TAG = "MESH_NO_ROUTER";
static uint8_t MESH_ID[6] = {0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC};

void app_main(void)
{
    esp_log_level_set("wifi", ESP_LOG_WARN);
    esp_log_level_set("wifi_init", ESP_LOG_WARN);
    esp_log_level_set("mesh", ESP_LOG_WARN);
    esp_log_level_set("phy_init", ESP_LOG_WARN);
    // Konfigurera larmknappen på GPIO 14
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BUTTON_PIN),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_ERROR_CHECK(esp_mesh_init());

    mesh_cfg_t cfg = MESH_INIT_CONFIG_DEFAULT();
    memcpy(cfg.mesh_id.addr, MESH_ID, 6);

    cfg.channel = 6;
    cfg.mesh_ap.max_connection = 6;
    strcpy((char *)cfg.mesh_ap.password, "evakuering123");

    strcpy((char *)cfg.router.ssid, "NO_ROUTER");
    cfg.router.ssid_len = strlen("NO_ROUTER");
    strcpy((char *)cfg.router.password, "12345678");

    ESP_ERROR_CHECK(esp_mesh_set_config(&cfg));
    ESP_ERROR_CHECK(esp_mesh_set_topology(MESH_TOPO_TREE));
    ESP_ERROR_CHECK(esp_mesh_set_max_layer(6));

    // Styr startroll utifrån MESH_NODE_ID men tillåt dynamisk failover
    #if (MESH_NODE_ID == 1)
        ESP_LOGI(TAG, "Startar som initiell ROOT");
        ESP_ERROR_CHECK(esp_mesh_set_type(MESH_ROOT));
    #else
        ESP_LOGI(TAG, "Startar som CHILD (söker efter Root)");
        ESP_ERROR_CHECK(esp_mesh_set_type(MESH_IDLE));    
    #endif

    // Tillåt att rollen kan ändras dynamiskt vid bortfall
    ESP_ERROR_CHECK(esp_mesh_fix_root(false));

    ESP_ERROR_CHECK(esp_mesh_start());

    ESP_LOGI(TAG, "Startar RX task");
    xTaskCreate(mesh_rx_task, "mesh_rx", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "Startar TX task");
    xTaskCreate(mesh_tx_task, "mesh_tx", 4096, NULL, 5, NULL);

    while (true)
    {      
        ESP_LOGI(TAG,
            "Layer=%d Root=%d Ansluten=%d",
            esp_mesh_get_layer(),
            esp_mesh_is_root(),
            (esp_mesh_is_root() || esp_mesh_get_layer() > 0));

        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}