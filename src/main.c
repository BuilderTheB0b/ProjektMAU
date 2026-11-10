/**
 * @file main.c
 * @brief Initiering och start av ESP-MESH-nätverket.
 *
 * Programmet initierar NVS, WiFi och ESP-MESH. Därefter
 * konfigureras mesh-nätverket med nätverks-ID, kanal och
 * routerinställningar. Två FreeRTOS-tasker startas:
 * mesh_tx_task() för sändning av meddelanden och
 * mesh_rx_task() för mottagning av meddelanden.
 *
 * Systemet skriver kontinuerligt ut nätverkets status,
 * inklusive lager, root-status och anslutningsstatus.
 **/

#include "esp_log.h"
#include "esp_mesh.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "mesh_tasks.h"
#include "driver/gpio.h"

#define BUTTON_PIN GPIO_NUM_14

static const char *TAG = "MESH_TEST";

static uint8_t MESH_ID[6] = {0x12,0x34,0x56,0x78,0x9A,0xBC};

void app_main(void)
{
        
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BUTTON_PIN), // Set the pin
        .mode         = GPIO_MODE_INPUT,              // Set as output
        .pull_up_en   = GPIO_PULLUP_ENABLE,           // Disable pull-up
        .pull_down_en = GPIO_PULLDOWN_DISABLE,         // Disable pull-down
        .intr_type    = GPIO_INTR_DISABLE,             // Disable interrupt
    };
    gpio_config(&io_conf);


    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_ERROR_CHECK(esp_mesh_init());

    mesh_cfg_t cfg = MESH_INIT_CONFIG_DEFAULT();

    memcpy(cfg.mesh_id.addr, MESH_ID, 6);

    cfg.channel = 6;
    cfg.mesh_ap.max_connection = 6;

    strcpy((char *)cfg.mesh_ap.password, "ivan1234");

    strcpy((char *)cfg.router.ssid, "Ivans iPhone");
    cfg.router.ssid_len = strlen("Ivans iPhone");
    strcpy((char *)cfg.router.password, "ivan1234");

    ESP_ERROR_CHECK(esp_mesh_set_config(&cfg));
    ESP_ERROR_CHECK(esp_mesh_start());

    ESP_LOGI(TAG, "Creating RX task");
    xTaskCreate(
        mesh_rx_task,
        "mesh_rx",
        4096,
        NULL,
        5,
        NULL);

    ESP_LOGI(TAG, "Creating TX task");

    xTaskCreate(
        mesh_tx_task,
        "mesh_tx",
        4096,
        NULL,
        5,
        NULL);
        while (true)
        {      
            ESP_LOGI(TAG,
                "Layer=%d Root=%d Connected=%d",
                esp_mesh_get_layer(),
                esp_mesh_is_root(),
                esp_mesh_is_root() ||
                esp_mesh_get_layer() > 0);

                vTaskDelay(pdMS_TO_TICKS(5000));
        }
}