#include <nvs_flash.h>
#include <esp_event.h>
#include <esp_wifi.h>
#include <esp_log.h>
#include <esp_netif.h>
#include <esp_netif_ip_addr.h>


void init_nvs() {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}


void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    }

    if(event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI("TEST_ESP32", "Got IP: " IPSTR,  IP2STR(&event->ip_info.ip));
    }
}

void app_main(void)
{
    init_nvs();

    esp_netif_init();

    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    wifi_sta_config_t cfg_sta = {
        .ssid = "levi",
        .password = "2005050521",
    };
    //  而直接将wifi_sta_config_t(或指针)转为wifi_config_t(或指针)是GCC的拓展语法，如下
    esp_wifi_set_config(WIFI_IF_STA, (wifi_config_t *) &cfg_sta);

    esp_wifi_set_mode(WIFI_MODE_STA);

    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL, NULL);

    esp_wifi_start();

}

