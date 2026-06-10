#include "ManagerWifi.h"

static const char* TAG = "ManagerWifi";

// Wifi group definitions
static EventGroupHandle_t wifi_event_group = nullptr;
static constexpr int WIFI_CONNECTED_BIT     = BIT0;
static constexpr int WIFI_FAIL_BIT          = BIT1;
static constexpr int MAX_RETRY              = 15;
static int retry_num = 0;

// Wifi status flags definitions
static bool netif_ready = false;
static bool wifi_ready  = false;

static void wifi_event_handler(void*, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (retry_num < MAX_RETRY) {
            esp_wifi_connect();
            retry_num++;
            ESP_LOGI(TAG, "Retrying WiFi connection...");
        } else {
            xEventGroupSetBits(wifi_event_group, WIFI_FAIL_BIT);
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        auto* event = static_cast<ip_event_got_ip_t*>(event_data);
        ESP_LOGI(TAG, "IP: " IPSTR, IP2STR(&event->ip_info.ip));
        retry_num = 0;
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

ManagerWifi::ManagerWifi()
{

}

void ManagerWifi::init()
{
    initNvs();
}

void ManagerWifi::initNvs()
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}

bool ManagerWifi::hasSavedCredentials()
{
    Credentials credentials = loadCredentials();
    return credentials.valid();
}

ManagerWifi::Credentials ManagerWifi::loadCredentials()
{
    Credentials credentials;

    nvs_handle_t nvsh;
    esp_err_t err = nvs_open("config", NVS_READWRITE, &nvsh);
    if (err != ESP_OK) {
        return credentials;
    }

    size_t ssid_len = 0;
    size_t pass_len = 0;

    err = nvs_get_str(nvsh, "wifi_ssid", nullptr, &ssid_len);
    if (err != ESP_OK || ssid_len == 0) {
        nvs_close(nvsh);
        return credentials;
    }

    std::string ssid(ssid_len, '\0');
    err = nvs_get_str(nvsh, "wifi_ssid", ssid.data(), &ssid_len);
    if (err != ESP_OK) {
        nvs_close(nvsh);
        return credentials;
    }
    if (!ssid.empty() && ssid.back() == '\0') ssid.pop_back();

    err = nvs_get_str(nvsh, "wifi_pass", nullptr, &pass_len);
    if (err == ESP_OK && pass_len > 0) {
        std::string pass(pass_len, '\0');
        err = nvs_get_str(nvsh, "wifi_pass", pass.data(), &pass_len);
        if (err == ESP_OK) {
            if (!pass.empty() && pass.back() == '\0') pass.pop_back();
            credentials.password = pass;
        }
    }

    credentials.ssid = ssid;
    nvs_close(nvsh);
    return credentials;
}

void ManagerWifi::saveCredentials(const Credentials& credentials)
{
    nvs_handle_t nvsh;
    ESP_ERROR_CHECK(nvs_open("config", NVS_READWRITE, &nvsh));
    ESP_ERROR_CHECK(nvs_set_str(nvsh, "wifi_ssid", credentials.ssid.c_str()));
    ESP_ERROR_CHECK(nvs_set_str(nvsh, "wifi_pass", credentials.password.c_str()));
    ESP_ERROR_CHECK(nvs_commit(nvsh));
    nvs_close(nvsh);
}

void ManagerWifi::clearCredentials()
{
    nvs_handle_t nvsh;
    if (nvs_open("config", NVS_READWRITE, &nvsh) == ESP_OK) {
        nvs_erase_key(nvsh, "wifi_ssid");
        nvs_erase_key(nvsh, "wifi_pass");
        nvs_commit(nvsh);
        nvs_close(nvsh);
    }
}

std::string ManagerWifi::readLine(const char* prompt, bool allow_empty)
{
    char buff[128];

    while (true) {
        printf("\n%s", prompt);
        fflush(stdout);

        if (fgets(buff, sizeof(buff), stdin) == NULL) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        size_t len = std::strlen(buff);
        while (len > 0 && (buff[len - 1] == '\n' || buff[len - 1] == '\r')) {
            buff[--len] = '\0';
        }

        if (allow_empty || len > 0) {
            return std::string(buff);
        }
    }
}

ManagerWifi::Credentials ManagerWifi::serialProvisioning()
{
    Credentials credentials;

    printf("\n=== USB WIFI CONFIG MODE ===\n");
    credentials.ssid = readLine("SSID: ");
    credentials.password = readLine("Password: ", true);

    std::string save = readLine("Save inside NVS? [y/N]: ", true);
    if (!save.empty() && (save[0] == 'y' || save[0] == 'Y')) {
        saveCredentials(credentials);
        printf("Credentials stored\n");
    }

    return credentials;
}

bool ManagerWifi::connect(const Credentials& credentials)
{
    if (!credentials.valid()) {
        return false;
    }

    wifi_event_group = xEventGroupCreate();

    if (!netif_ready) {
        ESP_ERROR_CHECK(esp_netif_init());
        ESP_ERROR_CHECK(esp_event_loop_create_default());
        esp_netif_create_default_wifi_sta();
        netif_ready = true;
    }

    if (!wifi_ready) {
        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_wifi_init(&cfg));
        wifi_ready = true;
    }

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, nullptr, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, nullptr, &instance_got_ip));

    wifi_config_t wifi_config = {};
    std::strncpy(reinterpret_cast<char*>(wifi_config.sta.ssid),
                 credentials.ssid.c_str(),
                 sizeof(wifi_config.sta.ssid));
    std::strncpy(reinterpret_cast<char*>(wifi_config.sta.password),
                 credentials.password.c_str(),
                 sizeof(wifi_config.sta.password));

    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.sta.pmf_cfg.capable = true;
    wifi_config.sta.pmf_cfg.required = false;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    EventBits_t bits = xEventGroupWaitBits(
        wifi_event_group,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
        pdTRUE,
        pdFALSE,
        pdMS_TO_TICKS(20000));

    bool ret = (bits & WIFI_CONNECTED_BIT);

    esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip);
    esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id);
    vEventGroupDelete(wifi_event_group);
    wifi_event_group = nullptr;

    return ret;
}