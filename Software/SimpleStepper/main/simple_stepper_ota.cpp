#include "simple_stepper_ota.h"

static const char* TAG = "ManagerOTA";

// LED blink definitions
#define OTA_BLINK_PERIOD_MS 200
#define OTA_BLINK_COUNT     5

// OTA buffer size definition
#define OTA_BUFFSIZE 1024

// OTA mDNS server nstring definition
#define OTA_MDNS_HOST_NAME "StepperDriverOTA"

static const char *UPLOAD_HTML = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <title>ESP32 OTA Update</title>
</head>
<body>
    <h2>ESP32 OTA Update</h2>
    <form method="POST" action="/update" enctype="application/octet-stream">
        <p>Select firmware .bin file:</p>
        <input type="file" id="fileInput" />
        <button type="button" onclick="uploadFile()">Upload</button>
    </form>
    <pre id="status"></pre>

    <script>
        async function uploadFile() {
            const file = document.getElementById('fileInput').files[0];
            const status = document.getElementById('status');

            if (!file) {
                status.textContent = "Please select a file.";
                return;
            }

            status.textContent = "Uploading...";
            try {
                const response = await fetch('/update', {
                    method: 'POST',
                    headers: {
                        'Content-Type': 'application/octet-stream',
                        'X-Filename': file.name
                    },
                    body: file
                });

                const text = await response.text();
                status.textContent = text;
            } catch (e) {
                status.textContent = "Upload failed: " + e;
            }
        }
    </script>
</body>
</html>
)rawliteral";


ManagerOTA::ManagerOTA(int button, int led) : button_gpio(button), led_gpio(led)
{

}

esp_err_t ManagerOTA::startWebServer()
{
    httpd_config_t config   = HTTPD_DEFAULT_CONFIG();
    config.stack_size       = 8192;

    esp_err_t err = httpd_start(&server, &config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server");
        return err;
    }

    httpd_uri_t root = {};
    root.uri        = "/";
    root.method     = HTTP_GET;
    root.handler    = rootGetHandler;
    root.user_ctx   = nullptr;

    httpd_uri_t update = {};
    update.uri      = "/update";
    update.method   = HTTP_POST;
    update.handler  = updatePostHandler;
    update.user_ctx = nullptr;

    httpd_register_uri_handler(server, &root);
    httpd_register_uri_handler(server, &update);

    ESP_LOGI(TAG, "OTA web server started");
    return ESP_OK;
}

void ManagerOTA::stopWebServer()
{
    if (server != nullptr) {
        httpd_stop(server);
        server = nullptr;
    }
}

void ManagerOTA::startServiceMDNS()
{
    ESP_ERROR_CHECK(mdns_init());
    ESP_ERROR_CHECK(mdns_hostname_set(OTA_MDNS_HOST_NAME));
    ESP_ERROR_CHECK(mdns_instance_name_set("Stepper Driver ESP32-C6 OTA Web Server"));
    ESP_ERROR_CHECK(mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0));
}

bool ManagerOTA::isOtaButtonPressed()
{
    bool ret = gpio_get_level(static_cast<gpio_num_t>(button_gpio)) == 0;
    if (ret)
        xTaskCreate(&ManagerOTA::blinkTask, "prov_blink", 2048, this, 5, nullptr);
    return ret;
}

void ManagerOTA::blinkTask(void* pvParameters)
{
    auto* self = static_cast<ManagerOTA*>(pvParameters);
    gpio_num_t led = static_cast<gpio_num_t>(self->led_gpio);

    for (int i = 0; i < OTA_BLINK_COUNT; ++i) {
        gpio_set_level(led, 0);
        vTaskDelay(pdMS_TO_TICKS(OTA_BLINK_PERIOD_MS));
        gpio_set_level(led, 1);
        vTaskDelay(pdMS_TO_TICKS(OTA_BLINK_PERIOD_MS));
    }

    gpio_set_level(led, 0);
    vTaskDelete(nullptr);
}

esp_err_t ManagerOTA::rootGetHandler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, UPLOAD_HTML, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

esp_err_t ManagerOTA::updatePostHandler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "OTA upload started, length = %d", req->content_len);

    const esp_partition_t *update_partition = esp_ota_get_next_update_partition(nullptr);
    if (update_partition == nullptr) {
        ESP_LOGE(TAG, "No OTA partition available");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No OTA partition available");
        return ESP_FAIL;
    }

    esp_ota_handle_t ota_handle = 0;
    esp_err_t err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "OTA begin failed: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA begin failed");
        return err;
    }

    char buf[OTA_BUFFSIZE];
    int remaining       = req->content_len;
    int received_total  = 0;
    bool header_checked = false;

    while (remaining > 0) {
        int recv_len = remaining > OTA_BUFFSIZE ? OTA_BUFFSIZE : remaining;
        int ret = httpd_req_recv(req, buf, recv_len);

        if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
            continue;
        }

        if (ret <= 0) {
            ESP_LOGE(TAG, "Upload receive failed");
            esp_ota_abort(ota_handle);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Upload receive failed");
            return ESP_FAIL;
        }

        if (!header_checked) {
            if (ret > (int)(sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t) + sizeof(esp_app_desc_t))) {

                esp_app_desc_t new_app_info;
                std::memcpy(&new_app_info, &buf[sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t)], sizeof(esp_app_desc_t));

                ESP_LOGI(TAG, "New firmware version: %.32s", new_app_info.version);
                header_checked = true;
            }
        }

        err = esp_ota_write(ota_handle, buf, ret);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "OTA write failed: %s", esp_err_to_name(err));
            esp_ota_abort(ota_handle);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA write failed");
            return err;
        }

        remaining -= ret;
        received_total += ret;
        ESP_LOGI(TAG, "Received %d bytes", received_total);
    }

    err = esp_ota_end(ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "OTA end failed: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA end failed");
        return err;
    }

    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "OTA set boot partition failed: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA set boot partition failed");
        return err;
    }

    httpd_resp_sendstr(req, "Firmware uploaded successfully, rebooting...");

    ESP_LOGI(TAG, "OTA successful, rebooting...");
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();

    return ESP_OK;
}
