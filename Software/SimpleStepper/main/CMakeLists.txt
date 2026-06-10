idf_component_register(SRCS "StepperDriverMain.cpp" "ManagerOTA.cpp" "ManagerWifi.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES
                        esp_driver_gpio
                        esp_driver_uart
                        esp_driver_usb_serial_jtag
                        esp_wifi
                        esp_event
                        esp_netif
                        esp_http_server
                        app_update
                        bootloader_support
                        nvs_flash
                        mdns
                        vfs
                       )
