idf_component_register(SRCS "simple_stepper_main.cpp" "simple_stepper_ota.cpp" "simple_stepper_wifi.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES
                        esp_driver_gpio
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
