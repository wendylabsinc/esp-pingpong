idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES wendy_core
                    PRIV_REQUIRES wendy_pki wendy_net_peer wendy_ble wendy_com wendy_wifi esp_wifi esp_driver_i2s esp_driver_gpio
                                  esp_driver_rmt esp_timer nvs_flash)
