# secret.h is the shared key the boards use: tools/genkey.sh writes it there.
idf_component_register(SRCS "beacon.c"
                    INCLUDE_DIRS "../../firmware/main"
                    PRIV_REQUIRES esp_wifi esp_event esp_netif esp_timer nvs_flash mbedtls)
