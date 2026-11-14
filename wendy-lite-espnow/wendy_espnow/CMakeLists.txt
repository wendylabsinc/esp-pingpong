set(srcs "")
if(CONFIG_WENDY_ESPNOW)
    list(APPEND srcs "src/wendy_espnow.c" "src/ens.c")
endif()

idf_component_register(
    SRCS ${srcs}
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "src"
    REQUIRES wendy_pki
    PRIV_REQUIRES freertos esp_timer esp_wifi esp_hw_support mbedtls log heap
)
