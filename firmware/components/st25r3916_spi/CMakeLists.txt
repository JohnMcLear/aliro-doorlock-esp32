# ST25R3916 / ST25R3916B NFC-A reader over SPI, providing the m5nfc interface
# that esp-matter's Aliro door lock delegate uses. See include/m5nfc.h.
idf_component_register(
    SRCS "src/st25r3916_spi_nfc.cpp" "src/st25r3916_hal.cpp"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "src" "third_party/yondoor_isodep"
    PRIV_REQUIRES esp_driver_spi esp_driver_gpio esp_timer log freertos heap)
