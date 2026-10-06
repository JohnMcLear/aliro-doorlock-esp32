# ST25R300 NFC-A reader over SPI, providing the m5nfc interface that
# esp-matter's Aliro door lock delegate uses. See include/m5nfc.h.
#
# The chip-independent frame logic and the verbatim ISO-DEP headers are shared
# with the ST25R3916 driver next door rather than copied, so a fix there is a
# fix here. Only their include directories are used; st25r3916_spi itself is
# not built into a variant that selects this component.
idf_component_register(
    SRCS "src/st25r300_spi_nfc.cpp" "src/st25r300_hal.cpp"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "src"
                      "../st25r3916_spi/src"
                      "../st25r3916_spi/third_party/yondoor_isodep"
    PRIV_REQUIRES esp_driver_spi esp_driver_gpio esp_timer log freertos heap)
