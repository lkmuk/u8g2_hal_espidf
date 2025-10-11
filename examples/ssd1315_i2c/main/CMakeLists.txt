idf_component_register(SRCS "demo_i2c_ssd1315.cc"
                    PRIV_REQUIRES 
                        esp_driver_gpio
                        esp_driver_i2c
                        u8g2
                        u8g2_hal_espidf
                    INCLUDE_DIRS ".")