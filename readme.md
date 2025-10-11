U8G2 HAL for ESP-IDF v5 and beyond
====================================

## Motivation

U8G2 is an excellent cross-platform library for driving various kinds of monochrome displays. It also includes some handy graphics functions. To achieve the cross-platform support, the U8G2 implementation relies on two HAL callbacks, "communication callback" and "gpio and delay callback". 
See https://github.com/olikraus/u8g2/wiki/Porting-to-new-MCU-platform. 
U8G2 already has a HAL for Arduino framework. For other programming frameworks like ESP-IDF, we need a U8G2 HAL.

The HAL implementation of this project enhances the [existing HAL implementation for ESP IDF](https://github.com/mkfrey/u8g2-hal-esp-idf.git) in several dimensions.

1. **Future-proof**: Replaced the deprecated, legacy `i2c.h` APIs with the new ones from `i2c_master.h`. This means you can still use this HAL for future ESP32 chips (which requires newer ESP-IDF releases, which probably will no longer have the legacy API). 

   > In fact, the existing implementation **already cannot** compile for ESP32C6. But the root cause of this issue is its lack of configurability. See [^1] for further elaboration.

2. **Configurability**: The goal is to let you configure the I2C master-slave connection as if you were directly using the ESP-IDF's `i2c_master.h` API. With this new HAL implementation, you can easily configure things like

   * which I2C port to use (perhaps the LP_I2C on ESP32C6)
   * I2C clock speed (for each slave device) 
   * Whether to enable the MCU-side pullup switch

3. **Allow multiple displays**: Unlike the previous implementation, this implementation allows you to have multiple displays driven by U8G2 at the same time, perhaps on the same serial bus.

4. **Allow safe resource sharing**: I2C bus has always been designed to support multiple slaves, 
so a good U8G2 HAL implementation shall support multiple displays safe sharing the same bus. 
It's unclear whether the previous implementation achieves this.

## Minimal working examples

Currently, this implementation only supports hardware I2C.
Btw it uses C++ for more better maintainability.
See the examples below:

- [x] [Driving a single display](./examples/ssd1315_i2c/readme.md)
- [x] Driving two displays (in the same example above).


## A valid ESP32-C6 patch for the previous implementation

[^1]: The existing implementation hard-coded the choice of I2C port, which is not always valid. For ESP32C6, `SOC_I2C_NUM == 2` but  `I2C_NUM_1` is not defined! You can make the old implementation work on ESP32C6 by patch the line 21 of the header: 

   ```c
   #if SOC_I2C_NUM > 1
   #define I2C_MASTER_NUM I2C_NUM_1
   #else
   #define I2C_MASTER_NUM I2C_NUM_0
   #endif
   ```
   with 
   ```c
   #if SOC_I2C_NUM > 1
   #define I2C_MASTER_NUM I2C_NUM_0 // <-- changed, or use LP_I2C ???
   #else
   #define I2C_MASTER_NUM I2C_NUM_0
   #endif
   ```
