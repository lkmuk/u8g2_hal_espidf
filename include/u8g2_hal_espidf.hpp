#pragma once

#include "u8g2.h"

#include "driver/gpio.h"
#include "driver/i2c_master.h"

#include <vector>
#include <stdint.h>

namespace u8g2_hal
{
  struct Display_aux_pins
  {
    gpio_num_t reset = GPIO_NUM_NC;
    gpio_num_t dc = GPIO_NUM_NC;
  };

  /// @brief I2C payload byte buffer for U8G2
  class ByteBuilder
  {
  public:
    /// @note Set maxByteSize to a reasonably large value. 
    /// Otherwise, heap allocation will happen once/occassionally while updating the display.
    ByteBuilder(uint16_t maxByteSize)
    {
      m_bytes.reserve(maxByteSize);
    }

    /// @brief reset the buffer to an empty state.
    void reset();

    /// @brief concatenate new bytes into the buffer.
    /// @note Don't call this to add the slave address. It will be handled by the ESP-IDF driver
    void add_bytes(const uint8_t *src, uint16_t num);

    /// @brief Actually send the accumulated payload bytes
    /// @param devHandle
    /// @param timeout
    void commit(i2c_master_dev_handle_t devHandle, int timeout) const;

  protected:
    // doesn't store the slave address!
    std::vector<uint8_t> m_bytes;
  };

  class U8g2_Hal_I2C
  {
  public:
    /// @param waittime_ms set the waiting time for an ACK (or I2C resource availability?)
    ///        Anyway, kolban set this to a very conservative 1 second.
    ///        I just find it too high, 100 ms should be more than enough.
    /// @param maxPayloadSize the expected maxium number of payload bytes in
    ///        a single I2C transaction excluding START bit, ACK bit,
    ///        STOP bits, and the slave address byte.
    ///        The intention is to eliminate the need of heap allocation
    ///        while in real-time operation.
    ///        For a full row update for a 128x64 display, this should be
    ///        in the range of 128/8 = 16.
    /// @param aux the auxiliary DC and reset pin numbers (default = not connected)
    /// @note Since some data member (m_hDisp) can only be initialized at runtime,
    ///       full initialization requires also calling @sa begin(...).
    U8g2_Hal_I2C(uint32_t waittime_ms = 100, uint16_t maxPayloadSize = 32, const Display_aux_pins &aux = {})
        : m_payload(maxPayloadSize), m_waittime_ms(waittime_ms), m_auxPins(aux)
    {
    }

    /// @brief The second step of initializing the HAL 
    /// @note needs to be done at runtime. The first step is the constructor.
    /// @param busHandle This must be already initialized! See ESP-IDF doc.
    /// @param cfg
    /// @return ESP_OK if the slave device is successfully added to the bus.
    esp_err_t begin(i2c_master_bus_handle_t busHandle, const i2c_device_config_t &cfg)
    {
      /// @note Certaintly unnecessary if you use this HAL implementation!
      // u8g2_SetI2CAddress(panel, cfg.device_address);

      return i2c_master_bus_add_device(busHandle, &cfg, &m_hDisp);
    }

    esp_err_t deinit()
    {
      return i2c_master_bus_rm_device(m_hDisp);
    }

    uint8_t i2c_byte_cb(
        u8x8_t *u8x8,
        uint8_t msg,
        uint8_t arg_int,
        void *arg_ptr);

    uint8_t gpio_and_delay_cb(
        u8x8_t *u8x8,
        uint8_t msg,
        uint8_t arg_int,
        void *arg_ptr) const;

  protected:
    /// @note deferred initialization: initialization.
    i2c_master_dev_handle_t m_hDisp = nullptr; // handle to the display
    ByteBuilder m_payload;
    uint32_t m_waittime_ms; // timeout for a I2C transaction to start
    Display_aux_pins m_auxPins;
  };
} // namespace u8g2_hal
