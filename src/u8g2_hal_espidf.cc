#include "u8g2_hal_espidf.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
static char TAG[] = "u8g2_hal";

namespace u8g2_hal
{

  void ByteBuilder::reset()
  {
    m_bytes.clear();
  }

  void ByteBuilder::commit(i2c_master_dev_handle_t devHandle, int timeout) const
  {
    i2c_master_transmit(devHandle, m_bytes.data(), m_bytes.size(), timeout);
  }

  void ByteBuilder::add_bytes(const uint8_t *src, uint16_t num)
  {
    ESP_LOGD(TAG, "adding %d B", num);
    for (uint16_t ii = 0; ii < num; ++ii)
    {
      m_bytes.push_back(src[ii]);
    }

    /// this memcpy style copy can be more efficient in principle,
    /// but U8G2 tends to split up add bytes one by one, so the
    /// performance advantage is likely insignificant, or even worse off.
    /// Since the vanilla version above is more readable, we keep it!
    //
    // m_bytes.insert(m_bytes.end(), src, src+num);
  }

  /////////////////////////////////////////
  ////////////////////////////////////////

  uint8_t U8g2_Hal_I2C::i2c_byte_cb(
      u8x8_t *u8x8,
      uint8_t msg,
      uint8_t arg_int,
      void *arg_ptr)
  {
    switch (msg)
    {
    case U8X8_MSG_BYTE_SET_DC:
      if (m_auxPins.dc != GPIO_NUM_NC)
      {
        gpio_set_level(m_auxPins.dc, arg_int);
      }
      break;

    case U8X8_MSG_BYTE_INIT:
      /// @note called only once
      ESP_LOGD(TAG, ". Already initialized, nothing to do here");
      break;

    case U8X8_MSG_BYTE_SEND:
    {
      // WTF??
      uint8_t *data_ptr = (uint8_t *)arg_ptr;
      ESP_LOG_BUFFER_HEXDUMP(TAG, data_ptr, arg_int, ESP_LOG_VERBOSE);

      m_payload.add_bytes(data_ptr, arg_int);
      break;
    }

    case U8X8_MSG_BYTE_START_TRANSFER:
      ESP_LOGD(TAG, "Start (preparing) a I2C transfer");
      m_payload.reset();
      break;

    case U8X8_MSG_BYTE_END_TRANSFER:
      ESP_LOGD(TAG, "Commit I2C transfer");
      m_payload.commit(m_hDisp, m_waittime_ms);
      break;
    }
    return 0;
  }

  uint8_t U8g2_Hal_I2C::gpio_and_delay_cb(
      u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) const
  {
    switch (msg)
    {
    // Initialize the GPIO and DELAY HAL functions.  If the pins for DC and
    // RESET have been specified then we define those pins as GPIO outputs.
    case U8X8_MSG_GPIO_AND_DELAY_INIT:
    {
      uint64_t bitmask = 0;
      if (m_auxPins.dc != GPIO_NUM_NC)
      {
        bitmask = bitmask | (1ull << m_auxPins.dc);
      }
      if (m_auxPins.reset != GPIO_NUM_NC)
      {
        bitmask = bitmask | (1ull << m_auxPins.reset);
      }

      if (bitmask == 0)
      {
        break;
      }
      gpio_config_t gpioConfig;
      gpioConfig.pin_bit_mask = bitmask;
      gpioConfig.mode = GPIO_MODE_OUTPUT;
      gpioConfig.pull_up_en = GPIO_PULLUP_DISABLE;
      gpioConfig.pull_down_en = GPIO_PULLDOWN_ENABLE;
      gpioConfig.intr_type = GPIO_INTR_DISABLE;
      gpio_config(&gpioConfig);
      break;
    }
    case U8X8_MSG_GPIO_RESET:
      if (m_auxPins.reset != GPIO_NUM_NC)
      {
        gpio_set_level(m_auxPins.reset, arg_int);
      }
      break;
    case U8X8_MSG_GPIO_I2C_CLOCK:
    case U8X8_MSG_GPIO_I2C_DATA:
      ESP_LOGW(TAG, "software I2C not supported");
      break;

      // Delay for the number of milliseconds passed in through arg_int.
    case U8X8_MSG_DELAY_MILLI:
      vTaskDelay(pdMS_TO_TICKS(arg_int));
      break;
    default:
      break;
    }
    return 0;
  }
} // namespace