#include "u8g2.h"
#include "u8g2_hal_espidf.hpp"

#include "driver/i2c_master.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <cstring>
#include "esp_log.h"
static char TAG[] = "demo";


/// You should adjust these for your ESP32xx chip/board and OLED display(s)

// #define I2C_PORT LP_I2C_NUM_0 // would require SDA on GPIO_NUM_6 and SCL on GPIO_NUM_7
#define I2C_PORT -1 // let ESP-IDF decide for you.
#define SDA_PIN GPIO_NUM_20
#define SCL_PIN GPIO_NUM_19

// slave address of the SSD1315 OLED driver
/// @note 7-bit number (no!!! left shifting)
#define OLED_I2C_ADDR1 0x3C
/// @note Please comment this out if it's not avilable!
// #define SECOND_OLED_IS_AVAILABLE 
#define OLED_I2C_ADDR2 0x3D // requires tying D/C# to Vcc
// max: 400 kHz for SSD1315
#define OLED_I2C_HZ 300000

static u8g2_t panel1; // for display1
static u8g2_hal::U8g2_Hal_I2C u8g2hal1;
/// @note I attempted to use C++ lambda and std::bind but
///   failed to convert it into plain C function pointer,
///   so this boilerplate is the last resort.
static uint8_t disp1_commCb(u8x8_t *a, uint8_t b, uint8_t c, void *d)
{
  return u8g2hal1.i2c_byte_cb(a, b, c, d);
}
static uint8_t disp1_gpioCb_(u8x8_t *a, uint8_t b, uint8_t c, void *d)
{
  return u8g2hal1.gpio_and_delay_cb(a, b, c, d);
}

static u8g2_t panel2; // for display2
static u8g2_hal::U8g2_Hal_I2C u8g2hal2;
/// @note I attempted to use C++ lambda and std::bind but
///   failed to convert it into plain C function pointer,
///   so this boilerplate is the last resort.
static uint8_t disp2_commCb(u8x8_t *a, uint8_t b, uint8_t c, void *d)
{
  return u8g2hal2.i2c_byte_cb(a, b, c, d);
}
static uint8_t disp2_gpioCb_(u8x8_t *a, uint8_t b, uint8_t c, void *d)
{
  return u8g2hal2.gpio_and_delay_cb(a, b, c, d);
}


extern "C" void app_main(void);
  
void init_u8g2_hal();
void init_u8g2_panel();
// helper function
void resetAllPixels(u8g2_t *pPanel, bool on, uint16_t w = 128, uint16_t h = 64);

/******************************
 *  Function definition
 *******************************/

void app_main(void)
{
  init_u8g2_hal();
  init_u8g2_panel();

  // test if all pixels are working
  ESP_LOGI(TAG, "Switching all pixels on");
  resetAllPixels(&panel1, true, 128, 64);
  resetAllPixels(&panel2, true, 128, 64);
  u8g2_SendBuffer(&panel1);
  u8g2_SendBuffer(&panel2);
  vTaskDelay(pdMS_TO_TICKS(1000));
  // leave the right and bottom edges lit up (just for fun)
  ESP_LOGI(TAG, "Switching most pixels off");
  resetAllPixels(&panel1, false, 128-8, 64-8);
  resetAllPixels(&panel2, false, 128-8, 64-8);
  u8g2_SendBuffer(&panel1);
  u8g2_SendBuffer(&panel2);
  vTaskDelay(pdMS_TO_TICKS(200));

  ESP_LOGI(TAG, "Watch the numbers rolling!");
  // cycle through 6 digits, updated at 10 Hz  (the last byte is ASCII terminating NUL)
  char buf[7];
  u8g2_SetFont(&panel1, u8g2_font_spleen16x32_mr);
  u8g2_SetFont(&panel2, u8g2_font_spleen16x32_mr);

  for (uint32_t ii = 0;;)
  {
    if (++ii >= 1000000)
    {
      ii = 0;
    }
    sprintf(buf, "%06ld", ii);

    u8g2_DrawStr(&panel1, 10, 64-10, buf);
    u8g2_SendBuffer(&panel1);

    // print only the last 3 digits
    u8g2_DrawStr(&panel2, 10, 64-10, buf+3);
    u8g2_SendBuffer(&panel2);
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}


void init_u8g2_hal()
{
  i2c_master_bus_handle_t hMaster;
  i2c_master_bus_config_t cfgMaster = {
      .i2c_port = I2C_PORT,
      .sda_io_num = SDA_PIN,
      .scl_io_num = SCL_PIN,
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .glitch_ignore_cnt = 7,
      .intr_priority = 0,
      .trans_queue_depth = 0, // keep this to 0
      .flags = {
          // many OLED display modules already have pullup! 
          // Disable it only if you know what you are doing (i.e. reducing current consumption)
          .enable_internal_pullup = 1, 
          .allow_pd = 0,
      }};
  ESP_ERROR_CHECK(i2c_new_master_bus(&cfgMaster, &hMaster));

  i2c_device_config_t cfgDisplay = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = OLED_I2C_ADDR1, // that's the real thing (ignore the hacky one inside U8G2)
      .scl_speed_hz = OLED_I2C_HZ,
      .scl_wait_us = 1000, // 0 for use the default register value
      .flags =
          {
              .disable_ack_check = false,
          }};

  ESP_ERROR_CHECK(u8g2hal1.begin(hMaster, cfgDisplay));

  cfgDisplay.device_address = OLED_I2C_ADDR2; // alternative address (require hardware changes!)
#ifndef SECOND_OLED_IS_AVAILABLE
  cfgDisplay.flags.disable_ack_check = true; // otherwise, you would see NACK errors
#endif
  ESP_ERROR_CHECK(u8g2hal2.begin(hMaster, cfgDisplay));
}

void init_u8g2_panel()
{
  u8g2_Setup_ssd1306_i2c_128x64_noname_f(
      &panel1, U8G2_R0, disp1_commCb, disp1_gpioCb_);

  u8g2_Setup_ssd1306_i2c_128x64_noname_f(
      &panel2, U8G2_R0, disp2_commCb, disp2_gpioCb_);

  /// @note this is redundant, only meant for the U8G2 HAL, but our HAL doesn't use it.
  // u8x8_SetI2CAddress(&panel1.u8x8, 0x78);

  ESP_LOGI(TAG, "u8g2_InitDisplay");
  u8g2_InitDisplay(&panel1);  // send init sequence to the display, display is in sleep mode after this
  u8g2_InitDisplay(&panel2);

  // required!!!
  ESP_LOGI(TAG, "u8g2_SetPowerSave off");
  u8g2_SetPowerSave(&panel1, 0);  // wake up display
  u8g2_SetPowerSave(&panel2, 0);

}

void resetAllPixels(u8g2_t *pPanel, bool on, uint16_t w, uint16_t h)
{
  if (on)
  {
    u8g2_SetDrawColor(pPanel, 1);
    u8g2_DrawBox(pPanel, 0, 0, w, h);
  }
  else
  {
    // u8g2_ClearBuffer(pPanel);

    u8g2_SetDrawColor(pPanel, 0);
    u8g2_DrawBox(pPanel, 0, 0, w, h);
    u8g2_SetDrawColor(pPanel, 1);

  }
}
