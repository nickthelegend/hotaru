/* No console, so the ring reports the actual esp_err_t.
 * Bypasses I2SClass and drives the IDF driver directly for TX on 13/14/21.
 *
 * READOUT, repeating:
 *   WHITE pixels = which call failed   1 new_channel  2 init_std  3 enable
 *   RED   pixels = (err & 0xFF)        1 NO_MEM  2 INVALID_ARG  3 INVALID_STATE
 *                                      5 NOT_FOUND  (ESP_ERR_* low byte)
 *   ALL GREEN, no red = every call succeeded and a tone is playing
 */
#include <Adafruit_NeoPixel.h>
#include <driver/i2s_std.h>
Adafruit_NeoPixel ring(16, 15, NEO_GRB + NEO_KHZ800);

int failStep = 0;
esp_err_t failErr = ESP_OK;
i2s_chan_handle_t tx = NULL;

void show(int n, uint8_t r, uint8_t g, uint8_t b, int ms) {
  ring.clear();
  for (int i = 0; i < n && i < 16; i++) ring.setPixelColor(i, r, g, b);
  ring.show(); delay(ms); ring.clear(); ring.show(); delay(300);
}

void setup() {
  ring.begin(); ring.setBrightness(70); ring.clear(); ring.show();

  i2s_chan_config_t cc = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
  esp_err_t e = i2s_new_channel(&cc, &tx, NULL);
  if (e != ESP_OK) { failStep = 1; failErr = e; return; }

  i2s_std_config_t sc = {
    .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(16000),
    .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                    I2S_SLOT_MODE_MONO),
    .gpio_cfg = {
      .mclk = I2S_GPIO_UNUSED,
      .bclk = (gpio_num_t)13,
      .ws   = (gpio_num_t)14,
      .dout = (gpio_num_t)21,
      .din  = I2S_GPIO_UNUSED,
      .invert_flags = { false, false, false },
    },
  };
  e = i2s_channel_init_std_mode(tx, &sc);
  if (e != ESP_OK) { failStep = 2; failErr = e; return; }

  e = i2s_channel_enable(tx);
  if (e != ESP_OK) { failStep = 3; failErr = e; return; }
}

void loop() {
  if (failStep) {
    show(failStep, 120, 120, 120, 700);        // which call
    show(failErr & 0xFF, 255, 0, 0, 700);      // the error code
    delay(700);
    return;
  }
  // it worked -- green ring and an audible tone
  for (int i = 0; i < 16; i++) ring.setPixelColor(i, 0, 200, 0);
  ring.show();
  int16_t blk[256]; float ph = 0, inc = 2.0f * PI * 880.0f / 16000.0f;
  size_t w;
  for (int k = 0; k < 12; k++) {
    for (int i = 0; i < 256; i++) { blk[i] = (int16_t)(sinf(ph) * 9000); ph += inc; }
    i2s_channel_write(tx, blk, sizeof(blk), &w, 200);
  }
  ring.clear(); ring.show();
  delay(900);
}
