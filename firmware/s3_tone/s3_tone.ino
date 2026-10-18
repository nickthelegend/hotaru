/* Continuous loud sweeping tone on the MAX98357A. Never stops.
 * Poke at the wiring while it runs -- you'll hear the moment it works.
 * Ring pulses in time with the sweep so you can see it is still running.
 *
 * If this is silent, it is NOT the ESP32: I2S init is checked and reported.
 * Check, in order:
 *   SD pin  -> must be FLOATING. Tied to GND = amplifier shut down.
 *   VIN     -> meter it on the amp board, should be ~5 V
 *   speaker -> on the + and - screw terminals
 */
#include <Adafruit_NeoPixel.h>
#include <driver/i2s_std.h>
Adafruit_NeoPixel ring(16, 15, NEO_GRB + NEO_KHZ800);
i2s_chan_handle_t tx = NULL;
bool ok = false;

void setup() {
  Serial.begin(115200); delay(1500);
  ring.begin(); ring.setBrightness(40);
  Serial.println("\n--- continuous tone test ---");

  i2s_chan_config_t cc = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
  if (i2s_new_channel(&cc, &tx, NULL) != ESP_OK) { Serial.println("new_channel FAILED"); return; }
  i2s_std_config_t sc = {
    .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(16000),
    .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
    .gpio_cfg = { .mclk = I2S_GPIO_UNUSED, .bclk = (gpio_num_t)13, .ws = (gpio_num_t)14,
                  .dout = (gpio_num_t)21, .din = I2S_GPIO_UNUSED,
                  .invert_flags = { false, false, false } },
  };
  if (i2s_channel_init_std_mode(tx, &sc) != ESP_OK) { Serial.println("init FAILED"); return; }
  if (i2s_channel_enable(tx) != ESP_OK) { Serial.println("enable FAILED"); return; }
  ok = true;
  Serial.println("I2S up. Tone is playing CONTINUOUSLY at near-full volume.");
  Serial.println("If silent -> SD pin tied to GND? VIN missing? speaker not on +/-?");
}

void loop() {
  if (!ok) { delay(1000); return; }
  static float hz = 400;
  hz += 60; if (hz > 1600) hz = 400;
  int16_t blk[256]; static float ph = 0; size_t w;
  float inc = 2.0f * PI * hz / 16000.0f;
  for (int k = 0; k < 10; k++) {                      // ~0.16 s per step
    for (int i = 0; i < 256; i++) {
      blk[i] = (int16_t)(sinf(ph) * 28000);           // near full scale
      ph += inc; if (ph > 2*PI) ph -= 2*PI;
    }
    i2s_channel_write(tx, blk, sizeof(blk), &w, 300);
  }
  int n = (int)((hz - 400) / 1200.0f * 16);
  for (int i = 0; i < 16; i++) ring.setPixelColor(i, i < n ? ring.Color(0,180,255) : 0);
  ring.show();
}
