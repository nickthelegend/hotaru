/* Full diagnostic, now that serial works. Reports the real esp_err_t for I2S
 * instead of asking anyone to count pixels. */
#include <Adafruit_NeoPixel.h>
#include <driver/i2s_std.h>
#include <esp_system.h>

Adafruit_NeoPixel ring(16, 15, NEO_GRB + NEO_KHZ800);

void tryI2S(const char *what, int bclk, int ws, int dout, int din, int bits) {
  i2s_chan_handle_t tx = NULL, rx = NULL;
  i2s_chan_config_t cc = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
  esp_err_t e = i2s_new_channel(&cc, dout >= 0 ? &tx : NULL, din >= 0 ? &rx : NULL);
  if (e != ESP_OK) { Serial.printf("  %-4s new_channel   FAIL %s\n", what, esp_err_to_name(e)); return; }
  i2s_std_config_t sc = {
    .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(16000),
    .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
                  bits == 32 ? I2S_DATA_BIT_WIDTH_32BIT : I2S_DATA_BIT_WIDTH_16BIT,
                  I2S_SLOT_MODE_MONO),
    .gpio_cfg = { .mclk = I2S_GPIO_UNUSED, .bclk = (gpio_num_t)bclk,
                  .ws = (gpio_num_t)ws,
                  .dout = dout >= 0 ? (gpio_num_t)dout : I2S_GPIO_UNUSED,
                  .din  = din  >= 0 ? (gpio_num_t)din  : I2S_GPIO_UNUSED,
                  .invert_flags = { false, false, false } },
  };
  i2s_chan_handle_t h = tx ? tx : rx;
  e = i2s_channel_init_std_mode(h, &sc);
  if (e != ESP_OK) { Serial.printf("  %-4s init_std      FAIL %s\n", what, esp_err_to_name(e)); goto done; }
  e = i2s_channel_enable(h);
  if (e != ESP_OK) { Serial.printf("  %-4s enable        FAIL %s\n", what, esp_err_to_name(e)); goto done; }
  Serial.printf("  %-4s OK  bclk=%d ws=%d dout=%d din=%d %d-bit\n", what, bclk, ws, dout, din, bits);
  if (tx) {
    int16_t blk[256]; float ph=0, inc=2.0f*PI*880.0f/16000.0f; size_t w;
    for (int k=0;k<20;k++){ for(int i=0;i<256;i++){blk[i]=(int16_t)(sinf(ph)*9000);ph+=inc;}
      i2s_channel_write(h, blk, sizeof(blk), &w, 200); }
    Serial.println("       -> tone sent, listen for it");
  }
done:
  if (tx) { i2s_channel_disable(tx); i2s_del_channel(tx); }
  if (rx) { i2s_channel_disable(rx); i2s_del_channel(rx); }
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("\n======== HOTARU diagnostic ========");
  Serial.printf("reset reason: %d   free heap: %lu\n", (int)esp_reset_reason(),
                (unsigned long)ESP.getFreeHeap());

  Serial.println("\n[ring] GPIO 15, 16 LEDs, brightness 40");
  ring.begin(); ring.setBrightness(40);
  for (int i=0;i<16;i++){ ring.clear(); ring.setPixelColor(i,255,255,255); ring.show(); delay(70); }
  for (int i=0;i<16;i++) ring.setPixelColor(i,255,255,255);
  ring.show();
  Serial.println("  ring driven WHITE now -- is it lit?");

  Serial.println("\n[i2s] amp, MAX98357A  bclk13 lrc14 din21");
  tryI2S("amp", 13, 14, 21, -1, 16);
  Serial.println("[i2s] mic, INMP441    sck10  ws11  sd12");
  tryI2S("mic", 10, 11, -1, 12, 32);
  Serial.println("\n==================================");
}

void loop() {
  static uint32_t t = 0;
  if (millis() - t > 3000) { t = millis(); Serial.printf("alive %lus\n", millis()/1000); }
}
