/* s3_song.ino — HOTARU wakes up.
 *
 * An original chime + melody, with the ring lit from the music: hue follows
 * pitch, brightness follows the note's envelope, so the lamp breathes in time.
 *
 * Three things make this sound like music instead of beeps:
 *   ENVELOPE   12 ms attack, 70 ms release. A square-edged note clicks at both
 *              ends -- that click is most of what makes buzzer music nasty.
 *   HARMONICS  sine + 0.35 second + 0.14 third. A pure sine is thin; a little
 *              harmonic content reads as warm.
 *   REST       real gaps between notes. Silence is what gives phrasing.
 */
#include <Adafruit_NeoPixel.h>
#include <driver/i2s_std.h>

#define RATE 22050
Adafruit_NeoPixel ring(16, 15, NEO_GRB + NEO_KHZ800);
i2s_chan_handle_t tx = NULL;
bool ok = false;

// C major
#define C4 261.63f
#define D4 293.66f
#define E4 329.63f
#define F4 349.23f
#define G4 392.00f
#define A4 440.00f
#define B4 493.88f
#define C5 523.25f
#define D5 587.33f
#define E5 659.25f
#define G5 783.99f
#define REST 0.0f

struct Note { float hz; uint16_t ms; };

// "waking up" — a rising figure that settles, then lifts at the end
const Note SONG[] = {
  {C4,150},{E4,150},{G4,150},{C5,300},{REST,80},
  {B4,150},{G4,150},{C5,360},{REST,140},
  {E5,180},{D5,180},{C5,180},{G4,180},{REST,80},
  {A4,150},{B4,150},{C5,420},{REST,200},
  {G4,140},{C5,140},{E5,140},{G5,560},{REST,600},
};
const int N_SONG = sizeof(SONG)/sizeof(SONG[0]);

void ringForNote(float hz, float amp) {
  if (hz <= 0) { ring.clear(); ring.show(); return; }
  // map C4..G5 onto a warm->cool hue sweep
  float t = constrain((hz - C4) / (G5 - C4), 0.f, 1.f);
  uint16_t hue = (uint16_t)(6000 + t * 34000);          // amber -> cyan
  uint8_t v = (uint8_t)(constrain(amp, 0.f, 1.f) * 255);
  int lit = 3 + (int)(t * 13);
  for (int i = 0; i < 16; i++)
    ring.setPixelColor(i, i < lit ? ring.ColorHSV(hue, 210, v) : 0);
  ring.show();
}

void playNote(float hz, int ms) {
  int total = (int)((long)RATE * ms / 1000);
  int atk = RATE * 12 / 1000, rel = RATE * 70 / 1000;
  if (atk + rel > total) { atk = total/4; rel = total/4; }
  static float ph = 0;
  int16_t blk[256]; size_t w;
  float inc = 2.0f * PI * hz / RATE;
  int done = 0;
  while (done < total) {
    int n = min(256, total - done);
    float envAvg = 0;
    for (int i = 0; i < n; i++) {
      int pos = done + i;
      float env = 1.0f;
      if (pos < atk)              env = (float)pos / atk;
      else if (pos > total - rel) env = (float)(total - pos) / rel;
      envAvg += env;
      float s = 0;
      if (hz > 0) s = sinf(ph) + 0.35f*sinf(2*ph) + 0.14f*sinf(3*ph);
      blk[i] = (int16_t)(s * env * 9000);
      ph += inc; if (ph > 2*PI) ph -= 2*PI;
    }
    i2s_channel_write(tx, blk, n*2, &w, 400);
    ringForNote(hz, envAvg / n);
    done += n;
  }
}

void setup() {
  Serial.begin(115200); delay(1200);
  ring.begin(); ring.setBrightness(60); ring.clear(); ring.show();
  Serial.println("\nHOTARU wakes up.");

  i2s_chan_config_t cc = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
  if (i2s_new_channel(&cc, &tx, NULL) != ESP_OK) { Serial.println("i2s new FAILED"); return; }
  i2s_std_config_t sc = {
    .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(RATE),
    .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
    .gpio_cfg = { .mclk = I2S_GPIO_UNUSED, .bclk = (gpio_num_t)13, .ws = (gpio_num_t)14,
                  .dout = (gpio_num_t)21, .din = I2S_GPIO_UNUSED,
                  .invert_flags = { false, false, false } },
  };
  if (i2s_channel_init_std_mode(tx, &sc) != ESP_OK) { Serial.println("i2s init FAILED"); return; }
  if (i2s_channel_enable(tx) != ESP_OK) { Serial.println("i2s enable FAILED"); return; }
  ok = true;
  Serial.println("playing.");
}

void loop() {
  if (!ok) { delay(1000); return; }
  for (int i = 0; i < N_SONG; i++) playNote(SONG[i].hz, SONG[i].ms);
  ring.clear(); ring.show();
  delay(1500);
}
