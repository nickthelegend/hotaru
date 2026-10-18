/* Amp alone, on the AUTO-allocated I2S port instead of an explicit I2S_NUM_1.
 * Ring GREEN = amp begin() succeeded, BLUE = still failing.
 * Then a repeating chirp so you can hear it. */
#include <Adafruit_NeoPixel.h>
#include <ESP_I2S.h>
#define RATE 16000
Adafruit_NeoPixel ring(16, 15, NEO_GRB + NEO_KHZ800);
I2SClass amp;                       // default ctor -> I2S_NUM_AUTO
bool ok = false;

void fill(uint8_t r,uint8_t g,uint8_t b){for(int i=0;i<16;i++)ring.setPixelColor(i,r,g,b);ring.show();}

void tone_(uint32_t hz,uint32_t ms,float v=0.30f){
  if(!ok)return;
  int total=(int)((uint64_t)RATE*ms/1000); int16_t blk[256];
  float ph=0,inc=2.0f*PI*hz/RATE; int done=0;
  while(done<total){int n=min(256,total-done);
    for(int i=0;i<n;i++){blk[i]=(int16_t)(sinf(ph)*32767.0f*v);ph+=inc;if(ph>2*PI)ph-=2*PI;}
    amp.write((const uint8_t*)blk,n*2); done+=n;}
}

void setup(){
  ring.begin(); ring.setBrightness(70); fill(0,0,0);
  amp.setPins(13, 14, 21, -1);      // BCLK, LRC, DIN, no din
  ok = amp.begin(I2S_MODE_STD, RATE, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO);
  for(int k=0;k<3;k++){ fill(ok?0:0, ok?255:0, ok?0:255); delay(350); fill(0,0,0); delay(200);} 
}
void loop(){
  tone_(660,150); tone_(990,180);
  fill(ok?0:0, ok?40:0, ok?0:40);
  delay(1200);
}
