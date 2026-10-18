/* Onboard RGB LED only. Nothing touches GPIO 15 or the ring.
 * Cycles R -> G -> B on the two pins S3 devkits commonly use (48, then 38).
 * If this lights, the board is alive and the fault is the ring or its wiring. */
void setup() {}
void loop() {
  neopixelWrite(48, 60, 0, 0); neopixelWrite(38, 60, 0, 0); delay(1000);
  neopixelWrite(48, 0, 60, 0); neopixelWrite(38, 0, 60, 0); delay(1000);
  neopixelWrite(48, 0, 0, 60); neopixelWrite(38, 0, 0, 60); delay(1000);
}
