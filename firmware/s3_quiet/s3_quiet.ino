/* Silence. No I2S, no ring, nothing driven. Leaves the board idle so the
 * speaker and LEDs stay quiet and dark until you flash something else. */
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("idle - nothing playing");
}
void loop() { delay(1000); }
