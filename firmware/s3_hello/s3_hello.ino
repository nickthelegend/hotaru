void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("HELLO FROM S3");
}
void loop() {
  Serial.printf("tick %lu\n", millis() / 1000);
  delay(1000);
}
