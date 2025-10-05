#define LED_PIN 32

void setup() {
  pinMode(LED_PIN, OUTPUT); // ピンを出力に設定
}

void loop() {
  digitalWrite(LED_PIN, LOW); 
  delay(500); 
  digitalWrite(LED_PIN, HIGH);
  delay(500);           
}
