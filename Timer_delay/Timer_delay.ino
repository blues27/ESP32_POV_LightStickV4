#define LED_PIN 32
#define POT_PIN 34

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  analogReadResolution(12);
}

void loop() {
  Serial.printf("time = %d \n\r",millis());
  digitalWrite(LED_PIN, !digitalRead(LED_PIN)); 
  int max_i = random(1,1000);   
  for(int i=0;i<max_i;i++){
    int analogValue = analogRead(POT_PIN);
  }
  delay(100); 
}
