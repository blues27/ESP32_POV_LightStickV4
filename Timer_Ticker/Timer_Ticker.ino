#include <Ticker.h>

#define LED_PIN 32
#define POT_PIN 34

Ticker blinker;

void ticker_handler(void){
  Serial.printf("time = %d \n\r",millis());
  digitalWrite(LED_PIN, !digitalRead(LED_PIN)); 
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  analogReadResolution(12);
  blinker.attach(0.1, ticker_handler);
}

void loop() {
  int max_i = random(1,1000);   
  for(int i=0;i<max_i;i++){
    int analogValue = analogRead(POT_PIN);
  }
  delay(100); 
}
