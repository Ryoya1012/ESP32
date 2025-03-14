//Experimental machine Ver.3.0
//2025-02-23
//Light LED test

#include <Arduino.h>

int LED1 = 33;  // LEDを7番ピンに接続
int LED2 = 23;  // LEDを7番ピンに接続
int LED3 = 22;  // LEDを7番ピンに接続
int LED4 = 21;  // LEDを7番ピンに接続

void setup() {
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(LED4, OUTPUT);
}

void loop() {
  digitalWrite(LED1, HIGH); // 点灯
  delay(10);
  digitalWrite(LED1, LOW);  // 消灯
  delay(10);
  digitalWrite(LED2, HIGH); // 点灯
  delay(10);
  digitalWrite(LED2, LOW);  // 消灯
  delay(10);
  digitalWrite(LED3, HIGH); // 点灯
  delay(10);
  digitalWrite(LED3, LOW);  // 消灯
  delay(10);
  digitalWrite(LED4, HIGH); // 点灯
  delay(10);
  digitalWrite(LED4, LOW);  // 消灯
  delay(10);
}
