//Experimental machine Ver.3.0
//UART Communication Test(Receive side)
//2025-2-20

#include <HardwareSerial.h>

HardwareSerial mySerial(1); // UART1を使用

void setup() {
  Serial.begin(115200);
  while (!Serial); // シリアル接続待機

  // UART1の初期化（TX=GPIO32, RX=GPIO33）
  mySerial.begin(9600, SERIAL_8N1, 32, 33);  

  Serial.println("UART受信側の準備完了");
}

void loop() {
  // デバッグ用：available()の状態を確認
  Serial.print("Available: ");
  Serial.println(mySerial.available());  // available()が0でないか確認
  
  if (mySerial.available()) {
   String incomingStream = mySerial.readStringUntil('\n');
    Serial.print("受信データ: ");
    Serial.println(incomingStream);
  } else {
    Serial.println("データ未受信...");  // データが来ていないことの確認
  }
  
  delay(100);  // デバッグ用に少し待つ
}
