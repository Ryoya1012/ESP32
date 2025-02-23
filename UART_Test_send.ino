//Experimental machine Ver.3.0
//UART Communication Test(Send side)
//2025-2-20

#include <HardwareSerial.h>

HardwareSerial mySerial(1); // UART1を使用

void setup() {
  Serial.begin(115200);  // シリアルモニタとの通信を115200ボーレートで開始
  while (!Serial);       // シリアル接続待機

  // UART1の初期化（TX=GPIO32, RX=GPIO33）
  mySerial.begin(9600, SERIAL_8N1, 32, 33);  

  Serial.println("送信側の準備完了");
}

void loop() {
  // シリアルモニタから入力された文字を送信
  if (Serial.available()) {
    // シリアルモニタから1文字受信
    char incomingChar = Serial.read();
    
    // 受信した文字をUART1経由で送信
    mySerial.write(incomingChar);  // UART1送信

    // 送信したデータをシリアルモニタに表示
    Serial.print("送信データ: ");
    Serial.println(incomingChar);
  }
}
