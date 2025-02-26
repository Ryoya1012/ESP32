#include <Arduino.h>
#include <HX711_asukiaaa.h>  // 正しいライブラリ名を確認

int pinsDout[] = { 21, 32};
const int numPins = sizeof( pinsDout) / sizeof( pinsDout[0]);
int pinSlk = 22;

HX711_asukiaaa::Reader reader(pinsDout, numPins, pinSlk);

#define LOAD_CELL_RATED_VOLT 0.0075f //1.5mV/V
#define LOAD_CELL_RATED_GRAM 10000.0f  // 10kN

//入力抵抗調整
//-------------------------------------------------------------//
// 1000Ω > 安定性重視
// 1500Ω > 高精度向け(若干ノイズが減る可能性あり)
// なし(0Ω) > そのまま使えるがノイズに弱いことがある
#define HX711_R1 1000.0
//フィルタ抵抗
//-------------------------------------------------------------//
// 4700Ω > 応答速度が速くなる(ノイズ耐性は低下)
// 8200Ω > 標準設定(バランスが良い)
// 10000Ω > ノイズ耐性が上がるが応答が少し遅くなる
#define HX711_R2 1000.0

HX711_asukiaaa::Parser parser(LOAD_CELL_RATED_VOLT, LOAD_CELL_RATED_GRAM, HX711_R1, HX711_R2);
float offsetGrams[numPins];

float calibrationFactors[numPins] = { 1, 1}; //各センサに対する校正係数

void setup() {
  Serial.begin(115200);
  Serial.println("Start");
  reader.begin();

  while( reader.read() != 0)
  {
    Serial.println("Failed initial reading. Retry.");
    delay(500);
  }
  for( int i = 0; i < reader.doutLen; i++)
  {
    offsetGrams[i] = parser.parseToGram(reader.values[i]);
  }
}

void loop() {
  reader.read();
  String output = "";
  for (int i = 0; i < reader.doutLen; ++i) {
    float gram = parser.parseToGram(reader.values[i]) - offsetGrams[i];
    float newtons = gram * 0.00981 * calibrationFactors[i];  // 1g = 0.00981N

    output += "sensor" + String(i) + ":" + String(newtons, 2) + "N,";
  }
  output.trim(); //最後の余分な空白を削除
  output.remove( output.length() - 1); //最後のカンマを削除
  Serial.println(output);
  delay(10);
}
