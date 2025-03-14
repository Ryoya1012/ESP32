#include <Arduino.h>
#include <HX711_asukiaaa.h>

int pinsDout[] = {21, 32};
const int numPins = sizeof(pinsDout) / sizeof(pinsDout[0]);
int pinSlk = 22;

HX711_asukiaaa::Reader reader(pinsDout, numPins, pinSlk);

#define LOAD_CELL_RATED_VOLT 0.0075f
#define LOAD_CELL_RATED_GRAM 10000.0f

#define HX711_R1 1000.0
#define HX711_R2 1000.0

HX711_asukiaaa::Parser parser(LOAD_CELL_RATED_VOLT, LOAD_CELL_RATED_GRAM, HX711_R1, HX711_R2);
float offsetGrams[numPins];
float calibrationFactors[numPins] = {1, 1};

// 時間管理
unsigned long prevMicros = 0;
const unsigned long intervalMicros = 5000; // 5ms (200Hz)

void setup() {
  Serial.begin(115200);  // シリアル通信の設定

  // ロードセルの初期化をコア0で行う
  xTaskCreatePinnedToCore(loadCellTask, "LoadCellTask", 2048, NULL, 1, NULL, 0);

  // シリアル出力の処理をコア1で行う
  xTaskCreatePinnedToCore(serialOutputTask, "SerialOutputTask", 2048, NULL, 1, NULL, 1);
}

void loop() {
  // メインループは空でもOK
}

void loadCellTask(void *pvParameters) {
  reader.begin();

  while (reader.read() != 0) {
    delay(500);
  }

  for (int i = 0; i < reader.doutLen; i++) {
    offsetGrams[i] = parser.parseToGram(reader.values[i]);
  }

  // ロードセルの読み取りを行う
  while (true) {
    unsigned long currentMicros = micros();
    if ( currentMicros - prevMicros >= intervalMicros) {
      prevMicros = currentMicros;
      reader.read();
      // ロードセルデータ処理はここで行う
    }
  }
}

void serialOutputTask(void *pvParameters) {
  while (true) {
    // シリアル出力処理
    String output = "";
    for (int i = 0; i < reader.doutLen; ++i) {
      float gram = parser.parseToGram(reader.values[i]) - offsetGrams[i];
      float newtons = gram * 0.00981 * calibrationFactors[i];
      output += "sensor" + String(i) + ":" + String(newtons, 2) + "N,";
    }
    output.remove(output.length() - 1); // 最後のカンマを削除
    Serial.println(output);

    // シリアル出力間隔の調整（10msごとに出力）
    delay(10);  // 10msごとに出力
  }
}
