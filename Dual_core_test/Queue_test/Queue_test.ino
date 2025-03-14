//Dual core Queue test
//Date 2025-03-01

TaskHandle_t thp[2]; //マルチスレッドのタスクハンドル格納
QueueHandle_t xQueue_1; //Queueを使いう準備[キューハンドル名]

void setup() {
  Serial.begin( 115200);

  xQueue_1 = xQueueCreate( 10, 16);
  //xQueuCreate( [待ち行列の席数], [１席当たりのバイト数]);

  //スレッドの準備
  xTaskCreatePinnedToCore( Core0a, "Core0a", 4096, NULL, 2, &thp[0], 0);
  xTaskCreatePinnedToCore( Core1a, "Core1a", 4096, NULL, 1, &thp[1], 0);
}

void loop() {
  Serial.println("[main] loop");
  delay(100);
}

void Core1a(void *args)
{
  int a = 0; //送信データの数
  while(1)
  {
    xQueueSend( xQueue_1, &a, 0);
    //xQueueSend([キューハンドル名], [送る値], [待ち行列の席が空きまで待つ時間])
    a++;
    delay(1000);
  }
}

void Core0a( void *args)
{
  int b = 0; //データ受信用の変数
  while(1)
  {
    /*メッセージ受信待ち*/
    xQueueReceive( xQueue_1, &b, portMAX_DELAY);
    //xQueueReceive([キューハンドル名], [データを受信するアドレス], [キュー空を待つ最大時間. portMAX_DELAYで永久待ち])
    Serial.print("[Core0a] xQueueReceive:");
    Serial.println(b);
    delay(1);
  }
}