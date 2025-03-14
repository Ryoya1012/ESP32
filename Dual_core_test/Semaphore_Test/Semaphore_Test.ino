//Dual core Semaphore practice
//Date 2025-03-01
//Point 
/*
・xSemaphoreTake([セマフォハンドル名], [ウェイトタイム]のウェイトタイムはミリ秒単位)
・xSemaphoreGiveFromISR()は他のスレッドが持っているセマフォ権をリリースさせることが出来る
*/

/*

*/

SemaphoreHandle_t xBinarySemaphore; //セマフォの準備(型宣言とセマフォハンドル名)
TaskHandle_t thp[2]; //タスクハンドルのポインタ

void setup() {
  Serial.begin( 115200);
  xBinarySemaphore = xSemaphoreCreateBinary(); //セマフォハンドルを作成

  //コア0を指定してタスク(スレッド)を作成
  xTaskCreatePinnedToCore( Mario, "Mario", 4096, NULL, 2, &thp[0], 0);
  //コア1を指定してタスク(スレッド)を作成
  xTaskCreatePinnedToCore( Luige, "Luige", 4096, NULL, 1, &thp[1], 1);
}

void loop() { //メインループ(スレッド)
  int xSemaTama = pdFALSE;
  //このスレッドから他のスレッドが持っているセマフォを強制リリースさせる
  xSemaphoreGiveFromISR( xBinarySemaphore, &xSemaTama);
  Serial.println("mainloop");
  delay(2000); //メインループの待ち時間
}

void Mario( void *args)
{
  bool xSemaTama; //セマフォ値を受け取るためのローカル変数
  Serial.println("Mario Start.");
  while(1)
  {
    //セマフォ値を受け取る
    xSemaTama = xSemaphoreTake( xBinarySemaphore, 2000);
    //xSemaphoreTake([セマフォハンドル名], [ウェイトタイム])
    if( xSemaTama == pdTRUE)  //セマフォ権を取得できたかの確認. pdTRUEはセマフォ判定専用のTrue
    {
      Serial.println("Mario:take");
      //xSemaphoreGive( xBinarySemaphore); //瀬間p補をリリースする命令
    }else
    {
      Serial.println("Mario:wait");
    }
    delay(1);
  }
}

void Luige( void *args)
{
  bool xSemaTama;
  Serial.println("Luige Start.");
  while(1)
  {
    xSemaTama = xSemaphoreTake(xBinarySemaphore, 2000);
    if(xSemaTama == pdTRUE)
    {
      Serial.println("Luige: Take");
      //xSemaphoreGive(xBinarySemaphore); //セマフォをリリースする命令
    }else
    {
      Serial.println("Luige:wait");
    }
    delay(1);
    }
}