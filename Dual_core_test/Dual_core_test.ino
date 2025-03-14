//Dual Core practice 
//Date 2025-03-01

//マルチスレッドのタスクハンドル格納用
//タスクハンドルはアドレスで渡す必要があるため, ここで配列として設定する
TaskHandle_t thp[1]; //マルチスレッドのタスクハンドル格納
//コアを跨いでアクセスできるようにGlobal変数
long a = 0;

//スレッドの設定はここで実施
void setup() {
  Serial.begin( 115200);

  //スレッドを立てることを宣言
  xTaskCreatePinnedToCore( Core0a, "Core0a", 4096, NULL, 3, &thp[1], 0);
  //xTaskCreatePinnedToCore()がスレッドの宣言
  //内容は([タスク名], "[タスク名]", [スタックメモリサイズ(4096 or 8192)], NULL, [タスク優先順位](1-24, 大きいほど優先順位が高い), [宣言したタスクハンドルのポインタ(&thp[0])], [Core ID(0 or 1)])
}

void loop() { //メインCPU(core0)で実行するプログラム
  Serial.println(a);
  delay(100);
}

void Core0a( void *args) //サブCPU(core1)で実施するプログラム
{
  while(1)
  {
    delay(1);
    a++;
  }
}