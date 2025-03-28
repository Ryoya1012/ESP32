//Eveprimental machine Ver.3.0
//slave (Driving motor control and Reading Tension)
//SPI mastre
//Author Ryoya SATO
//2025-02-25

#include <Arduino.h>
#include "driver/twai.h"
#include <Wire.h>
#include <HX711_asukiaaa.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "CAN_library.h"

#define RX_PIN 4
#define TX_PIN 5

int setup_done = 0; //setup()が完了したかどうかのフラグ

int LED1 = 25;
int LED2 = 13;
int LED3 = 26;
int LED4 = 12;

uint16_t current_1 = 0;
uint16_t processed_current1 = 0;
uint16_t current_2 = 0;
uint16_t processed_current2 = 0;
float current_average = 0;

bool recieved1 = false;
bool recieved2 = false;

int mode = 0;
char lan;

int pinsDout[] = { 21, 32};
const int numPins = sizeof( pinsDout) / sizeof( pinsDout[0]) ;
int pinSlk = 22;
//センサデータを格納するバッファ
float tensionValues[2];
float tensionValues1[2];

HX711_asukiaaa::Reader reader( pinsDout, numPins, pinSlk);
String output = "";

TaskHandle_t thp[2]; //マルチスレッドのタスクハンドル格納

QueueHandle_t tensionQueue; //Queueを使う準備

#define LOAD_CELL_RATED_VOLT 0.0075f
#define LOAD_CELL_RATED_GRAM 10000.0f

//入力抵抗調整
#define HX711_R1 1000.0

//フィルタ抵抗
#define HX711_R2 1000.0

//時間管理
unsigned long prevMicros = 0;
const unsigned long intervalMicros = 10000; //10ms

HX711_asukiaaa::Parser parser( LOAD_CELL_RATED_VOLT, LOAD_CELL_RATED_GRAM, HX711_R1, HX711_R2);
float offsetGrams[numPins];
float calibrationFactors[numPins] = { 1, 1}; //各センサに対する校正係数

//LED 初期化
void initLED()
{
  pinMode( LED1, OUTPUT);
  pinMode( LED2, OUTPUT);
  pinMode( LED3, OUTPUT);
  pinMode( LED4, OUTPUT);
}

// CAN 初期化
void initCAN() {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TX_PIN, (gpio_num_t)RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    
    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        Serial.println("CANドライバのインストール成功");
    } else {
        Serial.println("CANドライバのインストール失敗");
    }
    
    if (twai_start() == ESP_OK) {
        Serial.println("CAN通信開始");
    } else {
        Serial.println("CAN通信開始失敗");
    }
}

void init_HX711()
{
  while( reader.read() != 0)
  {
    Serial.println("Failed initial reading... Retry.");
    delay(500);
  }
  for( int i = 0; i < reader.doutLen; i++)
  {
    offsetGrams[i] = parser.parseToGram( reader.values[i]);
  }
}

void read_Tension()
{
  for( int i = 0; i < reader.doutLen; i++)
  {
    float gram = parser.parseToGram(reader.values[i] - offsetGrams[i]);
    float newtons = gram * 0.00981 * calibrationFactors[i];

    tensionValues[i] = newtons; //配列に格納
    //output += "Tension" + String(i) + ":" + String(newtons, 2) + "N,";
  }
  Serial1.print("sensor0:");
  Serial1.print(tensionValues[0]);
  Serial1.print(",");
  Serial1.print("sensor2:");
  Serial1.println(tensionValues[1]);
  //output.trim();
  //output.remove( output.length() - 1);
  //Serial.println(output);
}

// CAN メッセージ送信
void send_CAN_message(uint32_t id, uint8_t *data, uint8_t len) {
    twai_message_t message;
    message.identifier = id;
    message.extd = 0;
    message.data_length_code = len;
    memcpy(message.data, data, len);
    
    twai_transmit( &message, pdMS_TO_TICKS(10));
}

// CAN メッセージ受信
void receiveCANMessage() {
    twai_message_t message;
    //esp_err_t res = twai_receive( &message, portMAX_DELAY); //受信待機
    //if ( res == ESP_OK) 
    //{
      if( message.identifier == 0x281)
      {
      Serial.println("148");
      //ID_1からの電流値を解析
      current_1 = ( message.data[0] << 5) | message.data[1];
      processed_current1 = current_1; //校正式
      Serial.print(processed_current1);
      }
      else if( message.identifier == 0x290)
      {
        Serial.println("155");
        //ID_2からの電流値を解析
        current_2 = ( message.data[0] << 5) | message.data[1];
        processed_current2 = current_2; //校正式
        Serial.print(processed_current2);
      }
    //}
}

void Predicted_Traction_Model()
{
  float sum = 0, tension_average = 0;
  uint16_t length = 2;
  for( int i = 0; i < length; i++)
  {
    sum += tensionValues[i];
  }
  tension_average = sum / length;
  current_average = ( current_1 + current_2) / 2;
  //駆動トルクへの変換式
}

//個別コマンド
void NMTPRE()
{
  twai_message_t NMTPRE;
  NMTPRE.identifier = 0x00;
  NMTPRE.data_length_code = 2;
  NMTPRE.data[0] = CAN_Commands.PreOP[0];
  NMTPRE.data[1] = CAN_Commands.PreOP[1];
  uint32_t message_id = NMTPRE.identifier;
  send_CAN_message( message_id, NMTPRE.data, NMTPRE.data_length_code);
}

void NMTOP()
{
  twai_message_t NMTOP;
  NMTOP.identifier = 0x00;
  NMTOP.data_length_code = 2;
  NMTOP.data[0] = CAN_Commands.Operation[0];
  NMTOP.data[1] = CAN_Commands.Operation[1];
  uint32_t message_id = NMTOP.identifier;
  send_CAN_message( message_id, NMTOP.data, NMTOP.data_length_code);
}

void THE_P_Read1()
{
  twai_message_t THE_P_Read1;
  THE_P_Read1.identifier = Motor1_IDs.Tx_PDO02;
  THE_P_Read1.data_length_code = 4;
  THE_P_Read1.data[0] = CAN_Commands.Read_Current1[0];
  THE_P_Read1.data[1] = CAN_Commands.Read_Current1[1];
  THE_P_Read1.data[2] = CAN_Commands.Read_Current1[2];
  THE_P_Read1.data[3] = CAN_Commands.Read_Current1[3];
  uint32_t message_id = THE_P_Read1.identifier;
  send_CAN_message( message_id, THE_P_Read1.data, THE_P_Read1.data_length_code);
}

void THE_C_Read2()
{
  twai_message_t THE_C_Read2;
  THE_C_Read2.identifier = Motor2_IDs.Tx_PDO02;
  THE_C_Read2.data_length_code = 4;
  THE_C_Read2.data[0] = CAN_Commands.Read_Current2[0];
  THE_C_Read2.data[1] = CAN_Commands.Read_Current2[1];
  THE_C_Read2.data[2] = CAN_Commands.Read_Current2[2];
  THE_C_Read2.data[3] = CAN_Commands.Read_Current2[3];
  uint32_t message_id = THE_C_Read2.identifier;
  send_CAN_message( message_id, THE_C_Read2.data, THE_C_Read2.data_length_code);
}

void Read_Current1_1()
{
  twai_message_t Read_Current1_1;
  Read_Current1_1.identifier = Motor1_IDs.Tx_PDO03;
  Read_Current1_1.data_length_code = 4;
  Read_Current1_1.data[0] = CAN_Commands.Read_Current1[0];
  Read_Current1_1.data[1] = CAN_Commands.Read_Current1[1];
  Read_Current1_1.data[2] = CAN_Commands.Read_Current1[2];
  Read_Current1_1.data[3] = CAN_Commands.Read_Current1[3];
  uint32_t message_id = Read_Current1_1.identifier;
  send_CAN_message( message_id, Read_Current1_1.data, Read_Current1_1.data_length_code);
}

void Read_Current2_1()
{
  twai_message_t Read_Current2_1;
  Read_Current2_1.identifier = Motor2_IDs.Tx_PDO03;
  Read_Current2_1.data_length_code = 4;
  Read_Current2_1.data[0] = CAN_Commands.Read_Current2[0];
  Read_Current2_1.data[1] = CAN_Commands.Read_Current2[1];
  Read_Current2_1.data[2] = CAN_Commands.Read_Current2[2];
  Read_Current2_1.data[3] = CAN_Commands.Read_Current2[3];
  uint32_t message_id = Read_Current2_1.identifier;
  send_CAN_message( message_id, Read_Current2_1.data, Read_Current2_1.data_length_code);
}

void OP_Mode1_1()//PVM
{
  twai_message_t OP_Mode1_1;
  OP_Mode1_1.identifier = Motor1_IDs.Rx_PDO01;
  OP_Mode1_1.data_length_code = 1;
  OP_Mode1_1.data[0] = CAN_Commands.OP3[0];
  uint32_t message_id = OP_Mode1_1.identifier;
  send_CAN_message( message_id, OP_Mode1_1.data, OP_Mode1_1.data_length_code);
}

void OP_Mode1_2()//CSV
{
  twai_message_t OP_Mode1_2;
  OP_Mode1_2.identifier = Motor1_IDs.Rx_PDO01;
  OP_Mode1_2.data_length_code = 1;
  OP_Mode1_2.data[0] = CAN_Commands.OP9[0];
  uint32_t message_id = OP_Mode1_2.identifier;
  send_CAN_message( message_id, OP_Mode1_2.data, OP_Mode1_2.data_length_code);
}

void OP_Mode1_3()//CST
{
  twai_message_t OP_Mode1_3;
  OP_Mode1_3.identifier = Motor1_IDs.Rx_PDO01;
  OP_Mode1_3.data_length_code = 1;
  OP_Mode1_3.data[0] = CAN_Commands.OPA[0];
  uint32_t message_id = OP_Mode1_3.identifier;
  send_CAN_message( message_id, OP_Mode1_3.data, OP_Mode1_3.data_length_code);
}

void ControlWord1_1()
{
  twai_message_t ControlWord1_1;
  ControlWord1_1.identifier = Motor1_IDs.Rx_PDO02;
  ControlWord1_1.data_length_code = 2;
  ControlWord1_1.data[0] = CAN_Commands.Conw1[0];
  ControlWord1_1.data[1] = CAN_Commands.Conw1[1];
  uint32_t message_id = ControlWord1_1.identifier;
  send_CAN_message( message_id, ControlWord1_1.data, ControlWord1_1.data_length_code);
}

void ControlWord1_2()
{
  twai_message_t ControlWord1_2;
  ControlWord1_2.identifier = Motor1_IDs.Rx_PDO02;
  ControlWord1_2.data_length_code = 2;
  ControlWord1_2.data[0] = CAN_Commands.Conw2[0];
  ControlWord1_2.data[1] = CAN_Commands.Conw2[1];
  uint32_t message_id = ControlWord1_2.identifier;
  send_CAN_message( message_id, ControlWord1_2.data, ControlWord1_2.data_length_code);
}

void ControlWord1_3()
{
  twai_message_t ControlWord1_3;
  ControlWord1_3.identifier = Motor1_IDs.Rx_PDO02;
  ControlWord1_3.data_length_code = 2;
  ControlWord1_3.data[0] = CAN_Commands.Conw3[0];
  ControlWord1_3.data[1] = CAN_Commands.Conw3[1];
  uint32_t message_id = ControlWord1_3.identifier;
  send_CAN_message( message_id, ControlWord1_3.data, ControlWord1_3.data_length_code);
}

//ID2
void OP_Mode2_1() //PVM
{
  twai_message_t OP_Mode2_1;
  OP_Mode2_1.identifier = Motor2_IDs.Rx_PDO01;
  OP_Mode2_1.data_length_code = 1;
  OP_Mode2_1.data[0] = CAN_Commands.OP3[0];
  uint32_t message_id = OP_Mode2_1.identifier;
  send_CAN_message( message_id, OP_Mode2_1.data, OP_Mode2_1.data_length_code);
}

void OP_Mode2_2() //CSV
{
  twai_message_t OP_Mode2_2;
  OP_Mode2_2.identifier = Motor2_IDs.Rx_PDO01;
  OP_Mode2_2.data_length_code = 1;
  OP_Mode2_2.data[0] = CAN_Commands.OP9[0];
  uint32_t message_id = OP_Mode2_2.identifier;
  send_CAN_message( message_id, OP_Mode2_2.data, OP_Mode2_2.data_length_code);
}

void OP_Mode2_3()
{
  twai_message_t OP_Mode2_3;
  OP_Mode2_3.identifier = Motor2_IDs.Rx_PDO01;
  OP_Mode2_3.data_length_code = 1;
  OP_Mode2_3.data[0] = CAN_Commands.OPA[0];
  uint32_t message_id = OP_Mode2_3.identifier;
  send_CAN_message( message_id, OP_Mode2_3.data, OP_Mode2_3.data_length_code);
}

void ControlWord2_1()
{
  twai_message_t ControlWord2_1;
  ControlWord2_1.identifier = Motor2_IDs.Rx_PDO02;
  ControlWord2_1.data_length_code = 2;
  ControlWord2_1.data[0] = CAN_Commands.Conw1[0];
  ControlWord2_1.data[1] = CAN_Commands.Conw1[1];
  uint32_t message_id = ControlWord2_1.identifier;
  send_CAN_message( message_id, ControlWord2_1.data, ControlWord2_1.data_length_code);
}

void ControlWord2_2()
{
  twai_message_t ControlWord2_2;
  ControlWord2_2.identifier = Motor2_IDs.Rx_PDO02;
  ControlWord2_2.data_length_code = 2;
  ControlWord2_2.data[0] = CAN_Commands.Conw2[0];
  ControlWord2_2.data[1] = CAN_Commands.Conw2[1];
  uint32_t message_id = ControlWord2_2.identifier;
  send_CAN_message( message_id, ControlWord2_2.data, ControlWord2_2.data_length_code);
}

void ControlWord2_3()
{
  twai_message_t ControlWord2_3;
  ControlWord2_3.identifier = Motor2_IDs.Rx_PDO02;
  ControlWord2_3.data_length_code = 2;
  ControlWord2_3.data[0] = CAN_Commands.Conw3[0];
  ControlWord2_3.data[1] = CAN_Commands.Conw3[1];
  uint32_t message_id = ControlWord2_3.identifier;
  send_CAN_message( message_id, ControlWord2_3.data, ControlWord2_3.data_length_code);
}

void Target_Velocity_1()
{
  twai_message_t Target_Velocity_1;
  Target_Velocity_1.identifier = Motor1_IDs.Rx_PDO03;
  Target_Velocity_1.data_length_code = 4;
  Target_Velocity_1.data[0] = CAN_Commands.Profile_Velocity[0];
  Target_Velocity_1.data[1] = CAN_Commands.Profile_Velocity[1];
  Target_Velocity_1.data[2] = CAN_Commands.Profile_Velocity[2];
  Target_Velocity_1.data[3] = CAN_Commands.Profile_Velocity[3];
  uint32_t message_id = Target_Velocity_1.identifier;
  send_CAN_message( message_id, Target_Velocity_1.data, Target_Velocity_1.data_length_code);
}

void Target_Velocity_2()
{
  twai_message_t Target_Velocity_2;
  Target_Velocity_2.identifier = Motor2_IDs.Rx_PDO03;
  Target_Velocity_2.data_length_code = 4;
  Target_Velocity_2.data[0] = CAN_Commands.Profile_Velocity[0];
  Target_Velocity_2.data[1] = CAN_Commands.Profile_Velocity[1];
  Target_Velocity_2.data[2] = CAN_Commands.Profile_Velocity[2];
  Target_Velocity_2.data[3] = CAN_Commands.Profile_Velocity[3];
  uint32_t message_id = Target_Velocity_2.identifier;
  send_CAN_message( message_id, Target_Velocity_2.data, Target_Velocity_2.data_length_code);
}

void Target_Velocity_re1()
{
  twai_message_t Target_Velocity_re1;
  Target_Velocity_re1.identifier = Motor1_IDs.Rx_PDO03;
  Target_Velocity_re1.data_length_code = 4;
  Target_Velocity_re1.data[0] = CAN_Commands.Profile_Velocity_re[0];
  Target_Velocity_re1.data[1] = CAN_Commands.Profile_Velocity_re[1];
  Target_Velocity_re1.data[2] = CAN_Commands.Profile_Velocity_re[2];
  Target_Velocity_re1.data[3] = CAN_Commands.Profile_Velocity_re[3];
  uint32_t message_id = Target_Velocity_re1.identifier;
  send_CAN_message( message_id, Target_Velocity_re1.data, Target_Velocity_re1.data_length_code);
}

void Target_Velocity_re2()
{
  twai_message_t Target_Velocity_re2;
  Target_Velocity_re2.identifier = Motor2_IDs.Rx_PDO03;
  Target_Velocity_re2.data_length_code = 4;
  Target_Velocity_re2.data[0] = CAN_Commands.Profile_Velocity_re[0];
  Target_Velocity_re2.data[1] = CAN_Commands.Profile_Velocity_re[1];
  Target_Velocity_re2.data[2] = CAN_Commands.Profile_Velocity_re[2];
  Target_Velocity_re2.data[3] = CAN_Commands.Profile_Velocity_re[3];
  uint32_t message_id = Target_Velocity_re2.identifier; 
  send_CAN_message( message_id, Target_Velocity_re2.data, Target_Velocity_re2.data_length_code);
}

long DX1, DX2;
float EX1, EX2;

char STR1[5] = { '\0'};
char STR2[5] = { '\0'};

// DEXTRA1関数
void DEXTRA1()
{
  DX1 = 0.001 * strtol(STR1, NULL, 16);  // HEXから10進数に変換してDX1に格納
  EX1 = (float)DX1;  // 変換後の値をEX1に格納
}

// DEXTRA2関数
void DEXTRA2()
{
  DX2 = 0.001 * strtol(STR2, NULL, 16);  // HEXから10進数に変換してDX2に格納
  EX2 = (float)DX2;  // 変換後の値をEX2に格納
}

void printData( float* tensionValues1)
{
  Serial.print( current_1, 2);
  Serial.print(",");
  Serial.print( current_2, 2);

  for( int i = 0; i < reader.doutLen; i++)
  {
    Serial.print(",");
    Serial.print( tensionValues1[i], 2); 
  }
  Serial.println();
}

void command()
{
  switch(lan)
  {
    case 'F': //Forward
      OP_Mode1_2();
      OP_Mode2_2();
      digitalWrite( LED1, HIGH); //led on
      delay(10);

      ControlWord1_1();
      ControlWord2_1();
      digitalWrite( LED2, HIGH);//led on
      delay(10);

      ControlWord1_2();
      ControlWord2_2();
      digitalWrite( LED3, HIGH); //led on
      delay(10);

      Target_Velocity_1();
      Target_Velocity_re2();
      digitalWrite( LED4, HIGH); //led on
      delay(10);

      mode = 1;
      lan = 0;
      break;
    
    case 'B': //Back
      OP_Mode1_2();
      OP_Mode2_2();
      digitalWrite( LED1, HIGH); //led on
      delay(10);

      ControlWord1_1();
      ControlWord2_1();
      digitalWrite( LED2, HIGH); //led on
      delay(10);

      ControlWord1_2();
      ControlWord2_2();
      digitalWrite( LED3, HIGH);//led on
      delay(10);

      Target_Velocity_re1();
      Target_Velocity_2();
      digitalWrite( LED4, HIGH); //led on
      delay(10);
    
      mode = 2;
      lan = 0;
      break;
    
    case 'E': //左信地旋回
      OP_Mode2_2();
      ControlWord2_1();
      digitalWrite( LED1, HIGH); //led on
      digitalWrite( LED2, HIGH);
      delay(10);
      ControlWord2_2();
      digitalWrite( LED3, HIGH); //led on
      Target_Velocity_re2();
      digitalWrite( LED4, HIGH);
      delay(10);

      mode = 3;
      lan = 0;
      break;
    
    case 'R': //左超信地旋回
      OP_Mode1_2();
      OP_Mode2_2();
      digitalWrite( LED1, HIGH); //led on
      delay(10);

      ControlWord1_1();
      ControlWord2_1();
      digitalWrite( LED2, HIGH);//led on
      delay(10);

      ControlWord1_2();
      ControlWord2_2();
      digitalWrite( LED3, HIGH);//led on
      delay(10);

      Target_Velocity_re1();
      Target_Velocity_re2();
      digitalWrite( LED4, HIGH);//led on
      delay(10);

      mode = 4;
      lan = 0;
      break;
    
    case 'T': //右信地旋回
      OP_Mode1_2();
      ControlWord1_1();
      digitalWrite( LED1, HIGH);//led on
      digitalWrite( LED2, HIGH);
      delay(10);

      ControlWord1_2();
      Target_Velocity_1();
      digitalWrite( LED3, HIGH);
      digitalWrite( LED4, HIGH);//led on
      delay(10);

      mode = 5;
      lan = 0;
      break;
    
    case 'Y': //右超信地旋回
      OP_Mode1_2();
      OP_Mode2_2();
      digitalWrite( LED1, HIGH);//led on
      delay(10);

      ControlWord1_1();
      ControlWord2_1();
      digitalWrite( LED2, HIGH);//led on
      delay(10);

      ControlWord1_2();
      ControlWord2_2();
      digitalWrite( LED3, HIGH);//led on
      delay(10);

      Target_Velocity_1();
      Target_Velocity_2();
      digitalWrite( LED4, HIGH);//led on
      delay(10);

      mode = 6;
      lan = 0;
      break;

    case 'S':
      mode = 0;
      lan = 0;
      digitalWrite( LED1, LOW);
      digitalWrite( LED2, LOW);
      digitalWrite( LED3, LOW);
      digitalWrite( LED4, LOW);
      break;
  }
}

void setMode()
{
  //現在のモードに基づいて動作を実行
  if( mode == 0)
  {
    //Stop
    ControlWord1_3();
    ControlWord2_3();
    //digitalWrite(LED3_PIN, LOW);
    //digitalWrite(LED4_PIN, LOW);

  }else if( mode == 1)
  {
    //Forward
    Target_Velocity_1();
    Target_Velocity_re2();

  }else if( mode == 2)
  {
    //Back
    Target_Velocity_re1();
    Target_Velocity_2();

  }else if( mode == 3)
  {
    //左信地旋回
    Target_Velocity_re2();

  }else if( mode == 4)
  {
    //左超信地旋回
    Target_Velocity_re1();
    Target_Velocity_re2();
    
  }else if( mode == 5)
  {
    //右信地旋回
    Target_Velocity_1();

  }else if( mode == 6)
  {
    //右超信地旋回
    Target_Velocity_1();
    Target_Velocity_2();
  }
}

void setup() {
  Serial.begin( 115200);
  Serial1.begin( 115200, SERIAL_8N1, 18, 19);
  tensionQueue = xQueueCreate( 10, 16);
  //ロードセルの初期化をコア0で行う
  xTaskCreatePinnedToCore( loadCellTask, "LoadCellTask", 4096, NULL, 1, &thp[0], 0);
  //loadCellTask:実行するタスク, "LoadCellTask":タスク名, 2048:スタックメモリ, NULL:Pass, 1:タスクの優先順位, NULL:作成したタスクのハンドルオブジェクト, 0:コア番号
  //CAN通信・シリアル出力の処理をコア1で行う
  xTaskCreatePinnedToCore( serialOutputTask, "SerialOUTputTask", 4096, NULL, 1, &thp[1], 1);
  delay(500);
  initLED();
  delay(500);
  initCAN();
  delay(500);
  init_HX711();
  delay(1000);
  NMTPRE();
  delay(500);
  digitalWrite( LED1, HIGH);//LED1 Light
  NMTOP();
  digitalWrite( LED2, HIGH);//LED2 Light
  delay(500);
  Serial.println("Setup done...");
  setup_done = 1;//setup()の終了を通知
}

void loop() {
  delay(100);
}

void serialOutputTask( void *pvParameters ) {
  float tensionValues1[2];
  while(setup_done==0)
  {
    vTaskDelay( 10/portTICK_PERIOD_MS);
  }
  while(1) {
    Serial.println("688");
    static unsigned long lastTime = 0;
    unsigned long currentTime = millis();
    
    // UARTコマンドを確認
    if(Serial.available()) {
      lan = Serial.read();
      command();
    }
    
    setMode();
    Serial.println("699");  // この行は表示されるか？
    receiveCANMessage();  // CANメッセージの受信確認
    Serial.println("701");  // この行は表示されるか？
    
    if (xQueueReceive(tensionQueue, &tensionValues1, 10)) {
      Serial.println("703");  // キューからデータを取得
      printData(tensionValues1);
      Serial.println("705");  // データの表示
    } else {
      Serial.println("Queue Empty!");  // キューが空の場合
    }
    
    delay(5);
  }
}


void loadCellTask(void *pvParameters) //Tension 
{
  while(setup_done==0)
  {
    vTaskDelay( 10/portTICK_PERIOD_MS);
  }
  while(1)
  {
    unsigned long currentMicros = micros();
    if( currentMicros - prevMicros >= intervalMicros)
    {
      Serial.println("717");
      prevMicros = currentMicros;
      Serial.println("719");
      read_Tension();
      Serial.println("721");
      xQueueSend( tensionQueue, &tensionValues, 0);
      Serial.println("723");
    }
    delay(5);
  }
}