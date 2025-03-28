//Experimental machine Ver.3.0
//2025-03-14
//Author Ryoya SATO

#include <Arduino.h>
#include "driver/twai.h"
#include "CAN_library.h"
#include <HX711_Git.h>
#include <cstring>
#include "CANMessage.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

QueueHandle_t rx_queue;

//CAN通信用ピン
#define RX_PIN 4
#define TX_PIN 5

unsigned long previousMillis = 0;
const long interval = 10;  // 1秒=1000ms

//マルチスレッドのタスクハンドル格納
TaskHandle_t Process[3];

//グローバル変数
float processed_current1 = 0;
float processed_current2 = 0;
float tensionValues[2];
float tensionValues1[2];
uint32_t raw_value1 = 0;
uint32_t raw_value2 = 0;
char lan;
int mode = 0;
int Setup = 0;

//LED_setup
int LED1 = 25;
int LED2 = 13;
int LED3 = 26;
int LED4 = 12;

long DX1, DX2;
char STR1[5] = { '\0'};
char STR2[5] = { '\0'};

//HX711関係
int pinsDout[] = { 21, 32};
const int numPins = sizeof( pinsDout) / sizeof( pinsDout[0]) ;
int pinSlk = 22;
HX711_asukiaaa::Reader reader( pinsDout, numPins, pinSlk);

#define LOAD_CELL_RATED_VOLT 0.0075f
#define LOAD_CELL_RATED_GRAM 10000.0f
//入力抵抗調整
#define HX711_R1 1000.0

//フィルタ抵抗
#define HX711_R2 1000.0

HX711_asukiaaa::Parser parser( LOAD_CELL_RATED_VOLT, LOAD_CELL_RATED_GRAM, HX711_R1, HX711_R2);
float offsetGrams[numPins];
float calibrationFactors[numPins] = { 208.4486, 205.6171}; //各センサに対する校正係数

static bool driver_installed = false;

static void handle_rx_message( twai_message_t &message)
{
  switch( message.identifier)
  {
    case 0x281:
      uint8_t reversed_data1[5] ; 
      for( int i = 0; i < message.data_length_code; i++)
      {
        reversed_data1[i] = message.data[message.data_length_code - 1-i];
      }      
      raw_value1 = ( reversed_data1[0] << 24) | ( reversed_data1[1] << 16) | ( reversed_data1[2] << 8) | reversed_data1[3];
      processed_current1 = *((float*)&raw_value1);
      break;
  
    case 0x290:
      uint8_t reversed_data2[5] ; 
      for( int i = 0; i < message.data_length_code; i++)
      {
        reversed_data2[i] = message.data[message.data_length_code - 1-i];
      }
      raw_value2 = ( reversed_data2[0] << 24) | ( reversed_data2[1] << 16) | ( reversed_data2[2] << 8) | reversed_data2[3];
      processed_current2 = *((float*)&raw_value2);
      break;
    
    default:
      //その他のIDを無視する
      break;
  }
}

// CANの初期化
void initCAN() 
{
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TX_PIN, (gpio_num_t)RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    //twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    twai_filter_config_t f_config = {
    .acceptance_code = (0x280 << 21),  // 基準となるID（共通部分）
    .acceptance_mask = ~(0x1F << 16),  // 最下位5ビット（0x1F）を無視する
    .single_filter = true
    };
    g_config.rx_queue_len = 20;

    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        Serial.println("CANドライバのインストール成功");
    } else {
        Serial.println("CANドライバのインストール失敗");
        return;
    }
    
    if (twai_start() == ESP_OK) {
        Serial.println("CAN通信開始");
    } else {
        Serial.println("CAN通信開始失敗");
    }
    //アラートの設定
    uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_ERROR | TWAI_ALERT_RX_QUEUE_FULL;
    if( twai_reconfigure_alerts( alerts_to_enable, NULL) == ESP_OK)
    {
      Serial.println("CAN Alerts reconfigured");
    }else{
      Serial.println("Failed to reconfigure alerts");
    }
    driver_installed = true;
}

//LED_setup
void ledsetup()
{
  pinMode( LED1, OUTPUT);
  pinMode( LED2, OUTPUT);
  pinMode( LED3, OUTPUT);
  pinMode( LED4, OUTPUT);
}

void init_HX711()
{
  reader.begin();
  
  while( reader.read() != 0)
  {
    Serial.println("Failed initial reading... Retry.");
    delay(100);
  }
  
  //オフセット処理
  for( int i = 0; i < reader.doutLen; i++)
  {
    offsetGrams[i] = parser.parseToGram( reader.values[i]);
  }
}

// CANメッセージ送信
void send_CAN_message(uint32_t id, uint8_t *data, uint8_t len) 
{
    twai_message_t message;
    message.identifier = id;
    message.extd = 0;
    message.data_length_code = len;
    memcpy(message.data, data, len);
    
    if (twai_transmit(&message, pdMS_TO_TICKS(10)) != ESP_OK) {
        Serial.println("CAN送信失敗");
    }
}

void setup() 
{
  Serial.begin( 500000);
  Serial1.begin( 115200, SERIAL_8N1, 18, 19);
  init_HX711();
 // delay(500);
  initCAN();
  ledsetup();
  xTaskCreatePinnedToCore( handleUARTTask, "UART Task", 8192, NULL, 4,  &Process[0], 0);
  xTaskCreatePinnedToCore( handleLoadcellTask, "Loadcell Task", 8192, NULL, 3,  &Process[1], 1);
 // xTaskCreatePinnedToCore( SerialplotTask, "Serial plot Task", 8192, NULL, 2,  &Process[2], 1);
  xTaskCreatePinnedToCore( CAN_receive_task, "CAN_RX_TASK", 8192, NULL, 5, &Process[3], 0);
  //delay(500);
  NMTPRE();
  digitalWrite( LED1, HIGH);
  delay(1000);
  NMTOP();
  digitalWrite( LED2, HIGH);
  delay(1000);
  Setup = 1;
}

void loop()
{
  vTaskDelay(  10 / portTICK_PERIOD_MS);
}

void CAN_receive_task( void *Process) //PRO_CORE Thread1(動確済み)
{
  while(1)
  {
  unsigned long currentMillis = millis();
  twai_message_t message;
  if(driver_installed)
  {
    if(currentMillis - previousMillis >= interval)
    {
    //Serial.println("OK");
    sendTHE_P_Read1();  // 空パケット送信
    sendTHE_C_Read2();  // 空パケット送信
    if( twai_receive( &message, 0) == ESP_OK)
    {
      handle_rx_message( message);
    }
    previousMillis = currentMillis;
    } 
  }
  vTaskDelay(  10 / portTICK_PERIOD_MS);
  }
}

//UART受信タスク
void handleUARTTask( void *Process)  //PRO_CORE Thread2(動確済み)
{
  while(1)
  {
    if( Serial1.available())
    {
      lan = Serial1.read();
      command();
    }
    setMode();
    vTaskDelay(  10 / portTICK_PERIOD_MS); 
  }
}

//HX711管理タスク
void handleLoadcellTask( void *Process) //ADD_CORE Thread1
{
  while(1)
  {
    auto readState = reader.read();
     if (readState == HX711_asukiaaa::ReadState::Success) {
      for (int i = 0; i < reader.doutLen; ++i) {
       float gram = parser.parseToGram(reader.values[i]) - offsetGrams[i];
       float newtons = gram * 0.00981 * calibrationFactors[i];
       tensionValues[i] = newtons; //配列に格納
      }
  }
  vTaskDelay(  10 / portTICK_PERIOD_MS); 
  }
}

//シリアルプロット管理タスク
/*
void SerialplotTask( void *Process) //ADD_CORE THread2
{
  while(1)
  {
    unsigned long currentMillis = millis(); // ここで更新
  if(Setup)
  {
    if(currentMillis - previousMillis >= interval)
    {
      Serial.printf("Current1: %.2f, Current2: %.2f, Tension1: %.2f, Tension2: %.2f\r\n", processed_current1, processed_current2, tensionValues[0], tensionValues[1]);
      Serial1.printf("sensor0: %.2f, sensor1: %.2f\n", tensionValues[0], tensionValues[1]);
      previousMillis = currentMillis;
    }
  }
  vTaskDelay(  10 / portTICK_PERIOD_MS); 
  //delay(10);
  }
}
*/

//個別コマンド
void CAN_receive()
{
  if(!driver_installed)
  {
    vTaskDelay(  10 / portTICK_PERIOD_MS);
    return;
  }
  //アラートを確認
  uint32_t alerts_triggered;
  twai_read_alerts( &alerts_triggered, pdMS_TO_TICKS(10));
  twai_status_info_t twaistatus;
  twai_get_status_info( &twaistatus);

  //アラート処理
  if( alerts_triggered & TWAI_ALERT_ERR_PASS)
  {
    Serial.println("Alert: TWAI controller has become error passive.");
  }
  if( alerts_triggered & TWAI_ALERT_BUS_ERROR)
  {
    Serial.println("Alert: A(Bit, stuff, CRC, From, ACK)error has occurred on the bus.");
    Serial.printf("Bus eeror count: %lu\n", twaistatus.bus_error_count);
  }
  if( alerts_triggered & TWAI_ALERT_RX_QUEUE_FULL)
  {
    Serial.println("Alert: The RX queue is full causing a received frame to be lost.");
    Serial.printf("RX buffered: %lu\t", twaistatus.msgs_to_rx);
    Serial.printf("RX ,issed: %lu\t", twaistatus.rx_missed_count);
    Serial.printf("RX overrun %lu\n", twaistatus.rx_overrun_count);
  }

  //受信データ処理
  if( alerts_triggered & TWAI_ALERT_RX_DATA)
  {
    twai_message_t message;
    esp_err_t res;

    if(( twai_receive( &message, 0)) == ESP_OK)
    {
      if( message.data_length_code == 4)
        handle_rx_message( message);
    }
  }
}


void sendTHE_P_Read1()
{
    uint8_t data[4] = {
        CAN_Commands.Read_Current1[0],
        CAN_Commands.Read_Current1[1],
        CAN_Commands.Read_Current1[2],
        CAN_Commands.Read_Current1[3]
    };
    send_CAN_message(Motor1_IDs.Tx_PDO02, data, 4);
}

void sendTHE_C_Read2()
{
    uint8_t data[4] = {
        CAN_Commands.Read_Current2[0],
        CAN_Commands.Read_Current2[1],
        CAN_Commands.Read_Current2[2],
        CAN_Commands.Read_Current2[3]
    };
    send_CAN_message(Motor2_IDs.Tx_PDO02, data, 4);
}

void Read_Current1_1()
{
      uint8_t data[4] = {
        CAN_Commands.Read_Current1[0],
        CAN_Commands.Read_Current1[1],
        CAN_Commands.Read_Current1[2],
        CAN_Commands.Read_Current1[3]
    };
    send_CAN_message( Motor1_IDs.Tx_PDO03, data, 4);
}

void Read_Current2_1()
{
      uint8_t data[4] = {
        CAN_Commands.Read_Current2[0],
        CAN_Commands.Read_Current2[1],
        CAN_Commands.Read_Current2[2],
        CAN_Commands.Read_Current2[3]
    };
    send_CAN_message( Motor2_IDs.Tx_PDO03, data, 4);
}

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

void command()
{
  switch(lan)
  {
    case 'F': //Forward
      OP_Mode1_2();
      OP_Mode2_2();
      digitalWrite( LED1, HIGH); //led on
      vTaskDelay(pdMS_TO_TICKS(10));

      ControlWord1_1();
      ControlWord2_1();
      digitalWrite( LED2, HIGH);//led on
      vTaskDelay(pdMS_TO_TICKS(10));

      ControlWord1_2();
      ControlWord2_2();
      digitalWrite( LED3, HIGH); //led on
      vTaskDelay(pdMS_TO_TICKS(10));

      Target_Velocity_1();
      Target_Velocity_re2();
      digitalWrite( LED4, HIGH); //led on
      vTaskDelay(pdMS_TO_TICKS(10));

      mode = 1;
      lan = 0;
      break;
    
    case 'B': //Back
      OP_Mode1_2();
      OP_Mode2_2();
      digitalWrite( LED1, HIGH); //led on
      vTaskDelay(pdMS_TO_TICKS(10));

      ControlWord1_1();
      ControlWord2_1();
      digitalWrite( LED2, HIGH); //led on
      vTaskDelay(pdMS_TO_TICKS(10));

      ControlWord1_2();
      ControlWord2_2();
      digitalWrite( LED3, HIGH);//led on
      vTaskDelay(pdMS_TO_TICKS(10));

      Target_Velocity_re1();
      Target_Velocity_2();
      digitalWrite( LED4, HIGH); //led on
      vTaskDelay(pdMS_TO_TICKS(10));
    
      mode = 2;
      lan = 0;
      break;
    
    case 'E': //左信地旋回
      OP_Mode2_2();
      ControlWord2_1();
      digitalWrite( LED1, HIGH); //led on
      digitalWrite( LED2, HIGH);
      vTaskDelay(pdMS_TO_TICKS(10));
      ControlWord2_2();
      digitalWrite( LED3, HIGH); //led on
      Target_Velocity_re2();
      digitalWrite( LED4, HIGH);
      vTaskDelay(pdMS_TO_TICKS(10));

      mode = 3;
      lan = 0;
      break;
    
    case 'R': //左超信地旋回
      OP_Mode1_2();
      OP_Mode2_2();
      digitalWrite( LED1, HIGH); //led on
      vTaskDelay(pdMS_TO_TICKS(10));

      ControlWord1_1();
      ControlWord2_1();
      digitalWrite( LED2, HIGH);//led on
      vTaskDelay(pdMS_TO_TICKS(10));

      ControlWord1_2();
      ControlWord2_2();
      digitalWrite( LED3, HIGH);//led on
      vTaskDelay(pdMS_TO_TICKS(10));

      Target_Velocity_re1();
      Target_Velocity_re2();
      digitalWrite( LED4, HIGH);//led on
      vTaskDelay(pdMS_TO_TICKS(10));

      mode = 4;
      lan = 0;
      break;
    
    case 'T': //右信地旋回
      OP_Mode1_2();
      ControlWord1_1();
      digitalWrite( LED1, HIGH);//led on
      digitalWrite( LED2, HIGH);
      vTaskDelay(pdMS_TO_TICKS(10));

      ControlWord1_2();
      Target_Velocity_1();
      digitalWrite( LED3, HIGH);
      digitalWrite( LED4, HIGH);//led on
      vTaskDelay(pdMS_TO_TICKS(10));

      mode = 5;
      lan = 0;
      break;
    
    case 'Y': //右超信地旋回
      OP_Mode1_2();
      OP_Mode2_2();
      digitalWrite( LED1, HIGH);//led on
      vTaskDelay(pdMS_TO_TICKS(10));

      ControlWord1_1();
      ControlWord2_1();
      digitalWrite( LED2, HIGH);//led on
      vTaskDelay(pdMS_TO_TICKS(10));

      ControlWord1_2();
      ControlWord2_2();
      digitalWrite( LED3, HIGH);//led on
      vTaskDelay(pdMS_TO_TICKS(10));

      Target_Velocity_1();
      Target_Velocity_2();
      digitalWrite( LED4, HIGH);//led on
      vTaskDelay(pdMS_TO_TICKS(10));

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
    digitalWrite(LED3, LOW);
    digitalWrite(LED4, LOW);

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
