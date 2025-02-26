//Eveprimental machine Ver.3.0
//slave (Driving motor control and Reading Tension)
//Author Ryoya SATO
//2025-02-25

#include <Arduino.h>
#include "HX711.h"
#include "driver/twai.h"
#include <Wire.h>

#define RX_PIN 4
#define TX_PIN 5
uint16_t current_1 = 0;
uint16_t current_2 = 0;

bool recieved1 = false;
bool recieved2 = false;

int mode = 0;
char lan;

// Define of EPOS4 CAN ID
#define ID_1_Tx_PDO01 0x181
#define ID_1_Tx_PDO02 0x281
#define ID_1_Tx_PDO03 0x381
#define ID_1_Tx_PDO04 0x481

#define ID_1_Rx_PDO01 0x201
#define ID_1_Rx_PDO02 0x301
#define ID_1_Rx_PDO03 0x401
#define ID_1_Rx_PDO04 0x501

#define ID_2_Tx_PDO01 0x190
#define ID_2_Tx_PDO02 0x290
#define ID_2_Tx_PDO03 0x390
#define ID_2_Tx_PDO04 0x490

#define ID_2_Rx_PDO01 0x220
#define ID_2_Rx_PDO02 0x320
#define ID_2_Rx_PDO03 0x420
#define ID_2_Rx_PDO04 0x520

#define sync 0x80
#define Reset 0x00

const uint8_t Profile_Velocity[4] = { 0xC4, 0x09, 0x00, 0x00}; // 2500rpm
const uint8_t Profile_Velocity_re[4] = { 0x3C, 0xF6, 0xFF, 0xFF}; // -2500rpm

/////////////CAN Data Frame/////////////
//Mode of operation
const uint8_t OP3[1] = {0x03}; //PVM
const uint8_t OP9[1] = {0x09}; //CSV
const uint8_t OPA[1] = {0x0A}; //SCT

const uint8_t Conw1[2] = {0x06,0x00}; //Shutdown
const uint8_t Conw2[2] = {0x0F,0x00}; //Switchon & enable
const uint8_t Conw3[2] = {0x0B,0x00}; //Quick stop
//starthoming actual value
const uint8_t Conw4[3] = { 0x00, 0x25, 0x00};
const uint8_t Conw5[3] = { 0x00, 0x0F, 0x00};
const uint8_t Conw6[3] = { 0x00, 0x01, 0x00};

const uint8_t ResetP1[4] = { 0x64, 0x00, 0x00, 0x00};
const uint8_t PreOP[2] = { 0x80, 0x00};
const uint8_t Operation[2] = { 0x01, 0x00};
const uint8_t Reset_NMT[2] = { 0x82, 0x00};

const uint8_t Read_Current1[4] = { 0x00, 0x00, 0x00, 0x00};
const uint8_t Read_Current2[4] = { 0x00, 0x00, 0x00, 0x00};

const uint8_t PVM[4] = { 0x03, 0x00, 0x64, 0x00};
const uint8_t SCV[3] = { 0x09, 0x00, 0x64}; 

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
    esp_err_t res = twai_receive( &message, portMAX_DELAY); //受信待機
    if ( res == ESP_OK) 
    {
      if( message.identifier == 0x281)
      {
      //ID_1からの電流値を解析
      current_1 = ( message.data[0] << 8) | message.data[1];
      }
      else if( message.identifier == 0x290)
      {
        //ID_2からの電流値を解析
        current_2 = ( message.data[0] << 8) | message.data[1];
      }
      Serial.print("Left Current : ");
      Serial.print( current_1);
      Serial.print("Right Current : ");
      Serial.println( current_2);
    }
}

//個別コマンド
void NMTPRE()
{
  twai_message_t NMTPRE;
  NMTPRE.identifier = 0x00;
  NMTPRE.data_length_code = 2;
  NMTPRE.data[0] = PreOP[0];
  NMTPRE.data[1] = PreOP[1];
  uint32_t message_id = NMTPRE.identifier;
  send_CAN_message( message_id, NMTPRE.data, NMTPRE.data_length_code);
}

void NMTOP()
{
  twai_message_t NMTOP;
  NMTOP.identifier = 0x00;
  NMTOP.data_length_code = 2;
  NMTOP.data[0] = Operation[0];
  NMTOP.data[1] = Operation[1];
  uint32_t message_id = NMTOP.identifier;
  send_CAN_message( message_id, NMTOP.data, NMTOP.data_length_code);
}

void THE_P_Read1()
{
  twai_message_t THE_P_Read1;
  THE_P_Read1.identifier = ID_1_Tx_PDO02;
  THE_P_Read1.data_length_code = 4;
  THE_P_Read1.data[0] = Read_Current1[0];
  THE_P_Read1.data[1] = Read_Current1[1];
  THE_P_Read1.data[2] = Read_Current1[2];
  THE_P_Read1.data[3] = Read_Current1[3];
  uint32_t message_id = THE_P_Read1.identifier;
  send_CAN_message( message_id, THE_P_Read1.data, THE_P_Read1.data_length_code);
}

void THE_C_Read2()
{
  twai_message_t THE_C_Read2;
  THE_C_Read2.identifier = ID_2_Tx_PDO02;
  THE_C_Read2.data_length_code = 4;
  THE_C_Read2.data[0] = Read_Current2[0];
  THE_C_Read2.data[1] = Read_Current2[1];
  THE_C_Read2.data[2] = Read_Current2[2];
  THE_C_Read2.data[3] = Read_Current2[3];
  uint32_t message_id = THE_C_Read2.identifier;
  send_CAN_message( message_id, THE_C_Read2.data, THE_C_Read2.data_length_code);
}

void Read_Current1_1()
{
  twai_message_t Read_Current1_1;
  Read_Current1_1.identifier = ID_1_Tx_PDO03;
  Read_Current1_1.data_length_code = 4;
  Read_Current1_1.data[0] = Read_Current1[0];
  Read_Current1_1.data[1] = Read_Current1[1];
  Read_Current1_1.data[2] = Read_Current1[2];
  Read_Current1_1.data[3] = Read_Current1[3];
  uint32_t message_id = Read_Current1_1.identifier;
  send_CAN_message( message_id, Read_Current1_1.data, Read_Current1_1.data_length_code);
}

void Read_Current2_1()
{
  twai_message_t Read_Current2_1;
  Read_Current2_1.identifier = ID_2_Tx_PDO03;
  Read_Current2_1.data_length_code = 4;
  Read_Current2_1.data[0] = Read_Current2[0];
  Read_Current2_1.data[1] = Read_Current2[1];
  Read_Current2_1.data[2] = Read_Current2[2];
  Read_Current2_1.data[3] = Read_Current2[3];
  uint32_t message_id = Read_Current2_1.identifier;
  send_CAN_message( message_id, Read_Current2_1.data, Read_Current2_1.data_length_code);
}

void OP_Mode1_1()//PVM
{
  twai_message_t OP_Mode1_1;
  OP_Mode1_1.identifier = ID_1_Rx_PDO01;
  OP_Mode1_1.data_length_code = 1;
  OP_Mode1_1.data[0] = OP3[0];
  uint32_t message_id = OP_Mode1_1.identifier;
  send_CAN_message( message_id, OP_Mode1_1.data, OP_Mode1_1.data_length_code);
}

void OP_Mode1_2()//CSV
{
  twai_message_t OP_Mode1_2;
  OP_Mode1_2.identifier = ID_1_Rx_PDO01;
  OP_Mode1_2.data_length_code = 1;
  OP_Mode1_2.data[0] = OP9[0];
  uint32_t message_id = OP_Mode1_2.identifier;
  send_CAN_message( message_id, OP_Mode1_2.data, OP_Mode1_2.data_length_code);
}

void OP_Mode1_3()//CST
{
  twai_message_t OP_Mode1_3;
  OP_Mode1_3.identifier = 1;
  OP_Mode1_3.data_length_code = 1;
  OP_Mode1_3.data[0] = OPA[0];
  uint32_t message_id = OP_Mode1_3.identifier;
  send_CAN_message( message_id, OP_Mode1_3.data, OP_Mode1_3.data_length_code);
}

void ControlWord1_1()
{
  twai_message_t ControlWord1_1;
  ControlWord1_1.identifier = ID_1_Rx_PDO02;
  ControlWord1_1.data_length_code = 2;
  ControlWord1_1.data[0] = Conw1[0];
  ControlWord1_1.data[1] = Conw1[1];
  uint32_t message_id = ControlWord1_1.identifier;
  send_CAN_message( message_id, ControlWord1_1.data, ControlWord1_1.data_length_code);
}

void ControlWord1_2()
{
  twai_message_t ControlWord1_2;
  ControlWord1_2.identifier = ID_1_Rx_PDO02;
  ControlWord1_2.data_length_code = 2;
  ControlWord1_2.data[0] = Conw2[0];
  ControlWord1_2.data[1] = Conw2[1];
  uint32_t message_id = ControlWord1_2.identifier;
  send_CAN_message( message_id, ControlWord1_2.data, ControlWord1_2.data_length_code);
}

void ControlWord1_3()
{
  twai_message_t ControlWord1_3;
  ControlWord1_3.identifier = ID_1_Rx_PDO02;
  ControlWord1_3.data_length_code = 2;
  ControlWord1_3.data[0] = Conw3[0];
  ControlWord1_3.data[1] = Conw3[1];
  uint32_t message_id = ControlWord1_3.identifier;
  send_CAN_message( message_id, ControlWord1_3.data, ControlWord1_3.data_length_code);
}

//ID2
void OP_Mode2_1() //PVM
{
  twai_message_t OP_Mode2_1;
  OP_Mode2_1.identifier = ID_2_Rx_PDO01;
  OP_Mode2_1.data_length_code = 1;
  OP_Mode2_1.data[0] = OP3[0];
  uint32_t message_id = OP_Mode2_1.identifier;
  send_CAN_message( message_id, OP_Mode2_1.data, OP_Mode2_1.data_length_code);
}

void OP_Mode2_2() //CSV
{
  twai_message_t OP_Mode2_2;
  OP_Mode2_2.identifier = ID_2_Rx_PDO01;
  OP_Mode2_2.data_length_code = 1;
  OP_Mode2_2.data[0] = OP9[0];
  uint32_t message_id = OP_Mode2_2.identifier;
  send_CAN_message( message_id, OP_Mode2_2.data, OP_Mode2_2.data_length_code);
}

void OP_Mode2_3()
{
  twai_message_t OP_Mode2_3;
  OP_Mode2_3.identifier = ID_2_Rx_PDO01;
  OP_Mode2_3.data_length_code = 1;
  OP_Mode2_3.data[0] = OPA[0];
  uint32_t message_id = OP_Mode2_3.identifier;
  send_CAN_message( message_id, OP_Mode2_3.data, OP_Mode2_3.data_length_code);
}

void ControlWord2_1()
{
  twai_message_t ControlWord2_1;
  ControlWord2_1.identifier = ID_2_Rx_PDO02;
  ControlWord2_1.data_length_code = 2;
  ControlWord2_1.data[0] = Conw1[0];
  ControlWord2_1.data[1] = Conw1[1];
  uint32_t message_id = ControlWord2_1.identifier;
  send_CAN_message( message_id, ControlWord2_1.data, ControlWord2_1.data_length_code);
}

void ControlWord2_2()
{
  twai_message_t ControlWord2_2;
  ControlWord2_2.identifier = ID_2_Rx_PDO02;
  ControlWord2_2.data_length_code = 2;
  ControlWord2_2.data[0] = Conw2[0];
  ControlWord2_2.data[1] = Conw2[1];
  uint32_t message_id = ControlWord2_2.identifier;
  send_CAN_message( message_id, ControlWord2_2.data, ControlWord2_2.data_length_code);
}

void ControlWord2_3()
{
  twai_message_t ControlWord2_3;
  ControlWord2_3.identifier = ID_2_Rx_PDO02;
  ControlWord2_3.data_length_code = 2;
  ControlWord2_3.data[0] = Conw3[0];
  ControlWord2_3.data[1] = Conw3[1];
  uint32_t message_id = ControlWord2_3.identifier;
  send_CAN_message( message_id, ControlWord2_3.data, ControlWord2_3.data_length_code);
}

void Target_Velocity_1()
{
  twai_message_t Target_Velocity_1;
  Target_Velocity_1.identifier = ID_1_Rx_PDO03;
  Target_Velocity_1.data_length_code = 4;
  Target_Velocity_1.data[0] = Profile_Velocity[0];
  Target_Velocity_1.data[1] = Profile_Velocity[1];
  Target_Velocity_1.data[2] = Profile_Velocity[2];
  Target_Velocity_1.data[3] = Profile_Velocity[3];
  uint32_t message_id = Target_Velocity_1.identifier;
  send_CAN_message( message_id, Target_Velocity_1.data, Target_Velocity_1.data_length_code);
}

void Target_Velocity_2()
{
  twai_message_t Target_Velocity_2;
  Target_Velocity_2.identifier = ID_2_Rx_PDO03;
  Target_Velocity_2.data_length_code = 4;
  Target_Velocity_2.data[0] = Profile_Velocity[0];
  Target_Velocity_2.data[1] = Profile_Velocity[1];
  Target_Velocity_2.data[2] = Profile_Velocity[2];
  Target_Velocity_2.data[3] = Profile_Velocity[3];
  uint32_t message_id = Target_Velocity_2.identifier;
  send_CAN_message( message_id, Target_Velocity_2.data, Target_Velocity_2.data_length_code);
}

void Target_Velocity_re1()
{
  twai_message_t Target_Velocity_re1;
  Target_Velocity_re1.identifier = ID_1_Rx_PDO03;
  Target_Velocity_re1.data_length_code = 4;
  Target_Velocity_re1.data[0] = Profile_Velocity_re[0];
  Target_Velocity_re1.data[1] = Profile_Velocity_re[1];
  Target_Velocity_re1.data[2] = Profile_Velocity_re[2];
  Target_Velocity_re1.data[3] = Profile_Velocity_re[3];
  uint32_t message_id = Target_Velocity_re1.identifier;
  send_CAN_message( message_id, Target_Velocity_re1.data, Target_Velocity_re1.data_length_code);
}

void Target_Velocity_re2()
{
  twai_message_t Target_Velocity_re2;
  Target_Velocity_re2.identifier = ID_2_Rx_PDO03;
  Target_Velocity_re2.data_length_code = 4;
  Target_Velocity_re2.data[0] = Profile_Velocity_re[0];
  Target_Velocity_re2.data[1] = Profile_Velocity_re[1];
  Target_Velocity_re2.data[2] = Profile_Velocity_re[2];
  Target_Velocity_re2.data[3] = Profile_Velocity_re[3];
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

void command()
{
  switch(lan)
  {
    case 'F': //Forward
      OP_Mode1_2();
      OP_Mode2_2();
      //led on
      delay(10);

      ControlWord1_1();
      ControlWord2_1();
      //led on
      delay(10);

      ControlWord1_2();
      ControlWord2_2();
      //led on
      delay(10);

      Target_Velocity_1();
      Target_Velocity_re2();
      //led on
      delay(10);

      mode = 1;
      lan = 0;
      break;
    
    case 'B': //Back
      OP_Mode1_2();
      OP_Mode2_2();
      //led on
      delay(10);

      ControlWord1_1();
      ControlWord2_1();
      //led on
      delay(10);

      ControlWord1_2();
      ControlWord2_2();
      //led on
      delay(10);

      Target_Velocity_re1();
      Target_Velocity_2();
      //led on
      delay(10);
    
      mode = 2;
      lan = 0;
      break;
    
    case 'E': //左信地旋回
      OP_Mode2_2();
      ControlWord2_1();
      //led on
      delay(10);
      ControlWord2_2();
      //led on
      Target_Velocity_re2();
      delay(10);

      mode = 3;
      lan = 0;
      break;
    
    case 'R': //左超信地旋回
      OP_Mode1_2();
      OP_Mode2_2();
      //led on
      delay(10);

      ControlWord1_1();
      ControlWord2_1();
      //led on
      delay(10);

      ControlWord1_2();
      ControlWord2_2();
      //led on
      delay(10);

      Target_Velocity_re1();
      Target_Velocity_re2();
      //led on
      delay(10);

      mode = 4;
      lan = 0;
      break;
    
    case 'T': //右信地旋回
      OP_Mode1_2();
      ControlWord1_1();
      //led on
      delay(10);

      ControlWord1_2();
      Target_Velocity_1();
      //led on
      delay(10);

      mode = 5;
      lan = 0;
      break;
    
    case 'Y': //右超信地旋回
      OP_Mode1_2();
      OP_Mode2_2();
      //led on
      delay(10);

      ControlWord1_1();
      ControlWord2_1();
      //led on
      delay(10);

      ControlWord1_2();
      ControlWord2_2();
      //led on
      delay(10);

      Target_Velocity_1();
      Target_Velocity_2();
      //led on
      delay(10);

      mode = 6;
      lan = 0;
      break;

    case 'S':
      mode = 0;
      lan = 0;
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
  initCAN();
  delay(1000);
  NMTPRE();
  delay(500);
  //LED1 Light
  NMTOP();
  //LED2 Light
  delay(500);
}

void loop() {
  //UARTコマンドを確認
  if(Serial.available())
  {
    lan = Serial.read();
    command();
  }
  setMode();
  receiveCANMessage();
 // read_Tension();
 // Predicted_Traction_Model();
  delay(10); //Samoling rate 10ms
}
