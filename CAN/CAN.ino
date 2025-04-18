//Exeprimental machine ver.3.0
//CAN Test(send - receive)
//2025-03-02

#include <Arduino.h>
#include "driver/twai.h"

#define RX_PIN 4
#define TX_PIN 5

uint16_t current1 = 0;
uint16_t current2 = 0;

//uint8_t recceived_data[5];

// EPOS CAN ID Def(Left or Right)
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

/////////////Data Frame/////////////
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

const uint8_t received_buffer1[4] = { 0x00, 0x00, 0x00, 0x00};
const uint8_t received_buffer2[4] = { 0x00, 0x00, 0x00, 0x00};

const uint8_t PVM[4] = { 0x03, 0x00, 0x64, 0x00};
const uint8_t SCV[3] = { 0x09, 0x00, 0x64};

uint8_t received_data1[5];  // 受信データ1の配列
uint8_t received_data2[5];  // 受信データ2の配列

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

void received_data1()
{
  twai_message_t received_data1;
  received_data1.identifier = ID_1_Tx_PDO02;
  received_data1.data_length_code = 4;
  received_data1.data[0] = received_buffer1[0];
  received_data1.data[1] = received_buffer1[1];
  received_data1.data[2] = received_buffer1[2];
  received_data1.data[3] = received_buffer1[3];
  uint32_t message_id = received_data1.identifier;
  send_CAN_message( message_id, received_data1.data, received_data1.data_length_code);
}

void received_data2()
{
  twai_message_t received_data2;
  received_data2.identifier = ID_2_Tx_PDO02;
  received_data2.data_length_code = 4;
  received_data2.data[0] = received_buffer1[0];
  received_data2.data[1] = received_buffer1[1];
  received_data2.data[2] = received_buffer1[2];
  received_data2.data[3] = received_buffer2[3];
  uint32_t message_id = received_data2.identifier;
  send_CAN_message( message_id, received_data2.data, received_data2.data_length_code);
}

long DX1, DX2;
float EX1, EX2;

char STR1[5] = { '\0'};
char STR2[5] = { '\0'};

// DEXTRA1関数
void DEXTRA1() {
  DX1 = 0.001 * strtol(STR1, NULL, 16);  // HEXから10進数に変換してDX1に格納
  EX1 = (float)DX1;  // 変換後の値をEX1に格納
}

// DEXTRA2関数
void DEXTRA2() {
  DX2 = 0.001 * strtol(STR2, NULL, 16);  // HEXから10進数に変換してDX2に格納
  EX2 = (float)DX2;  // 変換後の値をEX2に格納
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

// CAN メッセージ送信
void send_CAN_message(uint32_t id, uint8_t *data, uint8_t len) {
    twai_message_t message;
    message.identifier = id;
    message.extd = 0;
    message.data_length_code = len;
    memcpy(message.data, data, len);
    twai_transmit(&message, pdMS_TO_TICKS(10));
}

void processCANMessage() {
twai_message_t message;
    {
      float currentA, currentB;
    if (message.identifier == 0x281) { 
        memcpy(received_data1, message.data, 5);
        snprintf(STR1, 11, "%02x%02x%02x%02x", received_data1.data[3], received_data1.data[2], received_data1.data[1], received_data1.data[0]);
        DX1 = strtoul(STR1, NULL, 16);
        currentA = (abs(0.001f * static_cast<float>(DX1)));
    }

    if (message.identifier == 0x290) { 
        memcpy(received_data2, message.data, 5);
        snprintf(STR2, 11, "%02x%02x%02x%02x", received_data2.data[3], received_data2.data[2], received_data2.data[1], received_data2.data[0]);
        DX2 = strtoul(STR2, NULL, 16);
        currentB = (abs(0.001f * static_cast<float>(DX2)));
    }
    }
}

void setup()
{
    Serial.begin( 115200);
    initCAN();
    delay(1000);
    NMTPRE();
    delay(500);
    NMTOP();
    delay(500);
}

void loop()
{
 // processCANMessage();
   CAN_message_t message;
  if (CAN.available()) {
    CAN.read(message);  // メッセージを読み取る

    // 受信データ1の処理
    memcpy(received_data1, message.data, 5);  // message.dataから受信データ1にコピー
    snprintf(STR1, 11, "%02x%02x%02x%02x%02x", received_data1[0], received_data1[1], received_data1[2], received_data1[3], received_data1[4]);
    
    // 受信データ1をシリアルモニタに表示
    Serial.print("Received Data 1: ");
    Serial.println(STR1);
    
    // 受信データ2の処理
    memcpy(received_data2, message.data, 5);  // message.dataから受信データ2にコピー
    snprintf(STR2, 11, "%02x%02x%02x%02x%02x", received_data2[0], received_data2[1], received_data2[2], received_data2[3], received_data2[4]);
    
    // 受信データ2をシリアルモニタに表示
    Serial.print("Received Data 2: ");
    Serial.println(STR2);
  }
  }