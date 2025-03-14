//Experimental machine Ver.3.0 EPOS4 Control
//Data 2025-03-02
//Author Ryoya SATO

#include <Arduino.h>
#include "driver\twai.h"
#include "CAN_library.h"
#include <HX711_asukiaaa.h>

#define RX_PIN 4
#define TX_PIN 5

int LED1 = 25;
int LED2 = 13;
int LED3 = 26;
int LED4 = 12;

//受信するCAN identifer 
#define TARGET_ID_1 0x281
#define TARGET_ID_2 0x290

char lan;
int mode = 0;

//電流値データ格納変数
float current1, current2;

//CANの初期化
void initCAN()
{
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TX_PIN, (gpio_num_t)RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) 
  {
    Serial.println("CANドライバのインストール成功");
  } else 
  {
    Serial.println("CANドライバのインストール失敗");
  }

  if( twai_start() == ESP_OK)
  {
    Serial.println("CAN通信開始");
  }else{
    Serial.println("CAN通信開始失敗");
  }
}

void initLED()
{
  pinMode( LED1, OUTPUT);
  pinMode( LED2, OUTPUT);
  pinMode( LED3, OUTPUT);
  pinMode( LED4, OUTPUT);
}

//CANメッセージ送信
void send_CAN_message( uint32_t id, uint8_t *data, uint8_t len)
{
  twai_message_t message;
  message.identifier = id;
  message.extd = 0;
  message.data_length_code = len;
  memcpy( message.data, data, len);

  twai_transmit( &message, pdMS_TO_TICKS(10));
}

//CANメッセージ受信
void receiveCANMessage() //ID = 0x281
{
  twai_message_t message;
  esp_err_t res = twai_receive( &message, 10 / portTICK_PERIOD_MS);
  if( res == ESP_OK)
  {
    if( message.identifier == TARGET_ID_1)
    {
      uint32_t raw_current1 = 0;
      for( int i = 0 ; i < message.data_length_code; i++)
      {
        //ビックエンディアンからリトルエンディアンに変換するためにバイト順を逆にする
        raw_current1 |= ((uint32_t)message.data[message.data_length_code - 1 - i]) << (i*8);
      }
      current1 = *(float*)&raw_current1; //uint32_t を floatに変換
      //電流値の校正する一文を挿入
    }
  }
  if( message.identifier == TARGET_ID_2)
  {
    uint32_t raw_current2 = 0;
    for( int j = 0 ; j < message.data_length_code; j++)
    {
      //ビックエンディアンからリトルエンディアンに変換するためにバイト順を逆にする
      raw_current2 |= ((uint32_t)message.data[message.data_length_code - 1 - j]) << (j*8);
    }
    current2 = *(float*)&raw_current2; //uint32_t を floatに変換
    //電流値の校正する一文を挿入
  }
}

void setup() {
 Serial.begin( 115200);
 initCAN();
 NMTPRE();
 //digitalWrite( LED1, HIGH);
 delay(50);
 NMTOP();
 //digitalWrite( LED2, HIGH);
 delay(50);
}

void loop() {
  sendTHE_P_Read1();
  sendTHE_C_Read2();
  if( Serial.available())
  {
    lan = Serial.read();
    command();
  }
  setMode();
  receiveCANMessage();
  delay(10);
}


//個別コマンド
void sendTHE_P_Read1()
{
  twai_message_t THE_P_Read1;
  THE_P_Read1.identifier = Motor1_IDs.Tx_PDO02;
  THE_P_Read1.data_length_code = 4;
  THE_P_Read1.data[0] = CAN_Commands.Read_Current1[0];
  THE_P_Read1.data[1] = CAN_Commands.Read_Current1[1];
  THE_P_Read1.data[2] = CAN_Commands.Read_Current1[2];
  THE_P_Read1.data[3] = CAN_Commands.Read_Current1[3];
  send_CAN_message( THE_P_Read1.identifier, THE_P_Read1.data, THE_P_Read1.data_length_code);
}

void sendTHE_C_Read2()
{
  twai_message_t THE_C_Read2;
  THE_C_Read2.identifier = Motor2_IDs.Tx_PDO02;
  THE_C_Read2.data_length_code = 4;
  THE_C_Read2.data[0] = CAN_Commands.Read_Current2[0];
  THE_C_Read2.data[1] = CAN_Commands.Read_Current2[1];
  THE_C_Read2.data[2] = CAN_Commands.Read_Current2[2];
  THE_C_Read2.data[3] = CAN_Commands.Read_Current2[3];
  send_CAN_message( THE_C_Read2.identifier, THE_C_Read2.data, THE_C_Read2.data_length_code);
}

void NMTPRE()
{
  twai_message_t NMTPRE;
  NMTPRE.identifier = 0x00;
  NMTPRE.data_length_code = 2;
  NMTPRE.data[0] = CAN_Commands.PreOP[0];
  NMTPRE.data[1] = CAN_Commands.PreOP[1];
  send_CAN_message( NMTPRE.identifier, NMTPRE.data, NMTPRE.data_length_code);
}

void NMTOP()
{
  twai_message_t NMTOP;
  NMTOP.identifier = 0x00;
  NMTOP.data_length_code = 2;
  NMTOP.data[0] = CAN_Commands.Operation[0];
  NMTOP.data[1] = CAN_Commands.Operation[1];
  send_CAN_message( NMTOP.identifier, NMTOP.data, NMTOP.data_length_code);
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