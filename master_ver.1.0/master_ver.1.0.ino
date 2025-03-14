//Experimental machine Ver.3.0
//master
//Date 2025-03-07
//Author Ryoya SATO

#include <Arduino.h>
#include "driver/ledc.h"

//Pin assign
int DIRR = 25;
int SLPR = 21;
int PWMR = 0;

int DIRL = 33;
int SLPL = 32;
int PWML = 12;

char lan;

int Setup = 0;

float sensor0Value, sensor1Value;
float target_tension = 0;
float T = 0.01;

float p_gain1 = 1.0;
float i_gain1 = 0.1;
float d_gain1 = 0.01;

float p_gain2 = 1.0;
float i_gain2 = 0.1;
float d_gain2 = 0.01;

float Error_force1_pre = 0, Error_force2_pre = 0;
float force1_integral = 0, force2_integral = 0;
float force1_derivative = 0, force2_derivative = 0;

//マルチスレッドのタスクハンドル格納
TaskHandle_t Process[1];

void pin_set()
{
  pinMode( DIRR, OUTPUT);
  pinMode( SLPR, OUTPUT);
  pinMode( PWMR, OUTPUT);
  pinMode( DIRL, OUTPUT);
  pinMode( SLPL, OUTPUT);
  pinMode( PWML, OUTPUT);

  //PWMチャンネル設定
  ledcAttach( PWMR, 1000, 8);
  ledcAttach( PWML, 1000, 8);
}

void command( char lan)
{
  switch( lan)
  {
    case 'F':
    case 'B':
    case 'E':
    case 'R':
    case 'T':
    case 'Y':
    case 'S':
      Serial1.print( lan);
    break;

    case 'Z': //目標履帯張力にする初期動作
      Setup = 1;
      Serial.println("Tension control start.");
    break;

    default:
      Serial.println("warning command error");
    break;
  }
}

void setup() {
  Serial.begin( 115200);
  Serial1.begin( 115200, SERIAL_8N1, 18, 19);
  xTaskCreatePinnedToCore( Tension_controlTask, "TensionControlTask", 8192, NULL, 1, &Process[0], 0);
  pin_set();
}

void loop() {
    //目標張力の設定処理
  Serial.println("Enter target tension : ");
  while(Serial.available()==0)
  {
    delay(10);
  }
  target_tension = Serial.parseFloat(); //目標張力を設定
  Serial.print("Target Tension set to : ");
  Serial.println(target_tension);

  delay(500); //文字化け防止のため待機
  //コマンド入力モードに移行
  Serial.print("Input command : "); //最初のプロンプト表示
  while(1)
  {
    while(!Serial.available()) //入力がある場合のみ処理
    {
      delay(10);
    }

      lan = Serial.read();
      if( lan == '\n' || lan == '\r')
      {
        continue;
      }
      Serial.println( lan);
      command( lan);

      //次のコマンド入力待ちの表示
      Serial.print("Input command : ");
    }

}

void Tension_controlTask( void *Process)
{
  while( Setup == 0)
  {
    delay(10);
  }
  while(1)
  {
    //UARTデータを読む
    read_uart();
    //Serial.println( target_tension);
    float Error_force1 = target_tension - sensor0Value;
    float Error_force2 = target_tension - sensor1Value;
    force1_integral += Error_force1 * T;
    force2_integral += Error_force2 * T;
    force1_derivative = ( Error_force1 - Error_force1_pre) / T; 
    force2_derivative = ( Error_force2 - Error_force2_pre) / T;
    float force1_control_signal = p_gain1 * Error_force1 + i_gain1 * force1_integral + d_gain1 * force1_derivative;
    float force2_control_signal = p_gain2 * Error_force2 + i_gain2 * force2_integral + d_gain2 * force2_derivative;

    //モータ駆動この部分の調整が必要
    int pwm_force1 = constrain( abs( force1_control_signal), 0, 255);
    int pwm_force2 = constrain( abs( force2_control_signal), 0, 255);

    if( target_tension > sensor0Value)
    {
      digitalWrite( DIRL, force1_control_signal > 0 ? HIGH : LOW); //適宜変更する(> or <)
    }else if( target_tension < sensor0Value)
    {
      digitalWrite( DIRL, force1_control_signal > 0 ? HIGH : LOW); //適宜変更する(> or <)
    }

    if( target_tension > sensor1Value)
    {
      digitalWrite( DIRL, force1_control_signal > 0 ? HIGH : LOW); //適宜変更する(> or <)
    }else if( target_tension < sensor1Value)
    {
      digitalWrite( DIRR, force2_control_signal > 0 ? HIGH : LOW); //適宜変更する(> or <)
    }
    ledcWrite( 0, pwm_force1);
    ledcWrite( 1, pwm_force2);

    //エラー更新
    Error_force1_pre = Error_force1;
    Error_force2_pre = Error_force2;

    delay(10); //control loop delay
  }
}

void read_uart() //確認済み
{
  if( Serial1.available())
  {
    String receivedData = Serial1.readStringUntil('\n');
    
    //データの分割
    int sensor0Index = receivedData.indexOf("sensor0");
    int sensor1Index = receivedData.indexOf("sensor1");

    if(sensor0Index != -1 && sensor1Index != -1)
    {
      int commandIndex = receivedData.indexOf(","); //カンマの位置を取得
    if( commandIndex != -1)
    {
      //sensor0の値を抽出
      sensor0Value = receivedData.substring(sensor0Index + 8, commandIndex).toFloat();
      //sensor1の値を抽出
      sensor1Value = receivedData.substring(sensor1Index + 8).toFloat();
      /*
      Serial.print("sensor0:");
      Serial.print(sensor0Value);
      Serial.print("sensor1:");
      Serial.println(sensor1Value);
      */
    }
    else
    {
      Serial.println("Warning Invalid UARTformat");
    }
    }
  }
}