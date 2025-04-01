//Experimental machine Ver.3.0
//master
//Motor PID controll , UART communication , Pluse counter, AutoTune
//Date 2025-04-01
//Author Ryoya SATO

#include <Arduino.h>
#include "driver/ledc.h"
#include "driver/pcnt.h"

#define ENCODER1_A 34//A1相
#define ENCODER1_B 35//B1相
#define ENCODER2_A 26//A2相
#define ENCODER2_B 27//B2相

#define PCNT_UNIT_ENCA PCNT_UNIT_0 // エンコーダ1
#define PCNT_UNIT_ENCB PCNT_UNIT_1 // エンコーダ2

//ゲインの最適化を行う
float T = 0.02;

//PD制御
//最適ゲイン(2025/04/01)
float p_gain1 = 1.38;
float i_gain1 = 0.11;
float d_gain1 = 0.02;

//最適化用のパラメータ
float p_gain_max = 10.0;
float p_gain_min = 0.1;
float max_iterations = 10000; //最大調整回数 
float sensor0_prev;
//最適ゲイン(2025/04/01)
float p_gain2 = 1.01;
float i_gain2 = 0.03;
float d_gain2 = 0.03;

//システムの応答を評価するための変数
float previous_error = 0;
float integral = 0;
unsigned long previous_time = 0;

bool tuning_in_progress = false;

float Error_force1_pre = 0, Error_force2_pre = 0;
float force1_integral = 0, force2_integral = 0;
float force1_derivative = 0, force2_derivative = 0;

float sensor0Value, sensor1Value;
float oldsensor0;
float oldsensor1;
float target_tension = 0;

TaskHandle_t TensionControlTaskHandle = NULL; //タスクハンドル

int16_t baseCountEnc1 = 0; //エンコーダ1の基準値
int16_t baseCountEnc2 = 0; //エンコーダ2の基準値
int16_t countEnc1 = 0; 
int16_t countEnc2 = 0;
const int Motor_speed = 50; //モータの移動速度(PWM値)

//Pin assign
int DIRR = 25;
int SLPR = 21;
int PWMR = 0;

int DIRL = 14;
int SLPL = 32;
int PWML = 12;

char lan;

int Setup = 0;
int Home = 0;
int Move = 0;

//マルチスレッドのタスクハンドル格納
TaskHandle_t Process[2];

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
      Serial.print("Forward.");
    break;
    case 'B':
      Serial.print("Back.");
    break;
    case 'E':
      Serial.print("Left pivot turn.");
    break;
    case 'R':
      Serial.print("Left super pivot turn.");
    break;
    case 'T':
      Serial.print("Right pivot turn.");
    break;
    case 'Y':
      Serial.print("Right super pivot turn.");
    break;
    case 'S':
      Serial.print("Stop.");
    break;

    case 'Z': //目標履帯張力にする初期動作
      Home = 0;
      Setup = 1;
      Move = 0;
      Serial.println("Tension control start.");
    break;

    case 'H': //原点復帰
      Setup = 0;
      Home = 1;
      Move = 0;
      Serial.println("Return to origin.");
    break;

    case 'P': //All stop
      Setup = 0;
      Home = 0;
      Move = 0;
      Serial.println("All System stop.");
    break;

    case 'K':
      tuning_in_progress = true;
      Serial.println("AutoTune started");
    break;

    case 'M':
      Serial.println("Enter target tension : ");
      while(Serial.available()==0)
      {
        vTaskDelay(5000 / portTICK_PERIOD_MS);
      }
      target_tension = Serial.parseFloat(); //目標張力を設定
      Serial.print("Target Tension reset to : ");
      Serial.println(target_tension);

      vTaskDelay(500 / portTICK_PERIOD_MS);
    break;

    default:
      Serial.println("Warning command error.");
    break;
  }
}

void setup() {
  Serial.begin( 115200);
  Serial1.begin( 500000, SERIAL_8N1, 18, 19);
  pinMode(ENCODER2_A, INPUT_PULLUP);
  pinMode(ENCODER2_B, INPUT_PULLUP);
  setupPCNT();
  xTaskCreatePinnedToCore( Tension_controlTask, "TensionControlTask", 8192, NULL, 2, &Process[0], 1);
  xTaskCreatePinnedToCore( moveToHomePosition, "ReturnToOriginalTask", 8192, NULL, 0, &Process[1], 0);
  pin_set();
}

void loop() {
    //目標張力の設定処理
  Serial.println("Enter target tension : ");
  while(Serial.available()==0)
  {
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
  target_tension = Serial.parseFloat(); //目標張力を設定
  Serial.print("Target Tension set to : ");
  Serial.println(target_tension);

  vTaskDelay(500 / portTICK_PERIOD_MS);
  //コマンド入力モードに移行
  Serial.print("Input command : "); //最初のプロンプト表示
  while(1)
  {
    while(!Serial.available()) //入力がある場合のみ処理
    {
      vTaskDelay(10 / portTICK_PERIOD_MS);
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


    vTaskDelay(10 / portTICK_PERIOD_MS);
    }

}

void Tension_controlTask( void *Process)
{
  while(1)
  {
  while( Setup == 0)
  {
    digitalWrite( SLPL, LOW);
    digitalWrite( DIRL, LOW);
    digitalWrite( SLPR, LOW);
    digitalWrite( DIRR, LOW);   
    ledcWrite( PWML, 0);
    ledcWrite( PWMR, 0);     
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
  while(Setup == 1)
  {
    //UARTデータを読む
    read_uart();
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
      digitalWrite( SLPL, HIGH);
      digitalWrite( DIRL, LOW); //適宜変更する(> or <)
      ledcWrite( PWML, pwm_force1);
    }else if( target_tension < sensor0Value)
    {
      digitalWrite( SLPL, HIGH);
      digitalWrite( DIRL, HIGH); //適宜変更する(> or <)
      ledcWrite( PWML, pwm_force1);
    }
    
    if( target_tension > sensor1Value)
    {
      digitalWrite( SLPR, HIGH);
      digitalWrite( DIRR, LOW); //適宜変更する(> or <)
      ledcWrite( PWMR, pwm_force2);
    }else if( target_tension < sensor1Value)
    {
      digitalWrite( SLPR, HIGH);
      digitalWrite( DIRR, HIGH); //適宜変更する(> or <)
      ledcWrite( PWMR, pwm_force2);
    }
    
    //エラー更新
    Error_force1_pre = Error_force1;
    Error_force2_pre = Error_force2;  

    if(tuning_in_progress)
    {
      Serial.println("AutoTunePID running...");
      AutoTunePID();
    }
    if(!tuning_in_progress)
    {
        Serial.print("Final P Gain:");
        Serial.println(p_gain1);
        Serial.print("Final I Gain:");
        Serial.println(i_gain1);
        Serial.print("Final D Gain");
        Serial.println(d_gain1);
    }

    vTaskDelay(2 / portTICK_PERIOD_MS);
    }

    vTaskDelay(2 / portTICK_PERIOD_MS);
  }
}

void AutoTunePID() {
    Serial.println("AutoTunePID running...");

    float system_response = GetSystemResponse();

    float p_gain_min = 0.01, p_gain_max = 20.0;
    float i_gain_min = 0.001, i_gain_max = 5.0;
    float d_gain_min = 0.001, d_gain_max = 5.0;

    float error = abs(target_tension - sensor0Value);
    static float best_error = 99999;  // 最小偏差（大きな値で初期化）
    static float best_p = 0, best_i = 0, best_d = 0;
    static float integral = 0;
    static float previous_error = 0;
    static unsigned long previous_time = millis();
    static unsigned long last_tune_time = 0;

    // 直前のゲイン値
    static float prev_p = p_gain1, prev_i = i_gain1, prev_d = d_gain1;

    unsigned long current_time = millis();
    float dt = (current_time - previous_time) / 1000.0;
    if (dt < 0.01) return;

    float derivative = (error - previous_error) / dt;
    integral += error * dt;
    integral = constrain(integral, -i_gain_max, i_gain_max);

    unsigned long tuning_interval = 1000;
    if (millis() - last_tune_time > tuning_interval) {
        last_tune_time = millis();

        if (error < best_error) {
            p_gain1 += 0.1 * (1.0 - error / best_error);
            best_error = error;
            best_p = p_gain1;
            best_i = i_gain1;
            best_d = d_gain1;
        } else {
            p_gain1 *= 0.95;
        }

        p_gain1 = constrain(p_gain1, p_gain_min, p_gain_max);

        if (system_response > 0.5) {
            i_gain1 += 0.01 * system_response;
            d_gain1 += 0.01 * system_response;
        } else {
            i_gain1 -= 0.01 * system_response;
            d_gain1 -= 0.01 * system_response;
        }

        i_gain1 = constrain(i_gain1, i_gain_min, i_gain_max);
        d_gain1 = constrain(d_gain1, d_gain_min, d_gain_max);

        Serial.print("Updated P Gain: ");
        Serial.println(p_gain1);
    }

    // 変化が小さくなったら終了
    if (abs(p_gain1 - prev_p) < 0.000001 && abs(i_gain1 - prev_i) < 0.000001 && abs(d_gain1 - prev_d) < 0.000001) {
        Serial.println("Auto-tuning converged. Stopping tuning.");
        Setup=0;
        return;
    }

    // ゲインの前回値を更新
    prev_p = p_gain1;
    prev_i = i_gain1;
    prev_d = d_gain1;

    previous_error = error;
    previous_time = current_time;
    sensor0_prev = sensor0Value;

    Serial.println("Best Gains:");
    Serial.print("Best P Gain: ");
    Serial.println(best_p);
    Serial.print("Best I Gain: ");
    Serial.println(best_i);
    Serial.print("Best D Gain: ");
    Serial.println(best_d);
}


float GetSystemResponse()
{
 // read_uart();
  float error = target_tension - sensor0Value;
  //float Error_force2 = target_tension - sensor1Value;  
  static float previous_error = 0;
  static unsigned long previous_time = 0;

  unsigned long current_time = millis();
  float error_rate = ( error - previous_error) / ( current_time - previous_time);

  float system_response = abs(error_rate);

  previous_error = error;
  previous_time = current_time;

  return system_response;
}


void read_uart() //確認済み
{
  Serial1.print("L");
  if( Serial1.available())
  {
    String receivedData = Serial1.readStringUntil('\n');
    
    //データの分割
    int sensor0Index = receivedData.indexOf("sensor0");
    int sensor1Index = receivedData.indexOf("sensor1");

    //Serial1.printf("sensor0: %.2f, sensor1: %.2f", tensionValues[0], tensionValues[1]);

    if(sensor0Index != -1 && sensor1Index != -1)
    {
      int commandIndex = receivedData.indexOf(","); //カンマの位置を取得
    if( commandIndex != -1)
    {
      //sensor0の値を抽出
      sensor0Value = receivedData.substring(sensor0Index + 8, commandIndex).toFloat();
      //sensor0Value = receivedData.substring(sensor0Index + 8).toFloat();
      //sensor1の値を抽出
      sensor1Value = receivedData.substring(sensor1Index + 8).toFloat();
  
      Serial.print("sensor0:");
      Serial.print(sensor0Value);
      Serial.print(", sensor1:");
      Serial.println(sensor1Value);

    }
    else
    {
      Serial.println("Warning Invalid UARTformat");
    }
    }
  }
}

void setupPCNT()
{
    // エンコーダAの設定
    pcnt_config_t pcnt_config_A = {};
    pcnt_config_A.pulse_gpio_num = ENCODER1_A;
    pcnt_config_A.ctrl_gpio_num = ENCODER1_B;
    pcnt_config_A.lctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config_A.hctrl_mode = PCNT_MODE_KEEP;
    pcnt_config_A.channel = PCNT_CHANNEL_0;
    pcnt_config_A.unit = PCNT_UNIT_ENCA;
    pcnt_config_A.pos_mode = PCNT_COUNT_DIS;
    pcnt_config_A.neg_mode = PCNT_COUNT_INC;
    pcnt_config_A.counter_h_lim = 32767;
    pcnt_config_A.counter_l_lim = -32767;
    pcnt_unit_config(&pcnt_config_A);

    // エンコーダBの設定
    pcnt_config_t pcnt_config_B = {};
    pcnt_config_B.pulse_gpio_num = ENCODER2_A;
    pcnt_config_B.ctrl_gpio_num = ENCODER2_B;
    pcnt_config_B.lctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config_B.hctrl_mode = PCNT_MODE_KEEP;
    pcnt_config_B.channel = PCNT_CHANNEL_1;
    pcnt_config_B.unit = PCNT_UNIT_ENCB;
    pcnt_config_B.pos_mode = PCNT_COUNT_INC;
    pcnt_config_B.neg_mode = PCNT_COUNT_DIS;
    pcnt_config_B.counter_h_lim = 32767;
    pcnt_config_B.counter_l_lim = -32767;
    pcnt_unit_config(&pcnt_config_B);

    // カウンタ初期化
    pcnt_counter_pause(PCNT_UNIT_ENCA);
    pcnt_counter_pause(PCNT_UNIT_ENCB);
    pcnt_counter_clear(PCNT_UNIT_ENCA);
    pcnt_counter_clear(PCNT_UNIT_ENCB);
    pcnt_counter_resume(PCNT_UNIT_ENCA);
    pcnt_counter_resume(PCNT_UNIT_ENCB);
}

void moveToHomePosition( void *Process)
{
  int flag1 = 0;
  int flag2 = 0;
  float T = 0.01;
  float p_gain1 = 0.2;
  float i_gain1 = 0.1;
  float d_gain1 = 0.01;
  float p_gain2 = 0.2;
  float i_gain2 = 0.1;
  float d_gain2 = 0.01;

  float error1_pre = 0, error2_pre = 0;
  float position1_integral = 0, position2_integral = 0;
  float position1_derivative = 0, position2_derivative = 0; 

  TickType_t xLastWakeTime = xTaskGetTickCount();

  while(1)
  {
    while( Home  == 0){
      flag1 = 0;
      flag2 = 0;
      pcnt_get_counter_value(PCNT_UNIT_ENCA, &countEnc1);
      pcnt_get_counter_value(PCNT_UNIT_ENCB, &countEnc2);
      vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    while( Home == 1)
    {
      pcnt_get_counter_value(PCNT_UNIT_ENCA, &countEnc1); //EncoderA -> count1
      pcnt_get_counter_value(PCNT_UNIT_ENCB, &countEnc2); //EncoderB -> count2
      int error1 = baseCountEnc1 - countEnc1;
      int error2 = baseCountEnc2 - countEnc2;
      position1_integral += error1 * T;
      position2_integral += error2 * T;
      position1_derivative = ( error1 - error1_pre) / T;
      position2_derivative = ( error2 - error2_pre) / T;
      float position1_control_signal = p_gain1 * error1 + i_gain1 * position1_integral + d_gain1 * position1_derivative;
      float position2_control_signal = p_gain2 * error2 + i_gain2 * position2_integral + d_gain2 * position2_derivative;

      int pwm_position1 = constrain( abs( position1_control_signal), 0, 25);
      int pwm_position2 = constrain( abs( position2_control_signal), 0, 25);

      if( (error1 > 0) && ( flag1 == 0)) //縮める方
      {
        digitalWrite( SLPL, HIGH);
        digitalWrite( DIRL, LOW); //適宜変更する(> or <)
        ledcWrite( PWML, pwm_position1);
      }else if( (error1 < 0) && ( flag1 == 0)) //伸ばす方
      {
        digitalWrite( SLPL, HIGH);
        digitalWrite( DIRL, HIGH); //適宜変更する(> or <)
        ledcWrite( PWML, pwm_position1);
      }
      
      if( (error2 > 0) && ( flag2 == 0)) //縮める方
      {
        digitalWrite( SLPR, HIGH);
        digitalWrite( DIRR, LOW); //適宜変更する(> or <)
        ledcWrite( PWMR, pwm_position2);
      }else if( (error2 < 0) && ( flag2 == 0)) //伸ばす方
      {
        digitalWrite( SLPR, HIGH);
        digitalWrite( DIRR, HIGH); //適宜変更する(> or <)
        ledcWrite( PWMR, pwm_position2);
      }
    
      if( abs( error1) < 10)
      {
        digitalWrite( SLPL, LOW);
        digitalWrite( DIRL, LOW);
        ledcWrite( PWML, 0);
        flag1 = 1;
        break;
      }

      if( abs( error2) < 10)
      {
        digitalWrite( SLPR, LOW);
        digitalWrite( DIRR, LOW);
        ledcWrite( PWMR, 0);
        flag2 = 1;
      }
      if( (flag1 == 1) && ( flag2 == 1))
      {
        Serial.println("Reached Home position.");
        flag1 = 0;
        flag2 = 0;
        break;
      }
      //エラー更新
      error1_pre = error1;
      error2_pre = error2;

      vTaskDelayUntil(&xLastWakeTime, 10 / portTICK_PERIOD_MS);
    }
      vTaskDelayUntil(&xLastWakeTime, 10 / portTICK_PERIOD_MS);
  }
}