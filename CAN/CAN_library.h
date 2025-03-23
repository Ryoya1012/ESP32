#ifndef CAN_library_H
#define CAN_library_H

#include <Arduino.h>

// CAN ID 定義用の構造体
struct CAN_IDs{
    uint16_t Tx_PDO01;
    uint16_t Tx_PDO02;
    uint16_t Tx_PDO03;
    uint16_t Tx_PDO04;

    uint16_t Rx_PDO01;
    uint16_t Rx_PDO02;
    uint16_t Rx_PDO03;
    uint16_t Rx_PDO04;
};

//データフレーム構造体
struct CAN_Data{
    uint8_t Profile_Velocity[4];
    uint8_t Profile_Velocity_re[4];
    uint8_t OP3[1];
    uint8_t OP9[1];
    uint8_t OPA[1];
    uint8_t Conw1[2];
    uint8_t Conw2[2];
    uint8_t Conw3[2];
    uint8_t Conw4[2];
    uint8_t Conw5[2];
    uint8_t Conw6[2];
    uint8_t ResetP1[4];
    uint8_t PreOP[2];
    uint8_t Operation[2];
    uint8_t Reset_NMT[2];
    uint8_t Read_Current1[4];
    uint8_t Read_Current2[4];
    uint8_t PVM[4];
    uint8_t SCV[3];
};

//インスタンスの定義
extern const CAN_IDs Motor1_IDs;
extern const CAN_IDs Motor2_IDs;
extern const CAN_Data CAN_Commands;


//定義
constexpr uint8_t sync_byte = 0x80;
constexpr uint8_t Reset = 0x00;

#endif