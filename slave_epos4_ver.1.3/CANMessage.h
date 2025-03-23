#ifndef CANMESSAGE_H
#define CANMESSAGE_H

#include <cstdint>  // uint32_t, uint8_t を使用するためにインクルード

// CANMessage構造体の宣言
struct CANMessage {
    uint32_t id_msg;
    uint8_t data[8];  // データ部分（8バイト）
};

// msg1 と msg2 を外部変数として宣言
extern CANMessage msg1;
extern CANMessage msg2;

#endif
