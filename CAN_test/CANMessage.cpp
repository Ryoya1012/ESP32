//CANMessage.cpp
#include "CANMessage.h"
#include <cstdint>   // uint32_t、uint8_t など
#include <cstddef>   // size_t


void setCANMessageData(CANMessage &msg, uint32_t id, uint8_t *data, size_t length){
	msg.id_msg = id;
	for( size_t i = 0; i < length && i < sizeof(msg.data); i++){
		msg.data[i] = data[i];
  }
}
CANMessage msg1;
CANMessage msg2;


