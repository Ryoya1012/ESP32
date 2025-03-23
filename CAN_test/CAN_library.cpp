#include "CAN_library.h"

//EPOS4 モータ1の CAN ID
const CAN_IDs Motor1_IDs = {
  0x181, 0x281, 0x381, 0x481,  //trance
  0x201, 0x301, 0x401, 0x501  //Receive
};

//EPOS4 モータ2の CAN ID
const CAN_IDs Motor2_IDs = {
  0x190, 0x290, 0x390, 0x490,
  0x220, 0x320, 0x420, 0x520
};

//CAN コマンドデータ
const CAN_Data CAN_Commands = {
   { 0xC4, 0x09, 0x00, 0x00 }, //Profile Velocity (2500rpm)
   { 0x3C, 0xF6, 0xFF, 0xFF }, //Profile Velocity Re (-2500rpm)
   { 0x03 }, //OP3(PVM)
   { 0x09 }, //OP9(CSV)
   { 0x0A }, //OPA(SCT)
   { 0x06, 0x00 }, //Conw1 (Shutdown)
   { 0x0F, 0x00 }, //Conw2 (Switch ON & enable)
   { 0x0B, 0x00 }, //Conw3 (Quick stop)
   { 0x25, 0x00 }, //Conw4 (Start homing actual value)
   { 0x0F, 0x00 }, //Conw5
   { 0x01, 0x00 }, //Conw6
   { 0x64, 0x00, 0x00, 0x00 }, //ResetP1
   { 0x80, 0x00 }, //PreOP
   { 0x01, 0x00 }, //Operation
   { 0x82, 0x00 }, //Reset_NMT
   { 0x00, 0x00, 0x00, 0x00 }, //Read_Current1
   { 0x00, 0x00, 0x00, 0x00 }, //Read_Current2
   { 0x03, 0x00, 0x64, 0x00 }, //PVM
   { 0x09, 0x00, 0x64 } //SCV
};