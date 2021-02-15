#ifndef _BDD_MPI_INTERFACE_H_
#define _BDD_MPI_INTERFACE_H_

#include "1986ve8_lib/cm4ikmcu.h"


// структуры кадров
#pragma pack(2)
typedef struct // ситстемный кадр
{
  uint16_t label;  //+0
  uint16_t definer; //+2 
  uint16_t num; //+4
  uint32_t time; //+6
  // основные параметры
  uint16_t press_general; //+10
  uint16_t temp_general; //+12
  uint16_t current_24v; //+14
  // полные измерения
  uint16_t pressure[3]; //+16
  uint16_t temp[3]; //+22
  uint8_t mode; //+28
  uint8_t state; //+29
  uint8_t error; //+30
  uint8_t error_cnt; //+31
  uint16_t filler[15]; //+32
  // 
  uint16_t crc16; //+62 30
}typeSysFrames;

typedef struct // подробный кадр с работой ОАИ ДД
{
  uint16_t label;  //+0
  uint16_t definer; //+2 
  uint16_t num; //+4
  uint32_t time; //+6
  // основные параметры
  uint16_t oai_dd_1_report[13]; //+10
  uint16_t oai_dd_2_report[13]; //+36
  // 
  uint16_t crc16; //+62 30
}typeOAIDDFrames;
//

// прототипы функций
uint16_t mpi_int_frame_definer(uint8_t frame_modification, uint16_t device_number,  uint16_t fabrication_num, uint8_t frame_type);
uint16_t mpi_int_crc16(uint8_t *buf, uint8_t len);

#endif
