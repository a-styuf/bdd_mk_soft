#ifndef _OAI_DD_H_
#define _OAI_DD_H_

#include "1986ve8_lib/cm4ikmcu.h"

/** 
  * @brief  структура управления каналом oai_dd
  */
typedef struct
{
  float voltage_in, voltage_out;
  float current, resistance;
  float pressure;
  uint16_t pressure_u16;
  float temp;
} type_OAI_DD_model;

//

#endif



