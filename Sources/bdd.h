#ifndef _BDD_H_
#define _BDD_H_

#include "1986ve8_lib/cm4ikmcu.h"

#include "mpi.h"
#include "timers.h"
#include "adc.h"
#include "dac.h"
#include "gpio.h"
#include "termo_res.h"
#include "oai_dd.h"
#include "uarts.h"
#include "debug.h"

#pragma pack(2)
//----МПИ кадры
typedef struct // ситстемный кадр
{
  uint16_t label;  //+0
  uint16_t definer; //+2 
  uint16_t num; //+4
  uint32_t time; //+6
  // основные параметры
  uint16_t press_general; //+10
  uint16_t temp; //+12
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

typedef struct // ситстемный кадр
{
  uint16_t label;  //+0
  uint16_t definer; //+2 
  uint16_t num; //+4
  uint32_t time; //+6
  // основные параметры
  uint16_t filler[26]; //+10
  // 
  uint16_t crc16; //+62 30
}typeOAIDDFrames;

//----
typedef struct  // программная модель управления БДД
{
	uint8_t mode;
  uint8_t state;
  uint8_t error, error_cnt;
  uint32_t time_slot_cnter;
}type_BDD_control;


typedef struct  // программная модель управления БДД
{
  type_ADC_model adc_0;
  type_DAC_model dac;
  type_TRES_model tres_ims, tres_pirani_1, tres_pirani_2;
  type_OAI_DD_model oai_dd_1, oai_dd_2;
  type_MPI_model* mpi_ptr;
  type_SINGLE_GPIO rele_gpio, ku_gpio[2];
  //
  type_BDD_control control;
}type_BDD_model;

int8_t bdd_init(type_BDD_model* bdd_ptr);
void bdd_process(type_BDD_model* bdd_ptr, uint8_t period_ms);

#endif
