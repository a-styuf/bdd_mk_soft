#ifndef _DAC_H_
#define _DAC_H_

#include <math.h>
#include "1986ve8_lib/cm4ikmcu.h"

#define DAC_CHAN_CAL_COEF_A {+1.223E+3, +1.223E+3}  // DAC_CODE = a*U(V) + b
#define DAC_CHAN_CAL_COEF_B {-7.154E+0, -7.154E+0}

#pragma pack (2)

/** 
  * @brief  структура управления каналоми ЦАП
  */
typedef struct
{
  DACxControl* reg;
  uint16_t val; //значение, прописанное в регистр ЦАП
	float voltage; //установленное значение в В на выходе ЦАП
	float a, b; // калибровочные коэффициенты voltage = a*value + b
} type_DAC_channel;

/** 
  * @brief  структура програмная модель АЦП
  */
typedef struct
{
	type_DAC_channel ch[2];
} type_DAC_model;

//
void dac_init(type_DAC_model* adc_ptr);
void dac_set_ch_a_b(type_DAC_model* adc_ptr, float *a, float *b);
void dac_set_voltage(type_DAC_model* dac_ptr, uint8_t ch_num, float voltage);
void dac_set_code(type_DAC_model* dac_ptr, uint8_t ch_num, uint16_t code);
#endif



