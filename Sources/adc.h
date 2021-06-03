#ifndef _ADC_H_
#define _ADC_H_

#include "1986ve8_lib/cm4ikmcu.h"
#include <string.h>
#include "debug.h"

#define IRQn_ADC0 (IRQn_Type)119

#define ADC0_CHAN_NUM 24
#define ADC0_CHAN_TEMP 23

// из примера от Миландр
#define FACTORY_ADC_TEMP25      1700.      // ADC value = 1700 @ 25C = 1.36996V
#define FACTORY_ADC_AVG_SLOPE      6.      // ADC delta value @ 1C, from milandr demo project
#define FACTORY_TEMP25            25.

#define ADC_CHAN_CAL_COEF_A {5.124E-5, 5.124E-5, 5.124E-5, 5.124E-5, 5.124E-5, 5.124E-5, 5.124E-5, 5.124E-5, 5.124E-5, 5.124E-5}
#define ADC_CHAN_CAL_COEF_B {2.070E-3, 2.070E-3, 2.070E-3, 2.070E-3, 2.070E-3, 2.070E-3, 2.070E-3, 2.070E-3, 2.070E-3, 2.070E-3}

// for mean calculation
#define ADC_CHAN_MEAN_SIZE_2_POW 5
#define ADC_CHAN_MEAN_SIZE (1<<ADC_CHAN_MEAN_SIZE_2_POW)

#pragma pack(2)
/** 
  * @brief  структура управления каналом АЦП
  */
typedef struct
{
  uint16_t val;  // adc - 12 бит
  uint16_t val_mean;   // adc - 16 бит
  uint16_t val_buff[ADC_CHAN_MEAN_SIZE];
  uint32_t val_buff_wr_ptr;
  
	float voltage, inst_voltage; //пересчитанное значение в В на входе АЦП
	float a, b; // калибровочные коэффициенты voltage = a*value + b
  uint8_t adc_ch_num;
} type_ADC_channel;

/** 
  * @brief  структура програмная модель АЦП
  */
typedef struct
{
  ADCxControl* reg;
	type_ADC_channel ch[ADC0_CHAN_NUM];
  uint16_t adc_val_arr[ADC0_CHAN_NUM];
  float temp;
} type_ADC_model;

//
void adc_init(type_ADC_model* adc_ptr);
void adc_set_ch_a_b(type_ADC_model* adc_ptr, float *a, float *b);
float adc_ch_voltage(type_ADC_model* adc_ptr, uint8_t ch_num);
float adc_ch_inst_voltage(type_ADC_model* adc_ptr, uint8_t ch_num);
float calc_mcu_temp(type_ADC_model* adc_ptr);
void adc_process(type_ADC_model* adc_ptr, uint16_t period_ms);
float adc_get_ch_voltage(type_ADC_channel* adc_ch_ptr);
float adc_get_ch_inst_voltage(type_ADC_channel* adc_ch_ptr);
float get_mcu_temp(type_ADC_model* adc_ptr);
void adc_new_val_process(type_ADC_channel* adc_ch_ptr, uint16_t new_val);
void _adc_new_val_process(type_ADC_model* adc_ptr);
//
void INT_ADC0_Handler(void);
__weak void INT_ADC0_CallBack(void);
#endif



