#ifndef _OAI_DD_H_
#define _OAI_DD_H_

#include "1986ve8_lib/cm4ikmcu.h"
#include "termo_res.h"
#include "adc.h"
#include "dac.h"

// Подсчет тока на лампочке (диффиренциальное включение ОУ: U_out = dUin*(R1/R2))
#define I_R_SHUNT 51.0  // измерительное сопротивление для ткоа
#define I_R1_FB_AMPL 10.0E4  // сопротивление обратной связи
#define I_R2_FB_AMPL 51.0E3  // сопротивление во вход ОУ
#define I_K_FB_AMPL (I_R1_FB_AMPL/I_R2_FB_AMPL)  // кожффициент усиления обратной связи
#define I_A (1.0/(I_K_FB_AMPL*I_R_SHUNT)) // кожффициент пересчетв напряжения АЦП в ток
#define I_B 0.0
// Подсчет напряжения на лампочке (Включение с положительной обратной связью: Uout = Uin(1+R1/R2))
#define V_R1_FB_AMPL 39.0E3  // сопротивление обратной связи
#define V_R2_FB_AMPL 10.0E3  // сопротивление в землю
#define V_K_FB_AMPL (1+(V_R1_FB_AMPL/V_R2_FB_AMPL))  // кожффициент усиления обратной связи
#define V_A (1.0/(1+(V_R1_FB_AMPL/V_R2_FB_AMPL))) // кожффициент пересчетв напряжения АЦП в напряжение на зонде
#define V_B 0.0
// PID Resist-stabilisation
#define PID_RES_K 0.01
#define PID_RES_P 1
#define PID_RES_D 0.0001
#define PID_RES_I 0.001
#define PID_RES_REACTION_MAX_V 0.1
//
#define DESIRED_RESISTANCE 60
//
#define DD_DAC_MAX_VOLTAGE (2.2)
#define DD_DAC_MIN_VOLTAGE (0.6)

#pragma pack(2)
/** 
  * @brief  структура управления ПИД
  */
typedef struct
{
  float K_coeff, P_coeff, I_coeff, D_coeff;
  float P_reaction, I_reaction, D_reaction;
  float integral;
  float value, value_prev, derivate;
  float value_desired, error;
  float reaction_max;
  float reaction;
} type_PID_model;

/** 
  * @brief  структура управления каналом oai_dd
  */
typedef struct
{
  uint8_t num;
  type_TRES_model* t_res_ptr;
  type_ADC_channel* adc_v;
  type_ADC_channel* adc_i;
  type_DAC_channel* dac;
  type_PID_model pid_res, pid_current; 
  float dac_voltage;
  float v_a, v_b;
  float curr_a, curr_b;
  float pr_voltage, pr_current, pr_res;
  float pressure;
  uint16_t pressure_u16;
  float temp;
} type_OAI_DD_model;

//
int8_t oai_dd_init(type_OAI_DD_model* oai_dd_ptr, uint8_t num, type_TRES_model* t_res_ptr, type_ADC_channel* adc_ch_v_ptr, type_ADC_channel* adc_ch_i_ptr, type_DAC_channel* dac_ch_ptr, float v_a, float v_b, float curr_a, float curr_b);
void oai_dd_reset_val(type_OAI_DD_model* oai_dd_ptr, uint8_t num, type_TRES_model* t_res_ptr, type_ADC_channel* adc_ch_v_ptr, type_ADC_channel* adc_ch_i_ptr, type_DAC_channel* dac_ch_ptr, float v_a, float v_b, float curr_a, float curr_b);
void oai_dd_process(type_OAI_DD_model* oai_dd_ptr, uint16_t period_ms);
void oai_dd_pid_resistance(type_OAI_DD_model* oai_dd_ptr, uint16_t period_ms);
float oai_dd_get_voltage(type_OAI_DD_model* oai_dd_ptr);
float oai_dd_get_current(type_OAI_DD_model* oai_dd_ptr);
float oai_dd_get_temp(type_OAI_DD_model* oai_dd_ptr);
float oai_dd_get_pressure(type_OAI_DD_model* oai_dd_ptr);
float oai_dd_get_resistance(type_OAI_DD_model* oai_dd_ptr);
uint8_t oai_dd_get_str_report(type_OAI_DD_model* oai_dd_ptr, char* report);
//
void pid_init(type_PID_model* pid_ptr, float K, float P, float D, float I, float reaction_max);
void pid_reset(type_PID_model* pid_ptr);
void pid_set_desired_value(type_PID_model* pid_ptr, float desired_value);
void pid_set_coeff(type_PID_model* pid_ptr, float K, float P, float D, float I);
float pid_step_calc(type_PID_model* pid_ptr, float value, uint16_t period_ms);
uint8_t pid_get_str_report(type_PID_model* pid_ptr, char* report);

#endif



