#ifndef _IMS_H_
#define _IMS_H_

#include "1986ve8_lib/cm4ikmcu.h"
#include "termo_res.h"
#include "adc.h"
#include "dac.h"
#include "digital_filter.h"
#include "mvip.h"
#include "gpio.h"
//
#define IMS_MODE_OFF      0x00
#define IMS_MODE_SIMPLE   0x01
#define IMS_MODE_CALIBR   0x02
#define IMS_MODE_DEFAULT  IMS_MODE_SIMPLE
// oai_dd_status fields
#define IMS_PID_OK      1<<0
//
#define IMS_KU_1        0x00
#define IMS_KU_10       0x01
#define IMS_KU_100      0x02
#define IMS_KU_1000     0x03
#define IMS_KU_DEFAULT  IMS_KU_1
// oai_dd_mean_settings
#define IMS_TIME_S      1.0
// voltage filter settings
#define IMS_U_FILTER_T_CONST    1.
#define IMS_U_FILTER_T_SAMPLE   0.01
// temp filter settings
#define IMS_T_FILTER_T_CONST    1.
#define IMS_T_FILTER_T_SAMPLE   0.01
// ku_change_bound
#define IMS_TOP_VOLTAGE    3.0
#define IMS_BOT_VOLTAGE    0.3
#define IMS_PR_MIN_VOLTAGE    0.1
// Measurement timeout
#define  IMS_KU_DEAD_TIME_MS 1000
#define  IMS_CALIBRATION_TIME_MS 1000
// Rele state
#define  IMS_RELE_MEAS 0x01
#define  IMS_RELE_CALIB 0x00


#pragma pack(2)
/** 
  * @brief  frame report 26-byte
  */
typedef struct
{
  uint8_t state;            //+1
  uint8_t mode;             //+0
  int16_t pressure;         //+2
  int16_t temp;             //+4 °C/256
  int16_t voltage;          //+6 V/256
  uint16_t reserve[18];      //+8
} type_IMS_Frame_Report;    //26

/** 
  * @brief  структура управления каналом oai_dd
  */
typedef struct
{
  type_TRES_model* t_res_ptr;
  type_ADC_channel* adc_v;
  type_MVIP* mvip;
  type_SINGLE_GPIO *rele_gpio, *ku_gpio_0, *ku_gpio_1;
  type_DFilter_model filter_u, filter_temp;
  float v_a[4], v_b[4];
  float meas_voltage[4], pr_voltage[4];
  float curr_a[4], curr_b[4];
  uint8_t ku;
  uint16_t ku_dead_time;
  float pressure;
  uint16_t pressure_u16;
  float temp;
  uint8_t mode;
  uint8_t state;
  // calibration variables
  float zero_voltage[4];
  uint16_t calibration_clock_ms;
  uint8_t calibration_mode;
  //
  type_IMS_Frame_Report report;
} type_IMS_model;

//
int8_t ims_init(type_IMS_model* ims_ptr, type_TRES_model* t_res_ptr, type_ADC_channel* adc_ch_v_ptr, type_MVIP* mvip_ptr, type_SINGLE_GPIO *rele, type_SINGLE_GPIO *ku0, type_SINGLE_GPIO *ku1);
void ims_reset_val(type_IMS_model* ims_ptr);
void ims_set_mode(type_IMS_model* ims_ptr, uint8_t mode);
void ims_set_ku(type_IMS_model* ims_ptr, uint8_t ku);
void ims_process(type_IMS_model* ims_ptr, uint16_t period_ms);
void ims_simple_process(type_IMS_model* ims_ptr, uint16_t period_ms);
void ims_calibr_process(type_IMS_model* ims_ptr, uint16_t period_ms);
int8_t ims_range_change_checking(type_IMS_model* ims_ptr, float meas_voltage, float pr_voltage);
void ims_range_change(type_IMS_model* ims_ptr, uint16_t period_ms);
float ims_get_voltage(type_IMS_model* ims_ptr);
float ims_get_current(type_IMS_model* ims_ptr);
float ims_get_temp(type_IMS_model* ims_ptr);
float ims_get_pressure(type_IMS_model* ims_ptr);
uint8_t ims_get_str_report(type_IMS_model* ims_ptr, char* report);
void ims_get_frame_report(type_IMS_model* ims_ptr);
#endif



