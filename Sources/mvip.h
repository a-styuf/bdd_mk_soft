#ifndef _MVIP_H_
#define _MVIP_H_

#include <math.h>

#include "1986ve8_lib/cm4ikmcu.h"

#include "timers.h"
#include "adc.h"
#include "gpio.h"
#include "debug.h"
#include "oai_dd.h"  //todo: перенести pid в отдельный файл

//defines

// PID Current-stabilisation
#define PID_K 1.0
#define PID_P 0.001
#define PID_D 0.0
#define PID_I 0.0
#define MVIP_PID_MAX_REACTION 1

#define MVIP_VOLTAGE_DEFAULT 2500
#define MVIP_MAX_V 2800
#define MVIP_VOLTAGE_MAX_ERROR 50

#define MVIP_A 1.407E3   // рассчетное 1406
#define MVIP_B 1.12      // рассчетное 0.0

#define MVIP_I24_A 0.2
#define MVIP_I24_B 0.0

// states
#define MVIP_STATE_HV    1<<0
#define MVIP_STATE_PWR   1<<1

// mode
#define MVIP_MODE_OFF  0<<0
#define MVIP_MODE_ON   1<<0

// data structures
#pragma pack(2)
typedef struct  // программная модель управления БДД
{
  uint16_t h_voltage;
  uint16_t current;
  uint16_t fb_voltage;
  uint16_t pwm_val;  // 0-999
}type_MVIP_frame_report;

typedef struct  // программная модель управления БДД
{
	uint8_t mode;
  uint8_t state;
  uint16_t pwm_val;  // 0-999
  float pwm_val_float;  // 0-999
  float v_hv, v_hv_desired;
  float i_24V, v_fb;
  type_ADC_channel* adc_v_fb;
  type_ADC_channel* adc_curr;
  type_SINGLE_GPIO hv_inh;
  type_PID_model pid;
  type_MVIP_frame_report report;
}type_MVIP;

int8_t mvip_init(type_MVIP* mvip_ptr, type_ADC_channel* adc_v_fb, type_ADC_channel* adc_i24);
void mvip_process(type_MVIP* mvip_ptr, uint16_t period_ms);
void mvip_set_mode(type_MVIP* mvip_ptr, uint8_t mode);
void mvip_set_voltage(type_MVIP* mvip_ptr, float voltage);
uint8_t mvip_get_str_report(type_MVIP* mvip_ptr, char* report);
void mvip_form_report(type_MVIP* mvip_ptr);

#endif
