/**
  ******************************************************************************
  * @file           : mvip.c
  * @version        : v1.0
  * @brief          : библиотека для работы с програмной моделью БДД
  * @author			: Стюф Алексей/Alexe Styuf <a-styuf@yandex.ru>
  ******************************************************************************
  */

#include "mvip.h"

/**
  * @brief  иинициализация mvip
  * @param  mvip_ptr указатель на програмную модель устройства
  */
int8_t mvip_init(type_MVIP* mvip_ptr, type_ADC_channel* adc_v_fb, type_ADC_channel* adc_i24)
{
  //
  mvip_ptr->mode = MVIP_MODE_OFF;
  mvip_ptr->state = 0;
  mvip_ptr->pwm_val = 0;
  mvip_ptr->pwm_val_float = 0;
  //
	mvip_ptr->hv_inh.port = PORTE;
	mvip_ptr->hv_inh.num = 17;
  gpio_set(&mvip_ptr->hv_inh, 1);
  //
  mvip_ptr->adc_v_fb = adc_v_fb;
  mvip_ptr->adc_curr = adc_i24;
  //
  pid_init(&mvip_ptr->pid, PID_K, PID_P, PID_D, PID_I, MVIP_PID_MAX_REACTION);
  mvip_set_voltage(mvip_ptr, MVIP_VOLTAGE_DEFAULT);
  //
  return 1;
}

/**
  * @brief  обработка состояния mvip
  * @param  mvip_ptr указатель на програмную модель устройства
  */
void mvip_process(type_MVIP* mvip_ptr, uint16_t period_ms)
{
  float pwm_step;
  //
  mvip_ptr->v_fb = adc_get_ch_voltage(mvip_ptr->adc_v_fb);
  mvip_ptr->v_hv = MVIP_A*mvip_ptr->v_fb - MVIP_B;
  //
  mvip_ptr->i_24V = MVIP_I24_A*adc_get_ch_voltage(mvip_ptr->adc_curr) + MVIP_I24_B; 
  //
  pid_set_desired_value(&mvip_ptr->pid, mvip_ptr->v_hv_desired);
  if (MVIP_MAX_V < mvip_ptr->v_hv){
    mvip_ptr->pwm_val_float = 0;
  }
  else{
    pwm_step = pid_step_calc(&mvip_ptr->pid, mvip_ptr->v_hv, period_ms);
    mvip_ptr->pwm_val_float += pwm_step;
  }
  mvip_ptr->pwm_val = (uint16_t)mvip_ptr->pwm_val_float;
  //
  if (MVIP_VOLTAGE_MAX_ERROR >= fabs(mvip_ptr->pid.error)){
    mvip_ptr->state |= MVIP_STATE_HV;
  }
  else{
    mvip_ptr->state &= ~MVIP_STATE_HV;
  }
  //
  if (mvip_ptr->mode & MVIP_MODE_ON){
    gpio_set(&mvip_ptr->hv_inh, 0);
  }
  else{
    gpio_set(&mvip_ptr->hv_inh, 1);
  }
  //
  Timer_PWM_Set_Fp(mvip_ptr->pwm_val_float);
}

/**
  * @brief  установка режима работы
  * @param  mvip_ptr указатель на програмную модель устройства
  */
void mvip_set_mode(type_MVIP* mvip_ptr, uint8_t mode)
{
  mvip_ptr->mode = mode;
}

/**
  * @brief  установка выходного напряжения
  * @param  mvip_ptr указатель на програмную модель устройства
  */
void mvip_set_voltage(type_MVIP* mvip_ptr, float voltage)
{
  mvip_ptr->v_hv_desired = voltage;
  
}


/**
  * @brief  вывод отладочной информации в строку
  * @param  mvip_ptr указатель на програмную модель устройства
  * @param  report указатель на массив для отчета
  * @retval длина строки для отчета
  */
uint8_t mvip_get_str_report(type_MVIP* mvip_ptr, char* report)
{
  char report_str[128] = {0};
  sprintf(report_str, "MVIP: v_fb=%.2E v_hv=%.2E i_24v=%.2E pwm=%.1f%% md=0x%02X st=0x%02X", 
                      mvip_ptr->v_fb,
                      mvip_ptr->v_hv,
                      mvip_ptr->i_24V,
                      mvip_ptr->pwm_val/10.,
                      mvip_ptr->mode,
                      mvip_ptr->state); 
	memcpy(report, report_str, 127);
	report[127] = 0;
  return strlen(report_str);
}
