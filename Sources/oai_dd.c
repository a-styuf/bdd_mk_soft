  /**
  ******************************************************************************
  * @file           : oai_dd.c
  * @version        : v1.0
  * @brief          : библиотека для работы с каналом измерения ОАИ ДД
  * @author			: Стюф Алексей/Alexe Styuf <a-styuf@yandex.ru>
  ******************************************************************************
  */
  
#include "oai_dd.h"

/**
  * @brief  инициализация модели измерения канала ОАИ_ДД
  * @param  oai_dd_ptr указатель на програмную модель устройства
  * @param  t_res_ptr указатель на програмную модель измерителя температуры
  * @param  adc_ch_v_ptr указатель на програмную модель ацп для напряжения
  * @param  adc_ch_i_ptr указатель на програмную модель ацп для тока
  * @param  dac_ch_ptr указатель на програмную модель цап
  * @param  v_a коэффициент перевод напряжения АЦП в напряжение на лампе V=v_a*V_adc+v_b
  * @param  v_b коэффициент перевод напряжения АЦП в напряжение на лампе
  * @param  curr_a коэффициент перевод напряжения АЦП в ток на лампе I=curr_a*V_adc+curr_b
  * @param  curr_b коэффициент перевод напряжения АЦП в ток на лампе 
  * @retval результат инициализации
  */
int8_t oai_dd_init(type_OAI_DD_model* oai_dd_ptr, uint8_t num, type_TRES_model* t_res_ptr, type_ADC_channel* adc_ch_v_ptr, type_ADC_channel* adc_ch_i_ptr, type_DAC_channel* dac_ch_ptr, float v_a, float v_b, float curr_a, float curr_b)
{
  oai_dd_reset_val(oai_dd_ptr, num, t_res_ptr, adc_ch_v_ptr, adc_ch_i_ptr, dac_ch_ptr, v_a, v_b, curr_a, curr_b);
  //
  pid_init(&oai_dd_ptr->pid_res, PID_R_K, PID_R_P, PID_R_D, PID_R_I, PID_R_REACTION_MAX_V);
  pid_init(&oai_dd_ptr->pid_current, PID_I_K, PID_I_P, PID_I_D, PID_I_I, PID_I_REACTION_MAX_V);
  pid_set_desired_value(&oai_dd_ptr->pid_res, DESIRED_RESISTANCE);
  pid_set_desired_value(&oai_dd_ptr->pid_current, DESIRED_CURRENT_MA);
  //
  filter_init(&oai_dd_ptr->filter_res);
  filter_init(&oai_dd_ptr->filter_curr);
  filter_init(&oai_dd_ptr->filter_voltage);
  //
  filter_parameter_set(&oai_dd_ptr->filter_res, OAI_DD_TIME_S, 0.1);
  filter_parameter_set(&oai_dd_ptr->filter_curr, OAI_DD_TIME_S, 0.1);
  filter_parameter_set(&oai_dd_ptr->filter_voltage, OAI_DD_TIME_S, 0.1);
  return 1;
}

/**
  * @brief  инициализация модели измерения канала ОАИ_ДД
  * @param  oai_dd_ptr указатель на програмную модель устройства
  * @param  t_res_ptr указатель на програмную модель измерителя температуры
  * @param  adc_ch_ptr указатель на програмную модель ацп
  * @param  dac_ch_ptr указатель на програмную модель цап
  * @param  v_a коэффициент перевод напряжения АЦП в напряжение на лампе V=v_a*V_adc+v_b
  * @param  v_b коэффициент перевод напряжения АЦП в напряжение на лампе
  * @param  curr_a коэффициент перевод напряжения АЦП в ток на лампе I=curr_a*V_adc+curr_b
  * @param  curr_b коэффициент перевод напряжения АЦП в ток на лампе 
  * @retval результат инициализации
  */
void oai_dd_reset_val(type_OAI_DD_model* oai_dd_ptr, uint8_t num, type_TRES_model* t_res_ptr, type_ADC_channel* adc_ch_v_ptr, type_ADC_channel* adc_ch_i_ptr, type_DAC_channel* dac_ch_ptr, float v_a, float v_b, float curr_a, float curr_b)
{
  oai_dd_ptr->num = num,
  //
  oai_dd_ptr->pr_voltage = 0.;
  oai_dd_ptr->dac_voltage = 0.;
  oai_dd_ptr->pressure = 0.;
  oai_dd_ptr->pressure_u16 = 0;
  oai_dd_ptr->temp = 0.;
  oai_dd_ptr->mode = MODE_PID_R;
  oai_dd_ptr->state = 0;
  //
  oai_dd_ptr->t_res_ptr = t_res_ptr;
  oai_dd_ptr->adc_v = adc_ch_v_ptr;
  oai_dd_ptr->adc_i = adc_ch_i_ptr;
  oai_dd_ptr->dac = dac_ch_ptr;
  // 
  oai_dd_ptr->v_a = v_a;
  oai_dd_ptr->v_b = v_b;
  oai_dd_ptr->curr_a = curr_a;
  oai_dd_ptr->curr_b = curr_b;
}

/**
  * @brief  установка режима работы ОАИ_ДД
  * @param  oai_dd_ptr указатель на програмную модель устройства
| * @param  mode режим работы
  * @retval результат инициализации
  */
void oai_dd_set_mode(type_OAI_DD_model* oai_dd_ptr, uint8_t mode)
{
  oai_dd_ptr->mode = mode;
}

/**
  * @brief  обработка данных измерительного канеала ОАИ ДД
  * @param  oai_dd_ptr указатель на програмную модель устройства
  * @param  period_ms период вызова данной функции
  */
void oai_dd_process(type_OAI_DD_model* oai_dd_ptr, uint16_t period_ms)
{
  float adc_voltage;
  //обновление параметров канала из данных АЦП
  adc_voltage = adc_get_ch_voltage(oai_dd_ptr->adc_v);
  oai_dd_ptr->pr_voltage = oai_dd_ptr->v_a*adc_voltage + oai_dd_ptr->v_b;
  adc_voltage = adc_get_ch_voltage(oai_dd_ptr->adc_i);
  oai_dd_ptr->pr_current = oai_dd_ptr->curr_a*adc_voltage + oai_dd_ptr->curr_b;
  if(oai_dd_ptr->pr_current == 0) oai_dd_ptr->pr_res = 0.0;
  else oai_dd_ptr->pr_res = oai_dd_ptr->pr_voltage*1000./oai_dd_ptr->pr_current;
  oai_dd_ptr->temp = tres_get_temp(oai_dd_ptr->t_res_ptr);
  //обработка воздействия на выход ЦАП
  if (oai_dd_ptr->mode == MODE_PID_R){
    oai_dd_pid_resistance(oai_dd_ptr, period_ms);
  }
  else if (oai_dd_ptr->mode == MODE_PID_I){
    oai_dd_pid_current(oai_dd_ptr, period_ms);
  }
  else if (oai_dd_ptr->mode == MODE_PID_OFF){
    oai_dd_ptr->dac_voltage = DD_DAC_MIN_VOLTAGE;
  }
  //установка полученных значений в ЦАП
  dac_set_ch_voltage(oai_dd_ptr->dac, oai_dd_ptr->dac_voltage);
  //фильтрация выдаваемых параметров
  filter_process(&oai_dd_ptr->filter_res, oai_dd_ptr->pr_res, period_ms);
  filter_process(&oai_dd_ptr->filter_curr, oai_dd_ptr->pr_current, period_ms);
  filter_process(&oai_dd_ptr->filter_voltage, oai_dd_ptr->pr_voltage, period_ms);
  //создать отчет
  oai_dd_get_frame_report(oai_dd_ptr);
}

/**
  * @brief  подсчет увеличения воздействия в случае стабилизации по сопротивлению (оно же по температуре)
  * @param  oai_dd_ptr указатель на програмную модель устройства
  * @param  period_ms период вызова данной функции
  */
void oai_dd_pid_resistance(type_OAI_DD_model* oai_dd_ptr, uint16_t period_ms)
{
  float dac_step = 0, dac_voltage_new = 0;
  dac_step = pid_step_calc(&oai_dd_ptr->pid_res, oai_dd_ptr->pr_res, period_ms);
  dac_voltage_new = oai_dd_ptr->dac_voltage + dac_step;
  if (dac_voltage_new >= DD_DAC_MAX_VOLTAGE){
    oai_dd_ptr->dac_voltage = DD_DAC_MAX_VOLTAGE;
  }
  else if(dac_voltage_new <= DD_DAC_MIN_VOLTAGE){
    oai_dd_ptr->dac_voltage = DD_DAC_MIN_VOLTAGE;
  }
  else{
    oai_dd_ptr->dac_voltage = dac_voltage_new;
  }
}

/**
  * @brief  подсчет увелчения воздействия в случае стабилизации по току
  * @param  oai_dd_ptr указатель на програмную модель устройства
  * @param  period_ms период вызова данной функции
  */
void oai_dd_pid_current(type_OAI_DD_model* oai_dd_ptr, uint16_t period_ms)
{
  float dac_step = 0, dac_voltage_new = 0;
  dac_step = pid_step_calc(&oai_dd_ptr->pid_current, oai_dd_ptr->pr_current, period_ms);
  dac_voltage_new = oai_dd_ptr->dac_voltage + dac_step;
  if (dac_voltage_new >= DD_DAC_MAX_VOLTAGE){
    oai_dd_ptr->dac_voltage = DD_DAC_MAX_VOLTAGE;
  }
  else if(dac_voltage_new <= DD_DAC_MIN_VOLTAGE){
    oai_dd_ptr->dac_voltage = DD_DAC_MIN_VOLTAGE;
  }
  else{
    oai_dd_ptr->dac_voltage = dac_voltage_new;
  }
}

/**
  * @brief  вывод отладочной информации в строку
  * @param  oai_dd_ptr указатель на програмную модель устройства
  * @param  report указатель на массив для отчета
  * @retval длина строки для отчета
  */
uint8_t oai_dd_get_str_report(type_OAI_DD_model* oai_dd_ptr, char* report)
{
  char report_str[128] = {0};
  sprintf(report_str, "DD_%d: in=%.2E out=%.2E i=%.2E R=%.2f T=%.1f P=%.2E", 
                      oai_dd_ptr->num,
                      oai_dd_ptr->pr_voltage,
                      oai_dd_ptr->dac_voltage,
                      oai_dd_ptr->pr_current,
                      oai_dd_ptr->pr_res,
                      oai_dd_ptr->temp,
                      oai_dd_ptr->pressure); 
	memcpy(report, report_str, 127);
	report[127] = 0;
  return strlen(report_str);
}

void oai_dd_get_frame_report(type_OAI_DD_model* oai_dd_ptr)
{
  oai_dd_ptr->report.mode = oai_dd_ptr->mode;
  oai_dd_ptr->report.state = oai_dd_ptr->state;
  oai_dd_ptr->report.pressure = oai_dd_ptr->pressure_u16;
  oai_dd_ptr->report.temp = tres_get_temp_u16(oai_dd_ptr->t_res_ptr);
  oai_dd_ptr->report.dac_out = (int16_t)floor(oai_dd_ptr->dac_voltage*2560.);
  oai_dd_ptr->report.volt_in = (int16_t)floor(oai_dd_ptr->pr_voltage*2560.);
  oai_dd_ptr->report.curr_in = (int16_t)floor(oai_dd_ptr->pr_current*256);
  oai_dd_ptr->report.resistance = (int16_t)floor(oai_dd_ptr->pr_res*256);
  oai_dd_ptr->report.volt_in_mean = (int16_t)floor((filter_get_value(&oai_dd_ptr->filter_voltage))*2560.);
  oai_dd_ptr->report.curr_in_mean = (int16_t)floor((filter_get_value(&oai_dd_ptr->filter_curr))*256);
  oai_dd_ptr->report.resistance_mean = (int16_t)floor((filter_get_value(&oai_dd_ptr->filter_res))*256);
  oai_dd_ptr->report.time_const_volt = (int16_t)floor(oai_dd_ptr->filter_voltage.time_const*256);
  oai_dd_ptr->report.time_const_curr = (int16_t)floor(oai_dd_ptr->filter_curr.time_const*256);
  oai_dd_ptr->report.time_const_res = (int16_t)floor(oai_dd_ptr->filter_res.time_const*256);
}

/// PID ///

/**
  * @brief  подсчет шага для ПИД
  * @param  pid_ptr указатель на програмную модель устройства
  * @param  K нормализующий коэффициент
  * @param  P пропорциональный коэффициент 
  * @param  D диффиренциальный коэффициент
  * @param  I интегральный коэффициент
  * @param  reaction_max максимальное воздействие
  */
void pid_init(type_PID_model* pid_ptr, float K, float P, float D, float I, float reaction_max)
{
  pid_reset(pid_ptr);
  pid_ptr->K_coeff = K;
  pid_ptr->P_coeff = P;
  pid_ptr->D_coeff = D;
  pid_ptr->I_coeff = I;
  pid_ptr->reaction_max = reaction_max;
}

/**
  * @brief  подсчет шага для ПИД
  * @param  pid_ptr указатель на програмную модель устройства
  */
void pid_reset(type_PID_model* pid_ptr)
{
  pid_ptr->value = 0.0;
  pid_ptr->value_desired = 0.0;
  pid_ptr->value_prev = 0.0;
  pid_ptr->derivate = 0.0;
  pid_ptr->error = 0.0;
  pid_ptr->integral = 0.0;
  pid_ptr->K_coeff = 0.0;
  pid_ptr->P_coeff = 0.0;
  pid_ptr->D_coeff = 0.0;
  pid_ptr->I_coeff = 0.0;
  pid_ptr->P_reaction = 0.0;
  pid_ptr->D_reaction = 0.0;
  pid_ptr->I_reaction = 0.0;
  pid_ptr->reaction = 0.0;
  pid_ptr->reaction_max = 0.0;
}

/**
  * @brief  сброс ппамяти ПИД
  * @param  pid_ptr указатель на програмную модель устройства
  */
void pid_refreshet(type_PID_model* pid_ptr)
{
  pid_ptr->integral = 0.0;
  pid_ptr->P_reaction = 0.0;
  pid_ptr->D_reaction = 0.0;
  pid_ptr->I_reaction = 0.0;
  pid_ptr->reaction = 0.0;
}

/**
  * @brief  установка желаемого значения
  * @param  pid_ptr указатель на програмную модель устройства
  * @param  desired_value значение регулируемой величины, к которой происходит подстройка
  */
void pid_set_desired_value(type_PID_model* pid_ptr, float desired_value)
{
  pid_refreshet(pid_ptr);
  pid_ptr->value_desired = desired_value;
}

/**
  * @brief  установка коэффициентов
  * @param  pid_ptr указатель на програмную модель устройства
  * @param  P пропорциональный коэффициент 
  * @param  D диффиренциальный коэффициент
  * @param  I интегральный коэффициент
  */
void pid_set_coeff(type_PID_model* pid_ptr, float K, float P, float D, float I)
{
  pid_ptr->K_coeff = K;
  pid_ptr->P_coeff = P;
  pid_ptr->D_coeff = D;
  pid_ptr->I_coeff = I;
}

/**
  * @brief  подсчет шага для ПИД
  * @param  pid_ptr указатель на програмную модель устройства
  * @param  value значение регулируемой переменной
  * @param  period_ms период вызова данной функции
  */
float pid_step_calc(type_PID_model* pid_ptr, float value, uint16_t period_ms)
{
  pid_ptr->value = value;
  pid_ptr->derivate = (pid_ptr->value - pid_ptr->value_prev)/(period_ms/1000.);
  pid_ptr->error = pid_ptr->value_desired - pid_ptr->value;
  pid_ptr->integral += pid_ptr->error*(period_ms/1000.);
  pid_ptr->P_reaction = pid_ptr->K_coeff*(pid_ptr->P_coeff*pid_ptr->error);
  pid_ptr->D_reaction = pid_ptr->K_coeff*(pid_ptr->D_coeff*pid_ptr->derivate);
  pid_ptr->I_reaction = pid_ptr->K_coeff*(pid_ptr->I_coeff*pid_ptr->integral);
  pid_ptr->reaction = pid_ptr->P_reaction + pid_ptr->D_reaction + pid_ptr->I_reaction;
  //
  if (pid_ptr->reaction > pid_ptr->reaction_max) pid_ptr->reaction = pid_ptr->reaction_max;
  else if (pid_ptr->reaction < -(pid_ptr->reaction_max)) pid_ptr->reaction = -pid_ptr->reaction_max;
  //
  pid_ptr->value_prev = pid_ptr->value;
  //
  return pid_ptr->reaction;
}

/**
  * @brief  вывод отладочной информации для ПИД
  * @param  pid_ptr указатель на програмную модель устройства
  * @param  report указатель на массив для отчета
  * @retval длина массива для отчета
  */
uint8_t pid_get_str_report(type_PID_model* pid_ptr, char* report)
{
  char report_str[128] = {0};
  sprintf(report_str, "PID: P=%.3f D=%.3f I=%.3f React=%.3f", 
                      pid_ptr->P_reaction,
                      pid_ptr->D_reaction,
                      pid_ptr->I_reaction,
                      pid_ptr->reaction);
	memcpy(report, report_str, 127);
	report[127] = 0;
  return strlen(report_str);
}
