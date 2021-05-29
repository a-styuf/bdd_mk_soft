  /**
  ******************************************************************************
  * @file           : ims.c
  * @version        : v1.0
  * @brief          : библиотека для работы с каналом измерения ИнверсноМагнетронного датчика (ИМД - IMS)
  * @author			: Стюф Алексей/Alexe Styuf <a-styuf@yandex.ru>
  ******************************************************************************
  */
  
#include "ims.h"

/**
  * @brief  инициализация модели измерения канала ИМД
  * @param  ims_ptr указатель на програмную модель устройства
  * @param  t_res_ptr указатель на програмную модель измерителя температуры
  * @param  adc_ch_v_ptr указатель на програмную модель ацп для напряжения
  * @retval результат инициализации
  */
int8_t ims_init(type_IMS_model* ims_ptr, type_TRES_model* t_res_ptr, type_ADC_channel* adc_ch_v_ptr, type_MVIP* mvip_ptr, type_SINGLE_GPIO *rele, type_SINGLE_GPIO *ku0, type_SINGLE_GPIO *ku1)
{
  //
  ims_reset_val(ims_ptr);
  //
  ims_ptr->t_res_ptr = t_res_ptr;
  ims_ptr->adc_v = adc_ch_v_ptr;
  ims_ptr->mvip = mvip_ptr;
  // gpio
  ims_ptr->rele_gpio = rele;
  gpio_set(ims_ptr->rele_gpio, IMS_RELE_MEAS);
  ims_ptr->ku_gpio_0 = ku0;
  ims_ptr->ku_gpio_1 = ku1;
  ims_set_ku(ims_ptr, IMS_KU_DEFAULT);
  //filters
  filter_init(&ims_ptr->filter_u);
  filter_parameter_set(&ims_ptr->filter_u, IMS_U_FILTER_T_CONST, IMS_U_FILTER_T_SAMPLE);
  filter_init(&ims_ptr->filter_temp);
  filter_parameter_set(&ims_ptr->filter_temp, IMS_T_FILTER_T_CONST, IMS_T_FILTER_T_SAMPLE);
  // запускаем начальную калибровку
  ims_set_mode(ims_ptr, IMS_MODE_CALIBR);
  return 1;
}

/**
  * @brief  инициализация модели измерения канала ИМД
  * @param  ims_ptr указатель на програмную модель устройства
  * @retval результат инициализации
  */
void ims_reset_val(type_IMS_model* ims_ptr)
{
  ims_ptr->pressure = 0.;
  ims_ptr->ku = IMS_KU_DEFAULT;
  ims_ptr->ku_dead_time = IMS_KU_DEAD_TIME_MS;
  ims_ptr->pressure_u16 = 0;
  ims_ptr->temp = 0.;
  ims_ptr->state = 0;
  ims_ptr->calibration_mode = 0;
  ims_ptr->calibration_clock_ms = 0;
  // 
}

/**
  * @brief  установка режима работы ИМД (обертка для присвоения переменной)
  * @param  ims_ptr указатель на програмную модель устройства
| * @param  mode режим работы
  * @retval результат инициализации
  */
void ims_set_mode(type_IMS_model* ims_ptr, uint8_t mode)
{
  ims_ptr->mode_after_calibration = IMS_MODE_DEFAULT;
  ims_ptr->ku_after_calibration = ims_ptr->ku;
  ims_ptr->mode = mode;
  switch (ims_ptr->mode){
    case IMS_MODE_AUTO:
      //
      break;
    case IMS_MODE_CALIBR:

      ims_ptr->calibration_mode = 3;
  }
}

/**
  * @brief  установка КУ инструментальноко ОУ
  * @param  ims_ptr указатель на програмную модель устройства
| * @param  ку режим работы
  * @retval результат инициализации
  */
void ims_set_ku(type_IMS_model* ims_ptr, uint8_t ku)
{
  ims_ptr->ku = ku;
  gpio_set(ims_ptr->ku_gpio_0, ku & 0x01);
  gpio_set(ims_ptr->ku_gpio_1, ku & 0x02);
}

/**
  * @brief  обработка данных измерительного канеала ОАИ ДД
  * @param  ims_ptr указатель на програмную модель устройства
  * @param  period_ms период вызова данной функции
  */
void ims_process(type_IMS_model* ims_ptr, uint16_t period_ms)
{
  //получение температуры канала
  ims_ptr->temp = tres_get_temp(ims_ptr->t_res_ptr);
  filter_process(&ims_ptr->filter_temp, ims_ptr->temp, period_ms);
  //обновление параметров канала из данных АЦП
  ims_ptr->meas_voltage[ims_ptr->ku] = adc_get_ch_voltage(ims_ptr->adc_v);
  ims_ptr->pr_voltage[ims_ptr->ku] = ims_ptr->meas_voltage[ims_ptr->ku] - ims_ptr->zero_voltage[ims_ptr->ku];
  filter_process(&ims_ptr->filter_u, ims_ptr->pr_voltage[ims_ptr->ku], period_ms);  //важно, фильтр сбрасывается после изменения КУ в блок проверки изменения КУ
  //проверка режима работы
  if (ims_ptr->mode == IMS_MODE_AUTO){
    ims_auto_process(ims_ptr, period_ms);
  }
  else if (ims_ptr->mode == IMS_MODE_CALIBR){
    ims_calibr_process(ims_ptr, period_ms);
  }
  else if (ims_ptr->mode == IMS_MODE_OFF){
    
  }
  //создать отчет
  ims_get_frame_report(ims_ptr);
}

/**
  * @brief  обработка измерения ИМД
  * @param  ims_ptr указатель на програмную модель устройства
  * @param  period_ms период вызова данной функции
  */
void ims_auto_process(type_IMS_model* ims_ptr, uint16_t period_ms)
{
  ims_range_change(ims_ptr, period_ms);
}

/**
  * @brief  обработка измерения ИМД в ручном режиме
  * @param  ims_ptr указатель на програмную модель устройства
  * @param  period_ms период вызова данной функции
  */
void ims_manual_process(type_IMS_model* ims_ptr, uint16_t period_ms)
{
  //
}

/**
  * @brief  калибровка по всем КУ
  * @param  ims_ptr указатель на програмную модель устройства
  * @param  period_ms период вызова функции
  */
void ims_calibr_process(type_IMS_model* ims_ptr, uint16_t period_ms)
{
  //  выставляем необходимое КУ согласно режиму колибровки
  ims_set_ku(ims_ptr, ims_ptr->calibration_mode);
  //  переключаем реле на калибровку
  gpio_set(ims_ptr->rele_gpio, IMS_RELE_CALIB);
  //  
  if (ims_ptr->calibration_clock_ms >= IMS_CALIBRATION_TIME_MS){
    ims_ptr->calibration_clock_ms = 0;
    switch (ims_ptr->calibration_mode){
      case 0:
        ims_ptr->zero_voltage[ims_ptr->calibration_mode] = adc_get_ch_voltage(ims_ptr->adc_v);
        ims_set_mode(ims_ptr, ims_ptr->mode_after_calibration);
        ims_set_ku(ims_ptr, ims_ptr->ku_after_calibration);
        //  переключаем реле на измерение
        gpio_set(ims_ptr->rele_gpio, IMS_RELE_MEAS);
        break;
      case 1:
      case 2:
      case 3:
        ims_ptr->zero_voltage[ims_ptr->calibration_mode] = adc_get_ch_voltage(ims_ptr->adc_v);
        ims_ptr->calibration_mode -= 1;
        break;
    }
  }
  else{
    ims_ptr->calibration_clock_ms += period_ms;
  }
}

/**
  * @brief  определение и установка КУ на которое необходимо переключиться
  * @param  ims_ptr указатель на програмную модель устройства
  * @param  period_ms период вызова данной функции
  */
void ims_range_change(type_IMS_model* ims_ptr, uint16_t period_ms)
{
  int8_t need_to_change = 0, ku = 0;;
  //
  if (ims_ptr->ku_dead_time > 0){
    ims_ptr->ku_dead_time -= period_ms;
  }
  else{
    need_to_change = ims_range_change_checking(ims_ptr, ims_ptr->meas_voltage[ims_ptr->ku], ims_ptr->pr_voltage[ims_ptr->ku]);
    //
    if (need_to_change){
      if (need_to_change == 1){
        ku = (ims_ptr->ku < 3) ? (ims_ptr->ku + 1) : ims_ptr->ku;
        ims_set_ku(ims_ptr, ku);
      }
      else if (need_to_change == -1){
        ku = (ims_ptr->ku > 0) ? (ims_ptr->ku - 1) : ims_ptr->ku;
        ims_set_ku(ims_ptr, ku);
      }
      ims_ptr->ku_dead_time = IMS_KU_DEAD_TIME_MS;
      filter_reset(&ims_ptr->filter_u);
    }
    else{
      // NULL;
    }
  }
}

/**
  * @brief  определение необходимости переключения на другой диапазон
  * @param  ims_ptr указатель на програмную модель устройства
  * @param  meas_voltage напряжение АЦП
  * @param  pr_voltage напряжение для подсчета давления
  */
int8_t ims_range_change_checking(type_IMS_model* ims_ptr, float meas_voltage, float pr_voltage)
{
  //
  if (pr_voltage <= IMS_PR_MIN_VOLTAGE){
    return 1;
  }
  else if ((meas_voltage < IMS_BOT_VOLTAGE) || (meas_voltage > IMS_TOP_VOLTAGE)) {
    return -1;
  }
  else{
    return 0;
  }
  //
}

/**
  * @brief  вывод отладочной информации в строку
  * @param  ims_ptr указатель на програмную модель устройства
  * @param  report указатель на массив для отчета
  * @retval длина строки для отчета
  */
uint8_t ims_get_str_report(type_IMS_model* ims_ptr, char* report)
{
  char report_str[128] = {0};
  sprintf(report_str, "IMS: P=%.2E U=%.2E T=%.2E ku=0x%02X mode=%02X", 
                      ims_ptr->pressure,
                      ims_ptr->pr_voltage[1],
                      ims_ptr->temp,
                      ims_ptr->ku,
                      ims_ptr->mode
                      ); 
	memcpy(report, report_str, 127);
	report[127] = 0;
  return strlen(report_str);
}

void ims_get_frame_report(type_IMS_model* ims_ptr)
{
  uint8_t i=0;
  ims_ptr->report.mode = ims_ptr->mode;
  ims_ptr->report.state = ims_ptr->state;
  ims_ptr->report.pressure = ims_ptr->pressure_u16;
  ims_ptr->report.temp = (int16_t)floor(filter_get_value(&ims_ptr->filter_temp)*256);
  ims_ptr->report.active_voltage = (int16_t)floor(ims_ptr->pr_voltage[ims_ptr->ku]*1000);
  ims_ptr->report.ku = (int16_t)floor(ims_ptr->ku);
  for (i=0;i<4;i++){
    ims_ptr->report.pr_voltage_arr[i] = (int16_t)floor(ims_ptr->pr_voltage[i]*1000);
    ims_ptr->report.meas_voltage_arr[i] = (int16_t)floor(ims_ptr->meas_voltage[i]*1000);
    ims_ptr->report.zero_voltage_arr[i] = (int16_t)floor(ims_ptr->zero_voltage[i]*1000);
  }
  memcpy((uint8_t*)&ims_ptr->report.mvip_report, (uint8_t*)&ims_ptr->mvip->report, sizeof(type_MVIP_frame_report));
  memset((uint8_t*)ims_ptr->report.reserve, 0xFE, sizeof(ims_ptr->report.reserve));
}

