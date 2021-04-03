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
  ims_ptr->ku_gpio_0 = ku0;
  ims_ptr->ku_gpio_1 = ku1;
  //filters
  filter_init(&ims_ptr->filter_u);
  filter_parameter_set(&ims_ptr->filter_u, IMS_U_FILTER_T_CONST, IMS_U_FILTER_T_SAMPLE);
  filter_init(&ims_ptr->filter_temp);
  filter_parameter_set(&ims_ptr->filter_temp, IMS_T_FILTER_T_CONST, IMS_T_FILTER_T_SAMPLE);
  //
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
  ims_ptr->ku = 0;
  ims_ptr->dead_time = IMS_MEASURE_DEAD_TIME_MS;
  ims_ptr->pressure_u16 = 0;
  ims_ptr->temp = 0.;
  ims_ptr->mode = IMS_MODE_DEFAULT;
  ims_ptr->state = 0;
  // 
}

/**
  * @brief  установка режима работы ИМД
  * @param  ims_ptr указатель на програмную модель устройства
| * @param  mode режим работы
  * @retval результат инициализации
  */
void ims_set_mode(type_IMS_model* ims_ptr, uint8_t mode)
{
  ims_ptr->mode = mode;
}

/**
  * @brief  обработка данных измерительного канеала ОАИ ДД
  * @param  ims_ptr указатель на програмную модель устройства
  * @param  period_ms период вызова данной функции
  */
void ims_process(type_IMS_model* ims_ptr, uint16_t period_ms)
{
  
  //проверка температуры канала
  ims_ptr->temp = tres_get_temp(ims_ptr->t_res_ptr);
  filter_process(&ims_ptr->filter_temp, ims_ptr->temp, period_ms);
  //обновление параметров канала из данных АЦП
  ims_ptr->voltage[ims_ptr->ku] = adc_get_ch_voltage(ims_ptr->adc_v);
  filter_process(&ims_ptr->filter_u, ims_ptr->voltage[ims_ptr->ku], period_ms);
  //проверка режима работы
  if (ims_ptr->mode == IMS_MODE_SIMPLE){
    ims_simple_process(ims_ptr, period_ms);
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
void ims_simple_process(type_IMS_model* ims_ptr, uint16_t period_ms)
{
  ims_range_change(ims_ptr, period_ms);
}

/**
  * @brief  определение необходимости переключения на другой диапазон
  * @param  ims_ptr указатель на програмную модель устройства
  * @param  period_ms период вызова данной функции
  */
int8_t ims_range_change_checking(type_IMS_model* ims_ptr, float voltage)
{
  //
  if (voltage >= IMS_TOP_VOLTAGE){
    return 1;
  }
  else if (voltage <= IMS_BOT_VOLTAGE){
    return -1;
  }
  else{
    return 0;
  }
  //
}

/**
  * @brief  определение и установка КУ на которое необходимо переключиться
  * @param  ims_ptr указатель на програмную модель устройства
  * @param  period_ms период вызова данной функции
  */
void ims_range_change(type_IMS_model* ims_ptr, uint16_t period_ms)
{
  int8_t need_to_change = 0;
  //
  if (ims_ptr->dead_time > 0){
    ims_ptr->dead_time -= period_ms;
  }
  else{
    need_to_change = ims_range_change_checking(ims_ptr, ims_ptr->voltage[ims_ptr->ku]);
    //
    if (need_to_change){
      if (need_to_change == 1){
        ims_ptr->ku = (ims_ptr->ku < 4) ? (ims_ptr->ku + 1) : ims_ptr->ku;
      }
      else if (need_to_change == -1){
        ims_ptr->ku = (ims_ptr->ku > 0) ? (ims_ptr->ku - 1) : ims_ptr->ku;
      }
      ims_ptr->dead_time = IMS_MEASURE_DEAD_TIME_MS;
      filter_reset(&ims_ptr->filter_u);
    }
    else{
      // NULL;
    }
  }
}


void ims_calibr_process(type_IMS_model* ims_ptr, uint16_t period_ms)
{
  
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
                      ims_ptr->voltage,
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
  ims_ptr->report.mode = ims_ptr->mode;
  ims_ptr->report.state = ims_ptr->state;
  ims_ptr->report.pressure = ims_ptr->pressure_u16;
  ims_ptr->report.temp = (int16_t)floor(filter_get_value(&ims_ptr->filter_temp)*256);
  ims_ptr->report.voltage = (int16_t)floor(ims_ptr->voltage[0]*256);
  memset((uint8_t*)ims_ptr->report.reserve, 0xFE, 18);
}

