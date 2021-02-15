  /**
  ******************************************************************************
  * @file           : digital_filter.c
  * @version        : v1.0
  * @brief          : библиотека для фильтрации сигналов цифровыми фильтрами
  * @author			: Стюф Алексей/Alexe Styuf <a-styuf@yandex.ru>
  ******************************************************************************
  */
#include "digital_filter.h"


/**
  * @brief  инициализация фильтра
  * @param  filter_ptr указатель на програмную модель
  */
void filter_init(type_DFilter_model *filter_ptr)
{
	filter_parameter_set(filter_ptr, DF_DEFAULT_TIME_CONST_S, 0.1);
}

/**
  * @brief  установка параметров фильтра
  * @param  filter_ptr указатель на програмную модель
  * @param  time_const постоянная времени фильтра
  * @param  t_sample время сэмплирования
  */
void filter_parameter_set(type_DFilter_model *filter_ptr, float time_const, float t_sample)
{
	uint8_t i = 0;
  //
	
  filter_ptr->time_const = time_const;
  //
	for (i = 0; i < 8; i++){
		filter_ptr->value_in_buff[i]=0;
		filter_ptr->value_out_buff[i]=0;
	}
}

/**
  * @brief  установка параметров фильтра
  * @param  filter_ptr указатель на програмную модель
  * @param  period_ms период вызова функции
  */
void filter_process(type_DFilter_model *filter_ptr, float value_in, uint16_t period_ms)
{
	float samples_number = 0;
	samples_number = filter_ptr->time_const / filter_ptr->t_sample;
	filter_ptr->value_in = value_in;
	filter_ptr->value_out = filter_ptr->value_out_buff[0] + (value_in - filter_ptr->value_out_buff[0])/samples_number;
	filter_ptr->value_out_buff[0] = filter_ptr->value_out;
}

/**
  * @brief  установка параметров фильтра
  * @param  filter_ptr указатель на програмную модель
	* @retval отфильтрованное значение параметра
  */
float filter_get_value(type_DFilter_model *filter_ptr)
{
	return filter_ptr->value_out;
}
