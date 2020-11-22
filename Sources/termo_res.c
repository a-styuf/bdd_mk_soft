  /**
  ******************************************************************************
  * @file           : termo_res.c
  * @version        : v1.0
  * @brief          : библиотека для работы с термосопротивлениями типа PtXXX
  * @author			: Стюф Алексей/Alexe Styuf <a-styuf@yandex.ru>
  ******************************************************************************
  */

#include "termo_res.h"

/**
  * @brief  инициализация прогармной модели терморезистора
  */
void tres_init(type_TRES_model* t_res_ptr, type_ADC_channel* adc_ch)
{
  float cal_temp[16] = TRES_CAL_TEMP;
  float cal_res[16] = TRES_CAL_RES;
  //
  tres_set_parameters(t_res_ptr, adc_ch, TRES_V_REF, TRES_R1_REF, TRES_TYPE, cal_temp, cal_res);
  //
}

/**
  * @brief  установка параметров схемы термосопротивления
	* @param  t_res_ptr указатель на програмную модель устройства
	* @param  adc_ch указатель на канал АЦП для опроса термосопротивления
	* @param  vref напряжения опроса схемы с термосопротивлением
	* @param  r1_val сопротивление верхнего плеча схемы опроса 
	* @param  tres_val сопротивление терморезистора
  */
void tres_set_parameters(type_TRES_model* t_res_ptr, type_ADC_channel* adc_ch, float vref, float r1_val, float tres_val, float *cal_temp, float *cal_res)
{
	uint8_t i=0;
  t_res_ptr->adc_ch = adc_ch;
  t_res_ptr->v_ref = vref;
  t_res_ptr->v_out = 1.0;
  t_res_ptr->r1_val = r1_val;
  t_res_ptr->tres_val = tres_val;
  //
	for (i=0; i<16; i++){
		t_res_ptr->cal_temp[i] = cal_temp[i];
		t_res_ptr->cal_res[i] = cal_res[i];
	}
}

/**
  * @brief  получение температуры с датчика
	* @param  t_res_ptr указатель на програмную модель устройства
	* @retval значение температуры в float в 1°С
  */
float tres_get_temp(type_TRES_model* t_res_ptr)
{
  t_res_ptr->v_out = adc_get_ch_voltage(t_res_ptr->adc_ch);
  t_res_ptr->tres_val = _calc_tr_res(t_res_ptr->v_ref, t_res_ptr->v_out, t_res_ptr->r1_val);
  return _linear_interpolation(t_res_ptr->tres_val, t_res_ptr->cal_temp, t_res_ptr->cal_res, 16);
}

/**
  * @brief  получение температуры с датчика
	* @param  t_res_ptr указатель на програмную модель устройства
	* @retval значение температуры в int16_t в (1/256)°С
  */
uint16_t tres_get_temp_u16(type_TRES_model* t_res_ptr)
{
  t_res_ptr->temp_u16 = (uint16_t)(tres_get_temp(t_res_ptr)*256.);
  return t_res_ptr->temp_u16;
}

/**
  * @brief  функция линейной интерполяции
	* @param  x входное значение
	* @param  array_y массив с Y-значениями
  * @param  array_x массив с X-значениями (от меньшего к большему)
  * @param  length длина массивов
	* @retval значение  y соответствующее x; в случае выхода х за границу возвращаются крайние значения array_y
  */
float _linear_interpolation(float x, float* array_y, float* array_x, uint16_t length)
{
  float y=0;
  uint16_t n;
  //проверяем на адекватность значения x
  if (x < array_x[0]) return array_y[0];
  if (x > array_x[length-1]) return array_y[length-1];
  //проходим каждый из отрезков, для определения того, куда попадает X
  for (n=0; n<length-1; n++){
    if ((x >= array_x[n]) &&  (x <= array_x[n+1])){
      y = array_y[n] + (array_y[n+1] - array_y[n])*((x - array_x[n])/(array_x[n+1] - array_x[n]));
      return y;
    }
  }
  return 0;
}

/**
  * @brief  функция нахождения сопротивления термодатчика, как нижнее плечо в резестивном делителе
	* @param  u_ref напряжение в верхнем плече
	* @param  u_sign напряжение в нижнем плече
  * @param  r_1 сопротивление верхнего плеча
	* @retval сопротивление термодачика 
  */
float _calc_tr_res(float u_ref, float u_sign, float r_1)
{
  if ((u_ref - u_sign) == 0){
    return 0.0;
  }
  return r_1 * (u_sign/(u_ref - u_sign));
}
