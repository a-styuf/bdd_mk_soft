#ifndef _DIGITAL_FILTER_H_
#define _DIGITAL_FILTER_H_

#include "1986ve8_lib/cm4ikmcu.h"


#define DF_DEFAULT_TIME_CONST_S 1.0

#pragma pack(2)
typedef struct  //  max 62 - параетры ЦМ для сохоранения
{
	float time_const;
	float t_sample;
	float value_out, value_in;
	float value_in_buff[8], value_out_buff[8];
}type_DFilter_model;


void filter_init(type_DFilter_model *filter_ptr);
void filter_parameter_set(type_DFilter_model *filter_ptr, float time_const, float t_sample);
void filter_process(type_DFilter_model *filter_ptr, float time_const, uint16_t period_ms);
float filter_get_value(type_DFilter_model *filter_ptr);
void filter_reset(type_DFilter_model *filter_ptr);

#endif

