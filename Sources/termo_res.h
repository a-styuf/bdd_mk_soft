#ifndef _TERMO_RES_H_
#define _TERMO_RES_H_

#include <string.h>
#include <stdint.h>
#include "1986ve8_lib/cm4ikmcu.h"
#include "adc.h"

#define TRES_V_REF    (3.38)
#define TRES_R1_REF   (3E3) // 3k Ohm

#define TRES_TYPE   (1E3) // Pt1000

<<<<<<< Updated upstream
#define TRES_CAL_TEMP   {-50, -40, -30. -20, -10, -00, +10, +20, +30, +40, +50, +100, +200}
#define TRES_CAL_RES    {803.1, 842.7, 882.2, 921.6, 960.9, 1000.0, 1039.0, 1077.9, 1116.7, 1155.4, 1194.0, 1385, 1758.4}

=======
<<<<<<< HEAD
#define TRES_CAL_TEMP   {-50, -40, -30. -20, -10, -00, +10, +20, +30, +40, +50, +100, +200, +200, +200, +200}
#define TRES_CAL_RES    {803.1, 842.7, 882.2, 921.6, 960.9, 1000.0, 1039.0, 1077.9, 1116.7, 1155.4, 1194.0, 1385, 1758.4, 1758.4, 1758.4, 1758.4}


#pragma pack(2)
=======
#define TRES_CAL_TEMP   {-50, -40, -30. -20, -10, -00, +10, +20, +30, +40, +50, +100, +200}
#define TRES_CAL_RES    {803.1, 842.7, 882.2, 921.6, 960.9, 1000.0, 1039.0, 1077.9, 1116.7, 1155.4, 1194.0, 1385, 1758.4}

>>>>>>> main
>>>>>>> Stashed changes
/** 
  * @brief  структура управления отдельным термо-резистором, включенным по схеме резистивного делителя (R1 - верхнее плечо, термосопротивление - нижнее плечо)
  */
typedef struct
{
  type_ADC_channel* adc_ch;
  float cal_temp[16], cal_res[16];
  float v_ref, v_out;
  float r1_val, tres_val;
  float temp;
  uint16_t temp_u16;
} type_TRES_model;

//
void tres_init(type_TRES_model* t_res_ptr, type_ADC_channel* adc_ch);
void tres_set_parameters(type_TRES_model* t_res_ptr, type_ADC_channel* adc_ch, float vref, float r1_val, float tres_val, float *cal_temp, float *cal_res);
float tres_get_temp(type_TRES_model* t_res_ptr);
uint16_t tres_get_temp_u16(type_TRES_model* t_res_ptr);

float _linear_interpolation(float x, float* array_y, float* array_x, uint16_t length);
float _calc_tr_res(float u_ref, float u_sign, float r_1);

#endif
