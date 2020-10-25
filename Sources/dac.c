#include "dac.h"

/**
  * @brief  инициализация ЦАП
  */
void dac_init(type_DAC_model* dac_ptr)
{
  int i;
	float cal_a[2] = DAC_CHAN_CAL_COEF_A;
  float cal_b[2] = DAC_CHAN_CAL_COEF_B;
  DACxControl* dac_reg;
  //
  CLK_CNTR->KEY = _KEY_;
  CLK_CNTR->PER1_CLK |= (1<<26) | (1<<25);  //clock DAC0 and DAC1
  CLK_CNTR->KEY = 0x00;
  //
  dac_ptr->ch[0].reg = DAC0;
  dac_ptr->ch[1].reg = DAC1;
  //
  for (i=0;i<2;i++){
    dac_reg = dac_ptr->ch[i].reg;
    dac_reg->KEY = _KEY_;
    dac_reg->CONFIG0 =  (0x0<<8) | (0x0<<7) | (0x0<<4) | (0x0<<2) | (0x1<<1) | (0x1<<0); // DAC enable, continuous mode
    dac_reg->CONFIG1 =  (0x0<<8) | (0x0<<7) | (0x0<<3) | (0x0<<0); // all default
    dac_reg->KEY = 0;
  }
  //
  dac_set_ch_a_b(dac_ptr, cal_a, cal_b);
  //
}

/**
  * @brief  установка калибровочных коэффициентов для каналов ЦАП
  * @param  dac_ptr указатель на програмную модель устройства
	* @param  a калибровочный коэффициент a
	* @param  b калибровочный коэффициент b
  */
void dac_set_ch_a_b(type_DAC_model* dac_ptr, float *a, float *b)
{
	uint8_t i=0;
  for(i=0; i<2; i++){
    dac_ptr->ch[i].a = a[i];
    dac_ptr->ch[i].b = b[i];
  }
}

/**
  * @brief  установка канала ЦАП
	* @param  dac_ptr указатель на програмную модель устройства
	* @param  ch_num номер канала (0, 1)
	* @param  voltage напряжение на выходе ЦАП
  */
void dac_set_voltage(type_DAC_model* dac_ptr, uint8_t ch_num, float voltage)
{
  uint16_t dac_val = 0;
  if(ch_num>=2){
    return;
  }
  dac_val = (uint16_t)floor(dac_ptr->ch[ch_num].a * voltage + dac_ptr->ch[ch_num].b);
  if (dac_val > 0xFFF) dac_val = 0x0FFF;
  dac_ptr->ch[ch_num].reg->KEY = _KEY_;
  dac_ptr->ch[ch_num].reg->DATA = dac_val & 0x0FFF;
  //dac_ptr->ch[ch_num].reg->DATA = 1000;
  dac_ptr->ch[ch_num].reg->KEY = 0x00;
}

/**
  * @brief  установка канала ЦАП
	* @param  dac_ptr указатель на програмную модель устройства
	* @param  ch_num номер канала (0, 1)
	* @param  code числовое значение кода ЦАП для установки
  */
void dac_set_code(type_DAC_model* dac_ptr, uint8_t ch_num, uint16_t code)
{
  uint16_t dac_val = 0;
  if(ch_num>=2){
    return;
  }
  dac_val = code;
  if (dac_val > 0xFFF) dac_val = 0x0FFF;
  dac_ptr->ch[ch_num].reg->KEY = _KEY_;
  dac_ptr->ch[ch_num].reg->DATA = dac_val & 0x0FFF;
  dac_ptr->ch[ch_num].reg->KEY = 0x00;
}
