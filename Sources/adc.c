#include "adc.h"

type_ADC_model adc_0;

/**
  * @brief  инициализация АЦП0
  */
void adc_init(type_ADC_model* adc_ptr)
{
  int i, sm;
	float cal_a[64] = ADC_CHAN_CAL_COEF_A;
  float cal_b[64] = ADC_CHAN_CAL_COEF_B;
  CLK_CNTR->KEY = _KEY_;
  CLK_CNTR->PER1_CLK |= (1<<23);  //clock ADC0
  //
  adc_ptr->reg = ADC0;
  adc_ptr->reg->KEY = _KEY_;
  adc_ptr->reg->CONFIG1 = (255<<12)|(511<<3);  //��� ����� ���������
  /* ���������� */
  adc_ptr->reg->FIFOEN1 = 0xF0000000;
  adc_ptr->reg->CONFIG0 = 1;  //enable adc
  sm = 0;
  for(i=0; i<4; i++) {
    adc_ptr->reg->CONTROL = (60 + i)<<8 | 1;
    while((adc_ptr->reg->STATUS & 1)==0);
    sm = sm + (adc_ptr->reg->RESULT & 0xFFF);
    }
  sm = sm >> 2;
  adc_ptr->reg->FIFOEN1 = 0;
  adc_ptr->reg->CONFIG1 = (sm << 20) | (adc_ptr->reg->CONFIG1 & 0xFFFFF);
  /**/
  adc_ptr->reg->CONFIG2 = 0x01000004;  //enable interrupt, temp.sensor on
  adc_ptr->reg->CHSEL0 =  0x00FFFFFF; //0x0080007F;
  adc_ptr->reg->FIFOEN0 = 0x00FFFFFF; //0x0080007F;
  adc_ptr->reg->CONFIG0 = 0x00000025;  //enable adc, SELMODE=1, continues mode
  //
  adc_set_ch_a_b(adc_ptr, cal_a, cal_b);
  //
  NVIC_EnableIRQ(IRQn_ADC0);
  //
}

/**
  * @brief  установка калибровочных коэффициентов
  * @param  adc_ptr указатель на програмную модель устройства
	* @param  a калибровочный коэффициент a
	* @param  b калибровочный коэффициент b
  */
void adc_set_ch_a_b(type_ADC_model* adc_ptr, float *a, float *b)
{
	uint8_t i=0;
  for(i=0; i<ADC0_CHAN_NUM; i++){
    adc_ptr->ch[i].a = a[i];
    adc_ptr->ch[i].b = b[i];
    adc_ptr->ch[i].adc_ch_num = i;
  }
}

/**
  * @brief  запрос данных канала АЦП в В
	* @param  adc_ptr указатель на програмную модель устройства
	* @param  ch_num номер канала
	* @retval значение канала АЦП в В
  */
float adc_ch_voltage(type_ADC_model* adc_ptr, uint8_t ch_num)
{
	float adc_ch_voltage;
  if(ch_num>=ADC0_CHAN_NUM){
    return 0.0;
  }
  NVIC_DisableIRQ(IRQn_ADC0);
  adc_ch_voltage = adc_ptr->ch[ch_num].a*adc_ptr->ch[ch_num].val + adc_ptr->ch[ch_num].b;
  NVIC_EnableIRQ(IRQn_ADC0);
  return adc_ch_voltage;
}

/**
  * @brief  подсчет температуры (из примера Миландер)
  * @param  adc_ptr указатель на програмную модель устройства
	* @retval значение температуры МК
  */
float calc_mcu_temp(type_ADC_model* adc_ptr)
{
	float temp_fp;
	int16_t temp_adc_val;
  NVIC_DisableIRQ(IRQn_ADC0);
	temp_adc_val = (int16_t)adc_ptr->ch[ADC0_CHAN_TEMP].val; 
  NVIC_EnableIRQ(IRQn_ADC0);
	temp_fp = (-(temp_adc_val) + FACTORY_ADC_TEMP25)/FACTORY_ADC_AVG_SLOPE + FACTORY_TEMP25;
	return temp_fp;
}

/**
  * @brief  обработка всех каналов АЦП для получения корректного значения напряжения
	* @param  adc_ptr указатель на програмную модель устройства
  * @param  period_ms период вызова функции
  */
void adc_process(type_ADC_model* adc_ptr, uint16_t period_ms)
{
	uint8_t ch_num = 0;
  for(ch_num=0; ch_num<ADC0_CHAN_NUM; ch_num++){
    adc_ptr->ch[ch_num].voltage = adc_ch_voltage(adc_ptr, ch_num);
  }
  adc_ptr->temp = calc_mcu_temp(adc_ptr);
}

/**
  * @brief  запрос напряжения канала
  * @param  adc_ptr указатель на програмную модель устройства
  * @param  ch_num номер необходимого канала
	* @retval значение напряжения на входе АЦП
  */
float adc_get_ch_voltage(type_ADC_model* adc_ptr, uint8_t ch_num)
{
  volatile float adc_voltage = 0;
  adc_voltage = adc_ptr->ch[ch_num].voltage;
  return adc_voltage;
}

/**
  * @brief  запрос температуры (из примера Миландер)
  * @param  adc_ptr указатель на програмную модель устройства
	* @retval значение температуры МК
  */
float get_mcu_temp(type_ADC_model* adc_ptr)
{
	volatile float temp_fp;
	temp_fp = adc_ptr->temp;
	return temp_fp;
}

/**
  * @brief  обработчик прерывания АЦП
  */
void INT_ADC0_Handler(void) {
  uint32_t rslt;
  uint16_t chn, val;
  while(adc_0.reg->STATUS & 1) {
    rslt = adc_0.reg->RESULT;
    chn = *((uint16_t*)&rslt + 1);
    val = *((uint16_t*)&rslt);
    adc_0.ch[chn].val = val;
  }
}
