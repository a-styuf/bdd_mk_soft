#include "1986ve8_lib/cm4ikmcu.h"
#include <string.h>
#include <stdio.h>
#include "sysinit.h"
#include "wdt.h"
#include "uarts.h"
#include "debug.h"
#include "timers.h"
#include "adc.h"
#include "dac.h"
#include "gpio.h"



type_ADC_model adc_0 = {0};
type_DAC_model dac;
type_SINGLE_GPIO rele_gpio, ku_gpio[2];
//
uint8_t n, m=0;
float var_float = 0;
//

int main() {
	// инициализация переферии
	System_Init();
	WDT_Init();
	UART0_Init();
	Timers_Init();
	//
	adc_init(&adc_0);
	dac_init(&dac);
	//
	rele_gpio.num = 20;
	rele_gpio.port = PORTE;
	gpio_set(&rele_gpio, 0);
	ku_gpio[0].num = 18;
	ku_gpio[0].port = PORTE;
	gpio_set(&ku_gpio[0], 0);
	ku_gpio[1].num = 19;
	ku_gpio[1].port = PORTE;
	gpio_set(&ku_gpio[1], 0);
	
	Timers_Start(0, 1000);
	
	printf("BDD_MK is online");
	while(1) {
		WDRST;
		if (Timers_Status(0))
		{   
			Timers_Start(0, 1000); // перезапускаем таймер для формирования слота времени (возможная проблема - пропуск слота)
			//обработка процессов
			adc_process(&adc_0, 1000);
			dac_set_voltage(&dac, 0, adc_get_ch_voltage(&adc_0, 0));
			dac_set_voltage(&dac, 1, adc_get_ch_voltage(&adc_0, 0));
			//

			//
			for(n=0; n<10; n++){
				printf("%d: %4.2f; ", n, adc_get_ch_voltage(&adc_0, n));
			}
			printf("temp:%+2.1f", get_mcu_temp(&adc_0));
			// проверка работы реле
			if (m==1){
				m=0;
				gpio_set(&rele_gpio, 0);
				gpio_set(&ku_gpio[0], 0);
				gpio_set(&ku_gpio[1], 0);
			}
			else {
				m=1;
				gpio_set(&rele_gpio, 1);
				gpio_set(&ku_gpio[0], 0);
				gpio_set(&ku_gpio[1], 0);
			}
		}
	}
}

// Обработка callback-функций от перываний
/**
  * @brief  обработчик прерывания АЦП
  */
void INT_ADC0_CallBack(void) {
  uint32_t rslt;
  uint16_t chn, val;
  while(adc_0.reg->STATUS & 1) {
    rslt = adc_0.reg->RESULT;
    chn = *((uint16_t*)&rslt + 1);
    val = *((uint16_t*)&rslt);
    adc_0.ch[chn].val = val;
  }
}
