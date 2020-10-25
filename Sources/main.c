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
<<<<<<< Updated upstream
//#include "termo_res.h"
=======
<<<<<<< HEAD
#include "termo_res.h"
=======
//#include "termo_res.h"
>>>>>>> main
>>>>>>> Stashed changes



type_ADC_model adc_0;
type_DAC_model dac;
type_SINGLE_GPIO rele_gpio, ku_gpio[2];
type_TRES_model tres_ims, tres_pirani_1, tres_pirani_2;
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
	tres_init(&tres_ims, &adc_0.ch[1]);
	tres_init(&tres_pirani_1, &adc_0.ch[4]);
	tres_init(&tres_pirani_2, &adc_0.ch[7]);
	
	// gpio tmp
	rele_gpio.num = 20;
	rele_gpio.port = PORTE;
	gpio_set(&rele_gpio, 0);
	ku_gpio[0].num = 18;
	ku_gpio[0].port = PORTE;
	gpio_set(&ku_gpio[0], 0);
	ku_gpio[1].num = 19;
	ku_gpio[1].port = PORTE;
	gpio_set(&ku_gpio[1], 0);
<<<<<<< Updated upstream
=======
<<<<<<< HEAD
	//
=======
>>>>>>> Stashed changes
	//tres_init(&tres_ims, &adc_0.ch[1]);
	//tres_init(&tres_pirani_1, &adc_0.ch[4]);
	//tres_init(&tres_pirani_2, &adc_0.ch[7]);
	
>>>>>>> main
	Timers_Start(0, 1000);
	
	printf("BDD_MK is online");
	while(1) {
		WDRST;
		if (Timers_Status(0))
		{   
			Timers_Start(0, 2000); // перезапускаем таймер для формирования слота времени (возможная проблема - пропуск слота)
			//обработка процессов
			adc_process(&adc_0, 1000);
			dac_set_voltage(&dac, 0, adc_get_ch_voltage(&adc_0.ch[n]));
			dac_set_voltage(&dac, 1, adc_get_ch_voltage(&adc_0.ch[n]));
<<<<<<< Updated upstream
=======
<<<<<<< HEAD
=======
>>>>>>> Stashed changes
			printf("adc0:%5.2f\t", adc_get_ch_voltage(&adc_0.ch[0]));
			//

>>>>>>> main
			//
			for(n=0; n<10; n++){
				printf("%d: %4.2f; ", n, adc_get_ch_voltage(&adc_0.ch[n]));
			}
			printf("Tmk:%+2.1f\n", get_mcu_temp(&adc_0));
			// проверка работы реле
			if (gpio_get(&rele_gpio)){
				gpio_set(&rele_gpio, 0);
			}
			else {
				gpio_set(&rele_gpio, 1);
			}
<<<<<<< Updated upstream
			//printf("temp:%+2.1f\t", tres_get_temp(&tres_pirani_1));
			//printf("temp:%+2.1f\t", tres_get_temp(&tres_pirani_2));
			//printf("temp:%+2.1f\n", tres_get_temp(&tres_ims));
=======
<<<<<<< HEAD
			//
			printf("pt1:%+2.1f\t", tres_get_temp(&tres_pirani_1));
			printf("pt2:%+2.1f\t", tres_get_temp(&tres_pirani_2));
			printf("pt0:%+2.1f\n", tres_get_temp(&tres_ims));
=======
			//printf("temp:%+2.1f\t", tres_get_temp(&tres_pirani_1));
			//printf("temp:%+2.1f\t", tres_get_temp(&tres_pirani_2));
			//printf("temp:%+2.1f\n", tres_get_temp(&tres_ims));
>>>>>>> main
>>>>>>> Stashed changes
			//
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
