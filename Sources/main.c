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
#include "termo_res.h"
#include "oai_dd.h"

type_ADC_model adc_0;
type_DAC_model dac;
type_TRES_model tres_ims, tres_pirani_1, tres_pirani_2;
type_OAI_DD_model oai_dd_1, oai_dd_2;
//
type_SINGLE_GPIO rele_gpio, ku_gpio[2];
//
char report_str[128] = {0};
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
	oai_dd_init(&oai_dd_1, 1, &tres_pirani_1, &adc_0.ch[2], &adc_0.ch[3], &dac.ch[0], V_A, V_B, I_A, I_B);
	oai_dd_init(&oai_dd_2, 2, &tres_pirani_2, &adc_0.ch[5], &adc_0.ch[6], &dac.ch[1], V_A, V_B, I_A, I_B);
	
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
	//
	Timers_Start(0, 1000);
	
	printf("BDD_MK is online\n");
	while(1) {
		WDRST;
		if (Timers_Status(0))
		{   
			Timers_Start(0, 2000); // перезапускаем таймер для формирования слота времени (возможная проблема - пропуск слота)
			//обработка процессов
			adc_process(&adc_0, 1000);
			oai_dd_process(&oai_dd_1, 1000);
			oai_dd_process(&oai_dd_2, 1000);
			// получение отчета о работе
			oai_dd_get_str_report(&oai_dd_1, report_str);
			printf("%s\t", report_str);
			pid_get_str_report(&oai_dd_1.pid_res, report_str);
			printf("%s\n", report_str);
			// проверка работы реле
			if (gpio_get(&rele_gpio)){
				gpio_set(&rele_gpio, 0);
			}
			else {
				gpio_set(&rele_gpio, 1);
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
