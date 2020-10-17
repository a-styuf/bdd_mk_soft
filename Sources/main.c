#include "1986ve8_lib/cm4ikmcu.h"
#include <string.h>
#include <stdio.h>
#include "sysinit.h"
#include "wdt.h"
#include "uarts.h"
#include "debug.h"
#include "timers.h"
#include "adc.h"


extern type_ADC_model adc_0;
//
uint8_t n;
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
	
	Timers_Start(0, 1000);
	
	printf("BDD_MK is online");
	while(1) {
		WDRST;
		if (Timers_Status(0))
		{   
			Timers_Start(0, 1000); // перезапускаем таймер для формирования слота времени (возможная проблема - пропуск слота)
			//обработка процессов
			adc_process(&adc_0, 1000);
			//
			for(n=0; n<10; n++){

				printf("%d: %6.2f; ", n, adc_get_ch_voltage(&adc_0, n));
			}
			printf("temp:%+2.1f", get_mcu_temp(&adc_0));
		}
	}
}
