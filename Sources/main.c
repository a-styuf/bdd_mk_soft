#include "1986ve8_lib/cm4ikmcu.h"
#include <string.h>
#include <stdio.h>
#include "sysinit.h"
#include "wdt.h"
#include "bdd.h"
#include "mvip.h"

#define SOFT_VERSION "0.1.1"
//
extern type_MPI_model mpi;
type_BDD_model bdd;
//
char report_str[128] = {0};
uint8_t n, m=0;
uint16_t uint16_var;
float var_float = 0;
//
int main() {
	// инициализация переферии
	System_Init();
	WDT_Init();
	UART0_Init();
	Timers_Init();
	//инициализация МПИ
  mpi_init(MPI_ADDRES_DEFAULT);
	//
	Timers_Start(0, 1000);
	//
	bdd_init(&bdd, &mpi);
	printf("BDD MKO: v%s\n", SOFT_VERSION);
	//
	while(1) {
		WDRST;
		if (Timers_Status(0)) //timeslot 10 ms
		{   
			// smallest time slot - 10ms
			Timers_Start(0, 10); // перезапускаем таймер для формирования слота времени (возможная проблема - пропуск слота)
			//
			bdd_process(&bdd, 10);
		}
		// обработка приема данных ОУ МПИ
		if (mpi_process() != 0){
			if (mpi.subaddr == BDD_MK_FRAME_SADDR_COMMAND) {
				if (mpi.data[0] == 0x0FF1) {  //  проверка метки кадра (слово 0), слова с 1 по 4  - служебная информация,
					switch (mpi.data[5])
					{
					case BDD_MK_COMMAND_LINK_CHECK:
						break;
					case BDD_MK_COMMAND_SET_OAI_DD_1_MODE:
						oai_dd_set_mode(&bdd.oai_dd_1, (uint8_t)mpi.data[6]&0xFF);
						break;
					case BDD_MK_COMMAND_SET_OAI_DD_2_MODE:
						oai_dd_set_mode(&bdd.oai_dd_2, (uint8_t)mpi.data[6]&0xFF);
						break;
					case BDD_MK_COMMAND_SET_OAI_DD_1_FILTER:
						filter_parameter_set(&bdd.oai_dd_1.filter_res, mpi.data[6]/256., 0.1);
						filter_parameter_set(&bdd.oai_dd_1.filter_curr, mpi.data[7]/256., 0.1);
						filter_parameter_set(&bdd.oai_dd_1.filter_voltage, mpi.data[8]/256., 0.1);
						break;
					case BDD_MK_COMMAND_SET_OAI_DD_2_FILTER:
						filter_parameter_set(&bdd.oai_dd_2.filter_res, mpi.data[6]/256., 0.1);
						filter_parameter_set(&bdd.oai_dd_2.filter_curr, mpi.data[7]/256., 0.1);
						filter_parameter_set(&bdd.oai_dd_2.filter_voltage, mpi.data[8]/256., 0.1);
						break;
					case BDD_MK_COMMAND_SET_OAI_DD_1_PID_SETTING:
						pid_set_desired_value(&bdd.oai_dd_1.pid_res, mpi.data[6]/256.);
						pid_set_desired_value(&bdd.oai_dd_1.pid_current, mpi.data[7]/256.);
						pid_set_coeff(&bdd.oai_dd_1.pid_res, PID_R_K, mpi.data[8]/256., mpi.data[9]/256., mpi.data[10]/256.);
						pid_set_coeff(&bdd.oai_dd_1.pid_current, PID_R_K, mpi.data[11]/256., mpi.data[12]/256., mpi.data[13]/256.);
						break;
					case BDD_MK_COMMAND_SET_OAI_DD_2_PID_SETITNG:
						pid_set_desired_value(&bdd.oai_dd_2.pid_res, mpi.data[6]/256.);
						pid_set_desired_value(&bdd.oai_dd_2.pid_current, mpi.data[7]/256.);
						pid_set_coeff(&bdd.oai_dd_2.pid_res, PID_R_K, mpi.data[8]/256., mpi.data[9]/256., mpi.data[10]/256.);
						pid_set_coeff(&bdd.oai_dd_2.pid_current, PID_R_K, mpi.data[11]/256., mpi.data[12]/256., mpi.data[13]/256.);
						break;
					default:
						break;
					}
				}
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
	NVIC_DisableIRQ(IRQn_ADC0);
	//
	gpio_set(&bdd.stm[9], 1);
  while(bdd.adc_0.reg->STATUS & 1) {
    rslt = bdd.adc_0.reg->RESULT;
    chn = *((uint16_t*)&rslt + 1);
    val = *((uint16_t*)&rslt) << 4;
		adc_new_val_process(&bdd.adc_0.ch[chn], val);
  }
	gpio_set(&bdd.stm[9], 0);
	//
	NVIC_EnableIRQ(IRQn_ADC0);
}
