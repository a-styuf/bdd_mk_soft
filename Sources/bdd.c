/**
  ******************************************************************************
  * @file           : bdd.c
  * @version        : v1.0
  * @brief          : библиотека для работы с програмной моделью БДД
  * @author			: Стюф Алексей/Alexe Styuf <a-styuf@yandex.ru>
  ******************************************************************************
  */

#include "bdd.h"

/**
  * @brief  инициализация програмной модели БДД
  * @param  bdd_ptr указатель на програмную модель устройства
  * @retval  0
  */
int8_t bdd_init(type_BDD_model* bdd_ptr)
{
  //инициализация устройств МК
  adc_init(&bdd_ptr->adc_0);
  dac_init(&bdd_ptr->dac);
  //
  // gpio tmp
	bdd_ptr->rele_gpio.num = 20;
	bdd_ptr->rele_gpio.port = PORTE;
	gpio_set(&bdd_ptr->rele_gpio, 0);
	bdd_ptr->ku_gpio[0].num = 18;
	bdd_ptr->ku_gpio[0].port = PORTE;
	gpio_set(&bdd_ptr->ku_gpio[0], 0);
	bdd_ptr->ku_gpio[1].num = 19;
	bdd_ptr->ku_gpio[1].port = PORTE;
	gpio_set(&bdd_ptr->ku_gpio[1], 0);
	//
  //инициализация измерителей
  tres_init(&bdd_ptr->tres_ims, &bdd_ptr->adc_0.ch[1]);
  tres_init(&bdd_ptr->tres_pirani_1, &bdd_ptr->adc_0.ch[4]);
  tres_init(&bdd_ptr->tres_pirani_2, &bdd_ptr->adc_0.ch[7]);
  //
  oai_dd_init(&bdd_ptr->oai_dd_1, 1, &bdd_ptr->tres_pirani_1, &bdd_ptr->adc_0.ch[2], &bdd_ptr->adc_0.ch[3], &bdd_ptr->dac.ch[0], V_A, V_B, I_A, I_B);
  oai_dd_init(&bdd_ptr->oai_dd_2, 2, &bdd_ptr->tres_pirani_2, &bdd_ptr->adc_0.ch[5], &bdd_ptr->adc_0.ch[6], &bdd_ptr->dac.ch[1], V_A, V_B, I_A, I_B);
  //
  return 0;
}

/**
  * @brief  обработка состояния БДД
  * @param  bdd_ptr указатель на програмную модель устройства
  */
void bdd_ctrl_reset(type_BDD_model* bdd_ptr)
{
  bdd_ptr->control.mode = 0x00;
  bdd_ptr->control.state = 0x00;
  bdd_ptr->control.error = 0x00;
  bdd_ptr->control.error_cnt = 0x00;
  bdd_ptr->control.time_slot_cnter = 0x00;
}

/**
  * @brief  обработка состояния БДД
  * @param  bdd_ptr указатель на програмную модель устройства
  */
void bdd_process(type_BDD_model* bdd_ptr, uint8_t period_ms)
{
  char report_str[128] = {0};
  //
  bdd_ptr->control.time_slot_cnter += 1;
  //
  if ((bdd_ptr->control.time_slot_cnter % 1) == 0) {  // тайм-слот - period_ms * 1
    //обработка процессов
    adc_process(&bdd_ptr->adc_0, period_ms);
    oai_dd_process(&bdd_ptr->oai_dd_1, period_ms);
    oai_dd_process(&bdd_ptr->oai_dd_2, period_ms);
  }
  if ((bdd_ptr->control.time_slot_cnter % 10) == 0){  // тайм-слот - period_ms * 10
    // получение отчета о работе
    oai_dd_get_str_report(&bdd_ptr->oai_dd_1, report_str);
    printf("%s | ", report_str);
    oai_dd_get_str_report(&bdd_ptr->oai_dd_2, report_str);
    printf("%s\n", report_str);
  }
  if ((bdd_ptr->control.time_slot_cnter % 100) == 0){  // тайм-слот - period_ms * 100
    //
  }
}

