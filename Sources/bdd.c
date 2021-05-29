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
int8_t bdd_init(type_BDD_model* bdd_ptr, type_MPI_model* mpi_ptr)
{
  uint8_t i = 0;
  //GPIO
  for (i=0; i<8; i++){
    gpio_init(&bdd_ptr->gpio[i], PORTE, 16+i);
    gpio_set(&bdd_ptr->gpio[i], 0);
  }
  //STM
  for (i=0; i<10; i++){
    gpio_init(&bdd_ptr->stm_io[i], PORTE, 6+i);
    gpio_set(&bdd_ptr->stm_io[i], 0);
  }
  stm_bdd_init(&bdd_ptr->stm, bdd_ptr->stm_io);
  //инициализация устройств МК
  adc_init(&bdd_ptr->adc_0);
  dac_init(&bdd_ptr->dac);
  mvip_init(&bdd_ptr->mvip, &bdd_ptr->adc_0.ch[9], &bdd_ptr->adc_0.ch[8]);
  //mko
  bdd_ptr->mpi_ptr = mpi_ptr;
  //инициализация измерителей
  tres_init(&bdd_ptr->tres_ims, &bdd_ptr->adc_0.ch[1]);
  tres_init(&bdd_ptr->tres_pirani_1, &bdd_ptr->adc_0.ch[4]);
  tres_init(&bdd_ptr->tres_pirani_2, &bdd_ptr->adc_0.ch[7]);
  //
  oai_dd_init(&bdd_ptr->oai_dd_1, 1, &bdd_ptr->tres_pirani_1, &bdd_ptr->adc_0.ch[2], &bdd_ptr->adc_0.ch[3], &bdd_ptr->dac.ch[0], V_A, V_B, I_A, I_B);
  oai_dd_init(&bdd_ptr->oai_dd_2, 2, &bdd_ptr->tres_pirani_2, &bdd_ptr->adc_0.ch[5], &bdd_ptr->adc_0.ch[6], &bdd_ptr->dac.ch[1], V_A, V_B, I_A, I_B);
  ims_init(&bdd_ptr->ims, &bdd_ptr->tres_ims, &bdd_ptr->adc_0.ch[0], &bdd_ptr->mvip, &bdd_ptr->gpio[4], &bdd_ptr->gpio[2], &bdd_ptr->gpio[3]);
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
  bdd_ptr->control.time_slot_cnter += 1;
  //
  if ((bdd_ptr->control.time_slot_cnter % 1) == 0) {  // тайм-слот - period_ms * 1
    //обработка процессов
    adc_process(&bdd_ptr->adc_0, period_ms);
    ims_process(&bdd_ptr->ims, period_ms);
    mvip_process(&bdd_ptr->mvip, period_ms);
  }
  if ((bdd_ptr->control.time_slot_cnter % 10) == 0){  // тайм-слот - period_ms * 10
    //обработка oai_dd
    oai_dd_process(&bdd_ptr->oai_dd_1, period_ms*10);
    oai_dd_process(&bdd_ptr->oai_dd_2, period_ms*10);
    //формирование кадров для отображения в МКО и выкладывание их на подпадреса
    bdd_system_frame_form(bdd_ptr);
    mpi_wr_to_subaddr(BDD_MK_FRAME_SADDR_SYSTEM, (uint16_t*)&bdd_ptr->frame.system);
    bdd_oai_dd_frame_form(bdd_ptr);
    mpi_wr_to_subaddr(BDD_MK_FRAME_SADDR_OAI_DD, (uint16_t*)&bdd_ptr->frame.oai_dd);
    bdd_ims_dd_frame_form(bdd_ptr);
    mpi_wr_to_subaddr(BDD_MK_FRAME_SADDR_IMS_DD, (uint16_t*)&bdd_ptr->frame.ims_dd);
  }
  if ((bdd_ptr->control.time_slot_cnter % 100) == 0){  // тайм-слот - period_ms * 100
    //
  }
  // stm
  stm_bdd_process(&bdd_ptr->stm, bdd_ptr->pressure_16, bdd_ptr->temp_bdd, bdd_ptr->current_HV, period_ms);
}

/**
  * @brief  формирование кадра с параметрами работы ОАИ ДД
  * @param  bdd_ptr указатель на програмную модель устройства
  */
void bdd_oai_dd_frame_form(type_BDD_model* bdd_ptr)
{
  bdd_ptr->frame.oai_dd.label = 0x0FF1;
  bdd_ptr->frame.oai_dd.definer = mpi_int_frame_definer(
                                                        BDD_MK_FRAME_MODIFICATOR, 
                                                        BDD_MK_FRAME_DEVICE_NUMBER, 
                                                        BDD_MK_FRAME_FABRICATION_NUM,
                                                        BDD_MK_FRAME_TYPE_OAI_DD);
  bdd_ptr->frame.oai_dd.num = 0x00;
  bdd_ptr->frame.oai_dd.time = Get_Time_s();
  //
  memcpy((uint8_t*)bdd_ptr->frame.oai_dd.oai_dd_1_report, (uint8_t*)&bdd_ptr->oai_dd_1.report, sizeof(type_OAI_Frame_Report));
  memcpy((uint8_t*)bdd_ptr->frame.oai_dd.oai_dd_2_report, (uint8_t*)&bdd_ptr->oai_dd_2.report, sizeof(type_OAI_Frame_Report));
  //
  bdd_ptr->frame.oai_dd.crc16 = mpi_int_crc16((uint8_t*)&bdd_ptr->frame.oai_dd, 62);
}

/**
  * @brief  формирование кадра с параметрами работы ИМД датчика давления
  * @param  bdd_ptr указатель на програмную модель устройства
  */
void bdd_ims_dd_frame_form(type_BDD_model* bdd_ptr)
{
  bdd_ptr->frame.ims_dd.label = 0x0FF1;
  bdd_ptr->frame.ims_dd.definer = mpi_int_frame_definer(
                                                        BDD_MK_FRAME_MODIFICATOR, 
                                                        BDD_MK_FRAME_DEVICE_NUMBER, 
                                                        BDD_MK_FRAME_FABRICATION_NUM,
                                                        BDD_MK_FRAME_TYPE_IMS_DD);
  bdd_ptr->frame.ims_dd.num = 0x00;
  bdd_ptr->frame.ims_dd.time = Get_Time_s();
  //
  memcpy((uint8_t*)bdd_ptr->frame.ims_dd.ims_dd_report, (uint8_t*)&bdd_ptr->ims.report, sizeof(type_IMS_Frame_Report));
  //
  bdd_ptr->frame.ims_dd.crc16 = mpi_int_crc16((uint8_t*)&bdd_ptr->frame.ims_dd, 62);
}

/**
  * @brief  формирование кадра с общими параметрами
  * @param  bdd_ptr указатель на програмную модель устройства
  */
void bdd_system_frame_form(type_BDD_model* bdd_ptr)
{
  bdd_ptr->frame.system.label = 0x0FF1;
  bdd_ptr->frame.system.definer = mpi_int_frame_definer(
                                                        BDD_MK_FRAME_MODIFICATOR, 
                                                        BDD_MK_FRAME_DEVICE_NUMBER, 
                                                        BDD_MK_FRAME_FABRICATION_NUM,
                                                        BDD_MK_FRAME_TYPE_SYSTEM);
  bdd_ptr->frame.system.num = 0x00;
  bdd_ptr->frame.system.time = Get_Time_s();
  //
  bdd_ptr->frame.system.press_general = 0x00;
  bdd_ptr->frame.system.temp_general = 0x00;
  bdd_ptr->frame.system.current_24v = 0x00;
  //
  bdd_ptr->frame.system.pressure[0] = bdd_ptr->oai_dd_1.pressure_u16;
  bdd_ptr->frame.system.pressure[1] = bdd_ptr->oai_dd_2.pressure_u16;
  bdd_ptr->frame.system.pressure[2] = 0x00;
  bdd_ptr->frame.system.temp[0] = bdd_ptr->oai_dd_1.t_res_ptr->temp_u16;
  bdd_ptr->frame.system.temp[1] = bdd_ptr->oai_dd_2.t_res_ptr->temp_u16;
  bdd_ptr->frame.system.temp[2] = 0x00;
  //
  bdd_ptr->frame.system.mode = bdd_ptr->control.mode;
  bdd_ptr->frame.system.state = bdd_ptr->control.state;
  bdd_ptr->frame.system.error = bdd_ptr->control.error;
  bdd_ptr->frame.system.error_cnt = bdd_ptr->control.error_cnt;
  //
  bdd_ptr->frame.system.crc16 = mpi_int_crc16((uint8_t*)&bdd_ptr->frame.system, 62);
}
