  /**
  ******************************************************************************
  * @file           : stm_bdd.c
  * @version        : v1.0
  * @brief          : функция обработки СТМ для БДД_МК
  * @author			    : Стюф Алексей/Alexe Styuf <a-styuf@yandex.ru>
  ******************************************************************************
  */

#include "stm_bdd.h"

/**
  * @brief  инициализация модели stm
  * @param  stm_ptr указатель на програмную модель устройства
  */
void stm_bdd_init(type_STM_BDD* stm_ptr, type_SINGLE_GPIO* stm_io)
{
  stm_ptr->gpio = stm_io;
  stm_bdd_set(stm_ptr, 0x00);
  stm_ptr->state = 0;
  stm_ptr->msob_timeout = 0;
  stm_ptr->addr_timeout = 0;
  stm_ptr->debug_timeout = 0;
  stm_ptr->addr = 0;
  memset((uint8_t*)stm_ptr->data, 0x00, STM_BDD_DATA_PARTS_NUM);
}

/**
  * @brief  обработка параметров СТМ
  * @param  pressure значение давления
  * @param  tempretaure значение температуры
  * @param  current значение потребления высоковольного ВИП
  * @param  period_ms указатель на програмную модель устройства
  */
void stm_bdd_process(type_STM_BDD* stm_ptr, uint16_t pressure, uint8_t tempretaure, uint8_t current, uint32_t period_ms)
{
  if (stm_ptr->debug_timeout == 0){
    // KPDD
    stm_ptr->state &= ~(1<<STM_BDD_KPDD);
    stm_ptr->state |= (1<<STM_BDD_KPDD);
    // MSOB
    if (stm_ptr->msob_timeout > STM_BDD_MSOB_PERIOD_MS) stm_ptr->msob_timeout = 0;
    else stm_ptr->msob_timeout += period_ms;

    if ((stm_ptr->msob_timeout >= 0) && (stm_ptr->msob_timeout < STM_BDD_MSOB_ON_MS)){
      stm_ptr->state &= ~(1<<STM_BDD_MSOB);
      stm_ptr->state |= (1<<STM_BDD_MSOB);
    }
    else{
      stm_ptr->state &= ~(1<<STM_BDD_MSOB);
    }
    // ADDR and DATA
    _stm_bdd_data_form(stm_ptr, pressure, tempretaure, current);
    if (stm_ptr->addr_timeout > STM_BDD_ADDR_PERIOD_MS) {
      stm_ptr->addr_timeout = 0;
      stm_ptr->addr += 1;
      if (stm_ptr->addr >= STM_BDD_DATA_PARTS_NUM) {
        stm_ptr->addr = 0;
      }
    }
    else stm_ptr->addr_timeout += period_ms;
    stm_ptr->state &= ~(0x0F<<STM_BDD_ADDR);
    stm_ptr->state |= ((stm_ptr->addr)<<STM_BDD_ADDR);
    stm_ptr->state &= ~(0x0F<<STM_BDD_DATA);
    stm_ptr->state |= ((stm_ptr->data[stm_ptr->addr])<<STM_BDD_ADDR);
  }
  else{
    stm_ptr->debug_timeout -= period_ms;
    if (stm_ptr->debug_timeout > STM_BDD_DEBUG_TIMEOUT_MAX_MS){
      stm_ptr->debug_timeout = 0;
    }
  }
}

/**
  * @brief  установка значения все STM
  * @param  stm_ptr указатель на програмную модель устройства
  * @param  stm_val значения для установки (используются только 9-0)
  */
void stm_bdd_set(type_STM_BDD* stm_ptr, uint16_t stm_val)
{
  uint8_t i = 0;
  stm_ptr->state = stm_val;
  for(i=0; i<STM_BDD_NUM; i++){
    gpio_set(&stm_ptr->gpio[i], (stm_val>>i)&0x01);
  }
}

/**
  * @brief  формирования кадра данных
  * @param  stm_ptr указатель на програмную модель устройства
  * @param  pressure значение давления
  * @param  tempretaure значение температуры
  * @param  current значение потребления высоковольного ВИП
  */
void _stm_bdd_data_form(type_STM_BDD* stm_ptr, uint16_t pressure, uint8_t tempretaure, uint8_t current)
{
  uint8_t i = 0;
  uint32_t data_word = 0x00;
  data_word = ((pressure & 0xFFFF) << 16) | ((tempretaure & 0xFF) << 8) | ((current & 0xFF) << 0);
  for (i = 0; i < STM_BDD_DATA_PARTS_NUM; i++){
    stm_ptr->data[i] = ((data_word >> (4*i)) & 0x0F);
  }
}

/**
  * @brief  установка значения всех STM, блокирующая на timeout другие изменения
  * @param  stm_ptr указатель на програмную модель устройства
  * @param  stm_val значения для установки (используются только 9-0)
  * @param  timeout_ms время блокировки изменения в мс
  */
void stm_bdd_debug_set(type_STM_BDD* stm_ptr, uint16_t stm_val, uint32_t timeout_ms)
{
  stm_bdd_set(stm_ptr, stm_val);
  stm_ptr->debug_timeout = timeout_ms;
}
