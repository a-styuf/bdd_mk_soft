  /**
  ******************************************************************************
  * @file           : gpio.c
  * @version        : v1.0
  * @brief          : функции для работы с GPIO. Явный недостаток - невозможность управления одновременно групой gpio.
  * @author			: Стюф Алексей/Alexe Styuf <a-styuf@yandex.ru>
  ******************************************************************************
  */

#include "gpio.h"

/**
  * @brief  инициализация модели GPIO
  * @param  gpio_ptr указатель на програмную модель устройства
  * @param  port указатель на порт устройства типа PortControl
  * @param  num номер GPIO внутри port
  */
void gpio_init(type_SINGLE_GPIO* gpio_ptr, PortControl* port, uint8_t num)
{
  gpio_ptr->port  = port;
  gpio_ptr->num = num;
}

/**
  * @brief  установка значения отдельного GPIO
  * @param  gpio_ptr указатель на програмную модель устройства
  * @param  val значения для установки (0 - 0, не 0 - 1)
  */
void gpio_set(type_SINGLE_GPIO* gpio_ptr, uint8_t val)
{
  if (val){
    gpio_ptr->port->SRXTX = (1<<(gpio_ptr->num));
  }
  else{
    gpio_ptr->port->CRXTX = (1<<(gpio_ptr->num));
  }
}

/**
  * @brief  чтение значения GPIO
  * @param  gpio_ptr указатель на програмную модель устройства
  * @retval  значение GPIO - 0 или 1
  */
uint8_t gpio_get(type_SINGLE_GPIO* gpio_ptr)
{
  return (gpio_ptr->port->RXTX >> gpio_ptr->num) & 0x1;
}
