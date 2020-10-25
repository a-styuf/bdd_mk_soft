  /**
  ******************************************************************************
  * @file           : gpio.c
  * @version        : v1.0
  * @brief          : функции для работы с GPIO. Явный недостаток - невозможность управления одновременно групой gpio.
  * @author			: Стюф Алексей/Alexe Styuf <a-styuf@yandex.ru>
  ******************************************************************************
  */

#include "gpio.h"

void gpio_set(type_SINGLE_GPIO* gpio_ptr, uint8_t val)
{
  if (val){
    gpio_ptr->port->SRXTX = (1<<(gpio_ptr->num));
  }
  else{
    gpio_ptr->port->CRXTX = (1<<(gpio_ptr->num));
  }
}


uint8_t gpio_get(type_SINGLE_GPIO* gpio_ptr)
{
  return (gpio_ptr->port->RXTX >> gpio_ptr->num) & 0x1;
}
