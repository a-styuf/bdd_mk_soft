#ifndef _GPIO_H_
#define _GPIO_H_

#include "1986ve8_lib/cm4ikmcu.h"


/** 
  * @brief  структура управления одним GPIO
  */
typedef struct
{
  PortControl* port;
  uint8_t num;
} type_SINGLE_GPIO;

//
void gpio_set(type_SINGLE_GPIO* gpio_ptr, uint8_t val);
uint8_t gpio_get(type_SINGLE_GPIO* gpio_ptr);

#endif



