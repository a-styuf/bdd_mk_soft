#ifndef _STM_BDD_H_
#define _STM_BDD_H_

#include <string.h>
#include "1986ve8_lib/cm4ikmcu.h"
#include "gpio.h"

#define STM_BDD_KPDD 0
#define STM_BDD_MSOB 1
#define STM_BDD_ADDR 2
#define STM_BDD_DATA 6

#define STM_BDD_NUM 10
#define STM_BDD_DATA_PARTS_NUM 16

//MSOB
#define STM_BDD_MSOB_ON_MS (2000)
#define STM_BDD_MSOB_PERIOD_MS (1800000)

#define STM_BDD_ADDR_PERIOD_MS (1000)

#define STM_BDD_DEBUG_TIMEOUT_MAX_MS (18000000)

#pragma pack(2)
/** 
  * @brief  структура управления одним GPIO
  */
typedef struct
{
  type_SINGLE_GPIO* gpio;
  uint16_t state;
  uint8_t addr, data[STM_BDD_DATA_PARTS_NUM];
  uint32_t msob_timeout;
  uint32_t addr_timeout;
  uint32_t debug_timeout;
} type_STM_BDD;

//
void stm_bdd_init(type_STM_BDD* stm_ptr, type_SINGLE_GPIO* stm_io);
void stm_bdd_process(type_STM_BDD* stm_ptr, uint16_t pressure, uint8_t tempretaure, uint8_t current, uint32_t period_ms);
void stm_bdd_set(type_STM_BDD* stm_ptr, uint16_t stm_val);
void _stm_bdd_data_form(type_STM_BDD* stm_ptr, uint16_t pressure, uint8_t tempretaure, uint8_t current);
void stm_bdd_debug_set(type_STM_BDD* stm_ptr, uint16_t stm_val, uint32_t timeout_ms);

#endif



