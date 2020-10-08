  /**
  ******************************************************************************
  * @file           : debug.c
  * @version        : v1.0
  * @brief          : функции для отладки, отключаемы define DEBUG
  * @author			: Стюф Алексей/Alexe Styuf <a-styuf@yandex.ru>
  ******************************************************************************
  */

#include "debug.h"

/**
  * @brief  вывод на экран строки
  * @param  str: 0-терминатед строка для вывода в отладочный UART
  */
void dbg_print(char *buff)
{
#ifdef DEBUG
  if (strlen(buff) >= 127){
    buff[127] = 0;
  }
  UART0_SendPacket((uint8_t*)buff, strlen(buff), UART_CRC_DIS);
#endif
}
