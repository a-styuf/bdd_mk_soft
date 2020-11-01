#ifndef _timer_H_
#define _timer_H_

#include <string.h>
#include <stdio.h>
#include <math.h>

#define IRQn_TMR0 (IRQn_Type)93
#define IRQn_TMR1 (IRQn_Type)94
#define IRQn_TMR2 (IRQn_Type)95

#pragma pack(2)
typedef struct  //структура для удобства работы с временем в 1/256 секунд
{
    uint32_t mid_part;
    uint8_t low_part;
    uint8_t high_part;
    uint16_t zero_part;
}typeCMTime;

void Timers_Init(void);
void Timers_Start(uint8_t num, uint32_t time_ms);
void Timers_Stop(uint8_t num);
uint8_t Timers_Status(uint8_t num);
void Time_Set(uint64_t time, int16_t* diff_time_s, int8_t* diff_time_low); // time в 1/(2^16) секундах
uint32_t Get_Time_s(void);
void Get_Time_sec_parts(uint32_t* sec, uint8_t* parts);
  
#endif

