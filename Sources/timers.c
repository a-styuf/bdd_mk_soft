#include "1986ve8_lib/cm4ikmcu.h"
#include "timers.h"


uint8_t timer_status = 0, high_byte_time_s = 0;
uint32_t interval_start_time = 0, interval_stop_time = 0;
uint8_t pwm_adder = 0, pwm_adder_cnter = 0;
uint16_t CCR1_base = 0;

void Timers_Init(void)
{
    CLK_CNTR->KEY = _KEY_;
    CLK_CNTR->PER0_CLK |= (1<<23)|(1<<24)|(1<<25);  //включение Timer0, Timer1, Timer2
    CLK_CNTR->TIM0_CLK = (1<<16)| 39;  //timer clock freq = 1 MHz
    CLK_CNTR->TIM1_CLK = (1<<16)| 249;  //timer clock freq = 160 kHz
    CLK_CNTR->TIM2_CLK = (1<<16)| 3;  //timer clock freq = 10 MHz
    // CLK_CNTR->TIM3_CLK - используется дл UART
    //
    MDR_TMR0->CNTRL = 0x00000000;
    MDR_TMR1->CNTRL = 0x00000000;
    MDR_TMR2->CNTRL = 0x00000000; // используется для PWM
    //Настраиваем работу таймеров
    //Таймер 0
    MDR_TMR0->CNT = 0x00000000;  //Начальное значение счетчика
    MDR_TMR0->PSG = 999;  //Предделитель частоты //из 1 МГц получаем 1 кГц
    MDR_TMR0->IE = 0x00000002;  //Разрешение генерировать прерывание при CNT = ARR
    //Таймер 1 - счетчик глобального времени
    MDR_TMR1->CNT = 0x00000000;  //Начальное значение счетчика
    MDR_TMR1->PSG = 624;  //Предделитель частоты //из 10 кГц получаем 256 Гц (время БКУ 6-ти байтовое, но используем только 5)
    MDR_TMR1->ARR = 0xFFFFFFFF;//считаем постоянно
    MDR_TMR1->IE = 0x00000002;  //Разрешение генерировать прерывание при CNT = ARR
    MDR_TMR1->CNTRL = 0x00000001;  //Счет вверх по TIM_CLK. Разрешение работы таймера
    MDR_TMR1->STATUS = 0x00;
    //Таймер 2 - ШИМ
    MDR_TMR2->CNT = 0x00000000;  //Начальное значение счетчика
    MDR_TMR2->PSG = 0;  //Предделитель частоты 1
    MDR_TMR2->IE = 0x00000002;  //Разрешение генерировать прерывание при CNT = ARR, передний фронту на входе ETR
    MDR_TMR2->CH1_CNTRL = 0x00000C00;  // OCCM = 6: 1, если DIR = 0 (счет вверх), CNT < CCR, иначе 0;
    MDR_TMR2->ARR = (100 - 1); //период PWM - 100kHz
    MDR_TMR2->CH1_CNTRL1 = 0x00000909;
    MDR_TMR2->CNTRL = 0x00000001;  //Счет вверх по TIM_CLK. Разрешение работы таймера
    //
    NVIC_EnableIRQ(IRQn_TMR0);
    NVIC_EnableIRQ(IRQn_TMR1);
    NVIC_EnableIRQ(IRQn_TMR2);
    
}

void INT_TMR0_Handler(void) 
{
    timer_status |= (1 << 0);
    MDR_TMR0->STATUS = 0x0000;
    MDR_TMR0->CNT = 0x00000000;
    MDR_TMR0->CNTRL = 0x00000000;
}

void INT_TMR1_Handler(void) 
{
    if (MDR_TMR1->STATUS & 0x02) high_byte_time_s += 1;
    MDR_TMR1->STATUS = 0;
}

void INT_TMR2_Handler(void) 
{
    MDR_TMR2->STATUS = 0x0000;
    pwm_adder_cnter += 1;
    pwm_adder_cnter &= 0x0F;
    if (pwm_adder_cnter == 0) MDR_TMR2->CCR1 = CCR1_base + 1;
    if (pwm_adder_cnter == pwm_adder) MDR_TMR2->CCR1 = CCR1_base;

}

// работа с измерительными интервалами: Timer0 используется в создании таймслотов - нигде больше не использовать; Timer1 - можно использовать в остальных местах, но следить за отсутствием одновременного использования
void Timers_Start(uint8_t num, uint32_t time_ms)
{  
    switch (num)
    {
        case 0:
            NVIC_DisableIRQ(IRQn_TMR0);
            MDR_TMR0->ARR = time_ms;  //задание времени в мс
            MDR_TMR0->CNT = 0x00000000;  //Начальное значение счетчика
            MDR_TMR0->CNTRL = 0x00000001;  //Счет вверх по TIM_CLK. Разрешение работы таймера
            NVIC_EnableIRQ(IRQn_TMR0);
            break;
    }
}

void Timers_Stop(uint8_t num)
{  
    switch (num)
    {
        case 0:
            NVIC_DisableIRQ(IRQn_TMR0);
            MDR_TMR0->CNTRL = 0x00000000;  //Счет вверх по TIM_CLK. Разрешение работы таймера
            MDR_TMR0->ARR = 0x00;  //задание времени в мс
            MDR_TMR0->CNT = 0x00000000;  //Начальное значение счетчика
            NVIC_EnableIRQ(IRQn_TMR0);
            break;
    }
}

uint8_t Timers_Status(uint8_t num) // если статус 0 - таймер еще считает; 1-закончил счет
{
    uint8_t status = 0x0000;
    NVIC_DisableIRQ(IRQn_TMR0);
    status = (timer_status & (1 << num));
    if (status != 0)
    {
        timer_status &= ~(1 << num);
    }
    NVIC_EnableIRQ(IRQn_TMR0);
    return status;
}

// работа с реальным временем
void Time_Set(uint64_t time, int16_t* diff_time_s, int8_t* diff_time_low) // time в 1/(2^16) секундах
{
    volatile int64_t diff_time = 0;
    uint64_t current_time = 0;
	typeCMTime cm_time = {0, 0, 0, 0};
	
    NVIC_DisableIRQ(IRQn_TMR2);
    //
    cm_time.low_part = 0x00;
    cm_time.mid_part = MDR_TMR1->CNT;
    cm_time.high_part = high_byte_time_s;
    cm_time.zero_part = 0x0000;
    //
    memcpy((uint8_t*)&current_time, (uint8_t*)&cm_time, 8);
    diff_time = time - current_time;
    //сохраняем системные данные связанные с синхронизацией
    *diff_time_s = (diff_time  >> 16) & 0xFFFF;
    *diff_time_low = (diff_time >> 8) & 0xFF;
    //сохраняем системные данные связанные с синхронизацией
    MDR_TMR1->CNT = (time >> 8) & 0xFFFFFFFF;
    high_byte_time_s = (time >> 40) & 0xFF;
    //
    NVIC_EnableIRQ(IRQn_TMR2);
}

uint32_t Get_Time_s(void) // получаем время от таймера, а потом обрезаем до секунд 
{
    uint32_t time_s = 0, time_s_old = 0;
    time_s_old = MDR_TMR1->CNT;
    time_s = ((uint64_t)high_byte_time_s << 24) + ((time_s_old >> 8) & 0xFFFFFF);
    return time_s;
}

void Get_Time_sec_parts(uint32_t* sec, uint8_t* parts) // получение полного времени
{
    uint32_t time_s_old = 0, time_s = 0;
	
    time_s_old = MDR_TMR1->CNT;
	time_s = ((uint32_t)high_byte_time_s << 24) + ((time_s_old >> 8) & 0xFFFFFF); 
    *sec = time_s;
    *parts = time_s_old & 0xFF;
}

// работа с интервалами на основе реального времени - недоделано и пока не используется
void SetInterval(uint16_t interval) //фиксируем запуск ожидания конца указанного интервала
{
    interval_start_time = Get_Time_s();
    interval_stop_time = interval_start_time + interval;
}

uint8_t Check_Interval(void) //проверка срабатывания интервала
{
    if ((interval_stop_time <= Get_Time_s()) & (interval_start_time != 0))  
    {
        interval_start_time = 0;
        interval_stop_time = 0;
        return 1;
    }
    else if (interval_start_time >= Get_Time_s())
    {
        interval_start_time = 0;
        interval_stop_time = 0;
        return 1;
    }
    else
    {
        return 0;
    }    
}

/**
  * @brief  установка значения PWM для таймера 2
  * @param  pwm_val знчение заполеннности PWM; 0-999 в 0.1%
  */
void Timer_PWM_Set(uint16_t pwm_val)
{   
    if (pwm_val >= 100){
        MDR_TMR2->CCR1 = 99;
    }
    else{
        MDR_TMR2->CCR1 = pwm_val;
    }
}

/**
  * @brief  установка значения PWM для таймера 2 в float
  * @param  pwm_val_fp знчение заполеннности PWM; 0-99 в 1%, дробная часть также пработает до точности 1/(1<<16)
  */
void Timer_PWM_Set_Fp(float pwm_val_fp)
{   
    if (pwm_val_fp >= 100){
        CCR1_base = 99;
        pwm_adder = 0.9*(1<<4);
    }
    else{
        CCR1_base = (uint16_t)pwm_val_fp;
        pwm_adder = (uint32_t)(pwm_val_fp*(1<<4))&0xF;
    }
}

