/**
  ******************************************************************************
  * @file           : sysinit.c
  * @version        : v1.0
  * @brief          : первичная инициализация МК для БДД-МК
  * @author         : Стюф Алексей/Alexe Styuf <a-styuf@yandex.ru>
  ******************************************************************************
  */
#include "1986ve8_lib/cm4ikmcu.h"


#ifdef _EXT_MEM_RLS_
const uint64_t ExtBusCfgCode[] __attribute__ ((at(0x10000400))) = {0x71, 0x71, 0,0,0,0,0,0,0,0};
#endif


extern uint32_t __Vectors;

void System_Init() {
  *((uint32_t*)0xE000ED08) = (uint32_t)&__Vectors;

  BKP->KEY = _KEY_;
  BKP->REG_60_TMR0 |= 1<<28; // diasable POR
  BKP->REG_60_TMR1 |= 1<<28; //
  BKP->REG_60_TMR2 |= 1<<28; //
    
  FT_CNTR->KEY = _KEY_;
  FT_CNTR->RESET_EVENT0 = 0x0; // hide all errors
  FT_CNTR->RESET_EVENT1 = 0x0; //
  FT_CNTR->RESET_EVENT2 = 0x0; //
  FT_CNTR->RESET_EVENT3 = 0x0; //
  FT_CNTR->RESET_EVENT4 = 0x0; //

  /*-------------------- Ports Init --------------------------*/
  CLK_CNTR->KEY = _KEY_;
  CLK_CNTR->PER0_CLK |= (1<<13)|(1<<14)|(1<<15)|(1<<16)|(1<<17);   //port A,B,C,D,E clock enable

  PORTA->KEY = _KEY_;
  PORTA->SANALOG =   0xFFFFFFFF;
  PORTA->CFUNC[2]  = 0xF0FF0000;
  PORTA->SFUNC[2] =  0xC0CC0000;
  PORTA->CFUNC[3]  = 0x00FF0FFF;
  PORTA->SFUNC[3] =  0x00CC0CCC;
  PORTA->SPWR[1] =   0x003C0F00;
  PORTA->SPULLDOWN = 0xFFFFFFFF;

  PORTB->KEY = _KEY_;
  PORTB->SANALOG =   0xFFFFFFFF;
  PORTB->CFUNC[0] =  0x0000FFFF;
  PORTB->SFUNC[0] =  0x00000000;
  PORTB->CFUNC[1] =  0xFFFFFFF0;
  PORTB->SFUNC[1] =  0x50000000;  // UART0
  PORTB->CFUNC[2] =  0xFF00000F;
  PORTB->SFUNC[2] =  0x99000005;
  PORTB->SPWR[0] =   0x80000000;
  PORTB->SPWR[1] =   0x0000A002;
  PORTB->SPULLDOWN = 0x00C00000;
  PORTB->SPULLUP   = 0x0001860F;

  PORTC->KEY = _KEY_;
  PORTC->SANALOG =   0xEDFC00FF;
  PORTC->CANALOG =   0x0203FF00;
  PORTC->CFUNC[0] =  0xFFFFFFFF;
  PORTB->SFUNC[0] =  0x00000000;  
  PORTC->SPULLDOWN = 0x3FFC00FF;
  
  PORTD->KEY = _KEY_;
  PORTD->SANALOG =   0xFFFFFFFF;
  PORTD->SPWR[0] =   0x0FFFFFFF;
  PORTD->SPWR[1] =   0xF003C3C0;
  PORTD->SPULLDOWN = 0x3E97E000;
  
  PORTE->KEY = _KEY_;
  PORTE->SANALOG =   0xFFFFFFFF;
  PORTE->CFUNC[0] =  0xFF000000;
  PORTE->SFUNC[0] =  0x00000000;
  PORTE->CFUNC[1] =  0xFFFFFFFF;
  PORTE->SFUNC[1] =  0x00000000;
  PORTE->CFUNC[2] =  0xFFFFFFFF;
  PORTE->SFUNC[2] =  0x00000000;
  PORTE->SOE      =  0x00FFFFC0;
  PORTE->SPWR[0]  =   0x55555FFF;
  PORTE->SPWR[1]  =   0x00005555;
  
  /*------------ enable regions -----------*/
  EXT_BUS_CNTR->KEY = _KEY_;
  EXT_BUS_CNTR->RGN0_CNTRL = 0x4000361;
  EXT_BUS_CNTR->RGN2_CNTRL = 0x321;

  /*---------- System clock, PLL0 ----------*/
  CLK_CNTR->KEY = _KEY_;
  CLK_CNTR->HSE0_CLK = (1<<27)|(1<<28); // Enable HSE0 gen
  while((CLK_CNTR->HSE0_STAT & (1 << 20)) == 0); //wait HSE0
  CLK_CNTR->PLL0_CLK = (2<<29)|(1<<28)|(1<<27)|(8<<8)|(1<<0);  //PLL0 On, Fout = 10*8/2 = 40MHz
  while((CLK_CNTR->PLL0_CLK & (1<<28)) == 0); //wait PLL ready
  CLK_CNTR->MAX_CLK = 8;  //MAX_CLOCK = PLL0 = 40 MHz
  CLK_CNTR->CPU_CLK = 0;  //Core clock: MAX_CLOCK

  /*---------- Clock for ADC0 -------------*/
  CLK_CNTR->ADC0_CLK = ((uint32_t)1<<28)|(1<<16)| 7;  //HSE0 clock for ADC, freq=0.5 MHz
  CLK_CNTR->DAC0_CLK = ((uint32_t)1<<28)|(1<<16)| 7;  //HSE0 clock for DAC0, freq=0.5 MHz
  CLK_CNTR->DAC1_CLK = ((uint32_t)1<<28)|(1<<16)| 7;  //HSE0 clock for DAC1, freq=0.5 MHz

#ifndef _INT_RAM_
  ICACHE->KEY = _KEY_;
  ICACHE->CNTL = 0x0B;
#endif
}



