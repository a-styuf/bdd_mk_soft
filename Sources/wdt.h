#ifndef _WDT_H_
#define _WDT_H_


#define WDRST do{WDT->KEY=0x5555; WDT->KEY=0xAAAA;}while(0)


int WDT_Init(void);


#endif
