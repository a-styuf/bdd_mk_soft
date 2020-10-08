#include "1986ve8_lib/cm4ikmcu.h"

//~ 0.8 sec
#define WD_TIME    25000


int WDT_Init() {
  int i;
  WDT->KEY = 0x5555;
  if(WDT->KEY == 0x5555) {
    WDT->PRL = WD_TIME;
    if(WDT->PRL == WD_TIME) {
      WDT->KEY = 0xAAAA;
      for(i=0; i<200; i++);
      if(WDT->CNT == WD_TIME) {
        WDT->KEY = 0xCCCC;
        if(WDT->KEY == 0xCCCC) {
          WDT->EN = 0x3333;
          return 0;
          }
        }
      }
    }
  return -1;
}




