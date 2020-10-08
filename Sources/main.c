#include "1986ve8_lib/cm4ikmcu.h"
#include <string.h>
#include <stdio.h>
#include "sysinit.h"
#include "wdt.h"
#include "uarts.h"
#include "debug.h"

int main() {
	// инициализация переферии
	System_Init();
	WDT_Init();
	UART0_Init();
	
	dbg_print("BDD_MK: Hi, user!");
	while(1) {
		WDRST;

	}
}

