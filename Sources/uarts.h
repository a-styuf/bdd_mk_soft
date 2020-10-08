#ifndef _UARTS_H_
#define _UARTS_H_

#include <stdint.h>

void UART0_Init(void);
void UART0_SendPacket(uint8_t *buff, uint8_t leng, uint8_t en_crc_flg);
int8_t UART0_PacketInWaitingOrReady(void);
int8_t UART0_GetPacket(uint8_t *buff, uint8_t *leng);

#define UART_CRC_ENA (0x01)
#define UART_CRC_DIS (0x00)

#endif

