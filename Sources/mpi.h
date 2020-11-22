#ifndef _MPI_H_
#define _MPI_H_

#include "1986ve8_lib/cm4ikmcu.h"

#define IRQn_MIL_STD_15531 (IRQn_Type)117
#define MPI_ADDRES_DEFAULT 13

#define MPI_COMMAND_SUBADDR 30

#pragma pack(2)
typedef struct  //  max 62 - параетры ЦМ для сохоранения
{
	uint16_t cw; //+0
	uint16_t aw; //+2
	uint8_t addr; //+4
	uint8_t r_w; //+5
	uint8_t subaddr; //+6
	uint8_t leng; //+7
	uint16_t msg; //+8
	uint8_t num; //+10
	uint8_t gap; //+11
	uint8_t rcv_a;  //+12 //проверка номера приемника: А или Б
	uint8_t rcv_b;  //+13 //проверка номера приемника: А или Б
  uint16_t ivect; //+14
	uint8_t error; //+16
	uint8_t error_cnt; //+17
  uint16_t data[32]; //+18
}type_MPI_model;


void mpi_init(uint8_t mpi_addr);
uint16_t mpi_process(void);
void mpi_get_error(uint8_t* error, uint8_t* error_cnt);
void mpi_set_busy(void);
void mpi_release_busy(void);
void mpi_set_aw_bit_7(uint8_t val);
int8_t mpi_wr_to_subaddr(uint8_t subaddr, uint16_t* data);
int8_t mpi_rd_from_subaddr(uint8_t subaddr, uint16_t* data);
void mpi_block_transmitter(void);
__weak void mpi_amko_callback(void);


#endif

