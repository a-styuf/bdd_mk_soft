/*
библиотека для МКО/МПИ 1986ВЕ8Т
Реализовано:
 - КУ 2, 4, 5, 8;
 - отключение работы передатчика;
 - выставление АМКО;
 - установка сигнала занято.
*/
#include <string.h>
#include "mpi.h"

type_MPI_model mpi;

/**
  * @brief  инициализация програмной модели МПИ-ядра
  * @param  mpi_addr адрес ОУ МПИ (0 - отключит устройство)
  */
void mpi_init(uint8_t mpi_addr)
{
	CLK_CNTR->KEY = _KEY_;
	CLK_CNTR->PER1_CLK |= (1<<19);  //enable clock for MIL0
	CLK_CNTR->PER0_CLK |= (1<<13);  //clock for PORTA
	//
	MIL_STD_15531->CONTROL = 1;  //reset
	MIL_STD_15531->CONTROL = (1<<20)|(40<<11)|(3<<4)|(2<<2)|((mpi_addr & 0x1F)<<6);
	MIL_STD_15531->StatusWord1 = ((mpi_addr & 0x1F)<<11);
	//
	PORTA->KEY = _KEY_;
	PORTA->SANALOG =   0xFFFFFFFF;
	PORTA->CFUNC[2]  = 0xF0FF0000;
	PORTA->SFUNC[2] =  0xC0CC0000;
	PORTA->CFUNC[3]  = 0x00FF0FFF;
	PORTA->SFUNC[3] =  0x00CC0CCC;
	PORTA->SPWR[1] =   0x003C0F00;
	PORTA->SPULLDOWN = 0xFFFFFFFF;
	//
	/*enable interrupt*/
	MIL_STD_15531->INTEN = 0x0004;  //valmess int
	NVIC_EnableIRQ(IRQn_MIL_STD_15531);
	//
	mpi.addr = mpi_addr;
	mpi.error_cnt = 0;
	mpi.error = 0;
	mpi.aw = (mpi.addr & 0x1F)<<11;
	if (mpi_addr == 0){ //отключаем МКО
		MIL_STD_15531->CONTROL = 1;  //reset
	}
}

/**
  * @brief  обработка приема данных по результатам прерывания от ядра МПИ
  * @param  error ошибка ОУ МПИ
  * @param  error_cnt кол-во ошибок МПИ
  */
uint16_t mpi_process(void) 
{
	uint8_t i=0;
	uint16_t ret;
	//
	NVIC_DisableIRQ(IRQn_MIL_STD_15531);  
	//
	ret = mpi.ivect;
	if (ret != 0) {
		mpi.addr = ((ret >> 11) & 0x1F);
		mpi.r_w = ((ret >> 10) & 0x01);
		//mko_cw.subaddr = ((ret >> 5) & 0x1F);
		//mko_cw.leng = ret & 0x1F;
		if (mpi.r_w == 0x00) { //работаем только с приемом данных
			for (i=0; i<32; i++) {
					mpi.data[i] = (uint16_t)MIL_STD_15531->DATA[mpi.subaddr*32 + i];
			}
		}
		else {
			ret = 0;
		}
	}
	mpi.ivect = 0;
	//
	NVIC_EnableIRQ(IRQn_MIL_STD_15531);  
	//
	return ret;
}

/**
  * @brief  запрос ошибкок и счетчика ошибок ОУ МПИ
  * @param  error ошибка ОУ МПИ
  * @param  error_cnt кол-во ошибок МПИ
  */
void mpi_get_error(uint8_t* error, uint8_t* error_cnt) 
{
	*error = mpi.error;
	*error_cnt = mpi.error_cnt;
}

/**
  * @brief  устновка бита занятости ОУ МПИ в 1
  */
void mpi_set_busy(void)
{
	mpi.aw |= (0x01 << 3);
	MIL_STD_15531->StatusWord1 |= mpi.aw;
}

/**
  * @brief  устновка бита занятости ОУ МПИ в 0
  */
void mpi_release_busy(void)
{
	mpi.aw &= ~(0x01 << 3);
	MIL_STD_15531->StatusWord1 = mpi.aw;
}

/**
  * @brief  установка 7-го бита ответного слово (для обозначения отсутсвия новых данных при чтении)
  * @param  val значение бита (0 или 1)
  */
void mpi_set_aw_bit_7(uint8_t val)
{
	if (val & 0x01) mpi.aw |= (0x01 <<7);
	else mpi.aw &= ~(0x01 <<7);
	MIL_STD_15531->StatusWord1 |= mpi.aw;
}

/**
  * @brief  запись данных на выбранный подадрес ОУ МПИ
  * @param  subaddr номер подадреса (1-31)
  * @param  data указатель на  данные (3 х 16-бит)
  * @retval статус записи : 1 - успешно, -1 - недопустимы подарес
  */
int8_t mpi_wr_to_subaddr(uint8_t subaddr, uint16_t* data)
{
	uint8_t i;
	if ((subaddr >= 1) && (subaddr <= 31)){
		mpi_set_busy();
		for (i=0; i<32; i++)
		{
				MIL_STD_15531->DATA[subaddr*32 + i] = (uint32_t)data[i];
		}
		mpi_release_busy();
		return 1;
	}
	else{
		return -1;
	}
}

/**
  * @brief  чтение данных из выбранного подадреса ОУ МПИ
  * @param  subaddr номер подадреса (1-31)
  * @param  data указатель на  данные (3 х 16-бит)
  * @retval статус записи : 1 - успешно, -1 - недопустимы подарес
  */
int8_t mpi_rd_from_subaddr(uint8_t subaddr, uint16_t* data)
{
	uint8_t i;
	if ((subaddr >= 1) && (subaddr <= 31)){
		for (i=0; i<32; i++)
		{
				data[i] = MIL_STD_15531->DATA[subaddr*32 + i] & 0xFFFF;
		}
		return 1;
	}
	else{
		return -1;
	}
}

/**
  * @brief  отключение ОУ МПИ
  */
void mpi_block_transmitter(void)
{
	MIL_STD_15531->CONTROL = 1;  //reset
}

/**
  * @brief обработка прерывания от ядра МПИ
  */
void INT_MIL0_Handler(void) 
{
	mpi.cw = MIL_STD_15531->CommandWord1;
	mpi.msg = MIL_STD_15531->MSG;
	mpi.leng = mpi.cw & 0x1F;
	mpi.rcv_a = ((MIL_STD_15531->STATUS >> 9) & 0x01);
	mpi.rcv_b = ((MIL_STD_15531->STATUS >> 10) & 0x01);
	MIL_STD_15531->STATUS  &= ~(3 << 9); // обнуляем флаг активности каналов МКО
	mpi_amko_callback();
	mpi.error = (MIL_STD_15531->ERROR & 0xFFFF);
	if (MIL_STD_15531->ERROR == 0) {
		mpi.ivect = (uint16_t)mpi.cw;
		if (mpi.msg == 0x0410) { // обработка служебных команд по ГОСТУ Р 52070-2003
			switch (mpi.leng){
				case 2: //передать ответное слово
					// ничего не делаем, ответное слово передается ядром
					break;
				case 4: //блокировать передатчик
					if (mpi.rcv_a){
						MIL_STD_15531->CONTROL &= ~(1 << 5); //блокируем передатчик Б
					}
					else if(mpi.rcv_b){
						MIL_STD_15531->CONTROL &= ~(1 << 4); //блокируем передатчик А
					}
					break;
				case 5: //разблокировать передатчик
					if (mpi.rcv_a){
						MIL_STD_15531->CONTROL |= (1 << 5); //разблокируем передатчик Б
					}
					else if(mpi.rcv_b){
						MIL_STD_15531->CONTROL |= (1 << 4); //разблокируем передатчик А
					}
					break;
				case 8: //разблокировать передатчик
					MIL_STD_15531->CONTROL |= 1;  //reset
					MIL_STD_15531->CONTROL &= ~1; //clear reset
					break;
			}
		}
		else	{ // обработка приема данных ОУ МПИ
			if (((mpi.cw >> 10) & 0x01) != 0) return;  //работаем только с приемом
			mpi.subaddr = ((mpi.cw >> 5) & 0x1F);
			if ((mpi.subaddr >= 1) && (mpi.subaddr <= 31)) {
				if (mpi.leng == 0) mpi.leng = 32;
				memcpy((char*)&MIL_STD_15531->DATA[mpi.subaddr*32], (char*)&MIL_STD_15531->DATA[mpi.subaddr*32], mpi.leng << 2);
			}
		}
	}
	else{
		mpi.error_cnt ++;
	}
}

/**
  * @brief вызов события по прием или отправке какого-либо пакета ОУ МПИ
  */
__weak void mpi_amko_callback(void)
{
	//
}
