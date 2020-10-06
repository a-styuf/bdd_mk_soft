/*
 *-----------------------------------------------------------------------------
 * The confidential and proprietary information contained in this file may
 * only be used by a person authorised under and to the extent permitted
 * by a subsisting licensing agreement from ARM Limited.
 *
 *            (C) COPYRIGHT 2009-2010 ARM Limited.
 *                ALL RIGHTS RESERVED
 *
 * This entire notice must be reproduced on all copies of this file
 * and copies of this file may only be made by a person if such person is
 * permitted to do so under the terms of a subsisting license agreement
 * from ARM Limited.
 *
 *      SVN Information
 *
 *
 *      Revision            : $Revision: 142008 $
 *
 *      Release information : cortexm4_r0p1_00rel0
 *-----------------------------------------------------------------------------
 */

#ifndef __CM4IKMCU_H__
#define __CM4IKMCU_H__

/*
 * ==========================================================================
 * ---------- Interrupt Number Definition -----------------------------------
 * ==========================================================================
 */

typedef enum IRQn
{
/******  Cortex-M4 Processor Exceptions Numbers ***************************************************/
  NonMaskableInt_IRQn         = -14,    /*!< 2 Non Maskable Interrupt                             */
  HardFault_IRQn              = -13,    /*!< 3 Cortex-M4 Hard Fault Interrupt                     */
  SVCall_IRQn                 = -5,     /*!< 11 Cortex-M4 SV Call Interrupt                       */
  PendSV_IRQn                 = -2,     /*!< 14 Cortex-M4 Pend SV Interrupt                       */
  SysTick_IRQn                = -1,     /*!< 15 Cortex-M4 System Tick Interrupt                   */

/******  CM4IKMCU Cortex-M4 specific Interrupt Numbers ********************************************/
  GPIO_IRQn                   = 0,      /*!< GPIO Interrupt                                       */
                                        /*!< maximum of 32 Interrupts are possible                */
  CM4IKMCU_IRQ01_IRQn         = 1,
  CM4IKMCU_IRQ02_IRQn         = 2,
  CM4IKMCU_IRQ03_IRQn         = 3,
  CM4IKMCU_IRQ04_IRQn         = 4,
  CM4IKMCU_IRQ05_IRQn         = 5,
  CM4IKMCU_IRQ06_IRQn         = 6,
  CM4IKMCU_IRQ07_IRQn         = 7,
  CM4IKMCU_IRQ08_IRQn         = 8,
  CM4IKMCU_IRQ09_IRQn         = 9,
  CM4IKMCU_IRQ10_IRQn         = 10,
  CM4IKMCU_IRQ11_IRQn         = 11,
  CM4IKMCU_IRQ12_IRQn         = 12,
  CM4IKMCU_IRQ13_IRQn         = 13,
  CM4IKMCU_IRQ14_IRQn         = 14,
  CM4IKMCU_IRQ15_IRQn         = 15,
  CM4IKMCU_IRQ16_IRQn         = 16,
  CM4IKMCU_IRQ17_IRQn         = 17,
  CM4IKMCU_IRQ18_IRQn         = 18,
  CM4IKMCU_IRQ19_IRQn         = 19,
  CM4IKMCU_IRQ20_IRQn         = 20,
  CM4IKMCU_IRQ21_IRQn         = 21,
  CM4IKMCU_IRQ22_IRQn         = 22,
  CM4IKMCU_IRQ23_IRQn         = 23,
  CM4IKMCU_IRQ24_IRQn         = 24,
  CM4IKMCU_IRQ25_IRQn         = 25,
  CM4IKMCU_IRQ26_IRQn         = 26,
  CM4IKMCU_IRQ27_IRQn         = 27,
  CM4IKMCU_IRQ28_IRQn         = 28,
  CM4IKMCU_IRQ29_IRQn         = 29,
  CM4IKMCU_IRQ30_IRQn         = 30,
  CM4IKMCU_IRQ31_IRQn         = 31
} IRQn_Type;


/*
 * ==========================================================================
 * ----------- Processor and Core Peripheral Section ------------------------
 * ==========================================================================
 */

/* Configuration of the Cortex-M4 Processor and Core Peripherals */
#define __MPU_PRESENT           1       /*!< cm4ikmcu does not provide a MPU present or not       */
#define __NVIC_PRIO_BITS        2       /*!< cm4ikmcu Supports 2 Bits for the Priority Levels     */
#define __Vendor_SysTickConfig  0       /*!< Set to 1 if different SysTick Config is used         */

#define _KEY_ 0x8555AAA1

#include "core_cm4.h"                   /* Cortex-M4 processor and core peripherals               */
//#include "system_cm4ikmcu.h"            /* cm4ikmcu System                                        */



/**
 * Initialize the system clock
 *
 * @param  none
 * @return none
 *
 * @brief  Setup the microcontroller system
 *         Initialize the PLL and update the SystemFrequency variable
 */
extern void SystemInit (void);


/******************************************************************************/
/*                Device Specific Peripheral registers structures             */
/******************************************************************************/

/*--------------------- General Purpose Input and Ouptut ---------------------*/
typedef union
{
  __IO uint32_t WORD;
  __IO uint16_t HALFWORD[2];
  __IO uint8_t  BYTE[4];
} GPIO_Data_TypeDef;

typedef struct
{
  GPIO_Data_TypeDef DATA [256];
  GPIO_Data_TypeDef DIR;
  uint32_t RESERVED[3];
  GPIO_Data_TypeDef IE;
} GPIO_TypeDef;


/***
*  DMA control
**/
typedef struct
{
  __IO  uint32_t STATUS;
  __IO  uint32_t CONFIG;
  __IO  uint32_t CTRL_BASE_PTR;
  __IO  uint32_t ALT_CTRL_BASE_PTR;
  __IO  uint32_t WAITONREG_STATUS;
  __IO  uint32_t CHNL_SW_REQUEST;
  __IO  uint32_t CHNL_USEBURST_SET;
  __IO  uint32_t CHNL_USEBURST_CLR;
  __IO  uint32_t CHNL_REQ_MASK_SET;
  __IO  uint32_t CHNL_REQ_MASK_CLR;
  __IO  uint32_t CHNL_ENABLE_SET;
  __IO  uint32_t CHNL_ENABLE_CLR;
  __IO  uint32_t CHNL_PRI_ALT_SET;
  __IO  uint32_t CHNL_PRI_ALT_CLR;
  __IO  uint32_t CHNL_PRIORITY_SET;
  __IO  uint32_t CHNL_PRIORITY_CLR;
  uint32_t RESERVED[3];
  __IO  uint32_t ERR_CLR;
  __IO  uint32_t CHMUX0;
  __IO  uint32_t CHMUX1;
  __IO  uint32_t CHMUX2;
  __IO  uint32_t CHMUX3;
  __IO  uint32_t CHMUX4;
  __IO  uint32_t CHMUX5;
  __IO  uint32_t CHMUX6;
  __IO  uint32_t CHMUX7;
} DMAControl;

#define DMA_CNTR_BASE     (( uint32_t) 0xe0042000)

#define DMA       ((DMAControl *) DMA_CNTR_BASE)

/***
*  DMA Primary data structure
**/

typedef struct
{
  __IO  uint32_t PRIMARY_CH0_SOURCE_END_POINTER;
  __IO  uint32_t PRIMARY_CH0_DEST_END_POINTER;
  __IO  uint32_t PRIMARY_CH0_CONTROL;
  __IO  uint32_t PRIMARY_CH0_UNUSED;
 
  __IO  uint32_t PRIMARY_CH1_SOURCE_END_POINTER;
  __IO  uint32_t PRIMARY_CH1_DEST_END_POINTER;
  __IO  uint32_t PRIMARY_CH1_CONTROL;
  __IO  uint32_t PRIMARY_CH1_UNUSED;

  __IO  uint32_t PRIMARY_CH2_SOURCE_END_POINTER;
  __IO  uint32_t PRIMARY_CH2_DEST_END_POINTER;
  __IO  uint32_t PRIMARY_CH2_CONTROL;
  __IO  uint32_t PRIMARY_CH2_UNUSED;
 
  __IO  uint32_t PRIMARY_CH3_SOURCE_END_POINTER;
  __IO  uint32_t PRIMARY_CH3_DEST_END_POINTER;
  __IO  uint32_t PRIMARY_CH3_CONTROL;
  __IO  uint32_t PRIMARY_CH3_UNUSED;

  __IO  uint32_t PRIMARY_CH4_SOURCE_END_POINTER;
  __IO  uint32_t PRIMARY_CH4_DEST_END_POINTER;
  __IO  uint32_t PRIMARY_CH4_CONTROL;
  __IO  uint32_t PRIMARY_CH4_UNUSED;
 
  __IO  uint32_t PRIMARY_CH5_SOURCE_END_POINTER;
  __IO  uint32_t PRIMARY_CH5_DEST_END_POINTER;
  __IO  uint32_t PRIMARY_CH5_CONTROL;
  __IO  uint32_t PRIMARY_CH5_UNUSED;

  __IO  uint32_t PRIMARY_CH6_SOURCE_END_POINTER;
  __IO  uint32_t PRIMARY_CH6_DEST_END_POINTER;
  __IO  uint32_t PRIMARY_CH6_CONTROL;
  __IO  uint32_t PRIMARY_CH6_UNUSED;
 
  __IO  uint32_t PRIMARY_CH7_SOURCE_END_POINTER;
  __IO  uint32_t PRIMARY_CH7_DEST_END_POINTER;
  __IO  uint32_t PRIMARY_CH7_CONTROL;
  __IO  uint32_t PRIMARY_CH7_UNUSED;

  __IO  uint32_t PRIMARY_CH8_SOURCE_END_POINTER;
  __IO  uint32_t PRIMARY_CH8_DEST_END_POINTER;
  __IO  uint32_t PRIMARY_CH8_CONTROL;
  __IO  uint32_t PRIMARY_CH8_UNUSED;
 
  __IO  uint32_t PRIMARY_CH9_SOURCE_END_POINTER;
  __IO  uint32_t PRIMARY_CH9_DEST_END_POINTER;
  __IO  uint32_t PRIMARY_CH9_CONTROL;
  __IO  uint32_t PRIMARY_CH9_UNUSED;

  __IO  uint32_t PRIMARY_CH10_SOURCE_END_POINTER;
  __IO  uint32_t PRIMARY_CH10_DEST_END_POINTER;
  __IO  uint32_t PRIMARY_CH10_CONTROL;
  __IO  uint32_t PRIMARY_CH10_UNUSED;
 
  __IO  uint32_t PRIMARY_CH11_SOURCE_END_POINTER;
  __IO  uint32_t PRIMARY_CH11_DEST_END_POINTER;
  __IO  uint32_t PRIMARY_CH11_CONTROL;
  __IO  uint32_t PRIMARY_CH11_UNUSED;

  __IO  uint32_t PRIMARY_CH12_SOURCE_END_POINTER;
  __IO  uint32_t PRIMARY_CH12_DEST_END_POINTER;
  __IO  uint32_t PRIMARY_CH12_CONTROL;
  __IO  uint32_t PRIMARY_CH12_UNUSED;
 
  __IO  uint32_t PRIMARY_CH13_SOURCE_END_POINTER;
  __IO  uint32_t PRIMARY_CH13_DEST_END_POINTER;
  __IO  uint32_t PRIMARY_CH13_CONTROL;
  __IO  uint32_t PRIMARY_CH13_UNUSED;

  __IO  uint32_t PRIMARY_CH14_SOURCE_END_POINTER;
  __IO  uint32_t PRIMARY_CH14_DEST_END_POINTER;
  __IO  uint32_t PRIMARY_CH14_CONTROL;
  __IO  uint32_t PRIMARY_CH14_UNUSED;
 
  __IO  uint32_t PRIMARY_CH15_SOURCE_END_POINTER;
  __IO  uint32_t PRIMARY_CH15_DEST_END_POINTER;
  __IO  uint32_t PRIMARY_CH15_CONTROL;
  __IO  uint32_t PRIMARY_CH15_UNUSED;

  __IO  uint32_t PRIMARY_CH16_SOURCE_END_POINTER;
  __IO  uint32_t PRIMARY_CH16_DEST_END_POINTER;
  __IO  uint32_t PRIMARY_CH16_CONTROL;
  __IO  uint32_t PRIMARY_CH16_UNUSED;
 
  __IO  uint32_t PRIMARY_CH17_SOURCE_END_POINTER;
  __IO  uint32_t PRIMARY_CH17_DEST_END_POINTER;
  __IO  uint32_t PRIMARY_CH17_CONTROL;
  __IO  uint32_t PRIMARY_CH17_UNUSED;

  __IO  uint32_t PRIMARY_CH18_SOURCE_END_POINTER;
  __IO  uint32_t PRIMARY_CH18_DEST_END_POINTER;
  __IO  uint32_t PRIMARY_CH18_CONTROL;
  __IO  uint32_t PRIMARY_CH18_UNUSED;
 
  __IO  uint32_t PRIMARY_CH19_SOURCE_END_POINTER;
  __IO  uint32_t PRIMARY_CH19_DEST_END_POINTER;
  __IO  uint32_t PRIMARY_CH19_CONTROL;
  __IO  uint32_t PRIMARY_CH19_UNUSED;

  __IO  uint32_t PRIMARY_CH20_SOURCE_END_POINTER;
  __IO  uint32_t PRIMARY_CH20_DEST_END_POINTER;
  __IO  uint32_t PRIMARY_CH20_CONTROL;
  __IO  uint32_t PRIMARY_CH20_UNUSED;
 
  __IO  uint32_t PRIMARY_CH21_SOURCE_END_POINTER;
  __IO  uint32_t PRIMARY_CH21_DEST_END_POINTER;
  __IO  uint32_t PRIMARY_CH21_CONTROL;
  __IO  uint32_t PRIMARY_CH21_UNUSED;

  __IO  uint32_t PRIMARY_CH22_SOURCE_END_POINTER;
  __IO  uint32_t PRIMARY_CH22_DEST_END_POINTER;
  __IO  uint32_t PRIMARY_CH22_CONTROL;
  __IO  uint32_t PRIMARY_CH22_UNUSED;
 
  __IO  uint32_t PRIMARY_CH23_SOURCE_END_POINTER;
  __IO  uint32_t PRIMARY_CH23_DEST_END_POINTER;
  __IO  uint32_t PRIMARY_CH23_CONTROL;
  __IO  uint32_t PRIMARY_CH23_UNUSED;

  __IO  uint32_t PRIMARY_CH24_SOURCE_END_POINTER;
  __IO  uint32_t PRIMARY_CH24_DEST_END_POINTER;
  __IO  uint32_t PRIMARY_CH24_CONTROL;
  __IO  uint32_t PRIMARY_CH24_UNUSED;
 
  __IO  uint32_t PRIMARY_CH25_SOURCE_END_POINTER;
  __IO  uint32_t PRIMARY_CH25_DEST_END_POINTER;
  __IO  uint32_t PRIMARY_CH25_CONTROL;
  __IO  uint32_t PRIMARY_CH25_UNUSED;

  __IO  uint32_t PRIMARY_CH26_SOURCE_END_POINTER;
  __IO  uint32_t PRIMARY_CH26_DEST_END_POINTER;
  __IO  uint32_t PRIMARY_CH26_CONTROL;
  __IO  uint32_t PRIMARY_CH26_UNUSED;
 
  __IO  uint32_t PRIMARY_CH27_SOURCE_END_POINTER;
  __IO  uint32_t PRIMARY_CH27_DEST_END_POINTER;
  __IO  uint32_t PRIMARY_CH27_CONTROL;
  __IO  uint32_t PRIMARY_CH27_UNUSED;

  __IO  uint32_t PRIMARY_CH28_SOURCE_END_POINTER;
  __IO  uint32_t PRIMARY_CH28_DEST_END_POINTER;
  __IO  uint32_t PRIMARY_CH28_CONTROL;
  __IO  uint32_t PRIMARY_CH28_UNUSED;
 
  __IO  uint32_t PRIMARY_CH29_SOURCE_END_POINTER;
  __IO  uint32_t PRIMARY_CH29_DEST_END_POINTER;
  __IO  uint32_t PRIMARY_CH29_CONTROL;
  __IO  uint32_t PRIMARY_CH29_UNUSED;

  __IO  uint32_t PRIMARY_CH30_SOURCE_END_POINTER;
  __IO  uint32_t PRIMARY_CH30_DEST_END_POINTER;
  __IO  uint32_t PRIMARY_CH30_CONTROL;
  __IO  uint32_t PRIMARY_CH30_UNUSED;
 
  __IO  uint32_t PRIMARY_CH31_SOURCE_END_POINTER;
  __IO  uint32_t PRIMARY_CH31_DEST_END_POINTER;
  __IO  uint32_t PRIMARY_CH31_CONTROL;
  __IO  uint32_t PRIMARY_CH31_UNUSED;
} DMA_PrimaryData_TypeDef;

/***
*  DMA Alternate data structure
**/

typedef struct
{
  __IO  uint32_t ALTER_CH0_SOURCE_END_POINTER;
  __IO  uint32_t ALTER_CH0_DEST_END_POINTER;
  __IO  uint32_t ALTER_CH0_CONTROL;
  __IO  uint32_t ALTER_CH0_UNUSED;
 
  __IO  uint32_t ALTER_CH1_SOURCE_END_POINTER;
  __IO  uint32_t ALTER_CH1_DEST_END_POINTER;
  __IO  uint32_t ALTER_CH1_CONTROL;
  __IO  uint32_t ALTER_CH1_UNUSED;

  __IO  uint32_t ALTER_CH2_SOURCE_END_POINTER;
  __IO  uint32_t ALTER_CH2_DEST_END_POINTER;
  __IO  uint32_t ALTER_CH2_CONTROL;
  __IO  uint32_t ALTER_CH2_UNUSED;
 
  __IO  uint32_t ALTER_CH3_SOURCE_END_POINTER;
  __IO  uint32_t ALTER_CH3_DEST_END_POINTER;
  __IO  uint32_t ALTER_CH3_CONTROL;
  __IO  uint32_t ALTER_CH3_UNUSED;

  __IO  uint32_t ALTER_CH4_SOURCE_END_POINTER;
  __IO  uint32_t ALTER_CH4_DEST_END_POINTER;
  __IO  uint32_t ALTER_CH4_CONTROL;
  __IO  uint32_t ALTER_CH4_UNUSED;
 
  __IO  uint32_t ALTER_CH5_SOURCE_END_POINTER;
  __IO  uint32_t ALTER_CH5_DEST_END_POINTER;
  __IO  uint32_t ALTER_CH5_CONTROL;
  __IO  uint32_t ALTER_CH5_UNUSED;

  __IO  uint32_t ALTER_CH6_SOURCE_END_POINTER;
  __IO  uint32_t ALTER_CH6_DEST_END_POINTER;
  __IO  uint32_t ALTER_CH6_CONTROL;
  __IO  uint32_t ALTER_CH6_UNUSED;
 
  __IO  uint32_t ALTER_CH7_SOURCE_END_POINTER;
  __IO  uint32_t ALTER_CH7_DEST_END_POINTER;
  __IO  uint32_t ALTER_CH7_CONTROL;
  __IO  uint32_t ALTER_CH7_UNUSED;

  __IO  uint32_t ALTER_CH8_SOURCE_END_POINTER;
  __IO  uint32_t ALTER_CH8_DEST_END_POINTER;
  __IO  uint32_t ALTER_CH8_CONTROL;
  __IO  uint32_t ALTER_CH8_UNUSED;
 
  __IO  uint32_t ALTER_CH9_SOURCE_END_POINTER;
  __IO  uint32_t ALTER_CH9_DEST_END_POINTER;
  __IO  uint32_t ALTER_CH9_CONTROL;
  __IO  uint32_t ALTER_CH9_UNUSED;

  __IO  uint32_t ALTER_CH10_SOURCE_END_POINTER;
  __IO  uint32_t ALTER_CH10_DEST_END_POINTER;
  __IO  uint32_t ALTER_CH10_CONTROL;
  __IO  uint32_t ALTER_CH10_UNUSED;
 
  __IO  uint32_t ALTER_CH11_SOURCE_END_POINTER;
  __IO  uint32_t ALTER_CH11_DEST_END_POINTER;
  __IO  uint32_t ALTER_CH11_CONTROL;
  __IO  uint32_t ALTER_CH11_UNUSED;

  __IO  uint32_t ALTER_CH12_SOURCE_END_POINTER;
  __IO  uint32_t ALTER_CH12_DEST_END_POINTER;
  __IO  uint32_t ALTER_CH12_CONTROL;
  __IO  uint32_t ALTER_CH12_UNUSED;
 
  __IO  uint32_t ALTER_CH13_SOURCE_END_POINTER;
  __IO  uint32_t ALTER_CH13_DEST_END_POINTER;
  __IO  uint32_t ALTER_CH13_CONTROL;
  __IO  uint32_t ALTER_CH13_UNUSED;

  __IO  uint32_t ALTER_CH14_SOURCE_END_POINTER;
  __IO  uint32_t ALTER_CH14_DEST_END_POINTER;
  __IO  uint32_t ALTER_CH14_CONTROL;
  __IO  uint32_t ALTER_CH14_UNUSED;
 
  __IO  uint32_t ALTER_CH15_SOURCE_END_POINTER;
  __IO  uint32_t ALTER_CH15_DEST_END_POINTER;
  __IO  uint32_t ALTER_CH15_CONTROL;
  __IO  uint32_t ALTER_CH15_UNUSED;

  __IO  uint32_t ALTER_CH16_SOURCE_END_POINTER;
  __IO  uint32_t ALTER_CH16_DEST_END_POINTER;
  __IO  uint32_t ALTER_CH16_CONTROL;
  __IO  uint32_t ALTER_CH16_UNUSED;
 
  __IO  uint32_t ALTER_CH17_SOURCE_END_POINTER;
  __IO  uint32_t ALTER_CH17_DEST_END_POINTER;
  __IO  uint32_t ALTER_CH17_CONTROL;
  __IO  uint32_t ALTER_CH17_UNUSED;

  __IO  uint32_t ALTER_CH18_SOURCE_END_POINTER;
  __IO  uint32_t ALTER_CH18_DEST_END_POINTER;
  __IO  uint32_t ALTER_CH18_CONTROL;
  __IO  uint32_t ALTER_CH18_UNUSED;
 
  __IO  uint32_t ALTER_CH19_SOURCE_END_POINTER;
  __IO  uint32_t ALTER_CH19_DEST_END_POINTER;
  __IO  uint32_t ALTER_CH19_CONTROL;
  __IO  uint32_t ALTER_CH19_UNUSED;

  __IO  uint32_t ALTER_CH20_SOURCE_END_POINTER;
  __IO  uint32_t ALTER_CH20_DEST_END_POINTER;
  __IO  uint32_t ALTER_CH20_CONTROL;
  __IO  uint32_t ALTER_CH20_UNUSED;
 
  __IO  uint32_t ALTER_CH21_SOURCE_END_POINTER;
  __IO  uint32_t ALTER_CH21_DEST_END_POINTER;
  __IO  uint32_t ALTER_CH21_CONTROL;
  __IO  uint32_t ALTER_CH21_UNUSED;

  __IO  uint32_t ALTER_CH22_SOURCE_END_POINTER;
  __IO  uint32_t ALTER_CH22_DEST_END_POINTER;
  __IO  uint32_t ALTER_CH22_CONTROL;
  __IO  uint32_t ALTER_CH22_UNUSED;
 
  __IO  uint32_t ALTER_CH23_SOURCE_END_POINTER;
  __IO  uint32_t ALTER_CH23_DEST_END_POINTER;
  __IO  uint32_t ALTER_CH23_CONTROL;
  __IO  uint32_t ALTER_CH23_UNUSED;

  __IO  uint32_t ALTER_CH24_SOURCE_END_POINTER;
  __IO  uint32_t ALTER_CH24_DEST_END_POINTER;
  __IO  uint32_t ALTER_CH24_CONTROL;
  __IO  uint32_t ALTER_CH24_UNUSED;
 
  __IO  uint32_t ALTER_CH25_SOURCE_END_POINTER;
  __IO  uint32_t ALTER_CH25_DEST_END_POINTER;
  __IO  uint32_t ALTER_CH25_CONTROL;
  __IO  uint32_t ALTER_CH25_UNUSED;

  __IO  uint32_t ALTER_CH26_SOURCE_END_POINTER;
  __IO  uint32_t ALTER_CH26_DEST_END_POINTER;
  __IO  uint32_t ALTER_CH26_CONTROL;
  __IO  uint32_t ALTER_CH26_UNUSED;
 
  __IO  uint32_t ALTER_CH27_SOURCE_END_POINTER;
  __IO  uint32_t ALTER_CH27_DEST_END_POINTER;
  __IO  uint32_t ALTER_CH27_CONTROL;
  __IO  uint32_t ALTER_CH27_UNUSED;

  __IO  uint32_t ALTER_CH28_SOURCE_END_POINTER;
  __IO  uint32_t ALTER_CH28_DEST_END_POINTER;
  __IO  uint32_t ALTER_CH28_CONTROL;
  __IO  uint32_t ALTER_CH28_UNUSED;
 
  __IO  uint32_t ALTER_CH29_SOURCE_END_POINTER;
  __IO  uint32_t ALTER_CH29_DEST_END_POINTER;
  __IO  uint32_t ALTER_CH29_CONTROL;
  __IO  uint32_t ALTER_CH29_UNUSED;

  __IO  uint32_t ALTER_CH30_SOURCE_END_POINTER;
  __IO  uint32_t ALTER_CH30_DEST_END_POINTER;
  __IO  uint32_t ALTER_CH30_CONTROL;
  __IO  uint32_t ALTER_CH30_UNUSED;
 
  __IO  uint32_t ALTER_CH31_SOURCE_END_POINTER;
  __IO  uint32_t ALTER_CH31_DEST_END_POINTER;
  __IO  uint32_t ALTER_CH31_CONTROL;
  __IO  uint32_t ALTER_CH31_UNUSED;
} DMA_AlterData_TypeDef;
/***

*  Cache control
**/
typedef struct
{
  __IO  uint32_t KEY;
  __IO  uint32_t CNTL;
  __I uint32_t HIT_CNT;
  __I uint32_t MISS_CNT;
  uint32_t RESERVED[4];
  __IO  uint32_t ECC_CNTL;
  __IO  uint32_t ECC_ADDR;
  __IO  uint32_t ECC_DATA;
  __IO  uint32_t ECC_ECC;
} CacheControl;

#define ICACHE_BASE     (( uint32_t) 0xe0044000)
#define DCACHE_BASE     (( uint32_t) 0xe0045000)

#define ICACHE        ((CacheControl *) ICACHE_BASE)
#define DCACHE        ((CacheControl *) DCACHE_BASE)

#define CC_CACHE_ENABLE ((uint32_t) 0x1)
#define CC_CACHE_CLR  ((uint32_t) (1<<1))
#define CC_OTP_ENABLE ((uint32_t) (1<<2))
#define CC_EXTB_ENABLE  ((uint32_t) (1<<3))
#define CC_HIT_CLR  ((uint32_t) (1<<4))
#define CC_MISS_CLR ((uint32_t) (1<<5))

#define CC_KEY      ((uint32_t) 0x8555aaa1)

/***
*  SCRABER control
**/
typedef struct
{
  __IO  uint32_t CNTRL;
  __IO  uint32_t S_ADDR;
  __IO  uint32_t F_ADDR;
  __IO  uint32_t DATA;
  __IO  uint32_t SECC;
} SCRControl;

#define SCR_CNTR_BASE     (( uint32_t) 0xe0043000)

#define SCR_CNTR      ((SCRControl *) SCR_CNTR_BASE)

/***
*  FT control
**/
typedef struct
{
  __IO  uint32_t KEY;
  __IO  uint32_t CONTROL;
  __IO  uint32_t STATUS;
  __IO  uint32_t TIMEOUT;
  __IO  uint32_t TICKCNT;
  __IO  uint32_t FIRSTEVENT;
  __IO  uint32_t LASTEVENT;
  __IO  uint32_t TIMEOUTCNT;
  __IO  uint32_t EVENT[13];
  __IO  uint32_t RESET_EVENT0;
  __IO  uint32_t RESET_EVENT1;
  __IO  uint32_t RESET_EVENT2;
  __IO  uint32_t RESET_EVENT3;
  __IO  uint32_t RESET_EVENT4;
  __IO  uint32_t IE_EVENT5;
  __IO  uint32_t IE_EVENT6;
  __IO  uint32_t IE_EVENT7;
  __IO  uint32_t IE_EVENT8;
  __IO  uint32_t IE_EVENT9;
  __IO  uint32_t IE_EVENT10;
  __IO  uint32_t IE_EVENT11;
  __IO  uint32_t IE_EVENT12;
  __IO  uint32_t FT_STAT0;
  __IO  uint32_t FT_STAT1;
} FTControl;

#define FT_CNTR_BASE      (( uint32_t) 0x40003000)

#define FT_CNTR       ((FTControl *) FT_CNTR_BASE)


/***
*  OTP control
**/

typedef struct
{
  __IO  uint32_t KEY;
  __IO  uint32_t CNTR;
  __IO  uint32_t ADR;
  __IO  uint32_t WDATA;
  __IO  uint32_t RDATA;
  __IO  uint32_t ECCCS;
  __IO  uint32_t ECCADR;
  __IO  uint32_t ECCDATA;
  __IO  uint32_t ECCECC;
  __IO  uint32_t TEST_TUNING;
} OTPControl;


#define OTP_CNTR_BASE     (( uint32_t) 0x40006000)

#define OTP_CNTR      ((OTPControl *) OTP_CNTR_BASE)
#define OTP_CNTR_WS     ((uint32_t) 0x7)

/***
*  ROM,RAMC,RAMD control
**/

typedef struct
{
  __IO  uint32_t KEY;
  __IO  uint32_t ECCCS;
  __IO  uint32_t ECCADR;
  __IO  uint32_t ECCDATA;
  __IO  uint32_t ECCECC;
  __IO  uint32_t TEST_TUNING;
} MEMControl;


#define ROM_CNTR_BASE     (( uint32_t) 0x40009000)
#define RAMC_CNTR_BASE      (( uint32_t) 0x40007000)
#define RAMD_CNTR_BASE      (( uint32_t) 0x40008000)

#define ROM_CNTR      ((MEMControl *) ROM_CNTR_BASE)
#define RAMC_CNTR     ((MEMControl *) RAMC_CNTR_BASE)
#define RAMD_CNTR     ((MEMControl *) RAMD_CNTR_BASE)

#define RCACHE_BASE     (( uint32_t) 0x4000b000)
#define RCACHE        ((CacheControl *) RCACHE_BASE)
/***
*  EXTBUS control
**/
typedef struct
{
  __IO  uint32_t KEY;
  __IO  uint32_t RGN0_CNTRL;
  __IO  uint32_t RGN1_CNTRL;
  __IO  uint32_t RGN2_CNTRL;
  __IO  uint32_t RGN3_CNTRL;
  __IO  uint32_t RGN4_CNTRL;
  __IO  uint32_t RGN5_CNTRL;
  __IO  uint32_t RGN6_CNTRL;
  __IO  uint32_t RGN7_CNTRL;
  
  __IO  uint32_t RGN0_ECCBASE;
  __IO  uint32_t RGN1_ECCBASE;
  __IO  uint32_t RGN2_ECCBASE;
  __IO  uint32_t RGN3_ECCBASE;
  __IO  uint32_t RGN4_ECCBASE;
  __IO  uint32_t RGN5_ECCBASE;
  __IO  uint32_t RGN6_ECCBASE;
  __IO  uint32_t RGN7_ECCBASE;
  
  __IO  uint32_t RGN0_ECCS;
  __IO  uint32_t RGN1_ECCS;
  __IO  uint32_t RGN2_ECCS;
  __IO  uint32_t RGN3_ECCS;
  __IO  uint32_t RGN4_ECCS;
  __IO  uint32_t RGN5_ECCS;
  __IO  uint32_t RGN6_ECCS;
  __IO  uint32_t RGN7_ECCS;
  

  __IO  uint32_t SPORT_PWR0;
  __IO  uint32_t SPORT_PWR1;
  __IO  uint32_t SPORT_PWR2;
  __IO  uint32_t SPORT_PWR3;
  
  __IO  uint32_t SPORT_PD0;
  __IO  uint32_t SPORT_PD1;
  __IO  uint32_t SPORT_PD2;
  __IO  uint32_t SPORT_PD3;

  __IO  uint32_t SPORT_CL0;
  __IO  uint32_t SPORT_CL1;
  __IO  uint32_t SPORT_CL2;
  __IO  uint32_t SPORT_CL3;
  
  __IO  uint32_t ECC_ADDR;
  __IO  uint32_t ECC_DATA;
  __IO  uint32_t ECC_ECC;
  
} EXTBUSControl;

#define EXT_BUS_CNTR_BASE     (( uint32_t) 0x40005000)

#define EXT_BUS_CNTR        ((EXTBUSControl *) EXT_BUS_CNTR_BASE)

#define EB_CNTR_MODE(mode)  ((mode & 3) << 4)
#define EB_CNTR_ECCMODE     4
#define EB_CNTR_ECCEN     2
#define EB_CNTR_EN      1
#define EB_CNTR_WS_HOLD(hold)  ((hold & 0xf) << 20)
#define EB_CNTR_WS_ACTIVE(act) ((act & 0xf) << 16)
#define EB_CNTR_WS_SETUP(setup) ((setup & 0xff) << 8)

/***
*  PWR control
**/

typedef struct
{
  __IO  uint32_t KEY;
  
  __IO  uint32_t CNTR0;
  __IO  uint32_t CNTR1;
  __IO  uint32_t CNTR2;
  
  __IO  uint32_t STAT;

  __IO  uint32_t ULIMIT;
  __IO  uint32_t CLIMIT;
 
} PWRControl;


#define PWR_BASE      (( uint32_t) 0x40002000)
#define PWR       ((PWRControl *) PWR_BASE)

/***
*  WDT control
**/


typedef struct
{
  __IO  uint32_t KEY;
  __IO  uint32_t PRL;
  __IO  uint32_t EN;
  __IO  uint32_t CNT;
 
} WDTControl;


#define WDT_BASE      (( uint32_t) 0x40004000)
#define WDT       ((WDTControl *) WDT_BASE)

/***
*  ADCx control
**/

typedef struct
{
  __IO  uint32_t KEY;
  
  __IO  uint32_t CONFIG0;
  __IO  uint32_t CONFIG1;
  __IO  uint32_t CONFIG2;
  __IO  uint32_t CONTROL;
  __IO  uint32_t STATUS;

  __IO  uint32_t RESULT;
  __IO  uint32_t RESULTCH_xx[64];

  __IO  uint32_t MAX_LEVEL;
  __IO  uint32_t MIN_LEVEL;

  __IO  uint32_t CHSEL0;
  __IO  uint32_t CHSEL1;

  __IO  uint32_t DMAREQ;
  __IO  uint32_t STATRCH0;
  __IO  uint32_t STATRCH1;
  
  __IO  uint32_t FIFOEN0;
  __IO  uint32_t FIFOEN1;
 
} ADCxControl;


#define ADC0_BASE     (( uint32_t) 0x400AA000)

#define ADC0        ((ADCxControl *) ADC0_BASE)

#define ADC1_BASE     (( uint32_t) 0x400AB000)

#define ADC1        ((ADCxControl *) ADC1_BASE)

/***
*  DACx control
**/

typedef struct
{
  __IO  uint32_t KEY;
  
  __IO  uint32_t CONFIG0;
  __IO  uint32_t CONFIG1;

  __IO  uint32_t DATA;

  __IO  uint32_t STATUS;
  __IO  uint32_t IMSK;
  __IO  uint32_t DMAREQ;
 
} DACxControl;


#define DAC0_BASE     (( uint32_t) 0x400AC000)

#define DAC0        ((DACxControl *) DAC0_BASE)

#define DAC1_BASE     (( uint32_t) 0x400AD000)

#define DAC1        ((DACxControl *) DAC1_BASE)


/***
*  CRC control
**/

typedef struct
{
  __IO  uint32_t KEY;
  __IO  uint32_t CTRL;
  __IO  uint32_t STAT;
  __IO  uint32_t DIN;
  __IO  uint32_t VALL;
  __IO  uint32_t VALH;
  __IO  uint32_t POLL;
  __IO  uint32_t POLH;
  __IO  uint32_t FINXORL;
  __IO  uint32_t FINXORH;
  __IO  uint32_t CNTFLOW;
} CRC_Control;

#define CRC_BASE      (( uint32_t) 0x400AF000)

#define CRC       ((CRC_Control *) CRC_BASE)


/***
*  ECC control
**/

typedef struct
{
  __IO  uint32_t KEY;
  __IO  uint32_t CTRL;
  __IO  uint32_t STAT;
  __IO  uint32_t DATIL;
  __IO  uint32_t DATIH;
  __IO  uint32_t ECCI;
  __IO  uint32_t DLER;
  __IO  uint32_t DHER;
  __IO  uint32_t ECER;
  __IO  uint32_t DATOL;
  __IO  uint32_t DATOH;
  __IO  uint32_t ECCO;
  __IO  uint32_t H000;
  __IO  uint32_t H032;
  __IO  uint32_t H100;
  __IO  uint32_t H132;
  __IO  uint32_t H200;
  __IO  uint32_t H232;
  __IO  uint32_t H300;
  __IO  uint32_t H332;
  __IO  uint32_t H400;
  __IO  uint32_t H432;
  __IO  uint32_t H500;
  __IO  uint32_t H532;
  __IO  uint32_t H600;
  __IO  uint32_t H632;
  __IO  uint32_t H700;
  __IO  uint32_t H732;
  
} ECC_Control;

#define ECC_BASE      (( uint32_t) 0x400B0000)

#define ECC       ((ECC_Control *) ECC_BASE)


/***
*  BKP control
**/

typedef struct
{
  __IO  uint32_t REG_xx[60];
  __IO  uint32_t KEY;
  __IO  uint32_t RESERV_0[3];
  
  __IO  uint32_t REG_60_TMR0;
  __IO  uint32_t REG_61_TMR0;
  __IO  uint32_t REG_62_TMR0;
  __IO  uint32_t REG_63_TMR0
  ;
  __IO  uint32_t REG_60_TMR1;
  __IO  uint32_t REG_61_TMR1;
  __IO  uint32_t REG_62_TMR1;
  __IO  uint32_t REG_63_TMR1;
  
  __IO  uint32_t REG_60_TMR2;
  __IO  uint32_t REG_61_TMR2;
  __IO  uint32_t REG_62_TMR2;
  __IO  uint32_t REG_63_TMR2;
  
  __IO  uint32_t RTC_CNT_TMR0;
  __IO  uint32_t RTC_DIV_TMR0;
  __IO  uint32_t RTC_PRL_TMR0;
  __IO  uint32_t RTC_ALRM_TMR0;
  __IO  uint32_t RTC_CS_TMR0;
  
  __IO  uint32_t RESERV_1[3];

  __IO  uint32_t RTC_CNT_TMR1;
  __IO  uint32_t RTC_DIV_TMR1;
  __IO  uint32_t RTC_PRL_TMR1;
  __IO  uint32_t RTC_ALRM_TMR1;
  __IO  uint32_t RTC_CS_TMR1;
  
  __IO  uint32_t RESERV_2[3];

  __IO  uint32_t RTC_CNT_TMR2;
  __IO  uint32_t RTC_DIV_TMR2;
  __IO  uint32_t RTC_PRL_TMR2;
  __IO  uint32_t RTC_ALRM_TMR2;
  __IO  uint32_t RTC_CS_TMR2;
  
  
} BKPControl;


#define BKP_BASE      (( uint32_t) 0x40001000)

#define BKP       ((BKPControl *) BKP_BASE)
#define BKP_KEY       ((uint32_t) 0x8555aaa1)

/***
*  CRYPTO control
**/

typedef struct
{
  __IO  uint32_t CRPT_CWR;
  __IO  uint32_t CRPT_SR;
  __IO  uint32_t CRPT_DATA;
  __IO  uint32_t CRPT_KR;
  __IO  uint32_t CRPT_CR;
  __IO  uint32_t CRPT_SYNR;
  __IO  uint32_t CRPT_IMIT;
  __IO  uint32_t CRPT_ITER;
} CRPTControl;


#define CRPT_BASE     (( uint32_t) 0x400AE000)

#define CRPT        ((CRPTControl *) CRPT_BASE)

/***
*   MIL1553 Control 
**/

typedef struct
{
  __IO  uint32_t DATA[1024];
  __IO  uint32_t CONTROL;
  __IO  uint32_t STATUS;
  __IO  uint32_t ERROR;
  __IO  uint32_t CommandWord1;
  __IO  uint32_t CommandWord2;
  __IO  uint32_t ModeData;
  __IO  uint32_t StatusWord1;
  __IO  uint32_t StatusWord2;
  __IO  uint32_t INTEN;
  __IO  uint32_t MSG;
} MIL1553Control;


#define MIL15531_BASE     (( uint32_t) 0x400A6000)
#define MIL15532_BASE     (( uint32_t) 0x400A8000)

#define MIL_STD_15531     ((MIL1553Control *) MIL15531_BASE)
#define MIL_STD_15532     ((MIL1553Control *) MIL15532_BASE)

/***
*  ARINC429R Control 
**/

typedef struct
{
  __IO  uint32_t CONTROL1;
  __IO  uint32_t CONTROL2;
  __IO  uint32_t CONTROL3;
  __IO  uint32_t STATUS1;
  __IO  uint32_t STATUS2;
  __IO  uint32_t CONTROL4;
  __IO  uint32_t CONTROL5;
  __IO  uint32_t CHANNEL;
  __IO  uint32_t LABEL;
  __IO  uint32_t DATA_R;
  uint32_t RESERVED0[2];
  __IO  uint32_t DATA_R1;
  __IO  uint32_t DATA_R2;
  __IO  uint32_t DATA_R3;
  __IO  uint32_t DATA_R4;
  __IO  uint32_t DATA_R5;
  __IO  uint32_t DATA_R6;
  __IO  uint32_t DATA_R7;
  __IO  uint32_t DATA_R8;
  uint32_t RESERVED1[6];
  __IO  uint32_t INTMASK;
  uint32_t RESERVED2;
  __IO  uint32_t CONTROL8;
  __IO  uint32_t CONTROL9;
} ARINC429RControl;

typedef struct
{
  __IO  uint32_t DATA_R[1024];
} ARINC429RData;

#define ARINC429R1_BASE     (( uint32_t) 0x4009E000)
#define ARINC429R2_BASE     (( uint32_t) 0x400A2000)
#define ARINC429RD1_BASE    (( uint32_t) 0x4009F000)
#define ARINC429RD2_BASE    (( uint32_t) 0x400A3000)

#define ARINC429R1      ((ARINC429RControl *) ARINC429R1_BASE)
#define ARINC429R2      ((ARINC429RControl *) ARINC429R2_BASE)
#define ARINC429RD1     ((ARINC429RData *) ARINC429RD1_BASE)
#define ARINC429RD2     ((ARINC429RData *) ARINC429RD2_BASE)

/***
*   ARINC429T Control 
**/

typedef struct
{
  __IO  uint32_t CONTROL1;
  __IO  uint32_t CONTROL2;
  __IO  uint32_t STATUS;
  __IO  uint32_t DATA1_T;
  __IO  uint32_t DATA2_T;
  __IO  uint32_t DATA3_T;
  __IO  uint32_t DATA4_T;
  __IO  uint32_t CONTROL3;
  __IO  uint32_t CONTROL4;
} ARINC429TControl;

typedef struct
{
  __IO  uint32_t DATA_T[1024];
} ARINC429TData;

#define ARINC429T1_BASE     (( uint32_t) 0x400A0000)
#define ARINC429T2_BASE     (( uint32_t) 0x400A4000)
#define ARINC429TD1_BASE    (( uint32_t) 0x400A1000)
#define ARINC429TD2_BASE    (( uint32_t) 0x400A5000)

#define ARINC429T1      ((ARINC429TControl *) ARINC429T1_BASE)
#define ARINC429T2      ((ARINC429TControl *) ARINC429T2_BASE)
#define ARINC429TD1     ((ARINC429TData *) ARINC429TD1_BASE)
#define ARINC429TD2     ((ARINC429TData *) ARINC429TD2_BASE)

/***
*   CLOCK Control 
**/
typedef struct
{
  __IO  uint32_t KEY;
  __IO  uint32_t MAX_CLK;
  __IO  uint32_t CPU_CLK;
  __IO  uint32_t PER0_CLK;
  __IO  uint32_t PER1_CLK;
  
  __IO  uint32_t CPU_CHK0;
  __IO  uint32_t CPU_CHK1;
  __IO  uint32_t CPU_CHK2;
  __IO  uint32_t CPU_STAT;
  
  __IO  uint32_t LSI_CLK;
  __IO  uint32_t LSI_CHK0;
  __IO  uint32_t LSI_CHK1;
  __IO  uint32_t LSI_CHK2;
  __IO  uint32_t LSI_STAT;
  
  __IO  uint32_t HSI_STAT;
  
  __IO  uint32_t LSE_CLK;
  __IO  uint32_t LSE_CHK0;
  __IO  uint32_t LSE_CHK1;
  __IO  uint32_t LSE_CHK2;
  __IO  uint32_t LSE_STAT;
  
  __IO  uint32_t HSE0_CLK;
  __IO  uint32_t HSE0_CHK0;
  __IO  uint32_t HSE0_CHK1;
  __IO  uint32_t HSE0_CHK2;
  __IO  uint32_t HSE0_STAT;

  __IO  uint32_t HSE1_CLK;
  __IO  uint32_t HSE1_CHK0;
  __IO  uint32_t HSE1_CHK1;
  __IO  uint32_t HSE1_CHK2;
  __IO  uint32_t HSE1_STAT;
  
  __IO  uint32_t PLL0_CLK;
  __IO  uint32_t PLL0_CHK0;
  __IO  uint32_t PLL0_CHK1;
  __IO  uint32_t PLL0_CHK2;
  __IO  uint32_t PLL0_STAT;
  
  __IO  uint32_t PLL1_CLK;
  __IO  uint32_t PLL1_CHK0;
  __IO  uint32_t PLL1_CHK1;
  __IO  uint32_t PLL1_CHK2;
  __IO  uint32_t PLL1_STAT;
  
  __IO  uint32_t PLL2_CLK;
  __IO  uint32_t PLL2_CHK0;
  __IO  uint32_t PLL2_CHK1;
  __IO  uint32_t PLL2_CHK2;
  __IO  uint32_t PLL2_STAT;
  
  __IO  uint32_t PLL3_CLK;
  __IO  uint32_t PLL3_CHK0;
  __IO  uint32_t PLL3_CHK1;
  __IO  uint32_t PLL3_CHK2;
  __IO  uint32_t PLL3_STAT;
  
  __IO  uint32_t PLL4_CLK;
  __IO  uint32_t PLL4_CHK0;
  __IO  uint32_t PLL4_CHK1;
  __IO  uint32_t PLL4_CHK2;
  __IO  uint32_t PLL4_STAT;
  
  __IO  uint32_t PLL5_CLK;
  __IO  uint32_t PLL5_CHK0;
  __IO  uint32_t PLL5_CHK1;
  __IO  uint32_t PLL5_CHK2;
  __IO  uint32_t PLL5_STAT;
  
  __IO  uint32_t PLL6_CLK;
  __IO  uint32_t PLL6_CHK0;
  __IO  uint32_t PLL6_CHK1;
  __IO  uint32_t PLL6_CHK2;
  __IO  uint32_t PLL6_STAT;
  
  __IO  uint32_t PLL7_CLK;
  __IO  uint32_t PLL7_CHK0;
  __IO  uint32_t PLL7_CHK1;
  __IO  uint32_t PLL7_CHK2;
  __IO  uint32_t PLL7_STAT;
  
  __IO  uint32_t CAN0_CLK;
  __IO  uint32_t CAN1_CLK;
  __IO  uint32_t CAN2_CLK;
  __IO  uint32_t CAN3_CLK;
  __IO  uint32_t CAN4_CLK;
  __IO  uint32_t CAN5_CLK;

  __IO  uint32_t TIM0_CLK;
  __IO  uint32_t TIM1_CLK;
  __IO  uint32_t TIM2_CLK;
  __IO  uint32_t TIM3_CLK;
  __IO  uint32_t TIM4_CLK;
  __IO  uint32_t TIM5_CLK;


  __IO  uint32_t MIL0_CLK;
  __IO  uint32_t MIL1_CLK;
  __IO  uint32_t MIL2_CLK;
  __IO  uint32_t MIL3_CLK;
  
  __IO  uint32_t ARC0_CLK;
  __IO  uint32_t ARC1_CLK;
  __IO  uint32_t ARC2_CLK;
  __IO  uint32_t ARC3_CLK;
  
  __IO  uint32_t EMAC0_CLK;
  __IO  uint32_t EMAC1_CLK;
  __IO  uint32_t EPHY0_CLK;
  __IO  uint32_t EPHY1_CLK;
  
  __IO  uint32_t SPW0_CLK;
  __IO  uint32_t SPW1_CLK;
  __IO  uint32_t SPHY0_CLK;
  __IO  uint32_t SPHY1_CLK;
  
  __IO  uint32_t UART0_CLK;
  __IO  uint32_t UART1_CLK;
  __IO  uint32_t UART2_CLK;
  __IO  uint32_t UART3_CLK;
  __IO  uint32_t UART4_CLK;
  __IO  uint32_t UART5_CLK;
  
  __IO  uint32_t SSP0_CLK;
  __IO  uint32_t SSP1_CLK;
  __IO  uint32_t SSP2_CLK;
  __IO  uint32_t SSP3_CLK;
  __IO  uint32_t SSP4_CLK;
  __IO  uint32_t SSP5_CLK;
  
  __IO  uint32_t USB_CLK;
  
  __IO  uint32_t ADC0_CLK;
  __IO  uint32_t ADC1_CLK;
  
  __IO  uint32_t DAC0_CLK;
  __IO  uint32_t DAC1_CLK;
  
  __IO  uint32_t RTC_CLK;
  
} CLKControl;

#define CLK_CNTR_BASE     (( uint32_t) 0x40000000)

#define CLK_CNTR      ((CLKControl *) CLK_CNTR_BASE)

/*
#define BKP_BASE      (( uint32_t) 0x040001000)
#define BKP_KEY       ((uint32_t) 0x8555aaa1)

*/

/***
*  PORT control
**/

typedef struct
{
  __IO  uint32_t KEY;
  __IO  uint32_t RXTX;
  __IO  uint32_t SRXTX;
  __IO  uint32_t CRXTX;
  
  __IO  uint32_t SOE;
  __IO  uint32_t COE;

  __IO  uint32_t SFUNC[4];

  __IO  uint32_t CFUNC[4];
  
  __IO  uint32_t SANALOG;
  __IO  uint32_t CANALOG;
  
  __IO  uint32_t SPULLUP;
  __IO  uint32_t CPULLUP;
  
  __IO  uint32_t SPULLDOWN;
  __IO  uint32_t CPULLDOWN;

  __IO  uint32_t SPD;
  __IO  uint32_t CPD;

  __IO  uint32_t SPWR[2];
  __IO  uint32_t CPWR[2];

  __IO  uint32_t SCL;
  __IO  uint32_t CCL;

  __IO  uint32_t SIE;
  __IO  uint32_t CIE;

  __IO  uint32_t SIT;
  __IO  uint32_t CIT;
  
  __IO  uint32_t SIR;
  __IO  uint32_t CIR; 

  __IO  uint32_t HCUR; 
 
} PortControl;



#define PORTA_BASE      (( uint32_t) 0x40080000)
#define PORTB_BASE      (( uint32_t) 0x40081000)
#define PORTC_BASE      (( uint32_t) 0x40082000)
#define PORTD_BASE      (( uint32_t) 0x40083000)
#define PORTE_BASE      (( uint32_t) 0x40084000)
#define PORTF_BASE      (( uint32_t) 0x40085000)

#define PORTA           ((PortControl *) PORTA_BASE)
#define PORTB           ((PortControl *) PORTB_BASE)
#define PORTC           ((PortControl *) PORTC_BASE)
#define PORTD           ((PortControl *) PORTD_BASE)
#define PORTE           ((PortControl *) PORTE_BASE)
#define PORTF           ((PortControl *) PORTF_BASE)

#define FUNC_MASK 0x0f
#define FUNCREGNUM(pin) (pin >> 3)
#define _FUNCPIN(TYPE) TYPE ## FUNC
#define FUNCPIN(pin,TYPE) _FUNCPIN(TYPE)[FUNCREGNUM(pin)]
#define SFUNCPIN(pin) FUNCPIN(pin,S)
#define CFUNCPIN(pin) FUNCPIN(pin,C)
#define FUNCVAL(pin,func) ((func & FUNC_MASK) << ((pin % 8)<<2))

#define PWR_MASK 0x3
#define PWRREGNUM(pin) (pin >> 4)
#define _PWRPIN(type) type ## PWR
#define PWRPIN(pin, type) _PWRPIN(type)[PWRREGNUM(pin)]
#define SPWRPIN(pin) PWRPIN(pin, S)
#define CPWRPIN(pin) PWRPIN(pin, C)
#define PWRVAL(pin, pwr) ((pwr & PWR_MASK) << ((pin % 16) << 1))

/****
/ USB controller interface
**/
typedef struct
{
  __IO uint32_t HTXC;
  __IO uint32_t HTXT;
  __IO uint32_t HTXLC;
  __IO uint32_t HTXSE;
  __IO uint32_t HTXA;
  __IO uint32_t HTXE;
  __IO uint32_t HFN_L;
  __IO uint32_t HFN_H;
  __IO uint32_t HIS;
  __IO uint32_t HIM;
  __IO uint32_t HRXS;
  __IO uint32_t HRXP;
  __IO uint32_t HRXA;
  __IO uint32_t HRXE;
  __IO uint32_t HRXCS;
  __IO uint32_t HSTM;
  __IO uint32_t RESERVED1[16];
  __IO uint32_t HRXFD;
  __IO uint32_t RESERVED2;
  __IO uint32_t HRXFDC_L;
  __IO uint32_t HRXFDC_H;
  __IO uint32_t HRXFC;
  __IO uint32_t RESERVED3[11];
  __IO uint32_t HTXFD;
  __IO uint32_t RESERVED4[3];
  __IO uint32_t HTXFDC;
  __IO uint32_t RESERVED5[11];

  __IO uint32_t SEP0_CTRL;
  __IO uint32_t SEP0_STS;
  __IO uint32_t SEP0_TS;
  __IO uint32_t SEP0_NTS;
  __IO uint32_t SEP1_CTRL;
  __IO uint32_t SEP1_STS;
  __IO uint32_t SEP1_TS;
  __IO uint32_t SEP1_NTS;
  __IO uint32_t SEP2_CTRL;
  __IO uint32_t SEP2_STS;
  __IO uint32_t SEP2_TS;
  __IO uint32_t SEP2_NTS;
  __IO uint32_t SEP3_CTRL;
  __IO uint32_t SEP3_STS;
  __IO uint32_t SEP3_TS;
  __IO uint32_t SEP3_NTS;
  __IO uint32_t SC;
  __IO uint32_t SLS;
  __IO uint32_t SIS;
  __IO uint32_t SIM;
  __IO uint32_t SA;
  __IO uint32_t SFN_L;
  __IO uint32_t SFN_H;
  __IO uint32_t RESERVED6[9];
 
  __IO uint32_t SEP0_RXFD;
  __IO uint32_t RESERVED7;
  __IO uint32_t SEP0_RXFDC_L;
  __IO uint32_t SEP0_RXFDC_H;
  __IO uint32_t SEP0_RXFC;
  __IO uint32_t RESERVED8[11];
  __IO uint32_t SEP0_TXFD;
  __IO uint32_t RESERVED9[3];
  __IO uint32_t SEP0_TXFDC;
  __IO uint32_t RESERVED10[11];
  __IO uint32_t SEP1_RXFD;
  __IO uint32_t RESERVED11;
  __IO uint32_t SEP1_RXFDC_L;
  __IO uint32_t SEP1_RXFDC_H;
  __IO uint32_t SEP1_RXFC;
  __IO uint32_t RESERVED12[11];
  __IO uint32_t SEP1_TXFD;
  __IO uint32_t RESERVED13[3];
  __IO uint32_t SEP1_TXFDC;
  __IO uint32_t RESERVED14[11];
  __IO uint32_t SEP2_RXFD;
  __IO uint32_t RESERVED15;
  __IO uint32_t SEP2_RXFDC_L;
  __IO uint32_t SEP2_RXFDC_H;
  __IO uint32_t SEP2_RXFC;
  __IO uint32_t RESERVED16[11];
  __IO uint32_t SEP2_TXFD;
  __IO uint32_t RESERVED17[3];
  __IO uint32_t SEP2_TXFDC;
  __IO uint32_t RESERVED18[11];
  __IO uint32_t SEP3_RXFD;
  __IO uint32_t RESERVED19;
  __IO uint32_t SEP3_RXFDC_L;
  __IO uint32_t SEP3_RXFDC_H;
  __IO uint32_t SEP3_RXFC;
  __IO uint32_t RESERVED20[11];
  __IO uint32_t SEP3_TXFD;
  __IO uint32_t RESERVED21[3];
  __IO uint32_t SEP3_TXFDC;
  __IO uint32_t RESERVED22[11];
  
  __IO uint32_t HSCR;
  __IO uint32_t HSVR;
}
USB_Typedef;

#define USB_BASE  0x4009D000
#define USB   ((USB_Typedef *) USB_BASE)

/***
*  TIMER32 control
**/
typedef struct
{
  __IO  uint32_t CNT;
  __IO  uint32_t PSG;
  __IO  uint32_t ARR;
  __IO  uint32_t CNTRL;
  __IO  uint32_t CCR1;
  __IO  uint32_t CCR2;
  __IO  uint32_t CCR3;
  __IO  uint32_t CCR4;
  __IO  uint32_t CH1_CNTRL;
  __IO  uint32_t CH2_CNTRL;
  __IO  uint32_t CH3_CNTRL;
  __IO  uint32_t CH4_CNTRL;
  __IO  uint32_t CH1_CNTRL1;
  __IO  uint32_t CH2_CNTRL1;
  __IO  uint32_t CH3_CNTRL1;
  __IO  uint32_t CH4_CNTRL1;
  __IO  uint32_t CH1_DTG;
  __IO  uint32_t CH2_DTG;
  __IO  uint32_t CH3_DTG;
  __IO  uint32_t CH4_DTG;
  __IO  uint32_t BRKETR_CNTRL;
  __IO  uint32_t STATUS;
  __IO  uint32_t IE;
  __IO  uint32_t DMA_RE;
  __IO  uint32_t CH1_CNTRL2;
  __IO  uint32_t CH2_CNTRL2;
  __IO  uint32_t CH3_CNTRL2;
  __IO  uint32_t CH4_CNTRL2;
  __IO  uint32_t CCR11;
  __IO  uint32_t CCR21;
  __IO  uint32_t CCR31;
  __IO  uint32_t CCR41;
  __IO  uint32_t DMA_RECh1;
  __IO  uint32_t DMA_RECh2;
  __IO  uint32_t DMA_RECh3;
  __IO  uint32_t DMA_RECh4;
  
  
}MDR_TMR_TypeDef;

#define MDR_TMR_BASE                 (PERIPH_BASE + 0x80000)

#define MDR_TMR0_BASE     (MDR_TMR_BASE + 0xA000)
#define MDR_TMR1_BASE     (MDR_TMR_BASE + 0xB000)
#define MDR_TMR2_BASE     (MDR_TMR_BASE + 0xC000)
#define MDR_TMR3_BASE     (MDR_TMR_BASE + 0xD000)
#define MDR_TMR4_BASE     (MDR_TMR_BASE + 0xE000)
#define MDR_TMR5_BASE     (MDR_TMR_BASE + 0xF000)

#define MDR_TMR0                     ((MDR_TMR_TypeDef *) MDR_TMR0_BASE)
#define MDR_TMR1                     ((MDR_TMR_TypeDef *) MDR_TMR1_BASE)
#define MDR_TMR2                     ((MDR_TMR_TypeDef *) MDR_TMR2_BASE)
#define MDR_TMR3                     ((MDR_TMR_TypeDef *) MDR_TMR3_BASE)
#define MDR_TMR4                     ((MDR_TMR_TypeDef *) MDR_TMR4_BASE)
#define MDR_TMR5                     ((MDR_TMR_TypeDef *) MDR_TMR5_BASE)

/***
*  MDR_CAN control
**/
typedef struct
{
__IO  uint32_t ID;
__IO  uint32_t DLC;
__IO  uint32_t DATAL;
__IO  uint32_t DATAH;
}MDR_CAN_BUF_TypeDef;

typedef struct
{
__IO  uint32_t MASK;
__IO  uint32_t FILTER;
}MDR_CAN_BUF_FILTER_TypeDef;

typedef struct
{
  __IO  uint32_t CONTROL;  
  __IO  uint32_t STATUS;
  __IO  uint32_t BITTMNG;
  __IO  uint32_t Reserved0;
  __IO  uint32_t INT_EN;
  __IO  uint32_t Reserved1[2];
  __IO  uint32_t OVER;
  __IO  uint32_t RXID;
  __IO  uint32_t RXDLC;
  __IO  uint32_t RXDATAL;
  __IO  uint32_t RXDATAH;
  __IO  uint32_t TXID;
  __IO  uint32_t TXDLC;
  __IO  uint32_t DATAL;
  __IO  uint32_t DATAH;
  __IO  uint32_t BUF_CON[32];
  __IO  uint32_t INT_RX;
  __IO  uint32_t RX;
  __IO  uint32_t INT_TX;
  __IO  uint32_t TX;
  __IO  uint32_t Reserved2[76];
  MDR_CAN_BUF_TypeDef BUF[32];
  __IO  uint32_t Reserved3[64];
  MDR_CAN_BUF_FILTER_TypeDef FILTER[32];

}MDR_CAN_TypeDef;

#define MDR_CAN_BASE                 (PERIPH_BASE + 0x90000)

#define MDR_CAN0_BASE     (MDR_CAN_BASE + 0x0000)
#define MDR_CAN1_BASE     (MDR_CAN_BASE + 0x1000)
#define MDR_CAN2_BASE     (MDR_CAN_BASE + 0x2000)
#define MDR_CAN3_BASE     (MDR_CAN_BASE + 0x3000)
#define MDR_CAN4_BASE     (MDR_CAN_BASE + 0x4000)

#define MDR_CAN0                     ((MDR_CAN_TypeDef *) MDR_CAN0_BASE)
#define MDR_CAN1                     ((MDR_CAN_TypeDef *) MDR_CAN1_BASE)
#define MDR_CAN2                     ((MDR_CAN_TypeDef *) MDR_CAN2_BASE)
#define MDR_CAN3                     ((MDR_CAN_TypeDef *) MDR_CAN3_BASE)
#define MDR_CAN4                     ((MDR_CAN_TypeDef *) MDR_CAN4_BASE)

/***
*  MDR_SSP control
**/
typedef struct
{
  __IO  uint32_t CR0;
  __IO  uint32_t CR1;
  __IO  uint32_t DR;
  __IO  uint32_t SR;
  __IO  uint32_t CPSR;
  __IO  uint32_t IMSC;
  __IO  uint32_t RIS;
  __IO  uint32_t MIS;
  __IO  uint32_t ICR;
  __IO  uint32_t DMACR;
}MDR_SSP_TypeDef;

#define MDR_SSP_BASE                 (PERIPH_BASE + 0x90000)

#define MDR_SSP0_BASE     (MDR_SSP_BASE + 0x5000)
#define MDR_SSP1_BASE     (MDR_SSP_BASE + 0x6000)
#define MDR_SSP2_BASE     (MDR_SSP_BASE + 0x7000)
#define MDR_SSP3_BASE     (MDR_SSP_BASE + 0x8000)

#define MDR_SSP0                     ((MDR_SSP_TypeDef *) MDR_SSP0_BASE)
#define MDR_SSP1                     ((MDR_SSP_TypeDef *) MDR_SSP1_BASE)
#define MDR_SSP2                     ((MDR_SSP_TypeDef *) MDR_SSP2_BASE)
#define MDR_SSP3                     ((MDR_SSP_TypeDef *) MDR_SSP3_BASE)


/***
*  MDR_UART control
**/
typedef struct
{
  __IO  uint32_t DR;
  __IO  uint32_t RSR_ECR;
  __IO  uint32_t Reserved1[4];
  __IO  uint32_t FR;
  __IO  uint32_t Reserved2;
  __IO  uint32_t ILPR;
  __IO  uint32_t IBRD;
  __IO  uint32_t FBRD;
  __IO  uint32_t LCR_H;
  __IO  uint32_t CR;
  __IO  uint32_t IFLS;
  __IO  uint32_t IMSC;
  __IO  uint32_t RIS;
  __IO  uint32_t MIS;
  __IO  uint32_t ICR;
  __IO  uint32_t DMACR;
}MDR_UART_TypeDef;

#define MDR_UART_BASE                 (PERIPH_BASE + 0x90000)

#define MDR_UART0_BASE      (MDR_UART_BASE + 0x9000)
#define MDR_UART1_BASE      (MDR_UART_BASE + 0xA000)
#define MDR_UART2_BASE      (MDR_UART_BASE + 0xB000)
#define MDR_UART3_BASE      (MDR_UART_BASE + 0xC000)

#define MDR_UART0                     ((MDR_UART_TypeDef *) MDR_UART0_BASE)
#define MDR_UART1                     ((MDR_UART_TypeDef *) MDR_UART1_BASE)
#define MDR_UART2                     ((MDR_UART_TypeDef *) MDR_UART2_BASE)
#define MDR_UART3                     ((MDR_UART_TypeDef *) MDR_UART3_BASE)


/***
*  MDR_SPW control
**/
typedef struct
{
  __IO  uint32_t CONTROL;
  __IO  uint32_t STATUS;
  __IO  uint32_t DIV;
  __IO  uint32_t TIME;
  __IO  uint32_t PAUSE;
  __IO  uint32_t FIFORX;
  __IO  uint32_t FIFOTX;
  __IO  uint32_t CNTRX_PACK;
  __IO  uint32_t CNTRX0_PACK;
  __IO  uint32_t NUM_TXDESC;
  __IO  uint32_t NUM_RXDESC;
  __IO  uint32_t PHY_CNTR;
  __IO  uint32_t RESERV[4];
  __IO  uint32_t RXDESC[16];
  __IO  uint32_t TXDESC[16];
}MDR_SPW_TypeDef;

#define MDR_SPW_BASE                 (PERIPH_BASE + 0x80000)

#define MDR_SPW0_BASE     (MDR_SPW_BASE + 0x8000)
#define MDR_SPW1_BASE     (MDR_SPW_BASE + 0x9000)

#define MDR_SPW0                     ((MDR_SPW_TypeDef *) MDR_SPW0_BASE)
#define MDR_SPW1                     ((MDR_SPW_TypeDef *) MDR_SPW1_BASE)

/***
*  MDR_ETH control
**/
typedef struct
{
  __IO  uint16_t DILIMETR;
  __IO  uint16_t MAC_T;
  __IO  uint16_t MAC_M;
  __IO  uint16_t MAC_H;
  __IO  uint16_t HASH0;
  __IO  uint16_t HASH1;
  __IO  uint16_t HASH2;
  __IO  uint16_t HASH3;
  __IO  uint16_t IPG;
  __IO  uint16_t PSC;
  __IO  uint16_t BAG;
  __IO  uint16_t JITTERWND;
  __IO  uint16_t R_CFG;
  __IO  uint16_t X_CFG;
  __IO  uint32_t G_CFG;//__IO uint16_t G_CFGl;__IO  uint16_t G_CFGh;
  __IO  uint16_t IMR;
  __IO  uint16_t IFR;
  __IO  uint16_t MDIO_CTRL;
  __IO  uint16_t MDIO_DATA;
  __IO  uint16_t R_HEAD;
  __IO  uint16_t X_TAIL;
  __IO  uint16_t R_TAIL;
  __IO  uint16_t X_HEAD;
  __IO  uint16_t STAT;
  __IO  uint16_t RCOUNTER;
  __IO  uint16_t PHY_CONTROL;
  __IO  uint16_t PHY_STATUS;
  __IO  uint16_t PHY_CNTR_A;
}MDR_ETH_TypeDef;

#define MDR_ETH_BASE                 (SRAM_BASE + 0x1000000)

#define MDR_ETH0_BASE     (MDR_ETH_BASE + 0x00000)
#define MDR_ETH1_BASE     (MDR_ETH_BASE + 0x10000)

#define MDR_ETH0                     ((MDR_ETH_TypeDef *) MDR_ETH0_BASE)
#define MDR_ETH1                     ((MDR_ETH_TypeDef *) MDR_ETH1_BASE)


/******************************************************************************/
/*                         Peripheral memory map                              */
/******************************************************************************/
/* Peripheral and SRAM base address */
#define SRAM_BASE             ((     uint32_t)0x20000000)
#define PERIPH_BASE           ((     uint32_t)0x40000000)

/* Peripheral memory map */
#define GPIO_BASE                 (PERIPH_BASE + 0x0e000)

#define GPIO0_BASE                (GPIO_BASE)
#define GPIO1_BASE                (GPIO_BASE       + 0x0800)
#define GPIO2_BASE                (GPIO_BASE       + 0x1000)

#define SMPID     ((volatile uint32_t) *((uint32_t *)0xe00fe000))
#define SMPID_BASE        ((uint32_t) 0xe00fe000)
/******************************************************************************/
/*                         Peripheral declaration                             */
/******************************************************************************/
#define GPIO0                     ((GPIO_TypeDef *) GPIO0_BASE)
#define GPIO1                     ((GPIO_TypeDef *) GPIO1_BASE)
#define GPIO2                     ((GPIO_TypeDef *) GPIO2_BASE)


#endif



