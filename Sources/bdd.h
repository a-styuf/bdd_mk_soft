#ifndef _BDD_H_
#define _BDD_H_

#include "1986ve8_lib/cm4ikmcu.h"

#include "mpi.h"
#include "timers.h"
#include "adc.h"
#include "dac.h"
#include "gpio.h"
#include "termo_res.h"
#include "oai_dd.h"
#include "uarts.h"
#include "debug.h"
#include "bdd_mpi_interface.h"
#include "mvip.h"
#include "ims.h"
#include "stm_bdd.h"

//defines
#define BDD_MK_FRAME_MODIFICATOR 1
#define BDD_MK_FRAME_DEVICE_NUMBER 1
#define BDD_MK_FRAME_FABRICATION_NUM 12 

#define BDD_MK_FRAME_TYPE_SYSTEM 0x01
#define BDD_MK_FRAME_TYPE_OAI_DD 0x02
#define BDD_MK_FRAME_TYPE_IMS_DD 0x03

#define BDD_MK_FRAME_SADDR_SYSTEM 0x0F
#define BDD_MK_FRAME_SADDR_OAI_DD 0x01
#define BDD_MK_FRAME_SADDR_IMS_DD 0x02

// дефайны для управления командами
#define BDD_MK_FRAME_SADDR_COMMAND 30

#define BDD_MK_COMMAND_LINK_CHECK   0
#define BDD_MK_COMMAND_SET_OAI_DD_1_MODE  1
#define BDD_MK_COMMAND_SET_OAI_DD_2_MODE  2
#define BDD_MK_COMMAND_SET_OAI_DD_1_FILTER  3
#define BDD_MK_COMMAND_SET_OAI_DD_2_FILTER  4
#define BDD_MK_COMMAND_SET_OAI_DD_1_PID_SETTING  5
#define BDD_MK_COMMAND_SET_OAI_DD_2_PID_SETITNG  6
#define BDD_MK_COMMAND_SET_STM_DEBUG  7
#define BDD_MK_COMMAND_SET_IMS_MODE  8
#define BDD_MK_COMMAND_SET_IMS_KU  9
#define BDD_MK_COMMAND_SET_BDD_MODE 10

// режимы работы БДД
#define BDD_MK_MODE_OFF 0
#define BDD_MK_MODE_AUTO 1
#define BDD_MK_MODE_IMS 2
#define BDD_MK_MODE_OAI_DD 3
#define BDD_MK_MODE_DEFAULT BDD_MK_MODE_AUTO

//давление по умолчанию
#define BDD_MK_PRESSURE_DEFAULT 0x0000
#define BDD_MK_IMS_PWR_ON_TIMEOUT_MS 2000

#define BDD_MK_PR_SOURCE_DEFAULT 0x00
#define BDD_MK_PR_SOURCE_IMS 0x01
#define BDD_MK_PR_SOURCE_OAI_DD 0x02

// data structures
#pragma pack(2)
typedef struct  // программная модель управления БДД
{
	uint8_t mode;
  uint8_t state;
  //
  uint8_t pressure_source;
  uint8_t ims_pwron_flag, ims_pwron_timeout_flag;
  int32_t ims_pwron_timeout_counter;
  //
  uint8_t error, error_cnt;
  uint32_t time_slot_cnter;

}type_BDD_Control;

typedef struct  // программная модель управления БДД
{
	typeSysFrames system;
	typeOAIDDFrames oai_dd;
	typeIMSDDFrames ims_dd;
}type_BDD_Frames;

typedef struct  // программная модель управления БДД
{
  type_ADC_model adc_0;
  type_DAC_model dac;
  type_MVIP mvip;
  type_TRES_model tres_ims, tres_pirani_1, tres_pirani_2;
  type_OAI_DD_model oai_dd_1, oai_dd_2;
  type_IMS_model ims;
  type_MPI_model* mpi_ptr;
  type_SINGLE_GPIO gpio[8], stm_io[10];
  type_STM_BDD stm;
  //
  uint16_t pressure_16;
  uint8_t temp_bdd, current_HV;
  //
  type_BDD_Frames frame;
  //
  type_BDD_Control control;
}type_BDD_model;

// function prototypes

int8_t bdd_init(type_BDD_model* bdd_ptr, type_MPI_model* mpi_ptr);
void bdd_ctrl_reset(type_BDD_model* bdd_ptr);
void bdd_process(type_BDD_model* bdd_ptr, uint8_t period_ms);
void bdd_set_mode(type_BDD_model* bdd_ptr, uint8_t mode);
void bdd_state_check(type_BDD_model* bdd_ptr, uint8_t period_ms);
void bdd_oai_dd_frame_form(type_BDD_model* bdd_ptr);
void bdd_ims_dd_frame_form(type_BDD_model* bdd_ptr);
void bdd_system_frame_form(type_BDD_model* bdd_ptr);

#endif
