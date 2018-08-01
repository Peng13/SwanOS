/**
******************************************************************************
* @file    GPIO/InputInterrupt/BlueNRG1_it.c 
* @author  RF Application Team
* @version V1.1.0
* @date    September-2015
* @brief   Main Interrupt Service Routines.
*          This file provides template for all exceptions handler and
*          peripherals interrupt service routine.
******************************************************************************
* @attention
*
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
* TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
* DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
* FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
* CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
* <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "BlueNRG1_it.h"
#include "BlueNRG1_conf.h"
#include "SDK_EVAL_Config.h"


/** @addtogroup BlueNRG1_StdPeriph_Examples
* @{
*/

/** @addtogroup GPIO_Examples
* @{
*/ 

/** @addtogroup GPIO_InputInterrupt
* @{
*/ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define NO_BOUNCE_BUTTON_2            20

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile uint8_t check_push2 = 0;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M0 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
* @brief  This function handles NMI exception.
*/
void NMI_Handler(void)
{
  while(1);
}

/**
* @brief  This function handles Hard Fault exception.
*/
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1);
}


/**
* @brief  This function handles SVCall exception.
*/
void SVC_Handler(void)
{
}


/**
* @brief  This function handles PendSV exception.
*/
/* Modified by tianpeng for SwanOS switch context at 20180801 begin */
typedef void (*pFUNC)(void*);
typedef struct task_control_block
{
	uint32_t *p_stk;
	uint32_t delay_cnt;
	uint8_t  state; /* 0:sleep, 1: delay, 2: ready, 3: running */
	uint8_t  id;
	uint32_t *next;
} TCB, *pTCB;
extern pTCB os_cur_task, os_next_task;

__asm void PendSV_Handler(void)
{
	IMPORT os_cur_task
	IMPORT os_next_task
	LDR  R3, =os_cur_task
	LDR  R1, [R3]
	LDR  R2, =os_next_task
	LDR  R2, [R2]
	
	CMP  R1, R2
	BEQ  exitPendSV						;/* if os_cur_task == os_next_task, don't need to schedule, exit. */
	MRS  R0, PSP
	
	SUBS R0, R0, #32
	STR  R0, [R1]
	
	STMIA R0!, {R4-R7}
	MOV   R4, R8
	MOV   R5, R9
	MOV   R6, R10
	MOV   R7, R11
	STMIA R0!, {R4-R7}
	
popStk
	STR R2, [R3]							;/* set os_cur_task = os_next_task */
	LDR R0, [R2]
	
	ADDS  R0, R0, #16
	LDMIA R0!, {R4-R7}
	MOV   R8, R4
	MOV   R9, R5
	MOV   R10, R6
	MOV   R11, R7
	SUBS  R0, R0, #32
	LDMIA R0!, {R4-R7}
	ADDS  R0, R0, #16
	MSR   PSP, R0							;/* Set PSP reg point to new task stack. */
	
exitPendSV
	MOVS R0, #4								;/* Ensure exception return to thread mode and use PSP. */
	RSBS R0, #0
	BX  R0
	ALIGN
}
/* Modified by tianpeng for SwanOS switch context at 20180801 end */

/**
* @brief  This function handles SysTick Handler.
*/
/* Modified by tianpeng for SwanOS generate systicks every 10ms and schedule tasks at 20180801 begin */
extern uint32_t os_tick_cnt;
extern uint8_t os_poll_tsk_state(void);
extern void os_append_task(pTCB *list, pTCB task);
extern void os_update_task_delay(void);
extern void os_poll_task(void);
extern void os_schedule(void);
extern TCB os_tsk_tbl[3];
extern pTCB os_ready_task_header;
extern pTCB os_delay_task_header;
void SysTick_Handler(void)
{
	os_tick_cnt++;
	if (os_cur_task->id != 0) {
		os_append_task(&os_ready_task_header, &os_tsk_tbl[os_cur_task->id]);
	}
	os_update_task_delay();
	os_poll_task();
	
	os_schedule();
}
/* Modified by tianpeng for SwanOS generate systicks every 10ms and schedule tasks at 20180801 end */
/******************************************************************************/
/*                 BlueNRG-1 Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (system_bluenrg1.c).                                               */
/******************************************************************************/

/**
* @brief  This function handles GPIO interrupt request.
* @param  None
* @retval None
*/
/* Modified by tianpeng for SwanOS mark button event at 20180801 begin */
extern TCB os_tsk_tbl[3];
void GPIO_Handler(void)
{
	extern uint8_t btn1_press_flag, btn2_press_flag;
  /* If BUTTON_1 is pressed LED1 is ON */
  if(GPIO_GetITPendingBit(Get_ButtonGpioPin(BUTTON_1)) == SET) {
    GPIO_ClearITPendingBit(Get_ButtonGpioPin(BUTTON_1));
    btn1_press_flag = 1;
  }

  /* If BUTTON_2 is pressed LED2 is toggled */
  if(GPIO_GetITPendingBit(Get_ButtonGpioPin(BUTTON_2)) == SET) {
    GPIO_ClearITPendingBit(Get_ButtonGpioPin(BUTTON_2));
    btn2_press_flag = 1;
  }
}
/* Modified by tianpeng for SwanOS mark button event at 20180801 end */
/**
* @}
*/ 

/**
* @}
*/ 

/**
* @}
*/ 

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
