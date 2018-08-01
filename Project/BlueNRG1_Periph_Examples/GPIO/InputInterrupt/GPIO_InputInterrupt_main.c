
/******************** (C) COPYRIGHT 2015 STMicroelectronics ********************
* File Name          : GPIO/InputInterrupt/main.c 
* Author             : RF Application Team
* Version            : V1.1.0
* Date               : September-2015
* Description        : Code demostrating the GPIO interrupt functionality
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/


/* Includes ------------------------------------------------------------------*/
#include "BlueNRG_x_device.h"
#include "BlueNRG1_conf.h"
#include "SDK_EVAL_Config.h"
#include <stdio.h>

/* button flags */
uint8_t btn1_press_flag = 0;
uint8_t btn2_press_flag = 0;

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void LED_Init()
{
  /* Enable the GPIO Clock */
  SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_GPIO, ENABLE);
  
  /* Configure the LEDs */
  GPIO_InitType GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = Get_LedGpioPin(LED1) | Get_LedGpioPin(LED2) | Get_LedGpioPin(LED3);
  GPIO_InitStructure.GPIO_Mode = GPIO_Output;
  GPIO_InitStructure.GPIO_Pull = ENABLE;
  GPIO_InitStructure.GPIO_HighPwr = ENABLE;
  GPIO_Init(&GPIO_InitStructure);

  /* Put the LEDs off */
  GPIO_WriteBit(Get_LedGpioPin(LED1) | Get_LedGpioPin(LED2) | Get_LedGpioPin(LED3), Bit_RESET);
}

void Key_Init()
{
	/* Enable the GPIO Clock */
  SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_GPIO, ENABLE);

	/* Configure the push buttons */
  GPIO_InitType GPIO_InitStructure; 
  GPIO_InitStructure.GPIO_Pin = Get_ButtonGpioPin(BUTTON_1) | Get_ButtonGpioPin(BUTTON_2);
  GPIO_InitStructure.GPIO_Mode = GPIO_Input;
  GPIO_InitStructure.GPIO_Pull = DISABLE;
  GPIO_InitStructure.GPIO_HighPwr = DISABLE;
  GPIO_Init(&GPIO_InitStructure);
  
  /* Set the GPIO interrupt priority and enable it */
  NVIC_InitType NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = GPIO_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = LOW_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  /* Configures EXTI line for BUTTON_1 */
  GPIO_EXTIConfigType GPIO_EXTIStructure;
  GPIO_EXTIStructure.GPIO_Pin = Get_ButtonGpioPin(BUTTON_1);
  GPIO_EXTIStructure.GPIO_IrqSense = GPIO_IrqSense_Edge;
  GPIO_EXTIStructure.GPIO_Event = IRQ_ON_RISING_EDGE;
  GPIO_EXTIConfig(&GPIO_EXTIStructure);

  /* Configures EXTI line for BUTTON_2 */
  GPIO_EXTIStructure.GPIO_Pin = Get_ButtonGpioPin(BUTTON_2);
  GPIO_EXTIStructure.GPIO_Event = IRQ_ON_RISING_EDGE;
  GPIO_EXTIConfig(&GPIO_EXTIStructure);
  
  /* Clear pending interrupt */
  GPIO_ClearITPendingBit(Get_ButtonGpioPin(BUTTON_1) | Get_ButtonGpioPin(BUTTON_2));
  
  /* Enable the interrupt */
  GPIO_EXTICmd(Get_ButtonGpioPin(BUTTON_1) | Get_ButtonGpioPin(BUTTON_2), ENABLE);
}

/* Added by tianpeng for SwanOS this code block is the core of OS at 20180801 begin */
/*******************************************************************/
/******************** I will design an os here. ********************/
/*******************************************************************/
#define OS_TICK_FREQ 100
#define OS_MAX_TASK  3
#define OS_STACK_DEEP 128

#define OS_TASK_SLEEP   0
#define OS_TASK_DELAY   1
#define OS_TASK_READY   2
#define OS_TASK_RUNNING 3

typedef void (*pFUNC)(void*);
/* This struct is Task control block, every task has a TCB. */
typedef struct task_control_block
{
	uint32_t *p_stk;
	uint32_t delay_cnt;
	uint8_t  state; /* 0:sleep, 1: delay, 2: ready, 3: running */
	uint8_t  id;
	uint32_t *next;
} TCB, *pTCB;

/* Defined these two headers to point two list: ready task list and delay task list. */
pTCB os_ready_task_header = NULL;
pTCB os_delay_task_header = NULL;



uint32_t os_tick_cnt;
uint32_t os_tsk_stack[OS_MAX_TASK][OS_STACK_DEEP] = {{0}};
TCB os_tsk_tbl[OS_MAX_TASK] = {{0}};
uint32_t os_cur_task_id = 0;
uint8_t os_task_total = 0;

pTCB os_cur_task, os_next_task;


void os_append_task(pTCB *list, pTCB task)
{
	pTCB ptmp;
	
	if (*list == NULL) {
		*list = task;
	} else {
		ptmp = *list;
		while (ptmp->next != NULL) {
			ptmp = (pTCB)ptmp->next;
		}
		ptmp->next = (uint32_t *)task;
	}
	task->next = NULL;
}

void os_remove_task(pTCB *list, pTCB task)
{
	/* to be defined */
	pTCB ptmp = *list;

	if ((*list)->id == task->id) {
		(*list) = (pTCB)(*list)->next;
	} else {
		while (((pTCB)ptmp->next)->id != task->id) {
			ptmp = (pTCB)ptmp->next;
		}
	
		ptmp->next = ((pTCB)ptmp->next)->next;
	}
}

void os_task_init(pFUNC func)
{
	/* I will init task control block and init task stack here. */
	uint32_t *context = &os_tsk_stack[os_task_total][OS_STACK_DEEP-1];
	*(context--) = (uint32_t)0x01000000; /* xPSR */
	*(context--) = (uint32_t)func;       /* PC */
	*(context)   = (uint32_t)0xfffffffe; /* LR */
	context -= 5;
	/* Now context point to R0, we can put the argument to R0 if we have */
	context -= 8; /* Put this pointer to the end of the stack, I meant at the R4 location. */
	
	os_tsk_tbl[os_task_total].p_stk = context;
	os_tsk_tbl[os_task_total].delay_cnt = 0;
	os_tsk_tbl[os_task_total].state = OS_TASK_READY;
	os_tsk_tbl[os_task_total].id = os_task_total;

	if (os_task_total != 0) {		/* I don't want to add Idel task to my ready list. */
		os_append_task(&os_ready_task_header, &os_tsk_tbl[os_task_total]);
	}
	os_task_total++;
}


void os_poll_task(void)
{
	if (os_ready_task_header != NULL) {
		os_next_task = os_ready_task_header;
		os_remove_task(&os_ready_task_header, &os_tsk_tbl[os_ready_task_header->id]);
	} else {
		os_next_task = &os_tsk_tbl[0];
	}
}

/* This function is used to update the delay task's delay value. */
void os_update_task_delay(void)
{
	pTCB ptmp = os_delay_task_header;
	while (ptmp != NULL) {
		ptmp->delay_cnt--;
		if (ptmp->delay_cnt <= 0) {
			os_remove_task(&os_delay_task_header, &os_tsk_tbl[ptmp->id]);
			os_append_task(&os_ready_task_header, &os_tsk_tbl[ptmp->id]);
		}
		ptmp = (pTCB)ptmp->next;
	}
}


__asm void os_schedule(void)
{
  ldr r0, =0xe000ed04 ;/* Trigger a PendSV exception. */
	ldr r1, =0x10000000
	str r1, [r0]
	bx  lr
}


/* this function used to sleep current task for msleep, system will switch to other task until time is up */
void os_msleep(uint32_t msleep)
{
	/* Set current task to delay state, and trigger a os schedule. */
	os_cur_task->delay_cnt = msleep/10;
	os_cur_task->state = OS_TASK_DELAY;

	os_append_task(&os_delay_task_header, os_cur_task);
	
	os_poll_task();
  os_schedule();

}

__asm  void os_init_psp(uint32_t *pstk)
{
    SUBS    R0,#28
    MSR     PSP, R0             ;/* Mov new stack point to PSP */
    BX      LR	
    ALIGN
}

void os_start(void)
{
	os_cur_task = &os_tsk_tbl[0];
	os_next_task = &os_tsk_tbl[0];
	// os_next_task->state = OS_TASK_RUNNING;
	os_init_psp(&os_tsk_stack[0][OS_STACK_DEEP-1]);
	
	os_schedule();
}

void Key_Task()
{
	printf("[tianpeng] Key Task is up.\r\n");
	while(1)
	{
	  if (btn1_press_flag)
	  {
	  	btn1_press_flag = 0;
	  	SdkEvalLedToggle(LED1);
	  }
	  if (btn2_press_flag)
	  {
	  	btn2_press_flag = 0;
	  	SdkEvalLedToggle(LED2);
	  }
		printf("[tianpeng] this is Key task.\r\n");
		os_msleep(600);
  }
}

void Print_Task()
{
	printf("[tianpeng] Print Task is up.\r\n");
	while(1)
	{
		printf("[tianpeng] this is Print_Task.\r\n");
		os_msleep(1000);
	}
}

void Idle_Task()
{
	printf("[tianpeng] Idle Task is up.\r\n");
	while(1) {}
}

void os_init()
{
	/* set os tick time as 10ms                                      */
	/* And we will check for delay_list and ready_list for schedule. */
	SysTick_Config(SYST_CLOCK/OS_TICK_FREQ);
	os_tick_cnt = 0;
	os_task_init((pFUNC)Idle_Task);
}

/**
  * @brief  Main program code.
  * @param  None
  * @retval None
  */
int main(void)
{
  /* System initialization function */
  SystemInit();
  
  /* Identify BlueNRG1 platform */
  SdkEvalIdentification();
  SdkEvalComUartInit(UART_BAUDRATE);
	printf("[tianpeng] Uart init successful.\r\n");
  LED_Init();
	Key_Init();

	os_init();
	
	os_task_init((pFUNC)Key_Task);
	os_task_init((pFUNC)Print_Task);
	
  /* Infinite loop */
	/* But now, we have a operating system, so we don't need to while at here. */
	os_start();
}
/* Added by tianpeng for SwanOS this code block is the core of OS at 20180801 end */


#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}

#endif

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
