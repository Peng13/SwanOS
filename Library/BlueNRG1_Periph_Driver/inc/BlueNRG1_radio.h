/**
  ******************************************************************************
  * @file    BlueNRG1_radio.h
  * @author  RF Application Team
  * @version V1.1.0
  * @date    22-June-2017
  * @brief   This file contains all the functions prototypes for the radio firmware 
  *          library.
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
  * <h2><center>&copy; COPYRIGHT 2017 STMicroelectronics</center></h2>
  ******************************************************************************
  */
  
  /* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BLUENRG1_RADIO_H
#define BLUENRG1_RADIO_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "BlueNRG_x_device.h"


/** @addtogroup BLUENRG1_Peripheral_Driver BLUENRG1 Peripheral Driver
  * @{
  */


/** @addtogroup Radio_Peripheral  Radio Peripheral
  * @{
  */

/** @defgroup Radio_Exported_Constants Exported Constants
* @{
  */

  
#define MAX_PACKET_LENGTH       31
#define RECEIVE_BUFFER_LENGTH   45
#define SUCCESS_0               0
#define INVALID_PARAMETER_C0    0xC0
#define WAKEUP_NOTSET_C2        0xC2
#define RADIO_BUSY_C4           0xC4
#define COMMAND_DISALLOWED      0xC5
#define NULL_0                  0
#define BLUE_IDLE_0             0  
#define BLUE_BUSY_NOWAKEUP_T1   1 
#define BLUE_BUSY_NOWAKEUP_T2   2 
#define BLUE_BUSY_WAKEUP        3 
#define TIMESTAMP_POSITION_ACCESSADDRESS    0x40 
#define TIMESTAMP_POSITION_LASTBIT          0x80 /*C0*/  
#define HOT_ANA_CONFIG_TABLE_LENGTH         64
#define CONFIG_ERROR_EN                     0x10
#define SCA_DEFAULTVALUE                    488


#define ANALOG_HW_OFFSET        (86)  /* subtrack 2*86 in the 1st packet for XC oscillator */  
#define TIMING_ERROR_CORRECTION (69)  /* subtrack 2*69 in all packet except 1st for XC oscillator */

  
/** @defgroup Radio_Interrupt_Status_Bits  Interrupt Status bits
  * @{
  */

#define IRQ_RCV_OK              (1UL<<31)
#define IRQ_CRC_ERR             (1UL<<30)
#define IRQ_RCV_TRIG            (1UL<<29)
#define IRQ_CMD                 (1UL<<28)
#define IRQ_MD                  (1UL<<27)
#define IRQ_TIMEOUT             (1UL<<26)
#define IRQ_RCV_FAIL            (1UL<<25)
#define IRQ_DONE                (1UL<<24)
#define IRQ_ERR_ENC             (1UL<<23)
#define IRQ_TX_OK               (1UL<<22)
#define BIT_TX_SKIP             (1UL<<20)
#define IRQ_CONFIG_ERR          (1UL<<19)
#define BIT_TX_MODE             (1UL<<18)
#define BIT_TIME_OVERRUN        (1UL<<17)
#define BIT_ACT2_ERROR          (1UL<<16)
#define IRQ_WAKEUP_2            (1UL<<15)
#define BIT_AES_READY           (1UL<<12)

/**
  * @}
  */ 
  
  
/** @defgroup Radio_ActionTag_BitMask ActionTag BitMask
* @{
*/

#define PLL_TRIG                    0x01
#define TXRX                        0x02
#define TIMER_WAKEUP                0x04  
#define NS_EN                       0x08
#define INC_CHAN                    0x10
#define RELATIVE                    0x20
#define TIMESTAMP_POSITION          0x80 

/**
  * @}
  */



/**
* @}
*/

/** @defgroup Radio_Exported_Types Exported Types
* @{
*/

typedef struct
{
  volatile uint16_t next;
  volatile uint16_t datptr;
  volatile uint8_t  rcvlen[3];
  volatile uint8_t  timeout[3];
  volatile uint8_t  byte10;
  volatile uint8_t  byte11;
} BlueTransStruct;


typedef struct ActionPacket ActionPacket;

typedef struct {
  uint32_t *hot_ana_config_table;   /**< Set to NULL */
  uint8_t ls_source;                /**< Source for the 32 kHz slow speed clock: 1: internal RO; 0: external crystal */
  uint16_t hs_startup_time ;        /**< Start up time of the high speed (16 or 32 MHz) crystal oscillator in units of 625/256 us (~2.44 us)*/
} config_table_t;


typedef struct {
    int32_t freq;    
    int32_t period;    
    struct {
      uint32_t  Time_mT        : 24;
      uint32_t  started_flag   :  8;
    } calibr;    
} Clk32Context_t;

typedef struct {
    uint32_t back2backTime;
    uint8_t forceRadiotoStop;
    uint32_t rssiLevel[2];
    int32_t wakeupTime;
    uint16_t period_slow_patch;
    int32_t freq_global_debug;
    int32_t period_global_debug;
    int32_t hot_ana_config_table_a[HOT_ANA_CONFIG_TABLE_LENGTH>>2];
    Clk32Context_t Clk32Context;
    config_table_t hardware_config;
   // uint8_t tone_start_stop_flag;
    ActionPacket* current_action_packet;    
    uint8_t powerUpfirstPacket;

}RadioGlobalParameters_t;

extern RadioGlobalParameters_t globalParameters;


struct ActionPacket
{
  uint8_t StateMachineNo ;      /* State machine number */
  uint8_t ActionTag;            /* Action Tag: PLL_TRIG, TXRX, TIMER_WAKEUP, NS_EN, 
                                 * INC_CHAN, TIMESTAMP_POSITION, RELATIVE */
  uint32_t WakeupTime;          /* Contains the wakeup time in microsecond if it is relative.
                                 * It should not be more than 24 bits if it is absolute. 
                                 * Will only apply if TIMER_WAKEUP flag is set in ActionTag. */
  uint32_t ReceiveWindowLength; /* Sets listening window size based on microsecond. RX only */
  uint8_t *data;                /* Pointer to payload data (receive or transmit). Defined by user */
  uint32_t status;              /* The Status Register. Should be used in callback routines. */
  uint32_t timestamp_receive;   /* This field contains the timestamp when it was received. 
                                 * Intended to be used in the userDataHandler() callback routine. RX only. */
  int32_t rssi;                 /* The rssi of the packet was received with. RX only. */
  BlueTransStruct trans_packet;
  ActionPacket *next_true;      /* Pointer to next ActionPacket if condRoutine() returns TRUE */
  ActionPacket *next_false;     /* Pointer to next ActionPacket if condRoutine() returns FALSE */
  uint8_t (*condRoutine)(ActionPacket*);        /* User callback that decide the next ActionPacket to use.
                                                 * It is time critical. Routine must end within 45 us. */
  uint8_t (*dataRoutine)(ActionPacket*, ActionPacket*); /* User callback for managing data. */
  uint8_t trans_config;
};

/**
* @}
*/


/** @defgroup Radio_Exported_Macros Exported Macros
  * @{
  */

/**
  * @}
  */

/** @defgroup Radio_Private_Functions Private Functions 
* @{
*/
  
/**
  * @}
  */


/** @defgroup Radio_Exported_Functions Exported Functions
  * @{
  */

void RADIO_Init(uint16_t hs_startup_time, uint8_t low_speed_osc, uint32_t* hot_table, FunctionalState whitening);
uint8_t RADIO_GetStatus(uint32_t *time);
void RADIO_SetChannelMap(uint8_t StateMachineNo,uint8_t *chan_remap);
void RADIO_SetChannel(uint8_t StateMachineNo, uint8_t channel,uint8_t channel_increment); 
void RADIO_SetTxAttributes(uint8_t StateMachineNo, uint32_t NetworkID, uint32_t crc_init, uint32_t sca);
void RADIO_SetBackToBackTime(uint32_t back_to_back_time);  
void RADIO_SetTxPower(uint8_t PowerLevel);    
void RADIO_IRQHandler(void);
uint8_t RADIO_StopActivity(void);
void RADIO_SetReservedArea(ActionPacket *p); 
uint8_t RADIO_MakeActionPacketPending(ActionPacket *p);
void RADIO_CrystalCheck(void);
void RADIO_StartTone(uint8_t RF_channel, uint8_t powerLevel);
void RADIO_StopTone(void);
void RADIO_SetEncryptionCount(uint8_t StateMachineNo, uint8_t *count_tx, uint8_t *count_rcv);    
void RADIO_SetEncryptionAttributes(uint8_t StateMachineNo, uint8_t *enc_iv, uint8_t *enc_key); 
void RADIO_SetEncryptFlags(uint8_t StateMachineNo, FunctionalState EncryptFlagTx, FunctionalState EncryptFlagRcv);
void RADIO_EncryptPlainData(uint8_t *Key, uint8_t *plainData, uint8_t *cypherData);

/**
  * @}
  */

/**
  * @}
  */ 

/**
  * @}
  */ 

#ifdef __cplusplus
}
#endif

#endif /*BLUENRG1_RADIO_H */

/******************* (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/
