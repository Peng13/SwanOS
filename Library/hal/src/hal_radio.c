/**
******************************************************************************
* @file    hal_radio.c
* @author  AMG - RF Application team
* @version V1.0.0
* @date    17-July-2017
* @brief   BlueNRG-1,2 HAL radio APIs 
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
#include "hal_radio.h"

static ActionPacket aPacket[2]; 
static uint32_t accessAddress = 0x88888888;

static uint8_t nullfunction(ActionPacket* p)
{
  return TRUE;
}

static uint8_t dataRoutine_tx(ActionPacket* current_action_packet, ActionPacket* next)
{
  return TRUE;
}

static uint8_t condRoutine_rx(ActionPacket* p)
{
  uint8_t retValue;
  retValue = FALSE;
  if( (p->status & IRQ_DONE) != 0)
  { 
    /* received a packet */
    if( (p->status & BIT_TX_MODE) == 0) {
      if((p->status & IRQ_RCV_OK) != 0) {
        /* packet received without CRC error */ 
        retValue = TRUE;                
      }
      else if((p->status & IRQ_TIMEOUT) != 0) {
        /* packet not received in specified time. timeout error */ 
      }           
      else if((p->status & IRQ_CRC_ERR) != 0) {
        /* packet received with CRC error */  
      }            
    } 
  }

  return retValue; 
} 

/**
* @brief  This routine sets the network ID field for packet transmission and filtering for the receiving.
*         Only two devices with same networkID can communicate with each other.
* @param  ID: network ID based on bluetooth specification:
*           1. It shall have no more than six consecutive zeros or ones.
*           2. It shall not have all four octets equal.
*           3. It shall have no more than 24 transitions.
*           4. It shall have a minimum of two transitions in the most significant six bits.
*
* @retval uint8_t: return value
*           - 0x00 : Success.
*           - 0xC0 : Invalid parameter.
*/
uint8_t HAL_RADIO_SetNetworkID(uint32_t ID)
{
  accessAddress = ID;
  return 0;
}


/**
* @brief  This routine sends a packet on a specific channel and at a specific time.
* @param  channel: Frequency channel between 0 to 39.
* @param  time: Time of transmission in us. This is relative time regarding now.
*         The relative time cannot be less than 100 us otherwise it might wrap around after 32.7 second.
* @param  time_type: Determines time is relative or absolute: 0 means absolute, 1 means relative.
* @param  txBuffer: Pointer to TX data buffer. Second byte of this buffer must be the length of the data.
* @param  Callback: This function is being called as data routine.
*         First ActionPacket is current action packet and the second one is next action packet.
* @retval uint8_t return value
*           - 0x00 : Success.
*           - 0xC0 : Invalid parameter.
*           - 0xC4 : Radio is busy, receiving has not been triggered.
*/
uint8_t HAL_RADIO_SendPacket(uint8_t channel, 
                             uint32_t time, 
                             uint8_t time_type, 
                             uint8_t* txBuffer, 
                             uint8_t (*Callback)(ActionPacket*, ActionPacket*) )
{
  uint8_t returnValue = SUCCESS_0;
  uint32_t dummy;
  
  if(channel > 39) {
    returnValue = INVALID_PARAMETER_C0;
  }
  
  if(time_type > 1) {
    returnValue = INVALID_PARAMETER_C0;
  }
  
  if(RADIO_GetStatus(&dummy) != BLUE_IDLE_0) {
    returnValue = RADIO_BUSY_C4;
  }
  
  if(returnValue == SUCCESS_0) {
    uint8_t map[5]= {0xFF,0xFF,0xFF,0xFF,0xFF};
    RADIO_SetChannelMap(0, &map[0]);
    RADIO_SetChannel(0, channel, 0);
    RADIO_SetTxAttributes(0, accessAddress, 0x555555, SCA_DEFAULTVALUE);   
      
    aPacket[0].StateMachineNo = STATE_MACHINE_NO;
    aPacket[0].ActionTag =  TXRX | PLL_TRIG | TIMER_WAKEUP;
    aPacket[0].WakeupTime = time;
    aPacket[0].ReceiveWindowLength = 0; /* does not affect for Tx */
    aPacket[0].data = txBuffer;
    aPacket[0].next_true = NULL_0;
    aPacket[0].next_false = NULL_0;
    aPacket[0].condRoutine = nullfunction;
    aPacket[0].dataRoutine = Callback;
    
    if(time_type == 1) {
      aPacket[0].ActionTag |= RELATIVE;
    }    
    RADIO_SetReservedArea(&aPacket[0]); 
    returnValue = RADIO_MakeActionPacketPending(&aPacket[0]);
  }
  
  return returnValue; 
}


/**
* @brief  This routine sends a packet on a specific channel and at a certain time then wait for receiving acknowledge.
* @param  channel: Frequency channel between 0 to 39.
* @param  time: Time of transmission based on us. This is relative time regarding now.
*         The relative time cannot be less than 100 us otherwise it might wrap around after 32.7 second.
* @param  time_type: Determines time is relative or absolute. 0 means absolute, 1 means relative.
* @param  txBuffer: Pointer to TX data buffer. Secound byte of this buffer must be the length of the data.
* @param  rxBuffer: Pointer to RX data buffer. Secound byte of this buffer must be the length of the data.
* @param  receive_timeout: Time of RX window used to wait for the packet on us.
* @param  callback: This function is being called as data routine.
*         First ActionPacket is current action packet and the second one is next action packet.
* @retval uint8_t return value
*           - 0x00 : Success.
*           - 0xC0 : Invalid parameter.
*           - 0xC4 : Radio is busy, receiving has not been triggered.
*/
uint8_t HAL_RADIO_SendPacketWithAck(uint8_t channel, 
                                    uint32_t time, 
                                    uint8_t time_type, 
                                    uint8_t* txBuffer, 
                                    uint8_t* rxBuffer, 
                                    uint32_t receive_timeout,
                                    uint8_t (*Callback)(ActionPacket*, ActionPacket*) )
{
  uint8_t returnValue = SUCCESS_0;
  uint32_t dummy;
  
  if(channel > 39) {
    returnValue = INVALID_PARAMETER_C0;      
  }
  
  if(time_type > 1) {
    returnValue = INVALID_PARAMETER_C0;      
  }
  if(RADIO_GetStatus(&dummy) != BLUE_IDLE_0) {
    returnValue = RADIO_BUSY_C4;
  }
  
  if(returnValue == SUCCESS_0) {
    uint8_t map[5]= {0xFF,0xFF,0xFF,0xFF,0xFF};
    RADIO_SetChannelMap(0, &map[0]);
    RADIO_SetChannel(0, channel, 0);
    
    RADIO_SetTxAttributes(0, accessAddress, 0x555555, SCA_DEFAULTVALUE);   

    aPacket[0].StateMachineNo = STATE_MACHINE_NO;   
    aPacket[0].ActionTag =  TXRX | PLL_TRIG | TIMER_WAKEUP;  
    aPacket[0].WakeupTime = time;
    aPacket[0].ReceiveWindowLength = 0; /* does not affect for Tx */
    aPacket[0].data = txBuffer;
    aPacket[0].next_true = &aPacket[1];
    aPacket[0].next_false = &aPacket[1];   
    aPacket[0].condRoutine = nullfunction;
    aPacket[0].dataRoutine = dataRoutine_tx;
    
    if(time_type == 1) {
      aPacket[0].ActionTag |= RELATIVE;
    }

    aPacket[1].StateMachineNo = STATE_MACHINE_NO;   
    aPacket[1].ActionTag =  0;   
    aPacket[1].WakeupTime = time;
    aPacket[1].ReceiveWindowLength = receive_timeout; 
    aPacket[1].data = rxBuffer; 
    aPacket[1].next_true = NULL_0;
    aPacket[1].next_false = NULL_0;    
    aPacket[1].condRoutine = nullfunction;
    aPacket[1].dataRoutine = Callback;
    
    RADIO_SetReservedArea(&aPacket[0]);
    RADIO_SetReservedArea(&aPacket[1]);
    returnValue = RADIO_MakeActionPacketPending(&aPacket[0]);    
  }
  
  return returnValue; 
}


/**
* @brief  This routine receives a packet on a specific channel and at a certain time.
* @param  channel: Frequency channel between 0 to 39.
* @param  time: Time of transmission based on us. This is relative time regarding now.
*         The relative time cannot be less than 100 us otherwise it might wrap around after 32.7 second.
* @param  time_type: Determines time is relative or absolute. 0 means absolute, 1 means relative.
* @param  rxBuffer: Pointer to RX data buffer. Secound byte of this buffer must be the length of the data.
* @param  receive_timeout: Time of RX window used to wait for the packet on us.
* @param  timestamp_type: Determines where timestamp is taken: after network_id or last bit. 0: after last bit, 1: after network_id.
* @param  callback: This function is being called as data routine.
*         First ActionPacket is current action packet and the second one is next action packet.
* @retval uint8_t return value
*           - 0x00 : Success.
*           - 0xC0 : Invalid parameter.
*           - 0xC4 : Radio is busy, receiving has not been triggered.
*/
uint8_t HAL_RADIO_ReceivePacket(uint8_t channel, 
                                uint32_t time, 
                                uint8_t time_type,
                                uint8_t* rxBuffer,                       
                                uint32_t receive_timeout, 
                                uint8_t timestamp_type,
                                uint8_t (*Callback)(ActionPacket*, ActionPacket*) )
{
  uint8_t returnValue = SUCCESS_0;
  uint32_t dummy;
  
  if(channel > 39) {
    returnValue = INVALID_PARAMETER_C0;
  }
  
  if(time_type > 1) {
    returnValue = INVALID_PARAMETER_C0;
  }
  
  if(RADIO_GetStatus(&dummy) != BLUE_IDLE_0) {
    returnValue = RADIO_BUSY_C4;
  }
  
  if(returnValue == SUCCESS_0) {
    uint8_t map[5]= {0xFF,0xFF,0xFF,0xFF,0xFF};
    RADIO_SetChannelMap(0, &map[0]);
    RADIO_SetChannel(0, channel, 0);
    
    RADIO_SetTxAttributes(0, accessAddress, 0x555555, SCA_DEFAULTVALUE);
    
    aPacket[0].ActionTag = 0;    
    aPacket[0].StateMachineNo = STATE_MACHINE_NO;
    aPacket[0].ActionTag =  PLL_TRIG | TIMER_WAKEUP;
    aPacket[0].WakeupTime = time;
    aPacket[0].ReceiveWindowLength = receive_timeout;
    aPacket[0].data = rxBuffer;
    aPacket[0].next_true = NULL_0;
    aPacket[0].next_false = NULL_0;
    aPacket[0].condRoutine = nullfunction;
    aPacket[0].dataRoutine = Callback;
    
    if(timestamp_type == 1) /* time stamp after network address */
    {
      aPacket[0].ActionTag |= TIMESTAMP_POSITION;
    }
    
    if(time_type == 1) {
      aPacket[0].ActionTag |= RELATIVE;
    }
    RADIO_SetReservedArea(&aPacket[0]);
    returnValue = RADIO_MakeActionPacketPending(&aPacket[0]);
  }
  
  return returnValue;
}


/**
* @brief  This routine receives a packet on a specific channel and at a certain time.
*         Then sends a packet as an acknowledgment.
* @param  channel: frequency channel between 0 to 39.
* @param  time: time of transmission based on us. This is relative time regarding now.
*         The relative time cannot be less than 100 us otherwise it might wrap around after 32.7 second.
* @param  time_type: determines time is relative or absolute. 0 means absolute, 1 means relative.
* @param  rxBuffer: points to received data buffer. second byte of this buffer determines the length of the data.
* @param  txBuffer: points to data buffer to send. secound byte of this buffer must be the length of the buffer.
* @param  receive_timeout: Time of RX window used to wait for the packet on us.
* @param  timestamp_type: Determines where timestamp is taken: after network_id or last bit. 0: after last bit, 1: after network_id.
* @param  callback: This function is being called as data routine.
*         First ActionPacket is current action packet and the second one is next action packet.
* @retval uint8_t return value
*           - 0x00 : Success.
*           - 0xC0 : Invalid parameter.
*           - 0xC4 : Radio is busy, receiving has not been triggered.
*/
uint8_t HAL_RADIO_ReceivePacketWithAck(uint8_t channel, 
                                       uint32_t time,
                                       uint8_t time_type, 
                                       uint8_t* rxBuffer, 
                                       uint8_t* txBuffer,
                                       uint32_t receive_timeout, 
                                       uint8_t timestamp_type,
                                       uint8_t (*Callback)(ActionPacket*, ActionPacket*) )
{
  uint8_t returnValue = SUCCESS_0;
  uint32_t dummy;
  
  if(channel > 39) {
    returnValue = INVALID_PARAMETER_C0;      
  }
  
  if(time_type > 1) {
    returnValue = INVALID_PARAMETER_C0;      
  }
  
  if(RADIO_GetStatus(&dummy) != BLUE_IDLE_0) {
    returnValue = RADIO_BUSY_C4;
  }
    
  if(returnValue == SUCCESS_0) {
    uint8_t map[5]= {0xFF,0xFF,0xFF,0xFF,0xFF};
    RADIO_SetChannelMap(0, &map[0]);
    RADIO_SetChannel(0,channel,0);
    RADIO_SetTxAttributes(0, accessAddress,0x555555,SCA_DEFAULTVALUE);   
    
    aPacket[0].StateMachineNo = STATE_MACHINE_NO;
    aPacket[0].ActionTag =  PLL_TRIG | TIMER_WAKEUP;
    aPacket[0].WakeupTime = time;
    aPacket[0].ReceiveWindowLength = receive_timeout;
    aPacket[0].data = rxBuffer;
    aPacket[0].next_true = &aPacket[1];
    aPacket[0].next_false = NULL_0;
    aPacket[0].condRoutine = condRoutine_rx;
    aPacket[0].dataRoutine = Callback;
    
    /* time stamp after network address */
    if(timestamp_type == 1) {
      aPacket[0].ActionTag |= TIMESTAMP_POSITION;
    }
    
    aPacket[1].StateMachineNo = STATE_MACHINE_NO;
    aPacket[1].ActionTag =  TXRX;
    aPacket[1].WakeupTime = time;
    aPacket[1].ReceiveWindowLength = 0; /* does not affect for Tx */
    aPacket[1].data = txBuffer;
    aPacket[1].next_true = NULL_0;
    aPacket[1].next_false = NULL_0;
    aPacket[1].condRoutine = nullfunction;
    aPacket[1].dataRoutine = Callback;
    
    if(time_type == 1) {
      aPacket[0].ActionTag |= RELATIVE ;
    }
    RADIO_SetReservedArea(&aPacket[0]); 
    RADIO_SetReservedArea(&aPacket[1]); 
    returnValue = RADIO_MakeActionPacketPending(&aPacket[0]);
  }
  
  return returnValue; 
}


/******************* (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/
