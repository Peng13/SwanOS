/******************** (C) COPYRIGHT 2017 STMicroelectronics ********************
* File Name          : system_bluenrg1.c
* Author             : AMG - RF Application team
* Version            : V1.0.0
* Date               : 27-November-2017
* Description        : BlueNRG-1,BlueNRG-2 Low Level Init function
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/
/**
 * @file  system_bluenrg1.c
 * @brief BlueNRG-1,2 Low Level Init functions
 * <!-- Copyright 2017 by STMicroelectronics.  All rights reserved.       *80*-->

* \section SystemInit  System Initialization

  - The SystemInit() function is called on BlueNRG-1, BlueNRG-2 main application as first operation required for properly initialize the device:
    - It remaps the vector table and it configures all the device interrrupt priorities giving the 
       highest one to the Pendable request for system service  and to the BLE radio BLUE Controller ones.
      - It calls the DeviceConfiguration(TRUE,TRUE) function for performing the proper cold start configuration steps (if it system performs a cold start reset/power-on).
      - It disables all the  peripherals clock except NVM, SYSCTR, PKA and RNG
         - It's up to user application to enables the other peripherals clock according to his needs.
      - It clear PRIMASK to reenable global interrupt operations.

* \section DeviceConfiguration_Cold  Cold Start Configuration 

  - During the device initialization phase, after BlueNRG-1, BlueNRG-2 device reset/power-on (cold start), some specific parameters must be set 
    on BLE device controller registers, in order to define the following configurations:
    - Application mode: user or test mode
    - High speed crystal configuration: 32 or 16 MHz
    - Low speed crystal source: external 32 kHz oscillator or internal RO
    - SMPS: on or off (if on: 4.7 µH or 10 µH SMPS inductor)
    - BOR configuration
  - The BlueNRG-1, BlueNRG-2 controller registers values are defined on file system_bluenrg1.c through the cold start configuration table: 
    - COLD_START_CONFIGURATION.
    - This table defines the default configurations as follows:
      - User mode: ATB0_ANA_ENG_REG = USER_MODE_ATB0 (0x00), ATB1_ANA_ENG_REG = USER_MODE_ATB (0x30)
      - SMPS ON, 10 uH inductor: CLOCK_LOW_ENG_REG = SMPS_ON, RM1_DIG_ENG_REG = SMPS_10uH_RM1
      - 16 MHz high speed crystal: CLOCK_HIGH_ENG_REG = HIGH_FREQ_16M
      - External 32 kHz oscillator: CLOCK_ANA_USER_REG = LOW_FREQ_XO
      - BOR (brown-out threshold): disabled by default
    - At device initialization, on reset/power-on, the function SystemInit() sets the
      default cold start parameters defined on the COLD_START_CONFIGURATION table within
      the cold_start_config[] array by calling the  DeviceConfiguration(BOOL coldStart, BOOL waitLS_Ready) function with coldStart = TRUE and waitLS_Ready = TRUE. 
      User application can define its specific cold start settings, based on its application scenario, by setting some preprocessor options which act on specific fields of the cold_start_config[] array:
      - HS_SPEED_XTAL (high speed cystal: HS_SPEED_XTAL_16MHZ or HS_SPEED_XTAL_32MHZ)
      - LS_SOURCE     (Low speed crystal source: LS_SOURCE_EXTERNAL_32kHZ or LS_SOURCE_INTERNAL_RO)
      - SMPS_INDUCTOR (SPMS inductor disabled: SMPS_INDUCTOR_NONE or SPMS inductor enabled with 10uH: SMPS_INDUCTOR_10uH or SPMS inductor enabled with 4.7uH: SMPS_INDUCTOR_4_7uH)
      - BOR_CONFIG    (BOR enabled: BOR_ON or BOR disabled: BOR_OFF)

    - Regarding the ATB0_ANA_ENG_REG, ATB1_ANA_ENG_REG registers settings, some test modes are also available in order to address some test scenarios.
      User should sets such registers as follows:
  
        - Low speed crystal oscillator test mode:
        
            - cold_start_config[2] = LS_XTAL_MEAS_ATB0 (0x37)
            - cold_start_config[5] = LS_XTAL_MEAS_ATB1 (0x34)
      
        -  High speed start-up time test mode:  
      
            - cold_start_config[2] = HS_STARTUP_TIME_MEAS_ATB0 (0x04)
            - cold_start_config[5] = HS_STARTUP_TIME_MEAS_ATB1 (0x34)
      
        - TX/RX event alert enabling:
        
            - cold_start_config[2] = TX_RX_START_STOP_MEAS_ATB0 (0x38)
            - cold_start_config[5] = TX_RX_START_STOP_MEAS_ATB1 (0x34)

        - Please notice that the default user mode register setting must be restored for typical user mode application scenarios:
        
            - cold_start_config[2] = USER_MODE_ATB0 (0x00)
            - cold_start_config[5] = USER_MODE_ATB1 (0x30)

* \section General_Configuration  General Device Configuration 

 - Beyond the cold start configuration performed only  at device reset/power-on by calling the DeviceConfiguration() function with
   coldStart = TRUE, this function performs the following general configuration operations:

    - Setup RCO32K trimming value in PMU_ANA_USER_REG 
    - Setup LDO1V2 trimming value in ATB1_ANA_ENG_REG

* \section DeviceConfiguration_ALL  How to set Device Configuration Parameters?  

  - The selected device configuration parameters (cold start + RCO and LDO trimming values) are set within the BLE device controller, Radio configuration register,  through the
    following instructions executed on DeviceConfiguration() function:

        BLUE_CTRL->RADIO_CONFIG = 0x10000U | (uint16_t)((uint32_t)cold_start_config
        & 0x0000FFFFU);
        while ((BLUE_CTRL->RADIO_CONFIG & 0x10000) != 0);

  - Once the device configuration parameters are set, the DeviceConfiguration() function must wait until the High Speed (HS) crystal is ready.
    Since the slow clock period measurement is done automatically each time the device enters in active2 state and the HS cystal is ready, the 
    related Clock Gen interrupt signals that a slow clock period measurement has been done: 
    
        while(CKGEN_BLE->CLK32K_IT == 0); //Interrupt event for 32 kHz clock measurement

**/

#include "BlueNRG_x_device.h"
#include "BluenRG1_flash.h"
#include "misc.h"
#include "hal_types.h"

#define RESET_WAKE_DEEPSLEEP_REASONS 0x05
#define CRITICAL_PRIORITY 0
/* OTA tag used to  tag a  valid application on interrupt vector table*/
#if ST_OTA_SERVICE_MANAGER_APPLICATION
#define OTA_VALID_APP_TAG (0xAABBCCDD) /* OTA Service Manager has a special valid tag */
#else
#define OTA_VALID_APP_TAG (0xAA5555AA) 
#endif

#define BLUE_FLAG_TAG   (0x424C5545)

WEAK_FUNCTION(void NMI_Handler(void) {});
WEAK_FUNCTION(void HardFault_Handler(void) {});
WEAK_FUNCTION(void SVC_Handler(void) {});
WEAK_FUNCTION(void PendSV_Handler(void) {});
WEAK_FUNCTION(void SysTick_Handler(void) {});
WEAK_FUNCTION(void GPIO_Handler(void) {});
WEAK_FUNCTION(void NVM_Handler(void) {});
WEAK_FUNCTION(void UART_Handler(void) {});
WEAK_FUNCTION(void SPI_Handler(void) {});
WEAK_FUNCTION(void Blue_Handler(void) {});
WEAK_FUNCTION(void BATTERY_LOW_Handler(void) {});
WEAK_FUNCTION(void ADV_Handler(void) {});
WEAK_FUNCTION(void MFT1A_Handler(void) {});
WEAK_FUNCTION(void MFT1B_Handler(void) {});
WEAK_FUNCTION(void MFT2A_Handler(void) {});
WEAK_FUNCTION(void MFT2B_Handler(void) {});
WEAK_FUNCTION(void RTC_Handler(void) {});
WEAK_FUNCTION(void WDG_Handler(void) {});
WEAK_FUNCTION(void ADC_Handler(void) {});
WEAK_FUNCTION(void I2C2_Handler(void) {});
WEAK_FUNCTION(void I2C1_Handler(void) {});
WEAK_FUNCTION(void DMA_Handler(void) {});
WEAK_FUNCTION(void PKA_Handler(void) {});



/* ------------------------------------------------------------------------------
*   uint32_t ota_sw_activation
*
*  OTA SW activation
*
* ------------------------------------------------------------------------------ */
NO_INIT_SECTION(volatile uint32_t ota_sw_activation, ".ota_sw_activation");

/* ------------------------------------------------------------------------------
*    uint32_t savedMSP
*
*  Private storage to hold the saved stack pointer.  This variable is only used
*  in this file and should not be extern'ed.  In our current design we
*  do not use real context switching, but only context saving and restoring.
*  As such, we only need to keep track of the Main Stack Pointer (MSP). This
*  variable is used to hold the MSP between a save and a restore.
* ------------------------------------------------------------------------------ */
SECTION(".savedMSP")
REQUIRED(uint32_t savedMSP);


/* ------------------------------------------------------------------------------
*   uint8_t wakeupFromSleepFlag
*
*  A simple flag used to indicate if the wakeup occurred from Sleep or Standby 
*  condition.
*  If this flag is zero, an interrupt has affected the WFI instruction and the
*  BlueNRG-1 doesn't enter in deep sleep state. So, no context restore is
*  necessary.
*  If this flag is non-zero, the WFI instruction puts the BlueNRG-1 in deep sleep.
*  So, at wakeup time a context restore is necessary.
*  Note: The smallest unit of storage is a single byte.
*
*  NOTE: This flag must be cleared before the context restore is called
* ------------------------------------------------------------------------------ */
SECTION(".wakeupFromSleepFlag")
REQUIRED(uint8_t wakeupFromSleepFlag);
  

/* ------------------------------------------------------------------------------
*    uint32_t __blueflag_RAM
*
*  __blueflag_RAM
*
* ------------------------------------------------------------------------------ */
SECTION(".__blueflag_RAM")
REQUIRED(uint32_t __blueflag_RAM);

/* ------------------------------------------------------------------------------
*    uint32_t savedICSR
*
*  Private storage to save the Interrupt Control State register, to check the 
*  SysTick and PendSV interrupt status
*  This variable is only used during the samrt power management 
*  procedure 
* ------------------------------------------------------------------------------ */
uint32_t savedICSR;

/* ------------------------------------------------------------------------------
*  uint32_t savedSHCSR
*
*  Private storage to save the System Handler Control and State register, 
*  to check the SVCall interrupt status
*  This variable is only used during the samrt power management 
*  procedure 
* ------------------------------------------------------------------------------ */
uint32_t savedSHCSR;

/* ------------------------------------------------------------------------------
*  uint32_t savedNVIC_ISPR
*
*  Private storage to save the Interrupt Set Pending register, 
*  to check the NVIC interrupt status
*  This variable is only used during the smart power management 
*  procedure 
*  ------------------------------------------------------------------------------ */
uint32_t savedNVIC_ISPR;


int __low_level_init(void) 
{
  /* If the reset reason is a wakeup from sleep restore the context */
  if ((CKGEN_SOC->REASON_RST == 0) && (CKGEN_BLE->REASON_RST > RESET_WAKE_DEEPSLEEP_REASONS)) {
#ifndef NO_SMART_POWER_MANAGEMENT
          
  void CS_contextRestore(void);
  wakeupFromSleepFlag = 1; /* A wakeup from Standby or Sleep occurred */
  CS_contextRestore();     /* Restore the context */
  /* if the context restore worked properly, we should never return here */
  while(1) { ; }
#else
  return 0;
#endif   
  }
  return 1;
}

#ifdef __CC_ARM

void RESET_HANDLER(void)
{
  if(__low_level_init()==1)
    __main();
  else {
    __set_MSP((uint32_t)_INITIAL_SP);
    main();
  }
}


#else /* __CC_ARM */
#ifdef __GNUC__

extern unsigned long _etext;
extern unsigned long _sidata;		/* start address for the initialization values of the .data section. Defined in linker script */
extern unsigned long _sdata;		/* start address for the .data section. Defined in linker script */
extern unsigned long _edata;		/* end address for the .data section. Defined in linker script */
extern unsigned long _sbss;			/* start address for the .bss section. Defined in linker script */
extern unsigned long _ebss;			/* end address for the .bss section. Defined in linker script */
extern unsigned long _sbssblue;		/* start address for the section reserved for the Blue controller. Defined in linker script */
extern unsigned long _ebssblue;		/* end address for the section reserved for the Blue controller. Defined in linker script */
extern unsigned long _estack;		/* init value for the stack pointer. Defined in linker script */
extern unsigned long _sidata2;		/* start address for the initialization values of the special ram_preamble */
extern unsigned long _sdata2;		/* start address the special ram_preamble defined in linker script */
extern unsigned long _edata2;		/* end address the special ram_preamble defined in linker script */

extern int main(void);

void RESET_HANDLER(void)
{
  if(__low_level_init()==1)	{
    unsigned long *pulSrc, *pulDest;
    
    // Copy the data segment initializers from flash to SRAM.
    pulSrc = &_sidata;
    for(pulDest = &_sdata; pulDest < &_edata; )
    {
      *(pulDest++) = *(pulSrc++);
    }
    
    pulSrc = &_sidata2;
    for(pulDest = &_sdata2; pulDest < &_edata2; )
    {
      *(pulDest++) = *(pulSrc++);
    }

    // Zero fill the bss segment.
    for(pulDest = &_sbssblue; pulDest < &_ebssblue; )
    {
      *(pulDest++) = 0;
    }

    for(pulDest = &_sbss; pulDest < &_ebss; )
    {
      *(pulDest++) = 0;
    }
  }
  // Call the application's entry point.
  __set_MSP((uint32_t)_INITIAL_SP);
  main();
}

#endif /* __GNUC__ */
#endif /* __CC_ARM */


SECTION(".intvec")
REQUIRED(const intvec_elem __vector_table[]) = {
    {.__ptr = _INITIAL_SP},                   /* Stack address                      */
    {RESET_HANDLER},           		          /* Reset handler is C initialization. */
    {NMI_Handler},                            /* The NMI handler                    */
    {HardFault_Handler},                      /* The hard fault handler             */
    {(intfunc) OTA_VALID_APP_TAG},            /* OTA Application                    */
    {(intfunc) BLUE_FLAG_TAG},                /* Reserved for blue flag DTM updater */
    {0x00000000},                             /* Reserved                           */
    {0x00000000},                             /* Reserved                           */
    {0x00000000},                             /* Reserved                           */
    {0x00000000},                             /* Reserved                           */
    {0x00000000},                             /* Reserved                           */
    {SVC_Handler},                            /* SVCall                             */
    {0x00000000},                             /* Reserved                           */
    {0x00000000},                             /* Reserved                           */
    {PendSV_Handler},                         /* PendSV                             */
    {SysTick_Handler},                        /* SysTick_Handler                    */
    {GPIO_Handler},                           /* IRQ0:  GPIO                        */
    {NVM_Handler},                            /* IRQ1:  NVM                         */
    {0x00000000},                             /* IRQ2:                              */
    {0x00000000},                             /* IRQ3:                              */
    {UART_Handler},                           /* IRQ4:  UART                        */
    {SPI_Handler},                            /* IRQ5:  SPI                         */
    {Blue_Handler},                           /* IRQ6:  Blue                        */
    {WDG_Handler},                            /* IRQ7:  Watchdog                    */
    {0x00000000},                             /* IRQ8:                              */
    {0x00000000},                             /* IRQ9:                              */
    {0x00000000},                             /* IRQ10:                             */
    {0x00000000},                             /* IRQ11:                             */
    {0x00000000},                             /* IRQ12:                             */
    {ADC_Handler},                            /* IRQ13  ADC                         */
    {I2C2_Handler},                           /* IRQ14  I2C2                        */
    {I2C1_Handler},                           /* IRQ15  I2C1                        */
    {0x00000000},                             /* IRQ16                              */
    {MFT1A_Handler},                          /* IRQ17  MFT1 irq1                   */
    {MFT1B_Handler},                          /* IRQ18  MFT1 irq2                   */
    {MFT2A_Handler},                          /* IRQ19  MFT2 irq1                   */
    {MFT2B_Handler},                          /* IRQ20  MFT2 irq2                   */
    {RTC_Handler},                            /* IRQ21  RTC                         */
    {PKA_Handler},                            /* IRQ22  PKA                         */
    {DMA_Handler},                            /* IRQ23  DMA                         */
    {0x00000000},                             /* IRQ24                              */
    {0x00000000},                             /* IRQ25                              */
    {0x00000000},                             /* IRQ26                              */
    {0x00000000},                             /* IRQ27                              */
    {0x00000000},                             /* IRQ28                              */
    {0x00000000},                             /* IRQ29                              */
    {0x00000000},                             /* IRQ30                              */
    {0x00000000}                              /* IRQ31                              */
};

	

//------------------------------------------------------------------------------
//   uint32_t *app_base
//
// The application base address. Used by OTA IRQ stub file to determine the
// effective application base address and jump to the proper IRQ handler.
//
//------------------------------------------------------------------------------
SECTION(".app_base")
REQUIRED(uint32_t *app_base) = (uint32_t *) __vector_table;


SECTION(".bss.__blue_RAM")
REQUIRED(static uint8_t __blue_RAM[8*64+12]) = {0,};

/**
 * @name Device Configuration Registers
 *@{
 */

/**
 *@brief Analog Test Bus 0 register settings
 */
#define ATB0_ANA_ENG_REG    0x3F
/**
 *@brief Analog Test Bus 1 register settings
 */
#define ATB1_ANA_ENG_REG    0x3E
/**
 *@brief Rate Multiplier 1 register settings 
 */
#define RM1_DIG_ENG_REG     0x3C
/**
 *@brief Low Frequency Clock and SMPS register settings
 */
#define CLOCK_LOW_ENG_REG   0x3B
/**
 *@brief High Frequency Clock register settings
 */
#define CLOCK_HIGH_ENG_REG  0x3A
/**
 *@brief Power Management register settings
 */
#define PMU_ANA_ENG_REG     0x39
/**
 *@brief System Clock register settings
 */
#define CLOCK_ANA_USER_REG  0x34
/**
 *@brief System Power Management register settings
 */
#define PMU_ANA_USER_REG    0x35
//@} \\Device Configuration Registers

/**
 * @name Device Configuration values
 *@{
 */

/**
 * @brief Enable the low frequency RO
 */
#define LOW_FREQ_RO                 0x1B
/**
 * @brief Enable the external low frequency XO
 */
#define LOW_FREQ_XO                 0x5B
/**
 * @brief Enable the high frequency 16 MHz
 */
#define HIGH_FREQ_16M               0x40
/**
 * @brief Enable the high frequrency 32 MHz
 */
#define HIGH_FREQ_32M               0x44
/**
 * @brief Enable the SMPS
 */
#define SMPS_ON                     0x4C
/**
 * @brief Disable the SMPS
 */
#define SMPS_OFF                    0x6C
/**
 * @brief SMPS clock frequency value for 4.7 uH inductor
 */
#define SMPS_4_7uH_RM1              0x40
/**
 * @brief Power management configuration for 4.7 uH inductor
 */
#define SMPS_4_7uH_PMU              0xBE
/**
 * @brief SMPS clock frequency value for 10 uH inductor
 */
#define SMPS_10uH_RM1               0x20
/**
 * @brief Power management configuration for 10 uH inductor
 */
#define SMPS_10uH_PMU               0xB2
/**
 * @brief RCO32 trimming default values
 */
#define PMU_ANA_USER_RESET_VALUE    0x0B
/**
 * @brief Analog test bus 0 settings for 
 * normal application mode
 */
#define USER_MODE_ATB0              0x00
/**
 * @brief Analog test bus 1 settings for 
 * normal application mode
 */
#define USER_MODE_ATB1              0x30
/**
 * @brief Analog test bus 0 settings for 
 * low speed crystal measurement
 */
#define LS_XTAL_MEAS_ATB0           0x37
/**
 * @brief Analog test bus 1 settings for 
 * low speed crystal measurement
 */
#define LS_XTAL_MEAS_ATB1           0x34
/**
 * @brief Analog test bus 0 settings for 
 * high speed crystal startup time measurement
 */
#define HS_STARTUP_TIME_MEAS_ATB0   0x04
/**
 * @brief Analog test bus 1 settings for 
 * high speed crystal startup time measurement
 */
#define HS_STARTUP_TIME_MEAS_ATB1   0x34
/**
 * @brief Analog test bus 0 settings for 
 * Tx/Rx start stop signal measurement
 */
#define TX_RX_START_STOP_MEAS_ATB0  0x38
/**
 * @brief Analog test bus 1 settings for 
 * Tx/Rx start stop signal measurement
 */
#define TX_RX_START_STOP_MEAS_ATB1  0x34
//@} \\Device Configuration values

/**
 *@brief Number of configuration bytes to send over Blue SPI
 */
#define NUMBER_CONFIG_BYTE  0x02
/**
 *@brief End Configuration Tag
 */
#define END_CONFIG          0x00

/**
 * @brief Cold start configuration register table
 */
#define COLD_START_CONFIGURATION                                      \
{                                                                     \
  NUMBER_CONFIG_BYTE, ATB0_ANA_ENG_REG,   USER_MODE_ATB0,             \
  NUMBER_CONFIG_BYTE, ATB1_ANA_ENG_REG,   USER_MODE_ATB1,             \
  NUMBER_CONFIG_BYTE, RM1_DIG_ENG_REG,    SMPS_10uH_RM1,              \
  NUMBER_CONFIG_BYTE, CLOCK_LOW_ENG_REG,  SMPS_ON,                    \
  NUMBER_CONFIG_BYTE, CLOCK_HIGH_ENG_REG, HIGH_FREQ_16M,              \
  NUMBER_CONFIG_BYTE, PMU_ANA_ENG_REG,    SMPS_10uH_PMU,              \
  NUMBER_CONFIG_BYTE, CLOCK_ANA_USER_REG, LOW_FREQ_XO,                \
  NUMBER_CONFIG_BYTE, PMU_ANA_USER_REG,   PMU_ANA_USER_RESET_VALUE,   \
  END_CONFIG                                                          \
}

/**
 * @brief RCO32K trimming value flash location
 */
#define RCO32K_TRIMMING_FLASH_ADDR 0x100007E8
/**
 * @brief LDO1V2 trimming value flash location
 */
#define LDO1V2_TRIMMING_FLASH_ADDR 0x100007E4
/**
 * @brief Check bytes tag
 */
#define CHECK_BYTES                0xAA55


void DeviceConfiguration(BOOL coldStart, BOOL waitLS_Ready)
{
  uint32_t current_time;
  uint32_t Trimm_config; 
  volatile uint8_t cold_start_config[] = COLD_START_CONFIGURATION;

  if (coldStart) {    
    /* High Speed Crystal Configuration */
#if (HS_SPEED_XTAL == HS_SPEED_XTAL_32MHZ)
    cold_start_config[14] = HIGH_FREQ_32M;
    /* Set 32MHz_SEL bit in the System controller register */
    SYSTEM_CTRL->CTRL_b.MHZ32_SEL = 1;
#elif (HS_SPEED_XTAL == HS_SPEED_XTAL_16MHZ)
    cold_start_config[14] = HIGH_FREQ_16M;
#else
#error "No definition for High Speed Crystal"
#endif
  
    /* Low Speed Crystal Source */
#if (LS_SOURCE == LS_SOURCE_EXTERNAL_32KHZ)
    cold_start_config[20] = LOW_FREQ_XO;
#elif (LS_SOURCE == LS_SOURCE_INTERNAL_RO)
    cold_start_config[20] = LOW_FREQ_RO;
#else
#error "No definition for Low Speed Crystal Source"
#endif
  
    /* SMPS configuration */
#if (SMPS_INDUCTOR == SMPS_INDUCTOR_10uH)
    cold_start_config[11] = SMPS_ON;
    cold_start_config[8] = SMPS_10uH_RM1;
    cold_start_config[17] = SMPS_10uH_PMU;
#elif (SMPS_INDUCTOR == SMPS_INDUCTOR_4_7uH)
    cold_start_config[11] = SMPS_ON;
    cold_start_config[8] = SMPS_4_7uH_RM1;
    cold_start_config[17] = SMPS_4_7uH_PMU;
#elif (SMPS_INDUCTOR == SMPS_INDUCTOR_NONE)
    cold_start_config[11] = SMPS_OFF;
#else
#error "No definition for SMPS Configuration"
#endif

    /* BOR configuration. Note: this setup shall be executed after the SMPS configuration*/
#if (BOR_CONFIG == BOR_ON)
    /* Clear the 3 bit of the CLOCK_LOW_ENG_REG register */
    cold_start_config[11] &= ~(1<<2);
#elif (BOR_CONFIG == BOR_OFF)
    /* Nothing to do because the BOR is disabled by default */
#else
#error "No definition for BOR Configuration"
#endif

    /* Setup RCO32K trimming value in PMU_ANA_USER_REG  */
    Trimm_config = *(volatile uint32_t*)RCO32K_TRIMMING_FLASH_ADDR;
    if ((Trimm_config >> 16) == CHECK_BYTES)
      cold_start_config[23] = ((Trimm_config&0x7)<<4)|PMU_ANA_USER_RESET_VALUE;
 
    /* Setup LDO1V2 trimming value in ATB1_ANA_ENG_REG  */
    Trimm_config = *(volatile uint32_t*)LDO1V2_TRIMMING_FLASH_ADDR;
    if ((Trimm_config >> 16) == CHECK_BYTES) {
      cold_start_config[5] &= ~0x30;                 // Clear the register content of bit 4 and 5
      cold_start_config[5] |= (Trimm_config&0x3)<<4; // Store the LDO1V2 trimming value in bit 4 and 5
    }
    
    /* Cold start configuration device */
    BLUE_CTRL->RADIO_CONFIG = 0x10000U | (uint16_t)((uint32_t)cold_start_config & 0x0000FFFFU);
    while ((BLUE_CTRL->RADIO_CONFIG & 0x10000) != 0);
  }
  
  /* Wait until HS is ready. The slow clock period 
  * measurement is done automatically each time the
  * device enters in active2 state and the HS is ready.
  * The interrupt signals that a measurement is done.
  */
  while(CKGEN_BLE->CLK32K_IT == 0);
  CKGEN_BLE->CLK32K_IT = 1;
  CKGEN_BLE->CLK32K_COUNT = 23; //Restore the window length for slow clock measurement.
  CKGEN_BLE->CLK32K_PERIOD = 0;
  
  
  /* Wait until the RO or 32KHz is ready */
  if (waitLS_Ready) {
    current_time = *(volatile uint32_t *)0x48000010;
    while(((*(volatile uint32_t *)0x48000010)&0x10) == (current_time&0x10));
  }

  if (coldStart) {
#if (HS_SPEED_XTAL == HS_SPEED_XTAL_32MHZ)
    /* AHB up converter command register write*/
    AHBUPCONV->COMMAND = 0x15;
#endif
  }
  
}

void SystemInit(void)
{
  /* Remap the vector table */
  FLASH->CONFIG = FLASH_PREMAP_MAIN;

  /* Configure all the interrupts priority. 
  * The application can modify the interrupts priority.
  * The  PendSV_IRQn and BLUE_CTRL_IRQn SHALL maintain the highest priority
  */
  NVIC_SetPriority(PendSV_IRQn,    LOW_PRIORITY);
  NVIC_SetPriority(SysTick_IRQn,   LOW_PRIORITY);
  NVIC_SetPriority(GPIO_IRQn,      LOW_PRIORITY);
  NVIC_SetPriority(NVM_IRQn,       LOW_PRIORITY);
  NVIC_SetPriority(UART_IRQn,      LOW_PRIORITY);
  NVIC_SetPriority(SPI_IRQn,       LOW_PRIORITY);
  NVIC_SetPriority(BLUE_CTRL_IRQn, CRITICAL_PRIORITY);
  NVIC_SetPriority(WDG_IRQn,       LOW_PRIORITY);
  NVIC_SetPriority(ADC_IRQn,       LOW_PRIORITY);
  NVIC_SetPriority(I2C2_IRQn,      LOW_PRIORITY);
  NVIC_SetPriority(I2C1_IRQn,      LOW_PRIORITY);
  NVIC_SetPriority(MFT1A_IRQn,    LOW_PRIORITY);
  NVIC_SetPriority(MFT1B_IRQn,    LOW_PRIORITY);
  NVIC_SetPriority(MFT2A_IRQn,    LOW_PRIORITY);
  NVIC_SetPriority(MFT2B_IRQn,    LOW_PRIORITY);
  NVIC_SetPriority(RTC_IRQn,       LOW_PRIORITY);
  NVIC_SetPriority(PKA_IRQn,       LOW_PRIORITY);
  NVIC_SetPriority(DMA_IRQn,       LOW_PRIORITY);

  /* Device Configuration */
  DeviceConfiguration(TRUE, TRUE);
  /* Disable all the peripherals clock except NVM, SYSCTR, PKA and RNG */
  CKGEN_SOC->CLOCK_EN = 0xE0066;
  __enable_irq();
}


/******************* (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/
