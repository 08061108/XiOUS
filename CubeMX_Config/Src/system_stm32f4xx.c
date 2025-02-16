/**
  ******************************************************************************
  * @file    system_stm32f4xx.c
  * @author  MCD Application Team
  * @brief   CMSIS Cortex-M4 Device Peripheral Access Layer System Source File.
  *
  *   This file provides two functions and one global variable to be called from 
  *   user application:
  *      - SystemInit(): This function is called at startup just after reset and 
  *                      before branch to main program. This call is made inside
  *                      the "startup_stm32f4xx.s" file.
  *
  *      - SystemCoreClock variable: Contains the core clock (HCLK), it can be used
  *                                  by the user application to setup the SysTick 
  *                                  timer or configure other parameters.
  *                                     
  *      - SystemCoreClockUpdate(): Updates the variable SystemCoreClock and must
  *                                 be called whenever the core clock is changed
  *                                 during program execution.
  *
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/** @addtogroup CMSIS
  * @{
  */

/** @addtogroup stm32f4xx_system
  * @{
  */  
  
/** @addtogroup STM32F4xx_System_Private_Includes
  * @{
  */


#include "stm32f4xx.h"

#if !defined  (HSE_VALUE) 
  #define HSE_VALUE    ((uint32_t)25000000) /*!< Default value of the External oscillator in Hz */    /* 定义宏 HSE_VALUE，其值为 25000000，表示外部振荡器的默认频率为25 MHz--JR*/
#endif /* HSE_VALUE */

#if !defined  (HSI_VALUE)
  #define HSI_VALUE    ((uint32_t)16000000) /*!< Value of the Internal oscillator in Hz*/
#endif /* HSI_VALUE */

/**
  * @}
  */

/** @addtogroup STM32F4xx_System_Private_TypesDefinitions
  * @{
  */

/**
  * @}
  */

/** @addtogroup STM32F4xx_System_Private_Defines
  * @{
  */

/************************* Miscellaneous Configuration ************************/
/*!< Uncomment the following line if you need to use external SRAM or SDRAM as data memory  */
/* 用于根据不同的STM32F4系列微控制器型号，配置特定的内存选项和向量表（Vector Table）的偏移量--JR*/
#if defined(STM32F405xx) || defined(STM32F415xx) || defined(STM32F407xx) || defined(STM32F417xx)\
 || defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx)\
 || defined(STM32F469xx) || defined(STM32F479xx) || defined(STM32F412Zx) || defined(STM32F412Vx)       /* 检查当前编译的目标微控制器型号是否属于STM32F405xx、STM32F415xx、STM32F407xx等系列--JR*/
/* #define DATA_IN_ExtSRAM */
#endif /* STM32F40xxx || STM32F41xxx || STM32F42xxx || STM32F43xxx || STM32F469xx || STM32F479xx ||\
          STM32F412Zx || STM32F412Vx */
 
#if defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx)\
 || defined(STM32F446xx) || defined(STM32F469xx) || defined(STM32F479xx)
/* #define DATA_IN_ExtSDRAM */
#endif /* STM32F427xx || STM32F437xx || STM32F429xx || STM32F439xx || STM32F446xx || STM32F469xx ||\
          STM32F479xx */

/*!< Uncomment the following line if you need to relocate your vector Table in
     Internal SRAM. */
/* #define VECT_TAB_SRAM */
#define VECT_TAB_OFFSET  0x00 /* 定义向量表的基地址偏移量，确保其为0x200的倍数--JR*/     /*!< Vector Table base offset field. 
                                   This value must be a multiple of 0x200. */
/******************************************************************************/

/**
  * @}
  */

/** @addtogroup STM32F4xx_System_Private_Macros
  * @{
  */

/**
  * @}
  */

/** @addtogroup STM32F4xx_System_Private_Variables
  * @{
  */
  /* This variable is updated in three ways:
      1) by calling CMSIS function SystemCoreClockUpdate()
      2) by calling HAL API function HAL_RCC_GetHCLKFreq()
      3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency 
         Note: If you use this function to configure the system clock; then there
               is no need to call the 2 first functions listed above, since SystemCoreClock
               variable is updated automatically.
  */
uint32_t SystemCoreClock = 16000000;   /* 用于存储系统核心时钟（通常是HCLK，即AHB总线时钟）的当前频率，单位为赫兹（Hz）--JR*/
const uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};     /* 该数组用于存储AHB总线时钟（HCLK）的分频系数。AHB总线时钟通常由系统时钟（SYSCLK）通过分频器分频得到,前8个元素为 0，表示不分频--JR*/
const uint8_t APBPrescTable[8]  = {0, 0, 0, 0, 1, 2, 3, 4};       /* 前4个元素为 0，表示不分频,后4个元素为 1 到 4，表示不同的分频系数--JR*/
/**
  * @}
  */

/** @addtogroup STM32F4xx_System_Private_FunctionPrototypes
  * @{
  */

#if defined (DATA_IN_ExtSRAM) || defined (DATA_IN_ExtSDRAM)
  static void SystemInit_ExtMemCtl(void); 
#endif /* DATA_IN_ExtSRAM || DATA_IN_ExtSDRAM */

/**
  * @}
  */

/** @addtogroup STM32F4xx_System_Private_Functions
  * @{
  */

/**
  * @brief  Setup the microcontroller system
  *         Initialize the FPU setting, vector table location and External memory 
  *         configuration.
  * @param  None
  * @retval None
  */
void SystemInit(void)
{
  /* FPU settings ------------------------------------------------------------*/
  #if (__FPU_PRESENT == 1) && (__FPU_USED == 1)    /* 检查浮点运算单元（FPU）是否存在且被使用--JR*/
    SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));   /* 设置协处理器访问权限寄存器（CPACR）的第10和第11位，赋予FPU完全访问权限--JR*/           /* set CP10 and CP11 Full Access */
  #endif
  /* Reset the RCC clock configuration to the default reset state ------------*/
  /* Set HSION bit */
  RCC->CR |= (uint32_t)0x00000001;    /* RCC：复位和时钟控制寄存器（Reset and Clock Control）的指针 CR：控制寄存器，用于配置基本的时钟功能--JR*/

  /* Reset CFGR register */
  RCC->CFGR = 0x00000000;          /* CFGR：配置寄存器，用于设置系统时钟源和其他相关配置--JR*/

  /* Reset HSEON, CSSON and PLLON bits */
  RCC->CR &= (uint32_t)0xFEF6FFFF;

  /* Reset PLLCFGR register */
  RCC->PLLCFGR = 0x24003010;        /* PLLCFGR：锁相环配置寄存器，用于设置PLL的参数--JR*/

  /* Reset HSEBYP bit */
  RCC->CR &= (uint32_t)0xFFFBFFFF;

  /* Disable all interrupts */
  RCC->CIR = 0x00000000;            /* CIR：时钟中断寄存器，用于处理时钟相关的中断--JR*/

#if defined (DATA_IN_ExtSRAM) || defined (DATA_IN_ExtSDRAM)
  SystemInit_ExtMemCtl(); 
#endif /* DATA_IN_ExtSRAM || DATA_IN_ExtSDRAM */

  /* Configure the Vector Table location add offset address ------------------*/
#ifdef VECT_TAB_SRAM

  /* SCB：系统控制块指针 VTOR：向量表偏移寄存器，用于设置向量表的位置 SRAM_BASE：内部SRAM的基地址 VECT_TAB_OFFSET：向量表的偏移量，通常为0x00000000或0x200的倍数--JR*/
  SCB->VTOR = SRAM_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal SRAM */
#else
  SCB->VTOR = FLASH_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal FLASH */
#endif
}

/**
   * @brief  Update SystemCoreClock variable according to Clock Register Values.
  *         The SystemCoreClock variable contains the core clock (HCLK), it can
  *         be used by the user application to setup the SysTick timer or configure
  *         other parameters.
  *           
  * @note   Each time the core clock (HCLK) changes, this function must be called
  *         to update SystemCoreClock variable value. Otherwise, any configuration
  *         based on this variable will be incorrect.         
  *     
  * @note   - The system frequency computed by this function is not the real 
  *           frequency in the chip. It is calculated based on the predefined 
  *           constant and the selected clock source:
  *             
  *           - If SYSCLK source is HSI, SystemCoreClock will contain the HSI_VALUE(*)
  *                                              
  *           - If SYSCLK source is HSE, SystemCoreClock will contain the HSE_VALUE(**)
  *                          
  *           - If SYSCLK source is PLL, SystemCoreClock will contain the HSE_VALUE(**) 
  *             or HSI_VALUE(*) multiplied/divided by the PLL factors.
  *         
  *         (*) HSI_VALUE is a constant defined in stm32f4xx_hal_conf.h file (default value
  *             16 MHz) but the real value may vary depending on the variations
  *             in voltage and temperature.   
  *    
  *         (**) HSE_VALUE is a constant defined in stm32f4xx_hal_conf.h file (its value
  *              depends on the application requirements), user has to ensure that HSE_VALUE
  *              is same as the real frequency of the crystal used. Otherwise, this function
  *              may have wrong result.
  *                
  *         - The result of this function could be not correct when using fractional
  *           value for HSE crystal.
  *     
  * @param  None
  * @retval None
  */
void SystemCoreClockUpdate(void)
{

  /* --JR
  tmp: 临时变量，用于存储中间计算结果或寄存器读取值 pllvco: PLL输出频率（VCO），用于存储PLL计算得到的中间频率
  pllp: PLL分频因子，默认值为2 pllsource: PLL时钟源选择，用于确定PLL使用的是HSE还是HSI pllm: PLL乘数因子，从寄存器中读取
  */
  uint32_t tmp = 0, pllvco = 0, pllp = 2, pllsource = 0, pllm = 2;
  
  /* Get SYSCLK source -------------------------------------------------------*/
  tmp = RCC->CFGR & RCC_CFGR_SWS;    /* 通过位掩码提取当前系统时钟源（HSI、HSE或PLL）--JR*/

  /* --JR
  根据tmp的值判断当前系统时钟源：
  0x00: 使用HSI（高速内部时钟）作为系统时钟，SystemCoreClock设为HSI_VALUE
  0x04: 使用HSE（高速外部时钟）作为系统时钟，SystemCoreClock设为HSE_VALUE
  0x08: 表示系统时钟源为锁相环（PLL）
  */
  switch (tmp)
  {
    case 0x00:  /* HSI used as system clock source */
      SystemCoreClock = HSI_VALUE;
      break;
    case 0x04:  /* HSE used as system clock source */
      SystemCoreClock = HSE_VALUE;
      break;
    case 0x08:  /* PLL used as system clock source */

      /* PLL_VCO = (HSE_VALUE or HSI_VALUE / PLL_M) * PLL_N
         SYSCLK = PLL_VCO / PLL_P
         */    
      pllsource = (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) >> 22;  /* 从RCC->PLLCFGR寄存器中提取PLL时钟源选择位段，并右移22位以获得pllsource的值--JR*/
      pllm = RCC->PLLCFGR & RCC_PLLCFGR_PLLM;       /* 从RCC->PLLCFGR寄存器中提取PLL乘数因子M的值，并存储在pllm中--JR*/
      
      if (pllsource != 0)
      {
        /* HSE used as PLL clock source */
        pllvco = (HSE_VALUE / pllm) * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6);   /* 计算PLL输出电压控制振荡器（VCO）的频率--JR*/
      }
      else
      {
        /* HSI used as PLL clock source */
        pllvco = (HSI_VALUE / pllm) * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6);
      }

      pllp = (((RCC->PLLCFGR & RCC_PLLCFGR_PLLP) >>16) + 1 ) *2;   /* 计算PLL输出分频因子P的值--JR*/
      SystemCoreClock = pllvco/pllp;   /* 计算并设置系统核心时钟频率--JR*/
      break;
    default:
      SystemCoreClock = HSI_VALUE;     /* 将系统核心时钟频率默认设置为HSI的频率值--JR*/
      break;
  }
  /* Compute HCLK frequency --------------------------------------------------*/
  /* Get HCLK prescaler */

  /* AHBPscTable: 预定义的AHB预分频器表，根据预分频因子查找对应的值 RCC->CFGR & RCC_CFGR_HPRE: 读取AHB预分频器配置--JR*/
  /* 根据AHB预分频器的配置，查找对应的预分频因子 将SystemCoreClock右移相应的位数，得到最终的HCLK频率--JR*/
  tmp = AHBPrescTable[((RCC->CFGR & RCC_CFGR_HPRE) >> 4)];
  /* HCLK frequency */
  SystemCoreClock >>= tmp;
}

#if defined (DATA_IN_ExtSRAM) && defined (DATA_IN_ExtSDRAM)
#if defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx)\
 || defined(STM32F469xx) || defined(STM32F479xx)
/**
  * @brief  Setup the external memory controller.
  *         Called in startup_stm32f4xx.s before jump to main.
  *         This function configures the external memories (SRAM/SDRAM)
  *         This SRAM/SDRAM will be used as program data memory (including heap and stack).
  * @param  None
  * @retval None
  */
/* --JR
初始化外部存储器控制器，配置GPIO引脚、时钟和相关寄存器，以便STM32F4微控制器能够与外部SRAM或SDRAM通信
调用时机：通常在启动文件（如startup_stm32f4xx.s）中，在跳转到main函数之前调用
*/
void SystemInit_ExtMemCtl(void)
{
  __IO uint32_t tmp = 0x00;    /* 定义一个32位输入输出变量tmp，用于临时存储寄存器值--JR*/

  register uint32_t tmpreg = 0, timeout = 0xFFFF;    /* 定义两个32位寄存器变量tmpreg和timeout，分别用于临时存储寄存器值和设置超时计数--JR*/
  register __IO uint32_t index;      /* 定义一个32位输入输出寄存器变量index，用于循环计数--JR*/

  /* Enable GPIOC, GPIOD, GPIOE, GPIOF, GPIOG, GPIOH and GPIOI interface clock */
  RCC->AHB1ENR |= 0x000001F8;       /* 启用GPIOC至GPIOI接口的时钟。0x000001F8对应于二进制的00000001 11111000，即启用这些GPIO端口的时钟--JR*/

  /* Delay after an RCC peripheral clock enabling */
  tmp = READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOCEN);     /* 读取GPIOC时钟使能位，确保时钟已正确启用--JR*/
  
  /* Connect PDx pins to FMC Alternate function */
  /* 配置每个GPIO端口的AF（复用功能）和模式--JR*/
  GPIOD->AFR[0]  = 0x00CCC0CC;     /* 设置GPIO端口D的复用功能选择寄存器AFR[0]，配置特定引脚为FMC所需的复用功能--JR*/
  GPIOD->AFR[1]  = 0xCCCCCCCC;
  /* Configure PDx pins in Alternate function mode */  
  GPIOD->MODER   = 0xAAAA0A8A;     /* 设置GPIO端口D的模式寄存器MODER，配置引脚为复用功能模式--JR*/
  /* Configure PDx pins speed to 100 MHz */  
  GPIOD->OSPEEDR = 0xFFFF0FCF;     /* 设置GPIO端口D的速度寄存器OSPEEDR，配置引脚的工作速度--JR*/
  /* Configure PDx pins Output type to push-pull */  
  GPIOD->OTYPER  = 0x00000000;     /* 设置GPIO端口D的输出类型寄存器OTYPER，配置引脚为推挽输出--JR*/
  /* No pull-up, pull-down for PDx pins */ 
  GPIOD->PUPDR   = 0x00000000;     /* 设置GPIO端口D的上拉/下拉寄存器PUPDR，禁用上拉和下拉电阻--JR*/

  /* Connect PEx pins to FMC Alternate function */
  GPIOE->AFR[0]  = 0xC00CC0CC;
  GPIOE->AFR[1]  = 0xCCCCCCCC;
  /* Configure PEx pins in Alternate function mode */ 
  GPIOE->MODER   = 0xAAAA828A;
  /* Configure PEx pins speed to 100 MHz */ 
  GPIOE->OSPEEDR = 0xFFFFC3CF;
  /* Configure PEx pins Output type to push-pull */  
  GPIOE->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PEx pins */ 
  GPIOE->PUPDR   = 0x00000000;
  
  /* Connect PFx pins to FMC Alternate function */
  GPIOF->AFR[0]  = 0xCCCCCCCC;
  GPIOF->AFR[1]  = 0xCCCCCCCC;
  /* Configure PFx pins in Alternate function mode */   
  GPIOF->MODER   = 0xAA800AAA;
  /* Configure PFx pins speed to 50 MHz */ 
  GPIOF->OSPEEDR = 0xAA800AAA;
  /* Configure PFx pins Output type to push-pull */  
  GPIOF->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PFx pins */ 
  GPIOF->PUPDR   = 0x00000000;

  /* Connect PGx pins to FMC Alternate function */
  GPIOG->AFR[0]  = 0xCCCCCCCC;
  GPIOG->AFR[1]  = 0xCCCCCCCC;
  /* Configure PGx pins in Alternate function mode */ 
  GPIOG->MODER   = 0xAAAAAAAA;
  /* Configure PGx pins speed to 50 MHz */ 
  GPIOG->OSPEEDR = 0xAAAAAAAA;
  /* Configure PGx pins Output type to push-pull */  
  GPIOG->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PGx pins */ 
  GPIOG->PUPDR   = 0x00000000;
  
  /* Connect PHx pins to FMC Alternate function */
  GPIOH->AFR[0]  = 0x00C0CC00;
  GPIOH->AFR[1]  = 0xCCCCCCCC;
  /* Configure PHx pins in Alternate function mode */ 
  GPIOH->MODER   = 0xAAAA08A0;
  /* Configure PHx pins speed to 50 MHz */ 
  GPIOH->OSPEEDR = 0xAAAA08A0;
  /* Configure PHx pins Output type to push-pull */  
  GPIOH->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PHx pins */ 
  GPIOH->PUPDR   = 0x00000000;
  
  /* Connect PIx pins to FMC Alternate function */
  GPIOI->AFR[0]  = 0xCCCCCCCC;
  GPIOI->AFR[1]  = 0x00000CC0;
  /* Configure PIx pins in Alternate function mode */ 
  GPIOI->MODER   = 0x0028AAAA;
  /* Configure PIx pins speed to 50 MHz */ 
  GPIOI->OSPEEDR = 0x0028AAAA;
  /* Configure PIx pins Output type to push-pull */  
  GPIOI->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PIx pins */ 
  GPIOI->PUPDR   = 0x00000000;
  
/*-- FMC Configuration -------------------------------------------------------*/
  /* Enable the FMC interface clock */
  RCC->AHB3ENR |= 0x00000001;      /* 启用FMC接口的时钟--JR*/
  /* Delay after an RCC peripheral clock enabling */
  tmp = READ_BIT(RCC->AHB3ENR, RCC_AHB3ENR_FMCEN);    /* 读取FMC时钟使能位，确保时钟已正确启用--JR*/

  FMC_Bank5_6->SDCR[0] = 0x000019E4;
  FMC_Bank5_6->SDTR[0] = 0x01115351;      
  
  /* SDRAM initialization sequence */
  /* Clock enable command */
  FMC_Bank5_6->SDCMR = 0x00000011;            /* 发送时钟使能命令到SDRAM，初始化SDRAM控制器--JR*/
  tmpreg = FMC_Bank5_6->SDSR & 0x00000020;    

  /* 等待SDRAM控制器完成初始化操作，直到状态寄存器中的相应位被清除--JR*/
  while((tmpreg != 0) && (timeout-- > 0))
  {
    tmpreg = FMC_Bank5_6->SDSR & 0x00000020; 
  }

  /* Delay */
  for (index = 0; index<1000; index++);
  
  /* PALL command */
  FMC_Bank5_6->SDCMR = 0x00000012;           
  timeout = 0xFFFF;
  while((tmpreg != 0) && (timeout-- > 0))
  {
    tmpreg = FMC_Bank5_6->SDSR & 0x00000020; 
  }
  
  /* Auto refresh command */
  /*自动刷新--JR*/
  FMC_Bank5_6->SDCMR = 0x00000073;      /* 发送自动刷新命令，确保SDRAM正常工作--JR*/
  timeout = 0xFFFF;
  while((tmpreg != 0) && (timeout-- > 0))
  {
    tmpreg = FMC_Bank5_6->SDSR & 0x00000020; 
  }
 
  /* MRD register program */
  /* 模式寄存器--JR*/
  FMC_Bank5_6->SDCMR = 0x00046014;     /* 编程模式寄存器（MRD），设置SDRAM的操作模式--JR*/
  timeout = 0xFFFF;
  while((tmpreg != 0) && (timeout-- > 0))
  {
    tmpreg = FMC_Bank5_6->SDSR & 0x00000020; 
  } 
  
  /* Set refresh count */
  /*  刷新计数设置--JR*/
  tmpreg = FMC_Bank5_6->SDRTR;
  FMC_Bank5_6->SDRTR = (tmpreg | (0x0000027C<<1));   /* 设置刷新计数，确保SDRAM定期刷新以保持数据完整性--JR*/
  
  /* Disable write protection */
  /* 写保护禁用--JR*/
  tmpreg = FMC_Bank5_6->SDCR[0]; 
  FMC_Bank5_6->SDCR[0] = (tmpreg & 0xFFFFFDFF);      /* 禁用SDRAM的写保护功能，允许写入操作--JR*/

#if defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx)
  /* Configure and enable Bank1_SRAM2 */
  /* 配置Bank1的SRAM2控制器寄存器，设置内存类型、数据宽度和突发访问模式--JR*/
  FMC_Bank1->BTCR[2]  = 0x00001011;
  FMC_Bank1->BTCR[3]  = 0x00000201;
  
  /* 配置Bank1扩展的写时序寄存器，设置写操作的时序参数--JR*/
  FMC_Bank1E->BWTR[2] = 0x0fffffff;
#endif /* STM32F427xx || STM32F437xx || STM32F429xx || STM32F439xx */ 
#if defined(STM32F469xx) || defined(STM32F479xx)
  /* Configure and enable Bank1_SRAM2 */
  FMC_Bank1->BTCR[2]  = 0x00001091;
  FMC_Bank1->BTCR[3]  = 0x00110212;
  FMC_Bank1E->BWTR[2] = 0x0fffffff;
#endif /* STM32F469xx || STM32F479xx */

  (void)(tmp); 
}
#endif /* STM32F427xx || STM32F437xx || STM32F429xx || STM32F439xx || STM32F469xx || STM32F479xx */
#elif defined (DATA_IN_ExtSRAM) || defined (DATA_IN_ExtSDRAM)
/**
  * @brief  Setup the external memory controller.
  *         Called in startup_stm32f4xx.s before jump to main.
  *         This function configures the external memories (SRAM/SDRAM)
  *         This SRAM/SDRAM will be used as program data memory (including heap and stack).
  * @param  None
  * @retval None
  */
void SystemInit_ExtMemCtl(void)    /* 初始化系统的外部存储器控制器，配置GPIO引脚和FMC/FSMC寄存器，以便与外部SDRAM或SRAM进行通信--JR*/
{
  __IO uint32_t tmp = 0x00;
#if defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx)\
 || defined(STM32F446xx) || defined(STM32F469xx) || defined(STM32F479xx)
#if defined (DATA_IN_ExtSDRAM)
  register uint32_t tmpreg = 0, timeout = 0xFFFF;
  register __IO uint32_t index;

#if defined(STM32F446xx)
  /* Enable GPIOA, GPIOC, GPIOD, GPIOE, GPIOF, GPIOG interface
      clock */
  RCC->AHB1ENR |= 0x0000007D;    /* 启用GPIOA、GPIOC、GPIOD、GPIOE、GPIOF、GPIOG接口的时钟--JR*/
#else
  /* Enable GPIOC, GPIOD, GPIOE, GPIOF, GPIOG, GPIOH and GPIOI interface 
      clock */
  RCC->AHB1ENR |= 0x000001F8;
#endif /* STM32F446xx */  
  /* Delay after an RCC peripheral clock enabling */
  tmp = READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOCEN);
  
#if defined(STM32F446xx)
  /* Connect PAx pins to FMC Alternate function */
  /* 配置GPIO引脚为FMC/FSMC替代功能--JR*/
  GPIOA->AFR[0]  |= 0xC0000000;
  GPIOA->AFR[1]  |= 0x00000000;
  /* Configure PDx pins in Alternate function mode */
  GPIOA->MODER   |= 0x00008000;
  /* Configure PDx pins speed to 50 MHz */
  GPIOA->OSPEEDR |= 0x00008000;
  /* Configure PDx pins Output type to push-pull */
  GPIOA->OTYPER  |= 0x00000000;
  /* No pull-up, pull-down for PDx pins */
  GPIOA->PUPDR   |= 0x00000000;

  /* Connect PCx pins to FMC Alternate function */
  GPIOC->AFR[0]  |= 0x00CC0000;
  GPIOC->AFR[1]  |= 0x00000000;
  /* Configure PDx pins in Alternate function mode */
  GPIOC->MODER   |= 0x00000A00;
  /* Configure PDx pins speed to 50 MHz */
  GPIOC->OSPEEDR |= 0x00000A00;
  /* Configure PDx pins Output type to push-pull */
  GPIOC->OTYPER  |= 0x00000000;
  /* No pull-up, pull-down for PDx pins */
  GPIOC->PUPDR   |= 0x00000000;
#endif /* STM32F446xx */

  /* Connect PDx pins to FMC Alternate function */
  GPIOD->AFR[0]  = 0x000000CC;
  GPIOD->AFR[1]  = 0xCC000CCC;
  /* Configure PDx pins in Alternate function mode */  
  GPIOD->MODER   = 0xA02A000A;
  /* Configure PDx pins speed to 50 MHz */  
  GPIOD->OSPEEDR = 0xA02A000A;
  /* Configure PDx pins Output type to push-pull */  
  GPIOD->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PDx pins */ 
  GPIOD->PUPDR   = 0x00000000;

  /* Connect PEx pins to FMC Alternate function */
  GPIOE->AFR[0]  = 0xC00000CC;
  GPIOE->AFR[1]  = 0xCCCCCCCC;
  /* Configure PEx pins in Alternate function mode */ 
  GPIOE->MODER   = 0xAAAA800A;
  /* Configure PEx pins speed to 50 MHz */ 
  GPIOE->OSPEEDR = 0xAAAA800A;
  /* Configure PEx pins Output type to push-pull */  
  GPIOE->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PEx pins */ 
  GPIOE->PUPDR   = 0x00000000;

  /* Connect PFx pins to FMC Alternate function */
  GPIOF->AFR[0]  = 0xCCCCCCCC;
  GPIOF->AFR[1]  = 0xCCCCCCCC;
  /* Configure PFx pins in Alternate function mode */   
  GPIOF->MODER   = 0xAA800AAA;
  /* Configure PFx pins speed to 50 MHz */ 
  GPIOF->OSPEEDR = 0xAA800AAA;
  /* Configure PFx pins Output type to push-pull */  
  GPIOF->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PFx pins */ 
  GPIOF->PUPDR   = 0x00000000;

  /* Connect PGx pins to FMC Alternate function */
  GPIOG->AFR[0]  = 0xCCCCCCCC;
  GPIOG->AFR[1]  = 0xCCCCCCCC;
  /* Configure PGx pins in Alternate function mode */ 
  GPIOG->MODER   = 0xAAAAAAAA;
  /* Configure PGx pins speed to 50 MHz */ 
  GPIOG->OSPEEDR = 0xAAAAAAAA;
  /* Configure PGx pins Output type to push-pull */  
  GPIOG->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PGx pins */ 
  GPIOG->PUPDR   = 0x00000000;

#if defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx)\
 || defined(STM32F469xx) || defined(STM32F479xx)  
  /* Connect PHx pins to FMC Alternate function */
  GPIOH->AFR[0]  = 0x00C0CC00;
  GPIOH->AFR[1]  = 0xCCCCCCCC;
  /* Configure PHx pins in Alternate function mode */ 
  GPIOH->MODER   = 0xAAAA08A0;
  /* Configure PHx pins speed to 50 MHz */ 
  GPIOH->OSPEEDR = 0xAAAA08A0;
  /* Configure PHx pins Output type to push-pull */  
  GPIOH->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PHx pins */ 
  GPIOH->PUPDR   = 0x00000000;
  
  /* Connect PIx pins to FMC Alternate function */
  GPIOI->AFR[0]  = 0xCCCCCCCC;
  GPIOI->AFR[1]  = 0x00000CC0;
  /* Configure PIx pins in Alternate function mode */ 
  GPIOI->MODER   = 0x0028AAAA;
  /* Configure PIx pins speed to 50 MHz */ 
  GPIOI->OSPEEDR = 0x0028AAAA;
  /* Configure PIx pins Output type to push-pull */  
  GPIOI->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PIx pins */ 
  GPIOI->PUPDR   = 0x00000000;
#endif /* STM32F427xx || STM32F437xx || STM32F429xx || STM32F439xx || STM32F469xx || STM32F479xx */
  
/*-- FMC Configuration -------------------------------------------------------*/
  /* Enable the FMC interface clock */
  RCC->AHB3ENR |= 0x00000001;      /* 配置GPIO引脚为FMC/FSMC替代功能--JR*/
  /* Delay after an RCC peripheral clock enabling */
  tmp = READ_BIT(RCC->AHB3ENR, RCC_AHB3ENR_FMCEN);

  /* Configure and enable SDRAM bank1 */
#if defined(STM32F446xx)
  FMC_Bank5_6->SDCR[0] = 0x00001954;
#else  
  FMC_Bank5_6->SDCR[0] = 0x000019E4;
#endif /* STM32F446xx */
  FMC_Bank5_6->SDTR[0] = 0x01115351;      
  
  /* SDRAM initialization sequence */
  /* Clock enable command */
  FMC_Bank5_6->SDCMR = 0x00000011; 
  tmpreg = FMC_Bank5_6->SDSR & 0x00000020; 
  while((tmpreg != 0) && (timeout-- > 0))
  {
    tmpreg = FMC_Bank5_6->SDSR & 0x00000020; 
  }

  /* Delay */
  for (index = 0; index<1000; index++);
  
  /* PALL command */
  FMC_Bank5_6->SDCMR = 0x00000012;           
  timeout = 0xFFFF;
  while((tmpreg != 0) && (timeout-- > 0))
  {
    tmpreg = FMC_Bank5_6->SDSR & 0x00000020; 
  }
  
  /* Auto refresh command */
#if defined(STM32F446xx)
  FMC_Bank5_6->SDCMR = 0x000000F3;
#else  
  FMC_Bank5_6->SDCMR = 0x00000073;
#endif /* STM32F446xx */
  timeout = 0xFFFF;
  while((tmpreg != 0) && (timeout-- > 0))
  {
    tmpreg = FMC_Bank5_6->SDSR & 0x00000020; 
  }
 
  /* MRD register program */
#if defined(STM32F446xx)
  FMC_Bank5_6->SDCMR = 0x00044014;
#else  
  FMC_Bank5_6->SDCMR = 0x00046014;
#endif /* STM32F446xx */
  timeout = 0xFFFF;
  while((tmpreg != 0) && (timeout-- > 0))
  {
    tmpreg = FMC_Bank5_6->SDSR & 0x00000020; 
  } 
  
  /* Set refresh count */
  tmpreg = FMC_Bank5_6->SDRTR;
#if defined(STM32F446xx)
  FMC_Bank5_6->SDRTR = (tmpreg | (0x0000050C<<1));    /* 设置SDRAM的刷新计数，确保SDRAM定期刷新以保持数据完整性--JR*/
#else    
  FMC_Bank5_6->SDRTR = (tmpreg | (0x0000027C<<1));
#endif /* STM32F446xx */
  
  /* Disable write protection */
  tmpreg = FMC_Bank5_6->SDCR[0]; 
  FMC_Bank5_6->SDCR[0] = (tmpreg & 0xFFFFFDFF);        /* 禁用SDRAM的写保护，允许写入操作--JR*/
#endif /* DATA_IN_ExtSDRAM */
#endif /* STM32F427xx || STM32F437xx || STM32F429xx || STM32F439xx || STM32F446xx || STM32F469xx || STM32F479xx */

#if defined(STM32F405xx) || defined(STM32F415xx) || defined(STM32F407xx) || defined(STM32F417xx)\
 || defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx)\
 || defined(STM32F469xx) || defined(STM32F479xx) || defined(STM32F412Zx) || defined(STM32F412Vx)

#if defined(DATA_IN_ExtSRAM)
/*-- GPIOs Configuration -----------------------------------------------------*/
   /* Enable GPIOD, GPIOE, GPIOF and GPIOG interface clock */
  RCC->AHB1ENR   |= 0x00000078;     /* 通过位操作将AHB1ENR寄存器的第7到第3位（对应GPIOA到GPIOG）设置为1，启用这些GPIO端口的时钟--JR*/
  /* Delay after an RCC peripheral clock enabling */
  /* 通过READ_BIT宏读取AHB1ENR寄存器中GPIODEN位的状态，确保GPIO端口时钟已正确启用。这里的延迟操作有助于确保时钟稳定--JR*/
  tmp = READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIODEN);
  
  /* Connect PDx pins to FMC Alternate function */
  /* 每个GPIO端口的配置包括设置替代功能（AF）、模式、速度、输出类型和上拉/下拉--JR*/
  GPIOD->AFR[0]  = 0x00CCC0CC;
  GPIOD->AFR[1]  = 0xCCCCCCCC;
  /* Configure PDx pins in Alternate function mode */  
  GPIOD->MODER   = 0xAAAA0A8A;
  /* Configure PDx pins speed to 100 MHz */  
  GPIOD->OSPEEDR = 0xFFFF0FCF;
  /* Configure PDx pins Output type to push-pull */  
  GPIOD->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PDx pins */ 
  GPIOD->PUPDR   = 0x00000000;

  /* Connect PEx pins to FMC Alternate function */
  GPIOE->AFR[0]  = 0xC00CC0CC;
  GPIOE->AFR[1]  = 0xCCCCCCCC;
  /* Configure PEx pins in Alternate function mode */ 
  GPIOE->MODER   = 0xAAAA828A;
  /* Configure PEx pins speed to 100 MHz */ 
  GPIOE->OSPEEDR = 0xFFFFC3CF;
  /* Configure PEx pins Output type to push-pull */  
  GPIOE->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PEx pins */ 
  GPIOE->PUPDR   = 0x00000000;

  /* Connect PFx pins to FMC Alternate function */
  GPIOF->AFR[0]  = 0x00CCCCCC;
  GPIOF->AFR[1]  = 0xCCCC0000;
  /* Configure PFx pins in Alternate function mode */   
  GPIOF->MODER   = 0xAA000AAA;
  /* Configure PFx pins speed to 100 MHz */ 
  GPIOF->OSPEEDR = 0xFF000FFF;
  /* Configure PFx pins Output type to push-pull */  
  GPIOF->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PFx pins */ 
  GPIOF->PUPDR   = 0x00000000;

  /* Connect PGx pins to FMC Alternate function */
  GPIOG->AFR[0]  = 0x00CCCCCC;
  GPIOG->AFR[1]  = 0x000000C0;
  /* Configure PGx pins in Alternate function mode */ 
  GPIOG->MODER   = 0x00085AAA;
  /* Configure PGx pins speed to 100 MHz */ 
  GPIOG->OSPEEDR = 0x000CAFFF;
  /* Configure PGx pins Output type to push-pull */  
  GPIOG->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PGx pins */ 
  GPIOG->PUPDR   = 0x00000000;
  
/*-- FMC/FSMC Configuration --------------------------------------------------*/
  /* Enable the FMC/FSMC interface clock */
  RCC->AHB3ENR         |= 0x00000001;       /* 每个GPIO端口的配置包括设置替代功能（AF）、模式、速度、输出类型和上拉/下拉--JR*/

#if defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx)
  /* Delay after an RCC peripheral clock enabling */
  tmp = READ_BIT(RCC->AHB3ENR, RCC_AHB3ENR_FMCEN);    /* 通过READ_BIT确保FMC时钟已启用--JR*/
  /* Configure and enable Bank1_SRAM2 */
  /* 配置Bank1的时序和模式，以适应SRAM的工作要求--JR*/
  FMC_Bank1->BTCR[2]  = 0x00001011;
  FMC_Bank1->BTCR[3]  = 0x00000201;
  FMC_Bank1E->BWTR[2] = 0x0fffffff;
#endif /* STM32F427xx || STM32F437xx || STM32F429xx || STM32F439xx */ 
#if defined(STM32F469xx) || defined(STM32F479xx)
  /* Delay after an RCC peripheral clock enabling */
  tmp = READ_BIT(RCC->AHB3ENR, RCC_AHB3ENR_FMCEN);
  /* Configure and enable Bank1_SRAM2 */
  FMC_Bank1->BTCR[2]  = 0x00001091;
  FMC_Bank1->BTCR[3]  = 0x00110212;
  FMC_Bank1E->BWTR[2] = 0x0fffffff;
#endif /* STM32F469xx || STM32F479xx */
#if defined(STM32F405xx) || defined(STM32F415xx) || defined(STM32F407xx)|| defined(STM32F417xx)\
   || defined(STM32F412Zx) || defined(STM32F412Vx)
  /* Delay after an RCC peripheral clock enabling */
  tmp = READ_BIT(RCC->AHB3ENR, RCC_AHB3ENR_FSMCEN);
  /* Configure and enable Bank1_SRAM2 */
  FSMC_Bank1->BTCR[2]  = 0x00001011;
  FSMC_Bank1->BTCR[3]  = 0x00000201;
  FSMC_Bank1E->BWTR[2] = 0x0FFFFFFF;
#endif /* STM32F405xx || STM32F415xx || STM32F407xx || STM32F417xx || STM32F412Zx || STM32F412Vx */

#endif /* DATA_IN_ExtSRAM */
#endif /* STM32F405xx || STM32F415xx || STM32F407xx || STM32F417xx || STM32F427xx || STM32F437xx ||\
          STM32F429xx || STM32F439xx || STM32F469xx || STM32F479xx || STM32F412Zx || STM32F412Vx  */ 
  (void)(tmp); 
}
#endif /* DATA_IN_ExtSRAM && DATA_IN_ExtSDRAM */
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
