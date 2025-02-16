/**
  ******************************************************************************
  * @file    stm32f4xx_hal_conf_template.h
  * @author  MCD Application Team
  * @brief   HAL configuration template file.
  *          This file should be copied to the application folder and renamed
  *          to stm32f4xx_hal_conf.h.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F4xx_HAL_CONF_H
#define __STM32F4xx_HAL_CONF_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* ########################## Module Selection ############################## */
/**
  * @brief This is the list of modules to be used in the HAL driver
  */
#define HAL_MODULE_ENABLED          /* 用于选择要在HAL驱动中使用的模块----------------------JR*/


  /* #define HAL_ADC_MODULE_ENABLED   */
/* #define HAL_CRYP_MODULE_ENABLED   */
#define HAL_CAN_MODULE_ENABLED          /* 表示启用CAN（Controller Area Network）模块-------------------JR*/


/* #define HAL_CRC_MODULE_ENABLED   */
/* #define HAL_CRYP_MODULE_ENABLED   */
/* #define HAL_DAC_MODULE_ENABLED   */
/* #define HAL_DCMI_MODULE_ENABLED   */
/* #define HAL_DMA2D_MODULE_ENABLED   */
/* #define HAL_ETH_MODULE_ENABLED   */
/* #define HAL_NAND_MODULE_ENABLED   */
/* #define HAL_NOR_MODULE_ENABLED   */
/* #define HAL_PCCARD_MODULE_ENABLED   */
#define HAL_SRAM_MODULE_ENABLED           /* 启用SRAM（静态随机存取存储器）模块-------------------JR*/

/* #define HAL_SDRAM_MODULE_ENABLED   */
/* #define HAL_HASH_MODULE_ENABLED   */

#define HAL_I2C_MODULE_ENABLED        /* 启用I2C（Inter - Integrated Circuit）模块 与I2C通信（如I2C设备驱动、I2C数据传输等）有关的代码将被包含进来----JR*/
/* #define HAL_I2S_MODULE_ENABLED   */

#define HAL_IWDG_MODULE_ENABLED       /* 启用独立看门狗（IWDG）模块 独立看门狗是一种定时器，用于监控系统的运行状态----------------JR*/

/* #define HAL_LTDC_MODULE_ENABLED   */
/* #define HAL_RNG_MODULE_ENABLED   */
#define HAL_RTC_MODULE_ENABLED        /* 启用实时时钟（RTC）模块 RTC可以提供准确的时间和日期信息-------JR*/

/* #define HAL_SAI_MODULE_ENABLED   */
#define HAL_SD_MODULE_ENABLED         /* 启用SD（Secure Digital）卡模块相关的功能，包括SD卡的初始化、数据读写等操作-------JR*/

/* #define HAL_MMC_MODULE_ENABLED   */
#define HAL_SPI_MODULE_ENABLED       /* 启用SPI（Serial Peripheral Interface）模块 SPI是一种高速、全双工的通信接口，常用于连接微控制器和外设，如传感器、存储器等--JR*/

#define HAL_TIM_MODULE_ENABLED       /* 启用定时器（TIM）模块--------JR*/

#define HAL_UART_MODULE_ENABLED      /* 启用UART（Universal Asynchronous Receiver - Transmitter）模块 UART是一种异步串行通信接口，常用于与其他设备进行数据传输--JR*/

/* #define HAL_USART_MODULE_ENABLED   */
/* #define HAL_IRDA_MODULE_ENABLED   */
/* #define HAL_SMARTCARD_MODULE_ENABLED   */
/* #define HAL_SMBUS_MODULE_ENABLED   */
/* #define HAL_WWDG_MODULE_ENABLED   */
/* #define HAL_PCD_MODULE_ENABLED   */
/* #define HAL_HCD_MODULE_ENABLED   */
/* #define HAL_DSI_MODULE_ENABLED   */
/* #define HAL_QSPI_MODULE_ENABLED   */
/* #define HAL_QSPI_MODULE_ENABLED   */
/* #define HAL_CEC_MODULE_ENABLED   */
/* #define HAL_FMPI2C_MODULE_ENABLED   */
/* #define HAL_SPDIFRX_MODULE_ENABLED   */
/* #define HAL_DFSDM_MODULE_ENABLED   */
/* #define HAL_LPTIM_MODULE_ENABLED   */

#define HAL_GPIO_MODULE_ENABLED       /* 启用通用输入输出（GPIO）模块------JR*/

#define HAL_EXTI_MODULE_ENABLED      /* 启用外部中断/事件控制器（EXTI）模块 处理来自外部设备（如按键、传感器等）的中断请求------JR*/

#define HAL_DMA_MODULE_ENABLED       /* 启用直接内存访问（DMA）模块 DMA允许外设和内存之间直接进行数据传输-----JR*/

#define HAL_RCC_MODULE_ENABLED       /* 启用实时时钟控制器（RCC）模块---------JR*/

#define HAL_FLASH_MODULE_ENABLED     /* 启用闪存（FLASH）模块 闪存可用于存储程序代码、数据等--------JR*/

#define HAL_PWR_MODULE_ENABLED       /* 启用电源（PWR）模块 该模块与微控制器的电源管理相关，例如控制不同电源模式的切换--------JR*/

#define HAL_CORTEX_MODULE_ENABLED    /* 启用Cortex - M系列内核相关模块------JR*/


/* ########################## HSE/HSI Values adaptation ##################### */
/**
  * @brief Adjust the value of External High Speed oscillator (HSE) used in your application.
  *        This value is used by the RCC HAL module to compute the system frequency
  *        (when HSE is used as system clock source, directly or through the PLL).
  */

/*--JR
HSE（外部高速振荡器）
  如果没有预先定义HSE_VALUE 则将其定义为8MHz
    这个值用于RCC（实时时钟控制器）HAL模块计算系统频率，当HSE作为系统时钟源（直接或通过PLL）时使用
*/
#if !defined  (HSE_VALUE)
  #define HSE_VALUE    ((uint32_t)8000000U) /*!< Value of the External oscillator in Hz */
#endif /* HSE_VALUE */

/* 设置HSE启动超时时间为100ms，在启动HSE时，如果在100ms内没有成功启动则会触发相应错误处理-----JR*/
#if !defined  (HSE_STARTUP_TIMEOUT)
  #define HSE_STARTUP_TIMEOUT    ((uint32_t)100U)   /*!< Time out for HSE start up, in ms */
#endif /* HSE_STARTUP_TIMEOUT */


/**
  * @brief Internal High Speed oscillator (HSI) value.
  *        This value is used by the RCC HAL module to compute the system frequency
  *        (when HSI is used as system clock source, directly or through the PLL).
  */

/*--JR
HSI（内部高速振荡器
  默认的HSI值为16MHz，用于类似HSE的计算系统频率的情况（当HSI作为时钟源时）
*/
#if !defined  (HSI_VALUE)
  #define HSI_VALUE    ((uint32_t)16000000U) /*!< Value of the Internal oscillator in Hz*/
#endif /* HSI_VALUE */

/**
  * @brief Internal Low Speed oscillator (LSI) value.
  */
/*--JR
LSI（内部低速振荡器）
  定义LSI的典型值为32kHz
*/
#if !defined  (LSI_VALUE)
 #define LSI_VALUE  ((uint32_t)32000U)       /*!< LSI Typical Value in Hz*/
#endif /* LSI_VALUE */                      /*!< Value of the Internal Low Speed oscillator in Hz
                                             The real value may vary depending on the variations
                                             in voltage and temperature.*/


/**
  * @brief External Low Speed oscillator (LSE) value.
  */
/* LSE（外部低速振荡器）将LSE的值定义为32768Hz--------JR*/
#if !defined  (LSE_VALUE)
 #define LSE_VALUE  ((uint32_t)32768U)    /*!< Value of the External Low Speed oscillator in Hz */
#endif /* LSE_VALUE */

/* LSE启动超时时间设为5000ms--------------JR*/
#if !defined  (LSE_STARTUP_TIMEOUT)
  #define LSE_STARTUP_TIMEOUT    ((uint32_t)5000U)   /*!< Time out for LSE start up, in ms */
#endif /* LSE_STARTUP_TIMEOUT */


/**
  * @brief External clock source for I2S peripheral
  *        This value is used by the I2S HAL module to compute the I2S clock source
  *        frequency, this source is inserted directly through I2S_CKIN pad.
  */
/* 用于I2S HAL模块计算I2S时钟源频率的外部时钟源值----------JR*/
#if !defined  (EXTERNAL_CLOCK_VALUE)
  #define EXTERNAL_CLOCK_VALUE    ((uint32_t)12288000U) /*!< Value of the External audio frequency in Hz*/
#endif /* EXTERNAL_CLOCK_VALUE */


/* Tip: To avoid modifying this file each time you need to use different HSE,
   ===  you can define the HSE value in your toolchain compiler preprocessor. */

/* ########################### System Configuration ######################### */
/**
  * @brief This is the HAL system configuration section
  */
/*--JR
定义VDD（电源电压）的值为3300mV
设置滴答中断的优先级为0
USE_RTOS                     0U--表示不使用实时操作系统（如果设为1则表示使用）
PREFETCH_ENABLE              1U--启用预取功能
INSTRUCTION_CACHE_ENABLE     1U--允许指令缓存
DATA_CACHE_ENABLE            1U--允许数据缓存
*/
#define  VDD_VALUE		      ((uint32_t)3300U) /*!< Value of VDD in mv */
#define  TICK_INT_PRIORITY            ((uint32_t)0U)   /*!< tick interrupt priority */
#define  USE_RTOS                     0U
#define  PREFETCH_ENABLE              1U
#define  INSTRUCTION_CACHE_ENABLE     1U
#define  DATA_CACHE_ENABLE            1U


/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* ################## Ethernet peripheral configuration ##################### */

/* Section 1 : Ethernet peripheral configuration */

/* MAC地址定义--定义了以太网的MAC地址各个字节，MAC_ADDR0到MAC_ADDR5-----JR*/
/* MAC ADDRESS: MAC_ADDR0:MAC_ADDR1:MAC_ADDR2:MAC_ADDR3:MAC_ADDR4:MAC_ADDR5 */
#define MAC_ADDR0   2U
#define MAC_ADDR1   0U
#define MAC_ADDR2   0U
#define MAC_ADDR3   0U
#define MAC_ADDR4   0U
#define MAC_ADDR5   0U

/* --JR 缓冲区相关
  ETH_RX_BUF_SIZE和ETH_TX_BUF_SIZE被定义为ETH_MAX_PACKET_SIZE--表示接收和发送缓冲区的大小
  ETH_RXBUFNB和ETH_TXBUFNB--分别定义了接收和发送缓冲区的数量为4个
*/
/* Definition of the Ethernet driver buffers size and count */
#define ETH_RX_BUF_SIZE                ETH_MAX_PACKET_SIZE /* buffer size for receive               */
#define ETH_TX_BUF_SIZE                ETH_MAX_PACKET_SIZE /* buffer size for transmit              */
#define ETH_RXBUFNB                    ((uint32_t)4U)       /* 4 Rx buffers of size ETH_RX_BUF_SIZE  */
#define ETH_TXBUFNB                    ((uint32_t)4U)       /* 4 Tx buffers of size ETH_TX_BUF_SIZE  */

/* Section 2: PHY configuration section */

/*--JR
PHY（物理层）配置
  地址相关--设为0x01U，是以太网PHY芯片的地址
  延迟相关--PHY_RESET_DELAY和PHY_CONFIG_DELAY分别定义了PHY复位延迟和配置延迟的值
  读写超时相关--PHY_READ_TO和PHY_WRITE_TO定义了PHY读写操作的超时时间
*/
/* DP83848_PHY_ADDRESS Address*/
#define DP83848_PHY_ADDRESS           0x01U
/* PHY Reset delay these values are based on a 1 ms Systick interrupt*/
#define PHY_RESET_DELAY                 ((uint32_t)0x000000FFU)
/* PHY Configuration delay */
#define PHY_CONFIG_DELAY                ((uint32_t)0x00000FFFU)

#define PHY_READ_TO                     ((uint32_t)0x0000FFFFU)
#define PHY_WRITE_TO                    ((uint32_t)0x0000FFFFU)


/* Section 3: Common PHY Registers */
/* 基本控制与状态寄存器相关--JR*/
/* 基本控制寄存器--JR*/
#define PHY_BCR                         ((uint16_t)0x0000U)    /*!< Transceiver Basic Control Register   */
/* 基本状态寄存器--JR*/
#define PHY_BSR                         ((uint16_t)0x0001U)    /*!< Transceiver Basic Status Register    */


/* 功能操作相关宏定义--JR*/
/* 复位操作--JR*/
#define PHY_RESET                       ((uint16_t)0x8000U)  /*!< PHY Reset */

/* 环回模式选择--JR*/
#define PHY_LOOPBACK                    ((uint16_t)0x4000U)  /*!< Select loop-back mode */

/* 设置PHY为100Mbps全双工模式 这个值写入BCR寄存器后，PHY会按照100Mbps的速度并且全双工的方式运行--JR*/
#define PHY_FULLDUPLEX_100M             ((uint16_t)0x2100U)  /*!< Set the full-duplex mode at 100 Mb/s */

/* 100Mbps半双工模式--JR*/
#define PHY_HALFDUPLEX_100M             ((uint16_t)0x2000U)  /*!< Set the half-duplex mode at 100 Mb/s */

/* 10Mbps全双工模式--JR*/
#define PHY_FULLDUPLEX_10M              ((uint16_t)0x0100U)  /*!< Set the full-duplex mode at 10 Mb/s  */

/* 10Mbps半双工模式--JR*/
#define PHY_HALFDUPLEX_10M              ((uint16_t)0x0000U)  /*!< Set the half-duplex mode at 10 Mb/s  */

/* 当写入BCR寄存器时启用自动协商功能。这使得PHY能够与连接的对端设备自动协商最佳的通信模式，包括速率、双工模式等--JR*/
#define PHY_AUTONEGOTIATION             ((uint16_t)0x1000U)  /*!< Enable auto-negotiation function     */

/* 用于重新启动自动协商过程--JR*/
#define PHY_RESTART_AUTONEGOTIATION     ((uint16_t)0x0200U)  /*!< Restart auto-negotiation function    */

/* 选择PHY进入电源关闭模式，降低功耗--JR*/
#define PHY_POWERDOWN                   ((uint16_t)0x0800U)  /*!< Select the power down mode           */

/* 将PHY从MII（Media Independent Interface）隔离，防止PHY与其他部分的交互--JR*/
#define PHY_ISOLATE                     ((uint16_t)0x0400U)  /*!< Isolate PHY from MII                 */


/* 状态标志相关*/
/* 当这个标志被置位时，表示自动协商过程已经完成--JR*/
#define PHY_AUTONEGO_COMPLETE           ((uint16_t)0x0020U)  /*!< Auto-Negotiation process completed   */

/* 如果这个标志被置位，说明已经建立了有效的链路连接--JR*/
#define PHY_LINKED_STATUS               ((uint16_t)0x0004U)  /*!< Valid link established               */

/* 当检测到Jabber条件（发送方发送数据过快导致接收方无法处理）时，这个标志会被置位--JR*/
#define PHY_JABBER_DETECTION            ((uint16_t)0x0002U)  /*!< Jabber condition detected            */


/* Section 4: Extended PHY Registers */
/* 扩展PHY寄存器相关--JR*/
/* 它是PHY status register Offset（PHY状态寄存器偏移量），用于定位扩展PHY状态寄存器--JR*/
#define PHY_SR                          ((uint16_t)0x10U)    /*!< PHY status register Offset                      */

/* 是一个掩码，用于从PHY状态寄存器中提取速度相关的状态信息--JR*/
#define PHY_SPEED_STATUS                ((uint16_t)0x0002U)  /*!< PHY Speed mask                                  */

/* 用于提取双工模式相关的状态信息的掩码--JR*/
#define PHY_DUPLEX_STATUS               ((uint16_t)0x0004U)  /*!< PHY Duplex mask                                 */


/* ################## SPI peripheral configuration ########################## */

/* CRC FEATURE: Use to activate CRC feature inside HAL SPI Driver
* Activated: CRC code is present inside driver
* Deactivated: CRC code cleaned from driver
*/

/* 预处理器指令部分--JR*/
/* 一个预定义宏，将USE_SPI_CRC设置为0。可能表示在SPI（串行外围接口）通信中不使用CRC（循环冗余校验）--JR*/
#define USE_SPI_CRC                     0U

/* Includes ------------------------------------------------------------------*/
/**
  * @brief Include module's header file
  */

#ifdef HAL_RCC_MODULE_ENABLED
  #include "stm32f4xx_hal_rcc.h"
#endif /* HAL_RCC_MODULE_ENABLED */

#ifdef HAL_EXTI_MODULE_ENABLED
  #include "stm32f4xx_hal_exti.h"
#endif /* HAL_EXTI_MODULE_ENABLED */

#ifdef HAL_GPIO_MODULE_ENABLED
  #include "stm32f4xx_hal_gpio.h"
#endif /* HAL_GPIO_MODULE_ENABLED */

#ifdef HAL_DMA_MODULE_ENABLED
  #include "stm32f4xx_hal_dma.h"
#endif /* HAL_DMA_MODULE_ENABLED */

#ifdef HAL_CORTEX_MODULE_ENABLED
  #include "stm32f4xx_hal_cortex.h"
#endif /* HAL_CORTEX_MODULE_ENABLED */

#ifdef HAL_ADC_MODULE_ENABLED      /* ADC（模数转换器）模块--连续变化的模拟信号转换为离散的数字信号--JR*/
  #include "stm32f4xx_hal_adc.h" 
#endif /* HAL_ADC_MODULE_ENABLED */

#ifdef HAL_CAN_MODULE_ENABLED
  #include "stm32f4xx_hal_can.h"
#endif /* HAL_CAN_MODULE_ENABLED */

#ifdef HAL_CRC_MODULE_ENABLED
  #include "stm32f4xx_hal_crc.h"
#endif /* HAL_CRC_MODULE_ENABLED */

#ifdef HAL_CRYP_MODULE_ENABLED   /* 加密（CRYPTO）模块--JR*/
  #include "stm32f4xx_hal_cryp.h"
#endif /* HAL_CRYP_MODULE_ENABLED */

#ifdef HAL_SMBUS_MODULE_ENABLED
#include "stm32f4xx_hal_smbus.h"
#endif /* HAL_SMBUS_MODULE_ENABLED */

#ifdef HAL_DMA2D_MODULE_ENABLED
  #include "stm32f4xx_hal_dma2d.h"
#endif /* HAL_DMA2D_MODULE_ENABLED */

#ifdef HAL_DAC_MODULE_ENABLED
  #include "stm32f4xx_hal_dac.h"
#endif /* HAL_DAC_MODULE_ENABLED */

#ifdef HAL_DCMI_MODULE_ENABLED
  #include "stm32f4xx_hal_dcmi.h"
#endif /* HAL_DCMI_MODULE_ENABLED */

#ifdef HAL_ETH_MODULE_ENABLED
  #include "stm32f4xx_hal_eth.h"
#endif /* HAL_ETH_MODULE_ENABLED */

#ifdef HAL_FLASH_MODULE_ENABLED
  #include "stm32f4xx_hal_flash.h"
#endif /* HAL_FLASH_MODULE_ENABLED */

#ifdef HAL_SRAM_MODULE_ENABLED
  #include "stm32f4xx_hal_sram.h"
#endif /* HAL_SRAM_MODULE_ENABLED */

#ifdef HAL_NOR_MODULE_ENABLED
  #include "stm32f4xx_hal_nor.h"
#endif /* HAL_NOR_MODULE_ENABLED */

#ifdef HAL_NAND_MODULE_ENABLED
  #include "stm32f4xx_hal_nand.h"
#endif /* HAL_NAND_MODULE_ENABLED */

#ifdef HAL_PCCARD_MODULE_ENABLED
  #include "stm32f4xx_hal_pccard.h"
#endif /* HAL_PCCARD_MODULE_ENABLED */

#ifdef HAL_SDRAM_MODULE_ENABLED
  #include "stm32f4xx_hal_sdram.h"
#endif /* HAL_SDRAM_MODULE_ENABLED */

#ifdef HAL_HASH_MODULE_ENABLED
 #include "stm32f4xx_hal_hash.h"
#endif /* HAL_HASH_MODULE_ENABLED */

#ifdef HAL_I2C_MODULE_ENABLED
 #include "stm32f4xx_hal_i2c.h"
#endif /* HAL_I2C_MODULE_ENABLED */

#ifdef HAL_I2S_MODULE_ENABLED
 #include "stm32f4xx_hal_i2s.h"
#endif /* HAL_I2S_MODULE_ENABLED */

#ifdef HAL_IWDG_MODULE_ENABLED
 #include "stm32f4xx_hal_iwdg.h"
#endif /* HAL_IWDG_MODULE_ENABLED */

#ifdef HAL_LTDC_MODULE_ENABLED
 #include "stm32f4xx_hal_ltdc.h"
#endif /* HAL_LTDC_MODULE_ENABLED */

#ifdef HAL_PWR_MODULE_ENABLED
 #include "stm32f4xx_hal_pwr.h"
#endif /* HAL_PWR_MODULE_ENABLED */

#ifdef HAL_RNG_MODULE_ENABLED
 #include "stm32f4xx_hal_rng.h"
#endif /* HAL_RNG_MODULE_ENABLED */

#ifdef HAL_RTC_MODULE_ENABLED
 #include "stm32f4xx_hal_rtc.h"
#endif /* HAL_RTC_MODULE_ENABLED */

#ifdef HAL_SAI_MODULE_ENABLED
 #include "stm32f4xx_hal_sai.h"
#endif /* HAL_SAI_MODULE_ENABLED */

#ifdef HAL_SD_MODULE_ENABLED
 #include "stm32f4xx_hal_sd.h"
#endif /* HAL_SD_MODULE_ENABLED */

#ifdef HAL_MMC_MODULE_ENABLED
 #include "stm32f4xx_hal_mmc.h"
#endif /* HAL_MMC_MODULE_ENABLED */

#ifdef HAL_SPI_MODULE_ENABLED
 #include "stm32f4xx_hal_spi.h"
#endif /* HAL_SPI_MODULE_ENABLED */

#ifdef HAL_TIM_MODULE_ENABLED
 #include "stm32f4xx_hal_tim.h"
#endif /* HAL_TIM_MODULE_ENABLED */

#ifdef HAL_UART_MODULE_ENABLED
 #include "stm32f4xx_hal_uart.h"
#endif /* HAL_UART_MODULE_ENABLED */

#ifdef HAL_USART_MODULE_ENABLED
 #include "stm32f4xx_hal_usart.h"
#endif /* HAL_USART_MODULE_ENABLED */

#ifdef HAL_IRDA_MODULE_ENABLED
 #include "stm32f4xx_hal_irda.h"
#endif /* HAL_IRDA_MODULE_ENABLED */

#ifdef HAL_SMARTCARD_MODULE_ENABLED
 #include "stm32f4xx_hal_smartcard.h"
#endif /* HAL_SMARTCARD_MODULE_ENABLED */

#ifdef HAL_WWDG_MODULE_ENABLED
 #include "stm32f4xx_hal_wwdg.h"
#endif /* HAL_WWDG_MODULE_ENABLED */

#ifdef HAL_PCD_MODULE_ENABLED
 #include "stm32f4xx_hal_pcd.h"
#endif /* HAL_PCD_MODULE_ENABLED */

#ifdef HAL_HCD_MODULE_ENABLED
 #include "stm32f4xx_hal_hcd.h"
#endif /* HAL_HCD_MODULE_ENABLED */

#ifdef HAL_DSI_MODULE_ENABLED
 #include "stm32f4xx_hal_dsi.h"
#endif /* HAL_DSI_MODULE_ENABLED */

#ifdef HAL_QSPI_MODULE_ENABLED
 #include "stm32f4xx_hal_qspi.h"
#endif /* HAL_QSPI_MODULE_ENABLED */

#ifdef HAL_CEC_MODULE_ENABLED
 #include "stm32f4xx_hal_cec.h"
#endif /* HAL_CEC_MODULE_ENABLED */

#ifdef HAL_FMPI2C_MODULE_ENABLED
 #include "stm32f4xx_hal_fmpi2c.h"
#endif /* HAL_FMPI2C_MODULE_ENABLED */

#ifdef HAL_SPDIFRX_MODULE_ENABLED
 #include "stm32f4xx_hal_spdifrx.h"
#endif /* HAL_SPDIFRX_MODULE_ENABLED */

#ifdef HAL_DFSDM_MODULE_ENABLED
 #include "stm32f4xx_hal_dfsdm.h"
#endif /* HAL_DFSDM_MODULE_ENABLED */

#ifdef HAL_LPTIM_MODULE_ENABLED
 #include "stm32f4xx_hal_lptim.h"
#endif /* HAL_LPTIM_MODULE_ENABLED */

/* Exported macro ------------------------------------------------------------*/
/* 断言相关部分--JR*/
/* --JR
如果定义了USE_FULL_ASSERT
assert_param这个宏用于函数的参数检查 如果expr表达式的值为假（false），就会调用assert_failed函数
  并且将当前源文件的名称（通过__FILE__宏获取）和出错的源代码行号（通过__LINE__宏获取）传递给assert_failed函数
如果没有定义
  这意味着如果进行了参数检查，即使表达式为假也不会做任何事情，相当于关闭了断言功能
*/
#ifdef  USE_FULL_ASSERT
/**
  * @brief  The assert_param macro is used for function's parameters check.
  * @param  expr: If expr is false, it calls assert_failed function
  *         which reports the name of the source file and the source
  *         line number of the call that failed.
  *         If expr is true, it returns no value.
  * @retval None
  */
  #define assert_param(expr) ((expr) ? (void)0U : assert_failed((uint8_t *)__FILE__, __LINE__))
/* Exported functions ------------------------------------------------------- */
  void assert_failed(uint8_t* file, uint32_t line);
#else
  #define assert_param(expr) ((void)0U)
#endif /* USE_FULL_ASSERT */

/* 头文件保护结束标志--JR*/
#ifdef __cplusplus
}
#endif

#endif /* __STM32F4xx_HAL_CONF_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
