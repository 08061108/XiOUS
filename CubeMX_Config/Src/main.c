/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

RTC_HandleTypeDef hrtc;

SD_HandleTypeDef hsd;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim11;
TIM_HandleTypeDef htim13;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

SRAM_HandleTypeDef hsram1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* 模块初始化函数声明--JR*/
void SystemClock_Config(void);           /* 配置系统的时钟--JR*/
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_IWDG_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM13_Init(void);
static void MX_TIM11_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI2_Init(void);
static void MX_FSMC_Init(void);
static void MX_UART4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI3_Init(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();   /* 初始化HAL库，包括复位所有外设、初始化闪存接口和SysTick定时器等基本操作--JR*/


  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_RTC_Init();
  MX_IWDG_Init();
  MX_TIM14_Init();
  MX_TIM13_Init();
  MX_TIM11_Init();
  MX_SDIO_SD_Init();
  MX_TIM2_Init();
  MX_SPI2_Init();
  MX_FSMC_Init();
  MX_UART4_Init();
  MX_USART2_UART_Init();
  MX_CAN1_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)    /* 进入无限循环--JR*/
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};   /* RCC_OscInitStruct--结构体类型的变量，用于存储与振荡器初始化相关的参数 可能包含诸如振荡器类型、各个振荡器（如HSE、LSE、LSI等）的状态、PLL（锁相环）的相关设置等--JR*/
  
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};   /* 用于存储与总线时钟初始化相关的参数 例如总线的时钟类型（如HCLK、SYSCLK等）--JR*/
  
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};   /* 专门用于存储与外设时钟初始化相关的参数，在这里主要是针对RTC外设时钟的选择--JR*/

  
  /** Configure the main internal regulator output voltage
  */
  /* --JR
    配置电源相关的内容
     __HAL_RCC_PWR_CLK_ENABLE()--用于使电源能控制寄存器的时钟
     __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1)--是设置电压调节器的输出电压为特定值
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  
  
  /** Initializes the CPU, AHB and APB busses clocks
  */
  /* --JR
   设置振荡器类型为同时包含低速内部振荡器（LSI）、高速外部振荡器（HSE）和低速外部振荡器（LSE）
   通过按位或操作将不同的振荡器类型组合在一起
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE
                              |RCC_OSCILLATORTYPE_LSE;
  
  /* 分别对HSE、LSE、LSI的状态进行设置，这里将它们都设置为开启状态--JR*/
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;  /* 开启PLL--JR*/
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE; /* PLL的时钟源是HSE--JR*/
  
  /* 对PLL倍频系数等相关参数的设置--JR*/
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
 
  /* 如果对振荡器初始化配置失败，会调用Error_Handler()函数进行错误处理--JR*/
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  
  /** Initializes the CPU, AHB and APB busses clocks
  */
  /* 设置总线的时钟类型包含HCLK（高级高速总线时钟）、SYSCLK（系统时钟）、PCLK1（APB1外设时钟）和PCLK2（APB2外设时钟）--JR*/
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; /* 表明系统时钟源为PLL时钟--JR*/
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;        /* AHB总线的分频系数为1--JR*/
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;         /* 表示APB1总线的分频系数为4，即APB1时钟是HCLK的四分之一--JR*/
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;         /* 表示APB2总线的分频系数为2--JR*/

  /* 如果总线时钟配置失败，会调用Error_Handler()函数--JR*/
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;  /* 表示要配置的外设时钟为RTC时钟--JR*/
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;  /* RTC时钟源选择为LSE（低速外部振荡器）--JR*/
  
  /* RTC外设时钟配置失败，会调用Error_Handler()函数--JR*/
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */

 /* 初始化CAN1（Controller Area Network 1）通信接口的函数 CAN是一种在汽车和其他嵌入式系统中广泛使用的串行通信协议--JR*/
static void MX_CAN1_Init(void)   /* MX_CAN1_Init(void)--一个静态函数，用于初始化CAN1接口--JR*/
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  /* hcan1--CAN_HandleTypeDef类型的结构体变量，用于存储CAN1的配置信息和状态*/
  hcan1.Instance = CAN1;                     /* 将hcan1的实例设置为CAN1，表示这个结构体用于配置和控制CAN1外设--JR*/
  hcan1.Init.Prescaler = 16;                 /* 设置CAN时钟预分频器为16。这意味着CAN的时钟频率将是APB1时钟频率的1/16--JR*/
  hcan1.Init.Mode = CAN_MODE_NORMAL;         /* 设置CAN工作在正常模式--JR*/
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;    /* 设置同步跳转宽度（SJW）为1个时间量子（TQ） SJW定义了在同步过程中允许的时钟周期的最大变化量--JR*/
  hcan1.Init.TimeSeg1 = CAN_BS1_1TQ;         /* 设置时间段1（BS1）为1个TQ BS1是发送节点在发送位同步期间等待的时间--JR*/
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;         /* 设置时间段2（BS2）为1个TQ BS2是在重新同步之后发送节点发送位的时间--JR*/
  hcan1.Init.TimeTriggeredMode = DISABLE;    /* 禁用时间触发模式--JR*/
  hcan1.Init.AutoBusOff = DISABLE;           /* 禁用自动总线关闭功（当CAN节点检测到错误时，它通常会自动关闭总线以避免进一步的错误传播）--JR*/
  hcan1.Init.AutoWakeUp = DISABLE;           /* 禁用自动唤醒功能--JR*/
  hcan1.Init.AutoRetransmission = DISABLE;   /* 禁用自动重传功能--JR*/
  hcan1.Init.ReceiveFifoLocked = DISABLE;    /* 禁用接收FIFO锁定功能--JR*/
  hcan1.Init.TransmitFifoPriority = DISABLE; /* 禁用发送FIFO优先级功能--JR*/

  /* 初始化CAN外设失败，会调用Error_Handler()函数进行错误处理--JR*/
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  

  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
 /* 对I2C1（Inter - Integrated Circuit 1）进行初始化设置--JR*/
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  
  hi2c1.Instance = I2C1;                                  /* hi2c1--存储I2C1的相关配置信息和状态--JR*/
  hi2c1.Init.ClockSpeed = 100000;                         /* 设置I2C1的工作时钟频率为100kHz--JR*/
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;                 /* 定义了I2C时钟信号的高电平和低电平时间的比例关系--JR*/
  hi2c1.Init.OwnAddress1 = 0;                             /* 设置I2C1设备的从地址（如果作为设备使用）--JR*/
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;    /* 确定I2C地址的编码模式为7位地址模式--JR*/
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;   /* 表示禁用双地址模式 如果启用双地址模式，设备可以有两个不同的从地址--JR*/
  hi2c1.Init.OwnAddress2 = 0;                             /* 当双地址模式被禁用时，这个地址设置为0没有实际意义；如果双地址模式启用，这是第二个从地址--JR*/
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;   /*禁用通用呼叫模式（用于向所有连接到总线且支持通用呼叫的设备发送广播消息）--JR*/
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;       /* 禁用无拉伸模式 无拉伸模式下，主设备不需要等待从设备的时钟拉伸信号--JR*/
  
  /* 初始化失败，会调用Error_Handler()函数进行错误处理--JR*/
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */

static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_4;  /* 设置预分频器为4。预分频器用于降低IWDG的计数时钟频率--JR*/
  hiwdg.Init.Reload = 4095;                 /* 设置计数器的重载值，当计数器减到这个值时会重新加载初始值并继续计数--JR*/
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;              /* 设置小时格式为24小时制--JR*/
  hrtc.Init.AsynchPrediv = 127;                          /* 异步预分频器的值--JR*/
  hrtc.Init.SynchPrediv = 255;                           /* 同步预分频器的值。这两个预分频器共同决定了RTC的时钟频率--JR*/
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;                 /* 禁用RTC的输出功能--JR*/
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;   /* 如果输出功能启用，设置输出电平极性为高--JR*/
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;      /* 设置输出类型为开漏极输出--JR*/
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)  /* SDIO（安全数字输入输出）*/
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;                      /* 设置SDIO时钟信号的上升沿用于数据传输 --JR*/
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;                 /* 禁用时钟旁路功能--JR*/
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;          /* 禁用时钟节能模式--JR*/
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;                              /* 设置初始总线宽度为1位--JR*/
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;/* 禁用硬件流控制--JR*/
  hsd.Init.ClockDiv = 0;                                            /* 设置时钟分频系数为0，具体值取决于系统时钟配置--JR*/
  if (HAL_SD_Init(&hsd) != HAL_OK)
  {
    Error_Handler();
  }
  
  /* 是否成功配置了SDIO接口以支持4位宽总线操作--JR*/
  if (HAL_SD_ConfigWideBusOperation(&hsd, SDIO_BUS_WIDE_4B) != HAL_OK)
  {
    Error_Handler();
  }
  
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  /* hspi1是一个SPI_HandleTypeDef 类型的结构体变量，用于存储SPI外设的配置和状态信息--JR*/
  hspi1.Instance = SPI1;                                    /* 初始化SPI1外设--JR*/
  hspi1.Init.Mode = SPI_MODE_MASTER;                        /* 设置SPI的工作模式 SPI_MODE_MASTER 表示主模式，即STM32作为主设备控制通信--JR*/
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;              /* Direction: 设置数据传输方向 SPI_DIRECTION_2LINES 表示全双工通信，即同时支持发送和接收--JR*/
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;                  /* DataSize: 设置每次传输的数据位数 SPI_DATASIZE_8BIT 表示每次传输8位数据--JR*/
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;                /* CLKPolarity: 设置时钟极性 SPI_POLARITY_LOW 表示空闲时时钟线为低电平--JR*/
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;                    /* CLKPhase: 设置时钟相位 SPI_PHASE_1EDGE 表示在时钟的第一个边沿（上升沿）采样数据--JR*/
  hspi1.Init.NSS = SPI_NSS_SOFT;                            /* NSS: 设置片选（Slave Select）信号的管理方式 SPI_NSS_SOFT 表示软件管理片选信号，由软件控制片选的拉高和拉低--JR*/
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;   /* BaudRatePrescaler: 设置波特率预分频器 SPI_BAUDRATEPRESCALER_2 表示预分频系数为2，影响SPI的通信速率--JR*/
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;                   /* FirstBit: 设置数据传输的起始位 SPI_FIRSTBIT_MSB 表示最高有效位（MSB）先传输--JR*/
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;                   /* TIMode: 设置是否启用TI模式 SPI_TIMODE_DISABLE 表示禁用TI模式--JR*/
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;   /* CRCCalculation: 设置是否启用CRC校验 SPI_CRCCALCULATION_DISABLE 表示禁用CRC校验--JR*/
  hspi1.Init.CRCPolynomial = 10;                            /* CRCPolynomial: 设置CRC多项式值 如果启用了CRC校验，这里指定使用的多项式--JR*/
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)           
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;                                        /* Instance: 指定要初始化的SPI外设实例 这里指定为 SPI2，表示初始化SPI2外设--JR*/
  hspi2.Init.Mode = SPI_MODE_MASTER;                            /* Mode: 设置SPI的工作模式 SPI_MODE_MASTER: 主模式，STM32作为主设备控制通信--JR*/
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;                  /* Direction: 设置数据传输方向 SPI_DIRECTION_2LINES: 双向全双工通信，即同时支持发送和接收--JR*/
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;                      /* DataSize: 设置每次传输的数据位数 SPI_DATASIZE_8BIT: 每次传输8位数据--JR*/
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;                    /* CLKPolarity: 设置时钟极性 SPI_POLARITY_LOW: 空闲时（非传输状态）时钟线为低电平--JR*/
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;                        /* CLKPhase: 设置时钟相位 SPI_PHASE_1EDGE: 在时钟的第一个边沿（上升沿）采样数据--JR*/
  hspi2.Init.NSS = SPI_NSS_SOFT;                                /* NSS: 设置片选（Slave Select）信号的管理方式 SPI_NSS_SOFT: 软件管理片选信号，由软件控制片选的拉高和拉低--JR*/
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;       /* BaudRatePrescaler: 设置波特率预分频器，影响SPI的通信速率 SPI_BAUDRATEPRESCALER_2: 预分频系数为2，具体波特率取决于系统时钟--JR*/
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;                       /* FirstBit: 设置数据传输的起始位 SPI_FIRSTBIT_MSB: 最高位（Most Significant Bit, MSB）先传输--JR*/
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;                       /* TIMode: 设置是否启用TI模式（双向传输模式） SPI_TIMODE_DISABLE: 禁用TI模式--JR*/
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;       /* CRCCalculation: 设置是否启用CRC校验 SPI_CRCCALCULATION_DISABLE: 禁用CRC校验--JR*/
  hspi2.Init.CRCPolynomial = 10;                                /* CRCPolynomial: 设置CRC多项式值。如果启用了CRC校验，这里指定使用的多项式--JR*/
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;                                    /* 将SPI3外设的地址赋值给句柄结构体hspi3的Instance成员--JR*/
  hspi3.Init.Mode = SPI_MODE_MASTER;                        /* 设置 SPI 的工作模式为主设备（Master）--JR*/
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;              /* 设置 SPI 的数据传输方向为双线全双工（即同时有发送和接收线路）--JR*/
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;                  /* DataSize: 设置每次传输的数据位数为8位--JR*/
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;                /* CLKPolarity: 设置 SPI 时钟的空闲状态为低电平--JR*/
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;                    /* CLKPhase: 设置 SPI 数据在时钟的第一个边沿（上升沿或下降沿）进行采样--JR*/
  hspi3.Init.NSS = SPI_NSS_SOFT;                            /* NSS: 设置片选信号（NSS）为软件控制，即由软件通过 GPIO 控制，而不是硬件自动管理--JR*/
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;   /* BaudRatePrescaler: 设置 SPI 的波特率预分频器为2，影响 SPI 的时钟频率--JR*/
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;                   /* 设置数据传输时**最高有效位（MSB）**先发送--JR*/
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;                   /* 禁用TI模式（双线单向模式），确保使用标准的 SPI 模式--JR*/
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;   /* CRCCalculation: 禁用**循环冗余校验（CRC）**计算--JR*/
  hspi3.Init.CRCPolynomial = 10;                            /* CRCPolynomial: 如果启用了 CRC，这里设置 CRC 多项式为 10--JR*/
  if (HAL_SPI_Init(&hspi3) != HAL_OK)                       
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)           /* 定义了一个静态函数MX_TIM2_Init，用于初始化TIM2定时器--JR*/
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  /* 定义了两个配置结构体变量，用于设置定时器的时钟源和主从模式--JR*/
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};    /* TIM_ClockConfigTypeDef 是一个结构体类型，用于配置定时器的时钟源--JR*/
  TIM_MasterConfigTypeDef sMasterConfig = {0};        /* TIM_MasterConfigTypeDef 是一个结构体类型，用于配置定时器的主从模式和触发输出--JR*/

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */

  /* 设置了定时器的初始化参数--JR*/
  htim2.Instance = TIM2;                                             /* 设置定时器实例为TIM2--JR*/
  htim2.Init.Prescaler = 0;                                          /* 预分频器值，用于降低定时器的时钟频率--JR*/
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;                       /* CounterMode：计数模式，这里设置为向上计数模式--JR*/
  htim2.Init.Period = 0;                                             /* Period：自动重装载寄存器的值，定时器计数到这个值时会溢出--JR*/
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;                 /* ClockDivision：时钟分频因子，这里设置为不进行分频--JR*/
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;     /* AutoReloadPreload：自动重装载预装载寄存器的使能状态，这里设置为禁用--JR*/
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  /* 设置定时器的时钟源为内部时钟，并通过HAL_TIM_ConfigClockSource函数应用配置。如果配置失败，则调用Error_Handler函数处理错误--JR*/
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;                 /* MasterOutputTrigger：主输出触发源，这里设置为复位触发--JR*/
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;        /* MasterSlaveMode：主从模式，这里设置为禁用--JR*/
  
  /* 通过HAL_TIMEx_MasterConfigSynchronization函数应用主从模式的配置。如果配置失败，则调用Error_Handler函数处理错误--JR*/
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 0;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;                     /* 设置定时器的计数模式为向上计数（从0计数到自动重装载值）--JR*/
  htim11.Init.Period = 0;                                           /* 设置定时器的自动重装载寄存器的值（周期）--JR*/
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;               /* 设置定时器的时钟分频因子。这影响定时器的计数时钟频率--JR*/
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;   /* 设置自动重装载预装载寄存器的使能状态。禁用预装载寄存器意味着自动重装载值可以立即生效，而无需等待更新事件--JR*/
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

}

/**
  * @brief TIM13 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM13_Init(void)
{

  /* USER CODE BEGIN TIM13_Init 0 */

  /* USER CODE END TIM13_Init 0 */

  /* USER CODE BEGIN TIM13_Init 1 */

  /* USER CODE END TIM13_Init 1 */
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 0;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 0;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM13_Init 2 */

  /* USER CODE END TIM13_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 0;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 0;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;                           /* 设置串口实例为UART4。huart4是一个UART_HandleTypeDef类型的结构体变量，用于存储UART4的配置和状态信息--JR*/
  huart4.Init.BaudRate = 115200;                     /* 设置串口的波特率（Baud Rate）为115200。波特率决定了数据传输的速度，单位是bps（比特每秒）--JR*/
  huart4.Init.WordLength = UART_WORDLENGTH_8B;       /* 设置每个数据帧的字长（Word Length）为8位--JR*/
  huart4.Init.StopBits = UART_STOPBITS_1;            /* 设置每个数据帧的停止位（Stop Bits）为1位--JR*/
  huart4.Init.Parity = UART_PARITY_NONE;             /* 设置数据帧的奇偶校验位（Parity）为无校验--JR*/
  huart4.Init.Mode = UART_MODE_TX_RX;                /* 设置串口的工作模式为同时支持发送（Transmit）和接收（Receive）--JR*/
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;       /* 设置硬件流控制（Hardware Flow Control）为无--JR*/
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;   /* 设置过采样率（Over Sampling）为16倍。过采样用于提高接收数据的可靠性，特别是在低波特率或噪声较大的环境中--JR*/
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};    /* 声明并初始化一个GPIO_InitTypeDef类型的结构体变量GPIO_InitStruct。该结构体用于存储GPIO引脚的配置参数--JR*/

  /* GPIO Ports Clock Enable */
  /* 启用各个GPIO端口的时钟。在STM32中，使用外设之前必须先启用其对应的时钟--JR*/
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  /* 设置GPIOG端口的第11和第14引脚为低电平（逻辑0）--JR*/
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_11|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pins : PG11 PG14 */
  /* 配置GPIOG端口的第11和第14引脚为推挽输出模式，速度为低速，无上下拉电阻--JR*/
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : PG13 */
  /* 配置GPIOG端口的第13引脚为复用功能推挽输出模式，速度为非常高，复用功能为以太网（ETH）--JR*/
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

}

/* FSMC initialization function */
static void MX_FSMC_Init(void)
{

  /* USER CODE BEGIN FSMC_Init 0 */

  /* USER CODE END FSMC_Init 0 */

  FSMC_NORSRAM_TimingTypeDef Timing = {0};      /* 声明并初始化一个FSMC_NORSRAM_TimingTypeDef类型的结构体变量Timing，用于配置FSMC的时序参数。初始化为全零--JR*/

  /* USER CODE BEGIN FSMC_Init 1 */

  /* USER CODE END FSMC_Init 1 */

  /** Perform the SRAM1 memory initialization sequence
  */
  hsram1.Instance = FSMC_NORSRAM_DEVICE;              /* 设置hsram1结构体中的Instance成员，指定要初始化的FSMC设备为FSMC_NORSRAM_DEVICE（即NOR/SRAM设备）--JR*/
  hsram1.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;     /* 设置hsram1结构体中的Extended成员，指定扩展设备为FSMC_NORSRAM_EXTENDED_DEVICE--JR*/
  /* hsram1.Init */
  hsram1.Init.NSBank = FSMC_NORSRAM_BANK3;                             /* 设置hsram1结构体中的Init子结构的NSBank成员，指定使用FSMC的非复用SRAM银行3--JR*/
  hsram1.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;          /* 禁用数据地址复用功能--JR*/
  hsram1.Init.MemoryType = FSMC_MEMORY_TYPE_SRAM;                      /* 设置内存类型为SRAM--JR*/
  hsram1.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_16;         /* 设置内存数据总线宽度为16位--JR*/
  hsram1.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;        /* 禁用突发访问模式--JR*/
  hsram1.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;      /* 设置等待信号的极性为低电平有效--JR*/
  hsram1.Init.WrapMode = FSMC_WRAP_MODE_DISABLE;                       /* 禁用环绕模式--JR*/
  hsram1.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;           /* 设置等待信号在WS（等待状态）之前激活--JR*/
  hsram1.Init.WriteOperation = FSMC_WRITE_OPERATION_ENABLE;            /* 启用写操作--JR*/
  hsram1.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;                   /* 禁用等待信号--JR*/
  hsram1.Init.ExtendedMode = FSMC_EXTENDED_MODE_DISABLE;               /* 禁用扩展模式--JR*/
  hsram1.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;       /* 禁用异步等待--JR*/
  hsram1.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;                   /* 禁用写突发模式--JR*/
  hsram1.Init.PageSize = FSMC_PAGE_SIZE_NONE;                          /* 设置页面大小为无分页--JR*/
  /* Timing */
  Timing.AddressSetupTime = 15;                 /* 设置地址建立时间为15个时钟周期--JR*/
  Timing.AddressHoldTime = 15;                  /* 设置地址保持时间为15个时钟周期--JR*/
  Timing.DataSetupTime = 255;                   /* 设置数据建立时间为255个时钟周期--JR*/
  Timing.BusTurnAroundDuration = 15;            /* 设置总线回转持续时间为15个时钟周期--JR*/
  Timing.CLKDivision = 16;                      /* 设置时钟分频因子为16--JR*/
  Timing.DataLatency = 17;                      /* 设置数据延迟为17个时钟周期--JR*/
  Timing.AccessMode = FSMC_ACCESS_MODE_A;       /* 设置访问模式为模式A--JR*/
  /* ExtTiming */

  /* --JR
  调用HAL_SRAM_Init函数初始化SRAM1内存。传入hsram1配置结构体、Timing时序结构体和NULL作为扩展参数。
  如果初始化失败（返回值不是HAL_OK），则调用Error_Handler函数
  */
  if (HAL_SRAM_Init(&hsram1, &Timing, NULL) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FSMC_Init 2 */

  /* USER CODE END FSMC_Init 2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  /* --JR
  定义错误处理函数Error_Handler。当初始化过程中发生错误时，该函数会被调用
  当前实现为进入无限循环，实际应用中可以扩展为记录错误日志、点亮LED指示灯等
  */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
