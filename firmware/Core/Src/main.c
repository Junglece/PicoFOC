/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Foc.h"
#include "drv_tim.h"
#include "drv_can.h"
#include "drv_adc.h"
#include "can_proto.h"
#include "motor_msg.h"
#include "uart_proto.h"
#include "motor_ctrl.h"
#include "angle_as5600.h"
#include "pwm_3phase.h"
#include "motor_config.h"
#include "flash_storage.h"
#include "led_indicator.h"
#include "observer_diff.h"       /* 默认使用差分法观测器 */
/* 换龙伯格时取消下面这行的注释，同时注释掉上面 #include "observer_diff.h" */
/* #include "observer_luenberger.h" */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/** @brief FOC 控制频率 (Hz) —— 观测器和 FOC 共用的唯一频率源头 */
#define FOC_RATE_HZ  1000.0f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float vofa_message[8];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void VOFA_SendDebug(void);
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
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_CAN_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  /* ================================================================
   *  第 1 步：电机硬件配置
   *
   *  MotorConfig_t 集中管理极对数、母线电压、控制频率，
   *  同时注入 FOC 控制器和 PWM 驱动，消除重复配置。
   * ================================================================ */
  MotorConfig_t cfg = {
      .pole_pairs  = 7,
      .bus_voltage = 12.0f,
      .rate_hz     = FOC_RATE_HZ,
  };

  /* ================================================================
   *  第 2 步：创建硬件接口实例
   *
   *  AS5600_Create()  →  填充 AngleSensor_t，封装 I2C
   *  PWM3Phase_Create() → 填充 PwmOutput_t，封装 TIM3 PWM
   *                      set_duty 接收归一化值，不再需要母线电压
   *
   *  换编码器 / 换 PWM 方案 = 只改下面两行
   * ================================================================ */
  HAL_Delay(100);
  AngleSensor_t sensor = AS5600_Create(&hi2c1);
  PwmOutput_t   pwm    = PWM3Phase_Create(&htim3);
  HAL_Delay(100);

  /* ================================================================
   *  第 3 步：创建非易失存储接口 + 观测器 + 初始化 FOC 控制器
   *
   *  FlashStorage_Create 封装 STM32 Flash 擦写，
   *  换其它介质（EEPROM/FRAM）只需换一个工厂函数。
   *
   *  观测器（Observer）负责将原始角度转为滤波后角度和估计速度。
   *  默认用差分法（ObserverDiff_Create），换龙伯格只需：
   *     ObserverLuenberger_Config obs_cfg = { .rate_hz = FOC_RATE_HZ, ... };
   *     Observer_t angle_obs = ObserverLuenberger_Create(&obs_cfg);
   *
   *  FOC_Init(&Motor, &sensor, &pwm, &cfg, &flash, &angle_obs)
   *  所有电机参数通过 cfg 传入，存储接口通过 flash 传入。
   *
   *  ⚠ 频率改一个地方就够了：改 #define FOC_RATE_HZ，观测器和 FOC 都同步。
   * ================================================================ */
  NvStorage_t flash = FlashStorage_Create(0x0800FC00, 0xF0C0F0C0, 1024);

  /* 创建角度/速度观测器（默认用差分法，换龙伯格就把下一行换成 ObserverLuenberger_Create） */
  Observer_t angle_obs = ObserverDiff_Create(FOC_RATE_HZ);

  FOC_Init(&Motor, &sensor, &pwm, &cfg, &flash, &angle_obs);

  /* ================================================================
   *  第 4 步：设置 PID 参数
   *
   *  PID 已做时间归一化（integral *= dt），ki 值自动适配频率。
   *  你传入原来的 ki 值即可，效果不变。
   *
   *  原代码对照：
   *    pid_controller_create(&FOC_1.FOC_S_PID, 0.01, 0.004, 0, 1000, 1000, 6);
   *    → FOC_SetSpdPID(&Motor, 0.01, 0.004, 0, 6);
   *
   *    pid_controller_create(&FOC_1.FOC_P_PID, 5, 0, 0, 1000, 2000, 2000);
   *    → FOC_SetPosPID(&Motor, 5, 0, 0, 2000);
   * ================================================================ */
  FOC_SetSpdPID(&Motor, 0.01f,  0.004f, 0.0f, 6.0f);     /* 串级速度环（位置环内环） */
  FOC_SetPosPID(&Motor, 5.0f,   0.0f,   0.0f, 2000.0f);   /* 位置环 PD */
  FOC_SetSpdExtPID(&Motor, 0.05f, 0.01f, 0.0f, 6.0f);    /* 独立速度环 PI（速度模式专用） */

  /* ================================================================
   *  第 5 步：初始化驱动层 + 协议层
   *
   *  【初始化顺序很重要！】
   *  CAN_Init()       : 配置滤波器 + 启动 CAN 硬件
   *  MotorMsg_Init()  : 先初始化消息总线（消息总线准备就绪前，通信回调不能写它）
   *  CANProto_Init()  : 注册 CAN 回调（收到 CAN 帧 → 调 MotorMsg_OnFrameReceived）
   *  UARTProto_Init() : 使能 UART RX 中断（收到 UART 帧 → 调 MotorMsg_OnFrameReceived）
   *  TIM_Init()       : 最后启动时基中断（TIM1 硬件 20kHz → 软件 20 分频 → 1kHz FOC 循环）
   * ================================================================ */
  CAN_Init();
  HAL_Delay(100);
  MotorMsg_Init();    /* 先初始化消息总线 */
  CANProto_Init(1);    /* 再注册 CAN 回调，电机ID=1 → RX=0x001 TX=0x101 */
  UARTProto_Init();   /* 再使能 UART 接收 */
  TIM_Init();

  /* LED 初始状态：待机心跳（满亮度闪烁），覆盖默认的常灭 */
  LED_SetPattern(LED_PATTERN_HEARTBEAT);
  LED_SetBrightness(100);

  /* ================================================================
   *  第 6 步：注册控制回调 → 串起整个控制链
   *
   *  TIM1 中断 → Motor_Loop() → FOC 流水线
   *
   *  如果想用 ADC 电流采样（预留）：
   *    ADC_DMA_Mode_Init();
   * ================================================================ */
  TIM_RegisterCallback(Motor_Loop);

  /* 电流采样 DMA（ADC 预留，暂未使用） */
  ADC_DMA_Mode_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      VOFA_SendDebug();
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
 * @brief  通过 UART1 发送调试数据到 VOFA+ 串口示波器
 *
 * 帧格式（just-float 引擎）：
 *   8 × float (32 bytes) + 4-byte tail {0x00,0x00,0x80,0x7f}
 *
 * 通道映射：
 *   ch0 = 机械角度 (deg)    ch1 = 目标角度 (deg)
 *   ch2 = 运行模式           ch3 = 目标值 (rad/rpm/Nm)
 *   ch4 = 实际转速 (rpm)     ch5 = Uq 输出电压 (V)
 */
static void VOFA_SendDebug(void)
{
    vofa_message[0] = Motor.mech_angle;              /* ch0：机械角度 (deg)          */
    vofa_message[1] = Motor.target_angle;            /* ch1：目标角度 (deg)          */
    vofa_message[2] = (float)MotorMsg_GetLastMode(); /* ch2：运行模式                */
    vofa_message[3] = MotorMsg_GetLastTarget();      /* ch3：目标值 (rad/rpm/Nm)     */
    vofa_message[4] = Motor.speed;                   /* ch4：实际转速 (rpm)          */
    vofa_message[5] = Motor.Uq;                      /* ch5：Uq 输出电压 (V)         */
    vofa_message[6] = 0;
    vofa_message[7] = 0;

    UARTProto_SendVOFA(vofa_message, 8);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
