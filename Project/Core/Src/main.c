/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "oled.h"
#include "MPU6050.h"
#include "string.h"
#include "stdio.h"
#include "dwt_delay.h"
#include "Ultrasonic.h"
#include <math.h>
#include "motor.h"
#include <encoder.h>
#include "vofa.h"
#include "pid.h"
#include "BluetoothControl.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// 为了让中断也可以用，声明放在main函数外

extern volatile float accel_angle;  // 加速度计获取的角度
extern volatile float gyro_angle;   // 陀螺仪获取的角度
extern volatile float fused_angle;  // 融合后的角度
extern float gyro_x_speed;          // x轴的角速度
extern float gyro_z_speed;          // z轴的角速度

extern volatile uint8_t isFinish;  // 测量完成标志位
extern volatile uint16_t count;  // 定时器的值（高电平持续时间）

int Encoder_Left = 0;      // 左轮速度
int Encoder_Rigth = 0;     // 右轮速度

uint8_t System_Ready = 0; // 系统就绪标志位

uint8_t oled_task_stage = 0;   // OLED状态

extern VofaData g_vofa_frame;  // VOFA+公告板
extern float mechanical_balance_angle;  // 机械中值
extern volatile uint8_t IsParse ;           // 解析标识位
extern uint8_t Process_buffer[RX_BUFFER_SIZE + 1]; // 解析缓冲区
extern volatile uint8_t IsFall;           // 倒地标识位
extern float speed_filter_old;  // 经过滤波后的速度
extern volatile int target_speed;  // 目标速度
extern volatile int turn_cmd;  // 转向指令

extern uint32_t last_heartbeat_tick; // 上次心跳时间戳

#define CONTROL_TIMEOUT 500 // 控制指令超时时间，单位：毫秒

// uint8_t isvofa = 0;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  /*====================== MPU6050初始化 ======================*/
  MPU6050_Init();
  HAL_Delay(100);   // 等待传感器稳定
  // 上电静止校准
  MPU6050_Calibrate_Gyro();
  // 让陀螺仪初始角度等于加速度计角度
  MPU6050_Get_AccelAngle(); 
  gyro_angle = accel_angle; 
  fused_angle = accel_angle;
  // 允许滴答定时器中断开始工作
  System_Ready = 1;    // 不然在校准时，中断已经偷偷运行很久了

  /*====================== OLED初始化 ======================*/
  OLED_Init();  // 初始化oled屏幕
  OLED_Clear();  // 清屏
  // --- 打印固定不动的标签 (静态背景) ---
  // Line 0: "Angle:" (16号字体, 6个字符占 6*8=48 像素)
  OLED_ShowString(0, 0, (uint8_t*)"Angle:", 16); 

  // Line 3: 分隔线
  OLED_ShowString(0, 3, (uint8_t*)"--------------", 8); 

  // Line 4: "Dis:" 和 "cm" 
  OLED_ShowString(0, 4, (uint8_t*)"Dis:", 12);   
  OLED_ShowString(78, 4, (uint8_t*)"cm", 12);    // 24 + (9*6) = 78 (预留显示位)

  // Line 5: "L:" 和 "R:"
  OLED_ShowString(0, 6, (uint8_t*)"L:", 12);     
  OLED_ShowString(64, 6, (uint8_t*)"R:", 12);    // 在屏幕中间 64 像素处开始打印 R:

  /*====================== 超声波模块初始化 ======================*/  
  uint8_t isObstacle = 0;  // 0：没有障碍物，1：有障碍物 
  float distance;  // 障碍物距离

  /*====================== 电机驱动模块初始化 ======================*/  
  // 启动定时器1的通道1和4的PWM脉冲
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  // 关闭电机
  Set_Motor_Speed(0, 0);

  /*====================== 编码器测速模块初始化 ======================*/ 
  // 启动定时器2和4的所有通道
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

  /*====================== VOFA+模块初始化 ======================*/ 
  VOFA_Init();

  /*====================== PID模块初始化 ======================*/ 
  PID_Init();
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  char buf[20]; // 缓冲区，用于存放格式化后的字符串
  /*====================== 时间戳初始化 ======================*/ 
  uint32_t last_60mstime = HAL_GetTick();   // 一级时间戳
  uint32_t last_500mstime = HAL_GetTick();  // 二级时间戳
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  while (1)
  {
    uint32_t now = HAL_GetTick();

    // if (isvofa == 1)
    // {
    //   g_vofa_frame.Reality_angle = fused_angle;
    //   g_vofa_frame.expect_angle = mechanical_balance_angle;
    //   VOFA_SendDta();
    //   isvofa = 0;
    // }

    /*====================== 优先级1：超声波 ======================*/
    if (now - last_60mstime >= 60)
    {
      /*====================== 超声波测距 ======================*/
      if (isFinish == 0)  // 如果上一次没有算完，这一次就不发
      {
        Get_Distance();
      }
      last_60mstime = now;  // 更新时间戳
     
    }

    /*====================== 优先级2：OLED显示 ======================*/
    if (now - last_500mstime >= 125)
    {
      last_500mstime = now;
      switch (oled_task_stage) 
      {
        /*====================== 角度数据可视化 ======================*/
        case 0:
            // "Angle:" 占 48 像素，所以从 48 开始显示
            OLED_ShowSignedFloat(48, 0, fused_angle, 3, 2, 16);
            oled_task_stage = 1;
            break;
        /*====================== 速度数据可视化 ======================*/
        case 1:
            // "L:" 占 12 像素 -> x = 12
            OLED_ShowSignedNum(12, 6, Encoder_Left, 4, 12);
            // "R:" 在 64 像素处开始，标签占 12 像素 -> x = 64+12 = 76
            OLED_ShowSignedNum(76, 6, Encoder_Rigth, 4, 12);
            oled_task_stage = 2;
            break;
        /*====================== 障碍数据可视化 ======================*/
        case 2:
            OLED_ShowSignedFloat(24, 4, distance, 3, 1, 12);
            oled_task_stage = 3;
            break;
        case 3:
            // 其他显示...
            oled_task_stage = 0; // 回到第一步
            break;
      }
    }     

    /*====================== 避障逻辑 ======================*/
    if (isFinish == 1)
    {
      distance = count * 0.017;  // 获取平衡车离障碍物的距离
      if (distance > 0 && distance <= 30) isObstacle = 1;  
      else isObstacle = 0;
      isFinish = 0;  // 清除标识位
    }

    /*====================== 蓝牙指令处理 ======================*/
    if (IsParse == 1)
    {
      IsParse = 0;
      // VOFA_Parse_Command((char *)Process_buffer);  // 解析VOFA+数据包
      // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
      Bluetooth_Parse_Binary((uint8_t *)Process_buffer);  // 解析蓝牙数据包
    }

    /*====================== 软件看门狗 ======================*/
    if (HAL_GetTick() - last_heartbeat_tick > CONTROL_TIMEOUT)
    {
      // 超时，认为通信丢失
      target_speed = 0;  // 目标速度清零
      turn_cmd = 0;     // 转向指令清零
    }


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
}

/* USER CODE BEGIN 4 */

// 滴答定时器的中断回调
void HAL_SYSTICK_Callback(void)
{
  static uint32_t Htime = 0;  // static告诉编译器：这个变量只初始化一次
  static uint32_t Stime = 0;
  static float target_angle_from_speed = 0; // 缓存速度环输出的目标角度

  if (!System_Ready) return;  // 系统未就绪，直接返回

  Htime++;  // 5ms计数器
  Stime++;  // 40ms计数器

  if (Stime >= 40)
  {
    /*=================== 编码器测速 ===================*/
    // 数据波动大，做低通滤波
    // 大概率相信旧数据是因为极短时间内速度变化不会太大
    Encoder_Left = Read_Speed(&htim2);
    Encoder_Rigth = -1 * Read_Speed(&htim4);

    /*=================== 速度环PI控制器 ===================*/
    target_angle_from_speed = Speed_Control(Encoder_Left, Encoder_Rigth, fused_angle);

    Stime = 0;
  }

  if (Htime >= 5)
  {
    /*=================== 姿态解算 ===================*/
    MPU6050_get_FusedAngle_Optimized(0.005f);
    // isvofa = 1;
    
    if (fabs(fused_angle) > 40.0f)        IsFall = 1;
    else if (fabs(fused_angle) < 5.0f)    IsFall = 0;

    /*=================== 直立环PD控制器 ===================*/
    float pure_gyro_x_speed = gyro_x_speed - KalmanX.bias;  // 纯净的、去除了温漂的角速度
    int Upright_pwm = Upright_Control(fused_angle, pure_gyro_x_speed, target_angle_from_speed);
    
    /*=================== 转向环控制 ===================*/
    int Turn_pwm = Turn_Control(turn_cmd, gyro_z_speed);

    if (IsFall == 1) 
    {
      Set_Motor_Speed(0, 0); // 倒地强制关机
      target_angle_from_speed = 0;  // 把缓存的速度环输出值清零
    } 
    else 
    {
      // 这里的 target_angle_from_speed 是 40ms 更新一次的全局/静态变量
      Set_Motor_Speed(Upright_pwm + Turn_pwm, Upright_pwm - Turn_pwm);
    }
    
    Htime = 0;  // 复位计数器
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
