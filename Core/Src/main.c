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
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "fmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dm_motor_ctrl.h"
#include "bsp_can.h"
#include "lcd.h"
#include "keys.h"
#include "task.h"
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LED_ON  GPIO_PIN_RESET   /* LED 低电平点亮；若硬件为高有效改为 GPIO_PIN_SET */
#define LED_OFF GPIO_PIN_SET
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint32_t lcd_tick = 0;   /* last LCD refresh timestamp (ms), non-blocking 0.1s */
uint32_t ADC_Value = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* Periodic DM control-frame send.
 * TIM3 is configured by the user (see the init notes below); to start the
 * ~1 kHz control loop add:  MX_TIM3_Init();  HAL_TIM_Base_Start_IT(&htim3);  */

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
  MX_USART1_UART_Init();
  MX_FMC_Init();
  MX_TIM7_Init();
  MX_CAN1_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
    bsp_can_init();
    lcd_init();
    lcd_clear(BLACK);           /* 黑底，便于白字显示 */
    g_back_color = BLACK;       /* 字符背景色也设黑，刷新数字时能清掉旧像素 */
    lcd_show_string(10, 10, 200, 24, 24, "Ang:", WHITE);    /* 静态标签：摆杆角度 */
    lcd_show_string(10, 44, 200, 24, 24, "Pos:", WHITE);    /* 静态标签：电机位置 */
    lcd_show_string(10, 78, 200, 24, 24, "ADC:", WHITE);    /* 静态标签：摆杆ADC原始值 */


    HAL_ADC_Start_DMA(&hadc1, &ADC_Value, 1);
    dm_motor_init();
    pendulum_pid_init();        // 串级PID初始化

    motor[Motor1].ctrl.mode = spd_mode;
    motor[Motor1].ctrl.kp_set = 5.0f;
    motor[Motor1].ctrl.kd_set = 0.1f;


    HAL_Delay(1000);
     dm_motor_enable(&hcan1, &motor[Motor1]);


     HAL_TIM_Base_Start_IT(&htim3);      // 启动 1ms 控制中断

//    pendulum_pid_enable(1);            // 摆杆竖直扶正后再启动平衡（或由按键触发）

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		Key_Progress();

		/* 按键3：每次触发翻转一次运行状态（启动/停止平衡） */
		if (KEY3_Instance.key_flag)
		{
			KEY3_Instance.key_flag = 0;    // 读后清零
			pendulum_pid_toggle();
      	motor_pos_zero();
		}

		/* 按键0：将当前电机编码器位置映射为0 */
		if (KEY0_Instance.key_flag)
		{
			KEY0_Instance.key_flag = 0;    // 读后清零
			motor_pos_zero();
		}


        /* 红色LED指示运行状态：运行亮、停止灭 */
        HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, pendulum_pid_get_state() ? LED_ON : LED_OFF);

        /* 显示刷新：非阻塞 0.1s，仅更新数值（标签已在初始化时静态打印） */
        if (HAL_GetTick() - lcd_tick >= 100)
        {
            lcd_tick = HAL_GetTick();

            char buf[24];

            /* 摆杆角度偏差（counts，有符号） */
            sprintf(buf, "%+.1f", angle);
            lcd_fill(76, 10, 76 + 120, 10 + 24, BLACK);   // 先清旧值，避免残留
            lcd_show_string(76, 10, 120, 24, 24, buf, WHITE);

            /* 电机位置（rad，两位小数，有符号） */
            sprintf(buf, "%+.2f", motor_pos);
            lcd_fill(76, 44, 76 + 120, 44 + 24, BLACK);
            lcd_show_string(76, 44, 120, 24, 24, buf, WHITE);

            /* 摆杆 ADC 原始值（0~4095）：标定 PENDULUM_CENTER_ANGLE 时看这一行 */
            sprintf(buf, "%4lu", (unsigned long)(uint16_t)ADC_Value);
            lcd_fill(76, 78, 76 + 120, 78 + 24, BLACK);
            lcd_show_string(76, 78, 120, 24, 24, buf, WHITE);

          
        }
        HAL_Delay(10);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
