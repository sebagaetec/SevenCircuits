/**
  ******************************************************************************
  * @file    STemWin/STemWin_HelloWorld/Src/main.c
  * @author  MCD Application Team
  * @brief   This file provides main program functions
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "WM.h"
#include "lsm6ds3.h"
#include "stm32f413h_discovery_lcd.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t GUI_Initialized = 0;
TIM_HandleTypeDef TimHandle;
uint32_t uwPrescalerValue = 0;
I2C_HandleTypeDef hi2c2;
/* Private function prototypes -----------------------------------------------*/
static void BSP_Config(void);
static void SystemClock_Config(void);
void BSP_Background(void);

extern void MainTask(void);

/* Private functions ---------------------------------------------------------*/

void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hi2c->Instance==I2C2)
  {


    __HAL_RCC_GPIOB_CLK_ENABLE();


    GPIO_InitStruct.Pin = ARD_D15_Pin|ARD_D14_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


    __HAL_RCC_I2C2_CLK_ENABLE();


  }

}
static void MX_I2C2_Init(void)
{


  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    for(;;);
  }


}

void display_acce_xyz(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz){
	char displayText[30];

	GUI_Clear();  // Limpia la pantalla
	    // Convertir y mostrar el valor de cada eje en la pantalla

	sprintf(displayText, "Ace X: %d", ax/1000);
	GUI_DispStringAt(displayText, 10, 20);

	sprintf(displayText, "Ace Y: %d", ay/1000);
	GUI_DispStringAt(displayText, 10, 40);

	    sprintf(displayText, "Ace Z: %d", az/1000);
	    GUI_DispStringAt(displayText, 10, 60);

	    // Mostrar los datos del giroscopio
	            sprintf(displayText, "Gyro X: %d", gx);
	            //GUI_DispStringAt(displayText, 10, 100);
	            sprintf(displayText, "Gyro Y: %d", gy);
	            //GUI_DispStringAt(displayText, 10, 120);
	            sprintf(displayText, "Gyro Z: %d", gz);
	            //GUI_DispStringAt(displayText, 10, 140);
}

void display_bar_xyz(int16_t ax, int16_t ay, int16_t az) {
    char displayText[30];

    int16_t xBarLength = (ax / 256) + 50;  // Escalamos y centramos (50 es punto medio)
    int16_t yBarLength = (ay / 256) + 50;
    int16_t zBarLength = (az / 256) + 50;

    GUI_Clear();  // Limpia la pantalla

    GUI_SetColor(GUI_RED);    // Rojo para el eje X
    GUI_FillRect(10, 20, 10 + xBarLength, 40);
     sprintf(displayText, "X: %d", ax/1000);
        GUI_DispStringAt(displayText, 120, 25);

        GUI_SetColor(GUI_GREEN);  // Verde para el eje Y
        GUI_FillRect(10, 50, 10 + yBarLength, 70);
        sprintf(displayText, "Y: %d", ay/1000);
        GUI_DispStringAt(displayText, 120, 55);

        GUI_SetColor(GUI_BLUE);   // Azul para el eje Z
        GUI_FillRect(10, 80, 10 + zBarLength, 100);
        sprintf(displayText, "Z: %d", az/1000);
        GUI_DispStringAt(displayText, 120, 85);

}

void display_arrow(int16_t x, int16_t z) {
    int16_t centerX = 120;  // Coordenada X del centro de la pantalla
    int16_t centerY = 80;   // Coordenada Y del centro de la pantalla
    int16_t arrowX, arrowY;

    // Mapea los valores de aceleración a desplazamientos en pantalla
    arrowX = centerX + (x / 512);  // Escala y centra el valor de X
    arrowY = centerY - (z / 512);  // Escala y centra el valor de Z (inversión para sentido Y)

    // Limita la posición de la flecha dentro de los bordes de la pantalla
    if (arrowX < 10) arrowX = 10;
    if (arrowX > 230) arrowX = 230;
    if (arrowY < 10) arrowY = 10;
    if (arrowY > 150) arrowY = 150;

    // Limpia la pantalla para actualizar la flecha
    GUI_Clear();

    // Dibuja el fondo de referencia (opcional)
    GUI_SetColor(GUI_LIGHTGRAY);
    GUI_DrawLine(centerX, 10, centerX, 150);  // Línea vertical
    GUI_DrawLine(10, centerY, 230, centerY);  // Línea horizontal

    // Dibuja la flecha
    GUI_SetColor(GUI_RED);  // Flecha de color rojo
    GUI_DrawLine(centerX, centerY, arrowX, arrowY);  // Línea del centro al punto final
    GUI_FillCircle(arrowX, arrowY, 3);  // Punta de la flecha

    // Opcional: Muestra las coordenadas actuales
    char displayText[30];
    sprintf(displayText, "X: %d, Z: %d", x, z);
    GUI_DispStringAt(displayText, 10, 160);
}

void display_debug_3d_plane(int16_t x, int16_t y) {
	GUI_Clear();
	GUI_SetColor(GUI_RED);
	GUI_DrawLine(10, 10, 100, 100); // Línea diagonal
	HAL_Delay(1000);
}




/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /* STM32F4xx HAL library initialization:
       - Configure the Flash prefetch
       - Systick timer is configured by default as source of time base, but user
         can eventually implement his proper time base source (a general purpose
         timer for example or other time source), keeping in mind that Time base
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and
         handled in milliseconds basis.
       - Set NVIC Group Priority to 4
       - Low Level Initialization
   */
  HAL_Init();

  /* Configure the system clock 100 MHz */
  SystemClock_Config();

  /***********************************************************/

  /* Compute the prescaler value to have TIM3 counter clock equal to 10 KHz */
  uwPrescalerValue = (uint32_t) ((SystemCoreClock /2) / 10000) - 1;

  /* Set TIMx instance */
  TimHandle.Instance = TIM3;

  /* Initialize TIM3 peripheral as follows:
       + Period = 1000 - 1
       + Prescaler = ((SystemCoreClock/2)/10000) - 1
       + ClockDivision = 0
       + Counter direction = Up
  */
  TimHandle.Init.Period = 1000 - 1;
  TimHandle.Init.Prescaler = uwPrescalerValue;
  TimHandle.Init.ClockDivision = 0;
  TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
  TimHandle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if(HAL_TIM_Base_Init(&TimHandle) != HAL_OK)
  {
    while(1)
    {
    }
  }

  /*##-2- Start the TIM Base generation in interrupt mode ####################*/
  /* Start Channel1 */
  if(HAL_TIM_Base_Start_IT(&TimHandle) != HAL_OK)
  {
    while(1)
    {
    }
  }

  /***********************************************************/


  /* Initialize LCD and LEDs */
  BSP_Config();
  MX_I2C2_Init();
  /* Init the STemWin GUI Library */
  GUI_Init();
  GUI_Initialized = 1;

  /*inicializar acelerometro*/
  lsm6ds3_init();

  /* Activate the use of memory device feature */
  WM_SetCreateFlags(WM_CF_MEMDEV);

  MainTask();

  int16_t gx, gy, gz, ax, ay,az;

   while (1) {
          // Leer los datos del acelerómetro
	   lsm6ds3_read_accel(&ax, &ay, &az);

	   lsm6ds3_read_gyro(&gx, &gy, &gz);
          // Mostrar los datos en la pantalla

   //display_acce_xyz(ax, ay, az, gx, gy, gz);

   display_bar_xyz(ax, ay, az);

   //display_arrow(ax, az);

	   //display_debug_3d_plane(ax,ay);

   HAL_Delay(500);  // Espera antes de actualizar los datos
   }

  /* Infinite loop */
  for(;;);
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @param  htim: TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  BSP_Background();
}

/**
  * @brief TIM MSP Initialization
  *        This function configures the hardware resources used in this application:
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration
  * @param htim: TIM handle pointer
  * @retval None
  */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* TIMx Peripheral clock enable */
  __HAL_RCC_TIM3_CLK_ENABLE();

  /*##-2- Configure the NVIC for TIMx ########################################*/
  /* Set the TIMx priority */
  HAL_NVIC_SetPriority(TIM3_IRQn, 0, 1);

  /* Enable the TIMx global Interrupt */
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

/**
  * @brief  Initializes the STM32F413H-Discovery's LCD and LEDs resources.
  * @param  None
  * @retval None
  */
static void BSP_Config(void)
{
  /* Configure LED3 and LED4 */
  BSP_LED_Init(LED3);
  BSP_LED_Init(LED4);

  /* Enable the CRC Module */
  __HAL_RCC_CRC_CLK_ENABLE();
}

/**
* @brief  BSP_Background.
* @param  None
* @retval None
*/
void BSP_Background(void)
{
  /* toggle LED3 and LED4 each 100ms */
  BSP_LED_Toggle(LED3);
  BSP_LED_Toggle(LED4);
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 100000000
  *            HCLK(Hz)                       = 100000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 8
  *            PLL_N                          = 200
  *            PLL_P                          = 2
  *            PLL_Q                          = 7
  *            PLL_R                          = 2
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 3
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  HAL_StatusTypeDef ret = HAL_OK;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* The voltage scaling allows optimizing the power consumption when the device is
     clocked below the maximum system frequency, to update the voltage scaling value
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 200;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 2;
  ret = HAL_RCC_OscConfig(&RCC_OscInitStruct);

  if(ret != HAL_OK)
  {
    while(1) { ; }
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  ret = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3);
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
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
