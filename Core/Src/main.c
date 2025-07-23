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
	#include "cmsis_os.h"

	/* Private includes ----------------------------------------------------------*/
	/* USER CODE BEGIN Includes */
	#include "ssd1306.h"
	#include "stdlib.h"
	#include <stdbool.h>
	#include <stdio.h>
	#include <string.h>
	#include "stm32f3xx_hal_uart.h"  // ✅ this includes the UART HAL functions
	extern UART_HandleTypeDef huart2; // ✅ lets you access the UART handle
	//gittry

	typedef struct {
		int x, y;
	} Point;

	#define GRID_SIZE 4
	#define WIDTH 128
	#define HEIGHT 64
	#define MAX_LENGTH 100


	Point snake[MAX_LENGTH];
	int snake_length = 3;
	int dx = 1, dy = 0;
	Point food = {10, 10};
	bool gameOver = false;
	uint8_t rx_data[1];

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
	I2C_HandleTypeDef hi2c1;

	UART_HandleTypeDef huart2;

	/* Definitions for defaultTask */
	osThreadId_t defaultTaskHandle;
	const osThreadAttr_t defaultTask_attributes = {
	  .name = "defaultTask",
	  .stack_size = 128 * 4,
	  .priority = (osPriority_t) osPriorityNormal,
	};
	/* Definitions for SnakeMove */
	osThreadId_t SnakeMoveHandle;
	const osThreadAttr_t SnakeMove_attributes = {
	  .name = "SnakeMove",
	  .stack_size = 128 * 4,
	  .priority = (osPriority_t) osPriorityHigh,
	};
	/* Definitions for scoreWrite */
	osThreadId_t scoreWriteHandle;
	const osThreadAttr_t scoreWrite_attributes = {
	  .name = "scoreWrite",
	  .stack_size = 128 * 4,
	  .priority = (osPriority_t) osPriorityRealtime,
	};
	/* Definitions for OLED */
	osMutexId_t OLEDHandle;
	const osMutexAttr_t OLED_attributes = {
	  .name = "OLED"
	};
	/* Definitions for myBinarySem01 */
	osSemaphoreId_t myBinarySem01Handle;
	const osSemaphoreAttr_t myBinarySem01_attributes = {
	  .name = "myBinarySem01"
	};
	/* USER CODE BEGIN PV */
	#define OLED_LOCK()    osMutexAcquire(OLEDHandle, osWaitForever)
	#define OLED_UNLOCK()  osMutexRelease(OLEDHandle)
	/* USER CODE END PV */

	/* Private function prototypes -----------------------------------------------*/
	void SystemClock_Config(void);
	static void MX_GPIO_Init(void);
	static void MX_I2C1_Init(void);
	static void MX_USART2_UART_Init(void);
	void StartDefaultTask(void *argument);
	void StartTaskSnakeMove(void *argument);
	void StartTaskscoreWrite(void *argument);

	/* USER CODE BEGIN PFP */

	/* USER CODE END PFP */

	/* Private user code ---------------------------------------------------------*/
	/* USER CODE BEGIN 0 */
	void spawnFood() {
		food.x = (HAL_GetTick() / 10) % (WIDTH / GRID_SIZE - 2) + 1;
		food.y = (HAL_GetTick() / 13) % (HEIGHT / GRID_SIZE - 2) + 1;
	}

	void moveSnake(void)
	{
		bool ate = false;

		// head move
		int newX = snake[0].x + dx;
		int newY = snake[0].y + dy;

		// duvar kontrolü
		if (newX <= 0 || newX >= WIDTH/GRID_SIZE ||
			newY <= 0 || newY >= HEIGHT/GRID_SIZE) {
			snake_length = 3;
			snake[0].x = 10;
			snake[0].y = 10;
			dx = 1; dy = 0;
			return;
		}

		// food?
		if (newX == food.x && newY == food.y) {
			ate = true;
		}

		/* gövdeyi kaydır (şu anki uzunluk kadar) */
		for (int i = snake_length - 1; i > 0; i--) {
			snake[i] = snake[i - 1];
		}

		/* yeni kafa */
		snake[0].x = newX;
		snake[0].y = newY;

		if (ate) {
			if (snake_length < MAX_LENGTH) {
				/* yeni eleman: eski son segmentin kopyası */
				snake[snake_length] = snake[snake_length - 1];
				snake_length++;
			}
			spawnFood();
			osSemaphoreRelease(myBinarySem01Handle);
		}
	}


	//void tickDraw(){
	//	if(snake[0].x == food.x && snake[0].y == food.y ){
	//		ssd1306_DrawString(0, 0, "✅", White);
	//
	//	}
	//}



	void drawSnake() {
		ssd1306_Fill(Black);
		for (int i = 0; i < snake_length; i++) {
			int px = snake[i].x * GRID_SIZE;
			int py = snake[i].y * GRID_SIZE;
			for (int dx = 0; dx < GRID_SIZE; dx++) {
				for (int dy = 0; dy < GRID_SIZE; dy++) {
					ssd1306_DrawPixel(px + dx, py + dy, White);
				}
			}
		}

		int fx = food.x * GRID_SIZE;
		int fy = food.y * GRID_SIZE;
		for (int dx = 0; dx < GRID_SIZE; dx++) {
			for (int dy = 0; dy < GRID_SIZE; dy++) {
				ssd1306_DrawPixel(fx + dx, fy + dy, White);
			}
		}

		ssd1306_UpdateScreen();
	}




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
	  MX_I2C1_Init();
	  /* USER CODE END SysInit */

	  /* Initialize all configured peripherals */
	  MX_GPIO_Init();
	  MX_I2C1_Init();
	  MX_USART2_UART_Init();
	  /* USER CODE BEGIN 2 */
	  ssd1306_Init();
	   spawnFood();

	   HAL_UART_Receive_IT(&huart2, rx_data, 1);

	  /* USER CODE END 2 */

	  /* Init scheduler */
	  osKernelInitialize();
	  /* Create the mutex(es) */
	  /* creation of OLED */
	  OLEDHandle = osMutexNew(&OLED_attributes);

	  /* USER CODE BEGIN RTOS_MUTEX */
	  /* add mutexes, ... */
	  /* USER CODE END RTOS_MUTEX */

	  /* Create the semaphores(s) */
	  /* creation of myBinarySem01 */
	  myBinarySem01Handle = osSemaphoreNew(1, 0, &myBinarySem01_attributes);

	  /* USER CODE BEGIN RTOS_SEMAPHORES */
	  /* add semaphores, ... */
	  /* USER CODE END RTOS_SEMAPHORES */

	  /* USER CODE BEGIN RTOS_TIMERS */
	  /* start timers, add new ones, ... */
	  /* USER CODE END RTOS_TIMERS */

	  /* USER CODE BEGIN RTOS_QUEUES */
	  /* add queues, ... */
	  /* USER CODE END RTOS_QUEUES */

	  /* Create the thread(s) */
	  /* creation of defaultTask */
	  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

	  /* creation of SnakeMove */
	  SnakeMoveHandle = osThreadNew(StartTaskSnakeMove, NULL, &SnakeMove_attributes);

	  /* creation of scoreWrite */
	  scoreWriteHandle = osThreadNew(StartTaskscoreWrite, NULL, &scoreWrite_attributes);

	  /* USER CODE BEGIN RTOS_THREADS */
	  /* add threads, ... */
	  /* USER CODE END RTOS_THREADS */

	  /* USER CODE BEGIN RTOS_EVENTS */
	  /* add events, ... */
	  /* USER CODE END RTOS_EVENTS */

	  /* Start scheduler */
	  osKernelStart();

	  /* We should never get here as control is now taken by the scheduler */

	  /* Infinite loop */
	  /* USER CODE BEGIN WHILE */
	  while (1)
	  {



	//	  moveSnake();
	//	  drawSnake();
	//	 	      HAL_Delay(200);


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
	  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	  {
		Error_Handler();
	  }

	  /** Initializes the CPU, AHB and APB buses clocks
	  */
	  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
								  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
	  {
		Error_Handler();
	  }
	  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
	  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
	  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	  {
		Error_Handler();
	  }
	}

	/**
	  * @brief I2C1 Initialization Function
	  * @param None
	  * @retval None
	  */
	static void MX_I2C1_Init(void)
	{

	  /* USER CODE BEGIN I2C1_Init 0 */

	  /* USER CODE END I2C1_Init 0 */

	  /* USER CODE BEGIN I2C1_Init 1 */

	  /* USER CODE END I2C1_Init 1 */
	  hi2c1.Instance = I2C1;
	  hi2c1.Init.Timing = 0x00201D2B;
	  hi2c1.Init.OwnAddress1 = 0;
	  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	  hi2c1.Init.OwnAddress2 = 0;
	  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	  {
		Error_Handler();
	  }

	  /** Configure Analogue filter
	  */
	  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
	  {
		Error_Handler();
	  }

	  /** Configure Digital filter
	  */
	  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
	  {
		Error_Handler();
	  }
	  /* USER CODE BEGIN I2C1_Init 2 */

	  /* USER CODE END I2C1_Init 2 */

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
	  huart2.Init.BaudRate = 38400;
	  huart2.Init.WordLength = UART_WORDLENGTH_8B;
	  huart2.Init.StopBits = UART_STOPBITS_1;
	  huart2.Init.Parity = UART_PARITY_NONE;
	  huart2.Init.Mode = UART_MODE_TX_RX;
	  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	  if (HAL_UART_Init(&huart2) != HAL_OK)
	  {
		Error_Handler();
	  }
	  /* USER CODE BEGIN USART2_Init 2 */

	  /* USER CODE END USART2_Init 2 */

	}

	/**
	  * @brief GPIO Initialization Function
	  * @param None
	  * @retval None
	  */
	static void MX_GPIO_Init(void)
	{
	  /* USER CODE BEGIN MX_GPIO_Init_1 */

	  /* USER CODE END MX_GPIO_Init_1 */

	  /* GPIO Ports Clock Enable */
	  __HAL_RCC_GPIOF_CLK_ENABLE();
	  __HAL_RCC_GPIOA_CLK_ENABLE();
	  __HAL_RCC_GPIOB_CLK_ENABLE();

	  /* USER CODE BEGIN MX_GPIO_Init_2 */

	  /* USER CODE END MX_GPIO_Init_2 */
	}

	/* USER CODE BEGIN 4 */
	void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

			// Restart receive
		 HAL_UART_Receive_IT(&huart2, rx_data, 1);
		   // char msg[32];
		   // sprintf(msg, "Got: %c\r\n", rx_data);
		   // HAL_UART_Receive_IT(&huart2, (uint8_t*)msg, strlen(msg), 100);

			switch (rx_data[0]) {
				case 'w': dy = -1; dx = 0; break;
				case 's': dy = 1; dx = 0;  break;
				case 'a': dx = -1; dy = 0; break;
				case 'd': dx = 1; dy = 0;  break;
			}

	}
	/* USER CODE END 4 */

	/* USER CODE BEGIN Header_StartDefaultTask */
	/**
	  * @brief  Function implementing the defaultTask thread.
	  * @param  argument: Not used
	  * @retval None
	  */
	/* USER CODE END Header_StartDefaultTask */
	void StartDefaultTask(void *argument)
	{
	  /* USER CODE BEGIN 5 */
	  /* Infinite loop */
	  for(;;)
	  {
		osDelay(1);
	  }
	  /* USER CODE END 5 */
	}

	/* USER CODE BEGIN Header_StartTaskSnakeMove */
	/**
	* @brief Function implementing the SnakeMove thread.
	* @param argument: Not used
	* @retval None
	*/
	/* USER CODE END Header_StartTaskSnakeMove */
	void StartTaskSnakeMove(void *argument)
	{
	  /* USER CODE BEGIN StartTaskSnakeMove */
	  /* Infinite loop */
		//uint32_t nextWake = osKernelGetTickCount();
	  for(;;)
	  {
		   moveSnake();
		  // OLED_LOCK();
			drawSnake();
			//OLED_UNLOCK();
			osDelay(150);
	  }
	  /* USER CODE END StartTaskSnakeMove */
	}

	/* USER CODE BEGIN Header_StartTaskscoreWrite */
	/**
	* @brief Function implementing the scoreWrite thread.
	* @param argument: Not used
	* @retval None
	*/
	/* USER CODE END Header_StartTaskscoreWrite */
	void StartTaskscoreWrite(void *argument)
	{
		for (;;)
		{
			if (osSemaphoreAcquire(myBinarySem01Handle, osWaitForever) == osOK)
			{
				/* Blink ON */
				//OLED_LOCK();
				ssd1306_DrawPixel(0,0,White);
				ssd1306_DrawPixel(1,0,White);
				ssd1306_DrawPixel(0,1,White);
				ssd1306_DrawPixel(1,1,White);
				ssd1306_UpdateScreen();
				//OLED_UNLOCK();

				osDelay(150);  // kısa; gözle görünür

				/* Blink OFF */
				//OLED_LOCK();
				ssd1306_DrawPixel(0,0,Black);
				ssd1306_DrawPixel(1,0,Black);
				ssd1306_DrawPixel(0,1,Black);
				ssd1306_DrawPixel(1,1,Black);
				ssd1306_UpdateScreen();
				//OLED_UNLOCK();
			}
		}
	}


	/**
	  * @brief  Period elapsed callback in non blocking mode
	  * @note   This function is called  when TIM6 interrupt took place, inside
	  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
	  * a global variable "uwTick" used as application time base.
	  * @param  htim : TIM handle
	  * @retval None
	  */
	void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
	{
	  /* USER CODE BEGIN Callback 0 */

	  /* USER CODE END Callback 0 */
	  if (htim->Instance == TIM6)
	  {
		HAL_IncTick();
	  }
	  /* USER CODE BEGIN Callback 1 */

	  /* USER CODE END Callback 1 */
	}

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
