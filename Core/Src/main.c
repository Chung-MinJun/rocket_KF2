/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f1xx.h"
#include "BMP180.h"
#include "stdio.h"
#include "MPU6050.h"
#include "math.h"
#include "stdint.h"
#include "fatfs_sd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RAD_TO_DEG 57.29578
#define DEG_TO_RAD 0.017453
#define ALPHA 0.96                          // Complementary Filter alpha value
#define dt 0.00069                       	// check while loop time
#define desiredAngle 30.0
#define beta 0.006981
#define gravityMagnitude 9.81
//#define beta 0.1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
float temp, press, altitude;
int second = 0;
float firstAltitude;
int Parachute = 0;
int start = 0;    				//Parachute states


float gyroAngleX = 0.0f, gyroAngleY = 0.0f; 	// pitch(X) and roll(Y) angle (Euler Angle) by gyroscope
float accelAngleX = 0.0f, accelAngleY = 0.0f; 	// pitch(X) and roll(Y) angle (Euler Angle) by AccelerateScope
float compAngleX = 0.0f, compAngleY = 0.0f; 	// Complementary Filter result angle        URL https://blog.naver.com/intheglass14/222777512235 https://yjhtpi.tistory.com/352
float rocketVector[3] = { 0.0f };              // rocket vector(attitude) init
float vectorG[3] = {0.0f, 0.0f, -1.0f};
float z_Unitvector[3] = { 0.0f, 0.0f, 1.0f };   // Z-axis vector
float rocketAngle = 0.0f;    					// result of dot product Rocket Vector with Z-axis

uint32_t startTick, endTick, elapsedTicks, costTime_us; //just check while loop time
int iter = 0;
float prev_state[3]; 					// result of dot product Rocket Vector with Z-axis
float a, b, c = 0.0f;                         	// just acos variables
float SEq_1 = 1.0f, SEq_2 = 0.0f, SEq_3 = 0.0f, SEq_4 = 0.0f; // estimated orientation quaternion elements with initial conditions
//float realAccelX, realAccelY, realAccelZ;
//float gravityComponentAlongRocket;
//float gravityComponentOnX, gravityComponentOnY, gravityComponentOnZ;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void SysTick_Init(void);               // systick  = like tic toc func in matlab
void SysTick_Delay_us(uint32_t us);
void updateQuaternion(float w_x,float w_y,float w_z,float a_x,float a_y,float a_z);
void setInitialQuaternion(float yaw, float pitch, float roll);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// to use printf
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
 set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

PUTCHAR_PROTOTYPE {
	/* Place your implementation of fputc here */
	/* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
	HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, 0xFFFF);
	return ch;
}
// use printf

// sdcard part
FATFS fs; // file system
FIL fil;  // File
FILINFO fno;
FRESULT fresult; // result
UINT br, bw;     // File read/write count

/**** capacity related *****/
FATFS *pfs;
DWORD fre_clust;
uint32_t total, free_space;
char str[16]; // Enough to hold all numbers up to 32-bit int
// sdcard part

// mpu6050
RawData_Def myAccelRaw, myGyroRaw;
ScaledData_Def myAccelScaled, myGyroScaled,accAverage;
typedef struct{
	float w, x, y ,z;
}Quaternion;
Quaternion q = {1,0,0,0};

int timeout_start = 0;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	MPU_ConfigTypeDef myMpuConfig;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  SysTick_Init();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_FATFS_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	initialBMP180();
	MPU6050_Init(&hi2c1);
	myMpuConfig.Accel_Full_Scale = AFS_SEL_16g; 		// full scale setting
	myMpuConfig.ClockSource = Internal_8MHz;
	myMpuConfig.CONFIG_DLPF = DLPF_184A_188G_Hz;
	myMpuConfig.Gyro_Full_Scale = FS_SEL_500;
	myMpuConfig.Sleep_Mode_Bit = 0;
	MPU6050_Config(&myMpuConfig);
	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_Base_Start_IT(&htim3);

	fresult = f_mount(&fs, "/", 1);
	/* Check free space */
	f_getfree("", &fre_clust, &pfs);
	total = (uint32_t) ((pfs->n_fatent - 2) * pfs->csize * 0.5);
	free_space = (uint32_t) (fre_clust * pfs->csize * 0.5);
	/* Open file to write/ create a file if it doesn't exist */
	fresult = f_open(&fil, "file1.txt", FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
	/* Writing text */





  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		temp = readTrueTemp();
		press = readTruePress(0);
		altitude = readTrueAltitude(0);
		MPU6050_Get_Accel_Scale(&myAccelScaled);
		MPU6050_Get_Gyro_Scale(&myGyroScaled);
<<<<<<< Updated upstream
=======
<<<<<<< HEAD
		if(start==0&&myAccelScaled.z<=24.0f){
=======
>>>>>>> Stashed changes
    // set init state.
		if(start==0&&myAccelScaled.z<=1.0f){
>>>>>>> 3c3693736f86d048fccb21fa0d526dcd94f1e559
			for(int count=0;count<3;count++){
				accAverage.x=0,accAverage.y=0,accAverage.z=0;
				accAverage.x=+myAccelScaled.x;
				accAverage.y=+myAccelScaled.y;
				accAverage.z=+myAccelScaled.z;
			}
			accAverage.x=accAverage.x/3.0f;
			accAverage.y=accAverage.y/3.0f;
			accAverage.z=accAverage.z/3.0f;		//moving mean filter
			gyroAngleX = atan2f(accAverage.y,		// RAD pitch and roll by acc scope
					sqrtf(accAverage.x * accAverage.x
							+ accAverage.z * accAverage.z));
			gyroAngleY = atan2f(-accAverage.x, 	//-myacc.x to myacc.x
					sqrtf(accAverage.y * accAverage.y
							+ accAverage.z * accAverage.z));
			prev_state[0] = gyroAngleX;
			prev_state[1] = gyroAngleY;
			prev_state[2]=0.0f;
			setInitialQuaternion(prev_state[2], prev_state[1],prev_state[0]);
			firstAltitude = altitude;			// update initial altitude

			printf("%.2f\r\n",myAccelScaled.z);
			HAL_Delay(10);
			continue;		//start trigger
		}
		else{
			timeout_start = 1;
			start=1;
		}
		//Rocket launched
//		startTick = SysTick->VAL;
		//kalman filter Part


		updateQuaternion(myGyroScaled.x,myGyroScaled.y,myGyroScaled.z,myAccelScaled.x,myAccelScaled.y,myAccelScaled.z);
//		float SEq_1 = 1.0f, SEq_2 = 0.0f, SEq_3 = 0.0f, SEq_4 = 0.0f;
		rocketVector[0] =  2.0*(SEq_2*SEq_4 - SEq_1*SEq_3); // x축
		rocketVector[1] = 2.0 * (SEq_1 * SEq_2 + SEq_3 * SEq_4); // y component
		rocketVector[2] = 1.0 - 2.0 * (SEq_2 * SEq_2 + SEq_3 * SEq_3); // z component

		a = rocketVector[0] * z_Unitvector[0]
											+ rocketVector[1] * z_Unitvector[1]
																			  + rocketVector[2] * z_Unitvector[2];
		b = sqrt(rocketVector[0] * rocketVector[0]
												  + rocketVector[1] * rocketVector[1]
																					 + rocketVector[2] * rocketVector[2]);
		c = sqrt(z_Unitvector[0] * z_Unitvector[0]
												+ z_Unitvector[1] * z_Unitvector[1]
																				 + z_Unitvector[2] * z_Unitvector[2]);

		rocketAngle = acos(a / (b * c)) * RAD_TO_DEG;                            // inner product(dot product) Rocket vector with Z unit vector
		 // Gravity Norm

		// inner product 로켓 방향과 중력 벡터의 내적 계산
//		gravityComponentAlongRocket = (rocketVector[0] * vectorG[0] + rocketVector[1] * vectorG[1] + rocketVector[2] * vectorG[2]) * gravityMagnitude;

		// comp Gravity to rocket axis 각 축에 대한 중력 성분 계산
//		gravityComponentOnX = rocketVector[0] * gravityComponentAlongRocket;
//		gravityComponentOnY = rocketVector[1] * gravityComponentAlongRocket;
//		gravityComponentOnZ = rocketVector[2] * gravityComponentAlongRocket;
//		realAccelX = myAccelScaled.x + gravityComponentOnX;
//		realAccelY = myAccelScaled.y + gravityComponentOnY;
//		realAccelZ = myAccelScaled.z + gravityComponentOnZ;
//		printf("%.2f %.2f %.2f\r\n",realAccelX,realAccelY,realAccelZ);

//		printf("Rocket Angle: %.2f\r\n", rocketAngle);
		printf(" is real \r\n");
		if (Parachute==0){
			if (altitude-firstAltitude>=390){
				Parachute = 1;
				printf("Altitude\r\n");
				__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 125);					// deploy parachute
				continue;
			}
			else if (rocketAngle>desiredAngle){
				Parachute = 1;
				printf("desiredAngle30\r\n");
				__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 125);					// deploy parachute
				continue;
			}
//			else if (second>10){
//				Parachute = 1;
//				printf("Timeout!\r\n");
//
//				continue;
//			}
		}



		// sdcard part
		char buffer[100]; // Buffer to hold string

		sprintf(buffer,
				"altitude: %.2f Gyro: X=%.2f Y=%.2f Z=%.2f Accel: X=%.2f Y=%.2f Z=%.2f\r\n",
				altitude, myGyroScaled.x, myGyroScaled.y, myGyroScaled.z,
				myAccelScaled.x, myAccelScaled.y, myAccelScaled.z);
		if (fresult == FR_OK) {
			fresult = f_write(&fil, buffer, strlen(buffer), &bw); // Write the string to file
			f_sync(&fil);                    // Ensure data is written and saved
		}

//		if (sqrt(myGyroScaled.x^2+myGyroScaled.y^2+myGyroScaled.z^2)){
//
//		}
//		endTick = SysTick->VAL;             	// tic (check loop time)
//		// SysTick은 다운 카운터이므로 startTick > endTick
//		elapsedTicks = startTick > endTick ? startTick - endTick : startTick + (0xFFFFFF - endTick);
//		costTime_us = (elapsedTicks * 8) / (64000000 / 1000000);

		//		printf("Time taken: %lu microseconds\n", costTime_us);
		// sdcard part
	}
	// sdcard part
	if (fresult == FR_OK) {
		fresult = f_close(&fil);
	}
	// sdcard part

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
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
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1280 - 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000 - 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 6400 - 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10000 - 1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) // time out private function
{
   if(timeout_start == 1)
   {
      if (htim->Instance == TIM3) {
         second++;
         if (second == 10) {
//            printf("timeout ok\r\n");
            __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 125);					// deploy parachute
            second = 0;
         }
      }
   }
}

void SysTick_Init(void)                      // tic toc function - once of while
{
	SysTick->LOAD = 0xFFFFFF;                   // set max value
	SysTick->VAL = 0;                           // init present count to 0
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
	SysTick_CTRL_ENABLE_Msk;    // timer start using CPU Clock
}

void SysTick_Delay_us(uint32_t us)            // tic toc function
{
	SysTick->LOAD = us * (64000000 / 8 / 1000000) - 1;
	SysTick->VAL = 0;
	while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk))
		;
}
void updateQuaternion(float w_x,float w_y,float w_z,float a_x,float a_y,float a_z){
	//Local Variables
	float norm;
	float SEqDot_omega_1, SEqDot_omega_2, SEqDot_omega_3, SEqDot_omega_4; // quaternion derrivative from gyroscopes elements
	float f_1, f_2, f_3; // objective function elements
	float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // objective function Jacobian elements
	float SEqHatDot_1, SEqHatDot_2, SEqHatDot_3, SEqHatDot_4; // estimated direction of the gyroscope error
	// Axulirary variables to avoid reapeated calcualtions
	float halfSEq_1 = 0.5f * SEq_1;
	float halfSEq_2 = 0.5f * SEq_2;
	float halfSEq_3 = 0.5f * SEq_3;
	float halfSEq_4 = 0.5f * SEq_4;
	float twoSEq_1 = 2.0f * SEq_1;
	float twoSEq_2 = 2.0f * SEq_2;
	float twoSEq_3 = 2.0f * SEq_3;
	// Normalise the accelerometer measurement
	norm = sqrt(a_x * a_x + a_y * a_y + a_z * a_z);
	a_x /= norm;
	a_y /= norm;
	a_z /= norm;
	// Compute the objective function and Jacobian
	f_1 = twoSEq_2 * SEq_4 - twoSEq_1 * SEq_3 - a_x;
	f_2 = twoSEq_1 * SEq_2 + twoSEq_3 * SEq_4 - a_y;
	f_3 = 1.0f - twoSEq_2 * SEq_2 - twoSEq_3 * SEq_3 - a_z;
	J_11or24 = twoSEq_3; // J_11 negated in matrix multiplication
	J_12or23 = 2.0f * SEq_4;
	J_13or22 = twoSEq_1; // J_12 negated in matrix multiplication
	J_14or21 = twoSEq_2;
	J_32 = 2.0f * J_14or21; // negated in matrix multiplication
	J_33 = 2.0f * J_11or24; // negated in matrix multiplication
	// Compute the gradient (matrix multiplication)
	SEqHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1;
	SEqHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3;
	SEqHatDot_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1;
	SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2;
	// Normalise the gradient
	norm = sqrt(SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 + SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4);
	SEqHatDot_1 /= norm;
	SEqHatDot_2 /= norm;
	SEqHatDot_3 /= norm;
	SEqHatDot_4 /= norm;
	// Compute the quaternion derrivative measured by gyroscopes
	SEqDot_omega_1 = -halfSEq_2 * w_x - halfSEq_3 * w_y - halfSEq_4 * w_z;
	SEqDot_omega_2 = halfSEq_1 * w_x + halfSEq_3 * w_z - halfSEq_4 * w_y;
	SEqDot_omega_3 = halfSEq_1 * w_y - halfSEq_2 * w_z + halfSEq_4 * w_x;
	SEqDot_omega_4 = halfSEq_1 * w_z + halfSEq_2 * w_y - halfSEq_3 * w_x;
	// Compute then integrate the estimated quaternion derrivative
	SEq_1 += (SEqDot_omega_1 - (beta * SEqHatDot_1)) * dt;
	SEq_2 += (SEqDot_omega_2 - (beta * SEqHatDot_2)) * dt;
	SEq_3 += (SEqDot_omega_3 - (beta * SEqHatDot_3)) * dt;
	SEq_4 += (SEqDot_omega_4 - (beta * SEqHatDot_4)) * dt;
	// Normalise quaternion
	norm = sqrt(SEq_1 * SEq_1 + SEq_2 * SEq_2 + SEq_3 * SEq_3 + SEq_4 * SEq_4);
	SEq_1 /= norm;
	SEq_2 /= norm;
	SEq_3 /= norm;
	SEq_4 /= norm;
}

// Yaw, Pitch, Roll 값을 기반으로 쿼터니온을 설정합니다.
void setInitialQuaternion(float yaw, float pitch, float roll) { //yaw_z, pitch_y, roll_x
    float cy = cos(yaw * 0.5);
    float sy = sin(yaw * 0.5);
    float cp = cos(pitch * 0.5);
    float sp = sin(pitch * 0.5);
    float cr = cos(roll * 0.5);
    float sr = sin(roll * 0.5);

    SEq_1 = cy * cp * cr + sy * sp * sr; // q0
    SEq_2 = cy * cp * sr - sy * sp * cr; // q1
    SEq_3 = sy * cp * sr + cy * sp * cr; // q2
    SEq_4 = sy * cp * cr - cy * sp * sr; // q3
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
	while (1) {
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
