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
#include "math.h"
#include "string.h"
#include <stdbool.h>
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
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */


#define MPU6050_ADDR 0XD0

#define SMPLRT_DIV_REG 0X19
#define	GYRO_CONFIG_REG 0X1B
#define ACCEL_CONFIG_REG 0X1C
#define	ACCEL_XOUT_H_REG 0X3B
#define	TEMP_OUT_H_REG 0X41
#define	GYRO_XOUT_H_REG 0X43
#define PWR_MGMT_1_REG 0X6B
#define WHO_AM_I_REG 0X75

#define BMP180_ADDRESS 0xEE
#define atmPress 101325 //Pa
#define BMP180_I2C &hi2c1


// Low-pass filter variables
float alpha = 0.1;
float prev_x = 0, prev_y = 0, prev_z = 0;
float filt_x = 0, filt_y = 0, filt_z = 0;
const float PI=3.14159;
// High-pass filter variables
const float cutoff_freq = 5;  // Cutoff frequency in Hz
float RC = 1.0/ (2.0 * PI * cutoff_freq);
float dt = 0.01;
float prev_xh = 0, prev_yh = 0, prev_zh = 0;
float filt_xh = 0, filt_yh = 0, filt_zh = 0;

// Motion detection variables
float threshold = 0.5;  // Motion detection threshold
float prev_magn = 0;
float curr_magn = 0;
bool motion_detected = false;


uint16_t Accel_X_RAW, Accel_Y_RAW, Accel_Z_RAW,Gyro_X_RAW,Gyro_Y_RAW,Gyro_Z_RAW;
float Ax=0,Ay=0,Az=0,Gx,Gy,Gz;
short AC1=0,AC2=0,AC3=0,B1=0,B2=0,MB=0,MC=0,MD=0;
unsigned short AC4=0,AC5=0,AC6=0;

//................../
long UT=0;
short oss=0;
long UP=0;
long X1=0,X2=0,X3=0,B3=0,B5=0;
unsigned long B4=0,B7=0;
long B6=0;

float temperature, pressure, altitude;


void MPU6050_Init(void){
	uint8_t check, data;
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, 1000);
	if (check==104){
		data=0;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &data, 1, 1000);
		data = 0x07;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &data, 1, 1000);
		data=0;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &data, 1, 1000);
		data=0;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &data, 1, 1000);

	}
}



void MPU6050_READ_ACC(void){
	uint8_t Rec_Data[6];

	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 1000);

	Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);


	Ax = Accel_X_RAW/16384.0;
	Ay = Accel_Y_RAW/16384.0;
	Az = Accel_Z_RAW/16384.0;
}

void MPU6050_READ_GR(void){
	uint8_t Rec_Data[6];

	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR,GYRO_XOUT_H_REG, 1, Rec_Data, 6, 1000);

	Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

	Gx = Gyro_X_RAW/131.0;
	Gy = Gyro_Y_RAW/131.0;
	Gz = Gyro_Z_RAW/131.0;

}





void read_calliberation_data (void)
{
	uint8_t Callib_Data[22] = {0};
	uint16_t Callib_Start = 0xAA;
	HAL_I2C_Mem_Read(BMP180_I2C, BMP180_ADDRESS, Callib_Start, 1, Callib_Data,22, HAL_MAX_DELAY);

	AC1 = ((Callib_Data[0] << 8) | Callib_Data[1]);
	AC2 = ((Callib_Data[2] << 8) | Callib_Data[3]);
	AC3 = ((Callib_Data[4] << 8) | Callib_Data[5]);
	AC4 = ((Callib_Data[6] << 8) | Callib_Data[7]);
	AC5 = ((Callib_Data[8] << 8) | Callib_Data[9]);
	AC6 = ((Callib_Data[10] << 8) | Callib_Data[11]);
	B1 = ((Callib_Data[12] << 8) | Callib_Data[13]);
	B2 = ((Callib_Data[14] << 8) | Callib_Data[15]);
	MB = ((Callib_Data[16] << 8) | Callib_Data[17]);
	MC = ((Callib_Data[18] << 8) | Callib_Data[19]);
	MD = ((Callib_Data[20] << 8) | Callib_Data[21]);
}



// Get uncompensated Temp
uint16_t Get_UTemp (void)
{
	uint8_t datatowrite = 0x2E;
	uint8_t Temp_RAW[2] = {0};
	HAL_I2C_Mem_Write(BMP180_I2C, BMP180_ADDRESS, 0xF4, 1, &datatowrite, 1, 1000);
	HAL_Delay (5);  // wait 4.5 ms
	HAL_I2C_Mem_Read(BMP180_I2C, BMP180_ADDRESS, 0xF6, 1, Temp_RAW, 2, 1000);
	return ((Temp_RAW[0]<<8) + Temp_RAW[1]);
}

float BMP180_GetTemp (void)
{
	long Temp=0;
	UT = Get_UTemp();
	X1 = ((UT-AC6) * (AC5/(pow(2,15))));
	X2 = ((MC*(pow(2,11))) / (X1+MD));
	B5 = X1+X2;
	Temp = (B5+8)/(pow(2,4));
	return Temp/10.0;
}

// Get uncompensated Pressure
uint32_t Get_UPress (int oss)   // oversampling setting 0,1,2,3
{
	uint8_t datatowrite = 0x34+(oss<<6);
	uint8_t Press_RAW[3] = {0};
	HAL_I2C_Mem_Write(BMP180_I2C, BMP180_ADDRESS, 0xF4, 1, &datatowrite, 1, 1000);
	switch (oss)
	{
		case (0):
			HAL_Delay (5);
			break;
		case (1):
			HAL_Delay (8);
			break;
		case (2):
			HAL_Delay (14);
			break;
		case (3):
			HAL_Delay (26);
			break;
	}
	HAL_I2C_Mem_Read(BMP180_I2C, BMP180_ADDRESS, 0xF6, 1, Press_RAW, 3, 1000);
	return (((Press_RAW[0]<<16)+(Press_RAW[1]<<8)+Press_RAW[2]) >> (8-oss));
}


float BMP180_GetPress (int oss)
{
	long Press = 0;
	UP = Get_UPress(oss);
	X1 = ((UT-AC6) * (AC5/(pow(2,15))));
	X2 = ((MC*(pow(2,11))) / (X1+MD));
	B5 = X1+X2;
	B6 = B5-4000;
	X1 = (B2 * (B6*B6/(pow(2,12))))/(pow(2,11));
	X2 = AC2*B6/(pow(2,11));
	X3 = X1+X2;
	B3 = (((AC1*4+X3)<<oss)+2)/4;
	X1 = AC3*B6/pow(2,13);
	X2 = (B1 * (B6*B6/(pow(2,12))))/(pow(2,16));
	X3 = ((X1+X2)+2)/pow(2,2);
	B4 = AC4*(unsigned long)(X3+32768)/(pow(2,15));
	B7 = ((unsigned long)UP-B3)*(50000>>oss);
	if (B7<0x80000000) Press = (B7*2)/B4;
	else Press = (B7/B4)*2;
	X1 = (Press/(pow(2,8)))*(Press/(pow(2,8)));
	X1 = (X1*3038)/(pow(2,16));
	X2 = (-7357*Press)/(pow(2,16));
	Press = Press + (X1+X2+3791)/(pow(2,4));

	return Press;
}


float BMP180_GetAlt (int oss)
{
	float Press;
	Press = BMP180_GetPress (oss);
	return 44330*(1-(pow(((float)Press/(float)atmPress), 0.19029495718)));
}


void bolocmpu(void){

	  // Apply low-pass filter
	  filt_x = alpha * Ax + (1 - alpha) * prev_x;
	  filt_y = alpha * Ay + (1 - alpha) * prev_y;
	  filt_z = alpha * Az + (1 - alpha) * prev_z;
	  prev_x = filt_x;
	  prev_y = filt_y;
	  prev_z = filt_z;

	  // Apply high-pass filter
	  float alpha_h = RC / (RC + dt);
	  filt_xh = alpha_h * (filt_xh + Ax - prev_xh);
	  filt_yh = alpha_h * (filt_yh + Ay - prev_yh);
	  filt_zh = alpha_h * (filt_zh + Az - prev_zh);
	  prev_xh = Ax;
	  prev_yh = Ay;
	  prev_zh = Az;

	  // Calculate magnitude
	  curr_magn = sqrt(filt_x * filt_x + filt_y * filt_y + filt_z * filt_z);

	  // Check for motion
	  if (abs(curr_magn - prev_magn) > threshold) {
	    motion_detected = true;
	  } else {
	    motion_detected = false;
	  }
	  prev_magn = curr_magn;

}




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
	char chuoi[80];
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
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MPU6050_Init();
  /* USER CODE BEGIN 2 */
// uint8_t data[] = "AAA\n";
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  	MPU6050_READ_ACC();
	  	MPU6050_READ_GR();

	//  	  read_calliberation_data();
	// 	  HAL_Delay(1000);
	//  	temperature = BMP180_GetTemp();
	//  	HAL_Delay(1000);
	 // 	pressure = BMP180_GetPress(0);
	// 	HAL_Delay(1000);
	//  	altitude =BMP180_GetAlt(0);
	// 	HAL_Delay(1000);
	 	HAL_Delay(100);
	 	sprintf(chuoi,"X = %.2f  ",Ax);
	 	HAL_Delay(100);
		HAL_UART_Transmit(&huart2,(uint8_t*)chuoi,strlen(chuoi),HAL_MAX_DELAY);
		HAL_Delay(100);
		sprintf(chuoi,"Y = %.2f  ",Ay);
		HAL_Delay(100);
		HAL_UART_Transmit(&huart2,(uint8_t*)chuoi,strlen(chuoi), HAL_MAX_DELAY);
		HAL_Delay(100);
		sprintf(chuoi,"Z = %.2f   ..",Az);
		HAL_Delay(100);
		HAL_UART_Transmit(&huart2,(uint8_t*)chuoi,strlen(chuoi), HAL_MAX_DELAY);
		HAL_Delay(100);
		sprintf(chuoi," %f",PI);
		HAL_Delay(100);
				HAL_UART_Transmit(&huart2,(uint8_t*)chuoi,strlen(chuoi), HAL_MAX_DELAY);
				HAL_Delay(100);
		bolocmpu();

		char chuoiqb[]="Filtered:\n";
		HAL_Delay(100);
		HAL_UART_Transmit(&huart2,(uint8_t*)chuoiqb,strlen(chuoiqb), HAL_MAX_DELAY);
	 	sprintf(chuoi,"%.2f   ",filt_x);
	 	HAL_Delay(100);
		HAL_UART_Transmit(&huart2,(uint8_t*)chuoi,strlen(chuoi), HAL_MAX_DELAY);
		HAL_Delay(100);
		sprintf(chuoi," %.2f   ",filt_y);
		HAL_Delay(100);
		HAL_UART_Transmit(&huart2,(uint8_t*)chuoi,strlen(chuoi), HAL_MAX_DELAY);
		HAL_Delay(100);
		sprintf(chuoi," %.2f   ",filt_z);
		HAL_Delay(100);
		HAL_UART_Transmit(&huart2,(uint8_t*)chuoi,strlen(chuoi),HAL_MAX_DELAY);
		HAL_Delay(100);

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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 170;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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
