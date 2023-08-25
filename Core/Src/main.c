/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "lwip.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include <math.h>
#include "time.h"
#include "lwip/api.h"
#include "lwip/netif.h"
#include "lwip/apps/httpd.h"
#include "myapi.h"
#include "local_files.h"
#include "MyFlash.h"
#include <sys/socket.h>
#include <arpa/inet.h>
//#include "localtypes.h"
//#include "util.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define NTP_MS_TO_FS_U32  (4294967296.0 / 1000.0)
#define NTP_MS_TO_FS_U16  (65536.0 / 1000.0)
/* Number of seconds between 1970 and Feb 7, 2036 06:28:16 UTC (epoch 1) */
#define DIFF_SEC_1970_2036          ((uint32_t)2085978496L)
#define NTP_TIMESTAMP_DELTA 2208988800ull

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart7;

osThreadId defaultTaskHandle;
osThreadId myTask02Handle;
/* USER CODE BEGIN PV */
time_t rtc_read(void);
char calc_crc(char c,int cnt);
void tcpecho_init(void);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_UART7_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM1_Init(void);
void StartDefaultTask(void const * argument);
void tcpecho_thread(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// это же дебаг
int _write(int file, char *ptr,int len){
	int i=0;
	for(i=0;i<len;i++)
		ITM_SendChar((*ptr++));
	return len;
}

	 //string, counts and two smoking errors
int ERRORS=0;
int Tipe_Mes=0;// 1-ZDA  2-RMC
int Test=0;
long long int gps_unix=0;
int zpt=0;
int start_crc=0;
int z=0;
int ind=0;
int crc=0;
int count=0;
int dec;
int PPS_count=0;//вынес для проверки
int PPS_mass[10]={0};//вынес для проверки
int crc_pars=0;
char time_buff[9]={0};
char crc_buff[3]={0};
char buff[1]={0};
int dataReceived=1;
int dataTransmitted=1;
char year_str[2]={0};
int century=100;
//_______________________________0_______1_______2________3_______4______5_______6_______7_______8_______9______10______11_____12_____13___14____15____16_____17______18_____19______20_____21______22_____23_____24_____25_____26_____27______28_____29______30
//int offset_minutes[]={         0,      0,      0,       0,      0,     0,      0,      0,    -30,      0,      0,      0,     0,     0,   0,    0,   30,     0,      0,    30,      0,    30,      0,     0,     0,    30,     0,     0,      0,     0,      0 };
//int offset_hours[]={         -11,    -10,     -9,      -8,     -7,    -6,     -5,     -4,     -3,      3,     -2,     -1,     0,     1,   2,    3,    3,     4,      5,     5,      6,     6,      7,     8,     9,     9,    10,    11,     12,    13,     14 };
const int offset_unix[]={   -39600, -36000, -32400,  -28800, -25200,-21600, -18000, -14400, -12600, -10800,  -7200,  -3600,     0, -3600,7200,10800,12600, 14400,  18000, 19800,  21600, 23400,  25200, 28800, 32400, 34200, 36000, 39600,  43200, 46800,  50400};
struct tm Time_calc;

typedef struct
{
	uint8_t li_vn_mode;

	uint8_t stratum;
	uint8_t poll;
	uint8_t precision;

	uint32_t rootDelay;

	uint16_t rootDispersion_s;
	uint16_t rootDispersion_f;

	uint32_t refId;

	uint32_t refTm_s;
	uint32_t refTm_f;

	uint32_t origTm_s;
	uint32_t origTm_f;

	uint32_t rxTm_s;
	uint32_t rxTm_f;

	uint32_t txTm_s;
	uint32_t txTm_f;

} ntp_packet_t;

/* From GNSS PPS */
 uint32_t time_ref_s;
 uint32_t time_ref_f;

typedef enum {
	NTPD_UNSYNC = 0,
	NTPD_IN_LOCK = 1,
	NTPD_IN_HOLDOVER = 2,
	NTPD_DEGRADED = 3
} ntpd_status_status_t;
typedef struct {
	ntpd_status_status_t status;
	uint32_t requests_count;
	uint8_t stratum;
} ntpd_status_t;

ntpd_status_t ntpd_status = {
  .status = NTPD_IN_LOCK,
  .requests_count = 0,
  .stratum = 1
};
//RTCDateTime ntpd_datetime;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	memset(&gps,0,sizeof(gps));
	// ZDA-38;RMC-68



	 //включение ZDA
	 char MESZDA[]={0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x08, 0x01, 0x01, 0x00, 0x01, 0x01, 0x00, 0x0B, 0x6B};
	 char CONZDA[]={0xB5, 0x62, 0x06, 0x01, 0x02, 0x00, 0xF0, 0x08, 0x01, 0x19};

	 //отключение ZDA
	 //char MESZDA[]={0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x5B};
	 //char CONZDA[]={0xB5, 0x62, 0x06, 0x01, 0x02, 0x00, 0xF0, 0x08, 0x01, 0x19};

	 //отключение остального
	 char MESGGA[]={0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x23};
	 char CONGGA[]={0xB5, 0x62, 0x06, 0x01, 0x02, 0x00, 0xF0, 0x00, 0xF9, 0x11};

	 char MESGLL[]={0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2A};
	 char CONGLL[]={0xB5, 0x62, 0x06, 0x01, 0x02, 0x00, 0xF0, 0x01, 0xFA, 0x12};

	 char MESGSA[]={0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x31};
	 char CONGSA[]={0xB5, 0x62, 0x06, 0x01, 0x02, 0x00, 0xF0, 0x02, 0xFB, 0x13};

	 char MESGSV[]={0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x38};
	 char CONGSV[]={0xB5, 0x62, 0x06, 0x01, 0x02, 0x00, 0xF0, 0x03, 0xFC, 0x14};

	 char MESVTG[]={0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x46};
	 char CONVTG[]={0xB5, 0x62, 0x06, 0x01, 0x02, 0x00, 0xF0, 0x05, 0xFE, 0x16};

	 //отключение RMC на всякий
	 //char MESRMC[]={0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x3F};
	 //char CONRMC[]={0xB5, 0x62, 0x06, 0x01, 0x02, 0x00, 0xF0, 0x04, 0xFD, 0x15};

	 //включение RMC
	 char MESRMC[]={0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x01, 0x01, 0x00, 0x01, 0x01, 0x00, 0x07, 0x4F};
	 char CONRMC[]={0xB5, 0x62, 0x06, 0x01, 0x02, 0x00, 0xF0, 0x04, 0xFD, 0x15};

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  //MX_LWIP_Init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_UART7_Init();
  MX_RTC_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  //TIM1
  HAL_TIM_Base_Start_IT(&htim1);

  HAL_Delay(5000);

 //ON ZDA
  HAL_UART_Transmit(&huart7,(uint8_t*) MESZDA, 16, 1000);
  HAL_Delay(100);
  HAL_UART_Transmit(&huart7,(uint8_t*) CONZDA, 10, 1000);
  HAL_Delay(100);

  // OFF protokol
  HAL_UART_Transmit(&huart7,(uint8_t*) MESGGA, 16, 1000);
  HAL_Delay(100);
  HAL_UART_Transmit(&huart7,(uint8_t*) CONGGA, 10, 1000);
  HAL_Delay(100);

  HAL_UART_Transmit(&huart7,(uint8_t*) MESGLL, 16, 1000);
  HAL_Delay(100);
  HAL_UART_Transmit(&huart7,(uint8_t*) CONGLL, 10, 1000);
  HAL_Delay(100);

  HAL_UART_Transmit(&huart7,(uint8_t*) MESGSA, 16, 1000);
  HAL_Delay(100);
  HAL_UART_Transmit(&huart7,(uint8_t*) CONGSA, 10, 1000);
  HAL_Delay(100);

  HAL_UART_Transmit(&huart7,(uint8_t*) MESGSV, 16, 1000);
  HAL_Delay(100);
  HAL_UART_Transmit(&huart7,(uint8_t*) CONGSV, 10, 1000);
  HAL_Delay(100);

  HAL_UART_Transmit(&huart7,(uint8_t*) MESVTG, 16, 1000);
  HAL_Delay(100);
  HAL_UART_Transmit(&huart7,(uint8_t*) CONVTG, 10, 1000);
  HAL_Delay(100);

  //отключение и включение RMC на всякий
  HAL_UART_Transmit(&huart7,(uint8_t*) MESRMC, 16, 1000);
  HAL_Delay(100);
  HAL_UART_Transmit(&huart7,(uint8_t*) CONRMC, 10, 1000);
  HAL_Delay(100);

  //start the web server
  int offset =0;
 ReadDeviceAddressOffset((char*) &user_info, sizeof(user_info), offset);
 offset+=sizeof(user_info);
 //Обнуление PPS
 PPS_count=0;
 gps.year[0]='V';
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of myTask02 */
  osThreadDef(myTask02, tcpecho_thread, osPriorityIdle, 0, 128);
  myTask02Handle = osThreadCreate(osThread(myTask02), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_UART7;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInitStruct.Uart7ClockSelection = RCC_UART7CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
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

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 23;
  sTime.Minutes = 59;
  sTime.Seconds = 0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_SUNDAY;
  sDate.Month = RTC_MONTH_DECEMBER;
  sDate.Date = 31;
  sDate.Year = 0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 7999;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 49999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief UART7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART7_Init(void)
{

  /* USER CODE BEGIN UART7_Init 0 */

  /* USER CODE END UART7_Init 0 */

  /* USER CODE BEGIN UART7_Init 1 */

  /* USER CODE END UART7_Init 1 */
  huart7.Instance = UART7;
  huart7.Init.BaudRate = 9600;
  huart7.Init.WordLength = UART_WORDLENGTH_8B;
  huart7.Init.StopBits = UART_STOPBITS_1;
  huart7.Init.Parity = UART_PARITY_NONE;
  huart7.Init.Mode = UART_MODE_TX_RX;
  huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart7.Init.OverSampling = UART_OVERSAMPLING_16;
  huart7.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart7.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART7_Init 2 */

  /* USER CODE END UART7_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOJ_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Led_GPIO_Port, Led_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(EXT_RST_GPIO_Port, EXT_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Timled_GPIO_Port, Timled_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : OTG_HS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_HS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_HS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ULPI_D7_Pin ULPI_D6_Pin ULPI_D5_Pin ULPI_D2_Pin
                           ULPI_D1_Pin ULPI_D4_Pin */
  GPIO_InitStruct.Pin = ULPI_D7_Pin|ULPI_D6_Pin|ULPI_D5_Pin|ULPI_D2_Pin
                          |ULPI_D1_Pin|ULPI_D4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_VBUS_Pin */
  GPIO_InitStruct.Pin = OTG_FS_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_VBUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Led_Pin */
  GPIO_InitStruct.Pin = Led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Led_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PPS_Pin */
  GPIO_InitStruct.Pin = PPS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(PPS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Button_Pin */
  GPIO_InitStruct.Pin = Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ULPI_NXT_Pin */
  GPIO_InitStruct.Pin = ULPI_NXT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
  HAL_GPIO_Init(ULPI_NXT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ULPI_STP_Pin ULPI_DIR_Pin */
  GPIO_InitStruct.Pin = ULPI_STP_Pin|ULPI_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : EXT_RST_Pin */
  GPIO_InitStruct.Pin = EXT_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(EXT_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ULPI_CLK_Pin ULPI_D0_Pin */
  GPIO_InitStruct.Pin = ULPI_CLK_Pin|ULPI_D0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Timled_Pin */
  GPIO_InitStruct.Pin = Timled_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Timled_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	int PPS_Counter_period=0;
	//int PPS_count=0;//вынес для проверки
	//int PPS_mass[10]={0};//вынес для проверки
	if(GPIO_Pin == PPS_Pin) {
		if(PPS_count>2&&PPS_count<12){
		PPS_mass[PPS_count-2] = TIM1->CNT;
		}
		if(PPS_count==12){
			HAL_GPIO_TogglePin(Timled_GPIO_Port, Timled_Pin);
			PPS_Counter_period=(PPS_mass[0]+PPS_mass[1]+PPS_mass[2]+PPS_mass[3]+PPS_mass[4]+PPS_mass[5]+PPS_mass[6]+PPS_mass[7]+PPS_mass[8]+PPS_mass[9])/10;
			TIM1->ARR=PPS_Counter_period;
		}
	}
	if(PPS_count<13){
		PPS_count=PPS_count+1;
		TIM1->CNT = 0;//обнуление счетчика
		}



		}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	RTC_TimeTypeDef sTime = {0};
	RTC_DateTypeDef sDate = {0};
	if(huart == &huart7) {
		//$ message start
		if(buff[0]=='$'){
			count=0;
			zpt=0;
		}
		//Message error
		else if (count==1&&buff[0]!='G'){
			count=0;
			ERRORS++;
		}


		//CRC calculation
		int res = calc_crc(buff[0],count);
		if(res==1){
			//printf("crc=%d\t crc_buff=%s\t dec=%d\n\r",crc,crc_buff,dec);
			//RTC READ
			rtc_read();
			//printf("rtc_read=%llu\t",rtc_read());
			//comparison RTC&CRC
			//Time_calc.tm_wday = 1;//atoi(gps.);
			Time_calc.tm_mon = atoi(gps.month)-1;//-1 do January==0 month
			Time_calc.tm_mday = atoi(gps.day);
			if(year_str[0]=='0'&&year_str[1]=='0'){
				century=century+100;//atoi(gps.year)
			}
			Time_calc.tm_year = atoi(year_str) + century;
			Time_calc.tm_hour = atoi(gps.hours);
			Time_calc.tm_min = atoi(gps.minuttes);
			Time_calc.tm_sec = atoi(gps.seconds);
			gps_unix = mktime(&Time_calc);
			//printf("tm_year=%d\t tm_mon=%d\t tm_mday=%d\t tm_hour=%d\t tm_min=%d\t tm_sec=%d\n",Time_calc.tm_year,Time_calc.tm_mon,Time_calc.tm_mday,Time_calc.tm_hour,Time_calc.tm_min,Time_calc.tm_sec);
			//printf("rtc_read=%llu\t Time_calc=%llu\n",rtc_read(),gps_unix);

		}
		if(res==1&&gps_unix!=rtc_read()){

			time_ref_s=htonl(gps_unix- DIFF_SEC_1970_2036);
			sTime.Hours = Time_calc.tm_hour;
			sTime.Minutes = Time_calc.tm_min;
			sTime.Seconds = Time_calc.tm_sec;
			sTime.StoreOperation = RTC_STOREOPERATION_RESET;




			if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
			{
				Error_Handler();
			}
			sDate.Month = Time_calc.tm_mon;
			sDate.Date = Time_calc.tm_mday;
			sDate.Year = Time_calc.tm_year-century;
			if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
			{
				Error_Handler();
			}


		}
		//printf("rtc_read=%llu\t Time_calc=%llu\n",rtc_read(),gps_unix);

		//ZDA OR RMC
		if (count==3&&buff[0]=='Z'){
			Tipe_Mes=1;
		}
		else if(count==3&&buff[0]=='R'){
			Tipe_Mes=2;
		}
		//If ZDA
		if(Tipe_Mes==1){

			if(count==6&&buff[0]!=','){
				count=0;
				ERRORS++;
			}
			if(buff[0]==','){
				zpt++;
				ind=0;
			}
			if(zpt==1&&buff[0]!=','){
				time_buff[ind]=buff[0];
				ind++;
			}
			if(zpt==2&&buff[0]==','){
				gps.hours[0]=time_buff[0];
				gps.hours[1]=time_buff[1];
				gps.minuttes[0]=time_buff[2];
				gps.minuttes[1]=time_buff[3];
				gps.seconds[0]=time_buff[4];
				gps.seconds[1]=time_buff[5];
				gps.seconds[2]=time_buff[6];
				gps.seconds[3]=time_buff[7];
				gps.seconds[4]=time_buff[8];
			}
			if(zpt==2&&buff[0]!=','){

				time_buff[ind]=buff[0];
				ind++;
			}
			if(zpt==3&&buff[0]==','){
				gps.day[0]=time_buff[0];
				gps.day[1]=time_buff[1];
			}
			if(zpt==3&&buff[0]!=','){

				time_buff[ind]=buff[0];
				ind++;
			}
			if(zpt==4&&buff[0]==','){
				gps.month[0]=time_buff[0];
				gps.month[1]=time_buff[1];
			}
			if(zpt==4&&buff[0]!=','){

				time_buff[ind]=buff[0];
				ind++;
			}
			if(zpt==5&&buff[0]==','){
				gps.year[0]=time_buff[0];
				gps.year[1]=time_buff[1];
				gps.year[2]=time_buff[2];
				gps.year[3]=time_buff[3];
				year_str[0]=time_buff[2];
				year_str[1]=time_buff[3];
			}
		}


		//IF RMC
		if(Tipe_Mes==2){

			if(count==6&&buff[0]!=','){
				count=0;
				ERRORS++;
			}
			if(buff[0]==','){
				zpt++;
				ind=0;
			}
			if(zpt==1&&buff[0]!=','){

				time_buff[ind]=buff[0];
				ind++;
			}
			if(zpt==2&&buff[0]==','){
				gps.hours[0]=time_buff[0];
				gps.hours[1]=time_buff[1];
				gps.minuttes[0]=time_buff[2];
				gps.minuttes[1]=time_buff[3];
				gps.seconds[0]=time_buff[4];
				gps.seconds[1]=time_buff[5];
				gps.seconds[2]=time_buff[6];
				gps.seconds[3]=time_buff[7];
				gps.seconds[4]=time_buff[8];
			}
			if(zpt==2&&buff[0]!=','){

				time_buff[ind]=buff[0];
				ind++;
			}
			if(zpt==3&&buff[0]==','){
				gps.sinc[1]=time_buff[0];
			}

			if(zpt==9&&buff[0]!=','){

				time_buff[ind]=buff[0];
				ind++;
			}
			if(zpt==10&&buff[0]==','){
				gps.day[0]=time_buff[0];
				gps.day[1]=time_buff[1];
				gps.month[0]=time_buff[2];
				gps.month[1]=time_buff[3];
				gps.year[0]=time_buff[4];
				gps.year[1]=time_buff[5];
				year_str[0]=time_buff[4];
				year_str[1]=time_buff[5];
			}
		}

		//printf("buff=%c\tcount=%d\tzpt=%d\tind=%d\tTipe_Mes=%d\n\r",buff[0],count,zpt,ind,Tipe_Mes);
		//printf("crc_hx=%s\t crc=%d\t crc_buff=%s\t dec=%d\n\r",crc_hx,crc,crc_buff,dec);
		dataReceived=1;

		if( dataTransmitted != 0 ) {

			//HAL_UART_Transmit_IT(&huart6, (uint8_t *)buff, 1);

			dataReceived=0;
			dataTransmitted=0;
		}

		HAL_UART_Receive_IT (&huart7, (uint8_t *)buff, 1);
		gps.errors[1]=ERRORS;
		count++;
	}
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {

//	//if(huart == &huart6) {

		dataTransmitted=1;

		if( dataReceived != 0 ) {
			//HAL_UART_Transmit_IT(&huart6, (uint8_t *)buff, 1);
			dataReceived=0;
			dataTransmitted=0;
		}
	//}
}

char Hex_to_dec(char hex[2]){
    int i;
    int dig; /*to store digit*/
    int cont = 0;
    dec = 0;
    for (i = (strlen(hex) - 1); i >= 0; i--) {
        switch (hex[i]) {
        case 'A':
            dig = 10;
            break;
        case 'B':
            dig = 11;
            break;
        case 'C':
            dig = 12;
            break;
        case 'D':
            dig = 13;
            break;
        case 'E':
            dig = 14;
            break;
        case 'F':
            dig = 15;
            break;
        default:
            dig = hex[i] - 0x30;
        }
        dec = dec + (dig)*pow((double)16, (double)cont);
        cont++;
    }
    return dec;
}


char calc_crc(char c,int cnt){
	if (c=='*'){
		start_crc=0;
		crc_pars=1;
		z=0;
	}
	if(start_crc==1){
		crc^=c;
	}
	if(crc_pars==1&&c!='*'&&z<=1){
		crc_buff[z]=c;
		z++;
	}
	if(c=='\n'){
		Hex_to_dec(crc_buff);
		if(crc==dec){
			//Test++;
			return 1;
		}
	}
	if(cnt==0){
		start_crc=1;
		crc_pars=0;
		crc=0;
	}
//	printf("crc=%d\t crc_buff=%s\t dec=%d\n\r",crc,crc_buff,dec);
	return 0;
}

time_t rtc_read(void) {
	RTC_DateTypeDef dateStruct;
	RTC_TimeTypeDef timeStruct;
	struct tm timeinfo;

	hrtc.Instance = RTC;

	// Read actual date and time
	HAL_RTC_GetTime(&hrtc, &timeStruct, FORMAT_BIN); // Read time first!
	HAL_RTC_GetDate(&hrtc, &dateStruct, FORMAT_BIN);

	// Setup a tm structure based on the RTC
	// monday==1 sunday==7
	timeinfo.tm_wday = dateStruct.WeekDay;
	timeinfo.tm_mon = dateStruct.Month;//-1 do January==0 month
	timeinfo.tm_mday = dateStruct.Date;
	timeinfo.tm_year = dateStruct.Year + 100;
	timeinfo.tm_hour = timeStruct.Hours;
	timeinfo.tm_min = timeStruct.Minutes;
	timeinfo.tm_sec = timeStruct.Seconds;
	//printf("tm_wday=%d\t\n",timeinfo.tm_wday);

	// Convert to timestamp
	time_t t = mktime(&timeinfo)+offset_unix[user_info.zone];


	return t;
}

void tcpecho_init(void)
{
	sys_thread_new("tcpecho_thread", tcpecho_thread, NULL,DEFAULT_THREAD_STACKSIZE, 1);
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for LWIP */
  MX_LWIP_Init();
  /* USER CODE BEGIN 5 */
  httpd_init();
	/* Initialize tcp echo server */
	tcpecho_init();


	  ip4_addr_t add;
	  inet_aton(user_info.ip, &add);
	  setIP(add.addr);

	ip4_addr_t mask;
	inet_aton(user_info.netmask, &mask);
	setNetmask(mask.addr);
	int IPres=0;
	/* Infinite loop */
	for(;;)
	{

		HAL_UART_Receive_IT (&huart7, (uint8_t*)&buff, 1);

		if(HAL_GPIO_ReadPin (GPIOI, Button_Pin)){

			IPres=IPres+1;
		}
		else
		{
			HAL_GPIO_WritePin(Led_GPIO_Port, Led_Pin,GPIO_PIN_RESET);
			if(IPres>5){
				memset(&user_info,0,sizeof(user_info));
				strncpy(user_info.ip,"192.168.0.68",13);
				strncpy(user_info.netmask,"255.255.255.0",14);
				user_info.zone=12;
				//setIPaddr
				ip4_addr_t add;
				inet_aton(user_info.ip, &add);
				setIP(add.addr);
				//setNetMask
				ip4_addr_t mask;
				inet_aton(user_info.netmask, &mask);
				setNetmask(mask.addr);
			}
			IPres=0;

		}
		//HAL_GPIO_TogglePin(Led_GPIO_Port, Led1_Pin);
		HAL_GPIO_TogglePin(Led_GPIO_Port, Led_Pin);
		//HAL_UART_Receive(&huart7, (uint8_t*)RXstr, MESsize, 1000);
		//HAL_UART_Transmit(&huart6, (uint8_t*)str, 8, 1000);
		HAL_Delay(1000);
		//HAL_Delay(1000);
	}


  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_tcpecho_thread */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_tcpecho_thread */
void tcpecho_thread(void const * argument)
{
  /* USER CODE BEGIN tcpecho_thread */
  /* Infinite loop */
		struct netconn *conn;
		err_t err,recv_err;
		struct netbuf *buf;
		uint16_t buf_data_len;

		ntp_packet_t *ntp_packet_ptr;
		//RTCDateTime ntpd_datetime;
		//RTCDateTime ntpd_datetime;
		//struct tm tm_;
		//uint32_t tm_ms_;


		/* Create a new connection identifier. */
		conn = netconn_new(NETCONN_UDP);
		if (conn!=NULL)
		{
			/* Bind connection to well known port number 7. */
			err = netconn_bind(conn, NULL, 123);
			if (err == ERR_OK)
			{
				while (1)
				{
					while (( recv_err = netconn_recv(conn, &buf)) == ERR_OK)
					{
						do
						{
							netbuf_data(buf, (void **)&ntp_packet_ptr, &buf_data_len);

							if(buf_data_len < 48 || buf_data_len > 2048)
							{
								netbuf_delete(buf);
								continue;
							}
							ntp_packet_ptr->li_vn_mode = (0 << 6) | (4 << 3) | (4); // Leap Warning: None, Version: NTPv4, Mode: 4 - Server
							ntp_packet_ptr->stratum = ntpd_status.stratum;
							ntp_packet_ptr->poll = 5; // 32s
							ntp_packet_ptr->precision = -10; // ~1ms

							ntp_packet_ptr->rootDelay = 0; // Delay from GPS clock is ~zero
							ntp_packet_ptr->rootDispersion_s = 0;
							ntp_packet_ptr->rootDispersion_f = htonl(NTP_MS_TO_FS_U16 * 1.0); // 1ms
							ntp_packet_ptr->refId = ('G') | ('P' << 8) | ('S' << 16) | ('\0' << 24);
							/* Move client's transmit timestamp into origin fields */
							ntp_packet_ptr->origTm_s = ntp_packet_ptr->txTm_s;
							ntp_packet_ptr->origTm_f = ntp_packet_ptr->txTm_f;

							ntp_packet_ptr->refTm_s = time_ref_s;
							ntp_packet_ptr->refTm_f = time_ref_f;

							//rtcGetTime(&RTCD1, &ntpd_datetime);
							//rtcConvertDateTimeToStructTm(&ntpd_datetime, &tm_, &tm_ms_);

							ntp_packet_ptr->rxTm_s = htonl(rtc_read()- DIFF_SEC_1970_2036);//htonl(mktime(&tm_) - DIFF_SEC_1970_2036);
							ntp_packet_ptr->rxTm_f = 0;//htonl((NTP_MS_TO_FS_U32 * tm_ms_));

							/* Copy into transmit timestamp fields */
							ntp_packet_ptr->txTm_s = ntp_packet_ptr->rxTm_s;
							ntp_packet_ptr->txTm_f = ntp_packet_ptr->rxTm_f;

							netconn_send(conn, buf);
						}
						while (netbuf_next(buf) >= 0);

						netbuf_delete(buf);
					}
					/* Close connection and discard connection identifier. */
					//netconn_close(newconn);
					//netconn_delete(newconn);
					ntpd_status.requests_count++;
				}
			}
			else
			{
				netconn_delete(conn);
			}
		}

  /* USER CODE END tcpecho_thread */
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
//    if(htim->Instance == TIM1) //check if the interrupt comes from TIM1
//    {
//            HAL_GPIO_TogglePin(Timled_GPIO_Port, Timled_Pin);
//            //200 МГц APB2
//   }
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
