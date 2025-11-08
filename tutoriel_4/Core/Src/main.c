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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32l475e_iot01.h"
#include "capteurs.h"
#include "traitement_donnees.h"


#include <wifi.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>
#include <time.h>


int32_t write_data_TS(int32_t* socket, uint16_t* datalen, int nb_parameters, ...);

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


#define SSID "Javakjian"
#define PASSWORD "abcdefgh"


uint8_t RemoteIP[] = {10,34,124,150};
uint16_t RemotePORT = 8002;




uint8_t RemoteIP2[4] ;
uint16_t RemotePORT2 = 80;
#define thingspeak_APIkey_write "RH1GKC7P5NRDX6AJ"

#define thingspeak_APIkey_read "O729N63F5UVJ6IOZ"


#define WIFI_WRITE_TIMEOUT 10000
#define WIFI_READ_TIMEOUT 10000
#define CONNECTION_TRIAL_MAX 10

// --- Timing Constants ---
#define SENSOR_READ_INTERVAL_MS 10   // Read sensors every 1000ms
#define THINGSPEAK_SEND_INTERVAL_MS 30000 // Send every 60 seconds
#define THINGSPEAK_MIN_SEND_INTERVAL_MS 20000



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DFSDM_Channel_HandleTypeDef hdfsdm1_channel1;

I2C_HandleTypeDef hi2c2;

QSPI_HandleTypeDef hqspi;

SPI_HandleTypeDef hspi3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
extern SPI_HandleTypeDef hspi;
int initial_accel_x = 0;
int initial_accel_y = 0;
int initial_accel_z = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DFSDM1_Init(void);
static void MX_I2C2_Init(void);
static void MX_QUADSPI_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_SPI3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*Initialize WIFI module */
int wifi_connect(){
	int wifi_ok=0 ;
	uint8_t MAC_Addr[6] = {0};
	uint8_t IP_Addr[4] = {0};
	if(WIFI_Init() == WIFI_STATUS_OK) {

		printf("> WIFI Module Initialized.\n\r");
		if(WIFI_GetMAC_Address(MAC_Addr, sizeof(MAC_Addr)) == WIFI_STATUS_OK) {
			printf("> es-wifi module MAC Address : %X:%X:%X:%X:%X:%X\n\r",MAC_Addr[0],MAC_Addr[1],MAC_Addr[2],MAC_Addr[3],MAC_Addr[4],MAC_Addr[5]);
		} else {
			printf("> ERROR : CANNOT get MAC address\n\r");
		}
		if( WIFI_Connect(SSID, PASSWORD, WIFI_ECN_WPA2_PSK) == WIFI_STATUS_OK) {
			printf("> es-wifi module connected \n\r");
			if(WIFI_GetIP_Address(IP_Addr, sizeof(IP_Addr)) == WIFI_STATUS_OK){
				printf("> es-wifi module got IP Address : %d.%d.%d.%d\n\r",IP_Addr[0],IP_Addr[1],IP_Addr[2],IP_Addr[3]);
				wifi_ok=-1 ;
			}
			else{
				printf("> ERROR : es-wifi module CANNOT get IP address\n\r");
			}
		}
		else{
			printf("> ERROR : es-wifi module NOT connected\n\r");
		}
	}
	else{
		printf("31 . . . \n") ;

		printf( "> ERROR : WIFI Module cannot be initialized.\n\r");
	}
	return(wifi_ok);
}

int Server_Connect(int Socket, uint8_t RemoteIP[], uint16_t RemotePORT){
	int16_t Trials = CONNECTION_TRIAL_MAX;
	printf( "> Trying to connect to Server: %d.%d.%d.%d:%d ...\n\r",
	RemoteIP[0], RemoteIP[1], RemoteIP[2], RemoteIP[3],
	RemotePORT);
	while (Trials--){
		if( WIFI_OpenClientConnection(Socket, WIFI_TCP_PROTOCOL, "TCP_CLIENT", RemoteIP, RemotePORT, 0) == WIFI_STATUS_OK){
			printf("> TCP Connection opened successfully.\n\r");
			break;
		}
	}
	if(Socket == -1){
		printf( "> ERROR : Cannot open Connection\n\r");
	}
	return(Socket);
}


bool resolve_hostname(char *hostname, uint8_t *RemoteIP ){
/* get the host address */
	printf("\nResolve hostname %s\r\n", hostname);
	WIFI_Status_t result = WIFI_GetHostAddress(hostname, RemoteIP, 4);
	if (result != 0) {
		printf("Error! GetHostAddress(%s) returned: %d\r\n", hostname,result);
		return false;
	}
	printf("%s address is %d.%d.%d.%d\r\n", hostname, RemoteIP[0],
	RemoteIP[1], RemoteIP[2], RemoteIP[3]);
	return true;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */



	int32_t Socket2 = -1;

	float vf ;
	int v1,v2,v3,v4 ;
	char sbuffer[256];
	char message[100];


	int32_t Socket = -1;
	uint16_t Datalen;
	int32_t ret;
	char TxData [64] ; // reserve la zone de dialogue
	char RxData [500];
	float sensor_value ;


	// Variables for current readings and deltas
	int current_accel_x, current_accel_y, current_accel_z;
	int delta_accel_x, delta_accel_y, delta_accel_z;

	// Variables for magnitude (if still needed)
	float accel_magnitude = 0.0f;
	long long accel_x_sq, accel_y_sq, accel_z_sq;

	int chute = 0; // Current fall status (0 or 1)
	bool fall_detected_flag = false; // Flag to indicate a NEW fall was just detected

	uint32_t last_sensor_read_time = 0;
	uint32_t last_thingspeak_send_time = 0;
	uint32_t current_time = 0;

	// --- Snapshot variables to store data AT THE MOMENT OF THE FALL ---
	int fall_data_gyro_x, fall_data_gyro_y, fall_data_gyro_z;
	int fall_data_delta_x, fall_data_delta_y, fall_data_delta_z;
	int fall_data_chute; // This will always be 1
	int fall_data_magnitude;

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
  MX_DFSDM1_Init();
  MX_I2C2_Init();
  MX_QUADSPI_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */
  printf("****** Sensor initialization ******\n\n\r");
  BSP_TSENSOR_Init();
  BSP_HSENSOR_Init();
  BSP_PSENSOR_Init();
  BSP_MAGNETO_Init();
  BSP_GYRO_Init();
  BSP_ACCELERO_Init();

  init_sensors() ;
  configure_user_height(173.5f) ;

  /*Initialize WIFI module */

  if (wifi_connect()==-1)  {

		resolve_hostname("api.thingspeak.com",RemoteIP2);
		Socket2 = Server_Connect(1, RemoteIP2, RemotePORT2) ;
  	    //Socket =  Server_Connect(0, RemoteIP, RemotePORT) ;

  }

  float ax_before, ay_before, az_before = 0 ;

  float ax = 0;
  float ay = 0;
  float az = 0;



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1){
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3*/


	  uint32_t current_time = HAL_GetTick();  // ← C'EST TOUT !


	  setAccelXYZ();
	  setGyroXYZ() ;



	  int16_t gx_raw = get_gyro_int(0);
	  int16_t gy_raw = get_gyro_int(1);
	  int16_t gz_raw = get_gyro_int(2);

	  float ax_raw = get_accel(0);
	  float ay_raw = get_accel(1);
	  float az_raw = get_accel(2);


	  ax_before = ax ;
	  ay_before = ay ;
	  az_before = az ;

	   ax = accel_raw_to_g(ax_raw);
	   ay = accel_raw_to_g(ay_raw);
	   az = accel_raw_to_g(az_raw);

	  float gx = gyro_raw_to_dps(gx_raw);
	  float gy = gyro_raw_to_dps(gy_raw);
	  float gz = gyro_raw_to_dps(gz_raw);


	  int previous_chute_state = chute; // Store previous state
	  int chute_detected = 0 ;


	  if (current_time - last_sensor_read_time >= SENSOR_READ_INTERVAL_MS) {
	  		  last_sensor_read_time = current_time; // Update last read time


	  		  current_accel_x = get_accel(0);
	  		  current_accel_y = get_accel(1);
	  		  current_accel_z = get_accel(2);

	  		  delta_accel_x = current_accel_x - initial_accel_x;
	  		  delta_accel_y = current_accel_y - initial_accel_y;
	  		  delta_accel_z = current_accel_z - initial_accel_z;

	  		  accel_x_sq = (long long)delta_accel_x * delta_accel_x; // Use current accel for magnitude
	  		  accel_y_sq = (long long)delta_accel_y * delta_accel_y;
	  		  accel_z_sq = (long long)delta_accel_z * delta_accel_z;
	  		  accel_magnitude = sqrtf((float)(accel_x_sq + accel_y_sq + accel_z_sq));

	  		  // --- Fall Detection Logic (Replace simulation with your actual logic) ---
	  		if (detect_fall(current_time)) {
	  			  	  chute = 1;
	  			  	  chute_detected = 1 ;
	  			  } else {
	  				  chute = 0;
	  			  }

	  		  // --- End of Fall Detection Logic ---

	  		  // Set the flag and TAKE DATA SNAPSHOT if a *new* fall was just detected
	  		  if (chute == 1 && previous_chute_state == 0) {
	  			  // Only latch data if a fall is not already pending to be sent
	  			  if (!fall_detected_flag) {
	  				  fall_detected_flag = true;
	  				  printf("\n *** FALL DETECTED! (FLAG SET & DATA LATCHED) *** \n");

	  				  // --- Take the snapshot of the data AT THIS MOMENT ---
	  				  fall_data_gyro_x = get_gyro_int(0);
	  				  fall_data_gyro_y = get_gyro_int(1);
	  				  fall_data_gyro_z = get_gyro_int(2);
	  				  fall_data_delta_x = delta_accel_x;
	  				  fall_data_delta_y = delta_accel_y;
	  				  fall_data_delta_z = delta_accel_z;
	  				  fall_data_chute = 1; // Explicitly set to 1
	  				  fall_data_magnitude = (int)accel_magnitude;
	  				chute_detected = 0 ;
	  			  }
	  		  }

	  		  // Logs (Optional, can be removed to save time in loop)
	  		   /*
	  		  printf("---------------------------------------------------------------------------------\r\n");
	  		   printf("Time: %lu ms\r\n", current_time);
	  		   printf("GYRO X: %d, GYRO Y: %d, GYRO Z: %d \r\n", get_gyro_int(0), get_gyro_int(1), get_gyro_int(2));
	  		   printf("DELTA ACCEL X: %d, DELTA ACCEL Y: %d, DELTA ACCEL Z: %d \r\n", delta_accel_x, delta_accel_y, delta_accel_z);
	  		   printf("ACCEL Magnitude: %.2f\r\n", accel_magnitude);
	  		   printf("Current Fall Status: %d\r\n", chute);
	  		   */
	  		  // printf("DELTA ACCEL X: %d, DELTA ACCEL Y: %d, DELTA ACCEL Z: %d \r\n", delta_accel_x, delta_accel_y, delta_accel_z);
	  		  // printf("ACCEL Magnitude: %.2f\r\n", accel_magnitude);
	  	  } // End of sensor reading block


	  	  // --- 2. ThingSpeak Sending Logic ---

	  	  bool attempt_send = false; // Should we consider sending now?
	  	  bool is_fall_event_trigger = false; // Was the trigger a fall?

	  	  // Condition 1: New fall detected?
	  	  if (fall_detected_flag) {
	  		  attempt_send = true;
	  		  is_fall_event_trigger = true;
	  		  printf("Send Reason: Fall detected!\r\n");
	  	  }

	  	  // Condition 2: Regular send interval elapsed?
	  	  if (current_time - last_thingspeak_send_time >= THINGSPEAK_SEND_INTERVAL_MS) {
	  		  attempt_send = true;
	  		  is_fall_event_trigger = false;
	  		  printf("Send Reason: Regular interval (%lu ms).\r\n", THINGSPEAK_SEND_INTERVAL_MS);
	  	  }

	  	  // --- Check Rate Limit and Attempt Send ---
	  	  if (attempt_send) {
	  		  // Check if minimum interval since LAST successful send has passed
	  		  if (current_time - last_thingspeak_send_time >= THINGSPEAK_MIN_SEND_INTERVAL_MS) {

	  			  if (Socket2 != -1) {

	  				  if (is_fall_event_trigger) {
	  					  // --- Send the LATCHED FALL DATA ---
	  					  printf("Attempting Send - Reason: Fall Event (using latched data)\r\n");
	  					  ret = write_data_TS(&Socket2, &Datalen, 8,
	  										  fall_data_gyro_x, fall_data_gyro_y, fall_data_gyro_z,
	  										  fall_data_delta_x, fall_data_delta_y, fall_data_delta_z,
	  										  fall_data_chute, // This will be 1
	  										  fall_data_magnitude);
	  				  } else {
	  					  // --- Send the CURRENT data (regular update) ---
	  					  printf("Attempting Send - Reason: Regular Interval (using current data)\r\n");
	  					  ret = write_data_TS(&Socket2, &Datalen, 8,
	  										  get_gyro_int(0), get_gyro_int(1), get_gyro_int(2),
	  										  delta_accel_x, delta_accel_y, delta_accel_z,
	  										  chute, // This will be 0
	  										  (int)accel_magnitude);
	  				  }

	  				  // --- Handle Send Result ---
	  				  if (ret == WIFI_STATUS_OK) {
	  					  printf("Send successful to thingspeak.com... status %ld\r\n", ret);
	  					  last_thingspeak_send_time = HAL_GetTick(); // Update time of SUCCESSFUL send

	  					  // If this send was for a fall, reset the flag
	  					  if (is_fall_event_trigger) {
	  						  fall_detected_flag = false;
	  						  printf("Fall flag reset after successful send.\r\n");
	  					  }
	  				  } else {
	  					  printf("> ERROR : Failed to Send Data (%ld), connection closed\n\r", ret);
	  					  WIFI_CloseClientConnection(Socket2);
	  					  Socket2 = -1;
	  					  // Reconnect logic will be handled below
	  				  }
	  			  } else {
	  				   printf("Attempt Send: Socket disconnected, delaying send.\r\n");
	  				   // Reconnect logic will be handled below
	  			  }
	  		  } else {
	  			  // Minimum interval not yet passed
	  			  // Print this message only if a send was actually attempted
	  		  }
	  	  } // End of attempt_send block



	  	if ((Socket2 == -1) && (current_time - last_thingspeak_send_time >= 5000)) {
			  last_thingspeak_send_time = current_time;
			  WIFI_CloseClientConnection(Socket2);
			  if (wifi_connect() == -1) {
					printf("Wi-Fi not connected. Fixing hostname...\r\n");
					if(resolve_hostname("api.thingspeak.com",RemoteIP2)) {
						Socket2 = Server_Connect(1, RemoteIP2, RemotePORT2) ;
						if (Socket2 != -1) {
						   last_thingspeak_send_time = HAL_GetTick();
						   printf("Reconnected to Thingspeak.\r\n");
						}
					} else {
						 printf("> ERROR: Failed to solve hostname in reconnection.\r\n");
					}
			   } else {
				   printf("> ERROR: Failed to find WiFi.\r\n");
			   }

	  		  HAL_Delay(50); // e.g., 50ms
	  }






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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief DFSDM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DFSDM1_Init(void)
{

  /* USER CODE BEGIN DFSDM1_Init 0 */

  /* USER CODE END DFSDM1_Init 0 */

  /* USER CODE BEGIN DFSDM1_Init 1 */

  /* USER CODE END DFSDM1_Init 1 */
  hdfsdm1_channel1.Instance = DFSDM1_Channel1;
  hdfsdm1_channel1.Init.OutputClock.Activation = ENABLE;
  hdfsdm1_channel1.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm1_channel1.Init.OutputClock.Divider = 2;
  hdfsdm1_channel1.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm1_channel1.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel1.Init.Input.Pins = DFSDM_CHANNEL_FOLLOWING_CHANNEL_PINS;
  hdfsdm1_channel1.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
  hdfsdm1_channel1.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hdfsdm1_channel1.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel1.Init.Awd.Oversampling = 1;
  hdfsdm1_channel1.Init.Offset = 0;
  hdfsdm1_channel1.Init.RightBitShift = 0x00;
  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DFSDM1_Init 2 */

  /* USER CODE END DFSDM1_Init 2 */

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
  hi2c2.Init.Timing = 0x10D19CE4;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief QUADSPI Initialization Function
  * @param None
  * @retval None
  */
static void MX_QUADSPI_Init(void)
{

  /* USER CODE BEGIN QUADSPI_Init 0 */

  /* USER CODE END QUADSPI_Init 0 */

  /* USER CODE BEGIN QUADSPI_Init 1 */

  /* USER CODE END QUADSPI_Init 1 */
  /* QUADSPI parameter configuration*/
  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler = 2;
  hqspi.Init.FifoThreshold = 4;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_HALFCYCLE;
  hqspi.Init.FlashSize = 23;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN QUADSPI_Init 2 */

  /* USER CODE END QUADSPI_Init 2 */

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
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.battery_charging_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, M24SR64_Y_RF_DISABLE_Pin|M24SR64_Y_GPO_Pin|ISM43362_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ARD_D10_Pin|SPBTLE_RF_RST_Pin|ARD_D9_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ARD_D8_Pin|ISM43362_BOOT0_Pin|ISM43362_WAKEUP_Pin|LED2_Pin
                          |SPSGRF_915_SDN_Pin|ARD_D5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, USB_OTG_FS_PWR_EN_Pin|PMOD_RESET_Pin|STSAFE_A100_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPBTLE_RF_SPI3_CSN_GPIO_Port, SPBTLE_RF_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, VL53L0X_XSHUT_Pin|LED3_WIFI__LED4_BLE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPSGRF_915_SPI3_CSN_GPIO_Port, SPSGRF_915_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ISM43362_SPI3_CSN_GPIO_Port, ISM43362_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : M24SR64_Y_RF_DISABLE_Pin M24SR64_Y_GPO_Pin ISM43362_RST_Pin ISM43362_SPI3_CSN_Pin */
  GPIO_InitStruct.Pin = M24SR64_Y_RF_DISABLE_Pin|M24SR64_Y_GPO_Pin|ISM43362_RST_Pin|ISM43362_SPI3_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_OTG_FS_OVRCR_EXTI3_Pin SPSGRF_915_GPIO3_EXTI5_Pin SPBTLE_RF_IRQ_EXTI6_Pin PE1 */
  GPIO_InitStruct.Pin = USB_OTG_FS_OVRCR_EXTI3_Pin|SPSGRF_915_GPIO3_EXTI5_Pin|SPBTLE_RF_IRQ_EXTI6_Pin|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_EXTI13_Pin */
  GPIO_InitStruct.Pin = BUTTON_EXTI13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_EXTI13_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_A5_Pin ARD_A4_Pin ARD_A3_Pin ARD_A2_Pin
                           ARD_A1_Pin ARD_A0_Pin */
  GPIO_InitStruct.Pin = ARD_A5_Pin|ARD_A4_Pin|ARD_A3_Pin|ARD_A2_Pin
                          |ARD_A1_Pin|ARD_A0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 ARD_D0_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_0|ARD_D0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D10_Pin SPBTLE_RF_RST_Pin ARD_D9_Pin */
  GPIO_InitStruct.Pin = ARD_D10_Pin|SPBTLE_RF_RST_Pin|ARD_D9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D4_Pin */
  GPIO_InitStruct.Pin = ARD_D4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(ARD_D4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D7_Pin */
  GPIO_InitStruct.Pin = ARD_D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARD_D7_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D13_Pin ARD_D12_Pin ARD_D11_Pin */
  GPIO_InitStruct.Pin = ARD_D13_Pin|ARD_D12_Pin|ARD_D11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D3_Pin */
  GPIO_InitStruct.Pin = ARD_D3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARD_D3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D6_Pin */
  GPIO_InitStruct.Pin = ARD_D6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARD_D6_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D8_Pin ISM43362_BOOT0_Pin ISM43362_WAKEUP_Pin LED2_Pin
                           SPSGRF_915_SDN_Pin ARD_D5_Pin SPSGRF_915_SPI3_CSN_Pin */
  GPIO_InitStruct.Pin = ARD_D8_Pin|ISM43362_BOOT0_Pin|ISM43362_WAKEUP_Pin|LED2_Pin
                          |SPSGRF_915_SDN_Pin|ARD_D5_Pin|SPSGRF_915_SPI3_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LPS22HB_INT_DRDY_EXTI0_Pin LSM6DSL_INT1_EXTI11_Pin ARD_D2_Pin HTS221_DRDY_EXTI15_Pin
                           PMOD_IRQ_EXTI12_Pin */
  GPIO_InitStruct.Pin = LPS22HB_INT_DRDY_EXTI0_Pin|LSM6DSL_INT1_EXTI11_Pin|ARD_D2_Pin|HTS221_DRDY_EXTI15_Pin
                          |PMOD_IRQ_EXTI12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_OTG_FS_PWR_EN_Pin SPBTLE_RF_SPI3_CSN_Pin PMOD_RESET_Pin STSAFE_A100_RESET_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_PWR_EN_Pin|SPBTLE_RF_SPI3_CSN_Pin|PMOD_RESET_Pin|STSAFE_A100_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : VL53L0X_XSHUT_Pin LED3_WIFI__LED4_BLE_Pin */
  GPIO_InitStruct.Pin = VL53L0X_XSHUT_Pin|LED3_WIFI__LED4_BLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : VL53L0X_GPIO1_EXTI7_Pin LSM3MDL_DRDY_EXTI8_Pin */
  GPIO_InitStruct.Pin = VL53L0X_GPIO1_EXTI7_Pin|LSM3MDL_DRDY_EXTI8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PMOD_SPI2_SCK_Pin */
  GPIO_InitStruct.Pin = PMOD_SPI2_SCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PMOD_SPI2_SCK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PMOD_UART2_RTS_Pin PMOD_UART2_TX_Pin PMOD_UART2_RX_Pin */
  GPIO_InitStruct.Pin = PMOD_UART2_RTS_Pin|PMOD_UART2_TX_Pin|PMOD_UART2_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D15_Pin ARD_D14_Pin */
  GPIO_InitStruct.Pin = ARD_D15_Pin|ARD_D14_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
 switch (GPIO_Pin)
 {
 case (GPIO_PIN_1):
 {
 SPI_WIFI_ISR();
 break;
 }
 default:
 {
 break;
 }
 }
}
int __io_putchar(int ch)
{
 HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
 return ch;
}



int32_t write_data_TS(int32_t* socket, uint16_t* datalen, int nb_parameters, ...){
    char sbuffer[512];
    char fields[256] = "{";
    va_list args;
    va_start(args, nb_parameters);

    for (int i = 1; i <= nb_parameters-1; i++) {
        char tmp[32];
        int val = va_arg(args, int);
        sprintf(tmp, "\"field%d\": %d, ", i, val);
        strcat(fields, tmp);
    }
    char tmp[32];
    int val = va_arg(args, int);
    sprintf(tmp, "\"field%d\": %d}", nb_parameters, val);
    strcat(fields, tmp);

    va_end(args);

    snprintf(sbuffer, sizeof(sbuffer), "GET /update?api_key=%s HTTP/1.1\r\nHost:api.thingspeak.com\r\nContent-Type: application/json\r\nContent-Length:%d\r\n\r\n%s", thingspeak_APIkey_write, (int)strlen(fields),fields);
    printf("HTTP command %s\r\n", sbuffer);


    return WIFI_SendData(*socket, (uint8_t *)sbuffer, strlen(sbuffer), datalen, WIFI_WRITE_TIMEOUT);
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











/*
	  if(Socket != -1){
	   ret = WIFI_ReceiveData(Socket, (uint8_t *)RxData, sizeof(RxData)-1, &Datalen, WIFI_READ_TIMEOUT);
	   if(ret == WIFI_STATUS_OK){
		   if(Datalen > 0){
			   RxData[Datalen]=0;
			   printf("Received: %s\n\r",RxData);
			   switch(RxData[0]){
			   	   case 't':
			   		   sensor_value = BSP_TSENSOR_ReadTemp();
			   		   tmpInt1 = sensor_value;
			   		   tmpFrac = sensor_value - tmpInt1;
			   		   tmpInt2 = trunc(tmpFrac * 100);
			   		   sprintf(TxData," TEMPERATURE = %d.%02d\n\r", tmpInt1,tmpInt2);
			   		   break ;
			   	   case 'h':
			   		   sensor_value = BSP_HSENSOR_ReadHumidity();
			   		   tmpInt1 = sensor_value;
			   		   tmpFrac = sensor_value - tmpInt1;
			   		   tmpInt2 = trunc(tmpFrac * 100);
			   		   sprintf(TxData," HUMIDITY = %d.%02d\n\r", tmpInt1,tmpInt2);
			   		   break ;
			   	   case 'p':
			   		   sensor_value = BSP_PSENSOR_ReadPressure();
			   		   tmpInt1 = sensor_value;
			   		   tmpFrac = sensor_value - tmpInt1;
			   		   tmpInt2 = trunc(tmpFrac * 100);
			   		   sprintf(TxData," PRESSURE = %d.%02d\n\r", tmpInt1, tmpInt2);
			   		   break ;
			   	   case 'm':
			   		   BSP_MAGNETO_GetXYZ(pDataXYZ);
			   		   sprintf(TxData," MAGNETO_X,Y,Z = %d %d %d \n\r", pDataXYZ[0],pDataXYZ[1],pDataXYZ[2]);
			   		   break ;
			   	   case 'g':
			   		   BSP_GYRO_GetXYZ(pGyroDataXYZ);
			   		   sprintf(TxData," GYRO_X,Y,Z = %d %d %d \n\r", (int)(pGyroDataXYZ[0]*100.0),(int)(pGyroDataXYZ[1]*100),(int)(pGyroDataXYZ[2]*100));
			   		   break ;
			   	   case 'a':
			   		   BSP_ACCELERO_AccGetXYZ(pDataXYZ);
			   		   sprintf(TxData," ACCELERO_X,Y,Z = %d %d %d \n\r",pDataXYZ[0],pDataXYZ[1],pDataXYZ[2]);
			   		   break ;
			   	   default:
			   		   sprintf(TxData,"Tié le tigre du bengale\n\r");
			   		   break ;
			   }
			   ret = WIFI_SendData(Socket, (uint8_t *)TxData, strlen(TxData), &Datalen, WIFI_WRITE_TIMEOUT);
			   if (ret != WIFI_STATUS_OK)
			   {
				   printf("> ERROR : Failed to Send Data, connectionclosed\n\r");
				   break;
			   }
		   }
	   }
	   else{
		   printf("> ERROR : Failed to Receive Data, connection closed\n\r");
		   break;
	   }
	  }*/
