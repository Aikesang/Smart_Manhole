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
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "adc_pressure.h"
#include "stdio.h"
#include "string.h"
#include "stdbool.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART_BUFFER_SIZE 200
#define DC_MOTOR1_PWM_CHANNEL    TIM_CHANNEL_1
#define DC_MOTOR2_PWM_CHANNEL    TIM_CHANNEL_2
#define DC_MOTOR_MAX_RUN_TIME    300000
#define CURRENT_THRESHOLD        4.0f
#define NB_CHECK_WAKEUP_COUNT    1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
typedef enum{
	Motor1,
	Motor2
} Motor;


typedef enum {
    MOTOR_STOP = 0,
    MOTOR_RUNNING
} Motor_State;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Send_ATCommand(const char *command);
void Send_RawData(const char *data, uint16_t length);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

void Send_Wait_Start(const char* command, uint16_t timeout_ms);
void Send_Wait_Start_Raw(const char* data, uint16_t length, uint16_t timeout_ms);
bool Send_Wait_IsDone(void);

bool Publish_MQTT(const char *payload, uint16_t timeout_ms);

void Move_DCMotor(Motor motor, Motor_State state);
uint8_t Calibrate(Motor motor);
void Reset_NBModule(void);
bool Check_NB_Connection(void);
void Delay_Start(uint32_t ms);
bool Delay_Elapsed(void);

void RTC_Set_Wakeup_Timer(uint16_t seconds);
void Enter_Stop_Mode(void);
void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc);
bool Check_Subscribe(void);
static void MQTT_Publish(const char* payload);
static bool lid_is_open = false;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t rx_buffer[1];
uint8_t tx_buffer[UART_BUFFER_SIZE] = {0};
uint8_t rx_buffer_full[UART_BUFFER_SIZE] = {0};
volatile uint8_t rx_index = 0;



float current_from_motor;
float current_value;
volatile uint8_t response_ready = 0;
char response_buffer[UART_BUFFER_SIZE];
volatile bool command_response_received = false;
volatile bool command_response_ok = false;

volatile uint8_t calibration_done = 0;

volatile bool open_lid = false;  // Open cap command
volatile bool close_lid = false;  // Close cap command
volatile bool stop_lid = false;  // Stop cap command

uint32_t last_nb_check_ms = 0;
volatile uint8_t connection_fail_count = 0;
uint8_t wakeup_counter = 0;

uint32_t delay_start = 0;
uint32_t delay_duration = 0;
bool delay_active = false;
const char *waiting_response = NULL;

static bool sendInProgress = false;
static uint32_t sendStartTick = 0;
static uint16_t sendTimeout = 0;
static const char* sendCommand = NULL;

static char mqtt_payload_water[128];
static char mqtt_payload_current[128];
static char mqtt_cmd[150];

volatile bool wakeup_event = false;

Motor_State Motor1_State = MOTOR_STOP;  // Motor1状态
Motor_State Motor2_State = MOTOR_STOP;  // Motor2状态

int water_level_threshold = 50;
volatile bool check_cmd_received = false;
volatile bool check_subscribe = false;


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
  MX_ADC_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  ADC_Manual_Calibration();

  rx_buffer[0] = 0;
  memset(tx_buffer, 0, UART_BUFFER_SIZE);
  memset(rx_buffer_full, 0, UART_BUFFER_SIZE);

  HAL_UART_Receive_IT(&huart2, rx_buffer, 1);


  Send_Wait_Start("AT+QSCLK=0", 2000);
  while(!Send_Wait_IsDone()) {

   }

  Send_Wait_Start("AT+CPSMS=0", 2000);
  while(!Send_Wait_IsDone()) {

   }


  Send_Wait_Start("AT+CGDCONT=1,IP,quectelnb",2000);
  while(!Send_Wait_IsDone()) {

   }

  Send_Wait_Start("AT+QMTCFG=version,0,1",2000);
  while(!Send_Wait_IsDone()) {

   }

  Send_Wait_Start("AT+QMTOPEN=0,183.230.102.116,1883",2000);
  while(!Send_Wait_IsDone()) {

   }


  Send_Wait_Start("AT+QMTCONN=0,m002,I773FLY13q,version=2018-10-31&res=products%2FI773FLY13q%2Fdevices%2Fm002&et=1787896162&method=md5&sign=25YWkBFNMJ9Bag7ZrzuZxQ%3D%3D",3000);
  while(!Send_Wait_IsDone()) {

   }


  Send_Wait_Start("AT+QMTSUB=0,1,$sys/I773FLY13q/m002/#,0",2000);
  while(!Send_Wait_IsDone()) {

   }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  HAL_GPIO_WritePin(IO3_GPIO_Port, IO3_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOB, IO4_Pin, GPIO_PIN_SET);
	  HAL_Delay(100);

	  wakeup_counter++;
	  if(wakeup_counter >= NB_CHECK_WAKEUP_COUNT){

		  bool is_connected = Check_NB_Connection();
		  HAL_Delay(100);
		  bool is_subscribed = false;


		  if (is_connected)
		      {
			  	  HAL_Delay(100);
			  	  is_subscribed = Check_Subscribe();
			  	  HAL_Delay(100);
		      }

		  if (!is_connected || !is_subscribed)
		  {
		      Reset_NBModule();
		      HAL_Delay(100);
		  }
		      wakeup_counter = 0;
	  }


		if (open_lid) {
			open_lid = false;
			snprintf(mqtt_payload_current, sizeof(mqtt_payload_current), "{\"id\":\"123\",\"params\":{\"mnh\":{\"value\":\"open\"}}}");
			MQTT_Publish(mqtt_payload_current);



			uint8_t result = Calibrate(Motor2);
			if (result == 0) { lid_is_open = true; }
			if (result == 1){
				snprintf(mqtt_payload_current, sizeof(mqtt_payload_current), "{\"id\":\"123\",\"params\":{\"mnh\":{\"value\":\"overo\"}}}");
				MQTT_Publish(mqtt_payload_current);
			}
			if (result == 2){

				snprintf(mqtt_payload_current, sizeof(mqtt_payload_current), "{\"id\":\"123\",\"params\":{\"mnh\":{\"value\":\"timeo\"}}}");
				MQTT_Publish(mqtt_payload_current);
			}
			if (result == 3){

				snprintf(mqtt_payload_current, sizeof(mqtt_payload_current), "{\"id\":\"123\",\"params\":{\"mnh\":{\"value\":\"halto\"}}}");
				MQTT_Publish(mqtt_payload_current);
			}
		}

		if (close_lid) {
			close_lid = false;

			snprintf(mqtt_payload_current, sizeof(mqtt_payload_current), "{\"id\":\"123\",\"params\":{\"mnh\":{\"value\":\"close\"}}}");
			MQTT_Publish(mqtt_payload_current);

			uint8_t result = Calibrate(Motor1);

			if (result == 0) { lid_is_open = false; }  // <-- mark lid closed

			if (result == 1){

				snprintf(mqtt_payload_current, sizeof(mqtt_payload_current), "{\"id\":\"123\",\"params\":{\"mnh\":{\"value\":\"overc\"}}}");
				MQTT_Publish(mqtt_payload_current);
			}
			if (result == 2){

				snprintf(mqtt_payload_current, sizeof(mqtt_payload_current), "{\"id\":\"123\",\"params\":{\"mnh\":{\"value\":\"timec\"}}}");
				MQTT_Publish(mqtt_payload_current);
			}
			if (result == 3){

				snprintf(mqtt_payload_current, sizeof(mqtt_payload_current), "{\"id\":\"123\",\"params\":{\"mnh\":{\"value\":\"haltc\"}}}");
				MQTT_Publish(mqtt_payload_current);
			}
		}


		if( !open_lid && !close_lid )
		{
			int water_height_cm = (int)Get_Height_from_Pressure();//water level exceeding update
	    	if(water_height_cm > water_level_threshold && !lid_is_open){

	    		snprintf(mqtt_payload_water, sizeof(mqtt_payload_water), "{\"id\":\"123\",\"params\":{\"level\":{\"value\":%d}}}", water_height_cm);
	    		MQTT_Publish(mqtt_payload_water);


				snprintf(mqtt_payload_current, sizeof(mqtt_payload_current), "{\"id\":\"123\",\"params\":{\"mnh\":{\"value\":\"openw\"}}}");
				MQTT_Publish(mqtt_payload_current);

	    		uint8_t result = Calibrate(Motor2);

	            if (result == 0)
	            {
	                lid_is_open = true;  // <-- mark lid open on success
	            }
	    		if (result == 1){

					snprintf(mqtt_payload_current, sizeof(mqtt_payload_current), "{\"id\":\"123\",\"params\":{\"mnh\":{\"value\":\"overo\"}}}");
					MQTT_Publish(mqtt_payload_current);
	    		}
				if (result == 2){
					snprintf(mqtt_payload_current, sizeof(mqtt_payload_current), "{\"id\":\"123\",\"params\":{\"mnh\":{\"value\":\"timeo\"}}}");
					MQTT_Publish(mqtt_payload_current);

				}
				if (result == 3){
					snprintf(mqtt_payload_current, sizeof(mqtt_payload_current), "{\"id\":\"123\",\"params\":{\"mnh\":{\"value\":\"halto\"}}}");
					MQTT_Publish(mqtt_payload_current);
				}
			}

		}


 	        float battery_v = Get_Average_voltage(5); //Battery voltage update

	   	    snprintf(mqtt_payload_current, sizeof(mqtt_payload_current), "{\"id\":\"123\",\"params\":{\"battery\":{\"value\":%.1f}}}", battery_v);
	   	    MQTT_Publish(mqtt_payload_current);

	        uint32_t listen_timeout = 10000;
	        uint32_t listen_start = HAL_GetTick();
	        bool need_sleep = true;
	        while((HAL_GetTick() - listen_start) < listen_timeout)
	         {

	           if(open_lid || close_lid)
	           {
	             need_sleep = false;
	             break;
	           }
	           __WFI();
	         }

	      if (need_sleep)
	      {
		   	    snprintf(mqtt_payload_current, sizeof(mqtt_payload_current), "{\"id\":\"123\",\"params\":{\"mnh\":{\"value\":\"sleep\"}}}");
		   	    MQTT_Publish(mqtt_payload_current);


		        RTC_Set_Wakeup_Timer(330);
		        Enter_Stop_Mode();
	      }

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_RTC;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// Add this helper in USER CODE 4 section
static void MQTT_Publish(const char* payload)
{
    int payload_len = (int)strlen(payload);
    snprintf(mqtt_cmd, sizeof(mqtt_cmd),
             "AT+QMTPUB=0,0,0,0,$sys/I773FLY13q/m002/thing/property/post,%d",
             payload_len);
    Send_Wait_Start(mqtt_cmd, 1000);
    while (!Send_Wait_IsDone()) {}
    Send_Wait_Start(payload, 1000);
    while (!Send_Wait_IsDone()) {}
}




void RTC_Set_Wakeup_Timer(uint16_t seconds)
{
    // Disable write protection
    HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);

    // L0 series usually uses a 1Hz clock for the wakeup timer if configured with CK_SPRE
    // 0x0000 is the value for 16-bit counter.
    // Ensure CubeMX RTC WakeUp Clock is set to: RTC_WAKEUPCLOCK_CK_SPRE_16BITS

    if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, seconds, RTC_WAKEUPCLOCK_CK_SPRE_16BITS) != HAL_OK)
    {
        Error_Handler();
    }
}

void Enter_Stop_Mode(void)
{

    HAL_GPIO_WritePin(IO3_GPIO_Port, IO3_Pin, GPIO_PIN_RESET);
    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, IO4_Pin, GPIO_PIN_RESET);
    HAL_ADC_Stop(&hadc);
    // 1. Suspend SysTick to prevent it from waking the MCU every 1ms
    HAL_SuspendTick();

    // 2. Enter Stop Mode
    // Regulator Low Power implies deeper sleep but slower wake-up
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);


    SystemClock_Config();

    MX_GPIO_Init();


    MX_ADC_Init();
    MX_USART2_UART_Init();
    MX_TIM2_Init();


    HAL_ResumeTick();


    HAL_UART_Receive_IT(&huart2, rx_buffer, 1);
}

// RTC Interrupt Callback
void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc)
{
    // This function is called when the timer expires
    wakeup_event = true;
}


void Delay_Start(uint32_t ms) {
    delay_start = HAL_GetTick();
    delay_duration = ms;
    delay_active = true;
}

// Check if delay has elapsed
bool Delay_Elapsed(void) {
    if (!delay_active) return true;
    if ((HAL_GetTick() - delay_start) >= delay_duration) {
        delay_active = false;
        return true;
    }
    return false;
}


bool Check_NB_Connection(void)
{
	__disable_irq();
	memset(rx_buffer_full, 0, UART_BUFFER_SIZE);
	rx_index = 0;
	__enable_irq();
	check_cmd_received = false;
	// Check network attach status
	Send_Wait_Start("AT+CGATT?", 2000);
    while(!Send_Wait_IsDone()) {
    }

    bool result = check_cmd_received;
    memset(rx_buffer_full, 0, UART_BUFFER_SIZE);
    rx_index = 0;
    check_cmd_received = false;

   	return result;
}

bool Check_Subscribe(void)
{

	__disable_irq();
	memset(rx_buffer_full, 0, UART_BUFFER_SIZE);
	rx_index = 0;
	__enable_irq();
	check_subscribe = false;

    Send_Wait_Start("AT+QMTSUB=0,1,$sys/I773FLY13q/m002/#,0", 2000);
    while(!Send_Wait_IsDone()) {
    }

	bool result = check_subscribe;
	memset(rx_buffer_full, 0, UART_BUFFER_SIZE);
	rx_index = 0;
	check_subscribe = false;

	return result;


}

void Reset_NBModule(void)
{

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
    HAL_Delay(500);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_Delay(500);

	  rx_buffer[0] = 0;
	  memset(tx_buffer, 0, UART_BUFFER_SIZE);

	  __disable_irq();
	  memset(rx_buffer_full, 0, UART_BUFFER_SIZE);
	  rx_index = 0;
	  __enable_irq();

	  HAL_UART_Receive_IT(&huart2, rx_buffer, 1);

	  Send_Wait_Start("AT+QSCLK=0", 2000);
	    while(!Send_Wait_IsDone()) {

	     }

	    Send_Wait_Start("AT+CPSMS=0", 2000);
	    while(!Send_Wait_IsDone()) {

	     }

	    Send_Wait_Start("AT+CGDCONT=1,IP,quectelnb",2000);
	    while(!Send_Wait_IsDone()) {

	     }

	    Send_Wait_Start("AT+QMTCFG=version,0,1",2000);
	    while(!Send_Wait_IsDone()) {

	     }

	    Send_Wait_Start("AT+QMTOPEN=0,183.230.102.116,1883",2000);
	    while(!Send_Wait_IsDone()) {

	     }

	    Send_Wait_Start("AT+QMTCONN=0,m002,I773FLY13q,version=2018-10-31&res=products%2FI773FLY13q%2Fdevices%2Fm002&et=1787896162&method=md5&sign=25YWkBFNMJ9Bag7ZrzuZxQ%3D%3D",3000);
	    while(!Send_Wait_IsDone()) {

	     }

	    Send_Wait_Start("AT+QMTSUB=0,1,$sys/I773FLY13q/m002/#,0",2000);
	    while(!Send_Wait_IsDone()) {

	     }

}



void Move_DCMotor(Motor motor, Motor_State state)
{

	switch (motor)
	{
	case Motor1:
        if (state == MOTOR_RUNNING)
        {

            HAL_TIM_PWM_Start(&htim2, DC_MOTOR1_PWM_CHANNEL);
            Motor1_State = MOTOR_RUNNING;
        }
        else
        {

        	HAL_TIM_PWM_Stop(&htim2, DC_MOTOR1_PWM_CHANNEL);
            Motor1_State = MOTOR_STOP;
        }
        break;

	case Motor2:
		if (state == MOTOR_RUNNING)
		{
		   HAL_TIM_PWM_Start(&htim2, DC_MOTOR2_PWM_CHANNEL);
		   Motor2_State = MOTOR_RUNNING;
		}
		else
		{
			HAL_TIM_PWM_Stop(&htim2, DC_MOTOR2_PWM_CHANNEL);
		     Motor2_State = MOTOR_STOP;
		}
		break;

	default:
		break;
	}
}


uint8_t Calibrate(Motor motor)
{
    GPIO_PinState sensor_state;
	calibration_done = 0;
    GPIO_TypeDef* sensor_port = GPIOA;
    uint16_t sensor_pin = (motor == Motor1) ? GPIO_PIN_11 : GPIO_PIN_12;
    uint32_t start_time = HAL_GetTick();
    stop_lid = false;

    while (!calibration_done)
    {
    	sensor_state = HAL_GPIO_ReadPin(sensor_port, sensor_pin);
    	if (sensor_state == GPIO_PIN_RESET)
    	    {
    	      Move_DCMotor(motor, MOTOR_STOP);
    	      calibration_done = 1;
    	      return 0;
    	    }
    	else {
    	    	Move_DCMotor(motor, MOTOR_RUNNING);

    			current_from_motor = Get_Average_Current(10);
    			if (current_from_motor >= CURRENT_THRESHOLD)
    	        {
    				Move_DCMotor(motor, MOTOR_STOP);
    	            calibration_done = 1;
    	            return 1;
    	        }


    	        if ((HAL_GetTick() - start_time) > DC_MOTOR_MAX_RUN_TIME)
               {
    	        	Move_DCMotor(motor, MOTOR_STOP);
    	        	calibration_done = 1;
    	        	return 2;  //timeout
               }

    		}

    	if (stop_lid == true){
			Move_DCMotor(motor, MOTOR_STOP);
            calibration_done = 1;
            stop_lid = false;
            return 3;
    	}

        HAL_Delay(10);
    }
    return 0;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
       uint8_t received_char = rx_buffer[0];

        // Append received byte to buffer with overflow check
        if (rx_index < UART_BUFFER_SIZE - 1) {
            rx_buffer_full[rx_index++] = received_char;
        } else {
        	__disable_irq();
        	memset(rx_buffer_full, 0, UART_BUFFER_SIZE);
        	rx_index = 0;
        	__enable_irq();

        }


        if (received_char == '\n' ) {
            rx_buffer_full[rx_index] = '\0';  // Null-terminate string


         if (strstr((char*)rx_buffer_full, "QMTSUB: 0,1,0") != NULL) {
         	 check_subscribe = true;
         }

      	 if (strstr((char*)rx_buffer_full, "CGATT: 1") != NULL) {
      	     check_cmd_received = true;
      	 }

   	     if (strstr((char*)rx_buffer_full, "\"temp\":\"open\"") != NULL) {
   	         open_lid = true;
   	     } else if (strstr((char*)rx_buffer_full, "\"temp\":\"close\"") != NULL) {
   	         close_lid = true;
   	     } else if (strstr((char*)rx_buffer_full, "\"temp\":\"halt\"") != NULL){
   	    	 stop_lid = true;
   	     }

         char *level_ptr = NULL;
         level_ptr = strstr((char*)rx_buffer_full, "\"level\":");
         if (level_ptr != NULL) {
        	 level_ptr += 8;
        	 int new_level = atoi(level_ptr);

           if (new_level >= 0 && new_level <= 100) {
             water_level_threshold = new_level;
           }

         }
         __disable_irq();
         memset(rx_buffer_full, 0, UART_BUFFER_SIZE);
         rx_index = 0;
         __enable_irq();
        }

        // Rearm UART receive interrupt for next byte to keep receiving continuously
        HAL_UART_Receive_IT(&huart2, rx_buffer, 1);
    }
}



void Send_mqtt(const char *command) {

    snprintf((char*)tx_buffer, UART_BUFFER_SIZE, "%s\r\n", command);  // Add CRLF to the command
    HAL_UART_Transmit(&huart2, tx_buffer, strlen((char*)tx_buffer), 500);  // Send command
}

void Send_Wait_Start(const char* command, uint16_t timeout_ms) {
    Send_mqtt(command);
    sendCommand = command;
    sendTimeout = timeout_ms;
    sendStartTick = HAL_GetTick();
    sendInProgress = true;
}

bool Send_Wait_IsDone(void) {
    if (!sendInProgress) return true;

    uint32_t send_now = HAL_GetTick();
    if ((send_now - sendStartTick) >= sendTimeout) {
        sendInProgress = false;
        return true;  // timeout elapsed, assume send done
    }
    return false;  // still waiting
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
