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
#include "octospi.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../IMU/ICM42688P/icm42688p.h"
#include "../EEPROM/eeprom.h"
#include "../RECEIVER/FS-iA6B/FS-iA6B.h"
#include "../LED/MAIN_BOARD_RGB/ws2812.h"
#include "../HMC5883L/hmc5883l.h"
#include "dshot.h"
#include "../CMD/cmd.h"
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* *********** USART6 printf function code ************ */
int _write(int file, char* p, int len)
{
    for (int i = 0; i < len; i++) {
        while (!LL_USART_IsActiveFlag_TXE(USART6)) {
            if (LL_USART_IsActiveFlag_ORE(USART6) || LL_USART_IsActiveFlag_FE(USART6)) {
                LL_USART_ClearFlag_ORE(USART6);
                LL_USART_ClearFlag_FE(USART6);
                return -1; // Indicate error
            }
        }
        LL_USART_TransmitData8(USART6, *(p + i));
    }
    while (!LL_USART_IsActiveFlag_TC(USART6)) {}
    return len;
}
/* *********** USART6 printf function code ************ */


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MIN_THROTTLE 240   // Minimum DShot throttle (per protocol)
#define MAX_THROTTLE 2000 // Maximum DShot throttle (per protocol)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
unsigned char failsafe_flag = 0;
unsigned char motor_armed = 0; // Track arming state
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

extern uint8_t uart6_rx_flag;
extern uint8_t uart6_rx_data;
extern uint8_t uart4_rx_flag;
extern uint8_t uart4_rx_data;
extern uint8_t ibus_rx_buf[32];
extern uint8_t ibus_rx_cplt_flag;
extern uint8_t uart7_rx_data;
uint8_t telemetry_tx_buf[20];

extern uint16_t tim7_1ms_flag;
extern uint16_t tim7_2ms_flag;
extern uint16_t tim7_20ms_flag;
extern uint16_t tim7_50ms_flag;
extern uint16_t tim7_100ms_flag;
extern uint16_t tim7_200ms_flag;
extern uint16_t tim7_1000ms_flag;

float eeprom_pid_read[3];
float eeprom_gyro_read[3];
float eeprom_accel_read[3];
float eeprom_mag_read[3];
DualPID_t eeprom_roll_pid_read;
DualPID_t eeprom_pitch_pid_read;
PID_t eeprom_yaw_rate_pid_read;

AircraftLights_t aircraft_lights;

uint16_t my_motor_value[4] = {0, 0, 0, 0}; // Throttle values for 4 motors
static uint16_t last_dshot_throttle = MIN_THROTTLE; // For throttle smoothing

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

int Is_iBus_Throttle_Min(void);
int Is_iBus_Throttle_Armed(void);
void ESC_Calibration(void);
int Is_iBus_Received(void);
void check_command_timeout(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    HMC5883L_DMA_Complete_Callback(hi2c);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
    printf("I2C Error: %lu\n", hi2c->ErrorCode);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

	static int16_t gyro_offsets_lsb[3] = {8, 4, -2}; // Initialize to zero

	unsigned char motor_arming_flag = 0;
	unsigned short iBus_SwA_Prev = 0;
	unsigned char iBus_rx_cnt = 0;
	unsigned short ccr1, ccr2, ccr3, ccr4;

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_TIM5_Init();
  MX_OCTOSPI1_Init();
  MX_USART6_UART_Init();
  MX_TIM7_Init();
  MX_SPI3_Init();
  MX_UART4_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  LL_USART_EnableIT_RXNE(USART6);
//  StartupTone();
  LL_USART_EnableIT_RXNE_RXFNE(UART4);

  /* ======================== EEPROM Code START ======================== */

  if (EEPROM_Init() != W25Qxx_OK) {
	  printf("EEPROM Init Failed\r\n");
	  Error_Handler();
  }

  DroneConfig_t config;
  if (EEPROM_ReadConfig(&config) == W25Qxx_OK) {
	  printf("Config Loaded: Flight Mode %d, PID P: %.2f\r\n",
			  config.flight_mode, config.pid[0]);
  } else {
	  printf("No valid config found, loading defaults\r\n");
	  DroneConfig_t default_config = {
			  .accel_cal = {0.030256f, -0.026638f, -0.122559f},
			  .gyro_cal = {0.016646f, 0.173536f, -0.355303f},
			  .mag_cal = {1.50f, -1.50f, 0.50f},
			  .pid = {1.0f, 0.1f, 0.5f},
			  .flight_mode = 0,
			  .gps_lat = 0.0f,
			  .gps_lon = 0.0f,
			  .gps_alt = 0.0f,
			  .roll_pid = {
					  .out = {0.62f, 0.01f, 0.10f},
					  .in = {0.30f, 0.07f, 0.010f}
			  },
			  .pitch_pid = {
					  .out = {0.62f, 0.01f, 0.10f},
					  .in = {0.30f, 0.07f, 0.010f}
			  },
			  .yaw_rate_pid = {0.2f, 0.01f, 0.003f},
			  .lights = {
					  .rgb = {{255, 0, 0}, {0, 255, 0}, {0, 0, 255}, {255, 255, 255}},
					  .mode = 0
			  },
			  .crc = 0
	  };
	  default_config.crc = CalculateCRC32((uint8_t*)&default_config, sizeof(DroneConfig_t) - sizeof(uint32_t));
	  if (EEPROM_WriteConfig(&default_config) != W25Qxx_OK) {
		  printf("Failed to write default config\r\n");
		  Error_Handler();
	  }
	  printf("Default config written and verified\r\n");
  }

  if (EEPROM_GetPID(eeprom_pid_read) == W25Qxx_OK) {
	  printf("EEPROM PID read: P=%.2f, I=%.2f, D=%.2f\r\n",
			  eeprom_pid_read[0], eeprom_pid_read[1], eeprom_pid_read[2]);
  } else {
	  printf("Failed to read PID\r\n");
  }

  if (EEPROM_GetGyroCalibration(eeprom_gyro_read) == W25Qxx_OK) {
	  printf("EEPROM Gyro read: X=%.2f, Y=%.2f, Z=%.2f\r\n",
			  eeprom_gyro_read[0], eeprom_gyro_read[1], eeprom_gyro_read[2]);
  } else {
	  printf("Failed to read EEPROM Gyro Data\r\n");
  }

  if (EEPROM_GetAccelCalibration(eeprom_accel_read) == W25Qxx_OK) {
	  printf("EEPROM Accel read: X=%.2f, Y=%.2f, Z=%.2f\r\n",
			  eeprom_accel_read[0], eeprom_accel_read[1], eeprom_accel_read[2]);
  } else {
	  printf("Failed to read EEPROM Accel Data\r\n");
  }

  if (EEPROM_GetMagCalibration(eeprom_mag_read) == W25Qxx_OK) {
	  printf("EEPROM Mag read: X=%.2f, Y=%.2f, Z=%.2f\r\n",
			  eeprom_mag_read[0], eeprom_mag_read[1], eeprom_mag_read[2]);
  } else {
	  printf("Failed to read EEPROM Mag Data\r\n");
  }

  if (EEPROM_GetRollPID(&eeprom_roll_pid_read) == W25Qxx_OK) {
	  printf("Roll PID: Out P=%.3f, I=%.3f, D=%.3f, In P=%.3f, I=%.3f, D=%.3f\r\n",
			  eeprom_roll_pid_read.out.kp, eeprom_roll_pid_read.out.ki, eeprom_roll_pid_read.out.kd,
			  eeprom_roll_pid_read.in.kp, eeprom_roll_pid_read.in.ki, eeprom_roll_pid_read.in.kd);
  } else {
	  printf("Failed to read Roll PID\r\n");
  }

  if (EEPROM_GetPitchPID(&eeprom_pitch_pid_read) == W25Qxx_OK) {
	  printf("Pitch PID: Out P=%.3f, I=%.3f, D=%.3f, In P=%.3f, I=%.3f, D=%.3f\r\n",
			  eeprom_pitch_pid_read.out.kp, eeprom_pitch_pid_read.out.ki, eeprom_pitch_pid_read.out.kd,
			  eeprom_pitch_pid_read.in.kp, eeprom_pitch_pid_read.in.ki, eeprom_pitch_pid_read.in.kd);
  } else {
	  printf("Failed to read Pitch PID\r\n");
  }

  if (EEPROM_GetYawRatePID(&eeprom_yaw_rate_pid_read) == W25Qxx_OK) {
	  printf("Yaw Rate PID: P=%.3f, I=%.3f, D=%.3f\r\n",
			  eeprom_yaw_rate_pid_read.kp, eeprom_yaw_rate_pid_read.ki, eeprom_yaw_rate_pid_read.kd);
  } else {
	  printf("Failed to read Yaw Rate PID\r\n");
  }

  if (EEPROM_GetAircraftLights(&aircraft_lights) == W25Qxx_OK) {
	  printf("Lights: LED1(R=%d,G=%d,B=%d), LED2(R=%d,G=%d,B=%d), LED3(R=%d,G=%d,B=%d), LED4(R=%d,G=%d,B=%d), Mode=%d\r\n",
			  aircraft_lights.rgb[0][0], aircraft_lights.rgb[0][1], aircraft_lights.rgb[0][2],
			  aircraft_lights.rgb[1][0], aircraft_lights.rgb[1][1], aircraft_lights.rgb[1][2],
			  aircraft_lights.rgb[2][0], aircraft_lights.rgb[2][1], aircraft_lights.rgb[2][2],
			  aircraft_lights.rgb[3][0], aircraft_lights.rgb[3][1], aircraft_lights.rgb[3][2],
			  aircraft_lights.mode);
  } else {
	  printf("Failed to read Aircraft Lights\r\n");
  }

  /* ********************************* EEPROM Code END ********************************* */

  // Initialize PID gains
  //          roll_pid.out.kp = eeprom_roll_pid_read.out.kp;
  //          roll_pid.out.ki = eeprom_roll_pid_read.out.ki;
  //          roll_pid.out.kd = eeprom_roll_pid_read.out.kd;
  //          roll_pid.in.kp = eeprom_roll_pid_read.in.kp;
  //          roll_pid.in.ki = eeprom_roll_pid_read.in.ki;
  //          roll_pid.in.kd = eeprom_roll_pid_read.in.kd;
  //
  //          pitch_pid.out.kp = eeprom_pitch_pid_read.out.kp;
  //          pitch_pid.out.ki = eeprom_pitch_pid_read.out.ki;
  //          pitch_pid.out.kd = eeprom_pitch_pid_read.out.kd;
  //          pitch_pid.in.kp = eeprom_pitch_pid_read.in.kp;
  //          pitch_pid.in.ki = eeprom_pitch_pid_read.in.ki;
  //          pitch_pid.in.kd = eeprom_pitch_pid_read.in.kd;
  //
  //          yaw_rate_pid.kp = eeprom_yaw_rate_pid_read.kp;
  //          yaw_rate_pid.ki = eeprom_yaw_rate_pid_read.ki;
  //          yaw_rate_pid.kd = eeprom_yaw_rate_pid_read.kd;


  printf("\n");

  /* *********** iBus Calibration Check ************ */

  while (Is_iBus_Received() == 0) {
	  Buzzer_On(3000);
	  HAL_Delay(200);
	  Buzzer_Off();
	  HAL_Delay(200);
  }
  if (iBus.SwC == 2000) {
	  Buzzer_On(1500);
	  HAL_Delay(200);
	  Buzzer_On(2000);
	  HAL_Delay(200);
	  Buzzer_On(1500);
	  HAL_Delay(200);
	  Buzzer_On(2000);
	  HAL_Delay(200);
	  Buzzer_Off();
	  ESC_Calibration();
	  while (iBus.SwC != 1000) {
		  Is_iBus_Received();
		  Buzzer_On(1500);
		  HAL_Delay(200);
		  Buzzer_On(2000);
		  HAL_Delay(200);
		  Buzzer_Off();
	  }
  }

  /* *********** iBus Calibration Check END ************ */

  /* *********** iBus Throttle Check ************ */

  while (Is_iBus_Throttle_Min() == 0 || iBus.SwA == 2000) {
	  Buzzer_On(343);
	  HAL_Delay(70);
	  Buzzer_Off();
	  HAL_Delay(70);
  }

  /* *********** iBus Throttle Check END ************ */

  /* *********** ESC Startup START ************ */

  __HAL_TIM_DISABLE(&htim5); // Reset timers
  HAL_Delay(10);
  dshot_init(DSHOT600);
  main_led(0, 255, 0, 0, 1);

  // Arming sequence: Send 0 for 3 seconds
  for (uint32_t i = 0; i < 3000; i++) {
	  dshot_write(my_motor_value);
	  HAL_Delay(1);
  }

  my_motor_value[0] = 0; // Motor 1
  my_motor_value[1] = 0; // Motor 2
  my_motor_value[2] = 0; // Motor 3
  my_motor_value[3] = 0; // Motor 4

  main_led(0, 100, 0, 155, 1);

  /* *********** ESC Startup END ************ */

  LL_TIM_EnableCounter(TIM7);
  LL_TIM_EnableIT_UPDATE(TIM7);

  if(ICM42688P_Initialization() == 0)
  {
	  printf("=== Sensor Ready ===\n\n");

	  //	  ===================  Gyro Offset Calibration  ==================

	  //  int result = ICM42688P_CalibrateGyroRawOffsets(2000, gyro_offsets_lsb);
	  //  if (result == 0) {
	  //	  printf("Calibration complete: Gx=%d, Gy=%d, Gz=%d LSB\n",
	  //			  gyro_offsets_lsb[0], gyro_offsets_lsb[1], gyro_offsets_lsb[2]);
	  //  }

	  //	  =================  Gyro Offset Calibration END  =================

  }



  HMC5883L_Init();
  uint8_t hmc_id = HMC5883L_ReadReg(HMC5883L_ID_A);
  printf("HMC5883L ID: %c\n", hmc_id);
  printf("\n");

//  ICM42688P_WriteByte(0x13, (gyro_x_offset*-2)>>8);
//  ICM42688P_WriteByte(0x14, (gyro_x_offset*-2));
//
//  ICM42688P_WriteByte(0x15, (gyro_y_offset*-2)>>8);
//  ICM42688P_WriteByte(0x16, (gyro_y_offset*-2));
//
//  ICM42688P_WriteByte(0x17, (gyro_z_offset*-2)>>8);
//  ICM42688P_WriteByte(0x18, (gyro_z_offset*-2));

  main_led(0, 0, 0, 255, 1);
  HAL_Delay(500);
  main_led(0, 0, 0, 255, 0);
  HAL_Delay(500);
  main_led(0, 0, 0, 255, 1);
  HAL_Delay(500);
  main_led(0, 0, 0, 255, 0);
  HAL_Delay(500);
  main_led(0, 0, 255, 0, 1);
  HAL_Delay(500);
  main_led(0, 0, 255, 0, 0);
  HAL_Delay(500);

  main_led(0, 0, 255, 0, 0.1);


  while (Is_iBus_Throttle_Armed() == 0) {
	  // Debug: Print loop status
	  static uint32_t last_loop_print = 0;
	  if ((HAL_GetTick() - last_loop_print) >= 1000) {
		  last_loop_print = HAL_GetTick();
	  }

	  //	  calibration_task(); // Always call calibration_task
	  dshot_write(my_motor_value);
	  if (is_cmd_mode()) {
		  dshot_write(my_motor_value);
		  continue; // Skip all other tasks in command mode
	  }

	  if (tim7_20ms_flag == 1 && tim7_100ms_flag != 1) {
		  tim7_20ms_flag = 0;
		  check_command_timeout();
	  }

  }

//  /* *********** iBus Throttle Check ************ */


//  while(Is_iBus_Throttle_Min() == 0 || iBus.SwA != 2000)
//  {
//	  Buzzer_On(1200);
//	  HAL_Delay(300);
//	  Buzzer_Off();
//	  HAL_Delay(70);
//  }
//  // Uncomment only when iBus Receiver is connected
//  Buzzer_On(1092);
//  HAL_Delay(100);
//  Buzzer_On(592);
//  HAL_Delay(100);
//  Buzzer_On(292);
//  HAL_Delay(100);
//  Buzzer_Off();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  if(tim7_1ms_flag == 1)
	  {
		  tim7_1ms_flag = 0;

		  ccr1 = 240 + (((iBus.LV - 1000)*10 + (iBus.RV - 1500)*5 + (iBus.RH - 1500)*5 + (iBus.LH - 1500)*5 ) * 0.176); // back left
		  ccr2 = 240 + (((iBus.LV - 1000)*10 - (iBus.RV - 1500)*5 + (iBus.RH - 1500)*5 - (iBus.LH - 1500)*5 ) * 0.176); // front left
		  ccr3 = 240 + (((iBus.LV - 1000)*10 + (iBus.RV - 1500)*5 - (iBus.RH - 1500)*5 - (iBus.LH - 1500)*5 ) * 0.176); // back right
		  ccr4 = 240 + (((iBus.LV - 1000)*10 - (iBus.RV - 1500)*5 - (iBus.RH - 1500)*5 + (iBus.LH - 1500)*5 ) * 0.176); // front right

	  }

	  if(tim7_2ms_flag == 1)
	  {
		  tim7_2ms_flag = 0;

//		  printf("%d, %d, %d\n",
//				  (int)(ICM42688P.gyro_x), (int)(ICM42688P.gyro_y), (int)(ICM42688P.gyro_z));
//		  printf("%d\t%d\t%d\t%d\t\n", my_motor_value[2], my_motor_value[3], my_motor_value[1], my_motor_value[0]);

	  }

	  if(iBus.SwA == 2000 && iBus_SwA_Prev != 2000)
	  {
		  if(iBus.LV < 1010)
		  {
			  motor_arming_flag = 1;
		  }
		  else{
			  while(Is_iBus_Throttle_Min() == 0 || iBus.SwA == 2000)
			  {
				  Buzzer_On(1200);
				  HAL_Delay(300);
				  Buzzer_Off();
				  HAL_Delay(70);
			  }
		  }
	  }

	  iBus_SwA_Prev = iBus.SwA;

	  if(iBus.SwA != 2000)
	  {
		  motor_arming_flag = 0;
	  }

	  if(motor_arming_flag == 1)
	  {
		  if(failsafe_flag == 0)
		  {
			  my_motor_value[2] = ccr1 > 2000 ? 2000 : ccr1 < 240 ? 240 : ccr1;
			  my_motor_value[3] = ccr2 > 2000 ? 2000 : ccr2 < 240 ? 240 : ccr2;
			  my_motor_value[1] = ccr3 > 2000 ? 2000 : ccr3 < 240 ? 240 : ccr3;
			  my_motor_value[0] = ccr4 > 2000 ? 2000 : ccr4 < 240 ? 240 : ccr4;
			  dshot_write(my_motor_value);
//			  printf("%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t\n",
//					  iBus.RH, iBus.RV, iBus.LV, iBus.LH, iBus.SwA, iBus.SwB, iBus.SwC, iBus.SwD, iBus.VrA, iBus.VrB);
		  }
		  else
		  {
			  my_motor_value[2] = 0;
			  my_motor_value[3] = 0;
			  my_motor_value[1] = 0;
			  my_motor_value[0] = 0;
			  dshot_write(my_motor_value);
		  }
	  }
	  else
	  {
		  my_motor_value[2] = 0;
		  my_motor_value[3] = 0;
		  my_motor_value[1] = 0;
		  my_motor_value[0] = 0;
		  dshot_write(my_motor_value);
	  }

	  if(ibus_rx_cplt_flag == 1)
	  {
		  ibus_rx_cplt_flag = 0;
		  if(iBus_Check_CHKSUM(&ibus_rx_buf[0], 32) == 1)
		  {
			  LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_2);

			  iBus_Parsing(&ibus_rx_buf[0], &iBus);
			  iBus_rx_cnt++;

			  if(iBus_isActiveFailsafe(&iBus) == 1)
			  {
				  failsafe_flag = 1;
			  }
			  else
			  {
				  failsafe_flag = 0;
			  }
			  //			  printf("%d\t%d\t%d\t%d\t%d\t%d\n",
			  //					  iBus.RH, iBus.RV, iBus.LV, iBus.LH, iBus.SwA, iBus.SwC);
			  //			  HAL_Delay(100);
		  }
	  }

	  if(tim7_1000ms_flag == 1)
	  {
		  tim7_1000ms_flag = 0;

		  if(iBus_rx_cnt == 0)
		  {
			  failsafe_flag = 2;
		  }
		  iBus_rx_cnt = 0;
	  }

	  if(ICM42688P_DataReady() == 1)
	  {

		  HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_3);

		  ICM42688P_Get6AxisRawData(&ICM42688P.acc_x_raw, &ICM42688P.gyro_x_raw);

		  // Apply gyro offsets in software (subtract offsets from raw data)
		  int16_t gyro_x_corrected = ICM42688P.gyro_x_raw + gyro_offsets_lsb[0];	//  8
		  int16_t gyro_y_corrected = ICM42688P.gyro_y_raw + gyro_offsets_lsb[1];	//  4
		  int16_t gyro_z_corrected = ICM42688P.gyro_z_raw + gyro_offsets_lsb[2];	// -2

		  ICM42688P.gyro_x = gyro_x_corrected * 2000.f / 32768.f;
		  ICM42688P.gyro_y = gyro_y_corrected * 2000.f / 32768.f;
		  ICM42688P.gyro_z = gyro_z_corrected * 2000.f / 32768.f;

		  ICM42688P.acc_x = ICM42688P.acc_x_raw * 0.0004883f;
		  ICM42688P.acc_y = ICM42688P.acc_y_raw * 0.0004883f;
		  ICM42688P.acc_z = ICM42688P.acc_z_raw * 0.0004883f;

		  //		  printf("%d, %d, %d, %d, %d, %d\n",
		  //				  (int)(ICM42688P.gyro_x*100), (int)(ICM42688P.gyro_y*100), (int)(ICM42688P.gyro_z*100),
		  //				  (int)(ICM42688P.acc_x), (int)(ICM42688P.acc_x), (int)(ICM42688P.acc_x));

//		  printf("%d, %d, %d\n",
//				  (int)(ICM42688P.gyro_x), (int)(ICM42688P.gyro_y), (int)(ICM42688P.gyro_z));

		  //		  printf("%d, %d, %d\n", gyro_x_corrected, gyro_y_corrected, gyro_z_corrected);

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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 44;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

int Is_iBus_Throttle_Min(void){
	if(ibus_rx_cplt_flag == 1){
		ibus_rx_cplt_flag = 0;
		if(iBus_Check_CHKSUM(&ibus_rx_buf[0], 32) == 1)
		{
			iBus_Parsing(&ibus_rx_buf[0], &iBus);
			if(iBus.LV < 1010) return 1;
		}
	}
	return 0;
}

int Is_iBus_Throttle_Armed(void){
	if(ibus_rx_cplt_flag == 1){
		ibus_rx_cplt_flag = 0;
		if(iBus_Check_CHKSUM(&ibus_rx_buf[0], 32) == 1){
			iBus_Parsing(&ibus_rx_buf[0], &iBus);
			if(iBus.SwA >= 1900)  // HIGH = armed
				return 1;
		}
	}
	return 0;
}

void ESC_Calibration(void){

	   TIM5->CCR1 = 2000;
	   TIM5->CCR2 = 2000;
	   TIM5->CCR3 = 2000;
	   TIM5->CCR4 = 2000;

	   main_led(0, 255, 0, 0, 0.1);

	   HAL_Delay(7000);

	   TIM5->CCR1 = 1000;
	   TIM5->CCR2 = 1000;
	   TIM5->CCR3 = 1000;
	   TIM5->CCR4 = 1000;

	   main_led(0, 0, 0, 255, 0.1);

	   HAL_Delay(8000);

	   main_led(0, 0, 0, 255, 0);
}

int Is_iBus_Received(void){
	if(ibus_rx_cplt_flag == 1){
		ibus_rx_cplt_flag = 0;
		if(iBus_Check_CHKSUM(&ibus_rx_buf[0], 32) == 1)
		{
			iBus_Parsing(&ibus_rx_buf[0], &iBus);
			return 1;
		}
	}
	return 0;
}

void check_command_timeout(void) {
	if (cmd_receiving && !is_cmd_mode()) { // Only in normal mode
		if ((HAL_GetTick() - last_char_time) > CMD_TIMEOUT_MS) {
			cmd_receiving = 0;
			cmd_index = 0;
			printf("Main: Command timeout at %lu ms\n", HAL_GetTick());
			WS2812_Update();
		}
	}
}
/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

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
