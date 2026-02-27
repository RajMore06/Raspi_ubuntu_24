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
#include "string.h"
#include "stdlib.h"
#include "stdio.h"
#include "stdbool.h"
#include "math.h"
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
CAN_HandleTypeDef hcan;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8];
uint8_t RxData[8];
uint32_t TxMailbox;
HAL_StatusTypeDef i2cstatus;
uint16_t canhead;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

#define BNO055_CHIP_ID          0x00
#define BNO055_ACC_ID           0x01
#define BNO055_MAG_ID           0x02
#define BNO055_GYRO_ID          0x03
#define BNO055_SW_REV_ID_LSB    0x04
#define BNO055_SW_REV_ID_MSB    0x05
#define BNO055_BL_REV_ID        0x06
#define BNO055_PAGE_ID          0x07
#define BNO055_ADDRESS          0x28

#define BNO055_PAGE_ID          0x07
#define BNO055_ACC_DATA_X_LSB   0x08
#define BNO055_ACC_DATA_X_MSB   0x09
#define BNO055_ACC_DATA_Y_LSB   0x0A
#define BNO055_ACC_DATA_Y_MSB   0x0B
#define BNO055_ACC_DATA_Z_LSB   0x0C
#define BNO055_ACC_DATA_Z_MSB   0x0D
#define BNO055_MAG_DATA_X_LSB   0x0E
#define BNO055_MAG_DATA_X_MSB   0x0F
#define BNO055_MAG_DATA_Y_LSB   0x10
#define BNO055_MAG_DATA_Y_MSB   0x11
#define BNO055_MAG_DATA_Z_LSB   0x12
#define BNO055_MAG_DATA_Z_MSB   0x13
#define BNO055_GYR_DATA_X_LSB   0x14
#define BNO055_GYR_DATA_X_MSB   0x15
#define BNO055_GYR_DATA_Y_LSB   0x16
#define BNO055_GYR_DATA_Y_MSB   0x17
#define BNO055_GYR_DATA_Z_LSB   0x18
#define BNO055_GYR_DATA_Z_MSB   0x19
#define BNO055_EUL_HEADING_LSB  0x1A
#define BNO055_EUL_HEADING_MSB  0x1B
#define BNO055_EUL_ROLL_LSB     0x1C
#define BNO055_EUL_ROLL_MSB     0x1D
#define BNO055_EUL_PITCH_LSB    0x1E
#define BNO055_EUL_PITCH_MSB    0x1F
#define BNO055_QUA_DATA_W_LSB   0x20
#define BNO055_QUA_DATA_W_MSB   0x21
#define BNO055_QUA_DATA_X_LSB   0x22
#define BNO055_QUA_DATA_X_MSB   0x23
#define BNO055_QUA_DATA_Y_LSB   0x24
#define BNO055_QUA_DATA_Y_MSB   0x25
#define BNO055_QUA_DATA_Z_LSB   0x26
#define BNO055_QUA_DATA_Z_MSB   0x27
#define BNO055_LIA_DATA_X_LSB   0x28
#define BNO055_LIA_DATA_X_MSB   0x29
#define BNO055_LIA_DATA_Y_LSB   0x2A
#define BNO055_LIA_DATA_Y_MSB   0x2B
#define BNO055_LIA_DATA_Z_LSB   0x2C
#define BNO055_LIA_DATA_Z_MSB   0x2D
#define BNO055_GRV_DATA_X_LSB   0x2E
#define BNO055_GRV_DATA_X_MSB   0x2F
#define BNO055_GRV_DATA_Y_LSB   0x30
#define BNO055_GRV_DATA_Y_MSB   0x31
#define BNO055_GRV_DATA_Z_LSB   0x32
#define BNO055_GRV_DATA_Z_MSB   0x33
#define BNO055_TEMP             0x34
#define BNO055_CALIB_STAT       0x35
#define BNO055_ST_RESULT        0x36
#define BNO055_INT_STATUS       0x37
#define BNO055_SYS_CLK_STATUS   0x38
#define BNO055_SYS_STATUS       0x39
#define BNO055_SYS_ERR          0x3A
#define BNO055_UNIT_SEL         0x3B
#define BNO055_OPR_MODE         0x3D
#define BNO055_PWR_MODE         0x3E
#define BNO055_SYS_TRIGGER      0x3F
#define BNO055_TEMP_SOURCE      0x40
#define BNO055_AXIS_MAP_CONFIG  0x41
#define BNO055_AXIS_MAP_SIGN    0x42
#define BNO055_ACC_OFFSET_X_LSB 0x55
#define BNO055_ACC_OFFSET_X_MSB 0x56
#define BNO055_ACC_OFFSET_Y_LSB 0x57
#define BNO055_ACC_OFFSET_Y_MSB 0x58
#define BNO055_ACC_OFFSET_Z_LSB 0x59
#define BNO055_ACC_OFFSET_Z_MSB 0x5A
#define BNO055_MAG_OFFSET_X_LSB 0x5B
#define BNO055_MAG_OFFSET_X_MSB 0x5C
#define BNO055_MAG_OFFSET_Y_LSB 0x5D
#define BNO055_MAG_OFFSET_Y_MSB 0x5E
#define BNO055_MAG_OFFSET_Z_LSB 0x5F
#define BNO055_MAG_OFFSET_Z_MSB 0x60
#define BNO055_GYR_OFFSET_X_LSB 0x61
#define BNO055_GYR_OFFSET_X_MSB 0x62
#define BNO055_GYR_OFFSET_Y_LSB 0x63
#define BNO055_GYR_OFFSET_Y_MSB 0x64
#define BNO055_GYR_OFFSET_Z_LSB 0x65
#define BNO055_GYR_OFFSET_Z_MSB 0x66
#define BNO055_ACC_RADIUS_LSB   0x67
#define BNO055_ACC_RADIUS_MSB   0x68
#define BNO055_MAG_RADIUS_LSB   0x69
#define BNO055_MAG_RADIUS_MSB   0x6A

// BNO055 Page 1
#define BNO055_PAGE_ID          0x07
#define BNO055_ACC_CONFIG       0x08
#define BNO055_MAG_CONFIG       0x09
#define BNO055_GYRO_CONFIG_0    0x0A
#define BNO055_GYRO_CONFIG_1    0x0B
#define BNO055_ACC_SLEEP_CONFIG 0x0C
#define BNO055_GYR_SLEEP_CONFIG 0x0D
#define BNO055_INT_MSK          0x0F
#define BNO055_INT_EN           0x10
#define BNO055_ACC_AM_THRES     0x11
#define BNO055_ACC_INT_SETTINGS 0x12
#define BNO055_ACC_HG_DURATION  0x13
#define BNO055_ACC_HG_THRESH    0x14
#define BNO055_ACC_NM_THRESH    0x15
#define BNO055_ACC_NM_SET       0x16
#define BNO055_GYR_INT_SETTINGS 0x17
#define BNO055_GYR_HR_X_SET     0x18
#define BNO055_GYR_DUR_X        0x19
#define BNO055_GYR_HR_Y_SET     0x1A
#define BNO055_GYR_DUR_Y        0x1B
#define BNO055_GYR_HR_Z_SET     0x1C
#define BNO055_GYR_DUR_Z        0x1D
#define BNO055_GYR_AM_THRESH    0x1E
#define BNO055_GYR_AM_SET       0x1F

typedef enum {  // BNO-55 operation modes
	BNO055_OPERATION_MODE_CONFIG = 0x00,
	// Sensor Mode
	BNO055_OPERATION_MODE_ACCONLY = 0x01,
	BNO055_OPERATION_MODE_MAGONLY = 0x02,
	BNO055_OPERATION_MODE_GYRONLY = 0x03,
	BNO055_OPERATION_MODE_ACCMAG = 0x04,
	BNO055_OPERATION_MODE_ACCGYRO = 0x05,
	BNO055_OPERATION_MODE_MAGGYRO = 0x06,
	BNO055_OPERATION_MODE_AMG = 0x07,  // 0x07
	// Fusion Mode
	BNO055_OPERATION_MODE_IMU = 0x08,
	BNO055_OPERATION_MODE_COMPASS = 0x09,
	BNO055_OPERATION_MODE_M4G = 0x0A,
	BNO055_OPERATION_MODE_NDOF_FMC_OFF = 0x0B,
	BNO055_OPERATION_MODE_NDOF = 0x0C  // 0x0C
} OPRMode_t;

typedef enum {
	NormalPwr = 0x00, LowPwr = 0x01, SuspendPwr = 0x02
} PWRMode_t;

/** Remap settings **/
typedef enum {
	REMAP_CONFIG_P0 = 0x21, REMAP_CONFIG_P1 = 0x24, // default
	REMAP_CONFIG_P2 = 0x24,
	REMAP_CONFIG_P3 = 0x21,
	REMAP_CONFIG_P4 = 0x24,
	REMAP_CONFIG_P5 = 0x21,
	REMAP_CONFIG_P6 = 0x21,
	REMAP_CONFIG_P7 = 0x24
} axis_remap_config_t;

/** Remap Signs **/
typedef enum {
	REMAP_SIGN_P0 = 0x04, REMAP_SIGN_P1 = 0x00, // default
	REMAP_SIGN_P2 = 0x06,
	REMAP_SIGN_P3 = 0x02,
	REMAP_SIGN_P4 = 0x03,
	REMAP_SIGN_P5 = 0x01,
	REMAP_SIGN_P6 = 0x07,
	REMAP_SIGN_P7 = 0x05
} axis_remap_sign_t;

enum Accg_range {
	R_2G = 0, R_4G, R_8G, R_16G
};

enum Acc_BandWidth {
	B_7hz = 0, B_15hz, B_31hz, B_62hz, B_125hz, B_250hz, B_500hz, B_1000hz
};

enum Acc_opr {
	acc_normal = 0,
	acc_suspend,
	acc_lowpower1,
	acc_standby,
	acc_lowpoer2,
	acc_deepsuspend
};

enum Gyro_range {
	R_2000, R_1000, R_500, R_250, R_125
};

enum Gyro_BandWidth {
	B_523hz, B_230hz, B_116hz, B_47hz, B_23hz, B_12hz, B_64hz, B_32hz
};

enum Gyro_opr {
	Gyro_normal,
	Gyro_FastPower,
	Gyro_DeepSuspend,
	Gyro_Suspend,
	Gyro_AdvancedPowerSaved,
};

enum DataOutputRate {
	MB_2hz, MB_6hz, MB_8hz, MB_10hz, MB_15hz, MB_20hz, MB_25hz, MB_30hz
};

enum Mag_opr {
	Mag_LowPwr, Mag_Regular, Mag_EnhancedRegular, Mag_HighAccuracy
};

enum Mag_pwrMode {
	Mag_Normal, Mag_Sleep, Mag_Suspend, Mag_ForceMode
};

void BNO_Scan();
void BNO_POST();
void BNO_Calib();
void BNO_init();
void SetMode(OPRMode_t mode);
void GetMode();
void SetPowerMode(PWRMode_t mode_p);
void i2c_writeData(uint8_t regaddr, uint8_t txsubaddr, uint8_t data);
void i2c_readData(uint8_t addr, uint8_t subaddr, uint8_t *buf, uint8_t buf_size);
void readAccelData(int16_t *destination);
void readGyroData(int16_t *destination);
void readQuatData(int16_t *destination);
void readLIAData(int16_t *destination);
void readGRVData(int16_t *destination);
void readMagData(int16_t *destination);
void readEulData(int16_t *destination);
void BNO_SetAccOffset(double *dest_a);
void BNO_SetGyroOffset(float *dest_a);
void BNO_SetMagOffset(float *dest_a);
void BNO_setPage(uint8_t page);
void SerialWrite(char *uart2Data);
void can_transmitSingleByte(uint8_t Txid, uint8_t Txdata);
void canInit_recieveMultiID(void);
uint8_t sysg = 0x00;
uint8_t def = 0;
static float headingOffset = 0.0f;
static bool headingZeroed = false;
char print[8] = { '\0' };
char bgpr[128] = { '\0' };
int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output
int16_t quatCount[4];   // Stores the 16-bit signed quaternion output
int16_t EulCount[3];    // Stores the 16-bit signed Euler angle output
int16_t LIACount[3];    // Stores the 16-bit signed linear acceleration output
int16_t GRVCount[3];    // Stores the 16-bit signed gravity vector output
int a;
float head_h;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	//MX_CAN_Init();
	MX_I2C1_Init();
	MX_USART2_UART_Init();
	/* USER CODE BEGIN 2 */
	//canInit_recieveMultiID();
//	if (HAL_CAN_Start(&hcan) != HAL_OK) {
//		Error_Handler();
//	}

	//TxHeader.StdId = 0x691;
	//TxHeader.IDE = CAN_ID_STD;
	//TxHeader.DLC = 8;
	//TxHeader.RTR = CAN_RTR_DATA;

	HAL_Delay(1000);
	BNO_POST();
	uint8_t comm = 0x00 | 0x20;
	i2cstatus = HAL_I2C_Mem_Write(&hi2c1, BNO055_ADDRESS << 1,
	BNO055_SYS_TRIGGER, I2C_MEMADD_SIZE_8BIT, &comm, 1, 1000);
	if (i2cstatus != HAL_OK) {
		SerialWrite("fail! ");
	}
	HAL_Delay(800);
	BNO_setPage(0);
	i2c_writeData(BNO055_ADDRESS, BNO055_UNIT_SEL, 0x01);
	SetMode(BNO055_OPERATION_MODE_CONFIG);
	SetPowerMode(NormalPwr);
	SetMode(BNO055_OPERATION_MODE_NDOF);
	GetMode();
	HAL_Delay(25);
	BNO_Calib();
//	float head_h;
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
//		if (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0) > 0) {
//			if (HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &RxHeader, RxData)
//					!= HAL_OK) {
//				Error_Handler();
//			}
//			if (RxHeader.StdId == 0x01 && RxData[0] == 1) {
////				i2c_writeData(BNO055_ADDRESS, BNO055_SYS_TRIGGER, 0x20);
////				RxData[0] = 0;
////				BNO_POST();
//				uint8_t comm = 0x00 | 0x20;
//				i2cstatus = HAL_I2C_Mem_Write(&hi2c1, BNO055_ADDRESS << 1,
//				BNO055_SYS_TRIGGER, I2C_MEMADD_SIZE_8BIT, &comm, 1, 1000);
//				if (i2cstatus != HAL_OK) {
//					SerialWrite("fail! ");
//				}
//				HAL_Delay(700);
//				BNO_setPage(0);
//				i2c_writeData(BNO055_ADDRESS, BNO055_UNIT_SEL, 0x01);
//				SetMode(BNO055_OPERATION_MODE_CONFIG);
//				SetPowerMode(NormalPwr);
//				SetMode(BNO055_OPERATION_MODE_NDOF);
//				GetMode();
//				HAL_Delay(25);
//				BNO_Calib();
//				a++;
//			}
//		}
		//	a=0;
		readQuatData(quatCount);
		float q0,q1,q2,q3;
		q0 = (float)(quatCount[0]/16384.0);
		q1 = (float)(quatCount[1]/16384.0);
		q2 = (float)(quatCount[2]/16384.0);
		q3 = (float)(quatCount[3]/16384.0);

		float yaw = -atan2f(2.0f*(q0*q3+q1*q2),1.0f - 2.0f *(q2*q2+q3*q3));
		float yaw_deg = yaw * (180.0f / (float) M_PI);

		if (!headingZeroed) {
		    headingOffset = yaw_deg;
		    headingZeroed = true;
		}

		float zeroedHeading = yaw_deg - headingOffset;
		if (zeroedHeading < 0.0f) zeroedHeading += 360.0f;
		if (zeroedHeading >= 360.0f) zeroedHeading -= 360.0f;

//		readMagData(magCount);
//		float mx,my,mz;
//		mx = (float)(magCount[0]/16.0);
//		my = (float)(magCount[1]/16.0);
//		mz = (float)(magCount[2]/16.0);
//		//head_h = (float) EulCount[0] / 16;
//		//canhead = (uint16_t) (head_h * 100);
//		sprintf(bgpr,"Mx: %f, My: %f, Mz: %f\r\n", mx, my, mz);
//		SerialWrite(bgpr);

//		readEulData(EulCount);
//		head_h = (float) EulCount[0] / 16;
		//canhead = (uint16_t) (head_h * 100);
		sprintf(bgpr,"%.2f\r\n",zeroedHeading);
		SerialWrite(bgpr);

		//TxData[0] = (uint8_t) canhead;
		//TxData[1] = (uint8_t) (canhead >> 8);
		//HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);

	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
	PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief CAN Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN_Init(void) {

	/* USER CODE BEGIN CAN_Init 0 */

	/* USER CODE END CAN_Init 0 */

	/* USER CODE BEGIN CAN_Init 1 */

	/* USER CODE END CAN_Init 1 */
	hcan.Instance = CAN;
	hcan.Init.Prescaler = 8;
	hcan.Init.Mode = CAN_MODE_NORMAL;
	hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan.Init.TimeSeg1 = CAN_BS1_7TQ;
	hcan.Init.TimeSeg2 = CAN_BS2_8TQ;
	hcan.Init.TimeTriggeredMode = DISABLE;
	hcan.Init.AutoBusOff = DISABLE;
	hcan.Init.AutoWakeUp = DISABLE;
	hcan.Init.AutoRetransmission = DISABLE;
	hcan.Init.ReceiveFifoLocked = DISABLE;
	hcan.Init.TransmitFifoPriority = DISABLE;
	if (HAL_CAN_Init(&hcan) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN CAN_Init 2 */

	/* USER CODE END CAN_Init 2 */

}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.Timing = 0x2000090E;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE)
			!= HAL_OK) {
		Error_Handler();
	}

	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
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
static void MX_USART2_UART_Init(void) {

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
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
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
static void MX_GPIO_Init(void) {
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

void canInit_recieveMultiID(void) {
	RxHeader.IDE = CAN_ID_STD;
	RxHeader.DLC = 8;
	//rxHeader.StdId = ;
	RxHeader.RTR = CAN_RTR_DATA;

	CAN_FilterTypeDef canfilterconfig;

	canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
	canfilterconfig.FilterBank = 1;
	canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
	canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
	canfilterconfig.FilterIdHigh = 0x1fff;
	;
	canfilterconfig.FilterIdLow = 0xffff;
	canfilterconfig.FilterMaskIdHigh = 0;
	canfilterconfig.FilterMaskIdLow = 0;

	if (HAL_CAN_ConfigFilter(&hcan, &canfilterconfig) != HAL_OK) {
		Error_Handler();
	}

	if (HAL_CAN_Start(&hcan) != HAL_OK) {
		Error_Handler();
	}
}

void SerialWrite(char *uart2Data) {
	HAL_StatusTypeDef uart_status;
	uart_status = HAL_UART_Transmit(&huart2, (uint8_t*) uart2Data,
			strlen(uart2Data), 5);
	if (uart_status != HAL_OK) {
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
		HAL_Delay(1000);
	}

}

void BNO_reset() {

	HAL_Delay(2);
	i2c_writeData(BNO055_ADDRESS, BNO055_SYS_TRIGGER, 0x2);
	HAL_Delay(30);
	i2c_writeData(BNO055_ADDRESS, BNO055_SYS_TRIGGER, 0x00);
}

void BNO_Calib(void) {

	i2c_readData(BNO055_ADDRESS, BNO055_CALIB_STAT, &def, 2);
	if (def != 0x3f) {
		SerialWrite("NOT CALIBRATED \r\n");
		sprintf(print, "%x\r\n", def);
		SerialWrite(print);
	} else {

		SerialWrite("---CALIBRATED- \r\n");

	}

}

void readAccData(int16_t *destination) {
	uint8_t rawData[6];  // x/y/z gyro register data stored here
	i2c_readData(BNO055_ADDRESS, BNO055_ACC_DATA_X_LSB, &rawData[0], 6); // Read the six raw data registers sequentially into data array
	destination[0] = ((int16_t) rawData[1] << 8) | rawData[0]; // Turn the MSB and LSB into a signed 16-bit value
	destination[1] = ((int16_t) rawData[3] << 8) | rawData[2];
	destination[2] = ((int16_t) rawData[5] << 8) | rawData[4];
}

void readGyroData(int16_t *destination) {
	uint8_t rawData[6];  // x/y/z gyro register data stored here
	i2c_readData(BNO055_ADDRESS, BNO055_GYR_DATA_X_LSB, &rawData[0], 6); // Read the six raw data registers sequentially into data array
	destination[0] = ((int16_t) rawData[1] << 8) | rawData[0]; // Turn the MSB and LSB into a signed 16-bit value
	destination[1] = ((int16_t) rawData[3] << 8) | rawData[2];
	destination[2] = ((int16_t) rawData[5] << 8) | rawData[4];
}

void readMagData(int16_t *destination) {
	uint8_t rawData[6];  // x/y/z gyro register data stored here
	i2c_readData(BNO055_ADDRESS, BNO055_MAG_DATA_X_LSB, &rawData[0], 6); // Read the six raw data registers sequentially into data array
	destination[0] = ((int16_t) rawData[1] << 8) | rawData[0]; // Turn the MSB and LSB into a signed 16-bit value
	destination[1] = ((int16_t) rawData[3] << 8) | rawData[2];
	destination[2] = ((int16_t) rawData[5] << 8) | rawData[4];
}

void readEulData(int16_t *destination) {
	uint8_t rawData[6];  // x/y/z gyro register data stored here
	i2c_readData(BNO055_ADDRESS, BNO055_EUL_HEADING_LSB, &rawData[0], 6); // Read the six raw data registers sequentially into data array
	destination[0] = ((int16_t) rawData[1] << 8) | rawData[0]; // Turn the MSB and LSB into a signed 16-bit value
	destination[1] = ((int16_t) rawData[3] << 8) | rawData[2];
	destination[2] = ((int16_t) rawData[5] << 8) | rawData[4];
}

void readQuatData(int16_t *destination) {
	uint8_t rawData[8];
	i2c_readData(BNO055_ADDRESS, BNO055_QUA_DATA_W_LSB, &rawData[0], 8);
	destination[0] = ((int16_t) rawData[1] << 8) | rawData[0];
	destination[1] = ((int16_t) rawData[3] << 8) | rawData[2];
	destination[2] = ((int16_t) rawData[5] << 8) | rawData[4];
	destination[3] = ((int16_t) rawData[7] << 8) | rawData[6];
}

void BNO_Scan(void) {
	uint8_t Buffer[16] = { '\0' };
	HAL_StatusTypeDef i2c_status;
	for (uint8_t i = 0; i < 128; i += 1) {
		i2c_status = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t) (i << 1), 1, 10);
		if (i2c_status == HAL_OK) {
			sprintf(Buffer, "Device ID :0x%X \r\n", i);
			SerialWrite(Buffer);
			if (i == 40) {
				SerialWrite("BNO_DETECTED\r\n");
			}
			SerialWrite("\n");
		} else if (i2c_status != HAL_OK) {
			SerialWrite("******************\n");
		}
		HAL_Delay(10);
	}
	SerialWrite("Scanning Done \n");
}

void i2c_readData(uint8_t addr, uint8_t subaddr, uint8_t *buf, uint8_t buf_size) {
	// i2c_status = HAL_I2C_Master_Transmit(&hi2c1,addr << 1, &subaddr, 1, 10);
	uint8_t v = 0;
	if (HAL_I2C_Master_Transmit(&hi2c1, addr << 1, &subaddr, 1, HAL_MAX_DELAY)
			== HAL_OK) {
		v += 1;
	} else {
		SerialWrite("RX --- I2C --- ERROR \r\n");

	}
	//HAL_Delay(5);
	if (HAL_I2C_Master_Receive(&hi2c1, addr << 1, buf, buf_size, HAL_MAX_DELAY)
			== HAL_OK) {
		v += 1;
	}

	else {
		SerialWrite("RX --- I2C --- ERROR \r\n");

	}

}

void i2c_writeData(uint8_t regaddr, uint8_t txsubaddr, uint8_t data) {
	uint8_t readVal;
	uint8_t tx_i2c[2] = { txsubaddr, data };
	if (HAL_I2C_Master_Transmit(&hi2c1, regaddr << 1, tx_i2c, sizeof(tx_i2c),
	HAL_MAX_DELAY) == HAL_OK) {
		i2c_readData(regaddr, txsubaddr, &readVal, 1);
	} else if (HAL_I2C_Master_Transmit(&hi2c1, regaddr << 1, tx_i2c,
			sizeof(tx_i2c), HAL_MAX_DELAY) != HAL_OK) {
		SerialWrite("TX --- I2C --- ERROR \r\n");
	}
}

void BNO_POST() {
	int8_t k = 0;
	i2c_readData(BNO055_ADDRESS, BNO055_CHIP_ID, &def, 2);
	if (def == 160) {
		k += 1;
	}
	i2c_readData(BNO055_ADDRESS, BNO055_ACC_ID, &def, 2);
	if (def == 251) {
		k += 1;
	}
	i2c_readData(BNO055_ADDRESS, BNO055_GYRO_ID, &def, 2);
	if (def == 15) {
		k += 1;
	}
	i2c_readData(BNO055_ADDRESS, BNO055_MAG_ID, &def, 2);
	if (def == 50) {
		k += 1;
	}
	if (k == 4) {
		SerialWrite("CHIP DATA \r\n");

		HAL_Delay(1);
		uint8_t lsb = 0, msb = 0;
		i2c_readData(BNO055_ADDRESS, BNO055_SW_REV_ID_LSB, &def, 2);
		lsb = def;
		i2c_readData(BNO055_ADDRESS, BNO055_SW_REV_ID_MSB, &def, 2);
		msb = def;
		i2c_readData(BNO055_ADDRESS, BNO055_BL_REV_ID, &def, 2);
		HAL_Delay(25);
		i2c_readData(BNO055_ADDRESS, BNO055_ST_RESULT, &def, 4);

	}

}

void SetMode(OPRMode_t mode) {
	i2c_writeData(BNO055_ADDRESS, BNO055_OPR_MODE, mode);
	HAL_Delay(30);
}
void GetMode() {

	i2c_readData(BNO055_ADDRESS, BNO055_OPR_MODE, &def, 2);
	sprintf(print, "%d \n", def);
	SerialWrite(print);
}

void BNO_init() {
	sysg |= 0x20;
	SetMode(BNO055_OPERATION_MODE_CONFIG);
	//i2c_writeData(BNO055_ADDRESS, BNO055_OPR_MODE,0x00);
	GetMode();
	//SetPowerMode(LowPwr);
	HAL_Delay(25);
	// Select BNO055 gyro temperature source
	i2c_writeData(BNO055_ADDRESS, BNO055_PAGE_ID, 0x01);
	// Configure ACC
	i2c_writeData(BNO055_ADDRESS, BNO055_ACC_CONFIG,
			acc_normal << 5 | B_31hz << 2 | R_4G);
	//i2c_writeData(BNO055_ADDRESS, BNO055_ACC_CONFIG, acc_normal | B_31hz  |R_4G);
	// Configure GYR
	i2c_writeData(BNO055_ADDRESS, BNO055_GYRO_CONFIG_0, B_23hz << 3 | R_2000);
	//i2c_writeData(BNO055_ADDRESS, BNO055_GYRO_CONFIG_0, B_23hz | R_250);
	i2c_writeData(BNO055_ADDRESS, BNO055_GYRO_CONFIG_1, Gyro_normal);
	// Configure MAG
	i2c_writeData(BNO055_ADDRESS, BNO055_MAG_CONFIG,
			Mag_Normal << 5 | Mag_Regular << 3 | MB_30hz);
	//i2c_writeData(BNO055_ADDRESS, BNO055_MAG_CONFIG, Mag_Normal | Mag_Regular | MB_10hz );
	i2c_writeData(BNO055_ADDRESS, BNO055_PAGE_ID, 0x00);
	i2c_writeData(BNO055_ADDRESS, BNO055_TEMP_SOURCE, 0x01);
	i2c_writeData(BNO055_ADDRESS, BNO055_UNIT_SEL, 0x01);
	i2c_writeData(BNO055_ADDRESS, BNO055_AXIS_MAP_CONFIG, REMAP_CONFIG_P0);
	i2c_writeData(BNO055_ADDRESS, BNO055_AXIS_MAP_SIGN, REMAP_SIGN_P3);
	SetPowerMode(NormalPwr);
	SetMode(BNO055_OPERATION_MODE_NDOF);
	HAL_Delay(25);

	i2c_writeData(BNO055_ADDRESS, BNO055_SYS_TRIGGER, &sysg);
	HAL_Delay(30);
	BNO_Calib();
	i2c_writeData(BNO055_ADDRESS, BNO055_SYS_TRIGGER, 0x00);
	GetMode();

}

void SetPowerMode(PWRMode_t mode_p) {
	uint8_t vref = 0;
	i2c_writeData(BNO055_ADDRESS, BNO055_PWR_MODE, mode_p);
	i2c_readData(BNO055_ADDRESS, BNO055_PWR_MODE, &vref, 2);
	if (vref == 0) {
		SerialWrite("PWR MODE \n");
	} else if (vref == 1) {
		SerialWrite("LOW PWR MODE \n");
	}
}

void BNO_setPage(uint8_t page) {
	uint8_t vef = 0;
	i2c_writeData(BNO055_ADDRESS, BNO055_PAGE_ID, page);
	i2c_readData(BNO055_ADDRESS, BNO055_PWR_MODE, &vef, 2);

	if (vef == 0) {
		SerialWrite("PAGE 0 \n");
	} else if (vef == 1) {
		SerialWrite("PAGE 1 \n");
	}
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
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
