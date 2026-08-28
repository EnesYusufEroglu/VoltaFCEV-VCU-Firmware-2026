/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 *
 * ██╗   ██╗ ██████╗ ██╗  ████████╗ █████╗
 * ██║   ██║██╔═══██╗██║  ╚══██╔══╝██╔══██╗
 * ██║   ██║██║   ██║██║     ██║   ███████║
 * ╚██╗ ██╔╝██║   ██║██║     ██║   ██╔══██║
 *  ╚████╔╝ ╚██████╔╝███████╗██║   ██║  ██║
 *   ╚═══╝   ╚═════╝ ╚══════╝╚═╝   ╚═╝  ╚═╝
 *
 ******************************************************************************
 * @file           : main.c
 * @project        : VoltaFCEV Vehicle Control Unit Firmware
 * @version        : 2.4.0 (FreeRTOS integrated)
 * @author         : VCU TEAM
 * @date           : 28-08-2026
 ******************************************************************************
 * @note
 * This firmware handles CAN communication with BMS, motor controller
 * and steering module. Telemetry data is transmitted via LoRa (433 MHz).
 * Display output is managed through DWIN serial protocol.
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include <stdbool.h>
#include "ctype.h"
#include "math.h"
#include "stdlib.h"
#include "LoRa.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "receiveDriver_def.h"
#include "transmitDriver_def.h"
#include "receiveButtons_def.h"
#include "receiveH2_def.h"
#include "transmitH2_def.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// THINKING TTS2A103F39H1RA0 NTC'sinin fiziksel parametreleri
#define R0			10000.0f	// NTC'nin 25°C'deki nominal direnci
#define T0			298.15f		// 25°C'nin Kelvin karşılığı
#define BETA		3975.0f		// Datasheet'ten alınan BETA katsayısı
#define R_PULLDOWN	10000.0f	// Karttaki R4-R11 seri pull-down dirençleri (10k)
#define VCC         3.3f

#define DRIVER_RX_BUFFER_SIZE 80
#define BUTTONS_RX_BUFFER_SIZE 32
#define H2_RX_BUFFER_SIZE 64

#define ADC2_BUFFER_SIZE 1

#define MIN_PAGE 0
#define MAX_PAGE 1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc2;

CAN_HandleTypeDef hcan1;

DAC_HandleTypeDef hdac;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

/* Definitions for TelemetryTask */
osThreadId_t TelemetryTaskHandle;
const osThreadAttr_t TelemetryTask_attributes = {
  .name = "TelemetryTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for SafetyTask */
osThreadId_t SafetyTaskHandle;
const osThreadAttr_t SafetyTask_attributes = {
  .name = "SafetyTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for CommunicationTa */
osThreadId_t CommunicationTaHandle;
const osThreadAttr_t CommunicationTa_attributes = {
  .name = "CommunicationTa",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for DisplayTask */
osThreadId_t DisplayTaskHandle;
const osThreadAttr_t DisplayTask_attributes = {
  .name = "DisplayTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for dataMutex */
osMutexId_t dataMutexHandle;
const osMutexAttr_t dataMutex_attributes = {
  .name = "dataMutex"
};
/* Definitions for adc1Semaphore */
osSemaphoreId_t adc1SemaphoreHandle;
const osSemaphoreAttr_t adc1Semaphore_attributes = {
  .name = "adc1Semaphore"
};
/* Definitions for uart3TxSemaphore */
osSemaphoreId_t uart3TxSemaphoreHandle;
const osSemaphoreAttr_t uart3TxSemaphore_attributes = {
  .name = "uart3TxSemaphore"
};
/* Definitions for alertSemaphore */
osSemaphoreId_t alertSemaphoreHandle;
const osSemaphoreAttr_t alertSemaphore_attributes = {
  .name = "alertSemaphore"
};
/* Definitions for uart2TxSemaphore */
osSemaphoreId_t uart2TxSemaphoreHandle;
const osSemaphoreAttr_t uart2TxSemaphore_attributes = {
  .name = "uart2TxSemaphore"
};
/* Definitions for uart3RxSemaphore */
osSemaphoreId_t uart3RxSemaphoreHandle;
const osSemaphoreAttr_t uart3RxSemaphore_attributes = {
  .name = "uart3RxSemaphore"
};
/* Definitions for uart6RxSemaphore */
osSemaphoreId_t uart6RxSemaphoreHandle;
const osSemaphoreAttr_t uart6RxSemaphore_attributes = {
  .name = "uart6RxSemaphore"
};
/* Definitions for uart5RxSemaphore */
osSemaphoreId_t uart5RxSemaphoreHandle;
const osSemaphoreAttr_t uart5RxSemaphore_attributes = {
  .name = "uart5RxSemaphore"
};
/* Definitions for uart6TxSemaphore */
osSemaphoreId_t uart6TxSemaphoreHandle;
const osSemaphoreAttr_t uart6TxSemaphore_attributes = {
  .name = "uart6TxSemaphore"
};
/* Definitions for uart1RxSemaphore */
osSemaphoreId_t uart1RxSemaphoreHandle;
const osSemaphoreAttr_t uart1RxSemaphore_attributes = {
  .name = "uart1RxSemaphore"
};
/* Definitions for uart4RxSemaphore */
osSemaphoreId_t uart4RxSemaphoreHandle;
const osSemaphoreAttr_t uart4RxSemaphore_attributes = {
  .name = "uart4RxSemaphore"
};
/* Definitions for uart4TxSemaphore */
osSemaphoreId_t uart4TxSemaphoreHandle;
const osSemaphoreAttr_t uart4TxSemaphore_attributes = {
  .name = "uart4TxSemaphore"
};
/* USER CODE BEGIN PV */

CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8];
uint8_t RxData[8];
uint32_t TxMailbox;
uint32_t MotorControllerTxInformation[5];
uint32_t BMSTxInformation[5];
uint8_t DireksiyonTxInformation[5];

CAN_TxHeaderTypeDef h2TxHeader;
uint8_t h2TxData[8];
uint32_t h2TxMailbox;

// Ekran
uint8_t buffer[9] = { 0x5A, 0xA5, 0x05, 0x82 };
float kusurat;

// BYS
float min_cell_v = 5.0f, avg_cell_v = 0, max_cell_v = 0;
float battery_voltage = 0;		// Toplam paket voltajı
int16_t battery_current = 0;	// Anlık akım
uint8_t battery_soc = 0;		// Şarj durumu
float cell_voltages[48];		// Her hücrenin voltajı
float ntc_temperatures[4];
int temp0;
int temp1;
int temp2;
int temp3;
uint8_t min_temp = 0, max_temp = 0;
uint16_t v_raw, i_raw, s_raw;

// Motor sürücü
DriverToVcu_Data_t driver_rx_pkt;
VcuToDriver_Data_t driver_tx_pkt;
uint8_t driver_rx_buffer[DRIVER_RX_BUFFER_SIZE];
uint8_t driver_rx_byte;
volatile uint16_t driver_rx_index = 0;

uint16_t driver_error_code = 0;
int RPM = 0;
int ref = 0;
uint8_t speed;

// NTC ve H2 (ADC)
uint16_t driver_temp = 0;
uint16_t motor_temp = 0;
uint32_t adc_raw;
float ntc_voltage;
uint32_t h2_adc_value = 0;
uint16_t adc2_dma_buffer[ADC2_BUFFER_SIZE];

// H2 kartı
H2ToVcu_Data_t h2_rx_pkt;
VcuToH2_Data_t h2_tx_pkt;
uint8_t h2_rx_buffer[H2_RX_BUFFER_SIZE];
uint8_t h2_rx_byte;
volatile uint16_t h2_rx_index = 0;

float h2_current = 0.0f;
float h2_temp = 0.0f;

// Telemetri
TelemetryPacket packet;
const uint8_t ADDH = 0x07;
const uint8_t ADDL = 0x60; // 9600 baud, 2.4k airrate
const uint8_t CH = 0x1F;
const uint8_t SPED = 0x1A;
uint8_t config_cmd[] = { 0xC0, ADDH, ADDL, SPED, CH, 0xC0 };

// Zamanlama değişkenleri
uint32_t dataCurrentTime = 0;
uint32_t dataLastTime = 0;
uint32_t displayCurrentTime = 0;
uint32_t displayLastTime = 0;
uint32_t buttonCurrentTime = 0;
uint32_t buttonLastTime = 0;

// BYS zamanlayıcılar
static uint32_t bms_timer = 0;
static uint32_t cell_timer = 0;
static uint32_t ntc_timer = 0;
static uint32_t ysb_timer = 0;
static uint32_t h2_timer = 0;

// Direksiyon
Buttons_Data_t buttons_rx_pkt;
uint8_t buttons_rx_buffer[BUTTONS_RX_BUFFER_SIZE];
uint8_t buttons_rx_byte;
volatile uint8_t buttons_rx_index = 0;

uint8_t solSinyalButton = 0;
uint8_t sagSinyalButton = 0;
uint8_t dortluButton = 0;
uint8_t resetButton = 0;
uint8_t surucuKapatButton = 0;
uint8_t sayfaDegistirButton= 0;
uint8_t h2ArttirButton = 0;
uint8_t h2AzaltButton = 0;
uint8_t farButton = 0;
int8_t currentPage = 0;

// Korna ve flaşör bayrak takibi
bool korna = 0;
bool flasor = 0;

// Şarj modu kontrolü
bool charging = 0;
bool charger_status = 0;

// YŞB'ye gönderilen komutlar
float user_voltage = 55.0f;
float user_current = 20.0f;
uint8_t user_charge = 0;

// YŞB'den alınan veriler
float YSB_Voltage = 0.0f;
float YSB_Current = 0.0f;
uint8_t YSB_Status = 0;
uint8_t YSB_HardwareFault = 0;
uint8_t YSB_TemperatureFault = 0;
uint8_t YSB_InputVoltageFault = 0;
uint8_t YSB_BatteryFault = 0;
uint8_t YSB_CommunicationFault = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC2_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_UART5_Init(void);
static void MX_UART4_Init(void);
void vTelemetryTask(void *argument);
void vSafetyTask(void *argument);
void vCommunicationTask(void *argument);
void vDisplayTask(void *argument);

/* USER CODE BEGIN PFP */

void Dwin_Send_Int(uint16_t adress, uint16_t data);
void Dwin_Send_Float(uint16_t adress, float data);
void DWIN_SetPage(uint16_t page_id);
void Display_Startup_Animation();
void Start_Display(void);
void Send_To_Display();
void Send_Signals_To_Display();
void LoraMode();
void TelemetryStart();
void TelemetryData();
void Send_Request_BMS();
void Driver_Receive_Start();
uint8_t Calculate_Checksum(uint8_t *data, uint16_t len);
void Receive_Surucu();
void Receive_BMS();
void Receive_Direksiyon();
void System_On();
void Enable_Multiplexer();
static void MUX_Select(uint8_t channel);
uint16_t Read_ADC(ADC_HandleTypeDef *hadc, uint32_t channel);
int16_t Raw_To_Temp();
uint16_t Driver_Temperature(void);
uint16_t Motor_Temperature(void);
void Horn_Flasher_Control();
void Charge_Mode_Control();
void YSB_Send_Command(bool charging);
void Receive_Buttons(void);
void Charger_Send_Command(float voltage, float current, uint8_t charge);
void Receive_YSB(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART3) {
		receivedCmd = receiveBuffer;
		cmdReceivedFlag = true;

		osSemaphoreRelease(uart3RxSemaphoreHandle);
		HAL_UART_Receive_IT(&huart3, &receiveBuffer, 1);
		HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin); // Veri akışını gör
	}
	if (huart->Instance == UART5) {
		buttons_rx_buffer[buttons_rx_index] = buttons_rx_byte;
		buttons_rx_index++;

		if (buttons_rx_index >= BUTTONS_RX_BUFFER_SIZE) {
			buttons_rx_index = 0;
		}

		osSemaphoreRelease(uart5RxSemaphoreHandle);
		HAL_UART_Receive_IT(&huart5, &buttons_rx_byte, 1);
		HAL_GPIO_TogglePin(LED5_GPIO_Port, LED5_Pin); // Veri akışını gör
	}
	if (huart->Instance == USART6) { //SURUCU
		// 1. Gelen baytı tampona yaz
		driver_rx_buffer[driver_rx_index] = driver_rx_byte;
		driver_rx_index++;

		// 2. Başa sar (Ring Buffer)
		if (driver_rx_index >= DRIVER_RX_BUFFER_SIZE) {
			driver_rx_index = 0;
		}

		osSemaphoreRelease(uart6RxSemaphoreHandle);
		// 3. Kesmeyi yenile
		HAL_UART_Receive_IT(&huart6, &driver_rx_byte, 1);

		HAL_GPIO_TogglePin(LED6_GPIO_Port, LED6_Pin); // Veri akışını gör
	}
	if (huart->Instance == UART4) {	// H2 KARTI CAN İLE HABERLEŞECEK BU KISIM İPTAL
		h2_rx_buffer[h2_rx_index] = h2_rx_byte;
		h2_rx_index++;

		if (h2_rx_index >= H2_RX_BUFFER_SIZE) {
			h2_rx_index = 0;
		}

		osSemaphoreRelease(uart4RxSemaphoreHandle);
		HAL_UART_Receive_IT(&huart4, &h2_rx_byte, 1);
		HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin); // H2 veri akış ledi
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART2) {
		// DWIN Ekran UART gönderimi bittiğinde uyuyan görevi uyandır
		osSemaphoreRelease(uart2TxSemaphoreHandle);
	}
	if (huart->Instance == USART3) {
		// LoRa UART gönderimi (Tx) bittiğinde uyuyan görevi uyandır
		osSemaphoreRelease(uart3TxSemaphoreHandle);
	}
	if (huart->Instance == USART6) {
		// Motor sürücü UART gönderimi bittiğinde uyuyan görevi uyandır
		osSemaphoreRelease(uart6TxSemaphoreHandle);
	}
	if (huart->Instance == UART4) {
		// H2 kartı UART gönderimi bittiğinde uyuyan görevi uyandır
		osSemaphoreRelease(uart4TxSemaphoreHandle);
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	if (hadc->Instance == ADC1) {
		osSemaphoreRelease(adc1SemaphoreHandle);
	}
}

void Dwin_Send_Int(uint16_t adress, uint16_t data) {
	buffer[4] = (adress & 0xFF00) >> 8;
	buffer[5] = adress & 0xFF;
	buffer[6] = (data & 0xFF00) >> 8;
	buffer[7] = data & 0xFF;

	HAL_UART_Transmit(&huart2, buffer, 8, 20);
}

void Dwin_Send_Float(uint16_t adress, float data) {
	int decimal_kusurat;
	kusurat = data - (int) data;
	decimal_kusurat = (kusurat * 100);

	// 1. Paket
	buffer[4] = (adress & 0xFF00) >> 8;
	buffer[5] = adress & 0xFF;
	buffer[6] = ((int) data & 0xFF00) >> 8;
	buffer[7] = (int) data & 0xFF;
	HAL_UART_Transmit(&huart2, buffer, 8, 20);

	adress++;

	// 2. Paket
	buffer[4] = (adress & 0xFF00) >> 8;
	buffer[5] = adress & 0xFF;
	buffer[6] = (decimal_kusurat & 0xFF00) >> 8;
	buffer[7] = decimal_kusurat & 0xFF;
	HAL_UART_Transmit(&huart2, buffer, 8, 20);
}

void DWIN_SetPage(uint16_t page_id) {
    uint8_t command[10] = {
        0x5A, 0xA5,               // Frame header
        0x07,                     // Data length
        0x82,                     // Write instruction
        0x00, 0x84,               // Sayfa geçiş adresi (0x0084)
        0x5A, 0x01,               // Sabit — sayfa işlemini başlat
        (uint8_t)(page_id >> 8),  // Page ID high byte
        (uint8_t)(page_id & 0xFF) // Page ID low byte
    };
    HAL_UART_Transmit(&huart2, command, 10, 100);
}

void Display_Startup_Animation() {
	HAL_GPIO_WritePin(LED8_GPIO_Port, LED8_Pin, GPIO_PIN_SET);
	osDelay(500);
	for (int i = 0; i <= 60; i += 1) {
		Dwin_Send_Int(0x5000, i);
		Dwin_Send_Int(0x5100, i);
		Dwin_Send_Int(0x5200, i);
		Dwin_Send_Int(0x5300, i);
		Dwin_Send_Int(0x5400, i);
		Dwin_Send_Int(0x5500, i);
		Dwin_Send_Int(0x5600, i);
		Dwin_Send_Int(0x5700, i);
		osDelay(10);
	}
	for (int i = 60; i >= 0; i -= 1) {
		Dwin_Send_Int(0x5000, i);
		Dwin_Send_Int(0x5100, i);
		Dwin_Send_Int(0x5200, i);
		Dwin_Send_Int(0x5300, i);
		Dwin_Send_Int(0x5400, i);
		Dwin_Send_Int(0x5500, i);
		Dwin_Send_Int(0x5600, i);
		Dwin_Send_Int(0x5700, i);
		osDelay(10);
	}
	HAL_GPIO_WritePin(LED8_GPIO_Port, LED8_Pin, GPIO_PIN_RESET);
}

void Start_Display(){
	// 1. SAYFA
	Send_Signals_To_Display();
	Dwin_Send_Int(0x5100, battery_current);
	Dwin_Send_Int(0x5200, max_temp);
	Dwin_Send_Int(0x5300, h2_current);
	Dwin_Send_Int(0x5400, motor_temp);
	Dwin_Send_Int(0x5500, battery_voltage);
	Dwin_Send_Int(0x5600, driver_temp);
	Dwin_Send_Int(0x5700, speed);
	Dwin_Send_Int(0x5800, ref);
	Dwin_Send_Int(0x5900, battery_soc);
	Dwin_Send_Int(0x5950, driver_error_code);

	// 2. SAYFA
	Dwin_Send_Float(0x7300, min_cell_v);
	Dwin_Send_Float(0x7310, battery_voltage);
	Dwin_Send_Float(0x7320, max_cell_v);
	Dwin_Send_Int(0x7771, temp0);
	Dwin_Send_Int(0x7772, temp1);
	Dwin_Send_Int(0x7773, temp2);
	Dwin_Send_Int(0x7774, temp3);
	Dwin_Send_Int(0x7900, battery_soc);
	Dwin_Send_Int(0x7950, driver_error_code);
}

void Send_To_Display() {
	if (sayfaDegistirButton == 1) {
		currentPage++;
		if (currentPage > MAX_PAGE) currentPage = MIN_PAGE;
		DWIN_SetPage(currentPage);
		sayfaDegistirButton = 0;
	}

	// Ekran güncellemesinden önce hücre verilerini analiz et
	float sum_voltage= 0;
	uint8_t number_of_cells_read = 0;

	min_cell_v = 5.0f; // Taramadan önce sıfırla
	max_cell_v = 0.0f;

	for (int i = 0; i < 48; i++) {
		if (cell_voltages[i] <= 0.5f)
			continue;

		if (cell_voltages[i] < min_cell_v)
			min_cell_v = cell_voltages[i];
		if (cell_voltages[i] > max_cell_v)
			max_cell_v = cell_voltages[i];

		sum_voltage += cell_voltages[i];
		number_of_cells_read++;
	}

	if (number_of_cells_read > 0) {
		avg_cell_v = sum_voltage / number_of_cells_read;
	}

	// Sıcaklık ölçümleri
	driver_temp = Driver_Temperature();
	motor_temp = Motor_Temperature();

	// Değişim takibi için static değişkenler
	static int old_motor_temp = -999;
	static int old_max_temp = -999;
	static int old_driver_temp = -999;
	static float old_battery_voltage = -999.0f;
	static uint8_t old_speed = 255;
	static uint16_t old_battery_current = 65535;
	static uint16_t old_h2_current = 65535;
	static int old_ref = -999;
	static uint16_t old_driver_error_code = 999;
	static uint8_t old_battery_soc = 99;
	static uint8_t old_farButton = 255;

	static int old_temp0 = -999;
	static int old_temp1 = -999;
	static int old_temp2 = -999;
	static int old_temp3 = -999;

	static float old_cells[14] = { 0.0 };
	static float old_min_cell = -1.0f, old_max_cell = -1.0f;

	old_min_cell = min_cell_v;
	old_max_cell = max_cell_v;

	if (farButton != old_farButton) {
		Dwin_Send_Int(0x5000, farButton);
		old_farButton = farButton;
	}
	if (battery_current != old_battery_current) {
		Dwin_Send_Int(0x5100, battery_current);
		old_battery_current = battery_current;
	}
	if (max_temp != old_max_temp) {
		Dwin_Send_Int(0x5200, max_temp);
		old_max_temp = max_temp;
	}
	if (h2_current != old_h2_current) {
		Dwin_Send_Float(0x5300, h2_current);
		old_h2_current = h2_current;
	}
	if (motor_temp != old_motor_temp) {
		Dwin_Send_Int(0x5400, motor_temp);
        old_motor_temp = motor_temp;
    }
	if (battery_voltage != old_battery_voltage) {
		Dwin_Send_Int(0x5500, battery_voltage);
		old_battery_voltage = battery_voltage;
	}
	if (driver_temp != old_driver_temp) {
		Dwin_Send_Int(0x5600, driver_temp);
        old_driver_temp = driver_temp;
    }
	if (speed != old_speed) {
		Dwin_Send_Int(0x5700, speed);
		old_speed = speed;
	}
	if (ref != old_ref) {
		Dwin_Send_Int(0x5800, ref);
		old_ref = ref;
	}
	if (battery_soc != old_battery_soc) {
		Dwin_Send_Int(0x5900, battery_soc);
		Dwin_Send_Int(0x7900, battery_soc);
		old_battery_soc = battery_soc;
	}
	if (driver_error_code != old_driver_error_code) {
		Dwin_Send_Int(0x5950, driver_error_code);
		Dwin_Send_Int(0x7950, driver_error_code);
		old_driver_error_code = driver_error_code;
	}

	// v1..v9 arası 7010..7090, v10..v14 arası 7100..7140
	uint16_t cell_addresses[14] = {
		0x7010, 0x7020, 0x7030, 0x7040, 0x7050,
		0x7060, 0x7070, 0x7080, 0x7090, 0x7100,
		0x7110, 0x7120, 0x7130, 0x7140
	};

	for (int i = 0; i < 14; i++) {
		Dwin_Send_Float(cell_addresses[i], cell_voltages[i + 3]);
		old_cells[i] = cell_voltages[i + 3];
	}

	if (temp0 != old_temp0) {
		Dwin_Send_Int(0x7771, temp0);
		old_temp0 = temp0;
	}

	if (temp1 != old_temp1) {
		Dwin_Send_Int(0x7772, temp1);
		old_temp1 = temp1;
	}

	if (temp2 != old_temp2) {
		Dwin_Send_Int(0x7773, temp2);
		old_temp2 = temp2;
	}

	if (temp3 != old_temp3) {
		Dwin_Send_Int(0x7774, temp3);
		old_temp3 = temp3;
	}
}

void Send_Signals_To_Display() {
	uint32_t current_time = osKernelGetTickCount();
	uint8_t blink_flag = (current_time % 800) < 400 ? 1 : 0;

	uint8_t target_sol = 0, target_sag = 0, target_dortlu = 0;

	if (dortluButton == 1) {
		target_sol = blink_flag;
		target_sag = blink_flag;
		target_dortlu = blink_flag;
	} else {
		if (solSinyalButton) target_sol = blink_flag;
		if (sagSinyalButton == 1) target_sag = blink_flag;
	}

	static uint8_t old_sol = 255, old_sag = 255, old_dortlu = 255;
	static uint8_t old_flasor = 255;

	if (target_sol != old_sol) {
		Dwin_Send_Int(0x5001, target_sol);
		old_sol = target_sol;
	}
	if (target_sag != old_sag) {
		Dwin_Send_Int(0x5003, target_sag);
		old_sag = target_sag;
	}
	if (target_dortlu != old_dortlu) {
		Dwin_Send_Int(0x5002, target_dortlu);
		old_dortlu = target_dortlu;
	}
	if (flasor != old_flasor) {
		Dwin_Send_Int(0x5004, flasor);
		old_flasor = flasor;
	}
}

void LoraMode() {
	// --- 1. ADIM: AYAR MODUNA GEÇİŞ ---
	HAL_GPIO_WritePin(RF_PARAMETRE_GPIO_Port, RF_PARAMETRE_Pin, GPIO_PIN_SET);
	HAL_Delay(100);

	// --- 2. ADIM: KONFİGÜRASYON PAKETİNİ GÖNDER ---
	HAL_UART_Transmit(&huart3, config_cmd, 6, 100);
	HAL_Delay(500); // Modülün ayarları kaydetmesini bekle

	// --- 3. ADIM: NORMAL MODA GEÇİŞ ---
	HAL_GPIO_WritePin(RF_PARAMETRE_GPIO_Port, RF_PARAMETRE_Pin, GPIO_PIN_RESET);
	HAL_Delay(500); // İletişime hazır olması için bekle
}

void TelemetryStart() {
	// Lora başlatma
	LoRa_Init();
}

void TelemetryData() {
	currentTime = HAL_GetTick();

	// Telemetri paketine verileri kopyalarken Mutex'i al
	if (osMutexAcquire(dataMutexHandle, 10) == osOK) {
		packet.speed = speed;
		packet.tubeTemp = h2_temp;
		packet.batteryTemp = max_temp;
		packet.voltage = battery_voltage;
		packet.charge = battery_soc;

		osMutexRelease(dataMutexHandle);
	}

	packet.timeHigh = (currentTime >> 16);
	packet.timeMid = (currentTime >> 8);
	packet.timeLow = currentTime;

	packet.crc = CRC_Calculate((uint8_t*) &packet, sizeof(TelemetryPacket) - 1);
}

void Send_Request_BMS() {
	if (HAL_GetTick() - bms_timer > 450) {
		// Gönderme ID: Priority(0x18) + DataID(0x90) + BMS_Addr(0x01) + PC_Addr(0x40)
		TxHeader.ExtId = 0x18900140;
		TxHeader.IDE = CAN_ID_EXT;     // Daly BMS Genişletilmiş ID kullanır
		TxHeader.RTR = CAN_RTR_DATA;
		TxHeader.DLC = 8;

		memset(TxData, 0, 8); // Diğer byte'lar rezerve, 0 gönderiyoruz

		HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
		bms_timer = HAL_GetTick();
	}

	// --- HÜCRE GERİLİMİ SORGUSU (Her 1.5 saniyede bir) ---
	if (HAL_GetTick() - cell_timer > 300) {
		TxHeader.ExtId = 0x18950140; // Data ID: 0x95 [cite: 26]
		TxHeader.DLC = 8;
		memset(TxData, 0, 8);
		HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
		cell_timer = HAL_GetTick();
	}

	// --- NTC SICAKLIK SORGUSU (Her 1 saniyede bir) ---
	if (HAL_GetTick() - ntc_timer > 150) {
		TxHeader.ExtId = 0x18960140; // Data ID: 0x96
		TxHeader.DLC = 8;
		memset(TxData, 0, 8);
		HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
		ntc_timer = HAL_GetTick();
	}
}

void Driver_Receive_Start(){
	HAL_UART_Receive_IT(&huart6, &driver_rx_byte, 1);
}

// Basit Checksum Hesaplayıcı
uint8_t Calculate_Checksum(uint8_t *data, uint16_t len) {
    uint8_t crc = 0;
    for (uint16_t i = 0; i < len; i++) {
        crc += data[i];
    }
    return crc;
}

void Receive_Surucu(void) {
    uint8_t pkt_len = sizeof(DriverToVcu_Data_t);

    // 1. Henüz tam bir paket boyutu kadar veri gelmediyse çık
    if (driver_rx_index < pkt_len) {
        return;
    }

    // 2. Buffer içerisinde paketi ara
    for (int i = 0; i <= (driver_rx_index - pkt_len); i++) {

        // Paket Başlığı (SOF: 0xA55A) Kontrolü (Little-Endian)
        uint16_t possible_sof = (uint16_t)driver_rx_buffer[i] | ((uint16_t)driver_rx_buffer[i + 1] << 8);

        if (possible_sof == MOTOR_DRIVER_SOF) {

            // Checksum Doğrulaması
            uint8_t calc_crc = Calculate_Checksum(&driver_rx_buffer[i], pkt_len - 1);
            uint8_t rx_crc = driver_rx_buffer[i + pkt_len - 1];

            if (calc_crc == rx_crc) {
                // Veri Doğru -> Struct'a kopyala
                memcpy(&driver_rx_pkt, &driver_rx_buffer[i], pkt_len);

                // Global değişkenlere aktar
                RPM = driver_rx_pkt.rpm;
                ref = driver_rx_pkt.reference;
                driver_error_code = driver_rx_pkt.faults; // MCSDK ham hata kodu (1024, 128, 64...)

                // Hız hesaplama
                speed = (uint16_t)(RPM * 3.14159f * 0.55f * 60.0f / 1000.0f);

                // Paketi işledikten sonra buffer ve indeksi sıfırla
                driver_rx_index = 0;
                memset(driver_rx_buffer, 0, DRIVER_RX_BUFFER_SIZE);
                break;
            }
        }
    }

    // Buffer dolduysa ve geçerli paket bulunamadıysa taşmayı önlemek için sıfırla
    if (driver_rx_index >= DRIVER_RX_BUFFER_SIZE - 1) {
        driver_rx_index = 0;
    }
}

void Receive_BMS() {
	// Kuyrukta mesaj varsa en eski mesajı okuyup "RxData" dizisine ve "RxHeader" yapısına aktar.
	// Bu fonksiyon mesajı okur ve FIFO0'ı bir sonraki mesaj için boşaltır.
	// Sadece Extended ID (BMS) mesajlarını kabul et

	if (RxHeader.ExtId == 0x18904001) {
		BMSTxInformation[0] = RxHeader.ExtId; // Extended ID ise ExtId'yi kaydet
		BMSTxInformation[1] = RxHeader.DLC; // Mesajın kaç byte olduğunu (Data Length Code) kaydet.

		// TOPLAM VOLTAJ: Byte 0 ve 1 birleştirilir, 0.1 ile çarpılır
		v_raw = (RxData[0] << 8) | RxData[1];
		battery_voltage = v_raw * 0.1f;
		//battery_voltage = (uint8_t) (v_raw * 0.1f) ;

		// AKIM: Byte 4 ve 5 birleştirilir. 30000 offset çıkarılır ve 0.1 ile çarpılır
		i_raw = (RxData[4] << 8) | RxData[5];
		battery_current = (int16_t) ((i_raw - 30000) * 0.1f);
		//battery_current = (uint8_t)((i_raw - 30000) * 0.1f);

		// SOC: Byte 6 ve 7 birleştirilir, 0.1 ile çarpılır
		s_raw = (RxData[6] << 8) | RxData[7];
		battery_soc = (uint8_t) (s_raw * 0.1f);

		// Veri geldiğini anlamak için LED'i yak söndür
		HAL_GPIO_TogglePin(LED7_GPIO_Port, LED7_Pin);
	}

	else if (RxHeader.ExtId == 0x18954001) { // Hücre gerilimleri paketi
		// Byte 0: Frame numarasını verir (0'dan başlar)
		uint8_t frame_no = RxData[0];

		if (frame_no >= 1 && frame_no <= 16) { // Maksimum 16 frame gelebilir
		// Her frame içerisinde 3 adet hücre voltajı bulunur (Byte 1-2, 3-4, 5-6)
		// Veri formatı: 1 mV birimindedir

			// Hücre 1 (Bu frame'deki)
			uint16_t c1 = (RxData[1] << 8) | RxData[2];
			cell_voltages[frame_no * 3 + 0] = c1 / 1000.0f;
			//cell_voltages[frame_no * 3 + 0] = (uint8_t)(c1 / 1000.0f); // mV -> Volt dönüşümü

			// Hücre 2 (Bu frame'deki)
			uint16_t c2 = (RxData[3] << 8) | RxData[4];
			cell_voltages[frame_no * 3 + 1] = c2 / 1000.0f;
			//cell_voltages[frame_no * 3 + 1] = (uint8_t)(c2 / 1000.0f);

			// Hücre 3 (Bu frame'deki)
			uint16_t c3 = (RxData[5] << 8) | RxData[6];
			cell_voltages[frame_no * 3 + 2] = c3 / 1000.0f;
			//cell_voltages[frame_no * 3 + 2] = (uint8_t)(c3 / 1000.0f);

			float toplam_hucre_voltaji = 0;
			uint8_t okunan_hucre_sayisi = 0;
			for (int i = 0; i < 48; i++) {
				if (cell_voltages[i] <= 0.5f)
					continue;
				if (cell_voltages[i] < min_cell_v)
					min_cell_v = cell_voltages[i];
				if (cell_voltages[i] > max_cell_v)
					max_cell_v = cell_voltages[i];
				toplam_hucre_voltaji += cell_voltages[i];
				okunan_hucre_sayisi++;
			}
			if (okunan_hucre_sayisi > 0) {
				avg_cell_v = toplam_hucre_voltaji / okunan_hucre_sayisi;
			}
			//battery_soc = (uint8_t) (100 * toplam_hucre_voltaji / 58.8f);
		}
		HAL_GPIO_TogglePin(LED7_GPIO_Port, LED7_Pin);
	}

	else if (RxHeader.ExtId == 0x18964001) {
		// Byte 0: Frame numarasını verir (0, 1 veya 2)
		uint8_t frame_no = RxData[0];
		if (frame_no == 0) {  // İlk frame'de min/max sıfırla
			min_cell_v = 5.0f;
			max_cell_v = 0.0f;
		}
		if (frame_no == 1) {
			ntc_temperatures[0] = (float) RxData[1] - 40.0f;
			ntc_temperatures[1] = (float) RxData[2] - 40.0f;
			ntc_temperatures[2] = (float) RxData[3] - 40.0f;
			ntc_temperatures[3] = (float) RxData[4] - 40.0f;

			min_temp =
					(ntc_temperatures[0] < ntc_temperatures[1]) ?
							(uint8_t) ntc_temperatures[0] :
							(uint8_t) ntc_temperatures[1];
			max_temp =
					(ntc_temperatures[0] > ntc_temperatures[1]) ?
							(uint8_t) ntc_temperatures[0] :
							(uint8_t) ntc_temperatures[1];
			temp0 = ntc_temperatures[0];
			temp1 = ntc_temperatures[1];
			temp2 = ntc_temperatures[2];
			temp3 = ntc_temperatures[3];
		}
		HAL_GPIO_TogglePin(LED7_GPIO_Port, LED7_Pin);
	}
}

void Transmit_Motor_Driver(void) {  // BUTONLA KONTROL İPTAL EDİLDİ
    // Struct doldurma
    driver_tx_pkt.sof = MOTOR_DRIVER_SOF;
	driver_tx_pkt.driver_reset = (uint16_t) resetButton;
	driver_tx_pkt.engine_off = (uint16_t) surucuKapatButton;

    // Checksum hesapla (SOF'tan faults'a kadar olan alanlar)
    driver_tx_pkt.crc = Calculate_Checksum((uint8_t*)&driver_tx_pkt, sizeof(VcuToDriver_Data_t) - 1);

    // Paketi tek seferde binary olarak gönder
    if(HAL_UART_Transmit_IT(&huart6, (uint8_t*)&driver_tx_pkt, sizeof(VcuToDriver_Data_t)) == HAL_OK){
    	osSemaphoreAcquire(uart6TxSemaphoreHandle, pdMS_TO_TICKS(10));
    }
}

void System_On() {
//	Display_Startup_Animation();
	// Preşarj devresini ve ardından motor sürücüyü açar
	HAL_Delay(1000);
	HAL_GPIO_WritePin(YEDEK_GPIO_Port, YEDEK_Pin, 1); // ILK KONTAKTORU (DIRENCLI) AC
	HAL_Delay(2000);
	HAL_GPIO_WritePin(YSB_GPIO_Port, YSB_Pin, 1); // IKINCI KONTAKTORU (DIRENCSIZ) AC
	HAL_Delay(2000);
	HAL_GPIO_WritePin(YEDEK_GPIO_Port, YEDEK_Pin, 0); // ILK KONTAKTORU (DIRENCLI) KAPAT
}

void Enable_Multiplexer() {
	HAL_GPIO_WritePin(MUX_EN_GPIO_Port, MUX_EN_Pin, GPIO_PIN_RESET);
}

static void MUX_Select(uint8_t channel)
{
    HAL_GPIO_WritePin(MUX_S0_GPIO_Port, MUX_S0_Pin, (channel & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MUX_S1_GPIO_Port, MUX_S1_Pin, (channel & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MUX_S2_GPIO_Port, MUX_S2_Pin, (channel & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

uint16_t Read_ADC(ADC_HandleTypeDef *hadc, uint32_t channel)
{
	ADC_ChannelConfTypeDef sConfig = { 0 };
	sConfig.Channel = channel;
	sConfig.Rank = 1;
	HAL_ADC_ConfigChannel(hadc, &sConfig);

	// ADC'yi IT (Kesme) modunda başlat
	HAL_ADC_Start_IT(hadc);

	// Dönüşümün bitmesini kesme ve semafor ile bekle (Maks 10ms)
	if (osSemaphoreAcquire(adc1SemaphoreHandle, 10) != osOK) {
		HAL_ADC_Stop_IT(hadc);
		return 0xFFFF; // Hata / Timeout durumu
	}

	uint16_t value = HAL_ADC_GetValue(hadc);
	return value;
}

int16_t Raw_To_Temp(){
	adc_raw = Read_ADC(&hadc1, ADC_CHANNEL_9);

	if (adc_raw == 0 || adc_raw >= 4095 || adc_raw == 0xFFFF) {
		return -999;
	}

	ntc_voltage = ((float) adc_raw / 4095.0f) * VCC;

	// Pull-down devresi için NTC'nin o anki direncini hesapla (Gerilim bölücü formülü)
	float r_ntc = R_PULLDOWN * ((4095.0f / (float) adc_raw) - 1.0f);

	// Beta formülü ile sıcaklığı hesapla
	float tempK = 1.0f / ((1.0f / T0) + (1.0f / BETA) * logf(r_ntc / R0));

	// Celsius'a çevir
	int16_t tempC = (int16_t) (tempK - 273.15f);
	return tempC;
}

uint16_t Driver_Temperature(){
	Enable_Multiplexer();
	MUX_Select(0);
	osDelay(2);
    return Raw_To_Temp();
}

uint16_t Motor_Temperature(){
	Enable_Multiplexer();
	MUX_Select(1);
	osDelay(2);
	return Raw_To_Temp();
}

void Horn_Flasher_Control() {
	uint8_t loc_max_temp = 0;

	// Durumları takip etmek için static değişkenler (fonksiyondan çıkılsa bile değerini korur)
	static uint8_t sequence_step = 0;
	static uint32_t state_start_time = 0;

	// RTOS'un o anki milisaniye cinsinden zamanını al
	uint32_t current_time = osKernelGetTickCount();

	// Global değişkenden değeri yerel değişkene al
	if (osMutexAcquire(dataMutexHandle, 10) == osOK) {
		loc_max_temp = max_temp;
		osMutexRelease(dataMutexHandle);
	}

	//h2_adc_value = adc2_dma_buffer[0]; // ADC’nin okuduğu dijital değeri (ölçülen analog sinyalin dijital karşılığını) al.

	// SICAKLIK TEHLİKE SINIRINDAYSA UYARI DİZİSİNİ İŞLET
	if ((loc_max_temp > 55 && loc_max_temp < 70) || h2_adc_value > 200) {

		switch (sequence_step) {
		case 0: // Başlangıç durumu
			HAL_GPIO_WritePin(FLASOR_GPIO_Port, FLASOR_Pin, 1);
			HAL_GPIO_WritePin(KORNA_GPIO_Port, KORNA_Pin, 1);
			flasor = 1;
			korna = 1;
			state_start_time = current_time; // Zamanlayıcıyı başlat
			sequence_step = 1;               // Bir sonraki adıma geç
			break;

		case 1: // Flaşör ve kornayı kapatmak için 2000 ms bekle
			if ((current_time - state_start_time) >= 2000) {
				HAL_GPIO_WritePin(FLASOR_GPIO_Port, FLASOR_Pin, 0);
				HAL_GPIO_WritePin(KORNA_GPIO_Port, KORNA_Pin, 0);
				flasor = 0;
				korna = 0;
				state_start_time = current_time;
				sequence_step = 2;
			}
			break;

		case 2: // Flaşör ve kornayı açmak için 1000 ms bekle
			if ((current_time - state_start_time) >= 1000) {
				HAL_GPIO_WritePin(FLASOR_GPIO_Port, FLASOR_Pin, 1);
				HAL_GPIO_WritePin(KORNA_GPIO_Port, KORNA_Pin, 1);
				flasor = 1;
				korna = 1;
				state_start_time = current_time;
				sequence_step = 3;
			}
			break;

		case 3: // Flaşör ve kornayı kapatmak için 2000 ms bekle
			if ((current_time - state_start_time) >= 2000) {
				HAL_GPIO_WritePin(FLASOR_GPIO_Port, FLASOR_Pin, 0);
				HAL_GPIO_WritePin(KORNA_GPIO_Port, KORNA_Pin, 0);
				flasor = 0;
				korna = 0;
				state_start_time = current_time;
				sequence_step = 4;
			}
			break;

		case 4: // Diziyi başa sarmak için 1000 ms bekle
			if ((current_time - state_start_time) >= 1000) {
				// Sıcaklık hala tehlike sınırındaysa dizi baştan başlar
				sequence_step = 0;
			}
			break;
		}
	}

	// 2) SICAKLIK NORMALE DÖNDÜYSE BAŞLANGIÇ DURUMUNA DÖN
	else if (loc_max_temp <= 55) {
		HAL_GPIO_WritePin(FLASOR_GPIO_Port, FLASOR_Pin, 0);
		HAL_GPIO_WritePin(KORNA_GPIO_Port, KORNA_Pin, 0);
		flasor = 0;
		korna = 0;
		// Mod değişikliği durumunda ana devre yolunun sıfırlanması için adım 0'a çekilir
		sequence_step = 0;
	}

	// 3) KRİTİK HATA (70 DERECE ÜSTÜ)
	else {
		 // Sistemi kapat
		 HAL_GPIO_WritePin(YEDEK_GPIO_Port, YEDEK_Pin, 1);
	}
}

void Charge_Mode_Control() {
	if (YSB_Status == 1 && max_temp < 55 && battery_voltage < 58.8) {
		charging = 1;
		HAL_GPIO_WritePin(YSB_GPIO_Port, YSB_Pin, 0);
		YSB_Send_Command(charging); // YŞB'ye şarjı "BAŞLAT" komutu gönder
	} else if (YSB_Status == 0 || max_temp > 55 || battery_voltage >= 58.8 || battery_current <= 1.5) {
		charging = 0;
		HAL_GPIO_WritePin(YSB_GPIO_Port, YSB_Pin, 1);
		YSB_Send_Command(charging); // YŞB'ye şarjı "BİTİR" komutu gönder
	}
}

void YSB_Send_Command(bool charging) {
	memset(TxData, 0, 8);
	TxData[0] = charging;
	HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
}

void Direksiyon_Receive_Start() {
	HAL_UART_Receive_IT(&huart5, &buttons_rx_byte, 1);
}

void Receive_Buttons() {
	uint8_t pkt_len = sizeof(Buttons_Data_t);

	if (buttons_rx_index < pkt_len) {
		return;
	}

	for (int i = 0; i <= buttons_rx_index - pkt_len; i++) {

		if (buttons_rx_buffer[i] == BUTTONS_SOF) {

			uint8_t calc_crc = Calculate_Checksum(&buttons_rx_buffer[i], pkt_len - 1);
			uint8_t received_crc = buttons_rx_buffer[i + pkt_len - 1];

			if (calc_crc == received_crc) {
				memcpy(&buttons_rx_pkt, &buttons_rx_buffer[i], pkt_len);

				solSinyalButton = buttons_rx_pkt.solSinyal;
				sagSinyalButton = buttons_rx_pkt.sagSinyal;
				dortluButton = buttons_rx_pkt.dortlu;
				resetButton = buttons_rx_pkt.reset;
				surucuKapatButton = buttons_rx_pkt.surucuKapat;
				sayfaDegistirButton = buttons_rx_pkt.sayfaDegistir;
				farButton = buttons_rx_pkt.far;
				h2ArttirButton = buttons_rx_pkt.h2Arttir;
				h2AzaltButton = buttons_rx_pkt.h2Azalt;

				buttons_rx_index = 0;
				memset(buttons_rx_buffer, 0, BUTTONS_RX_BUFFER_SIZE);
				break;
			}
		}
	}
	if (buttons_rx_index >= (BUTTONS_RX_BUFFER_SIZE - 1)) {
		buttons_rx_index = 0;
	}
}

//YŞB'YE GÖNDERİLEN ŞARJ KONTROL KOMUTU
void Charger_Send_Command(float voltage, float current, uint8_t charge)
{
    uint16_t voltageValue;
    uint16_t currentValue;

    voltageValue = (uint16_t)(voltage * 10.0f);
    currentValue = (uint16_t)(current * 10.0f);

    if (HAL_GetTick() - ysb_timer > 1000) {
		TxHeader.ExtId = 0x1806E5F4; //gönderilen can mesajının id'si
		TxHeader.IDE = CAN_ID_EXT;
		TxHeader.RTR = CAN_RTR_DATA;
		TxHeader.DLC = 8;
		TxHeader.TransmitGlobalTime = DISABLE;

		memset(TxData, 0, 8); // Diğer byte'lar rezerve, 0 gönderiyoruz

		TxData[0] = (voltageValue >> 8) & 0xFF;
		TxData[1] = voltageValue & 0xFF;

		TxData[2] = (currentValue >> 8) & 0xFF;
		TxData[3] = currentValue & 0xFF;

		TxData[4] = charge;

		TxData[5] = 0;   // 0 = Charging mode

		TxData[6] = 0;
		TxData[7] = 0;

		HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
		ysb_timer = HAL_GetTick();
	}
}

//YŞB'DEN GELEN VERİNİN OKUNMASI
void Receive_YSB(){
	uint16_t outputVoltage;
	uint16_t outputCurrent;

	outputVoltage = ((uint16_t) RxData[0] << 8) | RxData[1];

	outputCurrent = ((uint16_t) RxData[2] << 8) | RxData[3];

	YSB_Voltage = outputVoltage / 10.0f;
	YSB_Current = outputCurrent / 10.0f;

	YSB_Status = RxData[4];

	YSB_HardwareFault = (YSB_Status >> 0) & 0x01;

	YSB_TemperatureFault = (YSB_Status >> 1) & 0x01;

	YSB_InputVoltageFault = (YSB_Status >> 2) & 0x01;

	YSB_BatteryFault = (YSB_Status >> 3) & 0x01;

	YSB_CommunicationFault = (YSB_Status >> 4) & 0x01;

	HAL_GPIO_TogglePin(LED8_GPIO_Port, LED8_Pin);
}

void Receive_H2_CAN() {
    // Little-Endian
    int16_t raw_temp = (int16_t)(RxData[0] | (RxData[1] << 8));
    int16_t raw_current = (int16_t)(RxData[2] | (RxData[3] << 8));

    h2_temp = (float)raw_temp / 100.0f;
    h2_current = (float)raw_current / 100.0f;
    HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
}

void Transmit_H2_CAN()
{
    if (HAL_GetTick() - h2_timer > 500) {
		h2TxHeader.StdId = 0x407; // F103 için belirlenen Buton Komut ID'si
		h2TxHeader.IDE = CAN_ID_STD;
		h2TxHeader.RTR = CAN_RTR_DATA;
		h2TxHeader.DLC = 8;
		h2TxHeader.TransmitGlobalTime = DISABLE;

		h2TxData[0] = h2ArttirButton; // Byte 0: Artır (1 veya 0)
		h2TxData[1] = h2AzaltButton;  // Byte 1: Azalt (1 veya 0)

		if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0) {
			HAL_CAN_AddTxMessage(&hcan1, &h2TxHeader, h2TxData, &h2TxMailbox);
			HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
		}

		h2_timer = HAL_GetTick();
	}
}

void H2_Receive_Start(void) {
    HAL_UART_Receive_IT(&huart4, &h2_rx_byte, 1);
}

void Receive_H2_UART(void) {
    uint8_t pkt_len = sizeof(H2ToVcu_Data_t);

    if (h2_rx_index < pkt_len) {
        return;
    }

    for (int i = 0; i <= (h2_rx_index - pkt_len); i++) {
        uint16_t possible_sof = h2_rx_buffer[i] | (h2_rx_buffer[i + 1] << 8);

        if (possible_sof == H2_SOF) {
            uint8_t calc_crc = Calculate_Checksum(&h2_rx_buffer[i], pkt_len - 1);
            uint8_t rx_crc = h2_rx_buffer[i + pkt_len - 1];

            if (calc_crc == rx_crc) {
                memcpy(&h2_rx_pkt, &h2_rx_buffer[i], pkt_len);

                // Verileri float formatına çevirip global değişkenlere aktar
                h2_temp = (float)h2_rx_pkt.temperature / 100.0f;
                h2_current = (float)h2_rx_pkt.current / 100.0f;

                h2_rx_index = 0; // Buffer'ı temizle
                break;
            }
        }
    }
}

void Transmit_H2_UART(void) {
    // Struct doldurma
    h2_tx_pkt.sof = H2_SOF;
	h2_tx_pkt.h2Azalt = (uint16_t) h2AzaltButton;
	h2_tx_pkt.h2Arttir = (uint16_t) h2ArttirButton;

    // Checksum hesapla (SOF'tan faults'a kadar olan alanlar)
	h2_tx_pkt.crc = Calculate_Checksum((uint8_t*)&h2_tx_pkt, sizeof(VcuToH2_Data_t) - 1);

    // Paketi tek seferde binary olarak gönder
    if(HAL_UART_Transmit_IT(&huart4, (uint8_t*)&h2_tx_pkt, sizeof(VcuToH2_Data_t)) == HAL_OK){
    	osSemaphoreAcquire(uart4TxSemaphoreHandle, pdMS_TO_TICKS(10));
    }
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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_DAC_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_ADC2_Init();
  MX_CAN1_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  MX_UART5_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */
	System_On();
	//Display_Startup_Animation();
	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	Driver_Receive_Start();
	H2_Receive_Start();
	Direksiyon_Receive_Start();
	LoraMode();
	TelemetryStart();
	HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc2_dma_buffer, ADC2_BUFFER_SIZE);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of dataMutex */
  dataMutexHandle = osMutexNew(&dataMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of adc1Semaphore */
  adc1SemaphoreHandle = osSemaphoreNew(1, 1, &adc1Semaphore_attributes);

  /* creation of uart3TxSemaphore */
  uart3TxSemaphoreHandle = osSemaphoreNew(1, 1, &uart3TxSemaphore_attributes);

  /* creation of alertSemaphore */
  alertSemaphoreHandle = osSemaphoreNew(1, 1, &alertSemaphore_attributes);

  /* creation of uart2TxSemaphore */
  uart2TxSemaphoreHandle = osSemaphoreNew(1, 1, &uart2TxSemaphore_attributes);

  /* creation of uart3RxSemaphore */
  uart3RxSemaphoreHandle = osSemaphoreNew(1, 1, &uart3RxSemaphore_attributes);

  /* creation of uart6RxSemaphore */
  uart6RxSemaphoreHandle = osSemaphoreNew(1, 1, &uart6RxSemaphore_attributes);

  /* creation of uart5RxSemaphore */
  uart5RxSemaphoreHandle = osSemaphoreNew(1, 1, &uart5RxSemaphore_attributes);

  /* creation of uart6TxSemaphore */
  uart6TxSemaphoreHandle = osSemaphoreNew(1, 1, &uart6TxSemaphore_attributes);

  /* creation of uart1RxSemaphore */
  uart1RxSemaphoreHandle = osSemaphoreNew(1, 1, &uart1RxSemaphore_attributes);

  /* creation of uart4RxSemaphore */
  uart4RxSemaphoreHandle = osSemaphoreNew(1, 1, &uart4RxSemaphore_attributes);

  /* creation of uart4TxSemaphore */
  uart4TxSemaphoreHandle = osSemaphoreNew(1, 1, &uart4TxSemaphore_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	// görevler (tasks) başlamadan önce içlerini "boşaltıyoruz".
	osSemaphoreAcquire(adc1SemaphoreHandle, 0);
	osSemaphoreAcquire(uart3TxSemaphoreHandle, 0);
	osSemaphoreAcquire(alertSemaphoreHandle, 0);
	//osSemaphoreAcquire(adc2SemaphoreHandle, 0);
	osSemaphoreAcquire(uart2TxSemaphoreHandle, 0);
	osSemaphoreAcquire(uart3RxSemaphoreHandle, 0);
	osSemaphoreAcquire(uart6RxSemaphoreHandle, 0);
	osSemaphoreAcquire(uart5RxSemaphoreHandle, 0);
	osSemaphoreAcquire(uart6TxSemaphoreHandle, 0);
	osSemaphoreAcquire(uart4RxSemaphoreHandle, 0);
	osSemaphoreAcquire(uart4TxSemaphoreHandle, 0);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of TelemetryTask */
  TelemetryTaskHandle = osThreadNew(vTelemetryTask, NULL, &TelemetryTask_attributes);

  /* creation of SafetyTask */
  SafetyTaskHandle = osThreadNew(vSafetyTask, NULL, &SafetyTask_attributes);

  /* creation of CommunicationTa */
  CommunicationTaHandle = osThreadNew(vCommunicationTask, NULL, &CommunicationTa_attributes);

  /* creation of DisplayTask */
  DisplayTaskHandle = osThreadNew(vDisplayTask, NULL, &DisplayTask_attributes);

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
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		/*
		 sendRequestBMS();
		 voltaCAN();
		 LoRa_Run();
		 dataCurrentTime = HAL_GetTick();
		 if ((dataCurrentTime - dataLastTime) >= 600) {
		 TelemetryData();
		 LoRa_Run(packet.speed,
		 packet.tubeTemp,
		 packet.batteryTemp,
		 packet.voltage,
		 packet.charge,
		 packet.timeHigh,
		 packet.timeMid,
		 packet.timeLow);

		 Send_To_Display();
		 dataLastTime = dataCurrentTime;
		 }
		 H2();
		 BMS_Horn_Control();
		 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

	// FİLTRE AYARLARI
	CAN_FilterTypeDef CAN_Filtre = { 0 }; // Çöp verileri temizle

	CAN_Filtre.FilterActivation = CAN_FILTER_ENABLE;
	CAN_Filtre.FilterBank = 0;
	CAN_Filtre.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	CAN_Filtre.FilterIdHigh = 0;
	CAN_Filtre.FilterIdLow = 0;
	CAN_Filtre.FilterMaskIdHigh = 0x0000; // Maske 0 olduğu için her ID'yi kabul edecek
	CAN_Filtre.FilterMaskIdLow = 0x0000;
	CAN_Filtre.FilterMode = CAN_FILTERMODE_IDMASK;
	CAN_Filtre.FilterScale = CAN_FILTERSCALE_32BIT;
	CAN_Filtre.SlaveStartFilterBank = 14;

	if (HAL_CAN_ConfigFilter(&hcan1, &CAN_Filtre) != HAL_OK) {
		Error_Handler();
	}

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

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
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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
  HAL_GPIO_WritePin(GPIOE, FLASOR_Pin|KORNA_Pin|SELENOID_VALF_Pin|YSB_Pin
                          |SISTEM_Pin|MUX_S0_Pin|MUX_S1_Pin|MUX_S2_Pin
                          |LED2_Pin|LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(YEDEK_GPIO_Port, YEDEK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MUX_EN_GPIO_Port, MUX_EN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RF_PARAMETRE_GPIO_Port, RF_PARAMETRE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED8_Pin|LED7_Pin|LED6_Pin|LED5_Pin
                          |LED4_Pin|LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : FLASOR_Pin KORNA_Pin SELENOID_VALF_Pin YSB_Pin
                           SISTEM_Pin MUX_S0_Pin MUX_S1_Pin MUX_S2_Pin
                           MUX_EN_Pin LED2_Pin LED1_Pin */
  GPIO_InitStruct.Pin = FLASOR_Pin|KORNA_Pin|SELENOID_VALF_Pin|YSB_Pin
                          |SISTEM_Pin|MUX_S0_Pin|MUX_S1_Pin|MUX_S2_Pin
                          |MUX_EN_Pin|LED2_Pin|LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : YEDEK_Pin */
  GPIO_InitStruct.Pin = YEDEK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(YEDEK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : FORWARD_Pin BACK_Pin */
  GPIO_InitStruct.Pin = FORWARD_Pin|BACK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : RF_PARAMETRE_Pin */
  GPIO_InitStruct.Pin = RF_PARAMETRE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RF_PARAMETRE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED8_Pin LED7_Pin LED6_Pin LED5_Pin
                           LED4_Pin LED3_Pin */
  GPIO_InitStruct.Pin = LED8_Pin|LED7_Pin|LED6_Pin|LED5_Pin
                          |LED4_Pin|LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	if (hcan->Instance == CAN1) {
		// "if" yerine "while" kullanıyoruz. İçerideki tüm paketleri toplar.
		while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {
			if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
				if (RxHeader.IDE == CAN_ID_STD) {
//                    Receive_Direksiyon();
					if (RxHeader.StdId == 0x103) {
						Receive_H2_CAN();
					}
				} else if (RxHeader.IDE == CAN_ID_EXT) {
					if (RxHeader.ExtId == 0x18FF50E5) {
						Receive_YSB();
					} else {
						Receive_BMS(); // Daly BMS ID'leri (0x18904001, 0x18954001, 0x18964001)
					}
				}
			}
		}
	}
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_vTelemetryTask */
/**
 * @brief  Function implementing the TelemetryTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_vTelemetryTask */
void vTelemetryTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	/* Infinite loop */
	for (;;) {
		TelemetryData();			// Verileri paketler
		LoRa_PushPacket(&packet);	// Kuyruğa ekler
		LoRa_Run();         		// ACK / Timeout / Gönderim durumlarını kontrol eder

		osDelay(600);				// Görevi 600 ms uyut
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_vSafetyTask */
/**
 * @brief Function implementing the SafetyTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_vSafetyTask */
void vSafetyTask(void *argument)
{
  /* USER CODE BEGIN vSafetyTask */
	/* Infinite loop */

	for (;;) {
		Horn_Flasher_Control();
		osDelay(50);	// Görevi 50 ms uyut
	}
  /* USER CODE END vSafetyTask */
}

/* USER CODE BEGIN Header_vCommunicationTask */
/**
* @brief Function implementing the CommunicationTa thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vCommunicationTask */
void vCommunicationTask(void *argument)
{
  /* USER CODE BEGIN vCommunicationTask */
	/* Infinite loop */
	for (;;) {
		Send_Request_BMS(); // Periyodik BMS sorguları (Zamanlama takipleri içindedir)
		Charger_Send_Command(user_voltage, user_current, user_charge);

		if (osSemaphoreAcquire(uart6RxSemaphoreHandle, 0) == osOK) {
			Receive_Surucu();
		}

		if (osSemaphoreAcquire(uart5RxSemaphoreHandle, 0) == osOK) {
			Receive_Buttons();
		}
		/*if (osSemaphoreAcquire(uart4RxSemaphoreHandle, 0) == osOK) {
		  Receive_H2_UART();
		}*/

		Transmit_H2_CAN();

		//Transmit_Motor_Driver();
		osDelay(20);		// Görevi 20 ms uyut
	}
  /* USER CODE END vCommunicationTask */
}

/* USER CODE BEGIN Header_vDisplayTask */
/**
 * @brief Function implementing the DisplayTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_vDisplayTask */
void vDisplayTask(void *argument)
{
  /* USER CODE BEGIN vDisplayTask */
	Start_Display();
	//Display_Startup_Animation();
	/* Infinite loop */
	for (;;) {
		Send_Signals_To_Display();
		Send_To_Display();
		osDelay(50);		// Görevi 50 ms uyut
	}
  /* USER CODE END vDisplayTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
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
	while (1) {
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
