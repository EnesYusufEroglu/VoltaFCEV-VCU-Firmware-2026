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
 * @version        : 2.0.0 (FreeRTOS integrated)
 * @author         : VCU TEAM
 * @date           : 29-07-2026
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
#include "math.h"
#include "stdlib.h"
#include "LoRa.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define R_PULLDOWN 10000.0f   // 10K pull-down direnci
#define VCC         3.3f
#define BETA        3950.0f
#define T0          298.15f   // 25°C = 298.15K
#define R0          10000.0f  // 25°C'de NTC direnci
#define BMS_RX_BUFFER_SIZE 256
#define DRIVER_RX_BUFFER_SIZE 128 // 30 yerine daha geniş bir alan verelim

#define FILTER_COEFF 0.001f
#define FLOAT_TOLERANCE 0.001f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

CAN_HandleTypeDef hcan1;

DAC_HandleTypeDef hdac;

I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

/* Definitions for TelemetryTask */
osThreadId_t TelemetryTaskHandle;
const osThreadAttr_t TelemetryTask_attributes = { .name = "TelemetryTask",
		.stack_size = 512 * 4, .priority = (osPriority_t) osPriorityNormal, };
/* Definitions for SafetyTask */
osThreadId_t SafetyTaskHandle;
const osThreadAttr_t SafetyTask_attributes = { .name = "SafetyTask",
		.stack_size = 256 * 4, .priority = (osPriority_t) osPriorityRealtime, };
/* Definitions for CanTask */
osThreadId_t CanTaskHandle;
const osThreadAttr_t CanTask_attributes = { .name = "CanTask", .stack_size = 512
		* 4, .priority = (osPriority_t) osPriorityAboveNormal, };
/* Definitions for DisplayTask */
osThreadId_t DisplayTaskHandle;
const osThreadAttr_t DisplayTask_attributes = { .name = "DisplayTask",
		.stack_size = 256 * 4, .priority = (osPriority_t) osPriorityLow, };
/* USER CODE BEGIN PV */

CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8];
uint8_t RxData[8];
uint32_t TxMailbox;
uint32_t MotorControllerTxInformation[5];
uint32_t BMSTxInformation[5];
uint8_t DireksiyonTxInformation [5];

uint8_t buffer[9] = { 0x5A, 0xA5, 0x05, 0x82 };
float kusurat;

float min_cell_v = 5.0f, avg_cell_v = 0, max_cell_v = 0;
float bms_voltage = 0;   	 // Toplam paket voltajı
int16_t bms_current = 0;  	 // Anlık akım
uint8_t bms_soc = 0;       	 // Şarj durumu
float cell_voltages[48];     // Her hücrenin voltajı
float ntc_temperatures[2];
uint8_t min_temp = 25, max_temp = 0;
uint16_t v_raw, i_raw, s_raw;

uint8_t surucu_data[8];
uint16_t RPM, ref;
uint16_t error = 0;
uint8_t speed;
uint16_t faults;

uint32_t h2_adc_value = 0;

TelemetryPacket packet;
const uint8_t ADDH = 0x07;
const uint8_t ADDL = 0x60; // 9600 baud, 2.4k airrate
const uint8_t CH = 0x1F;
const uint8_t SPED = 0x1A;

uint8_t config_cmd[] = { 0xC0, ADDH, ADDL, SPED, CH, 0xC0 };

uint32_t dataCurrentTime = 0;
uint32_t dataLastTime = 0;
uint32_t displayCurrentTime = 0;
uint32_t displayLastTime = 0;
uint32_t buttonCurrentTime = 0;
uint32_t buttonLastTime = 0;

static uint32_t bms_timer = 0;
static uint32_t cell_timer = 0;
static uint32_t ntc_timer = 0;

uint8_t buton_data[8];
uint8_t buton_sinyal_right ;
uint8_t buton_sinyal_left;
uint8_t buton_sinyal_hazard;
uint8_t buton_sinyal_page;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC_Init(void);
static void MX_UART4_Init(void);
static void MX_UART5_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_ADC2_Init(void);
static void MX_SPI2_Init(void);
static void MX_CAN1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM2_Init(void);
void vTelemetryTask(void *argument);
void vSafetyTask(void *argument);
void vCanTask(void *argument);
void vDisplayTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART3) {
		receivedCmd = receiveBuffer;
		cmdReceivedFlag = true;

		HAL_UART_Receive_IT(&huart3, &receiveBuffer, 1);
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART3) {
		/*
		 Gönderme işlemi başarıyla tamamlandı.
		 Burada ekstra bir işlem yapmana gerek yok,
		 STM32 arka planda huart3.gState durumunu READY (Hazır) haline getirecektir.
		 */
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
	buffer[4] = (adress & 0xFF00) >> 8;
	buffer[5] = adress & 0xFF;
	buffer[6] = ((int) data & 0xFF00) >> 8;
	buffer[7] = (int) data & 0xFF;
	HAL_UART_Transmit(&huart2, buffer, 8, 20);
	adress++;
	buffer[4] = (adress & 0xFF00) >> 8;
	buffer[5] = adress & 0xFF;
	buffer[6] = (decimal_kusurat & 0xFF00) >> 8;
	buffer[7] = decimal_kusurat & 0xFF;
	HAL_UART_Transmit(&huart2, buffer, 8, 20);
}

void Display_Startup_Animation() {
	HAL_GPIO_WritePin(LED8_GPIO_Port, LED8_Pin, 1);
	HAL_Delay(500);
	for (int i = 0; i <= 60; i += 1) {
		Dwin_Send_Int(0x5400, i);
		Dwin_Send_Int(0x6400, i);
		Dwin_Send_Int(0x7400, i);
		Dwin_Send_Int(0x8400, i);
		HAL_Delay(10);
	}
	for (int i = 60; i >= 0; i -= 1) {
		Dwin_Send_Int(0x5400, i);
		Dwin_Send_Int(0x6400, i);
		Dwin_Send_Int(0x7400, i);
		Dwin_Send_Int(0x8400, i);
		HAL_Delay(10);
	}
	HAL_GPIO_WritePin(LED8_GPIO_Port, LED8_Pin, 0);
}

void Send_To_Display() {
	static int old_motor_temp = -999;
	static int old_max_temp = -999;
	static float old_battery_voltage = -999.0f;
	static int old_RPM = -999;
	static uint16_t old_speed = 65535;
	static int old_ref = -999;
	// Hücre Voltajları için eski değerler
	static float old_cells[14] = { 0 };
	static uint16_t old_error = 999;
	static uint8_t old_charge = 99;
	static uint8_t old_bms_soc = 99;       	 // Şarj durumu

	// --------------------------------------------------------
	// PAGE 1 & GENEL
	// --------------------------------------------------------

	/*Dwin_Send_Int(0x5500, bms_soc);
	 Dwin_Send_Int(0x5550, bms_soc);
	 Dwin_Send_Int(0x6500, bms_soc);
	 Dwin_Send_Int(0x7500, bms_soc);
	 Dwin_Send_Int(0x8500, bms_soc);
	 Dwin_Send_Int(0x6550, bms_soc);
	 Dwin_Send_Int(0x7550, bms_soc);
	 Dwin_Send_Int(0x8550, bms_soc);*/
	Dwin_Send_Int(0x5000, error);
	Dwin_Send_Int(0x5200, RPM);
	Dwin_Send_Int(0x5500, ref);
	Dwin_Send_Int(0x5700, bms_soc);

	if (error != old_error) {
		Dwin_Send_Int(0x5000, error);
		old_error = error;
	}

	if (RPM != old_RPM) {
		Dwin_Send_Int(0x5200, RPM);
		old_RPM = RPM;
	}

	if (ref != old_ref) {
		Dwin_Send_Int(0x5500, ref);
		old_ref = ref;
	}

	if (bms_soc != old_bms_soc) {
		Dwin_Send_Int(0x5700, bms_soc);
		old_bms_soc = bms_soc;
	}

	/*if (RPM != old_RPM) {
	 Dwin_Send_Int(0x5500, RPM);
	 //Dwin_Send_Int(0x6400, RPM);
	 old_RPM = RPM;
	 }
	 if (max_temp != old_max_temp) {
	 Dwin_Send_Int(0x5200, max_temp);
	 old_max_temp = max_temp;
	 }*/
	if (bms_voltage != old_battery_voltage) {
		Dwin_Send_Int(0x5600, bms_voltage);
		//Dwin_Send_Float(0x7310, bms_voltage);
		old_battery_voltage = bms_voltage;
	}
//    if (VBUS != old_VBUS) {
//        Dwin_Send_Int(0x5300, VBUS);
//        Dwin_Send_Float(0x7310, VBUS);
//        old_VBUS = VBUS;
//    }
	if (RPM != old_RPM) {
		Dwin_Send_Int(0x5200, RPM);
		//Dwin_Send_Int(0x6400, RPM);
		old_RPM = RPM;
	}
	/*    if (speed != old_speed) {
	 Dwin_Send_Int(0x5400, speed);
	 Dwin_Send_Int(0x6400, speed);
	 Dwin_Send_Int(0x7400, speed);
	 Dwin_Send_Int(0x8400, speed);
	 old_speed = speed;
	 }

	 // --------------------------------------------------------
	 // PAGE 3 (HÜCRE VOLTAJLARI - DÜZELTİLDİ)
	 // --------------------------------------------------------

	 // Adres Listesi (Sizin ekran tasarımınıza göre birebir adresler)
	 // v1..v9 arası 7010..7090, v10..v14 arası 7100..7140
	 uint16_t cell_addresses[14] = {
	 0x7010, 0x7020, 0x7030, 0x7040, 0x7050,
	 0x7060, 0x7070, 0x7080, 0x7090,
	 0x7100, // v10 (Burada atlama var)
	 0x7110, 0x7120, 0x7130, 0x7140
	 };

	 for (int i = 0; i < 14; i++) {
	 Dwin_Send_Float(cell_addresses[i], cell_voltages[i+3]);
	 old_cells[i] = cell_voltages[i+3];
	 }

	 // Min/Max Cell
	 static float old_min_cell = -1, old_max_cell = -1;
	 Dwin_Send_Float(0x7300, min_cell_v);
	 old_min_cell = min_cell_v;

	 Dwin_Send_Float(0x7320, max_cell_v);
	 old_max_cell = max_cell_v;

	 // --------------------------------------------------------
	 // PAGE 4 (SICAKLIKLAR)
	 // --------------------------------------------------------
	 Dwin_Send_Int(0x8001, ntc_temperatures[0]);
	 Dwin_Send_Int(0x8002, ntc_temperatures[1]);*/
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
	packet.speed = speed;
	packet.tubeTemp = 30;
	packet.batteryTemp = max_temp;
	packet.voltage = bms_voltage;
	packet.charge = bms_soc;

	packet.timeHigh = (currentTime >> 16);
	packet.timeMid = (currentTime >> 8);
	packet.timeLow = currentTime;

	packet.crc = CRC_Calculate((uint8_t*) &packet, sizeof(TelemetryPacket) - 1);
}

void sendRequestBMS() {
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

void receiveSurucu(void) {
	MotorControllerTxInformation[0] = RxHeader.StdId; // Sadece standart ID ise kaydet
	MotorControllerTxInformation[1] = RxHeader.DLC;

	if (RxHeader.StdId == 0x431) {
		for (int i = 0; i < 8; i++) {
			surucu_data[i] = RxData[i];
		}

		RPM = (surucu_data[0] << 8) | surucu_data[1];
		speed = (uint8_t) (RPM * 3.14159 * 0.55 * 60 / 1000);
		ref = surucu_data[2];
		faults = (surucu_data[3] << 8) | surucu_data[4];

		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
	}
}

void receiveBMS(void) {
	// Kuyrukta mesaj varsa en eski mesajı okuyup "RxData" dizisine ve "RxHeader" yapısına aktar.
	// Bu fonksiyon mesajı okur ve FIFO0'ı bir sonraki mesaj için boşaltır.
	// Sadece Extended ID (BMS) mesajlarını kabul et

	if (RxHeader.ExtId == 0x18904001) {
		BMSTxInformation[0] = RxHeader.ExtId; // Extended ID ise ExtId'yi kaydet
		BMSTxInformation[1] = RxHeader.DLC; // Mesajın kaç byte olduğunu (Data Length Code) kaydet.

		// TOPLAM VOLTAJ: Byte 0 ve 1 birleştirilir, 0.1 ile çarpılır
		v_raw = (RxData[0] << 8) | RxData[1];
		bms_voltage = v_raw * 0.1f;
		//bms_voltage = (uint8_t) (v_raw * 0.1f) ;

		// AKIM: Byte 4 ve 5 birleştirilir. 30000 offset çıkarılır ve 0.1 ile çarpılır
		i_raw = (RxData[4] << 8) | RxData[5];
		bms_current = (int16_t) ((i_raw - 30000) * 0.1f);
		//bms_current = (uint8_t)((i_raw - 30000) * 0.1f);

		// SOC: Byte 6 ve 7 birleştirilir, 0.1 ile çarpılır
		s_raw = (RxData[6] << 8) | RxData[7];
		//bms_soc = (uint8_t)(s_raw * 0.1f);

		// Veri geldiğini anlamak için LED'i yak söndür
		HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
	}

	else if (RxHeader.ExtId == 0x18954001) { // Hücre gerilimleri paketi [cite: 23, 26]
		// Byte 0: Frame numarasını verir (0'dan başlar)
		uint8_t frame_no = RxData[0];

		if (frame_no < 16) { // Maksimum 16 frame gelebilir
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
			bms_soc = (uint8_t) (100 * toplam_hucre_voltaji / 58.8f);
		}
		HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);
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

			min_temp =
					(ntc_temperatures[0] < ntc_temperatures[1]) ?
							(uint8_t) ntc_temperatures[0] :
							(uint8_t) ntc_temperatures[1];
			max_temp =
					(ntc_temperatures[0] > ntc_temperatures[1]) ?
							(uint8_t) ntc_temperatures[0] :
							(uint8_t) ntc_temperatures[1];
		}
		HAL_GPIO_TogglePin(LED5_GPIO_Port, LED5_Pin);
	}
}

void recieveDireksiyon(void){

	if (RxHeader.StdId == 0x46) {
		DireksiyonTxInformation[0] = RxHeader.StdId; // Sadece standart ID ise kaydet
		DireksiyonTxInformation[1] = RxHeader.DLC;

		buton_sinyal_right = RxData[0];
		buton_sinyal_left= RxData[1];
		buton_sinyal_hazard = RxData[2];
		buton_sinyal_page = RxData[3];
		//buton_sinyal_korna = RxData[5];
		memcpy(buton_data, RxData, 8);
	}
}

void voltaCAN() {
	// Önce kutuda mesaj var mı diye bak, varsa İÇERİ GİR ve SADECE BİR KERE OKU
	while (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0) {
		if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData)
				== HAL_OK) {
			if (RxHeader.IDE == CAN_ID_STD) {
				receiveSurucu();
			} else if (RxHeader.IDE == CAN_ID_EXT) {
				receiveBMS();
			}
		}
	}
}

void System_On() {
//	Display_Startup_Animation();
	HAL_GPIO_WritePin(YSB_GPIO_Port, YSB_Pin, 1);		// Motor Sürücüyü aç
	HAL_GPIO_WritePin(FLASOR_GPIO_Port, FLASOR_Pin, 1);
//	HAL_GPIO_WritePin(KORNA_GPIO_Port, KORNA_Pin, 1);
	HAL_GPIO_WritePin(SISTEM_GPIO_Port, SISTEM_Pin, 1);
	HAL_GPIO_WritePin(YEDEK_GPIO_Port, YEDEK_Pin, 1);
}

void Enable_Multiplexer(void) {
	HAL_GPIO_WritePin(MUX_EN_GPIO_Port, MUX_EN_Pin, GPIO_PIN_RESET);
}

static void MUX_Select(uint8_t channel) {
	HAL_GPIO_WritePin(MUX_S0_GPIO_Port, MUX_S0_Pin, (channel & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MUX_S1_GPIO_Port, MUX_S1_Pin, (channel & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MUX_S2_GPIO_Port, MUX_S2_Pin, (channel & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

uint16_t Read_ADC(ADC_HandleTypeDef hadc, uint32_t channel) {
	ADC_ChannelConfTypeDef sConfig = { 0 };
	sConfig.Channel = channel;
	sConfig.Rank = 1;
	HAL_ADC_ConfigChannel(&hadc, &sConfig);

    HAL_ADC_Start(&hadc);

    HAL_StatusTypeDef result = HAL_ADC_PollForConversion(&hadc, 10); // 10ms timeout

    if (result != HAL_OK) {
        if (hadc.Instance == ADC2) {
            HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, GPIO_PIN_SET); // H2 sensörü
        } else {
            HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, GPIO_PIN_SET); // Driver/Motor temp
        }
        HAL_ADC_Stop(&hadc);
        return 0xFFFF;
    }

    HAL_ADC_Stop(&hadc);
    return HAL_ADC_GetValue(&hadc);
}

uint16_t Driver_Temperature() {
	Enable_Multiplexer();
	MUX_Select(2);
	uint32_t raw = Read_ADC(hadc1, ADC_CHANNEL_9);
	float voltage = ((float) raw / 4095.0f) * VCC;

	// Pull-down devresi için NTC direncini hesapla
	float r_ntc = ((VCC * R_PULLDOWN) / voltage) - R_PULLDOWN;

	// Beta formülü ile sıcaklığı hesapla
	float tempK = 1.0f / ((1.0f / T0) + (1.0f / BETA) * logf(r_ntc / R0));
	return (uint16_t) (tempK - 273.15f);
}

uint16_t Motor_Temperature() {
	Enable_Multiplexer();
	MUX_Select(1);
	uint32_t raw = Read_ADC(hadc1, ADC_CHANNEL_9);
	float voltage = ((float) raw / 4095.0f) * VCC;

	// Pull-down devresi için NTC direncini hesapla
	float r_ntc = ((VCC * R_PULLDOWN) / voltage) - R_PULLDOWN;

	// Beta formülü ile sıcaklığı hesapla
	float tempK = 1.0f / ((1.0f / T0) + (1.0f / BETA) * logf(r_ntc / R0));
	return (uint16_t) (tempK - 273.15f);
}

void BMS_Horn_Control() {
//    a_current_time = HAL_GetTick();  // Zamanı baştan alıyoruz
	// Batarya sıcaklığı 55 ile 70 derece arasında ise korna çal
	if (/*stop_state == false &&**/(max_temp > 55 && max_temp < 70)) {
		HAL_GPIO_WritePin(KORNA_GPIO_Port, KORNA_Pin, 1);  // Korna çal
		HAL_GPIO_WritePin(FLASOR_GPIO_Port, FLASOR_Pin, 1);
	} else if (max_temp < 55) {
		HAL_GPIO_WritePin(KORNA_GPIO_Port, KORNA_Pin, 0);  // Korna çal
		HAL_GPIO_WritePin(FLASOR_GPIO_Port, FLASOR_Pin, 0);
	}

	// Süre 3000ms'yi geçerse durdur
//    if (horn_state && (current_time - horn_time) > 3000) {
//    	horn_state = false;
//        stop_state = true;
//        HAL_GPIO_WritePin(KORNA_GPIO_Port, KORNA_Pin, 0);  // Korna durdur
//        HAL_GPIO_WritePin(FLASOR_GPIO_Port, FLASOR_Pin, 0);
//        horn_time = current_time;  // Korna bitiş zamanını ayarla
//        stop_time = HAL_GetTick();  // Durma zamanını ayarla
//    }

	// Batarya sıcaklığı 70 dereceden yüksekse sistemi kapat
	else if (max_temp >= 70) {
		HAL_GPIO_WritePin(SISTEM_GPIO_Port, SISTEM_Pin, 1);  // Sistemi kapat
	}
}

void H2() {
	h2_adc_value = Read_ADC(hadc2, ADC_CHANNEL_4); // ADC’nin okuduğu dijital değeri (ölçülen analog sinyalin dijital karşılığını) al.
	if ((h2_adc_value > 1500) /*&& (tube_temp > 55)*/) {
		HAL_GPIO_WritePin(SELENOID_VALF_GPIO_Port, SELENOID_VALF_Pin, 0);
		HAL_GPIO_WritePin(KORNA_GPIO_Port, KORNA_Pin, 1);
		HAL_GPIO_WritePin(FLASOR_GPIO_Port, FLASOR_Pin, 1);
	} else {
		HAL_GPIO_WritePin(SELENOID_VALF_GPIO_Port, SELENOID_VALF_Pin, 1);
		HAL_GPIO_WritePin(KORNA_GPIO_Port, KORNA_Pin, 0);
		HAL_GPIO_WritePin(FLASOR_GPIO_Port, FLASOR_Pin, 0);
	}
}

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
	MX_ADC1_Init();
	MX_DAC_Init();
	MX_UART4_Init();
	MX_UART5_Init();
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	MX_USART3_UART_Init();
	MX_USART6_UART_Init();
	MX_ADC2_Init();
	MX_SPI2_Init();
	MX_CAN1_Init();
	MX_I2C2_Init();
	MX_TIM2_Init();
	/* USER CODE BEGIN 2 */
	System_On();

	//Display_Startup_Animation();
	HAL_CAN_Start(&hcan1);
	LoraMode();
	TelemetryStart();
//  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

	/* USER CODE END 2 */

	/* Init scheduler */
	osKernelInitialize();

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
	/* creation of TelemetryTask */
	TelemetryTaskHandle = osThreadNew(vTelemetryTask, NULL,
			&TelemetryTask_attributes);

	/* creation of SafetyTask */
	SafetyTaskHandle = osThreadNew(vSafetyTask, NULL, &SafetyTask_attributes);

	/* creation of CanTask */
	CanTaskHandle = osThreadNew(vCanTask, NULL, &CanTask_attributes);

	/* creation of DisplayTask */
	DisplayTaskHandle = osThreadNew(vDisplayTask, NULL,
			&DisplayTask_attributes);

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

		 /**LoRa_Run(packet.speed,
		 packet.tubeTemp,
		 packet.batteryTemp,
		 packet.voltage,
		 packet.charge,
		 packet.timeHigh,
		 packet.timeMid,
		 packet.timeLow);*/
		/*	  LoRa_PushPacket(&packet);
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
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

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
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 168;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = ENABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_8;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
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
static void MX_ADC2_Init(void) {

	/* USER CODE BEGIN ADC2_Init 0 */

	/* USER CODE END ADC2_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

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
	if (HAL_ADC_Init(&hadc2) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_4;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
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
static void MX_CAN1_Init(void) {

	/* USER CODE BEGIN CAN1_Init 0 */

	/* USER CODE END CAN1_Init 0 */

	/* USER CODE BEGIN CAN1_Init 1 */

	/* USER CODE END CAN1_Init 1 */
	hcan1.Instance = CAN1;
	hcan1.Init.Prescaler = 12;
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
	if (HAL_CAN_Init(&hcan1) != HAL_OK) {
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
static void MX_DAC_Init(void) {

	/* USER CODE BEGIN DAC_Init 0 */

	/* USER CODE END DAC_Init 0 */

	DAC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN DAC_Init 1 */

	/* USER CODE END DAC_Init 1 */

	/** DAC Initialization
	 */
	hdac.Instance = DAC;
	if (HAL_DAC_Init(&hdac) != HAL_OK) {
		Error_Handler();
	}

	/** DAC channel OUT2 config
	 */
	sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
	sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
	if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN DAC_Init 2 */

	/* USER CODE END DAC_Init 2 */

}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void) {

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
	if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C2_Init 2 */

	/* USER CODE END I2C2_Init 2 */

}

/**
 * @brief SPI2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI2_Init(void) {

	/* USER CODE BEGIN SPI2_Init 0 */

	/* USER CODE END SPI2_Init 0 */

	/* USER CODE BEGIN SPI2_Init 1 */

	/* USER CODE END SPI2_Init 1 */
	/* SPI2 parameter configuration*/
	hspi2.Instance = SPI2;
	hspi2.Init.Mode = SPI_MODE_MASTER;
	hspi2.Init.Direction = SPI_DIRECTION_2LINES;
	hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi2.Init.NSS = SPI_NSS_SOFT;
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi2.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI2_Init 2 */

	/* USER CODE END SPI2_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 4294967295;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */
	HAL_TIM_MspPostInit(&htim2);

}

/**
 * @brief UART4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_UART4_Init(void) {

	/* USER CODE BEGIN UART4_Init 0 */

	/* USER CODE END UART4_Init 0 */

	/* USER CODE BEGIN UART4_Init 1 */

	/* USER CODE END UART4_Init 1 */
	huart4.Instance = UART4;
	huart4.Init.BaudRate = 115200;
	huart4.Init.WordLength = UART_WORDLENGTH_8B;
	huart4.Init.StopBits = UART_STOPBITS_1;
	huart4.Init.Parity = UART_PARITY_NONE;
	huart4.Init.Mode = UART_MODE_TX_RX;
	huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart4.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart4) != HAL_OK) {
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
static void MX_UART5_Init(void) {

	/* USER CODE BEGIN UART5_Init 0 */

	/* USER CODE END UART5_Init 0 */

	/* USER CODE BEGIN UART5_Init 1 */

	/* USER CODE END UART5_Init 1 */
	huart5.Instance = UART5;
	huart5.Init.BaudRate = 9600;
	huart5.Init.WordLength = UART_WORDLENGTH_8B;
	huart5.Init.StopBits = UART_STOPBITS_1;
	huart5.Init.Parity = UART_PARITY_NONE;
	huart5.Init.Mode = UART_MODE_TX_RX;
	huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart5.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart5) != HAL_OK) {
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
static void MX_USART1_UART_Init(void) {

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
	if (HAL_UART_Init(&huart1) != HAL_OK) {
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
	if (HAL_UART_Init(&huart2) != HAL_OK) {
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
static void MX_USART3_UART_Init(void) {

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
	if (HAL_UART_Init(&huart3) != HAL_OK) {
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
static void MX_USART6_UART_Init(void) {

	/* USER CODE BEGIN USART6_Init 0 */

	/* USER CODE END USART6_Init 0 */

	/* USER CODE BEGIN USART6_Init 1 */

	/* USER CODE END USART6_Init 1 */
	huart6.Instance = USART6;
	huart6.Init.BaudRate = 9600;
	huart6.Init.WordLength = UART_WORDLENGTH_8B;
	huart6.Init.StopBits = UART_STOPBITS_1;
	huart6.Init.Parity = UART_PARITY_NONE;
	huart6.Init.Mode = UART_MODE_TX_RX;
	huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart6.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart6) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART6_Init 2 */

	/* USER CODE END USART6_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOE,
			FLASOR_Pin | KORNA_Pin | SELENOID_VALF_Pin | YSB_Pin | SISTEM_Pin
					| Y_MUX_S0_Pin | Y_MUX_S1_Pin | Y_MUX_S2_Pin | Y_MUX_EN_Pin
					| LED2_Pin | LED1_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, YEDEK_Pin | MUX_S2_Pin | MUX_EN_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, MUX_S0_Pin | MUX_S1_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(RF_PARAMETRE_GPIO_Port, RF_PARAMETRE_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB,
	LED8_Pin | LED7_Pin | LED6_Pin | LED5_Pin | LED4_Pin | LED3_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pins : FLASOR_Pin KORNA_Pin SELENOID_VALF_Pin YSB_Pin
	 SISTEM_Pin Y_MUX_S0_Pin Y_MUX_S1_Pin Y_MUX_S2_Pin
	 Y_MUX_EN_Pin LED2_Pin LED1_Pin */
	GPIO_InitStruct.Pin = FLASOR_Pin | KORNA_Pin | SELENOID_VALF_Pin | YSB_Pin
			| SISTEM_Pin | Y_MUX_S0_Pin | Y_MUX_S1_Pin | Y_MUX_S2_Pin
			| Y_MUX_EN_Pin | LED2_Pin | LED1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pins : YEDEK_Pin MUX_S2_Pin MUX_EN_Pin */
	GPIO_InitStruct.Pin = YEDEK_Pin | MUX_S2_Pin | MUX_EN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : FORWARD_Pin BACK_Pin */
	GPIO_InitStruct.Pin = FORWARD_Pin | BACK_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : MUX_S0_Pin MUX_S1_Pin */
	GPIO_InitStruct.Pin = MUX_S0_Pin | MUX_S1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : RF_PARAMETRE_Pin */
	GPIO_InitStruct.Pin = RF_PARAMETRE_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(RF_PARAMETRE_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LED8_Pin LED7_Pin LED6_Pin LED5_Pin
	 LED4_Pin LED3_Pin */
	GPIO_InitStruct.Pin = LED8_Pin | LED7_Pin | LED6_Pin | LED5_Pin | LED4_Pin
			| LED3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/*void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
 if (hcan->Instance == CAN1) {
 if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
 // Bilgileri doldur
 TxInformation[0] = RxHeader.StdId;
 TxInformation[1] = RxHeader.ExtId;
 TxInformation[2] = RxHeader.IDE;
 TxInformation[3] = RxHeader.RTR;
 TxInformation[4] = RxHeader.DLC;

 // Veriyi surucu dizisine kopyala
 for (uint8_t i = 0; i < RxHeader.DLC; i++) {
 surucu[i] = RxData[i];
 }

 HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
 }
 }
 }*/
/* USER CODE END 4 */

/* USER CODE BEGIN Header_vTelemetryTask */
/**
 * @brief  Function implementing the TelemetryTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_vTelemetryTask */
void vTelemetryTask(void *argument) {
	/* USER CODE BEGIN 5 */
	/* Infinite loop */
	for (;;) {
		TelemetryData();          // Verileri paketler
		LoRa_PushPacket(&packet); // Kuyruğa ekler
		LoRa_Run();         // ACK / Timeout / Gönderim durumlarını kontrol eder

		osDelay(600);
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
void vSafetyTask(void *argument) {
	/* USER CODE BEGIN vSafetyTask */
	/* Infinite loop */

	for (;;) {
		H2();                 // Hidrojen sensörü ve vana kontrolü
		BMS_Horn_Control();   // Batarya sıcaklık ve acil durum korna kontrolü

		// Periyodik bekleme (İşlemciyi diğer görevlere bırakır)
		osDelay(50);
	}
	/* USER CODE END vSafetyTask */
}

/* USER CODE BEGIN Header_vCanTask */
/**
 * @brief Function implementing the CanTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_vCanTask */
void vCanTask(void *argument) {
	/* USER CODE BEGIN vCanTask */
	/* Infinite loop */
	for (;;) {
		sendRequestBMS(); // Periyodik BMS sorguları (Zamanlama takipleri içindedir)
		voltaCAN();       // FIFO'daki CAN mesajlarını okur

		osDelay(20);          // 20 ms Görevi Uyut	  }
		/* USER CODE END vCanTask */
	}
}

/* USER CODE BEGIN Header_vDisplayTask */
/**
 * @brief Function implementing the DisplayTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_vDisplayTask */
void vDisplayTask(void *argument) {
	/* USER CODE BEGIN vDisplayTask */
	/* Infinite loop */
	for (;;) {
		Send_To_Display();
		osDelay(200);             // 200 ms Görevi Uyut		}
		/* USER CODE END vDisplayTask */
	}
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM1 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM1) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

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
