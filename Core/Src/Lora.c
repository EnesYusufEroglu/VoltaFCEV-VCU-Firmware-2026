#include "LoRa.h"
#include "string.h"
#include "cmsis_os.h"  // RTOS ve Semafor fonksiyonları için gerekli

extern UART_HandleTypeDef huart3;
extern osSemaphoreId_t uart3TxSemaphoreHandle; // main.c'den gelen semafor

uint8_t receiveBuffer;
TelemetryPacket telemetryQueue[QUEUE_SIZE];
uint8_t queueFront = 0;
uint8_t queueRear = 0;
uint8_t queueCount = 0;
uint32_t currentTime;
volatile bool waitingAck = false;
uint32_t lastSendTime = 0;
volatile uint8_t  receivedCmd = 0;       // UART'tan gelen 1 baytlık komutu tutar
volatile bool     cmdReceivedFlag = false; // Komut geldi mi gelmedi mi (0 veya 1) bayrağı

uint8_t CRC_Calculate(uint8_t *data, uint8_t length)
{
	uint8_t crc = 0;
	for(uint8_t i = 0; i < length; i++)
    {
        crc ^= data[i];
    }
    return crc;
}

void LoRa_Init(void)
{
    HAL_UART_Receive_IT(&huart3, &receiveBuffer, 1);
}

void LoRa_PushPacket(TelemetryPacket *packet)
{
    if(queueCount >= QUEUE_SIZE)
    {
        return;
    }
    telemetryQueue[queueRear] = *packet;
    queueRear++;
    if(queueRear >= QUEUE_SIZE)
    {
        queueRear = 0;
    }
    queueCount++;
}

void LoRa_RemovePacket(void)
{
    if(queueCount == 0)
    {
        return;
    }
    queueFront++;
    if(queueFront >= QUEUE_SIZE)
    {
        queueFront = 0;
    }
    queueCount--;
}

void LoRa_SendNextPacket(void)
{
    if(queueCount == 0)
    {
        waitingAck = false;
        return;
    }

    if(waitingAck)
    {
        return; // Önceki paket ACK bekliyor, gönderme
    }

    if(huart3.gState != HAL_UART_STATE_READY)
    {
        return; // UART hazır değil, sonra dene
    }

    uint8_t txBuffer[3 + sizeof(TelemetryPacket)];
	txBuffer[0] = 0x07; // Hedef ADDH
	txBuffer[1] = 0x60; // Hedef ADDL
	txBuffer[2] = 0x1F; // Hedef Kanal (Channel)
	memcpy(&txBuffer[3], &telemetryQueue[queueFront], sizeof(TelemetryPacket));

	if (HAL_UART_Transmit_IT(&huart3, txBuffer, sizeof(txBuffer)) == HAL_OK) {
		// İletim süresince görevi (Task) beklemeye alıyoruz ki CPU serbest kalsın
		if (osSemaphoreAcquire(uart3TxSemaphoreHandle, 100) == osOK) {
			waitingAck = true;
			lastSendTime = HAL_GetTick();
		}
	}
}

void LoRa_ResendPacket(void)
{
    if(queueCount == 0)
    {
        waitingAck = false;
        return;
    }

    uint8_t txBuffer[3 + sizeof(TelemetryPacket)];
	txBuffer[0] = 0x07; // Hedef ADDH
	txBuffer[1] = 0x60; // Hedef ADDL
	txBuffer[2] = 0x1F; // Hedef Kanal
	memcpy(&txBuffer[3], &telemetryQueue[queueFront], sizeof(TelemetryPacket));

	// Bloking transmit YERİNE, IT ve Semafor kullanıyoruz
	if (HAL_UART_Transmit_IT(&huart3, txBuffer, sizeof(txBuffer)) == HAL_OK) {
		// CPU'yu bloklamadan iletimin (Tx) bitmesini bekliyoruz (Maks 600ms)
		if (osSemaphoreAcquire(uart3TxSemaphoreHandle, 600) == osOK) {
			waitingAck = true;
			lastSendTime = HAL_GetTick();
		}
	}
}

void LoRa_ProcessCommand(uint8_t command)
{
    switch(command)
    {
        case ACK_SUCCESS:
            LoRa_RemovePacket();
            waitingAck = false;
            // Sonraki LoRa_Run() çağrısında sıradaki paket gönderilecek
            break;

        case NACK_ERROR:
            // Aynı paketi tekrar gönder
            LoRa_ResendPacket();
            break;

        default:
            break;
    }
}

void LoRa_CheckTimeout(void)
{
    if(!waitingAck)
    {
        return;
    }

    if(queueCount == 0)
    {
        waitingAck = false;
        return;
    }

    if(HAL_GetTick() - lastSendTime > 600)
    {
        LoRa_ResendPacket();
        lastSendTime = HAL_GetTick();
    }
}

void LoRa_Run(/*uint8_t speed, uint8_t tubeTemp, uint8_t batteryTemp,
              uint8_t voltage, uint8_t charge, uint8_t timeHigh,
              uint8_t timeMid, uint8_t timeLow*/)
{
    /*static uint32_t previousPacketTime = 0;
    currentTime = HAL_GetTick();*/

    // Kesme "veri geldi" dediği an ana döngü sırası içinde işleniyor
    if (cmdReceivedFlag) {
		cmdReceivedFlag = false;          // Bayrağı indir
		LoRa_ProcessCommand(receivedCmd); // Fonksiyon burada güvenle çalışıyor
	}

    /*// Her 1000ms'de yeni paket oluştur
    if(currentTime - previousPacketTime >= 1000)
    {
        TelemetryPacket packet;
        packet.speed = speed;
        packet.tubeTemp = tubeTemp;
        packet.batteryTemp = batteryTemp;
        packet.voltage = voltage;
        packet.charge = charge;

        packet.timeHigh = (currentTime >> 16);
        packet.timeMid = (currentTime >> 8);
        packet.timeLow = currentTime;

        packet.crc = CRC_Calculate((uint8_t*)&packet, sizeof(TelemetryPacket) - 1);
        LoRa_PushPacket(&packet);
        previousPacketTime = currentTime;
    }*/

    // HER İTERASYONDA: ACK beklemiyorsa ve kuyrukta veri varsa gönder
    if(!waitingAck && queueCount > 0)
    {
        LoRa_SendNextPacket();
    }

    // Timeout kontrolü
    LoRa_CheckTimeout();
}
