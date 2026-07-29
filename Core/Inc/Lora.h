/*
 * Lora.h
 *
 *  Created on: May 19, 2026
 *      Author: Ayshe
 */

#ifndef INC_LORA_H_
#define INC_LORA_H_


#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#pragma pack(push,1)
typedef struct
{
    uint8_t speed;
    uint8_t tubeTemp;
    uint8_t batteryTemp;
    float voltage;
    uint8_t charge;
    uint8_t timeHigh;
    uint8_t timeMid;
    uint8_t timeLow;
    uint8_t crc;

}TelemetryPacket;
#pragma pack(pop) // byte byte diz boşluk bırakma

// Fonksiyon prototipleri
void LoRa_ProcessCommand(uint8_t command);
void LoRa_Run(void/*uint8_t speed, uint8_t tubeTemp ,uint8_t batteryTemp,uint8_t voltage ,uint8_t charge, uint8_t timeHigh,uint8_t timeMid,uint8_t timeLow*/);
void LoRa_CheckTimeout(void);
void LoRa_ResendPacket(void);
void LoRa_SendNextPacket(void);
void LoRa_RemovePacket(void);
void LoRa_Init(void);
void LoRa_PushPacket(TelemetryPacket *packet);
uint8_t CRC_Calculate(uint8_t *data, uint8_t length);

extern uint32_t currentTime;
extern uint8_t receiveBuffer;
extern volatile uint8_t receivedCmd;
extern volatile bool cmdReceivedFlag;


// Durum kodları
//#define DATA_REQUEST   0x01
#define ACK_SUCCESS    0x02
#define NACK_ERROR     0x03
#define LINK_TIMEOUT   0x04


#define QUEUE_SIZE 100


#endif /* INC_LORA_H_ */
