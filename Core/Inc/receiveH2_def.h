#ifndef RECEIVEH2_DEF_H
#define RECEIVEH2_DEF_H

#include <stdint.h>

#define H2_SOF  0x55AA  // Start of Frame (Paket Başlığı)

// Bayt hizalamasını sabitlemek için packed kullanıyoruz
typedef struct __attribute__((packed)) {
	uint16_t sof;			// Başlık: 0x55AA
	int16_t temperature;	// Sıcaklık (x100)
	int16_t current;		// Akım (x100)
	uint8_t crc;		// Checksum baytı
} H2ToVcu_Data_t;

#endif
