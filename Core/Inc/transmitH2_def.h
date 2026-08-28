#ifndef TRANSMITH2_DEF_H
#define TRANSMITH2_DEF_H

#include <stdint.h>

#define H2_SOF  0x55AA  // Start of Frame (Paket Başlığı)

// Bayt hizalamasını sabitlemek için packed kullanıyoruz
typedef struct __attribute__((packed)) {
    uint16_t sof;		// Paket Başlangıç İmzası
    uint8_t h2Arttir;
    uint8_t h2Azalt;
    uint8_t crc;		// Checksum baytı
} VcuToH2_Data_t;

#endif
