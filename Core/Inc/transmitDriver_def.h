#ifndef TRANSMITDRIVER_DEF_H
#define TRANSMITDRIVER_DEF_H

#include <stdint.h>

#define VCU_SOF  0xA55A  // Start of Frame (Paket Başlığı)

// Bayt hizalamasını sabitlemek için packed kullanıyoruz
typedef struct __attribute__((packed)) {
    uint16_t sof;        // Paket Başlangıç İmzası (0xA55A)
    uint8_t driver_reset;
    uint8_t engine_off;
    uint8_t crc;        // Basit Checksum / CRC
} VcuToDriver_Data_t;

#endif
