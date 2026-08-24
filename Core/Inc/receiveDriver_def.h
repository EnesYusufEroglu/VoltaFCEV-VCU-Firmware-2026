#ifndef RECEIVEDRIVER_DEF_H
#define RECEIVEDRIVER_DEF_H

#include <stdint.h>

#define MOTOR_DRIVER_SOF  0xA55A  // Start of Frame (Paket Başlığı)

// Bayt hizalamasını sabitlemek için packed kullanıyoruz
typedef struct __attribute__((packed)) {
    uint16_t sof;        // Paket Başlangıç İmzası (0xA55A)
    uint16_t rpm;        // Motor Motor Devri
    uint16_t reference;  // Gaz Referansı
    uint16_t faults;     // Hata Kodları (Bitmask)
    uint8_t  crc;        // Basit Checksum / CRC
} MotorDriverData_t;

#endif
