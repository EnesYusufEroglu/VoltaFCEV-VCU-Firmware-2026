#ifndef RECEIVE_BUTTONS_DEF_H
#define RECEIVE_BUTTONS_DEF_H

#include <stdint.h>

#define BUTTONS_SOF 0xAA

typedef struct __attribute__((packed))
{
	    uint8_t sof;
	    uint8_t solSinyal;
	    uint8_t sagSinyal;
	    uint8_t dortlu;
	    uint8_t far;
	    uint8_t h2Arttir;
	    uint8_t h2Azalt;
	    uint8_t reset;
	    uint8_t surucuKapat;
	    uint8_t sayfaDegistir;
	    uint8_t crc;
} ButtonsData_t;

#endif
