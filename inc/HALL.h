#ifndef __HALL_H
#define __HALL_H

typedef struct {
	uint32_t Time;
	uint32_t TimeOut;
	uint8_t  SequenceCnt;
	uint8_t  HuntTime;
	uint8_t  Result;
} HALL;

extern volatile HALL HALL_Data;

void HALL_init(void);

#endif
