#ifndef __HALL_H
#define __HALL_H

typedef struct {
	uint32_t Time;
	uint8_t  SequenceCnt;
	uint8_t  HuntTime;
} HALL;

extern volatile HALL HALL_Data;

void HALL_init(void);

#endif
