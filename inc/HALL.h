#ifndef __HALL_H
#define __HALL_H

typedef struct {
	uint32_t Time;
	uint8_t  SequenceCnt;
} HALL;

void HALL_init(void);

#endif
