#ifndef __USART_H
#define __USART_H

#define USART_FIFO_size  10

#define USART_FREE   0
#define USART_NOFREE 1

typedef struct {
	uint8_t data;
} USART_Queue;

void USART_init(void);

uint8_t USART_write(char);
void USART_putchar(char);
void USART_send(char *);

#endif
