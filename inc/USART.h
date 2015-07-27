#ifndef __USART_H
#define __USART_H

#define USART_FIFO_size  50

#define USART_FREE   0
#define USART_NOFREE 1

#define DEC          0
#define HEX          1
#define BIN          2

// Public functions
void _USART_init(void);
void _USART_send(char *);
void _USART_write_buf(uint32_t DATA, uint8_t TYPE);
// End of public functions

#endif
