#ifndef __USART_H
#define __USART_H

#define USART_FIFO_size  50

#define USART_FREE   0
#define USART_NOFREE 1

// Public functions
void _USART_init(void);
void _USART_send(char *);
// End of public functions

#endif
