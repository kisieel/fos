#ifndef __USART_H
#define __USART_H

#define DEC          0
#define HEX          1
#define BIN          2

// Public functions
extern void USART_init(void);
extern void USART_putchar(char ch);
extern void USART_send(char *);
extern void USART_write_buf(uint32_t DATA, uint8_t TYPE);
// End of public functions

#endif
