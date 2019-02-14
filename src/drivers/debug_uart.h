
#ifndef _DEBUG_UART_H_INCLUDED_
#define _DEBUG_UART_H_INCLUDED_

void DebugUart_init(void);

void DebugUart_printf(const char *fmt, ...);
void DebugUart_flush(void);

#endif /* _DEBUG_UART_H_INCLUDED_ */
