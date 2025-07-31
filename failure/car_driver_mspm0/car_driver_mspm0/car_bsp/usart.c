#include "usart.h"

void uart_send(UART_Regs *uart, uint8_t* data, size_t len) {
    for(size_t i = 0; i < len; i++) {
        while(DL_UART_isBusy(uart));          // 等待UART空闲
        DL_UART_transmitData(uart, data[i]);  // 发送单字节
    }
}

void uart1_send(uint8_t* data, size_t len){
    uart_send(UART1, data, len);
}



void uart2_send(uint8_t* data, size_t len){
    uart_send(UART2, data, len);
}

























