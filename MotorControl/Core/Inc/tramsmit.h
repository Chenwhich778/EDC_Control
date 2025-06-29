
#include "usart.h"

#define RX_BUFFER_SIZE 64   //接收字符串长度A
extern char uart_buffer[50];      // UART发送缓冲区
extern volatile char rx_buffer[RX_BUFFER_SIZE];
extern volatile uint8_t rx_index;
extern volatile uint8_t rx_ready;
extern char rx_char1;
extern char rx_char3;
extern char received_byte;

void Send_MultiData_FireWater(float speed_rpm, float pidsetpoint,  float speed_rpm_,  float pidsetpoint_);
void Parse_Data();