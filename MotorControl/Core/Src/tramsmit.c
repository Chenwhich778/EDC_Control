
#include "tramsmit.h"
#include "stdio.h"
#include "motor_control.h"
#include "string.h"
#include "stdlib.h"
#include "i2c.h"

char uart_buffer[50];      // UART发送缓冲区
volatile char rx_buffer[RX_BUFFER_SIZE];
volatile uint8_t rx_index = 0;
volatile uint8_t rx_ready = 0;
char rx_char1;
char rx_char3;
char received_byte;

void Send_MultiData_FireWater(float speed_rpm, float pidsetpoint,  float speed_rpm_,  float pidsetpoint_) {
    char buffer[64];
    int length = snprintf(buffer, sizeof(buffer), "%.2f, %.2f,%.2f,%.2f\r\n", speed_rpm, pidsetpoint, speed_rpm_, pidsetpoint_);

    // 通过 USART 发送字符串
    if (HAL_UART_Transmit(&huart1, (uint8_t*)buffer, length, 100) != HAL_OK) {
        Error_Handler();
    }
}

void Parse_Data() {
    char *token;
    float values[3];
    uint8_t count = 0;

    token = strtok((char *)rx_buffer, ",");
    while (token != NULL && count < 3) {
        values[count++] = atof(token);
        token = strtok(NULL, ",");
    }

    if (count == 3) { // 确保接收到三个值
       
    }
	}

// USART接收中断回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) { // 确保是目标USART
        // 检测结束符（回车或换行）
        if (rx_char1 == '\r' || rx_char1 == '\n') {
            if (rx_index > 0) { // 非空数据
                rx_buffer[rx_index] = '\0'; // 终止字符串
                rx_ready = 1; // 设置标志位
                rx_index = 0; // 重置索引
            }
        } else {
            // 将字符存入缓冲区，防止溢出
            if (rx_index < RX_BUFFER_SIZE - 1) {
                rx_buffer[rx_index++] = rx_char1;
            } else {
                // 缓冲区满，重置索引（可选：错误处理）
                rx_index = 0;
            }
        }
        // 重新启动接收中断
         HAL_UART_Receive_IT(&huart1, (uint8_t *)&rx_char1, 1);
    }
		else if(huart->Instance== USART3){
			
			HAL_UART_Receive_IT(&huart3, (uint8_t*)&rx_char3, 1);  //启动 USART 接收中断
		}
		else if(huart->Instance == USART6){
			HAL_UART_Receive_IT(&huart6, (uint8_t*)&received_byte, 1);  //启动 USART 接收中断
		}
}
