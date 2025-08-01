#include "car_bsp.h"

extern PID_Controller pid_left;
extern PID_Controller pid_right;
int x = 316;
int y = 266;

int R_pixel = 0;
int distance = 0;
bool Receive = false;


typedef struct {
    uint8_t buffers[2][256];  // 双缓冲区
    volatile uint8_t active_idx;     // 当前写入的缓冲区索引
    volatile uint16_t index;          // 当前写入位置
    volatile uint8_t frame_ready;    // 帧接收完成标志
} UART0_RxBuffer_t;

UART0_RxBuffer_t UART0_RxBuffer = {.active_idx = 0, .index = 0, .frame_ready = 0};

void usb_uart0_IRQ_init(void)
{
    NVIC_ClearPendingIRQ(USB_UART_0_INST_INT_IRQN);
    NVIC_EnableIRQ(USB_UART_0_INST_INT_IRQN);

}

uint8_t uart0_data=0;
void USB_UART_0_INST_IRQHandler(void)
{
    //如果产生了串口中断
    switch( DL_UART_getPendingInterrupt(USB_UART_0_INST) )
    {
        case DL_UART_IIDX_RX://如果是接收中断
            // 检查是否已接收到完整帧
            // 错误处理优先
           
            
            // 检查当前缓冲区是否可用
            if (UART0_RxBuffer.frame_ready) {
                // 丢弃数据直到当前帧被处理
                DL_UART_Main_receiveData(USB_UART_0_INST);
                break;
            }
            
            uint8_t data = DL_UART_Main_receiveData(USB_UART_0_INST);
            uint8_t *buf = UART0_RxBuffer.buffers[UART0_RxBuffer.active_idx];
            
            // 缓冲区溢出处理
            if (UART0_RxBuffer.index >= sizeof(UART0_RxBuffer.buffers[0])) {
                // 切换缓冲区并标记帧就绪（强制处理不完整帧）
                UART0_RxBuffer.active_idx ^= 1;  // 切换缓冲区
                UART0_RxBuffer.index = 0;
                UART0_RxBuffer.frame_ready = 1;
                buf = UART0_RxBuffer.buffers[UART0_RxBuffer.active_idx];
            }
            
            buf[UART0_RxBuffer.index++] = data;
            
            // 检测帧结束符
            if (data == '\n') {
                UART0_RxBuffer.frame_ready = 1;
            }
            break;
            
        default://其他的串口中断
            break;
    }
}

// 3. 改进帧处理函数
void UART0_ProcessFrame(void) {
    if (!UART0_RxBuffer.frame_ready) return;
    
    // 锁定当前缓冲区
    uint8_t process_idx = UART0_RxBuffer.active_idx ^ 1;  // 获取非活动缓冲区
    uint8_t *frame_data = UART0_RxBuffer.buffers[process_idx];
    uint16_t frame_len = UART0_RxBuffer.index;  // 保存处理时的长度
    
    // 重置接收状态（原子操作）
    __disable_irq();
    UART0_RxBuffer.frame_ready = 0;
    UART0_RxBuffer.active_idx ^= 1;  // 切换活动缓冲区
    UART0_RxBuffer.index = 0;
    __enable_irq();
    
    // 解析数据（使用独立缓冲区）
    char frame[128] = {0};
    uint16_t j = 0;
    for (uint16_t i = 0; i < frame_len && j < sizeof(frame)-1; i++) {
        if (frame_data[i] == '\n') break;
        if (frame_data[i] != '\r') frame[j++] = frame_data[i];
    }
    frame[j] = '\0';
    
    // // 解析有效帧
    // if (frame[0] == '!') {
    //     if (sscanf(&frame[1], "%d,%d", &x, &y) != 2) {
    //         x = 359;  // 解析失败时重置默认值
    //         y = 225;
    //     }
    //     if(x==0)
    //     {
    //         x = 359;  // 解析失败时重置默认值
    //         y = 225;
    //     }
    // }

    // if (sscanf(&frame[1], "%f,%f,%f", &pid_left.Kp, &pid_left.Ki,&pid_left.Kd) != 2) {

    // 解析有效帧
    if (frame[0] == '!') {
        if (sscanf(&frame[1], "%d,%d,%d,%d", &x, &y,&R_pixel,&distance) != 4) {
            x = 316;  // 解析失败时重置默认值
            y = 266;
        }
        if(x==0)
        {
            x = 316;  // 解析失败时重置默认值
            y = 266;
        }
        Receive = true;
    }
    // 非'!'帧不处理但不清空缓冲区

}
//重定向fputc函数
int fputc(int ch, FILE *stream)
{
    while( DL_UART_isBusy(USB_UART_0_INST) == true );
    DL_UART_Main_transmitData(USB_UART_0_INST, ch);
    return ch;
}

//重定向fputs函数
int fputs(const char* restrict s, FILE* restrict stream) {

    uint16_t char_len=0;
    while(*s!=0)
    {
        while( DL_UART_isBusy(USB_UART_0_INST) == true );
        DL_UART_Main_transmitData(USB_UART_0_INST, *s++);
        char_len++;
    }
    return char_len;
}
int puts(const char* _ptr)
{
 return 0;
}


void delay_ms(uint32_t ms)
{
	delay_cycles(32000 * ms);
}
void delay_us(uint32_t us)
{
	delay_cycles(32 * us);
}
