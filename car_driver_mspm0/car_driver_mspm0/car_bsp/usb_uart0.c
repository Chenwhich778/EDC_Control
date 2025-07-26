#include "car_bsp.h"


int x = 0;
int y = 0;
// 修改后的 UART0 接收缓冲区结构（使用普通缓冲区）
typedef struct {
    uint8_t buffer[128];  // 普通缓冲区
    uint16_t index;        // 当前写入位置
    uint8_t frame_ready;  // 帧接收完成标志
} UART0_RxBuffer_t;

UART0_RxBuffer_t UART0_RxBuffer = {0};  // 初始化缓冲区

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
            if (UART0_RxBuffer.frame_ready) {
                // 丢弃新数据直到当前帧被处理
                uint8_t dummy = DL_UART_Main_receiveData(USB_UART_0_INST);
                break;
            }
            
            uint8_t data = DL_UART_Main_receiveData(USB_UART_0_INST);  // 读取接收到的数据
            
            // 检查缓冲区是否已满
            if (UART0_RxBuffer.index >= sizeof(UART0_RxBuffer.buffer)) {
                // 缓冲区溢出，重置状态
                UART0_RxBuffer.index = 0;
                UART0_RxBuffer.frame_ready = 0;
            }
            
            // 存储数据到缓冲区
            UART0_RxBuffer.buffer[UART0_RxBuffer.index] = data;
            UART0_RxBuffer.index++;
            
            // 检测帧结束符 '\n'
            if (data == '\n') {
                UART0_RxBuffer.frame_ready = 1;  // 标记帧接收完成
            }
            break;

        default://其他的串口中断
            break;
    }
}

void UART0_ProcessFrame(void)
{
    if (!UART0_RxBuffer.frame_ready)
        return;  // 如果没有完整帧，直接返回

    char frame[128] = {0};  // 存储解析后的字符串
    uint16_t j = 0;         // frame 的索引
    
    // 复制数据到 frame 数组，忽略 '\r' 和 '\n'
    for (uint16_t i = 0; i < UART0_RxBuffer.index; i++) {
        uint8_t data = UART0_RxBuffer.buffer[i];
        
        if (data == '\n') {
            break;  // 遇到换行符停止复制
        }
        
        if (data != '\r') {  // 忽略回车符
            if (j < sizeof(frame) - 1) {
                frame[j++] = data;
            }
        }
    }
    frame[j] = '\0';  // 确保字符串终止

    // 解析以 '!' 开头的帧
    if (frame[0] == '!') {
        int result = sscanf(&frame[1], "%d,%d", &x, &y);  // 添加取地址符&
        // 如果解析失败（返回值小于2），可设置默认值
    }

    // 重置接收状态
    UART0_RxBuffer.index = 0;
    UART0_RxBuffer.frame_ready = 0;
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
