#include "car_bsp.h"


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
            //将发送过来的数据保存在变量中
            uart0_data = DL_UART_Main_receiveData(USB_UART_0_INST);
            printf("%d\r\n",uart0_data);
            //将保存的数据再发送出去
            break;

        default://其他的串口中断
            break;
    }
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
