#include "car_bsp.h"


void vcc_adc_IQR_init(void)
{
    NVIC_EnableIRQ(VCC_ADC_INST_INT_IRQN);
}



bool gCheckADC=false;
uint16_t get_vcc_adc_value(void)
{
    uint16_t gAdcResult =0;
    DL_ADC12_startConversion(VCC_ADC_INST);
    while (false == gCheckADC) {
           // __WFE();
        }
    gAdcResult = DL_ADC12_getMemResult(VCC_ADC_INST, VCC_ADC_ADCMEM_0);
    gCheckADC = false;
    return gAdcResult;
}
void VCC_ADC_INST_IRQHandler(void)
{
    //查询并清除ADC中断
    switch (DL_ADC12_getPendingInterrupt(VCC_ADC_INST))
    {
        //检查是否完成数据采集
        case DL_ADC12_IIDX_MEM0_RESULT_LOADED:
            gCheckADC = true;//将标志位置1
            break;
        default:
            break;
    }
}