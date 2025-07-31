#include "ti_msp_dl_config.h"
#include "string.h"
#include "gw_grayscale_sensor.h"
#define Sensor_Adrees 0x4c
unsigned char hardware_IIC_WirteByte(unsigned char Salve_Adress,unsigned char Reg_Address, unsigned char dat);
unsigned char hardware_IIC_WirteBytes(unsigned char Salve_Adress,unsigned char Reg_Address, unsigned char * dat, unsigned char len);
unsigned char hardware_IIC_ReadByte(unsigned char Salve_Adress,unsigned char Reg_Address); 
unsigned char hardware_IIC_ReadBytes(unsigned char Salve_Adress,unsigned char Reg_Address,unsigned char *data,unsigned char len);