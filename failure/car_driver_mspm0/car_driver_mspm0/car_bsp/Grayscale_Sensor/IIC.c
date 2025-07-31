#include "IIC.h"
unsigned char hardware_IIC_WirteByte(unsigned char Salve_Adress,unsigned char Reg_Address, unsigned char dat)
{	
    uint8_t temp[2];
    temp[0] = Reg_Address;
    temp[1] = dat;
    DL_I2C_fillControllerTXFIFO(I2C0, temp, 2);
    while (!(DL_I2C_getControllerStatus(I2C0) & DL_I2C_CONTROLLER_STATUS_IDLE));
    DL_I2C_startControllerTransfer(I2C0, Salve_Adress, DL_I2C_CONTROLLER_DIRECTION_TX, 2);
    while (DL_I2C_getControllerStatus(I2C0) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS);
    while (!(DL_I2C_getControllerStatus(I2C0) & DL_I2C_CONTROLLER_STATUS_IDLE));
    DL_I2C_flushControllerTXFIFO(I2C0);
		return 1;
}
unsigned char hardware_IIC_WirteBytes(unsigned char Salve_Adress,unsigned char Reg_Address, unsigned char * dat, unsigned char len)
{
		unsigned char temp[len+1];
	  temp[0] = Reg_Address;
		memcpy(temp+1,dat,len);
    DL_I2C_fillControllerTXFIFO(I2C0, temp, len+1);
    while (!(DL_I2C_getControllerStatus(I2C0) & DL_I2C_CONTROLLER_STATUS_IDLE));
    DL_I2C_startControllerTransfer(I2C0, Sensor_Adrees, DL_I2C_CONTROLLER_DIRECTION_TX, len+1);
    while (DL_I2C_getControllerStatus(I2C0) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS);
    while (!(DL_I2C_getControllerStatus(I2C0) & DL_I2C_CONTROLLER_STATUS_IDLE));
    DL_I2C_flushControllerTXFIFO(I2C0);
		return 1;
}
unsigned char hardware_IIC_ReadByte(unsigned char Salve_Adress,unsigned char Reg_Address) 
{
    unsigned char data;
    DL_I2C_fillControllerTXFIFO(I2C0, &Reg_Address, 1);
    while (!(DL_I2C_getControllerStatus(I2C0) & DL_I2C_CONTROLLER_STATUS_IDLE));
    DL_I2C_startControllerTransfer(I2C0, Salve_Adress, DL_I2C_CONTROLLER_DIRECTION_TX, 1);
    while (DL_I2C_getControllerStatus(I2C0) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS);
    while (!(DL_I2C_getControllerStatus(I2C0) & DL_I2C_CONTROLLER_STATUS_IDLE));
    DL_I2C_flushControllerTXFIFO(I2C0);
 
    DL_I2C_startControllerTransfer(I2C0, Salve_Adress, DL_I2C_CONTROLLER_DIRECTION_RX, 1);
    while (DL_I2C_getControllerStatus(I2C0) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS);
    while (!(DL_I2C_getControllerStatus(I2C0) & DL_I2C_CONTROLLER_STATUS_IDLE));
    data = DL_I2C_receiveControllerData(I2C0);
 
    return data;
}
unsigned char hardware_IIC_ReadBytes(unsigned char Salve_Adress,unsigned char Reg_Address,unsigned char *data,unsigned char len)
{ 
    DL_I2C_fillControllerTXFIFO(I2C0, &Reg_Address, 1);
    while (!(DL_I2C_getControllerStatus(I2C0) & DL_I2C_CONTROLLER_STATUS_IDLE));
    DL_I2C_startControllerTransfer(I2C0, Salve_Adress, DL_I2C_CONTROLLER_DIRECTION_TX, 1);
    while (DL_I2C_getControllerStatus(I2C0) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS);
    while (!(DL_I2C_getControllerStatus(I2C0) & DL_I2C_CONTROLLER_STATUS_IDLE));
    DL_I2C_flushControllerTXFIFO(I2C0);
 
    DL_I2C_startControllerTransfer(I2C0, Salve_Adress, DL_I2C_CONTROLLER_DIRECTION_RX, len);
    while (DL_I2C_getControllerStatus(I2C0) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS);
    while (!(DL_I2C_getControllerStatus(I2C0) & DL_I2C_CONTROLLER_STATUS_IDLE));
	
		for (uint8_t i = 0; i < len; i++) {
    data[i] = DL_I2C_receiveControllerData(I2C0);
		}
		return 1;
}