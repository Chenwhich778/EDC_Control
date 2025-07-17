#include "ICM20948.h"
#include "imu_task.h"
#include <math.h>


#define M_PI 3.14159265358979323846f
#define RAD_TO_DEG (180.0f / M_PI)
#define DEG_TO_RAD (M_PI / 180.0f)




ICM20948_ST_SENSOR_DATA gstGyroOffset = {0,0,0};

//??????????
short magnet[3];


// 卡尔曼滤波器结构体
typedef struct {
    float Q_angle;   // 过程噪声协方差（角度）
    float Q_bias;    // 过程噪声协方差（偏差）
    float R_measure; // 测量噪声协方差
    
    float angle;     // 计算出的角度
    float bias;      // 陀螺仪偏差
    float rate;      // 未经滤波的角速度
    
    float P[2][2];   // 误差协方差矩阵
} Kalman_t;

// 磁力计数据结构体
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} MagData_t;

// 全局变量
static Kalman_t kalmanYaw = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f,
    
    .angle = 0.0f,
    .bias = 0.0f,
    .rate = 0.0f,
    
    .P[0][0] = 0.0f,
    .P[0][1] = 0.0f,
    .P[1][0] = 0.0f,
    .P[1][1] = 0.0f
};

static float yaw = 0.0f;  // 最终的偏航角
static uint32_t last_time = 0;
static MagData_t magData = {0, 0, 0}; // 磁力计数据

/**************************************************************************
Function: Initialize TIM2 as the encoder interface mode
Input   : Gx, Gy, Gz: raw readings (plus or minus) of the x,y, and z axes of the gyroscope
Output  : 0: success, others: error code
函数功能：获得陀螺仪值(原始值)
入口参数：gx,gy,gz:陀螺仪x,y,z轴的原始读数(带正负)
返回  值：0:成功, 其他:错误代码
**************************************************************************/
u8 ICM20948_Get_Gyroscope(void)
{
	static int onece=0;
    u8 res;
    invMSGyroRead(&imu.gyro.x, &imu.gyro.y, &imu.gyro.z);
    if(SysVal.Time_count<CONTROL_DELAY) // 10 seconds before starting //开机前10秒
    {
		Led_Count=1; //LED high frequency flashing //LED高频闪烁
		Flag_Stop=1; //The software fails to flag location 1 //软件失能标志位置1		

    }
    else //10 seconds after starting //开机10秒后
    {
		if(onece==0)
		{
			Flag_Stop=0; //The software fails to flag location 0 //软件失能标志位置0
			onece=1;
		}	
		Led_Count=300; //The LED returns to normal flicker frequency //LED恢复正常闪烁频率
        //Save the raw data to update zero by clicking the user button
        //保存原始数据用于单击用户按键更新零点
		imu.Original_gyro.x = imu.gyro.x;
		imu.Original_gyro.y = imu.gyro.y;
		imu.Original_gyro.z = imu.gyro.z;

        //Removes zero drift data
        //去除零点漂移的数据
		imu.gyro.x = imu.Original_gyro.x - imu.Deviation_gyro.x;
		imu.gyro.y = imu.Original_gyro.y - imu.Deviation_gyro.y;
		imu.gyro.z = imu.Original_gyro.z - imu.Deviation_gyro.z;
    }

    return res;
}


u8 ICM20948_Get_Accel(void)
{
    u8 res;
    invMSAccelRead(&imu.accel.x, &imu.accel.y, &imu.accel.z); //得到加速度传感器数据
    if(SysVal.Time_count<CONTROL_DELAY) // 10 seconds before starting //开机前10秒
    {
		
    }
    else //10 seconds after starting //开机10秒后
    {
        //Save the raw data to update zero by clicking the user button
        //保存原始数据用于单击用户按键更新零点
		imu.Original_accel.x = imu.accel.x;
		imu.Original_accel.y = imu.accel.y;
		imu.Original_accel.z = imu.accel.z;

        //Removes zero drift data
        //去除零点漂移的数据
        imu.accel.x = imu.Original_accel.x - imu.Deviation_accel.x ;
        imu.accel.y = imu.Original_accel.y - imu.Deviation_accel.y ;
        imu.accel.z = imu.Original_accel.z - imu.Deviation_accel.z + 16384;
    }

    return res;
}



uint8_t ICM20948_getDeviceID(void)
{
    return I2C_ReadOneByte(I2C_ADD_ICM20948, REG_ADD_WIA);
}

static bool invmsICM20948Check(void)
{
    bool bRet = false;
    if(REG_VAL_WIA == I2C_ReadOneByte(I2C_ADD_ICM20948, REG_ADD_WIA))
    {
        bRet = true;
    }
    return bRet;
}

void invMSInit(void)
{
    if(invmsICM20948Check())//检测是否能识别到ICM20498器件
    {
        invmsICM20948Init(); //ICM20498初始化
    }
    return;
}

static void invmsICM20948Init(void)
{
    /* user bank 0 register */
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0);
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_PWR_MIGMT_1,  REG_VAL_ALL_RGE_RESET);
    delay_ms(10);
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_PWR_MIGMT_1,  REG_VAL_RUN_MODE);

    /* user bank 2 register */
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_2);

    I2C_WriteOneByte( I2C_ADD_ICM20948, REG_ADD_GYRO_SMPLRT_DIV, 0x07);
    I2C_WriteOneByte( I2C_ADD_ICM20948, REG_ADD_GYRO_CONFIG_1,REG_VAL_BIT_GYRO_DLPCFG_6 | REG_VAL_BIT_GYRO_FS_500DPS | REG_VAL_BIT_GYRO_DLPF);

    I2C_WriteOneByte( I2C_ADD_ICM20948, REG_ADD_ACCEL_SMPLRT_DIV_2,  0x07);
    I2C_WriteOneByte( I2C_ADD_ICM20948, REG_ADD_ACCEL_CONFIG,REG_VAL_BIT_ACCEL_DLPCFG_6 | REG_VAL_BIT_ACCEL_FS_2g | REG_VAL_BIT_ACCEL_DLPF);

    /* user bank 0 register */
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0);

    delay_ms(100);
    /* offset */
    invmsICM20948GyroOffset();

    invmsICM20948MagCheck();

    invmsICM20948WriteSecondary( I2C_ADD_ICM20948_AK09916|I2C_ADD_ICM20948_AK09916_WRITE,REG_ADD_MAG_CNTL2, REG_VAL_MAG_MODE_100HZ);
    return;
}

//角速度读取的底层实现
static void invmsICM20948GyroRead(int16_t* ps16X, int16_t* ps16Y, int16_t* ps16Z)
{
    uint8_t u8Buf[6];
    int16_t s16Buf[3] = {0};
    uint8_t i;
    int32_t s32OutBuf[3] = {0};
    static ICM20948_ST_AVG_DATA sstAvgBuf[3];
    static int16_t ss16c = 0;
    ss16c++;

    u8Buf[0]=I2C_ReadOneByte(I2C_ADD_ICM20948,REG_ADD_GYRO_XOUT_L);
    u8Buf[1]=I2C_ReadOneByte(I2C_ADD_ICM20948,REG_ADD_GYRO_XOUT_H);
    s16Buf[0]=	(u8Buf[1]<<8)|u8Buf[0];

    u8Buf[0]=I2C_ReadOneByte(I2C_ADD_ICM20948,REG_ADD_GYRO_YOUT_L);
    u8Buf[1]=I2C_ReadOneByte(I2C_ADD_ICM20948,REG_ADD_GYRO_YOUT_H);
    s16Buf[1]=	(u8Buf[1]<<8)|u8Buf[0];

    u8Buf[0]=I2C_ReadOneByte(I2C_ADD_ICM20948,REG_ADD_GYRO_ZOUT_L);
    u8Buf[1]=I2C_ReadOneByte(I2C_ADD_ICM20948,REG_ADD_GYRO_ZOUT_H);
    s16Buf[2]=	(u8Buf[1]<<8)|u8Buf[0];

#if 1
    for(i = 0; i < 3; i ++)
    {
        invmsICM20948CalAvgValue(&sstAvgBuf[i].u8Index, sstAvgBuf[i].s16AvgBuffer, s16Buf[i], s32OutBuf + i);
    }
    *ps16X = s32OutBuf[0] - gstGyroOffset.s16X;
    *ps16Y = s32OutBuf[1] - gstGyroOffset.s16Y;
    *ps16Z = s32OutBuf[2] - gstGyroOffset.s16Z;
#else
    *ps16X = s16Buf[0];
    *ps16Y = s16Buf[1];
    *ps16Z = s16Buf[2];
#endif
    return;
}

//加速度读取的底层实现
static void invmsICM20948AccelRead(int16_t* ps16X, int16_t* ps16Y, int16_t* ps16Z)
{
    uint8_t u8Buf[2];
    int16_t s16Buf[3] = {0};
    uint8_t i;
    int32_t s32OutBuf[3] = {0};
    static ICM20948_ST_AVG_DATA sstAvgBuf[3];

    u8Buf[0]=I2C_ReadOneByte(I2C_ADD_ICM20948,REG_ADD_ACCEL_XOUT_L);
    u8Buf[1]=I2C_ReadOneByte(I2C_ADD_ICM20948,REG_ADD_ACCEL_XOUT_H);
    s16Buf[0]=	(u8Buf[1]<<8)|u8Buf[0];

    u8Buf[0]=I2C_ReadOneByte(I2C_ADD_ICM20948,REG_ADD_ACCEL_YOUT_L);
    u8Buf[1]=I2C_ReadOneByte(I2C_ADD_ICM20948,REG_ADD_ACCEL_YOUT_H);
    s16Buf[1]=	(u8Buf[1]<<8)|u8Buf[0];

    u8Buf[0]=I2C_ReadOneByte(I2C_ADD_ICM20948,REG_ADD_ACCEL_ZOUT_L);
    u8Buf[1]=I2C_ReadOneByte(I2C_ADD_ICM20948,REG_ADD_ACCEL_ZOUT_H);
    s16Buf[2]=	(u8Buf[1]<<8)|u8Buf[0];

#if 1
    for(i = 0; i < 3; i ++)
    {
        invmsICM20948CalAvgValue(&sstAvgBuf[i].u8Index, sstAvgBuf[i].s16AvgBuffer, s16Buf[i], s32OutBuf + i);
    }
    *ps16X = s32OutBuf[0];
    *ps16Y = s32OutBuf[1];
    *ps16Z = s32OutBuf[2];

#else
    *ps16X = s16Buf[0];
    *ps16Y = s16Buf[1];
    *ps16Z = s16Buf[2];
#endif
    return;

}

//磁力计读取的底层实现
static void invmsICM20948MagRead(int16_t* ps16X, int16_t* ps16Y, int16_t* ps16Z)
{
   uint8_t counter = 20;
   uint8_t u8Data[MAG_DATA_LEN];
   int16_t s16Buf[3] = {0};
   uint8_t i;
   int32_t s32OutBuf[3] = {0};
   static ICM20948_ST_AVG_DATA sstAvgBuf[3];
   while( counter>0 )
   {
       delay_ms(10);
       invmsICM20948ReadSecondary( I2C_ADD_ICM20948_AK09916|I2C_ADD_ICM20948_AK09916_READ,
                                   REG_ADD_MAG_ST2, 1, u8Data);

       if ((u8Data[0] & 0x01) != 0)
           break;

       counter--;
   }

   if(counter != 0)
   {
       invmsICM20948ReadSecondary( I2C_ADD_ICM20948_AK09916|I2C_ADD_ICM20948_AK09916_READ,
                                   REG_ADD_MAG_DATA,
                                   MAG_DATA_LEN,
                                   u8Data);
       s16Buf[0] = ((int16_t)u8Data[1]<<8) | u8Data[0];
       s16Buf[1] = ((int16_t)u8Data[3]<<8) | u8Data[2];
       s16Buf[2] = ((int16_t)u8Data[5]<<8) | u8Data[4];
   }
   else
   {
//        printf("\r\n Mag is bussy \r\n");
   }
#if 1
   for(i = 0; i < 3; i ++)
   {
       invmsICM20948CalAvgValue(&sstAvgBuf[i].u8Index, sstAvgBuf[i].s16AvgBuffer, s16Buf[i], s32OutBuf + i);
   }

   *ps16X =  s32OutBuf[0];
   *ps16Y = -s32OutBuf[1];
   *ps16Z = -s32OutBuf[2];
#else
   *ps16X = s16Buf[0];
   *ps16Y = -s16Buf[1];
   *ps16Z = -s16Buf[2];
#endif
   return;
}

// 磁力计读取函数（外部可调用）
u8 ICM20948_Get_Mag(void)
{
    u8 res = 0;
    int16_t x, y, z;
    invmsICM20948MagRead(&x, &y, &z);
    return res;
}

// 卡尔曼滤波器初始化
void Kalman_Init(Kalman_t *kalman, float angle) {
    kalman->angle = angle;
    kalman->P[0][0] = 0.0f;
    kalman->P[0][1] = 0.0f;
    kalman->P[1][0] = 0.0f;
    kalman->P[1][1] = 0.0f;
}


static bool invmsICM20948MagCheck(void)
{
    bool bRet = false;
    uint8_t u8Ret[2];

    invmsICM20948ReadSecondary( I2C_ADD_ICM20948_AK09916|I2C_ADD_ICM20948_AK09916_READ,
                                REG_ADD_MAG_WIA1, 2,u8Ret);
    if( (u8Ret[0] == REG_VAL_MAG_WIA1) && ( u8Ret[1] == REG_VAL_MAG_WIA2) )
    {
        bRet = true;
    }

    return bRet;
}

static void invmsICM20948ReadSecondary(uint8_t u8I2CAddr, uint8_t u8RegAddr, uint8_t u8Len, uint8_t *pu8data)
{
    uint8_t i;
    uint8_t u8Temp;

    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_REG_BANK_SEL,  REG_VAL_REG_BANK_3); //swtich bank3
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_I2C_SLV0_ADDR, u8I2CAddr);
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_I2C_SLV0_REG,  u8RegAddr);
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_I2C_SLV0_CTRL, REG_VAL_BIT_SLV0_EN|u8Len);

    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0); //swtich bank0

    u8Temp = I2C_ReadOneByte(I2C_ADD_ICM20948,REG_ADD_USER_CTRL);
    u8Temp |= REG_VAL_BIT_I2C_MST_EN;
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_USER_CTRL, u8Temp);
    delay_ms(5);
    u8Temp &= ~REG_VAL_BIT_I2C_MST_EN;
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_USER_CTRL, u8Temp);

    for(i=0; i<u8Len; i++)
    {
        *(pu8data+i) = I2C_ReadOneByte(I2C_ADD_ICM20948, REG_ADD_EXT_SENS_DATA_00+i);

    }
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_3); //swtich bank3

    u8Temp = I2C_ReadOneByte(I2C_ADD_ICM20948,REG_ADD_I2C_SLV0_CTRL);
    u8Temp &= ~((REG_VAL_BIT_I2C_MST_EN)&(REG_VAL_BIT_MASK_LEN));
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_I2C_SLV0_CTRL,  u8Temp);

    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0); //swtich bank0

}

static void invmsICM20948WriteSecondary(uint8_t u8I2CAddr, uint8_t u8RegAddr, uint8_t u8data)
{
    uint8_t u8Temp;
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_REG_BANK_SEL,  REG_VAL_REG_BANK_3); //swtich bank3
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_I2C_SLV1_ADDR, u8I2CAddr);
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_I2C_SLV1_REG,  u8RegAddr);
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_I2C_SLV1_DO,   u8data);
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_I2C_SLV1_CTRL, REG_VAL_BIT_SLV0_EN|1);

    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0); //swtich bank0

    u8Temp = I2C_ReadOneByte(I2C_ADD_ICM20948,REG_ADD_USER_CTRL);
    u8Temp |= REG_VAL_BIT_I2C_MST_EN;
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_USER_CTRL, u8Temp);
    delay_ms(5);
    u8Temp &= ~REG_VAL_BIT_I2C_MST_EN;
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_USER_CTRL, u8Temp);

    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_3); //swtich bank3

    u8Temp = I2C_ReadOneByte(I2C_ADD_ICM20948,REG_ADD_I2C_SLV0_CTRL);
    u8Temp &= ~((REG_VAL_BIT_I2C_MST_EN)&(REG_VAL_BIT_MASK_LEN));
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_I2C_SLV0_CTRL,  u8Temp);

    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0); //swtich bank0

    return;
}

static void invmsICM20948CalAvgValue(uint8_t *pIndex, int16_t *pAvgBuffer, int16_t InVal, int32_t *pOutVal)
{
    uint8_t i;

    *(pAvgBuffer + ((*pIndex) ++)) = InVal;
    *pIndex &= 0x07;

    *pOutVal = 0;
    for(i = 0; i < 8; i ++)
    {
        *pOutVal += *(pAvgBuffer + i);
    }
    *pOutVal >>= 3;
}

static void invmsICM20948GyroOffset(void)
{
    uint8_t i;
    int16_t	s16Gx = 0, s16Gy = 0, s16Gz = 0;
    int32_t	s32TempGx = 0, s32TempGy = 0, s32TempGz = 0;
    for(i = 0; i < 32; i ++)
    {
        invmsICM20948GyroRead(&s16Gx, &s16Gy, &s16Gz);
        s32TempGx += s16Gx;
        s32TempGy += s16Gy;
        s32TempGz += s16Gz;
        delay_ms(10);
    }
    gstGyroOffset.s16X = s32TempGx >> 5;
    gstGyroOffset.s16Y = s32TempGy >> 5;
    gstGyroOffset.s16Z = s32TempGz >> 5;
    return;
}


//读取三轴加速度
static void invMSAccelRead(int16_t* ps16AccelX, int16_t* ps16AccelY, int16_t* ps16AccelZ)
{
    invmsICM20948AccelRead(ps16AccelX, ps16AccelY, ps16AccelZ);
    return;
}

//读取三轴角速度
static void invMSGyroRead(int16_t* ps16GyroX, int16_t* ps16GyroY, int16_t* ps16GyroZ)
{
    invmsICM20948GyroRead(ps16GyroX, ps16GyroY, ps16GyroZ);
    return;
}

//读取磁力计.若需要使用直接取消注释即可
static void invMSMagRead(int16_t* ps16MagnX, int16_t* ps16MagnY, int16_t* ps16MagnZ)
{
   invmsICM20948MagRead(ps16MagnX, ps16MagnY, ps16MagnZ);
   return;
}




// 卡尔曼滤波更新
float Kalman_Update(Kalman_t *kalman, float newAngle, float newRate, float dt) {
    // 预测步骤
    kalman->rate = newRate - kalman->bias;
    kalman->angle += dt * kalman->rate;
    
    // 更新协方差矩阵
    kalman->P[0][0] += dt * (dt * kalman->P[1][1] - kalman->P[0][1] - kalman->P[1][0] + kalman->Q_angle);
    kalman->P[0][1] -= dt * kalman->P[1][1];
    kalman->P[1][0] -= dt * kalman->P[1][1];
    kalman->P[1][1] += kalman->Q_bias * dt;
    
    // 计算卡尔曼增益
    float S = kalman->P[0][0] + kalman->R_measure;
    float K[2];
    K[0] = kalman->P[0][0] / S;
    K[1] = kalman->P[1][0] / S;
    
    // 更新估计
    float y = newAngle - kalman->angle;
    kalman->angle += K[0] * y;
    kalman->bias += K[1] * y;
    
    // 更新协方差
    float P00_temp = kalman->P[0][0];
    float P01_temp = kalman->P[0][1];
    
    kalman->P[0][0] -= K[0] * P00_temp;
    kalman->P[0][1] -= K[0] * P01_temp;
    kalman->P[1][0] -= K[1] * P00_temp;
    kalman->P[1][1] -= K[1] * P01_temp;
    
    return kalman->angle;
}


// 从磁力计数据计算偏航角
float calculateYawFromMag(int16_t mx, int16_t my) {
    // 计算磁力计偏航角（弧度）
    float yaw = atan2(-my, mx);
    
    // 转换为角度（0-360度）
    yaw *= RAD_TO_DEG;
    if(yaw < 0) yaw += 360.0f;
    
    return yaw;
}

// // 初始化函数（在系统启动时调用）
// void Yaw_Measurement_Init(void) {
//     // 初始化磁力计
//     invmsICM20948MagCheck();
//     invmsICM20948WriteSecondary(I2C_ADD_ICM20948_AK09916|I2C_ADD_ICM20948_AK09916_WRITE,
//                                REG_ADD_MAG_CNTL2, REG_VAL_MAG_MODE_100HZ);
    
//     // 读取初始磁力计值
//     ICM20948_Get_Mag();
    
//     // 计算初始偏航角
//     float initialYaw = calculateYawFromMag(magData.x, magData.y);
    
//     // 初始化卡尔曼滤波器
//     Kalman_Init(&kalmanYaw, initialYaw);
    
//     // 初始化时间
//     last_time = HAL_GetTick();
// }
