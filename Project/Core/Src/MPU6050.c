#include "MPU6050.h"

/**
 * @description: 向MPU6050写入一个字节
 * @param {uint8_t} reg_addr 寄存器地址
 * @param {uint8_t} write_byte 要写入的数据
 * @return {uint8_t} 0:成功, 1:失败
 */
uint8_t MPU6050_WriteByte(uint8_t reg_addr, uint8_t write_byte)
{
    IIC_Start();
    IIC_Send_Byte((MPU_IIC_ADDR << 1) | IIC_WR);
    if (IIC_Wait_Ack())
    {
        IIC_Stop();
        return 1;
    }
    
    IIC_Send_Byte(reg_addr);
    if (IIC_Wait_Ack())
    {
        IIC_Stop();
        return 1;
    }
    
    IIC_Send_Byte(write_byte);
    if (IIC_Wait_Ack())
    {
        IIC_Stop();
        return 1;
    }
    
    IIC_Stop();
    return 0;
}



/**
 * @description: 向MPU6050写入多个字节
 * @param {uint8_t} reg_addr 寄存器起始地址
 * @param {uint8_t} *write_bytes 数据缓冲区
 * @param {uint8_t} size 数据长度
 * @return {uint8_t} 0:成功, 1:失败
 */
uint8_t MPU6050_writeBytes(uint8_t reg_addr, uint8_t write_bytes[], uint8_t size)
{
    IIC_Start();

    IIC_Send_Byte((MPU_IIC_ADDR << 1) | IIC_WR);
    if (IIC_Wait_Ack())
    {
        IIC_Stop();
        return 1;
    }

    IIC_Send_Byte(reg_addr);
    if (IIC_Wait_Ack())
    {
        IIC_Stop();
        return 1;
    }

    for (int i = 0; i < size; i++)
    {
        IIC_Send_Byte(write_bytes[i]);
        if (IIC_Wait_Ack())
        {
            IIC_Stop();
            return 1;
        }
    }
    IIC_Stop();
    return 0;
}

/**
 * @description: 读取指定寄存器一个字节数据
 * @param {uint8_t} reg_addr 寄存器地址
 * * @param {uint8_t} *byte 接收的数据
 * @return {uint8_t} 0:成功, 1:失败
 */
uint8_t MPU6050_ReadByte(uint8_t reg_addr, uint8_t *byte)
{
    IIC_Start();
    IIC_Send_Byte((MPU_IIC_ADDR << 1) | IIC_WR);
    if (IIC_Wait_Ack())
    {
        IIC_Stop();
        return 1;
    }

    IIC_Send_Byte(reg_addr);
    if (IIC_Wait_Ack())
    {
        IIC_Stop();
        return 1;
    }

    IIC_Start();
    IIC_Send_Byte((MPU_IIC_ADDR << 1) | IIC_RD);
    if (IIC_Wait_Ack())
    {
        IIC_Stop();
        return 1;
    }

    *byte = IIC_Read_Byte(0);
    IIC_Stop();

    return 0;
}



/**
 * @description: 读取指定寄存器多个字节数据
 * @param {uint8_t} reg_addr 寄存器起始地址
 * @param {uint8_t} *bytes 接收缓冲区
 * @param {uint8_t} size 读取长度
 * @return {uint8_t} 0:成功, 1:失败
 */
uint8_t MPU6050_ReadBytes(uint8_t reg_addr, uint8_t *bytes, uint8_t size)
{
    IIC_Start();
    IIC_Send_Byte((MPU_IIC_ADDR << 1) | IIC_WR);
    if (IIC_Wait_Ack())
    {
        IIC_Stop();
        return 1;
    }

    IIC_Send_Byte(reg_addr);
    if (IIC_Wait_Ack())
    {
        IIC_Stop();
        return 1;
    }

    IIC_Start();
    IIC_Send_Byte((MPU_IIC_ADDR << 1) | IIC_RD);
    if (IIC_Wait_Ack())
    {
        IIC_Stop();
        return 1;
    }

    for (int i = 0; i < size; i++)
    {
        if (i == size - 1)  bytes[i] = IIC_Read_Byte(0);
        else bytes[i] = IIC_Read_Byte(1);
    }

    IIC_Stop();
    return 0;
}

/**
 * @description: 根据采样率设置低通滤波器
 * @param {uint16_t} rate
 * @return {*}
 */
void MPU6050_Set_DLPF_CFG(uint16_t rate)
{
    /* 采样定理 采样率 >= 2*带宽 才不失真 ===> 带宽 <= 采样率/2 */
    uint8_t cfg = 0;
    rate = rate / 2;
    if (rate > 188) cfg = 1;
    else if (rate > 98) cfg = 2;
    else if (rate > 42) cfg = 3;
    else if (rate > 20) cfg = 4;
    else if (rate > 10) cfg = 5;
    else cfg = 6;
    
    MPU6050_WriteByte(MPU_CFG_REG, cfg << 0);
}

/**
 * @description: 设置陀螺仪的采样率
 * @param {uint16_t} rate
 * @return {*}
 */
void MPU6050_SetGyroRate(uint16_t rate)
{
    /* 采样率=陀螺仪输出频率/(1+分频值) ===> 分频值 = (陀螺仪输出频率/采样率) -1 */
    uint8_t sample_div = 0;
    /* 1. 采样率范围: 芯片说明书12页，采样频率最小值=4，陀螺仪8k或1k */
    if (rate < 4) rate = 4;
    else if (rate > 1000) rate = 1000;

    /* 2. 根据需要的采样率，计算分频值 */
    sample_div = 1000 / rate - 1;

    /* 3. 将分频值设置到寄存器中 */
    MPU6050_WriteByte(MPU_SAMPLE_RATE_REG, sample_div);

    /* 4. 根据采样率去设置低通滤波器 */
    MPU6050_Set_DLPF_CFG(rate);
}


/**
 * @description: 初始化MPU6050
 * @return {uint8_t} 0:成功, 1:失败
 */
uint8_t MPU6050_Init(void)
{
    uint8_t dev_id = 0;

    // 0.初始化GPIO
    IIC_GPIO_Init();

    // 1.复位 - 延时 - 唤醒
    MPU6050_WriteByte(MPU_PWR_MGMT1_REG, 0x80);  // 往 0X6B 写 0x80
    HAL_Delay(100); 
    MPU6050_WriteByte(MPU_PWR_MGMT1_REG, 0x00);  // 往 0X6B 写 0x00

    // 2.陀螺仪量程
    MPU6050_WriteByte(MPU_GYRO_CFG_REG, 0x18);   // 往 0X1B 写 0x18

    // 3.加速度量程
    MPU6050_WriteByte(MPU_ACCEL_CFG_REG, 0x00);  // 往 0X1C 写 0x00

    // 4.中断/FIFO/I2C主机配置
    MPU6050_WriteByte(MPU_INT_EN_REG, 0x00);   // 关闭所有中断：往 0x38 写 0x00
    MPU6050_WriteByte(MPU_USER_CTRL_REG, 0x00); // 关闭第二IIC：往 0X6A 写 0x00
    MPU6050_WriteByte(MPU_FIFO_EN_REG, 0x00);   // 关闭FIFO：往 0x23 写 0x00
    MPU6050_WriteByte(MPU_INTBP_CFG_REG, 0X80); // INT引脚低电平有效

    // 5.读取ID
    MPU6050_ReadByte(MPU_DEVICE_ID_REG, &dev_id);

    // 检查ID
    if (dev_id == MPU_IIC_ADDR)
    {
        /* 5.1 设置时钟源，PLL X轴 */
        MPU6050_WriteByte(MPU_PWR_MGMT1_REG, 0x01);
        /* 5.2 设置采样率 100Hz */
        MPU6050_SetGyroRate(100);
        /* 5.3 开启工作 (虽然前面复位唤醒了，这里确保不进入睡眠) */
        MPU6050_WriteByte(MPU_PWR_MGMT2_REG, 0x00);
    }
    else
    {
        return 1; // ID不对，返回错误
    }

    return 0; // 初始化成功
}


/* ========================== 上层应用 ========================== */

/**
 * @description: 读取陀螺仪角速度的数据
 * @param {short} *gx, *gy, *gz 指针
 * @return {uint8_t} 0:成功, 1:失败
 */
uint8_t MPU6050_Get_Gyro(short *gx, short *gy, short *gz)
{
    uint8_t buff[6];
    uint8_t res;
    
    res = MPU6050_ReadBytes(MPU_GYRO_XOUTH_REG, buff, 6);
    
    if (res == 0)
    {
        *gx = ((uint16_t)buff[0] << 8) | buff[1];
        *gy = ((uint16_t)buff[2] << 8) | buff[3];
        *gz = ((uint16_t)buff[4] << 8) | buff[5];
        return 0; // 成功
    }
    
    return 1; // 失败
}

/**
 * @description: 读取加速度计的加速度数据
 * @param {short} *ax, *ay, *az 指针
 * @return {uint8_t} 0:成功, 1:失败
 */
uint8_t MPU6050_Get_Accel(short *ax, short *ay, short *az)
{
    uint8_t buff[6];
    uint8_t res;
    
    res = MPU6050_ReadBytes(MPU_ACCEL_XOUTH_REG, buff, 6);
    
    if (res == 0)
    {
        *ax = ((uint16_t)buff[0] << 8) | buff[1];
        *ay = ((uint16_t)buff[2] << 8) | buff[3];
        *az = ((uint16_t)buff[4] << 8) | buff[5];
        return 0; // 成功
    }
    
    return 1; // 失败
}

/*================================== 根据加速度求的角度 ==================================*/

volatile float accel_angle = 0; 

// 根据加速度求的角度
void MPU6050_Get_AccelAngle(void)
{
    short ax, ay, az;

    // 获取MPU6050的加速度
    MPU6050_Get_Accel(&ax, &ay, &az);

    // 通过受力分析得：角度的tan值为ay / az (为什么不需要除灵敏度比例因子，因为会给约掉)
    // atan2反正切函数：得到的是弧度,角度 = 弧度 * 180 / PI
    accel_angle = atan2(ay, az) * 180 / PI - 1.5f;
}


/*================================== 根据角速度求的角度 ==================================*/

volatile float gyro_x_offset = 0; // X轴零偏（直立环用）
volatile float gyro_z_offset = 0; // Z轴零偏（转向环用）

// 角速度 X Y轴校准函数
void MPU6050_Calibrate_Gyro(void)
{
    short gx, gy, gz;
    long sum_gx = 0;
    long sum_gz = 0;
    uint16_t sample_count = 200; // 采集200次数据
    
    for (uint16_t i = 0; i < sample_count; i++)
    {
        // 调用你原来的读取函数
        if (MPU6050_Get_Gyro(&gx, &gy, &gz) == 0)
        {
            sum_gx += gx; // 只累加 X 轴
            sum_gz += gz; // 只累加 Z 轴
        }
        else
        {
            i--; // 如果 I2C 读取失败，这次不算，重读
        }
        
        HAL_Delay(3); // 每次采集停 3ms，防止读取太快拿到重复数据
    }
    // 计算平均偏移量
    gyro_x_offset = (float)sum_gx / sample_count;
    gyro_z_offset = (float)sum_gz / sample_count;
}

// 定义一个全局变量或静态结构体来保存当前角度，因为陀螺仪计算的是相对变化量
volatile float gyro_angle = 0; // X轴角度


// 根据角速度求的角度
void MPU6050_Get_GyroAngle(float dt)
{
    short gx, gy, gz;
    
    // 1. 获取原始数据
    MPU6050_Get_Gyro(&gx, &gy, &gz);

    // 2. 转换为实际角速度 (度/秒)
    // 注意强制类型转换，或者除以 16.4f
    float gyro_x_speed = (gx - gyro_x_offset) / 16.4f;

    // 3. 积分：当前角度 = 上次角度 + (角速度 * 时间)
    // 注意：这里需要处理一下零点漂移（静止时可能有微小的输出），简易版先忽略
    
    // 如果数值太小（比如在静止时的噪声），可以强制归零，这叫死区处理
    if(fabs(gyro_x_speed) < 0.3f) gyro_x_speed = 0;  

    gyro_angle += gyro_x_speed * dt; 
}


/*================== 加速度计获取角度 + 陀螺仪获取角度 + 滤波 ==================*/
/*========================= ||     ||     ||     || ==========================*/
/*================================= 互补滤波 =================================*/

volatile float fused_angle = 0; // 融合后的角度

void MPU6050_Get_FusedAngle(float dt)
{
    short gx, gy, gz;
    
    // 1. 获取加速度计计算出的角度 
    MPU6050_Get_AccelAngle();  // 直接测量结果

    // 2. 获取角速度原始数据并扣除零偏，转换为 度/秒
    MPU6050_Get_Gyro(&gx, &gy, &gz);
    float gyro_x_speed = (gx - gyro_x_offset) / 16.4f;

    // 3. 互补滤波核心公式
    // 最终值 = 0.98 * (旧值 + 变化量) + 0.02 * (稳定参考值)
    // 其中变化值就是原始数据的积分，稳定参考值就是直接测量结果，旧值就是上一次测量结果（本身）
    fused_angle = 0.98f * (fused_angle + gyro_x_speed * dt) + 0.02f * accel_angle;
}


/*==================================== 融合 ====================================*/
/*========================== ||     ||     ||     || ===========================*/
/*================================== 姿态解算 ==================================*/

float gyro_x_speed;  // x轴角速度(直立环需要)
float gyro_z_speed;  // Z轴角速度（转向环用）

void MPU6050_get_FusedAngle_Optimized(float dt)
{
    uint8_t buff[14];
    short ax, ay, az, gx, gy, gz;
    float accel_angle_local;
    
    // 1. 一次性读取 14 个字节 (Accel X/Y/Z, Temp, Gyro X/Y/Z)
    if (MPU6050_ReadBytes(MPU_ACCEL_XOUTH_REG, buff, 14) != 0) {
        return; // 读取失败直接跳过
    }
    // 2. 原始数据解析 (大端转小端)
    ax = ((uint16_t)buff[0] << 8) | buff[1];
    ay = ((uint16_t)buff[2] << 8) | buff[3];
    az = ((uint16_t)buff[4] << 8) | buff[5];
    // buff[6][7] 是温度数据，如果不显示可以不管
    gx = ((uint16_t)buff[8] << 8) | buff[9];
    gy = ((uint16_t)buff[10] << 8) | buff[11];
    gz = ((uint16_t)buff[12] << 8) | buff[13];

    // 加速度计获取的角度（直接测量值）
    accel_angle_local = atan2((float)ay, (float)az) * 180.0f / PI - 1.5f;
    // 陀螺仪获取的X轴角速度（动态变化量）
    gyro_x_speed = (gx - gyro_x_offset) / 16.4f;
    // 陀螺仪获取的Z轴角速度
    gyro_z_speed = (gz - gyro_z_offset) / 16.4f;
    // 互补滤波
    fused_angle = 0.98f * (fused_angle + gyro_x_speed * dt) + 0.02f * accel_angle_local;
}
