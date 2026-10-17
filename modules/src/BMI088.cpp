#include "BMI088.hpp"
#include "tx_api.h"
#include "bsp_pwm.hpp"
#include <cmath>

namespace BMI088
{
    /*---------------------------内部小工具---------------------------*/
    /** SPI 阻塞传输的超时时间 [ms]：够慢速 SPI 传完，又不会把线程挂死 */
    #define BMI088_SPI_TIMEOUT 100u

    /**
     * @brief 加速度计：数据有效位（bit7 = data ready）
     * @note 加速度计的 SPI 读时序要先返回 1 个无效字节，所以状态寄存器落在 buf[1]
     */
    static GPIO_TypeDef *CsPortOf(enum BMI088_SENSOR cs)
    {
        return (cs == BMI088_CS_ACC) ? BMI088_ACC_CS_GPIO_PORT : BMI088_GYRO_CS_GPIO_PORT;
    }

    static uint16_t CsPinOf(enum BMI088_SENSOR cs)
    {
        return (cs == BMI088_CS_ACC) ? BMI088_ACC_CS_PIN : BMI088_GYRO_CS_PIN;
    }

    /**
     * @brief BMI088 acc gyro 标定
     * @note 标定后的陀螺仪零偏存储在 Gyro_offset 中。
     * @attention 不管工作模式是blocking还是IT,标定时都是blocking模式,所以不用担心中断关闭后无法标定(RobotInit关闭了全局中断)
     * @attention 标定精度和等待时间有关。
     * @todo 将标定次数(等待时间)变为参数供设定
     * @section 整体流程为1.累加加速度数据计算gNrom()
     *                   2.累加陀螺仪数据计算零飘
     *                   3. 如果标定过程运动幅度过大,重新标定
     *                   4.保存标定参数
     */
    void cBMI088::Calibrate()
    {
        const int calib_samples = 4000; // 采样次数
        float gyro_sum[3] = {0.0f, 0.0f, 0.0f};
        gyro_data_t temp_gyro;

        // 1. 清除旧的 Offset，防止叠加
        Gyro_offset[0] = 0.0f;
        Gyro_offset[1] = 0.0f;
        Gyro_offset[2] = 0.0f;

        // 2. 循环采样
        for (int i = 0; i < calib_samples; i++)
        {
            ReadGyroData(&temp_gyro); // 这里读取的是原始值（因为Offset已清零）
            gyro_sum[0] += temp_gyro.x;
            gyro_sum[1] += temp_gyro.y;
            gyro_sum[2] += temp_gyro.z;
            
            tx_thread_sleep(1); // 等待一个 ThreadX tick；实际耗时由 tick 频率决定。
        }

        // 3. 计算平均值作为零偏
        Gyro_offset[0] = gyro_sum[0] / calib_samples;
        Gyro_offset[1] = gyro_sum[1] / calib_samples;
        Gyro_offset[2] = gyro_sum[2] / calib_samples;
        
        // 如果零偏过大（例如超过 0.1 rad/s），可能是在运动中标定的，应报错或丢弃
        if (fabs(Gyro_offset[0]) > 0.1f || fabs(Gyro_offset[1]) > 0.1f || fabs(Gyro_offset[2]) > 0.1f)
        {
            self_test.CALIBRATE_ERR = true;
            // 恢复默认值或保留上次值
            Gyro_offset[0] = BMI088_GYRO_PRE_CALI_OFFSET_X; 
            Gyro_offset[1] = BMI088_GYRO_PRE_CALI_OFFSET_Y;
            Gyro_offset[2] = BMI088_GYRO_PRE_CALI_OFFSET_Z;
        }
        else
        {
            self_test.CALIBRATE_ERR = false;
        }
    }

    void cBMI088::TemperatureControl(float target_temp)
    {
        float temp = 0.0f;
        ReadAccTemperature(&temp);

        // 温度没读到就不加热：加热电阻在 IMU 旁边，闭着眼睛加热会把芯片烤了
        if (self_test.ACC_DATA_ERR || temp < -40.0f || temp > 125.0f)
        {
            self_test.TEMP_CTRL_ERR = true;
            PWM_SetDutyRatio(&HEATING_RESISTANCE_TIM, 0.0f, HEATING_RESISTANCE_CHANNEL);
            return;
        }
        self_test.TEMP_CTRL_ERR = false;

        // 目标温度 → PID → 占空比
        // TempPid.maxOut 定义为"满量程加热"，所以 result/maxOut 就是 [0,1] 的占空比
        TempPid.ref = target_temp;
        TempPid.fdb = temp;
        TempPid.UpdateResult();

        float duty = (TempPid.maxOut > 0.0f) ? (TempPid.result / TempPid.maxOut) : 0.0f;
        if (duty < 0.0f)
            duty = 0.0f;
        else if (duty > 1.0f)
            duty = 1.0f;

        PWM_SetDutyRatio(&HEATING_RESISTANCE_TIM, duty, HEATING_RESISTANCE_CHANNEL);
    }

    void cBMI088::VerifyAccChipID()
    {
        uint8_t pRxData[2]; //< 读取两个字节，第一个是需要丢弃的无效字节，第二个是芯片 ID

        ReadReg(BMI088_CS_ACC, ACC_CHIP_ID_ADDR, pRxData, 2); //< 读取加速度计chip id
        tx_thread_sleep(1);
        //< 如果chip id不等于预设值,则加速度计ID错误,初始化错误
        if (pRxData[1] != ACC_CHIP_ID_VAL)
        {
            self_test.ACC_CHIP_ID_ERR = true;
            self_test.INIT_ERR = true;
        }
        else if (pRxData[1] == ACC_CHIP_ID_VAL)
        {
            self_test.ACC_CHIP_ID_ERR = false;
        }
    }

    void cBMI088::VerifyGyroChipID()
    {
        uint8_t pRxData;                                                 //< 读取一个字节,chip id
        ReadReg(BMI088_CS_GYRO, GYRO_CHIP_ID_ADDR, &pRxData, 1); //< 读取陀螺仪chip id
        tx_thread_sleep(1);
        //< 如果chip id不等于预设值,则陀螺仪ID错误,初始化错误
        if (pRxData != GYRO_CHIP_ID_VAL)
        {
            self_test.GYRO_CHIP_ID_ERR = true;
            self_test.INIT_ERR = true;
        }
        else if (pRxData == GYRO_CHIP_ID_VAL)
        {
            self_test.GYRO_CHIP_ID_ERR = false;
        }
    }

    void cBMI088::VerifyAccData()
    {
        uint8_t status[2] = {0, 0};

        // ACC_STATUS(0x03) 的 bit7 是 data ready；加速度计前面还有一个无效字节，所以状态在 buf[1]
        ReadReg(BMI088_CS_ACC, ACC_STATUS_ADDR, status, 2);

        // 没有 ready 说明这一轮读到的是旧数据（或者 SPI 根本没通），标记数据不可信
        self_test.ACC_DATA_ERR = ((status[1] & 0x80u) == 0u);
    }

    void cBMI088::VerifyGyroData()
    {
        uint8_t status = 0;

        // GYRO_INT_STAT_1(0x0A) 的 bit7 是 data ready；陀螺仪读时序没有无效字节
        ReadReg(BMI088_CS_GYRO, GYRO_INT_STAT_1_ADDR, &status, 1);

        self_test.GYRO_DATA_ERR = ((status & 0x80u) == 0u);
    }

    void cBMI088::WriteReg(enum BMI088_SENSOR cs, uint8_t addr, uint8_t *data, uint8_t len)
    {
        if (data == nullptr || len == 0)
            return;

        GPIO_TypeDef *port = CsPortOf(cs);
        uint16_t pin = CsPinOf(cs);

        // 片选拉低：这片芯片开始听 SPI
        HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);

        // 写操作的地址最高位为 0（BMI088_SPI_WRITE_CODE = 0x7F 就是取低 7 位）
        uint8_t reg = addr & BMI088_SPI_WRITE_CODE;
        if (HAL_SPI_Transmit(&BMI088_SPI_HANDLE, &reg, 1, BMI088_SPI_TIMEOUT) != HAL_OK)
        {
            HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET); // 通信失败也要释放片选，否则总线一直占着
            return;
        }
        HAL_SPI_Transmit(&BMI088_SPI_HANDLE, data, len, BMI088_SPI_TIMEOUT);

        // 片选拉高：一次写事务结束
        HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
    }

    void cBMI088::ReadReg(enum BMI088_SENSOR cs, uint8_t addr, uint8_t *data, uint8_t len)
    {
        if (data == nullptr || len == 0)
            return;

        // 先把缓冲区清零：通信失败时返回 0 而不是未初始化的随机值
        for (uint8_t i = 0; i < len; i++)
            data[i] = 0;

        GPIO_TypeDef *port = CsPortOf(cs);
        uint16_t pin = CsPinOf(cs);

        HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);

        // 读操作的地址最高位为 1
        uint8_t reg = addr | BMI088_SPI_READ_CODE;
        if (HAL_SPI_Transmit(&BMI088_SPI_HANDLE, &reg, 1, BMI088_SPI_TIMEOUT) != HAL_OK)
        {
            HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
            return;
        }

        // 加速度计：地址之后先来 1 个无效字节，必须丢掉；因此真正有效的数据只有 len-1 个，
        // 且落在 data[1..len-1]（调用方传 len=2 读 chip id 就是这个约定）。
        // 陀螺仪：地址之后直接就是数据，data[0..len-1] 全部有效。
        if (cs == BMI088_CS_ACC)
        {
            uint8_t dummy = 0;
            HAL_SPI_Receive(&BMI088_SPI_HANDLE, &dummy, 1, BMI088_SPI_TIMEOUT);
            if (len > 1)
                HAL_SPI_Receive(&BMI088_SPI_HANDLE, data + 1, (uint16_t)(len - 1), BMI088_SPI_TIMEOUT);
        }
        else
        {
            HAL_SPI_Receive(&BMI088_SPI_HANDLE, data, len, BMI088_SPI_TIMEOUT);
        }

        HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
    }

    void cBMI088::Config()
    {
        tx_thread_sleep(10); //< 等待系统稳定

        /*-------------------------------------加速度计初始化-------------------------------------*/
        
        //< 先软重启，清空所有寄存器
        uint8_t pTxData;
        pTxData = ACC_SOFTRESET_VAL;
        WriteReg(BMI088_CS_ACC, ACC_SOFTRESET_ADDR, &pTxData, 1);
        tx_thread_sleep(100); //< 延时100ms,重启需要时间

        //< 打开加速度计电源
        pTxData = ACC_PWR_CTRL_ON;
        WriteReg(BMI088_CS_ACC, ACC_PWR_CTRL_ADDR, &pTxData, 1);
        tx_thread_sleep(150);

        //< 加速度计变成正常模式
        pTxData = ACC_PWR_CONF_ACT;
        WriteReg(BMI088_CS_ACC, ACC_PWR_CONF_ADDR, &pTxData, 1);
        tx_thread_sleep(10); //

        //< 测量范围
        pTxData = ACC_RANGE_6G;
        WriteReg(BMI088_CS_ACC, ACC_RANGE_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms

        pTxData = 0xAB;
        WriteReg(BMI088_CS_ACC, ACC_CONF_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms

        pTxData = 0x08;
        WriteReg(BMI088_CS_ACC, INT1_IO_CTRL_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms

        pTxData = 0x04;
        WriteReg(BMI088_CS_ACC, INT_MAP_DATA_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms

        /*-------------------------------------陀螺仪初始化-------------------------------------*/
        //< 先软重启，清空所有寄存器
        pTxData = GYRO_SOFTRESET_VAL;
        WriteReg(BMI088_CS_GYRO, GYRO_SOFTRESET_ADDR, &pTxData, 1);
        tx_thread_sleep(100); //< 延时100ms,重启需要时间

        pTxData = GYRO_RANGE_2000_DEG_S;
        WriteReg(BMI088_CS_GYRO, GYRO_RANGE_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms

        pTxData = GYRO_ODR_2000Hz_BANDWIDTH_230Hz | GYRO_LPM1_SUS;
        WriteReg(BMI088_CS_GYRO, GYRO_BANDWIDTH_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms

        pTxData = GYRO_LPM1_NOR;
        WriteReg(BMI088_CS_GYRO, GYRO_LPM1_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms

        pTxData = GYRO_DRDY_ON;
        WriteReg(BMI088_CS_GYRO, GYRO_INT_CTRL_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms

        pTxData = 0x00;
        WriteReg(BMI088_CS_GYRO, GYRO_INT3_INT4_IO_CONF_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms

        pTxData = 0x01;
        WriteReg(BMI088_CS_GYRO, GYRO_INT3_INT4_IO_MAP_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms
    }


    void cBMI088::ReadAccData(acc_data_t *data)
    {
        if (data == nullptr)
            return;

        // 多读 1 个字节：加速度计的第一个字节无效，有效数据落在 buf[1..6]
        uint8_t buf[ACC_XYZ_LEN + 1] = {0};
        ReadReg(BMI088_CS_ACC, ACC_X_LSB_ADDR, buf, ACC_XYZ_LEN + 1);

        // 小端：先低字节后高字节，拼成有符号 16 位
        const int16_t raw_x = (int16_t)(((uint16_t)buf[2] << 8) | buf[1]);
        const int16_t raw_y = (int16_t)(((uint16_t)buf[4] << 8) | buf[3]);
        const int16_t raw_z = (int16_t)(((uint16_t)buf[6] << 8) | buf[5]);

        // Config() 里配的是 ±6g，所以用 6g 的灵敏度（单位已经是 m/s²/LSB）
        data->x = (float)raw_x * IMU_ACCEL_6G_SEN;
        data->y = (float)raw_y * IMU_ACCEL_6G_SEN;
        data->z = (float)raw_z * IMU_ACCEL_6G_SEN;
    }

    void cBMI088::ReadGyroData(gyro_data_t *data)
    {
        if (data == nullptr)
            return;

        uint8_t buf[GYRO_XYZ_LEN] = {0};
        ReadReg(BMI088_CS_GYRO, GYRO_RATE_X_LSB_ADDR, buf, GYRO_XYZ_LEN);

        const int16_t raw_x = (int16_t)(((uint16_t)buf[1] << 8) | buf[0]);
        const int16_t raw_y = (int16_t)(((uint16_t)buf[3] << 8) | buf[2]);
        const int16_t raw_z = (int16_t)(((uint16_t)buf[5] << 8) | buf[4]);

        // Config() 里配的是 ±2000dps，灵敏度单位是 rad/s per LSB；
        // 再减去标定得到的零偏，输出就是可以直接给速度环用的角速度
        data->x = (float)raw_x * IMU_GYRO_2000_SEN - Gyro_offset[0];
        data->y = (float)raw_y * IMU_GYRO_2000_SEN - Gyro_offset[1];
        data->z = (float)raw_z * IMU_GYRO_2000_SEN - Gyro_offset[2];

        gyro_data = *data; ///< 同步到基类的缓存，其它线程可以直接取用
    }

    void cBMI088::ReadAccTemperature(float *temp)
    {
        if (temp == nullptr)
            return;

        // 同样多读 1 个字节丢掉无效字节：buf[1] = MSB、buf[2] = LSB
        uint8_t buf[TEMP_LEN + 1] = {0};
        ReadReg(BMI088_CS_ACC, TEMP_MSB_ADDR, buf, TEMP_LEN + 1);

        // 温度是 11 位有符号数：高 8 位 + 低 3 位
        const uint16_t raw11 = (uint16_t)(((uint16_t)buf[1] << 3) | (buf[2] >> 5));

        // 11 位补码：超过 1023 表示负数
        const int16_t temp11 = (raw11 > 1023u) ? (int16_t)(raw11 - 2048u) : (int16_t)raw11;

        // 0.125 ℃/LSB，23 ℃ 偏置
        *temp = (float)temp11 * TEMP_UNIT + TEMP_BIAS;
        acc_data.temperature = *temp;
    }

}
