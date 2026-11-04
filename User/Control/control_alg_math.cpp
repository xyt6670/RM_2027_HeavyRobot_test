/**
 * @file control_alg_math.cpp
 * @author yssickjgd 1345578933@qq.com
 * @brief 一些数学
 * @version 1.1
 * @date 2023-08-29 0.1 23赛季定稿
 * @date 2023-11-10 1.1 修改成cpp
 *
 * @copyright Copyright (c) 2023
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "control_alg_math.h"
/*
 * 【中文导读 5/5：辅助工具箱】这里不是 ARM 官方 arm_math 的源文件，是中科大的封装。
 * 电机 Data_Process -> Math_Endian_Reverse_16：把反馈中的高低字节换成 MCU 数值。
 * PID/电机还调用 Math_Abs、Math_Constrain：这些模板实现在 control_alg_math.h，请去头文件找。
 * control_alg_math.h 包含 arm_math.h；本文件 Math_Sinc 使用 CMSIS-DSP 的 arm_sin_f32。
 * 电机的功率限制还使用 arm_sqrt_f32；PID 的主要计算由 control_alg_pid.cpp 自己实现。
 * 求和、32位换序、映射等都是备用工具，不表示当前3508每个周期都会执行它们。
 */

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 布尔值反转
 *
 * @param Value 布尔值地址
 */
void Math_Boolean_Logical_Not(bool *Value)
{
    if (*Value == false)
    {
        *Value = true;
    }
    else if (*Value == true)
    {
        *Value = false;
    }
}

/**
 * @brief 16位大小端转换
 *
 * @param Address 地址
 */
void Math_Endian_Reverse_16(void *Address)
{
    // C++函数重载：同名的另一个版本有两个参数；编译器按参数列表区分。
    // 这个单参数版本直接修改 Address 指向的原存储区。
    uint8_t *temp_address_8 = (uint8_t *) Address;
    uint16_t *temp_address_16 = (uint16_t *) Address;
    *temp_address_16 = temp_address_8[0] << 8 | temp_address_8[1];
}

/**
 * @brief 16位大小端转换
 *
 * @param Source 源数据地址
 * @param Destination 目标存储地址
 * @return uint16_t 结果
 */
uint16_t Math_Endian_Reverse_16(void *Source, void *Destination)
{
    // 当前电机 Data_Process 调的是此双参数版本。例：收到 03 E8，解析为1000。
    // Source 是原始字节地址，Destination 是结果变量地址；void* 表示暂不指定数据类型。
    // (uint8_t*) 把地址视为字节指针，随后可用 [0]、[1] 读取两个字节。
    uint8_t *temp_address_8 = (uint8_t *) Source;
    uint16_t temp_address_16;
    temp_address_16 = temp_address_8[0] << 8 | temp_address_8[1];

    if (Destination != nullptr)
    {
        uint8_t *temp_source, *temp_destination;
        temp_source = (uint8_t *) Source;
        temp_destination = (uint8_t *) Destination;

        temp_destination[0] = temp_source[1];
        temp_destination[1] = temp_source[0];
    }

    return temp_address_16;
}

/**
 * @brief 32位大小端转换
 *
 * 
 * @param Address 地址
 */
void Math_Endian_Reverse_32(void *Address)
{
    uint8_t *temp_address_8 = (uint8_t *) Address;
    uint32_t *temp_address_32 = (uint32_t *) Address;
    *temp_address_32 = temp_address_8[0] << 24 | temp_address_8[1] << 16 | temp_address_8[2] << 8 | temp_address_8[3];
}

/**
 * @brief 32位大小端转换
 *
 * @param Source 源数据地址
 * @param Destination 目标存储地址
 * @return uint32_t 结果
 */
uint32_t Math_Endian_Reverse_32(void *Source, void *Destination)
{
    uint8_t *temp_address_8 = (uint8_t *) Source;
    uint32_t temp_address_32;
    temp_address_32 = temp_address_8[0] << 24 | temp_address_8[1] << 16 | temp_address_8[2] << 8 | temp_address_8[3];

    if (Destination != nullptr)
    {
        uint8_t *temp_source, *temp_destination;
        temp_source = (uint8_t *) Source;
        temp_destination = (uint8_t *) Destination;

        temp_destination[0] = temp_source[3];
        temp_destination[1] = temp_source[2];
        temp_destination[2] = temp_source[1];
        temp_destination[3] = temp_source[0];
    }

    return temp_address_32;
}

/**
 * @brief 求和
 *
 * @param Address 起始地址
 * @param Length 被加的数据的数量, 注意不是字节数
 * @return uint8_t 结果
 */
uint8_t Math_Sum_8(uint8_t *Address, uint32_t Length)
{
    uint8_t sum = 0;
    for (int i = 0; i < Length; i++)
    {
        sum += Address[i];
    }
    return (sum);
}

/**
 * @brief 求和
 *
 * @param Address 起始地址
 * @param Length 被加的数据的数量, 注意不是字节数
 * @return uint16_t 结果
 */
uint16_t Math_Sum_16(uint16_t *Address, uint32_t Length)
{
    uint16_t sum = 0;
    for (int i = 0; i < Length; i++)
    {
        sum += Address[i];
    }
    return (sum);
}

/**
 * @brief 求和
 *
 * @param Address 起始地址
 * @param Length 被加的数据的数量, 注意不是字节数
 * @return uint32_t 结果
 */
uint32_t Math_Sum_32(uint32_t *Address, uint32_t Length)
{
    uint32_t sum = 0;
    for (int i = 0; i < Length; i++)
    {
        sum += Address[i];
    }
    return (sum);
}

/**
 * @brief sinc函数的实现
 *
 * @param x 输入
 * @return float 输出
 */
float Math_Sinc(float x)
{
    // 仅调用本函数时才需要算 sinc；当前电流模式的3508主链没有调用它。
    // 分母为0则按极限求法
    if (Math_Abs(x) <= 2.0f * FLT_EPSILON)
    {
        return (1.0f);
    }

    return (arm_sin_f32(x) / x);
}

/**
 * @brief 将浮点数映射到整型
 *
 * @param x 浮点数
 * @param Float_1 浮点数1
 * @param Float_2 浮点数2
 * @param Int_1 整型1
 * @param Int_2 整型2
 * @return int32_t 整型
 */
int32_t Math_Float_To_Int(float x, float Float_1, float Float_2, int32_t Int_1, int32_t Int_2)
{
    float tmp = (x - Float_1) / (Float_2 - Float_1);
    int32_t out = tmp * (float) (Int_2 - Int_1) + Int_1;
    return (out);
}

/**
 * @brief 将整型映射到浮点数
 *
 * @param x 整型
 * @param Int_1 整型1
 * @param Int_2 整型2
 * @param Float_1 浮点数1
 * @param Float_2 浮点数2
 * @return float 浮点数
 */
float Math_Int_To_Float(int32_t x, int32_t Int_1, int32_t Int_2, float Float_1, float Float_2)
{
    float tmp = (float) (x - Int_1) / (float) (Int_2 - Int_1);
    float out = tmp * (Float_2 - Float_1) + Float_1;
    return (out);
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
