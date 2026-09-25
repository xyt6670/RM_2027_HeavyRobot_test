/**
 * @file alg_pid.cpp
 * @author yssickjgd 1345578933@qq.com
 * @brief PID算法
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 *
 * @copyright USTC-RoboWalker (c) 2023
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "alg_pid.h"
/*
 * 【中文导读 4/5：PID计算器】本文件不配置 CAN，也不发送报文。
 * Class_PID 是类型；电机类中的 PID_Angle、PID_Omega 是两个独立的 PID 对象。
 * 电机 cpp 的 PID_Calculate 在速度/角度模式下给 PID 设置 Target、Now，再调用计算函数。
 * 计算函数把结果存入本对象 Out，调用方通过 Get_Out() 取出。Get/Set 定义在 alg_pid.h。
 * 当前 app_motor.cpp 选择 CURRENT 模式：电机 PID_Calculate 直接跳过闭环计算，
 * 所以当前没有调用这里的周期 PID 计算，也没有初始化速度/角度 PID 参数。
 * 将来做速度控制，再学习 motor_3508.PID_Omega.Init(...) 及固定计算周期 D_T。
 */

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief PID初始化
 *
 * @param __K_P P值
 * @param __K_I I值
 * @param __K_D D值
 * @param __K_F 前馈
 * @param __I_Out_Max 积分限幅
 * @param __Out_Max 输出限幅
 * @param __D_T 时间片长度
 * @param __Dead_Zone 死区误差阈值
 * @param __I_Variable_Speed_A 变速积分误差阈值A
 * @param __I_Variable_Speed_B 变速积分误差阈值B
 * @param __I_Separate_Threshold 积分分离误差阈值
 * @param __D_First 是否开启微分先行
 */
void Class_PID::Init(float __K_P, float __K_I, float __K_D, float __K_F, float __I_Out_Max, float __Out_Max, float __D_T, float __Dead_Zone, float __I_Variable_Speed_A, float __I_Variable_Speed_B, float __I_Separate_Threshold, Enum_PID_D_First __D_First)
{
    // Class_PID::Init 表示“Class_PID 类的 Init 成员函数”。:: 表示所属范围。
    // 左边 K_P 属于当前对象，相当于 this->K_P；右边 __K_P 是调用时传入的形参。
    K_P = __K_P;
    K_I = __K_I;
    K_D = __K_D;
    K_F = __K_F;
    I_Out_Max = __I_Out_Max;
    Out_Max = __Out_Max;
    D_T = __D_T;
    Dead_Zone = __Dead_Zone;
    I_Variable_Speed_A = __I_Variable_Speed_A;
    I_Variable_Speed_B = __I_Variable_Speed_B;
    I_Separate_Threshold = __I_Separate_Threshold;
    D_First = __D_First;
}

/**
 * @brief PID调整值, 计算周期与D_T相同
 *
 * @return float 输出值
 */
void Class_PID::TIM_Calculate_PeriodElapsedCallback()
{
    // 普通成员函数，需要明确调用；TIM 字样不会让它自动成为 HAL 定时器回调。
    // 原注释写 return float，但实际返回类型是 void；结果存到成员 Out，再由 Get_Out 读取。
    // P输出
    float p_out = 0.0f;
    // I输出
    float i_out = 0.0f;
    // D输出
    float d_out = 0.0f;
    // F输出
    float f_out = 0.0f;
    // 误差
    float error;
    // 绝对值误差
    float abs_error;
    // 线性变速积分
    float speed_ratio;

    error = Target - Now;
    abs_error = Math_Abs(error);

    // 判断死区
    if (abs_error < Dead_Zone)
    {
        Target = Now;
        error = 0.0f;
        abs_error = 0.0f;
    }
    else if (error > 0.0f && abs_error > Dead_Zone)
    {
        error -= Dead_Zone;
    }
    else if (error < 0.0f && abs_error > Dead_Zone)
    {
        error += Dead_Zone;
    }

    // 计算p项

    p_out = K_P * error;

    // 计算i项

    if (I_Variable_Speed_A == 0.0f && I_Variable_Speed_B == 0.0f)
    {
        // 非变速积分
        speed_ratio = 1.0f;
    }
    else
    {
        // 变速积分
        if (abs_error <= I_Variable_Speed_A)
        {
            speed_ratio = 1.0f;
        }
        else if (I_Variable_Speed_A < abs_error && abs_error < I_Variable_Speed_B)
        {
            speed_ratio = (I_Variable_Speed_B - abs_error) / (I_Variable_Speed_B - I_Variable_Speed_A);
        }
        else if (abs_error >= I_Variable_Speed_B)
        {
            speed_ratio = 0.0f;
        }
    }
    // Ki 为 0 时不积分，既避免 I_Out_Max / Ki 除零，也避免重新启用
    // Ki 时突然恢复之前累积的积分。
    if (K_I == 0.0f)
    {
        Integral_Error = 0.0f;
        i_out = 0.0f;
    }
    else
    {
        if (I_Separate_Threshold == 0.0f ||
            abs_error < I_Separate_Threshold)
        {
            Integral_Error += speed_ratio * D_T * error;
        }
        else
        {
            // 误差超过积分分离阈值时清除积分。
            Integral_Error = 0.0f;
        }

        // 先更新积分再限幅，保证本次计算得到的积分输出也不会越界。
        if (I_Out_Max != 0.0f)
        {
            const float integral_error_max = Math_Abs(I_Out_Max / K_I);
            Math_Constrain(&Integral_Error,
                           -integral_error_max,
                           integral_error_max);
        }
        i_out = K_I * Integral_Error;
    }

    // 计算d项

    if (D_First == PID_D_First_DISABLE)
    {
        // 没有微分先行
        d_out = K_D * (error - Pre_Error) / D_T;
    }
    else
    {
        // 微分先行使能
        d_out = -K_D * (Now - Pre_Now) / D_T;
    }

    // 计算前馈

    f_out = (Target - Pre_Target) * K_F;

    // 计算输出

    Out = p_out + i_out + d_out + f_out;

    // 输出限幅
    if (Out_Max != 0.0f)
    {
        Math_Constrain(&Out, -Out_Max, Out_Max);
    }

    // 善后工作
    Pre_Now = Now;
    Pre_Target = Target;
    Pre_Out = Out;
    Pre_Error = error;
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
