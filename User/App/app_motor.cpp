/*
 * 【中文导读 1/5：先读本文件】应用层，负责把四个库接起来。
 * 阅读顺序：app_motor.cpp -> drv_can.cpp -> dvc_motor_dji.cpp 的 GM6020/C620 部分
 *           -> alg_pid.cpp（以后切换速度模式再细看）-> drv_math.cpp（按需查）。
 *
 * 从哪里来：Core/Src/main.c 初始化时调用 App_Motor_Init，循环调用 App_Motor_Poll。
 * 到哪里去：本文件调用两个电机对象的成员函数，以及 drv_can.cpp 的 CAN_Init/CAN_Send_Data。
 * 接收路线：HAL中断 -> drv_can.cpp -> 本文件 CAN_Motor_Callback -> 电机类解析。
 * 发送路线：本文件 Poll -> 电机类计算并填数组 -> 本文件调用 CAN_Send_Data 发整帧。
 *
 * C/C++连接：app_motor.h 用 extern "C" 声明两个 App 函数，main.c 可以正常调用它们。
 * 函数内部仍按 C++ 编译，所以可以使用 motor_3508 这个对象。
 * .h 主要提供声明；.cpp 提供实现；加入 Keil 文件组后，分别编译成 .o 再链接。
 */
#include "App/app_motor.h"
#include "2_Device/Motor/Motor_DJI/dvc_motor_dji.h"
#include "dvc_serialplot.h"
#include "drv_uart.h"
#include "1_Middleware/2_Algorithm/PID/alg_pid.h"
#include "1_Middleware/1_Driver/CAN/drv_can.h"

// 主循环轮询：控制与绘图独立；不是硬件定时器的严格实时周期。
static const uint32_t CONTROL_PERIOD_MS = 1U;
static const uint32_t PLOT_PERIOD_MS = 10U;
static const uint32_t MOTOR_FEEDBACK_TIMEOUT_MS = 100U;
// 当前只调GM6020，暂时关闭3508，避免旧的串口命令让另一台电机动作。
static const bool ENABLE_MOTOR_3508 = true;
static const bool ENABLE_MOTOR_6020 = true;
static const float MOTOR_3508_CURRENT_LIMIT_A = 2.0f;
// GM6020 双环的两级输出单位不同：速度环输出 A，电流环输出 V。
static const float MOTOR_6020_CURRENT_LIMIT_A = 5.0f;
static const float MOTOR_6020_OMEGA_OUT_LIMIT_A = 1.5f;
// 保留积分保护上限；初始 Ki=0，先单独调 Kp，再通过串口逐步加入 Ki。
static const float MOTOR_6020_OMEGA_I_OUT_LIMIT_A = 0.5f;
static const float MOTOR_6020_VOLTAGE_LIMIT_V = 12.0f;
static const float MOTOR_6020_CURRENT_I_OUT_LIMIT_V = 8.0f;
static const float MOTOR_6020_ANGLE_I_OUT_LIMIT_S = 0.2f;
static const float MOTOR_6020_ANGLE_OUT_LIMIT_S = 10.0f;
// 角度误差小于 0.01 rad（约 0.57°）时，外环不再给出目标速度。
static const float MOTOR_6020_ANGLE_DEAD_ZONE_RAD = 0.02f;
static const float MOTOR_6020_ANGLE_KP = 5.0f;
static const float MOTOR_6020_ANGLE_KI = 0.0f;
static const float MOTOR_6020_OMEGA_KP = 0.05f;
static const float MOTOR_6020_OMEGA_KI = 0.0f;
static const float MOTOR_6020_CURRENT_KP = 1.0f;
static const float MOTOR_6020_CURRENT_KI = 100.0f;
static uint32_t last_control_tick, last_plot_tick;
static bool motor_6020_target_initialized = false;
volatile uint32_t motor_control_missed_periods = 0;
volatile uint32_t serialplot_tx_count = 0;
volatile uint32_t serialplot_tx_error_count = 0;

static Class_Motor_DJI_C620 motor_3508;
static Class_Motor_DJI_GM6020 motor_6020;

float Now_Angle_6020 = 0.0f, Target_Angle_6020 = 0.0f;
float Applied_Angle_6020 = 0.0f;
float Now_Omega_3508 = 0.0f, Target_Omega_3508 = 0.0f;
float Now_Omega_6020 = 0.0f, Target_Omega_6020 = 0.0f;
float Applied_Omega_6020 = 0.0f;
float Now_Current_6020 = 0.0f, Target_Current_6020 = 0.0f;
float Target_Voltage_6020 = 0.0f;

volatile uint32_t motor_3508_rx_count = 0;
volatile uint32_t motor_6020_rx_count = 0;
volatile uint32_t motor_3508_last_rx_tick = 0;
volatile uint32_t motor_6020_last_rx_tick = 0;
volatile bool motor_3508_online = false;
volatile bool motor_6020_online = false;
volatile uint32_t serialplot_rx_count = 0;
volatile uint32_t serialplot_rx_valid_count = 0;
volatile uint32_t serialplot_rx_reject_count = 0;
volatile uint16_t serialplot_rx_last_length = 0;
volatile int8_t serialplot_rx_last_index = -1;
volatile float serialplot_rx_last_value = 0.0f;

static char Variable_Assignment_List[][SERIALPLOT_RX_VARIABLE_ASSIGNMENT_MAX_LENGTH] = {
    // 3508速度环
    "po",
    "io",
    "do",
    "fo",
    // 6020角度环：angle -> omega
    "pa6020",
    "ia6020",
    "da6020",
    "fa6020",
    // 6020速度环：omega -> current
    "pw6020",
    "iw6020",
    "dw6020",
    "fw6020",
    // 6020电流环：current -> voltage
    "pc6020",
    "ic6020",
    "dc6020",
    "fc6020",
    "omega3508",
    "angle6020",
    "omega6020",
};//串口绘图回传的时候我们的那些指令

static const uint8_t VARIABLE_ASSIGNMENT_COUNT =
    sizeof(Variable_Assignment_List) / sizeof(Variable_Assignment_List[0]);

// 只让1号C620反馈ID 0x201和GM6020反馈ID 0x209进入FIFO0。
static const uint16_t CAN1_ACCEPTED_STD_IDS[] = {0x201, 0x209};

// 调试器观察/修改用：单位 A；volatile 不代表线程安全，也不提供互斥。
volatile float motor_target_current_a = 0.3f;
volatile float motor_speed_rad_s = 0.0f;
volatile uint32_t motor_rx_count = 0;
volatile uint32_t motor_tx_error_count = 0;
static Class_Serialplot serialplot;

static void CAN_Motor_Callback(Struct_CAN_Rx_Buffer *rx)
{
    if (rx->Header.IDE == CAN_ID_STD && rx->Header.RTR == CAN_RTR_DATA &&
        rx->Header.DLC == 8 && rx->Header.StdId == 0x201)
    {
        motor_3508.CAN_RxCpltCallback(rx->Data);
        motor_3508_last_rx_tick = HAL_GetTick();
        ++motor_3508_rx_count;
        ++motor_rx_count;
    }
    else if (rx->Header.IDE == CAN_ID_STD && rx->Header.RTR == CAN_RTR_DATA &&
             rx->Header.DLC == 8 && rx->Header.StdId == 0x209)
    {
        motor_6020.CAN_RxCpltCallback(rx->Data);
        motor_6020_last_rx_tick = HAL_GetTick();
        ++motor_6020_rx_count;
        ++motor_rx_count;
    }   
}

UART_Call_Back Serialplot_UART_RxCpltCallback = [](uint8_t *Buffer, uint16_t Length) {
    // 接受“变量名=数值#”，例如 pa6020=0.05#、angle6020=1.2#。
    ++serialplot_rx_count;
    serialplot_rx_last_length = Length;
    serialplot_rx_last_index = -2;
    serialplot_rx_last_value = 0.0f;

    if (Buffer == NULL || Length >= UART_BUFFER_SIZE)
    {
        ++serialplot_rx_reject_count;
        return;
    }

    // 兼容串口工具自动添加的 CR/LF："omega6020=1.0#\r\n"。
    while (Length > 0U &&
           (Buffer[Length - 1U] == '\r' || Buffer[Length - 1U] == '\n'))
    {
        --Length;
    }
    if (Length < 4U || Buffer[Length - 1U] != '#')
    {
        ++serialplot_rx_reject_count;
        return;
    }

    uint16_t equal_pos = Length;
    for (uint16_t i = 1U; i < Length - 1U; ++i)
    {
        if (Buffer[i] == '=')
        {
            equal_pos = i;
            break;
        }

        const bool valid_name_char =
            (Buffer[i] >= 'a' && Buffer[i] <= 'z') ||
            (Buffer[i] >= 'A' && Buffer[i] <= 'Z') ||
            (Buffer[i] >= '0' && Buffer[i] <= '9') ||
            Buffer[i] == '_';
        if (!valid_name_char)
        {
            ++serialplot_rx_reject_count;
            return;
        }
    }
    if (equal_pos >= Length - 2U)
    {
        ++serialplot_rx_reject_count;
        return;
    }

    uint16_t first = equal_pos + 1U;
    if (Buffer[first] == '-')
        ++first;
    if (first >= Length - 1U)
    {
        ++serialplot_rx_reject_count;
        return;
    }

    bool digit_seen = false;
    bool dot_seen = false;
    for (uint16_t i = first; i < Length - 1; ++i)
    {
        if (Buffer[i] >= '0' && Buffer[i] <= '9') digit_seen = true;
        else if (Buffer[i] == '.' && !dot_seen) dot_seen = true;
        else
        {
            ++serialplot_rx_reject_count;
            return;
        }
    }
    if (!digit_seen)
    {
        ++serialplot_rx_reject_count;
        return;
    }
    Buffer[Length] = 0;
    serialplot.UART_RxCpltCallback(Buffer);
    const float value = (float)serialplot.Get_Variable_Value();
    const int8_t variable_index = serialplot.Get_Variable_Index();
    serialplot_rx_last_index = variable_index;
    serialplot_rx_last_value = value;
    if (variable_index < 0)
    {
        ++serialplot_rx_reject_count;
        return;
    }
    ++serialplot_rx_valid_count;

    switch (variable_index)
    {
        // 电机调PID
        case(0):
        {
            motor_3508.PID_Omega.Set_K_P(value);
        }
        break;
        case(1):
        {
            motor_3508.PID_Omega.Set_K_I(value);
        }
        break;
        case(2):
        {
            motor_3508.PID_Omega.Set_K_D(value);
        }
        break;
        case(3):
        {
            motor_3508.PID_Omega.Set_K_F(value);
        }
        break;
        case(4):
        {
            motor_6020.PID_Angle.Set_K_P(value);
        }
        break;
        case(5):
        {
            motor_6020.PID_Angle.Set_K_I(value);
        }
        break;
        case(6):
        {
            motor_6020.PID_Angle.Set_K_D(value);
        }
        break;
        case(7):
        {
            motor_6020.PID_Angle.Set_K_F(value);
        }
        break;
        case(8):
        {
            motor_6020.PID_Omega.Set_K_P(value);
        }
        break;
        case(9):
        {
            // 修改 Ki 前清除历史积分，防止旧误差在新参数下造成电流突跳。
            motor_6020.PID_Omega.Set_Integral_Error(0.0f);
            motor_6020.PID_Omega.Set_K_I(value);
        }
        break;
        case(10):
        {
            motor_6020.PID_Omega.Set_K_D(value);
        }
        break;
        case(11):
        {
            motor_6020.PID_Omega.Set_K_F(value);
        }
        break;
        case(12):
        {
            motor_6020.PID_Current.Set_K_P(value);
        }
        break;
        case(13):
        {
            motor_6020.PID_Current.Set_K_I(value);
        }
        break;
        case(14):
        {
            motor_6020.PID_Current.Set_K_D(value);
        }
        break;
        case(15):
        {
            motor_6020.PID_Current.Set_K_F(value);
        }
        break;
        case(16):
        {
            Target_Omega_3508 = value;
        }
        break;
        case(17):
        {
            Target_Angle_6020 = value;
        }
        break;
        case(18):
        {
            Target_Omega_6020 = value;
        }
        break;
    }
};
void App_Motor_Init(void)
{
    // PID速度环输出的单位都是A；Out_Max与电机类Current_Max形成双重限流。
    motor_3508.PID_Omega.Init(3.0f, 0.1f, 0.0f, 0.0f,
                              0.0f, MOTOR_3508_CURRENT_LIMIT_A,
                              CONTROL_PERIOD_MS * 0.001f);
    motor_3508.Init(&hcan1, Motor_DJI_ID_0x201,
                    Motor_DJI_Control_Method_OMEGA, 3591.0f / 187.0f,
                    Motor_DJI_Power_Limit_Status_DISABLE,
                    MOTOR_3508_CURRENT_LIMIT_A);

    motor_6020.PID_Angle.Init(MOTOR_6020_ANGLE_KP,
                              MOTOR_6020_ANGLE_KI,
                              0.0f, 0.0f,
                              MOTOR_6020_ANGLE_I_OUT_LIMIT_S,
                              MOTOR_6020_ANGLE_OUT_LIMIT_S,
                              CONTROL_PERIOD_MS * 0.001f,
                              MOTOR_6020_ANGLE_DEAD_ZONE_RAD);
    // 速度外环：输入 rad/s，输出目标电流 A。
    motor_6020.PID_Omega.Init(MOTOR_6020_OMEGA_KP,
                              MOTOR_6020_OMEGA_KI,
                              0.0f, 0.0f,
                              MOTOR_6020_OMEGA_I_OUT_LIMIT_A,
                              MOTOR_6020_OMEGA_OUT_LIMIT_A,
                              CONTROL_PERIOD_MS * 0.001f);
    // 电流内环：输入 A，输出目标电压 V。
    motor_6020.PID_Current.Init(MOTOR_6020_CURRENT_KP,
                                MOTOR_6020_CURRENT_KI,
                                0.0f, 0.0f,
                                MOTOR_6020_CURRENT_I_OUT_LIMIT_V,
                                MOTOR_6020_VOLTAGE_LIMIT_V,
                                CONTROL_PERIOD_MS * 0.001f);
    motor_6020.Init(&hcan1, Motor_DJI_ID_0x209,
                    Motor_DJI_Control_Method_ANGLE, 0.0f,
                    Motor_DJI_GM6020_Driver_Version_DEFAULT,
                    Motor_DJI_Power_Limit_Status_DISABLE,
                    MOTOR_6020_VOLTAGE_LIMIT_V,
                    MOTOR_6020_CURRENT_LIMIT_A);
    // 不带括号的 CAN_Motor_Callback 是把函数地址交出去，此处不是执行回调。
    // drv_can.cpp 会把它存入 CAN1_Manage_Object.Callback_Function，收到帧才执行。
    CAN_Init_With_Filters(&hcan1, CAN_Motor_Callback,
                          CAN1_ACCEPTED_STD_IDS,
                          static_cast<uint8_t>(sizeof(CAN1_ACCEPTED_STD_IDS) /
                                               sizeof(CAN1_ACCEPTED_STD_IDS[0])));
    serialplot.Init(&huart1, VARIABLE_ASSIGNMENT_COUNT,
                    (char **)Variable_Assignment_List);
    last_control_tick = last_plot_tick = HAL_GetTick();
    motor_6020_target_initialized = false;
    motor_control_missed_periods = 0;
    serialplot_tx_count = 0;
    serialplot_tx_error_count = 0;
    serialplot_rx_count = 0;
    serialplot_rx_valid_count = 0;
    serialplot_rx_reject_count = 0;
    serialplot_rx_last_length = 0;
    serialplot_rx_last_index = -1;
    serialplot_rx_last_value = 0.0f;

    // 对象及参数准备完后，最后启动接收，避免中断访问未初始化对象。
    UART_Init(&huart1, Serialplot_UART_RxCpltCallback,
              SERIALPLOT_RX_VARIABLE_ASSIGNMENT_MAX_LENGTH);
}

// 【main.c -> 这里】主循环反复调用；内部用 tick 判断是否到了下一次处理时间。
// 名字为 TIM_xxx 的电机函数也是普通函数，这里直接调用，并没有自动建立定时器中断。
void App_Motor_Poll(void)
{
    const uint32_t now = HAL_GetTick();
    const uint32_t elapsed = (uint32_t)(now - last_control_tick);

    // 控制优先，不能因为UART忙而跳过PID和CAN发送。
    if (elapsed >= CONTROL_PERIOD_MS)
    {
        const uint32_t periods = elapsed / CONTROL_PERIOD_MS;
        last_control_tick += periods * CONTROL_PERIOD_MS;
        motor_control_missed_periods += periods - 1U;
        motor_3508_online = motor_3508_rx_count > 0U &&
            (uint32_t)(now - motor_3508_last_rx_tick) <= MOTOR_FEEDBACK_TIMEOUT_MS;
        motor_6020_online = motor_6020_rx_count > 0U &&
            (uint32_t)(now - motor_6020_last_rx_tick) <= MOTOR_FEEDBACK_TIMEOUT_MS;

        if (ENABLE_MOTOR_3508)
        {
            if (motor_3508_online)
            {
                motor_3508.Set_Control_Method(Motor_DJI_Control_Method_OMEGA);
                motor_3508.Set_Target_Omega(Target_Omega_3508);
            }
            else
            {
                motor_3508.Set_Control_Method(Motor_DJI_Control_Method_CURRENT);
                motor_3508.Set_Target_Current(0.0f);
                motor_3508.PID_Omega.Set_Integral_Error(0.0f);
            }
            motor_3508.TIM_Calculate_PeriodElapsedCallback();
            if (CAN_Send_Data(&hcan1, 0x200, CAN1_0x200_Tx_Data, 8) != HAL_OK)
                ++motor_tx_error_count;
        }

        if (ENABLE_MOTOR_6020)
        {
            if (motor_6020_online)
            {
                // 反馈刚恢复时先保持当前位置，避免追逐上一次留下的角度目标。
                if (!motor_6020_target_initialized)
                {
                    Target_Angle_6020 = motor_6020.Get_Now_Angle();
                    motor_6020.PID_Angle.Set_Integral_Error(0.0f);
                    motor_6020.PID_Omega.Set_Integral_Error(0.0f);
                    motor_6020.PID_Current.Set_Integral_Error(0.0f);
                    motor_6020_target_initialized = true;
                }
                motor_6020.Set_Control_Method(Motor_DJI_Control_Method_ANGLE);
                motor_6020.Set_Target_Angle(Target_Angle_6020);
            }
           else
           {
               motor_6020.Set_Control_Method(Motor_DJI_Control_Method_CURRENT);
               motor_6020.Set_Target_Current(0.0f);
               motor_6020.PID_Omega.Set_Integral_Error(0.0f);
               motor_6020.PID_Current.Set_Integral_Error(0.0f);
               motor_6020.PID_Angle.Set_Integral_Error(0.0f);
               motor_6020_target_initialized = false;
           }
           motor_6020.TIM_Calculate_PeriodElapsedCallback();

            // 0x209反馈ID + 2023电流驱动，对应0x2FF控制帧的前两个字节。
            if (CAN_Send_Data(&hcan1, 0x2FF, CAN1_0x2ff_Tx_Data, 8) != HAL_OK)
                ++motor_tx_error_count;
        }
    }
    
    // 打包前确认DMA空闲；只跳过绘图，不影响上面的控制。
    if ((uint32_t)(now - last_plot_tick) >= PLOT_PERIOD_MS &&
        huart1.gState == HAL_UART_STATE_READY)
    {
        last_plot_tick = now;
        Now_Omega_3508 = motor_3508.Get_Now_Omega();
        Now_Angle_6020 = motor_6020.Get_Now_Angle();
        Now_Omega_6020 = motor_6020.Get_Now_Omega();
        Applied_Angle_6020 = motor_6020.Get_Target_Angle();
        Applied_Omega_6020 = motor_6020.Get_Target_Omega();
        Now_Current_6020 = motor_6020.Get_Now_Current();
        Target_Current_6020 = motor_6020.Get_Target_Current();
        Target_Voltage_6020 = motor_6020.Get_Target_Voltage();
        // 角度调试：前两路看位置，后两路同时确认电流内环能否跟随。
        serialplot.Set_Data(4, &Target_Angle_6020, &Now_Angle_6020,
                            &Target_Omega_3508, &Now_Omega_3508);
        serialplot.TIM_Write_PeriodElapsedCallback();
        if (TIM_UART_PeriodElapsedCallback() == HAL_OK)
            ++serialplot_tx_count;
        else
            ++serialplot_tx_error_count;
    }
}
