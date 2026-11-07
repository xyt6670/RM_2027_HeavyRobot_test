#include "control_gimbal.h"
#include "device_motor_dm.h"

static Class_Motor_DM_Normal *yaw_motor = nullptr;
static Gimbal_Config config;
static bool origin_set, enabling, running, fault_latched;
static float origin_rad, target_rad;
static uint32_t enable_start_ms, last_service_ms;

void Gimbal_Init(Class_Motor_DM_Normal &yaw, const Gimbal_Config &settings)
{
    yaw_motor = &yaw;
    config = settings;
    origin_set = enabling = running = fault_latched = false;
    origin_rad = target_rad = 0.0f;
    enable_start_ms = 0U;
    last_service_ms = 0U - 20U;
}


void Gimbal_Update(const Gimbal_Command &command, uint32_t now_ms, float dt_s)
{
    if (yaw_motor == nullptr) return;
    Struct_Motor_DM_Rx_Data_Normal feedback;
    yaw_motor->GetSnapshot(feedback);
    now_ms = HAL_GetTick(); // 快照之后取时间，避免反馈时间戳比调度传入时间更新。
    bool fresh = feedback.Has_Feedback &&
        (uint32_t)(now_ms - feedback.Last_Rx_ms) < config.feedback_timeout_ms;
    bool error = feedback.Control_Status != Motor_DM_Control_Status_DISABLE &&
                 feedback.Control_Status != Motor_DM_Control_Status_ENABLE;
    bool service_due = (uint32_t)(now_ms - last_service_ms) >= 20U;

    bool was_active = enabling || running;
    if (!command.enabled)
    {
        enabling = running = fault_latched = false;
        // 首次有效反馈确定练习中心；后续失能、再使能不移动软限位。
        if (!origin_set && fresh && !error)
        {
            origin_rad = target_rad = feedback.Now_Angle;
            origin_set = true;
        }
    }
    else if (!fresh || error || !origin_set || dt_s > 0.02f)
        fault_latched = true;

    // 软件保护不能替代机械限位。
    if (command.enabled && fresh && origin_set &&
        Math_Abs(feedback.Now_Angle - origin_rad) > config.travel_rad + 0.05f)
        fault_latched = true;

    if (!command.enabled || fault_latched)
    {
        enabling = running = false;
        if (was_active || service_due)
        {
            yaw_motor->CAN_Send_Exit();
            yaw_motor->CAN_Send_Read_Status();
            last_service_ms = now_ms;
        }
        return;
    }

    if (!running)
    {
        if (!enabling)
        {
            enabling = true;
            enable_start_ms = now_ms;
            target_rad = feedback.Now_Angle;
            yaw_motor->CAN_Send_Enter();
            return;
        }
        if ((uint32_t)(now_ms - enable_start_ms) >= config.enable_timeout_ms)
        {
            fault_latched = true;
            yaw_motor->CAN_Send_Exit();
            return;
        }
        if (feedback.Control_Status != Motor_DM_Control_Status_ENABLE)
        {
            if (service_due)
            {
                yaw_motor->CAN_Send_Enter();
                last_service_ms = now_ms;
            }
            return;
        }
        target_rad = feedback.Now_Angle; // 从当前位置接管，避免跳到零。
        running = true;
    }
    else if (feedback.Control_Status != Motor_DM_Control_Status_ENABLE)
    {
        fault_latched = true;
        yaw_motor->CAN_Send_Exit();
        return;
    }

    float rate = command.yaw_rate_rad_s;
    Math_Constrain(&rate, -config.rate_max_rad_s, config.rate_max_rad_s);
    target_rad += rate * dt_s;
    Math_Constrain(&target_rad, origin_rad - config.travel_rad, origin_rad + config.travel_rad);
    Math_Constrain(&target_rad, -yaw_motor->Get_Angle_Max(), yaw_motor->Get_Angle_Max());

    yaw_motor->Set_Control_Angle(target_rad);
    yaw_motor->Set_Control_Omega(0.0f);
    yaw_motor->Set_Control_Torque(0.0f);
    yaw_motor->Set_K_P(config.kp);
    yaw_motor->Set_K_D(config.kd);
    // Control决定何时发送，Device编码，BSP提交；不自动使能或清错。
    if (yaw_motor->CAN_Send_Control() != HAL_OK)
    {
        fault_latched = true;
        yaw_motor->CAN_Send_Exit();
    }
}
