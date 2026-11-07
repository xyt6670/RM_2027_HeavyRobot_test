#ifndef APP_CONFIG_H
#define APP_CONFIG_H
/* 练习默认值，必须与达妙上位机实际配置核对；不会修改电机参数。 */
#define APP_YAW_CAN_HANDLE hcan1
#define APP_YAW_CONTROL_ID 0x01U
#define APP_YAW_FEEDBACK_ID 0x11U
#define APP_YAW_PMAX 12.5f
#define APP_YAW_VMAX 30.0f
#define APP_YAW_TMAX 10.0f
#define APP_YAW_DIRECTION 1.0f
#define APP_YAW_RATE_MAX_RAD_S 0.3f
#define APP_YAW_TRAVEL_RAD 0.35f
#define APP_YAW_KP 2.0f
#define APP_YAW_KD 0.2f
#define APP_YAW_FEEDBACK_TIMEOUT_MS 100U
#define APP_YAW_ENABLE_TIMEOUT_MS 500U
#define APP_ROBOT_PERIOD_MS 2U
#endif
