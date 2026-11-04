#include "app_remote.h"

void App_Remote_Init(void)
{
    /* TODO: 初始化遥控/键鼠操作方式、按键边沿与默认停机命令。 */
    /* DR16设备解析和UART接收初始化不在本空骨架中隐式启动。 */
}

void App_Remote_Update(void)
{
    /* TODO: 读取有效DR16输入快照，映射底盘速度、云台目标与发射请求。 */
    /* TODO: 处理R/C/G等模式切换；输入失联时清除运行许可。 */
}
