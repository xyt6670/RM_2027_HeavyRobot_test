# 2027重装电控练习工程

当前练习：C板 + DT7/DR16通道0 + 达妙4310小yaw，裸机运行，不加Service、注册表、消息中心或RTOS。本轮基于已经提交的 `24f84cb` 继续修改，尚未提交、未烧录。

先读：[小yaw MIT练习：使用与调用链](docs/小yaw_MIT练习.md)，再看[调用链图](docs/小yaw_MIT调用链.svg)。

现场排查：[C板CAN排查顺序与本次USART3修复](docs/C板CAN排查顺序.md)。

```text
User/
  BSP/       bsp_drv_can、bsp_drv_uart：通信收发
  Device/    device_dr16、device_motor_dm：设备数据和协议
  Control/   control_gimbal.cpp：小yaw接管、积分、限位和发送
  App/       app_init.cpp：对象与初始化；app_remote.cpp：输入映射
             app_robot.cpp：2ms调度；app_config.h：练习参数
Core/        CubeMX生成的初始化和中断入口
Drivers/     HAL、CMSIS、DSP
MDK-ARM/     Keil工程
Build/       编译产物、测试报告，不提交Git
docs/        阅读说明
```

操作：首次上电/遥控断线恢复后，s1先下挡且通道0回中，再切上挡运行；中挡或下挡失能。推通道0改变位置目标，回中保持位置。默认练习范围为首次有效电机反馈位置左右0.35rad，不自动保存电机零点。

编译前重新加载 [Keil工程](MDK-ARM/can_motor_test.uvprojx)，防止旧窗口覆盖修改。分组只列源文件；头文件保留在磁盘并照常参与编译。

默认CAN1，控制ID `0x01`、反馈ID `0x11`，1Mbps、MIT模式，P/V/T映射范围12.5/30/10。**这些是待核对的练习参数，不是对实际电机配置的确认。** 接线、驱动版本、模式、映射范围和电机内部通信超时需按说明核对后上板。

旧的DR16观察练习及其验收文档属于上一个阶段；`g_remote_debug`已移除，DR16接收、解析、快照和100ms掉线检测仍保留。DJI电机库保留但本练习不调用。
