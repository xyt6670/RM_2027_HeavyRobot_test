# 2027重装电控练习工程

基于8a2aa35练习C板DT7＋DR16输入。当前已编写USART3接收、Class_DR16解析/掉线检测及通道0/1映射，初始化集中在app_init。无Service、无RTOS，未接入电机控制；实物接收仍需上板验收。

先读：[DR16类练习：实际调用链与Keil验收](docs/DR16类练习_调用链与验收.md)。本轮尚未提交，提交名由用户决定。

```text
User/
  BSP/       bsp_drv_can、bsp_drv_uart
  Device/    device_dr16、device_motor_dji、device_motor_dm、device_serialplot
  Control/   control_chassis、control_gimbal、control_shoot、control_alg_pid、control_alg_math
  App/       app_init.cpp、app_robot.c、app_remote.cpp
Core/        CubeMX生成的外设初始化和中断入口
Drivers/     HAL、CMSIS及DSP依赖
MDK-ARM/     Keil工程、启动文件与工具配置
Build/       编译输出和日志（不提交Git）
docs/        分层、开发顺序与DR16接入状态
```

- 打开Keil：[MDK-ARM/can_motor_test.uvprojx](MDK-ARM/can_motor_test.uvprojx)。工程和.ioc暂保留原来的can_motor_test名称，避免无关工具配置改动。
- 阅读：[目录、命名与开发顺序](docs/工程结构与开发顺序.md)。
- 阅读：[裸机App职责与DR16接入现状](docs/裸机App与DR16接入.md)。
- 程序入口：`main.c → App_Init`；主循环`UART_Poll → App_Robot_Poll(now_ms)`。
- 旧演示应用app_motor已移除，不再恢复；RTOS未加入。

电机类和Control骨架保持原样。DR16由旧C解析模块整理为device_dr16.cpp中的Class_DR16；app_remote使用对象引用，因此改为.cpp。Keil的app组已加入app_init.cpp/.h，并同步更新两个改名源文件；重新加载工程后再Build。

旧备份、过往说明与编译日志已移到工作区同级的`_工程归档/RM_2027_HeavyRobot_test/layout_20261028_140338/`，其中snapshot保存本轮整理前的源码与工程配置，rename_map.json记录全部重命名。
