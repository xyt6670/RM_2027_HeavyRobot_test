# 2027重装电控练习工程

用旧英雄硬件练习分层电控，当前采用裸机轮询。三个控制模块和输入映射仍是待填写骨架；未完成整车控制或DR16实物接收接入。

```text
User/
  BSP/       bsp_drv_can、bsp_drv_uart
  Device/    device_dr16、device_motor_dji、device_motor_dm、device_serialplot
  Control/   control_chassis、control_gimbal、control_shoot、control_alg_pid、control_alg_math
  App/       app_robot、app_remote
Core/        CubeMX生成的外设初始化和中断入口
Drivers/     HAL、CMSIS及DSP依赖
MDK-ARM/     Keil工程、启动文件与工具配置
Build/       编译输出和日志（不提交Git）
docs/        分层、开发顺序与DR16接入状态
```

- 打开Keil：[MDK-ARM/can_motor_test.uvprojx](MDK-ARM/can_motor_test.uvprojx)。工程和.ioc暂保留原来的can_motor_test名称，避免无关工具配置改动。
- 阅读：[目录、命名与开发顺序](docs/工程结构与开发顺序.md)。
- 阅读：[裸机App职责与DR16接入现状](docs/裸机App与DR16接入.md)。
- 程序入口：`main.c → App_Robot_Init / App_Robot_Poll`。
- 旧演示应用app_motor已移除，不再恢复；RTOS未加入。

已有驱动统一了文件名和物理目录，函数名、类名保持不变；另外新增了裸机App骨架。UART实现原来虽然叫.c，但实际按C++编译；现在明确命名为bsp_drv_uart.cpp。

旧备份、过往说明与编译日志已移到工作区同级的`_工程归档/RM_2027_HeavyRobot_test/layout_20261028_140338/`，其中snapshot保存本轮整理前的源码与工程配置，rename_map.json记录全部重命名。
