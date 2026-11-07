# DR16通道0控制4310小yaw

这一版从 `24f84cb`（明确dr16输入链路并实现隔离）继续写，没有切分支或提交。保留中科大电机类，去掉专门用于观察的 `App_Remote_Debug/g_remote_debug`，接通App→Control→Device→BSP。没有Service、RTOS或注册表。

## 先确认这次做的控制

通道0不是直接给电流，也不是“一推就跑到最大角度”。它先变成速度意图，再积分为位置目标：

```cpp
stick = 去中值、死区、归一化(ch0);       // -1~+1
command.yaw_rate_rad_s = stick * 0.3f;  // App，目标轨迹速度 rad/s
target_rad += command.yaw_rate_rad_s * dt_s; // Control，位置目标 rad
```

推摇杆时位置目标缓慢改变，回中后目标不再改变，电机保持这个位置。实际运动速度还受电机参数、负载和增益影响，不保证精确等于0.3rad/s。

MIT常用控制关系是：`tau = Kp*(p_des-p) + Kd*(v_des-v) + tau_ff`，再由驱动内部电流环执行。可以先理解成位置弹簧＋速度阻尼＋前馈力矩，没有积分项；不能直接称为完整PID。这里的Kd不是单片机上对误差做一次差分。

本练习设置 `p_des=target_rad`、`v_des=0`、`tau_ff=0`、Kp=2、Kd=0.2，靠位置目标轨迹移动，速度项提供阻尼。MIT的v_des是目标速度，不是限速参数。P/V/T_MAX是协议的编解码范围，不是本练习的运动范围，也不是力矩限幅保护。

依据：[达妙4310说明书的MIT模式章节（说明书转载）](https://aifitlab-wiki.super.site/damiao-docs/dm-j4310-2ec-v12-motor-instruction-manual)、[dmBots控制例程的control_mit与DM4310映射参数](https://github.com/dmBots/motor-control-routine/blob/master/orin%E8%BD%BD%E6%9D%BFcan%E6%8E%A7%E5%88%B6%E8%BE%BE%E5%A6%99%E7%94%B5%E6%9C%BA%E4%BE%8B%E7%A8%8B/dm_hw/src/hardware_interface/damiao.cpp)。说明书和示例涉及不同版本，最终以手上电机的固件和上位机读数为准。

## 各文件读什么

| 文件 | 职责及接口 |
|---|---|
| `app_init.cpp` | 定义唯一的remote和yaw_motor，初始化并传引用，绑定UART/CAN回调；最后开启接收 |
| `app_remote.cpp` | `App_Remote_Update(command)`：读取DR16快照，处理s1允许条件，把ch0变成速度命令；不碰电机 |
| `app_robot.cpp` | 主循环每2ms执行一次输入映射和`Gimbal_Update`，传实际dt，不补跑积压周期 |
| `control_gimbal.h` | 普通`Gimbal_Command`和`Gimbal_Config`，不包含DR16协议结构；Control不依赖app_config.h |
| `control_gimbal.cpp` | 接管、位置积分、软限位、反馈超时和发送时机；直接调用电机对象 |
| `device_motor_dm.cpp` | 解析反馈、保存数据、编码MIT报文、使能/失能/读取状态 |
| `bsp_drv_can.cpp` | 初始化过滤器和CAN接收中断，通过HAL提交发送；不决定云台运动 |

`app_config.h`只存练习配置，App把配置复制传给Control，Control不反向包含App配置。app_robot和control_gimbal因为调用C++接口，从.c改为.cpp；Keil已同步源文件名及C++11选项。头文件不重新加入分组。

## 上电后谁调用谁

```text
main → MX_DMA / MX_CAN / MX_USART初始化 → App_Init
  ├─ remote.Init
  ├─ yaw_motor.Init(CAN1,反馈ID,控制ID,MIT,P/V/T范围)
  ├─ App_Remote_Init(remote)：保存同一个遥控器对象的地址
  ├─ Gimbal_Init(yaw_motor,配置)：保存同一个电机对象的地址并复制配置
  ├─ App_Robot_Init：清命令、记录调度时间
  └─ CAN_Init_With_Filters + UART_Init：保存回调指针，开启接收

DR16字节 → UART空闲/完成事件 → BSP回调指针
  → app_init的Remote_RxCallback → remote.UART_RxCpltCallback：解析和记时

电机反馈 → CAN1_RX0中断 → BSP回调指针
  → app_init的Yaw_CAN_RxCallback：检查标准数据帧、反馈ID、长度8
  → yaw_motor.CAN_RxCpltCallback：解析传入字节，更新反馈时间

main循环 → UART_Poll维护接收 → App_Robot_Poll
  → App_Remote_Update：DR16.Update + GetSnapshot → Gimbal_Command
  → Gimbal_Update：电机GetSnapshot → 接管/限位/更新目标
  → yaw_motor.CAN_Send_Control → CAN_Send_Data → HAL_CAN_AddTxMessage
```

函数名带TIM不代表自动运行。这里没有启动控制定时器，控制由主循环每2ms调用。也没有调用旧的`TIM_1ms_CAN_PeriodElapsedCallback`，因此不会顺带发送DJI的0x200/0x2fe报文。小yaw所有CAN命令都由Control这一处决定发送时机。

## 达妙库怎么用：先认识Normal类

`Class_Motor_DM_Normal`代表一台普通独立CAN命令模式的达妙电机；Normal不是某一个固定控制模式，MIT是它支持的模式之一。`Class_Motor_DM_1_To_4`是另一种协议路径，本练习不用。

```cpp
// app_init：先绑定设备，不会自动把电机内部配置改成MIT。
yaw_motor.Init(&hcan1, 0x11, 0x01,
    Motor_DM_Control_Method_NORMAL_MIT, 12.5f, 30.0f, 10.0f);

// Control确认允许后发使能，并等待反馈状态变为1。
yaw_motor.CAN_Send_Enter();

// Control运行时：前五个Set只更新对象变量；最后一行才提交控制报文。
yaw_motor.Set_Control_Angle(target_rad); // rad
yaw_motor.Set_Control_Omega(0.0f);      // rad/s
yaw_motor.Set_K_P(2.0f);                // N·m/rad
yaw_motor.Set_K_D(0.2f);                // N·m/(rad/s)
yaw_motor.Set_Control_Torque(0.0f);     // N·m
yaw_motor.CAN_Send_Control();

// Control停止时：
yaw_motor.CAN_Send_Exit();
```

旧`TIM_Send_PeriodElapsedCallback`会在失能时自动使能、出错时自动清错；旧Alive回调也可能自动使能。本练习明确不调用这两个方法，也不调用保存零点。新的`CAN_Send_Control`仅在有反馈且反馈状态为使能时编码MIT控制帧，不自动使能或清故障，并返回CAN提交结果。

这次对Normal路径做了必要修正：反馈真正读取回调传入的字节；首次反馈不参与跨界比较；P/V/T按协议完整区间映射；加入有效反馈时间与短临界区快照；ID参数扩为uint16。1_To_4和DJI路径没有改造。

## 设备配置和实际操作

| 项目 | 本版默认值 | 上板前需要核对 |
|---|---|---|
| 板/遥控 | C板，DT7+DR16，USART3 DBUS | 不是FlySky iBUS；保留已打通的接收接线 |
| 总线 | CAN1，PD0 RX/PD1 TX，1Mbps | 接C板CAN1收发器端口的CAN_H/L，不把电机接到裸MCU引脚 |
| 控制ID/反馈ID | `0x01` / `0x11`，标准帧 | 对应电机上位机CAN_ID/ESC_ID和Master_ID/MST_ID |
| 模式 | MIT | 用上位机确认；Init不会写电机模式寄存器 |
| P/V/T_MAX | 12.5rad / 30rad/s / 10N·m | 必须与实际配置一致；旧库VMAX默认25，这次显式传30，不依赖旧默认值 |
| 方向 | +1 | 首次低速试转确认，需要反向改APP_YAW_DIRECTION为-1 |
| 命令轨迹速度 | 最大0.3rad/s | 约17.2°/s；实际响应取决于调参 |
| 练习软限位 | 首次有效反馈位置±0.35rad | 约±20°；必须先将机构放在有余量的位置再上电 |

初次练习先固定电机并留出转动空间。增益只是低速台架起点，未按你的实际负载调过。s1编号是协议里的s1，先确认实物哪一根拨杆与它对应。

1. 重新加载Keil工程，核对app_config.h和电机上位机参数；首次先让s1下挡、摇杆回中。
2. 程序失能并周期读取状态，拿到有效反馈后记录练习中心。若一直没有反馈，不会尝试带动力运行。
3. s1下挡且通道0回中过之后，上挡运行；中挡/下挡失能。首次使能从当前位置接管，不拉向机械零位。
4. 慢推通道0；回中保持位置。要看实际控制变量，可以在相关函数断点查看局部command、feedback、target_rad，不再维护一份观察专用全局结构体。
5. DR16有效帧超过100ms未更新，App给失能命令；恢复通信后必须再次下挡回中。电机反馈超时100ms、故障码、使能等待500ms超时、调度间隔超过20ms、已测角度越过练习边界0.05rad，或控制帧提交失败，都会停止控制并锁住，先切离运行挡再尝试。

首次中心只记录一次，切挡不会重设范围；MCU重启会重新记录。这个练习没有标定机械绝对限位，不能直接作为正式无滑环云台的限位方案。目标软限位也不能保证机构没有惯性越界。

**CAN线断开时，单片机发失能也可能送不到电机。必须在电机上位机里启用并验证驱动自身的通信超时保护；本工程不替你写TIMEOUT参数。** 不会自动发送清故障或保存零点命令；故障请查原因后按实际驱动方式处理。部分旧固件的读取状态命令支持情况也需要上板确认。

## 已验证与未验证

Keil ARMCC5编译通过，0错误、0警告。主机测试编译真实DR16、DM Normal、App、云台Control、数学库和UART BSP，CAN提交/HAL由测试替身提供，覆盖输入映射、启动允许、MIT编码、首次反馈、软限位、反馈超时、断线恢复、使能超时、错误码和发送失败。主机编译保留的旧库告警见Build/remote_tests/compiler.log，不能据此声称全工程在GCC下零告警。

尚未烧录；接线、真实ID、模式、上下位机范围、通信看门狗及实际电机响应仍需台架验收。测试入口为 `python tests/remote_input/run_tests.py`，最新结果在Build/remote_tests/result.json。
