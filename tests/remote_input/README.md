# DR16 → 小yaw主机测试

运行 `python tests/remote_input/run_tests.py`，需要PATH中有GCC/G++。不加入Keil工程，不烧录，不控制实物电机。

编译真实DR16、DM Normal、App、Gimbal、数学/PID库、UART BSP。替身仅提供HAL、CAN提交和用于主机编译的CMSIS接口。CAN BSP自身的硬件初始化和STM32实际中断不在本测试范围内。

覆盖协议字段与边界、18/36字节接收、无效帧不续期、100ms掉线、时间回绕、快照和原中断状态恢复、DMA HT忽略与接收恢复、MIT字节编码端点、回调数据独立于BSP共享缓存、首次角度反馈、启动允许、位置积分/保持/限位、反馈超时、使能超时、故障不自动清除、CAN提交失败停止，以及通信恢复不直接运行。

结果在Build/remote_tests/result.json。主机编译器日志在同目录compiler.log；对原有DM/数学库的已有switch、枚举比较、未用参数、符号比较告警保留提示，App和新Control仍按-Werror编译。主机测试通过不等于台架通过。
