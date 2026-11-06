# 主机端DR16接收链测试

运行 `python tests/remote_input/run_tests.py`，需要PATH中有本机GCC/G++。测试编译真实 `Class_DR16`、`app_init`、`app_remote`、`app_robot` 和中科大UART BSP，用stubs替代HAL和串口外设。输出位于已忽略的 `Build/remote_tests`，不向工程里生成固件源文件。

覆盖：初始化和真实函数指针桥、通道/拨杆/鼠标/键码解析、坏长度与坏帧不刷新有效时间、100ms掉线边界、重连、Tick回绕、HT忽略、TC两帧、UART后台恢复、BUSY_RX不重复重启、返回快照独立性、临界区恢复原PRIMASK、Update在临界区内取时间，以及无发送。

测试中的本地Class实例用于独立验证协议和状态；App的静态设备对象不通过测试专用getter暴露。端到端App测试把字节放进UART BSP实际接收缓存，再触发HAL接收事件。

这些测试不加入Keil固件；通过不代表波形、100k 8E1、板载反相、真实DMA时序、接线或对频已经验收。硬件仍须按本次练习说明上板验证。
