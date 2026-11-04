#include "main.h"
#include "device_dr16.h"
#include <string.h>
 RC_Ctl_t RC_CtrlData;
 uint8_t sbus_rx_buffer[256]; 
 static uint16_t last_key;
RC_Ctl_t RC_InitData = 
{
    .rc = {                 // 遥控通道部分初始值
        .ch0 = 1024,        // 通道0初始值（例如遥控器摇杆中位附近值）
        .ch1 = 1024,        // 通道1初始值
        .ch2 = 1024,        // 通道2初始值
        .ch3 = 1024,        // 通道3初始值
        .s1 = 0,            // 开关s1初始状态（0表示未触发）
        .s2 = 0             // 开关s2初始状态
    },
    .mouse = {              // 鼠标部分初始值
        .x = 0,             // 鼠标X轴位移初始值（0表示无移动）
        .y = 0,             // 鼠标Y轴位移初始值
        .z = 0,             // 鼠标Z轴（滚轮）位移初始值
        .press_l = 0,       // 鼠标左键初始状态（0表示未按下）
        .press_r = 0        // 鼠标右键初始状态
    },
    .key = {                // 按键部分初始值
        .v = 0              // 按键状态初始值（0表示无按键按下）
    },
    .IsDR16Received = 0     // DR16信号接收状态（0表示未接收）
};


uint8_t *GiveBuffer()
{
	return sbus_rx_buffer;
}
static void KeyProc(uint16_t Key,uint16_t array[3][16])
{
	uint8_t Key_Shift = (Key >> 4) & 1;
	uint8_t Key_Ctrl = (Key >> 5) & 1;
	uint8_t Keytype;
//	uint16_t Toggle_keys = (0x0000)
	if(Key_Shift && !Key_Ctrl)
	{
		Keytype = 1;//Shift组合
	}
	else if(Key_Ctrl && !Key_Shift)
	{
		Keytype = 2;//Ctrl组合
	}
	else
	{
		Keytype = 0;//普通按键
	}
	for(uint8_t i = 0;i < 16;i++)
	{
		uint16_t mask = 0x0001 << i;
		if(i == 4 || i == 5)
		{
			continue;
		}
		if((Key & mask) && !(last_key & mask))
		{
			array[Keytype][i] = 1;
		}
		if(!(Key & mask) && (last_key & mask))
		{
			array[Keytype][i] = 0;
		}
		if(Keytype == 1)
		{
			array[1][i] = 1;
		}
		if(Keytype == 0)
		{
			array[1][i] = 0;
		}
	}
	last_key = Key;
}
RC_Ctl_t *RemoteDataProcess(uint8_t *pData)
{
	/* UART/DMA接收由BSP管理；调用者确认完整有效帧后再交给本函数解析。 */
	RC_CtrlData.rc.ch0 = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF; //通道0
	RC_CtrlData.rc.ch1 = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5)) 
	& 0x07FF;//通道1
	RC_CtrlData.rc.ch2 = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) |
	((int16_t)pData[4] << 10)) & 0x07FF;//通道2
	RC_CtrlData.rc.ch3 = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5]<<7)) & 
	0x07FF;//通道3
	
	RC_CtrlData.rc.s1 = ((pData[5] >> 4) & 0x000C) >> 2;
	RC_CtrlData.rc.s2 = ((pData[5] >> 4) & 0x0003);//开关
	
	
	RC_CtrlData.mouse.x = ((int16_t)pData[6]) | ((int16_t)pData[7] << 8);
	RC_CtrlData.mouse.y = ((int16_t)pData[8]) | ((int16_t)pData[9] << 8);
	RC_CtrlData.mouse.z = ((int16_t)pData[10]) | ((int16_t)pData[11] << 8); 
	RC_CtrlData.mouse.press_l = pData[12];
	RC_CtrlData.mouse.press_r = pData[13];
	uint16_t current_keys = ((int16_t)pData[14]) | ((int16_t)pData[15] << 8);
	
	RC_CtrlData.key.v = current_keys;
	KeyProc(current_keys,RC_CtrlData.KeyCount);
	
	RC_CtrlData.IsDR16Received = 1;
	RC_InitData.IsDR16Received = 1;
	return &RC_CtrlData;
}
RC_Ctl_t *GetRCData()
{
    return &RC_CtrlData;
}
RC_Ctl_t *GetRCInitData()
{
	return &RC_InitData;
}
