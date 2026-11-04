#ifndef DEVICE_DR16_H
#define DEVICE_DR16_H
#include "main.h"
#define W 0
#define S 1
#define A 2
#define D 3
#define Shift 4
#define Ctrl 5
#define Q 6
#define E 7
#define R 8 
#define F 9
#define G 10 
#define Z 11 
#define X 12
#define C 13 
#define V 14 
#define B 15

#define KEY_NORMAL 0
#define KEY_SHIFT 1
#define KEY_CRTL 2
/* ----------------------- Data Struct ------------------------------------- */
typedef  struct
{
 struct
 {
	uint16_t ch0;
	uint16_t ch1;
	uint16_t ch2;
	uint16_t ch3;
	uint8_t s1;
	uint8_t s2;
 }rc;
 struct
 {
	int16_t x;
	int16_t y;
	int16_t z;
	uint8_t press_l;
	uint8_t press_r;
 }mouse;
 struct
 {
	uint16_t v;
 }key;
  uint16_t KeyCount[3][16];
 uint8_t  IsDR16Received;
}RC_Ctl_t;


#ifdef __cplusplus
extern "C" {
#endif

RC_Ctl_t *GetRCInitData(void);
RC_Ctl_t *RemoteDataProcess(uint8_t *pData);
uint8_t *GiveBuffer(void);
RC_Ctl_t *GetRCData(void);
#ifdef __cplusplus
}
#endif
#endif
