#include "stm32f4xx_hal.h"
#include "device_dr16.h"
#include "app_init.h"
#include "app_remote.h"
#include "app_robot.h"
#include "bsp_drv_uart.h"
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cstring>

uint32_t fake_primask=0;
static uint32_t now_ms, starts, tx_count;
static uint32_t event_type=HAL_UART_RXEVENT_IDLE;
static HAL_StatusTypeDef start_result=HAL_OK;
static bool receive_during_start, require_lock_on_tick;
static uint8_t start_frame[18];
static DMA_HandleTypeDef dma1={1}, dma3={1};
UART_HandleTypeDef huart1={USART1,&dma1,HAL_UART_STATE_READY,0};
UART_HandleTypeDef huart3={USART3,&dma3,HAL_UART_STATE_READY,0};
static unsigned checks;
#define CHECK(x) do { ++checks; if(!(x)) { std::fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#x); std::exit(1); } } while(0)
extern "C" uint32_t HAL_GetTick(void)
{
    if(require_lock_on_tick) { CHECK(fake_primask==1U); require_lock_on_tick=false; }
    return now_ms;
}
extern "C" HAL_UART_RxEventTypeTypeDef HAL_UARTEx_GetRxEventType(UART_HandleTypeDef *) { return event_type; }
extern "C" HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *h,uint8_t *p,uint16_t n)
{
    CHECK(p!=nullptr && n>0 && n<=UART_BUFFER_SIZE); ++starts;
    if(start_result!=HAL_OK) return start_result;
    if(h->RxState!=HAL_UART_STATE_READY) return HAL_BUSY;
    h->RxState=HAL_UART_STATE_BUSY_RX; h->hdmarx->half_enabled=1;
    if(receive_during_start) {
        receive_during_start=false; ++now_ms; std::memcpy(p,start_frame,18);
        event_type=HAL_UART_RXEVENT_IDLE; h->RxState=HAL_UART_STATE_READY;
        HAL_UARTEx_RxEventCallback(h,18);
    }
    return HAL_OK;
}
extern "C" HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *,uint8_t *,uint16_t)
{ ++tx_count; return HAL_OK; }

// Pack wire bits independently; do not reproduce the production unpacking formulas.
static void frame(uint8_t *b,uint16_t c0=1024,uint16_t c1=1024,uint16_t c2=1024,uint16_t c3=1024,uint8_t s1=1,uint8_t s2=3)
{
    std::memset(b,0,18);
    uint64_t bits=(uint64_t)c0|((uint64_t)c1<<11)|((uint64_t)c2<<22)|((uint64_t)c3<<33)|((uint64_t)s2<<44)|((uint64_t)s1<<46);
    for(unsigned i=0;i<6;++i) b[i]=(uint8_t)(bits>>(i*8));
    b[16]=0; b[17]=4;
}
static Struct_DR16_State read(Class_DR16 &remote)
{ Struct_DR16_State s; remote.GetSnapshot(s); return s; }
static void local_receive(Class_DR16 &remote,const uint8_t *b,uint16_t n,uint32_t t)
{ now_ms=t; remote.UART_RxCpltCallback(b,n,t); remote.Update(); }
static void inject(const uint8_t *b,uint16_t n,uint32_t t,uint32_t event=HAL_UART_RXEVENT_IDLE)
{
    now_ms=t; CHECK(n<=DR16_RX_BUFFER_LENGTH);
    std::memcpy(UART3_Manage_Object.Rx_Buffer,b,n); event_type=event;
    huart3.RxState=HAL_UART_STATE_READY; // HAL ends this NORMAL DMA reception before callback.
    HAL_UARTEx_RxEventCallback(&huart3,n); UART_Poll(); App_Remote_Update();
    CHECK(huart3.RxState==HAL_UART_STATE_BUSY_RX && dma3.half_enabled==0);
}

static void test_device()
{
    Class_DR16 remote; remote.Init(); Struct_DR16_State s=read(remote); uint8_t b[36];
    CHECK(s.data.rc.ch0==1024 && s.data.rc.ch1==1024 && s.data.rc.ch4==1024);
    CHECK(s.status.online==0 && s.status.has_valid_frame==0 && s.status.age_ms==UINT32_MAX);
    frame(b,364,1684,700,1500); b[6]=0x2e;b[7]=0xfb;b[8]=0xff;b[9]=0x7f;
    b[10]=0xff;b[11]=0xff;b[12]=1;b[14]=0x11;b[15]=0x80;
    local_receive(remote,b,18,0);s=read(remote);
    CHECK(s.data.rc.ch0==364 && s.data.rc.ch1==1684 && s.data.rc.ch2==700 && s.data.rc.ch3==1500);
    CHECK(s.data.mouse.x==-1234 && s.data.mouse.y==32767 && s.data.mouse.z==-1);
    CHECK(s.data.mouse.press_l==1 && s.data.mouse.press_r==0 && s.data.key.v==0x8011);
    CHECK(s.status.online==1 && s.status.valid_frame_count==1 && s.status.last_valid_rx_ms==0);
    for(uint8_t a=1;a<=3;++a) for(uint8_t d=1;d<=3;++d) {
        frame(b,1024,1024,1024,1024,a,d);local_receive(remote,b,18,20);s=read(remote);
        CHECK(s.data.rc.s1==a && s.data.rc.s2==d);
    }
    uint32_t count=s.status.valid_frame_count,last=s.status.last_valid_rx_ms;
    remote.UART_RxCpltCallback(nullptr,18,21); local_receive(remote,b,17,22);
    frame(b,363);local_receive(remote,b,18,23);frame(b,1685);local_receive(remote,b,18,24);
    frame(b,1024,1024,1024,1024,0,1);local_receive(remote,b,18,25);
    frame(b);b[12]=2;local_receive(remote,b,18,26);s=read(remote);
    CHECK(s.status.valid_frame_count==count && s.status.last_valid_rx_ms==last);
    CHECK(s.status.invalid_length_count==2 && s.status.invalid_frame_count==4);
    now_ms=119;remote.Update();CHECK(read(remote).status.online==1);
    now_ms=120;remote.Update();s=read(remote);CHECK(s.status.online==0 && s.status.age_ms==100 && s.status.timeout_count==1);
    now_ms=300;remote.Update();CHECK(read(remote).status.timeout_count==1);
    frame(b,1684);local_receive(remote,b,18,301);CHECK(read(remote).status.online==1);
    Struct_DR16_State saved=read(remote);frame(b,1100);local_receive(remote,b,18,302);
    CHECK(saved.data.rc.ch0==1684 && read(remote).data.rc.ch0==1100);
    CHECK(saved.status.last_valid_rx_ms==301 && read(remote).status.last_valid_rx_ms==302);
    frame(b,400);frame(b+18,1600);count=read(remote).status.valid_frame_count;
    local_receive(remote,b,36,310);s=read(remote);CHECK(s.status.valid_frame_count==count+2 && s.data.rc.ch0==1600);
    frame(b,363);frame(b+18,1234);count=s.status.valid_frame_count;
    local_receive(remote,b,36,311);s=read(remote);CHECK(s.status.valid_frame_count==count+1 && s.data.rc.ch0==1234);
    frame(b);local_receive(remote,b,18,UINT32_MAX-40U);
    now_ms=35;remote.Update();s=read(remote);CHECK(s.status.online==1 && s.status.age_ms==76);
    now_ms=70;remote.Update();CHECK(read(remote).status.online==0 && read(remote).status.age_ms==111);
    fake_primask=1;remote.Update();(void)read(remote);CHECK(fake_primask==1);fake_primask=0;
    require_lock_on_tick=true;remote.Update();CHECK(!require_lock_on_tick && fake_primask==0);
}
static void test_app_and_bsp()
{
    uint8_t b[36];now_ms=0;App_Init();App_Remote_Update();
    CHECK(g_remote_debug.online==0 && g_remote_debug.ch0_raw==1024);
    CHECK(g_remote_debug.ch0_normalized==0 && g_remote_debug.age_ms==UINT32_MAX);
    CHECK(UART3_Manage_Object.Callback_Function!=nullptr && UART3_Manage_Object.Rx_Buffer_Length==36 && dma3.half_enabled==0);
    frame(b);inject(b,18,0);CHECK(g_remote_debug.online==1 && g_remote_debug.valid_frames==1);
    for(uint8_t a=1;a<=3;++a) for(uint8_t d=1;d<=3;++d) {
        frame(b,364,1684,700,1500,a,d);inject(b,18,10+a*3+d);
        CHECK(g_remote_debug.ch0_raw==364 && g_remote_debug.ch1_raw==1684);
        CHECK(g_remote_debug.ch2_raw==700 && g_remote_debug.ch3_raw==1500);
        CHECK(g_remote_debug.ch0_centered==-660 && g_remote_debug.ch1_centered==660);
        CHECK(std::fabs(g_remote_debug.ch0_normalized+1.0f)<1e-6f && std::fabs(g_remote_debug.ch1_normalized-1.0f)<1e-6f);
        CHECK(g_remote_debug.s1==a && g_remote_debug.s2==d);
    }
    for(uint16_t v=364;v<=1684;v=(uint16_t)(v+11)) {
        frame(b,v,(uint16_t)(2048-v));inject(b,18,50);
        CHECK(g_remote_debug.ch0_raw==v && g_remote_debug.ch1_raw==2048-v);
        CHECK(g_remote_debug.ch0_normalized>=-1 && g_remote_debug.ch0_normalized<=1);
        CHECK(std::fabs(g_remote_debug.ch0_normalized+g_remote_debug.ch1_normalized)<1e-6f);
    }
    frame(b,1034,1014);inject(b,18,55);CHECK(g_remote_debug.ch0_normalized==0 && g_remote_debug.ch1_normalized==0);
    frame(b,1035,1013);inject(b,18,56);CHECK(g_remote_debug.ch0_normalized>0 && g_remote_debug.ch1_normalized<0);
    uint32_t valid=g_remote_debug.valid_frames;
    inject(b,17,70);frame(b,363);inject(b,18,71);frame(b,1685);inject(b,18,72);
    frame(b,1024,1024,1024,1024,0,1);inject(b,18,73);frame(b);b[12]=2;inject(b,18,74);
    CHECK(g_remote_debug.valid_frames==valid && g_remote_debug.invalid_lengths==1 && g_remote_debug.invalid_frames==4);
    now_ms=155;App_Remote_Update();CHECK(g_remote_debug.online==1);
    now_ms=156;App_Remote_Update();CHECK(g_remote_debug.online==0 && g_remote_debug.timeout_count==1);
    CHECK(g_remote_debug.ch0_normalized==0 && g_remote_debug.ch1_normalized==0);
    now_ms=300;App_Remote_Update();CHECK(g_remote_debug.timeout_count==1);
    frame(b,1684);inject(b,18,301);CHECK(g_remote_debug.online==1 && g_remote_debug.ch0_normalized==1);
    frame(b,400);frame(b+18,1600);valid=g_remote_debug.valid_frames;inject(b,36,310,HAL_UART_RXEVENT_TC);
    CHECK(g_remote_debug.valid_frames==valid+2 && g_remote_debug.ch0_raw==1600);
    inject(b,7,311);frame(b,1234);inject(b,18,320);CHECK(g_remote_debug.ch0_raw==1234);
    valid=g_remote_debug.valid_frames;uint32_t restarts=UART3_Manage_Object.Restart_Count;
    event_type=HAL_UART_RXEVENT_HT;HAL_UARTEx_RxEventCallback(&huart3,18);App_Remote_Update();CHECK(g_remote_debug.valid_frames==valid);
    for(unsigned i=0;i<30;++i) { UART_Poll();App_Remote_Update(); }
    CHECK(UART3_Manage_Object.Restart_Count==restarts);
    huart3.ErrorCode=1;huart3.RxState=HAL_UART_STATE_READY;HAL_UART_ErrorCallback(&huart3);
    CHECK(UART3_Manage_Object.Error_Count==1 && UART3_Manage_Object.Last_Error_Code==1);
    now_ms=330;UART_Poll();App_Remote_Update();CHECK(huart3.RxState==HAL_UART_STATE_BUSY_RX && UART3_Manage_Object.Restart_Count==restarts+1);
    huart3.RxState=HAL_UART_STATE_READY;start_result=HAL_ERROR;UART_Poll();
    CHECK(UART3_Manage_Object.Last_Start_Status==HAL_ERROR && huart3.RxState==HAL_UART_STATE_READY);
    start_result=HAL_OK;UART_Poll();CHECK(huart3.RxState==HAL_UART_STATE_BUSY_RX);
    frame(start_frame,1350);receive_during_start=true;huart3.RxState=HAL_UART_STATE_READY;now_ms=400;
    UART_Poll();App_Remote_Update();CHECK(g_remote_debug.online==1 && g_remote_debug.age_ms==0 && g_remote_debug.ch0_raw==1350);
    CHECK(fake_primask==0);
    HAL_UARTEx_RxEventCallback(&huart1,18);UART_Init(&huart1,nullptr,18);
    CHECK(UART_Rx_EnsureRunning(&huart1)==HAL_ERROR && UART_Rx_EnsureRunning(nullptr)==HAL_ERROR);
    uint32_t prev=starts;UART_Poll();CHECK(starts==prev);
    frame(b,1200);inject(b,18,1000);now_ms=1100;App_Robot_Poll(now_ms);
    CHECK(g_remote_debug.online==0 && g_remote_debug.ch0_normalized==0 && tx_count==0);
}
int main()
{
    test_device();test_app_and_bsp();
    std::printf("PASS: %u checks; Class_DR16 + app_init callback + App + UART BSP; no transmit calls.\n",checks);
}
