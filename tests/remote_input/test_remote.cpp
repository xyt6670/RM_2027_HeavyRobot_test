#include "stm32f4xx_hal.h"
#include "device_dr16.h"
#include "app_init.h"
#include "app_remote.h"
#include "app_robot.h"
#include "bsp_drv_uart.h"
#include "device_motor_dm.h"
#include "app_config.h"
#include <vector>
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
    HAL_UARTEx_RxEventCallback(&huart3,n); UART_Poll();
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

CAN_HandleTypeDef hcan1={CAN1}, hcan2={CAN2};
uint8_t CAN1_0x3fe_Tx_Data[8], CAN1_0x4fe_Tx_Data[8];
Struct_CAN_Manage_Object CAN1_Manage_Object={},CAN2_Manage_Object={};
struct Sent { uint16_t id; uint16_t length; uint8_t bytes[8]; };
static std::vector<Sent> sent;
static uint8_t can_result=HAL_OK;
static CAN_Call_Back routed_callback;
void CAN_Init_With_Filters(CAN_HandleTypeDef *h,CAN_Call_Back cb,const uint16_t *ids,uint8_t count)
{
    CHECK(h==&hcan1 && count==1 && ids[0]==APP_YAW_FEEDBACK_ID && cb!=nullptr);
    CAN1_Manage_Object.CAN_Handler=h; routed_callback=cb;
}
uint8_t CAN_Send_Data(CAN_HandleTypeDef *h,uint16_t id,uint8_t *data,uint16_t length)
{
    CHECK(h==&hcan1 && length<=8);
    Sent packet={id,length,{0}};std::memcpy(packet.bytes,data,length);sent.push_back(packet);
    return can_result;
}
static unsigned count_special(uint8_t code)
{
    unsigned n=0;
    for(const auto &p:sent)
    {
        bool match=p.id==APP_YAW_CONTROL_ID && p.length==8 && p.bytes[7]==code;
        for(unsigned j=0;j<7;++j) match=match && p.bytes[j]==0xff;
        if(match) ++n;
    }
    return n;
}
static bool is_control(const Sent &s)
{
    return s.id==APP_YAW_CONTROL_ID && s.length==8 && s.bytes[0]!=0xff;
}
static float decode_p(const Sent &s)
{ return (((uint32_t)s.bytes[0]<<8)|s.bytes[1])*25.0f/65535.0f-12.5f; }
static void feedback_bytes(uint8_t *b,float p,uint8_t status=1)
{
    unsigned raw=(unsigned)((p+12.5f)/25.0f*65535.0f);
    b[0]=(uint8_t)((status<<4)|APP_YAW_CONTROL_ID);b[1]=raw>>8;b[2]=raw;
    b[3]=0x7f;b[4]=0xf7;b[5]=0xff;b[6]=25;b[7]=26;
}
static void local_feedback(Class_Motor_DM_Normal &m,float p,uint8_t status=1)
{ uint8_t b[8];feedback_bytes(b,p,status);m.CAN_RxCpltCallback(b); }
static void routed_feedback(float p,uint8_t status=1,uint32_t id=APP_YAW_FEEDBACK_ID,uint32_t length=8)
{
    Struct_CAN_Rx_Buffer f={};f.Header.StdId=id;f.Header.DLC=length;
    feedback_bytes(f.Data,p,status);routed_callback(&f);
}

static void test_dm_and_control()
{
    CAN1_Manage_Object.CAN_Handler=&hcan1;
    Class_Motor_DM_Normal motor;motor.Init(&hcan1,0x11,1,Motor_DM_Control_Method_NORMAL_MIT,12.5f,30.0f,10.0f);
    Struct_Motor_DM_Rx_Data_Normal f;
    motor.GetSnapshot(f);CHECK(!f.Has_Feedback);
    sent.clear();CHECK(motor.CAN_Send_Control()==HAL_ERROR && sent.empty());
    now_ms=1;local_feedback(motor,3.0f);motor.GetSnapshot(f);
    CHECK(f.Has_Feedback && std::fabs(f.Now_Angle-3.0f)<0.001f && f.Total_Round==0);
    CHECK(f.Last_Rx_ms==1 && std::fabs(f.Now_Omega)<0.008f);
    uint8_t bad[8];feedback_bytes(bad,5.0f);bad[0]=0x12;now_ms=2;
    motor.CAN_RxCpltCallback(bad);motor.GetSnapshot(f);CHECK(f.Last_Rx_ms==1);
    bad[0]=0xf1;motor.CAN_RxCpltCallback(bad);motor.GetSnapshot(f);CHECK(f.Last_Rx_ms==1);
    fake_primask=1;motor.GetSnapshot(f);CHECK(fake_primask==1);fake_primask=0;
    motor.Set_Control_Angle(12.5f);motor.Set_Control_Omega(30.0f);motor.Set_Control_Torque(10.0f);
    motor.Set_K_P(500.0f);motor.Set_K_D(5.0f);
    CHECK(motor.CAN_Send_Control()==HAL_OK);
    for(unsigned i=0;i<8;++i)CHECK(sent.back().bytes[i]==0xff);
    motor.Set_Control_Angle(-12.5f);motor.Set_Control_Omega(-30.0f);motor.Set_Control_Torque(-10.0f);
    motor.Set_K_P(0.0f);motor.Set_K_D(0.0f);CHECK(motor.CAN_Send_Control()==HAL_OK);
    for(unsigned i=0;i<8;++i)CHECK(sent.back().bytes[i]==0);
    motor.Set_Control_Angle(NAN);CHECK(motor.CAN_Send_Control()==HAL_ERROR);
    now_ms=3;local_feedback(motor,0,8);CHECK(motor.CAN_Send_Control()==HAL_ERROR);
    CHECK(count_special(0xfb)==0 && count_special(0xfc)==0);
    // Actual Gimbal code: fresh disabled feedback -> explicit enable -> position control.
    const Gimbal_Config cfg={APP_YAW_KP,APP_YAW_KD,APP_YAW_RATE_MAX_RAD_S,APP_YAW_TRAVEL_RAD,APP_YAW_FEEDBACK_TIMEOUT_MS,APP_YAW_ENABLE_TIMEOUT_MS};
    Gimbal_Init(motor,cfg);sent.clear();now_ms=10;local_feedback(motor,0,0);
    Gimbal_Command cmd={0.3f,0};Gimbal_Update(cmd,now_ms,0.002f);
    CHECK(count_special(0xfd)==1 && count_special(0xfc)==0);
    cmd.enabled=1;now_ms=12;Gimbal_Update(cmd,now_ms,0.002f);CHECK(count_special(0xfc)==1);
    now_ms=14;local_feedback(motor,0,1);Gimbal_Update(cmd,now_ms,0.002f);CHECK(is_control(sent.back()));
    for(unsigned i=0;i<1000;++i)
    { now_ms+=2;local_feedback(motor,0,1);Gimbal_Update(cmd,now_ms,0.002f); }
    CHECK(decode_p(sent.back())>0.34f && decode_p(sent.back())<=0.351f);
    float held=decode_p(sent.back());cmd.yaw_rate_rad_s=0;
    now_ms+=2;local_feedback(motor,0,1);Gimbal_Update(cmd,now_ms,0.002f);
    CHECK(std::fabs(decode_p(sent.back())-held)<0.001f);
    // Feedback timeout latches off; a fresh packet alone cannot restart.
    now_ms+=100;Gimbal_Update(cmd,now_ms,0.002f);CHECK(count_special(0xfd)>=2);
    unsigned enters=count_special(0xfc);
    now_ms+=2;local_feedback(motor,0,1);Gimbal_Update(cmd,now_ms,0.002f);
    CHECK(count_special(0xfc)==enters);
    cmd.enabled=0;now_ms+=2;Gimbal_Update(cmd,now_ms,0.002f);
    cmd.enabled=1;now_ms+=2;Gimbal_Update(cmd,now_ms,0.002f);CHECK(count_special(0xfc)==enters+1);
    now_ms+=2;local_feedback(motor,0,1);Gimbal_Update(cmd,now_ms,0.002f);
    // CAN submission failure stops normal commands.
    can_result=HAL_BUSY;now_ms+=2;local_feedback(motor,0,1);Gimbal_Update(cmd,now_ms,0.002f);
    can_result=HAL_OK;now_ms+=2;local_feedback(motor,0,1);size_t before=sent.size();
    Gimbal_Update(cmd,now_ms,0.002f);
    for(size_t i=before;i<sent.size();++i) CHECK(!is_control(sent[i]));
    // Retry after deliberate disable; original bounds survive re-enable.
    cmd.enabled=0;now_ms+=20;Gimbal_Update(cmd,now_ms,0.002f);
    cmd.enabled=1;now_ms+=2;local_feedback(motor,0.41f,1);Gimbal_Update(cmd,now_ms,0.002f);
    CHECK(!is_control(sent.back()) && count_special(0xfb)==0);
    // Enable handshake times out despite fresh disabled feedback; no auto-clear on faults.
    Gimbal_Init(motor,cfg);cmd.enabled=0;now_ms=3000;local_feedback(motor,0,0);Gimbal_Update(cmd,now_ms,0.002f);
    cmd.enabled=1;now_ms=3002;Gimbal_Update(cmd,now_ms,0.002f);
    for(unsigned i=0;i<251;++i)
    { now_ms+=2;local_feedback(motor,0,0);Gimbal_Update(cmd,now_ms,0.002f); }
    CHECK(count_special(0xfd)>1);
    unsigned at_timeout=count_special(0xfc);now_ms+=2;local_feedback(motor,0,1);Gimbal_Update(cmd,now_ms,0.002f);
    CHECK(count_special(0xfc)==at_timeout);
    // Fault code 8 must never trigger the legacy automatic clear-error path.
    cmd.enabled=0;now_ms+=2;Gimbal_Update(cmd,now_ms,0.002f);
    cmd.enabled=1;now_ms+=2;local_feedback(motor,0,8);Gimbal_Update(cmd,now_ms,0.002f);
    CHECK(count_special(0xfb)==0 && count_special(0xfc)==at_timeout);
    // The current tool also documents low-numbered faults; retain and inhibit every one.
    const uint8_t low_faults[]={2,3,5,7};
    for(uint8_t code:low_faults)
    {
        Gimbal_Init(motor,cfg);cmd.enabled=0;now_ms+=20;local_feedback(motor,0,0);
        Gimbal_Update(cmd,now_ms,0.002f);
        unsigned before_enter=count_special(0xfc);
        now_ms+=2;local_feedback(motor,0,code);motor.GetSnapshot(f);
        CHECK(f.Has_Feedback && f.Last_Rx_ms==now_ms && f.Control_Status==code);
        CHECK(motor.CAN_Send_Control()==HAL_ERROR);
        cmd.enabled=1;Gimbal_Update(cmd,now_ms,0.002f);
        CHECK(count_special(0xfc)==before_enter);
    }
    // Scheduling timestamp may precede an ISR feedback timestamp; snapshot then fresh clock wins.
    Gimbal_Init(motor,cfg);cmd.enabled=0;now_ms=5000;local_feedback(motor,0,0);Gimbal_Update(cmd,4999,0.002f);
    cmd.enabled=1;now_ms=5002;Gimbal_Update(cmd,5001,0.002f);CHECK(count_special(0xfc)>enters);
}

static void test_app_and_bsp()
{
    sent.clear();uint8_t b[36];now_ms=0;App_Init();
    CHECK(UART3_Manage_Object.Callback_Function!=nullptr && UART3_Manage_Object.Rx_Buffer_Length==36 && dma3.half_enabled==0);
    routed_feedback(0,0);
    frame(b,1024,1024,1024,1024,1,3);inject(b,18,2);App_Robot_Poll(now_ms);
    CHECK(count_special(0xfc)==0); // Up at boot does not arm.
    frame(b,1024,1024,1024,1024,2,3);inject(b,18,4);App_Robot_Poll(now_ms);
    frame(b,1024,1024,1024,1024,1,3);inject(b,18,6);App_Robot_Poll(now_ms);
    CHECK(count_special(0xfc)==1);
    now_ms=8;routed_feedback(0,1);inject(b,18,8);App_Robot_Poll(now_ms);CHECK(is_control(sent.back()));
    // Mapping tested through actual App -> command -> Gimbal -> DM encoding.
    frame(b,1684,1024,1024,1024,1,3);
    for(unsigned i=0;i<20;++i)
    { now_ms+=2;routed_feedback(0,1);inject(b,18,now_ms);App_Robot_Poll(now_ms); }
    CHECK(decode_p(sent.back())>0.01f);
    unsigned enters=count_special(0xfc);
    now_ms+=100;routed_feedback(0,1);App_Robot_Poll(now_ms);CHECK(count_special(0xfd)>=2);
    // Reconnect while up does not restart.
    now_ms+=2;routed_feedback(0,0);inject(b,18,now_ms);App_Robot_Poll(now_ms);
    CHECK(count_special(0xfc)==enters);
    // BSP ignores HT; polling while busy does not restart DMA.
    unsigned starts_before=starts;event_type=HAL_UART_RXEVENT_HT;HAL_UARTEx_RxEventCallback(&huart3,18);
    for(unsigned i=0;i<30;++i)UART_Poll();
    CHECK(starts==starts_before);
    huart3.ErrorCode=1;huart3.RxState=HAL_UART_STATE_READY;HAL_UART_ErrorCallback(&huart3);UART_Poll();
    CHECK(UART3_Manage_Object.Error_Count==1 && huart3.RxState==HAL_UART_STATE_BUSY_RX);
    huart3.RxState=HAL_UART_STATE_READY;start_result=HAL_ERROR;UART_Poll();
    CHECK(UART3_Manage_Object.Last_Start_Status==HAL_ERROR);
    start_result=HAL_OK;UART_Poll();CHECK(huart3.RxState==HAL_UART_STATE_BUSY_RX);
    frame(start_frame,1350);receive_during_start=true;huart3.RxState=HAL_UART_STATE_READY;UART_Poll();
    CHECK(fake_primask==0 && count_special(0xfb)==0 && tx_count==0);
    // Independent mapping: endpoints, neutral, disconnect and rollover.
    Class_DR16 r;r.Init();App_Remote_Init(r);Gimbal_Command cmd;
    frame(b,1024,1024,1024,1024,2,3);local_receive(r,b,18,1000);App_Remote_Update(cmd);CHECK(!cmd.enabled);
    frame(b,1684,1024,1024,1024,1,3);local_receive(r,b,18,1001);App_Remote_Update(cmd);
    CHECK(cmd.enabled && std::fabs(cmd.yaw_rate_rad_s-0.3f)<1e-6f);
    frame(b,364,1024,1024,1024,1,3);local_receive(r,b,18,1002);App_Remote_Update(cmd);
    CHECK(std::fabs(cmd.yaw_rate_rad_s+0.3f)<1e-6f);
    frame(b,1034,1024,1024,1024,1,3);local_receive(r,b,18,1003);App_Remote_Update(cmd);CHECK(cmd.yaw_rate_rad_s==0);
    now_ms=1103;App_Remote_Update(cmd);CHECK(!cmd.enabled && cmd.yaw_rate_rad_s==0);
    local_receive(r,b,18,1104);App_Remote_Update(cmd);CHECK(!cmd.enabled);
}
int main()
{
    test_device();test_dm_and_control();test_app_and_bsp();
    std::printf("PASS: %u checks; DR16 + DM Normal + App/Gimbal + UART BSP; mock CAN/HAL, no hardware.\n",checks);
}
