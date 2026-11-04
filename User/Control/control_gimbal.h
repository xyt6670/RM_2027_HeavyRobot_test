#ifndef CONTROL_GIMBAL_H
#define CONTROL_GIMBAL_H

#ifdef __cplusplus
extern "C" {
#endif

void Gimbal_Init(void);
/* dt_s: 实际控制周期，单位秒。当前为待实现接口，不产生电机输出。 */
void Gimbal_Update(float dt_s);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_GIMBAL_H */
