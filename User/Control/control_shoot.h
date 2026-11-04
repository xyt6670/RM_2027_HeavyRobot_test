#ifndef CONTROL_SHOOT_H
#define CONTROL_SHOOT_H

#ifdef __cplusplus
extern "C" {
#endif

void Shoot_Init(void);
/* dt_s: 实际控制周期，单位秒。当前为待实现接口，不产生电机输出。 */
void Shoot_Update(float dt_s);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_SHOOT_H */
