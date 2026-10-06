#ifndef NECK_ANIM_H
#define NECK_ANIM_H

#include <stdint.h>

/* 单帧颈部动作：三轴电机弧度值。 */
typedef struct {
    float motor1_rad; /* 颈部电机 1（rad） */
    float motor2_rad; /* 颈部电机 2（rad） */
    float motor3_rad; /* 颈部电机 3（rad），yaw 方向 */
} NeckActionFrame;

/* 动作总帧数。 */
#define NECK_ACTION_ANIM_FRAME_COUNT (630)

/* 按行顺序的一帧一条记录。 */
extern const NeckActionFrame g_neck_action_anim[NECK_ACTION_ANIM_FRAME_COUNT];

#endif /* NECK_ANIM_H */
