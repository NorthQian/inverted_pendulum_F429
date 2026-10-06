#ifndef _KEYS_H_
#define _KEYS_H_

#include "main.h"

//#define key_htim htim6
//extern TIM_HandleTypeDef key_htim;

/**
 * @brief Key struct
 *
 */

typedef struct
{
    GPIO_TypeDef *GPIOx;    // GPIO PORT
    uint16_t GPIO_Pin;      // GPIO PIN
    uint8_t key_stg;        // use for tim
    uint8_t key_flag;       // use for judge
    uint8_t key_push_level; // the gpio level when pushing

} KeyInstance;

extern KeyInstance KEY0_Instance;
extern KeyInstance KEY1_Instance;
extern KeyInstance KEY2_Instance;
extern KeyInstance KEY3_Instance;

void Key_Scan_Init(uint8_t Freq_TIM);
void Key_Progress(void);
void Key_Scan(KeyInstance *p);

#endif