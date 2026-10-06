#include "keys.h"

KeyInstance KEY0_Instance = {GPIOH, GPIO_PIN_3, 0, 0, GPIO_PIN_RESET};
KeyInstance KEY1_Instance = {GPIOH, GPIO_PIN_2, 0, 0, GPIO_PIN_RESET};
KeyInstance KEY2_Instance = {GPIOC, GPIO_PIN_13, 0, 0, GPIO_PIN_RESET};
KeyInstance KEY3_Instance = {GPIOA, GPIO_PIN_0, 0, 0, GPIO_PIN_SET};

/**
 * @brief Initial TIM use for key scan
 * @note Do not forget NVIC settings
 * @param Freq_TIM TIM AHB Frequent
 * @param htim  pointer to TIM
 */

void Key_Scan_Init(uint8_t Freq_TIM)
{

//    key_htim.Instance->PSC = Freq_TIM * 100 - 1;
//    key_htim.Instance->ARR = 100;
//    /* get 100hz Tim*/
//    HAL_TIM_Base_Start_IT(&key_htim);
}

/**
 * @brief put it to TIM Handler
 *
 */
void Key_Progress(void)
{

//    if (__HAL_TIM_GET_FLAG(&key_htim, TIM_IT_UPDATE))
//    {
    Key_Scan(&KEY0_Instance);
    Key_Scan(&KEY1_Instance);
    Key_Scan(&KEY2_Instance);
    Key_Scan(&KEY3_Instance);
//        __HAL_TIM_CLEAR_FLAG(&key_htim, TIM_IT_UPDATE);
//    }
}

/**
 * @brief Key scan funtion
 *
 * @param Key_Instance
 */
void Key_Scan(KeyInstance *p)
{
    switch (p->key_stg)
    {
        case 0:
            if (HAL_GPIO_ReadPin(p->GPIOx, p->GPIO_Pin) == p->key_push_level)
            {
                p->key_stg = 1;
            }
            else
            {
                p->key_stg = 0;
            }

            break;

        case 1:
            if (HAL_GPIO_ReadPin(p->GPIOx, p->GPIO_Pin) == p->key_push_level)
            {
                p->key_stg = 2;
            }
            else
            {
                p->key_stg = 0;
            }

            break;

        case 2:
            if (HAL_GPIO_ReadPin(p->GPIOx, p->GPIO_Pin) != p->key_push_level)
            {
                p->key_stg = 0;

                p->key_flag = 1;
            }
            else
            {
                p->key_stg = 2;
            }

            break;
    }
}