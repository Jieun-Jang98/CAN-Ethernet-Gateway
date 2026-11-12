#ifndef KUSE_APP_MAIN_H
#define KUSE_APP_MAIN_H

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

void app_main_run(CAN_HandleTypeDef *hcan, UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif
