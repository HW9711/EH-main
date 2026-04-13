#ifndef __BSP_GPIO_H
#define __BSP_GPIO_H

#include "board_resource_map.h"

static inline GPIO_PinState Bsp_GpioRead(GPIO_TypeDef *port, uint16_t pin)
{
    return HAL_GPIO_ReadPin(port, pin);
}

static inline void Bsp_GpioWrite(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    HAL_GPIO_WritePin(port, pin, state);
}

static inline void Bsp_GpioToggle(GPIO_TypeDef *port, uint16_t pin)
{
    HAL_GPIO_TogglePin(port, pin);
}

#endif /* __BSP_GPIO_H */
