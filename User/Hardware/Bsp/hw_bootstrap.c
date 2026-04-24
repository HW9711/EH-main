#include "hw_bootstrap.h"
#include "bsp_board.h"
#include "bsp_i2c_bus.h"

void Hardware_PostInit(void)
{
    MX_I2C_Init();
}

void Hardware_BoardGpioInit(void)
{
    Board_GPIOConfiguration();
}
