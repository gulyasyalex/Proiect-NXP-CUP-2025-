#include <iostream>
#include "GpioFunctions/gpio_ctrl.h"
#include <linux/gpio.h>

int main(){
    int status;
    status = gpioOutSet("UART3_TXD",1);
    status = gpioOutSet("UART3_TXD",0);
    std::cout << status << std::endl;
    return 0;
}