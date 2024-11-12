#ifndef GPIO_CTRL_H
#define GPIO_CTRL_H

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

/**
 * @brief Sets the specified GPIO pin to the specified output value.
 * 
 * This function builds and executes a shell command to set the specified GPIO pin 
 * to an output value (high or low). The command used is based on the `debix-gpio` 
 * utility, which controls GPIO pins on the Debix Model B board.
 * 
 * @param pin A string representing the GPIO pin to configure (e.g., "GPIO5_IO03").
 * @param value A string representing the output value to set (e.g., 1 for high, 0 for low).
 * 
 * @return int Returns 0 on success. Returns 1 if there is an error executing the command.
 */
int gpioOutSet(const char *pin, int value){

    if (value != 0 && value != 1){
        return 1;
    }

    char command[50] = "debix-gpio ";
    char value_str[2];
    
    sprintf(value_str, "%d", value);
    
    strcat(command, pin);
    strcat(command, " out ");
    strcat(command, value_str);

    int status = system(command);

    if (status == -1) {
        printf("Error executing GPIO command");
        return 1;
    }
    return 0;
}

/**
 * @brief Configures the specified GPIO pin as an input with edge detection.
 * 
 * This function builds and executes a shell command to set the specified GPIO pin 
 * to input mode with a specified edge detection setting (e.g., none, rising, falling, or both).
 * The command is based on the `debix-gpio` utility, which manages GPIO pins on the Debix Model B board.
 * 
 * @param pin A string representing the GPIO pin to configure (e.g., "GPIO5_IO03").
 * @param edge A string representing the edge detection mode for the input pin.
 *             - "0" for no edge detection.
 *             - "1" for detecting rising edges.
 *             - "2" for detecting falling edges.
 *             - "3" for detecting both rising and falling edges.
 * 
 * @return int Returns 0 on success, and 1 if there is an error executing the command.
 */
int gpioInSet(const char *pin, int edge){

    if (edge < 0 && edge > 3){
        return 1;
    }

    char command[50] = "debix-gpio ";
    char value_str[2];

    sprintf(value_str, "%d", edge);

    strcat(command, pin);
    strcat(command, " in ");
    strcat(command, value_str);

    int status = system(command);

    if (status == -1) {
        printf("Error executing GPIO command");
        return 1;
    }
    return 0;
}
#endif