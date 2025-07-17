#ifndef HARDWARE_BIB_H
#define HARWDARE_BIB_H

#include "freertos/FreeRTOS.h"
#include "sdkconfig.h"
#include "driver/uart.h"
#include "driver/gpio.h"

void init_uart(void);

static void rx_task(void *arg);

#endif