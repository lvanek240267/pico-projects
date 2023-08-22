/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include "pico/stdlib.h" 

int main() 
{
#ifndef PICO_DEFAULT_LED_PIN
#warning blink example requires a board with a regular LED
#else
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;

    const uint LED_PIN15 = 15;
    const uint LED_PIN16 = 16;

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    gpio_init(LED_PIN15);
    gpio_set_dir(LED_PIN15, GPIO_OUT);

    gpio_init(LED_PIN16);
    gpio_set_dir(LED_PIN16, GPIO_OUT);
    
    setup_default_uart();
    printf("Hello, world!\n");

    int a = 0;
    int b = 10000;

    while (true) 
    {
        gpio_put(LED_PIN, 1);
        gpio_put(LED_PIN15, 0);
        gpio_put(LED_PIN16, 1);
        sleep_ms(250);

        a++;
        b--;
        printf("a = %d, b = %d\n", a, b);

        gpio_put(LED_PIN, 0);
        gpio_put(LED_PIN15, 1);
        gpio_put(LED_PIN16, 0);
        sleep_ms(250);
    }
#endif
}
