
#ifndef MICROROS_UTILS__H
#define MICROROS_UTILS__H

#include "hal_data.h"

#define sleep_ms(X) tx_thread_sleep(((ULONG) ((X / 1000.0) * TX_TIMER_TICKS_PER_SECOND)))

extern bsp_leds_t g_bsp_leds;

typedef enum led_color_t {
    LED_BLUE = 0,
    LED_GREEN = 1,
    LED_RED = 2,
} led_color_t;

static inline void set_led_status(led_color_t led, bool status);

static inline void set_led_status(led_color_t led, bool status)
{
    R_BSP_PinAccessEnable();
    R_BSP_PinWrite(
        (bsp_io_port_pin_t) g_bsp_leds.p_leds[led],
        (status) ? BSP_IO_LEVEL_HIGH : BSP_IO_LEVEL_LOW);
}

#endif  // MICROROS_UTILS__H
