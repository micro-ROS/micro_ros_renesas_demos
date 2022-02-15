
#ifndef MICROROS_UTILS__H
#define MICROROS_UTILS__H

#include "hal_data.h"

#define sleep_ms(X) R_BSP_SoftwareDelay(X, BSP_DELAY_UNITS_MILLISECONDS)

extern bsp_leds_t g_bsp_leds;

typedef enum led_color_t {
    LED_1,
    LED_2,
    LED_3,
    LED_4,
    LED_5,
    LED_6,
} led_color_t;

static inline void set_led_status(led_color_t led, bool status);

static inline void set_led_status(led_color_t led, bool status)
{
    R_BSP_PinAccessEnable();
    R_BSP_PinWrite(
        (bsp_io_port_pin_t) g_bsp_leds.p_leds[led],
        (!status) ? BSP_IO_LEVEL_HIGH : BSP_IO_LEVEL_LOW);
}

#endif  // MICROROS_UTILS__H
