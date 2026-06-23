#include "led_dev.h"

led led_dev::board_led(13);

static void blink()
{
    led_dev::board_led.on();
    delay(50);
    led_dev::board_led.off();
    delay(950);
}

void led_dev::init()
{
    led_dev::board_led.init();
}

void led_dev::task_entry(void *arg)
{
    (void)arg;

    while(true)
    {
        blink();
    }
}
