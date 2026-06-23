#include "led_dev.h"

namespace led_dev {

led board_led(13);

static void blink()
{
    board_led.on();
    delay(50);
    board_led.off();
    delay(950);
}

void init()
{
    board_led.init();
}

void task(void *arg)
{
    (void)arg;

    while(true)
    {
        blink();
    }
}

}
