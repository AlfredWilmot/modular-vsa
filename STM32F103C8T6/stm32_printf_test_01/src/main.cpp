#include <mbed.h>

#define LED_BUILTIN PC_13
#define SERIAL2_TX PA_2
#define SERIAL2_RX PA_3

Serial serial2(SERIAL2_TX, SERIAL2_RX, 115200);
DigitalOut led(LED_BUILTIN);

int main()
{
    // put your setup code here, to run once:
    serial2.printf("STM32 bluepill mbed test.\n");

    while (1)
    {
        // put your main code here, to run repeatedly:
        led = 0;
        wait_ms(500);
        led = 1;
        wait_ms(500);

        serial2.printf("Blinking...\n");
    }
}