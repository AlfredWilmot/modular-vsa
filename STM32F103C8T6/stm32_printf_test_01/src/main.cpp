#include <mbed.h>

#define LED_BUILTIN PC_13
#define SERIAL1_TX PB_6
#define SERIAL1_RX PB_7

Serial serial1(SERIAL1_TX, SERIAL1_RX, 115200);
DigitalOut led(LED_BUILTIN);
AnalogIn I_sense_1(PB_1);
AnalogIn I_sense_2(PB_0);



int main()
{
    // put your setup code here, to run once:
    serial1.printf("STM32 bluepill mbed test.\n");

    while (1)
    {

        led = 1;
        wait_ms(5);

        led = 0;
        wait_ms(5);

        serial1.printf("I-sense 1: %d\n\r", I_sense_1.read_u16());
        serial1.printf("I-sense 2: %d\n\r", I_sense_2.read_u16());

    }
}