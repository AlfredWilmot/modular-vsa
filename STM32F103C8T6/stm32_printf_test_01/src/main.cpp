#include <mbed.h>
#include "ros.h"
#include "std_msgs/UInt16.h"


#define LED_BUILTIN PC_13
#define SERIAL1_TX PA_9
#define SERIAL1_RX PA_10

const int prxml_encdr_addr = 77; //0x4D
const int dstl_encdr_addr = 72; //0x48


/* STM blue-pill serial tutorial here: https://www.shortn0tes.com/2017/06/how-to-use-platformio-to-develop-for.html*/


Serial serial1(SERIAL1_TX, SERIAL1_RX, 115200);
DigitalOut led(LED_BUILTIN);

I2C i2c_encoders(PB_7, PB_6);



int main()
{
    

    char raw_data[2];
    unsigned int data = 0;

    while (1)
    {
        
        /* Blink LED */
        led = 1;
        wait_ms(50);

        led = 0;
        wait_ms(50);


        /* I2C stuff */

        // i2c_encoders.start();
        i2c_encoders.read(prxml_encdr_addr, raw_data, 2, false); //sending byte packet (address + ReadBit)

        data = int(raw_data[0]<<8 | raw_data[1]);
        serial1.printf("Proximal_encoder: %d\n\r", data);

        // serial1.printf("I-sense 1: %d\n\r", I_sense_1.read_u16());
        // serial1.printf("I-sense 2: %d\n\r", I_sense_2.read_u16());


    }
}