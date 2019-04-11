# An overview of the parts needed for assembling the embedded segment control PCB
------
## Motor-driver IC:

* Requirements:
  * Source 2A max (1A is sufficient, but good to have wiggle-room for initial testing)
  * Bidirectional (Full-H bridge)

* Parts:
  * [BD622xxx Series data-sheet](https://docs-emea.rs-online.com/webdocs/1385/0900766b81385cec.pdf)
  * [Buy BD6222HFP-TR online](https://uk.rs-online.com/web/p/motor-driver-ics/6996752/)
  
  * [BD6735FV data-sheet](https://docs-emea.rs-online.com/webdocs/1623/0900766b8162326e.pdf)
  * [Buy BD6735FV online](https://uk.rs-online.com/web/p/motor-driver-ics/1714118/)
------
## Buck-converter: 6V motor PWR line, 5V MCU logic 
------
## Microprocessor (STM?)

* Requirement 
  * 4 ADC's for I-sensing (just in case this turns out to be useful)
  * 8 PWM pins for motor drivers
  * Programming pins (ST LINK)
  * RS485 MAX chip req?
  * Programming pins
  * USART for debugging?
------
