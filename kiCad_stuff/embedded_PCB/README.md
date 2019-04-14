# An overview of the parts needed for assembling the embedded segment control PCB
------
## Motor-driver IC:
------
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
* Requirements:
	* Buffers common segment actuator power-line from segment, giving a stable 6V, supplying up-to to 4-5A
------
## MAX345 transciever;
------
* Requirements:
	* Impedance matching transmission lines?
		* Terminating/ biasing circuits.

* Parts:
	* [MAX48X Series data-sheet](https://docs-emea.rs-online.com/webdocs/078a/0900766b8078aed6.pdf)
	* [Buy MAX485CSA+](https://uk.rs-online.com/web/p/line-transceivers/5404564/)
------
## Microprocessor (STM?)
------
* Requirement 
  * 4 ADC's for I-sensing (just in case this turns out to be useful)
  * 8 PWM pins for motor drivers
  * Programming pins (ST LINK)
  * RS485 MAX chip req?
  * Programming pins
  * USART for debugging?
  * MAKE SURE TO INCLUE PROBE POINTS FOR DEBUGGING

  
------
## Design notes
------

* Buck-converter is a switching voltage regulator:
	* A lot more efficient (>=80%!) than normal voltage regulator (which wastes any power that's unused by the load via thermal losses)
	* Generates noise due to switching
	* More expensive than voltage regulator