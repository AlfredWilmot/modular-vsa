EESchema Schematic File Version 4
LIBS:Current_sensing_circuits-cache
EELAYER 26 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Current_sensing_circuits-rescue:INA283 U1
U 1 1 5C8900FA
P 5900 3500
F 0 "U1" H 6050 3650 50  0000 L CNN
F 1 "INA283" H 6050 3400 50  0000 L CNN
F 2 "SMD_Packages:SOIC-8-N" H 5900 3500 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/ina282.pdf" H 5900 3500 50  0001 C CNN
	1    5900 3500
	1    0    0    -1  
$EndComp
$Comp
L Device:R R_shunt1
U 1 1 5C8901B3
P 5350 3500
F 0 "R_shunt1" V 5430 3500 50  0000 C CNN
F 1 "0.02" V 5250 3500 50  0000 C CNN
F 2 "Resistors_SMD:R_2512" V 5280 3500 50  0001 C CNN
F 3 "" H 5350 3500 50  0001 C CNN
	1    5350 3500
	1    0    0    -1  
$EndComp
Text GLabel 6550 3500 2    60   Output ~ 0
ADC_OUT
Wire Wire Line
	6550 3500 6200 3500
Text GLabel 5350 3200 1    60   BiDi ~ 0
PWR-
Text GLabel 5350 3800 3    60   BiDi ~ 0
PWR+
Wire Wire Line
	5350 3200 5350 3300
Wire Wire Line
	5350 3650 5350 3700
Wire Wire Line
	5600 3600 5600 3700
Wire Wire Line
	5600 3700 5350 3700
Connection ~ 5350 3700
Wire Wire Line
	5600 3400 5600 3300
Wire Wire Line
	5600 3300 5350 3300
Connection ~ 5350 3300
$Comp
L Device:C C_bypass1
U 1 1 5C890899
P 6050 3100
F 0 "C_bypass1" V 5900 3100 50  0000 L CNN
F 1 "0.1u" H 6075 3000 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 6088 2950 50  0001 C CNN
F 3 "" H 6050 3100 50  0001 C CNN
	1    6050 3100
	0    1    1    0   
$EndComp
Wire Wire Line
	5900 3100 5900 3300
Wire Wire Line
	6000 3300 6300 3300
Wire Wire Line
	6300 3300 6300 3100
Wire Wire Line
	6300 3100 6200 3100
Text GLabel 6300 3100 2    60   Input ~ 0
GND
Text GLabel 5900 3700 3    60   Input ~ 0
GND
Text GLabel 5900 3100 0    60   Input ~ 0
Vcc
Text GLabel 6000 3750 2    60   Input ~ 0
Vcc
Wire Wire Line
	6000 3750 6000 3700
$Comp
L Current_sensing_circuits-rescue:Conn_01x02 J1
U 1 1 5C890FE5
P 4900 3500
F 0 "J1" H 4900 3600 50  0000 C CNN
F 1 "Conn_01x02" H 4900 3300 50  0000 C CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x02_P2.54mm_Vertical" H 4900 3500 50  0001 C CNN
F 3 "" H 4900 3500 50  0001 C CNN
	1    4900 3500
	1    0    0    -1  
$EndComp
$Comp
L Current_sensing_circuits-rescue:Conn_01x03 J2
U 1 1 5C891056
P 7400 3500
F 0 "J2" H 7400 3700 50  0000 C CNN
F 1 "Conn_01x03" H 7400 3300 50  0000 C CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x03_P2.54mm_Vertical" H 7400 3500 50  0001 C CNN
F 3 "" H 7400 3500 50  0001 C CNN
	1    7400 3500
	-1   0    0    1   
$EndComp
Text GLabel 4700 3500 0    60   BiDi ~ 0
PWR+
Text GLabel 4700 3600 0    60   BiDi ~ 0
PWR-
Text GLabel 7750 3500 2    60   Input ~ 0
Vcc
Text GLabel 7750 3350 1    60   Input ~ 0
GND
Text GLabel 7750 3700 3    60   Input ~ 0
ADC_OUT
Wire Wire Line
	7600 3600 7750 3600
Wire Wire Line
	7750 3600 7750 3700
Wire Wire Line
	7750 3350 7750 3400
Wire Wire Line
	7750 3400 7600 3400
Wire Wire Line
	7600 3500 7750 3500
Wire Wire Line
	5350 3700 5350 3800
Wire Wire Line
	5350 3300 5350 3350
$EndSCHEMATC
