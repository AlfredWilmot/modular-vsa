import serial
import time 
import io

#inlcude 

ser = serial.Serial(
	port='/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_00000000-if00-port0',
	baudrate=115200,
	# parity=serial.PARITY_ODD,
	# stopbits=serial.STOPBITS_TWO,
	# bytesize=serial.SEVENBITS
)


# recieve data from MCU, publish it to ROS network.
# recieve motor data from ROS network, check if UART line is clear, transmit motor command data to MCU.
# MCU does the same (checks that UART line is clear before sending data)



# TEST 1: print confirmation message to terminal, whenever data is recieved from MCU.

# Test 2: Check that MCU is not sending data, send data to MCU, listen for response.