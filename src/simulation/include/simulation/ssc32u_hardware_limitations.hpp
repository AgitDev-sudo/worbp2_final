/*
 * ssc32u_hardware_limitations.hpp
 *
 * Author: agit
 */

#ifndef SSC32U_BOARD_INTERFACE_INCLUDE_SSC32U_HARDWARE_LIMITATIONS_H_
#define SSC32U_BOARD_INTERFACE_INCLUDE_SSC32U_HARDWARE_LIMITATIONS_H_

namespace ssc32u
{
	/*SSC32U servo board controller hardware limitations  */
	/*These limitations are extracted from the lynxmotion_ssc-32u_usb_user_guide.pdf*/
	//Note: Op pagina 24 staat dat time in microseconds is, maar in de voorbeelden en op pagina 25 staat dat het MILISECONDS is!!

    //ALLE DEFINES ZIJN IN MICROSECONDS
	// -90 graden
	/*@see page 4 & 5*/
	#define MIN_PULSE_WIDTH 500 // 500 µs = 0.5 ms
	// 0 graden: When board is turned on for the first time. It does not now what is position is. This default pwm value will then be used.
	/*@see page 4 & 5*/
	#define DEFAULT_PULSE_WIDTH 1500 // 1500 µs = 1.5 ms
	// 90 graden
	/*@see page 4 & 5*/
	#define MAX_PULSE_WIDTH 2500 // 2500 µs = 2.5 ms

	/*@see page 4 & 5*/
	/*R/C Timed Pulse (repeated every 20ms)*/
	#define SERVO_REFRESH_INTERVAL 20000 // minumim time to refresh servos in microseconds (µs) = 20 ms

    /*@see page 3*/
	#define MAX_SERVO_ALLOWED 32; //I/O channels [0-31]

}


#endif /* SSC32U_BOARD_INTERFACE_INCLUDE_SSC32U_HARDWARE_LIMITATIONS_H_ */
