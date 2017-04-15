/*
 * command_interface.h
 *
 *  Created on: Apr 15, 2017
 *      Author: Lincster
 */

#ifndef HABIP_COMMAND_INTERFACE_H_
#define HABIP_COMMAND_INTERFACE_H_

// Pi Hat Board Sensor Info Indicies
#define PI_HAT_SENSOR_CNT 10
#define PI_TD0 0
#define PI_TB0 1
#define PI_TB1 2
#define PI_TE0 3
#define PI_TE1 4
#define PI_P0 5
#define PI_P1 6
#define PI_H 7
#define PI_V 8
#define PI_C 9

int get_colon_count(const char* s);

#endif /* HABIP_COMMAND_INTERFACE_H_ */
