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

// DAQCS Board Sensor Info Indicies
#define DAQCS_SENSOR_CNT 16
#define DQ_TB0 0
#define DQ_P0 1
#define DQ_PB 2
#define DQ_V 3
#define DQ_C 4
#define DQ_XGY 5
#define DQ_XAC 6
#define DQ_YGY 7
#define DQ_YAC 8
#define DQ_ZGY 9
#define DQ_ZAC 10
#define DQ_MS 11
#define DQ_MC 12
#define DQ_MV 13
#define DQ_MD 14
#define DQ_ME 15

// response_buffer data status
#define OLD 0x00
#define NEW 0x01
#define ERROR 0xEE

int get_colon_count(const char* s);
void rmv_start_end_chars(char* s);
void store_response_val(int brd_num, char* sns, char* val);
void read_response_val(int brd_num, char* sns, char** val);
void parse_cmd_from_comms(char* msg);
void parse_response(char* msg);
void one_colon_extract(char* msg, char** first, char** second);
void two_colon_extract(char* msg, char** first, char** second, char** third);
#endif /* HABIP_COMMAND_INTERFACE_H_ */
