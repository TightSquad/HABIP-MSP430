/*
 * command_interface.c
 *
 *  Created on: Apr 15, 2017
 *      Author: Lincster
 */
#include "command_interface.h"
#include <string.h>
#include "common.h"

char response_buffer_b0[PI_HAT_SENSOR_CNT][MSG_LEN]={};
char response_buffer_b1[PI_HAT_SENSOR_CNT][MSG_LEN]={};
char response_buffer_b2[PI_HAT_SENSOR_CNT][MSG_LEN]={};
char response_buffer_b3[PI_HAT_SENSOR_CNT][MSG_LEN]={};
char response_buffer_b4[DAQCS_SENSOR_CNT][MSG_LEN]={};

char response_status_b0[PI_HAT_SENSOR_CNT] = {{OLD}};
char response_status_b1[PI_HAT_SENSOR_CNT] = {{OLD}};
char response_status_b2[PI_HAT_SENSOR_CNT] = {{OLD}};
char response_status_b3[PI_HAT_SENSOR_CNT] = {{OLD}};
char response_status_b4[DAQCS_SENSOR_CNT] = {{OLD}};

void rmv_start_end_chars(char* s){
	if(strstr(s,"{")!=NULL){
		strncpy(s,s+1,strlen(s)-1);
		s[strlen(s)-1]='\0';
	}
	if(strstr(s,"}")!=NULL){
		s[strlen(s)-1]='\0';
	}
}

int get_colon_count(const char* s){
	// Note: if passed a string with 3 or more colons, will return 2.
	char* pcolon = strstr(s,":");
	if(pcolon == NULL){
		return 0;
	}
	else{
		char* pcolon2 = strstr(pcolon+1,":");
		if(pcolon2 == NULL){
			return 1;
		}
		else {
			return 2;
		}
	}
}

void parse_cmd_from_comms(char* msg){
	char msg_orig[MSG_LEN];
	char* comms2_cmd = "";
	char* comms2_val = "";
	char* comms3_cmd = "";
	char* comms3_brd = "";
	char* comms3_sns = "";
	strcpy(msg_orig,msg);
	rmv_start_end_chars(msg);
	int count = get_colon_count(msg);
	switch(count)
		{
			case 0:
				// Insert specific code for handling 0 colon commands or call fnc
				if(strcmp(msg,"01")==0){
					// Respond back with latest of every sensor imaginable from reponse_buffer for each board
				}
				else if(strcmp(msg,"FF")==0){
					// Trigger Cutdown
				}
				else {
					// error msg?
				}
				break;
			case 1:
				// Insert specific code for handling 1 colon commands or call fnc
				one_colon_extract(msg,&comms2_cmd,&comms2_val);
				if((strcmp(comms2_cmd,"03")==0)||(strcmp(comms2_cmd,"04")==0)){
					// forward reaction wheel command to motor msp
				}
				else if(strcmp(comms2_cmd,"05")==0){
					if(strcmp(comms2_val,"B0")==0){
						// Send Board Reset to B0
					}
					else if(strcmp(comms2_val,"B1")==0){
						// Send Board Reset to B1
					}
					else if(strcmp(comms2_val,"B2")==0){
						// Send Board Reset to B2
					}
					else if(strcmp(comms2_val,"B3")==0){
						// Send Board Reset to B3
					}
					else {
						// error msg?
					}
				}
				else if(strcmp(comms2_cmd,"06")==0){
					// send same msg to all 4 pis and to motor MSP
					// update own timestamp.
				}
				else {
					// error msg?
				}

				break;
			case 2:
				// Insert specific code for handling 2 colon commands or call fnc
				two_colon_extract(msg,&comms3_cmd,&comms3_brd,&comms3_sns);
				// TODO: LP future can do error checking for making sure valid msg from comms for other areas
				if(strcmp(comms3_brd,"B0")==0){
					// forward command to B0
				}
				else if(strcmp(comms3_brd,"B1")==0){
					// forward command to B1
				}
				else if(strcmp(comms3_brd,"B2")==0){
					// forward command to B2
				}
				else if(strcmp(comms3_brd,"B3")==0){
					// forward command to B3
					// TODO: decide on adding logic to ensure a response for every 00 sent
				}
				else if(strcmp(comms3_brd,"B4")==0){
					// forward command to Motor MSP
					// note sensors may be deactivated atm
				}
				else {
					// error msg?
				}
				break;

			default: break;
		}
}
void one_colon_extract(char* msg, char** first, char** second){
	*first = strtok(msg,":");
	*second = strtok(NULL,":");
}
void two_colon_extract(char* msg, char** first, char** second, char** third){
	*first = strtok(msg,":");
	*second = strtok(NULL,":");
	*third = strtok(NULL,":");
}
