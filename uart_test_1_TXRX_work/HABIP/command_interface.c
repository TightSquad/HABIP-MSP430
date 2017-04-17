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
	rmv_start_end_chars(msg);
	int count = get_colon_count(msg);
	switch(count)
		{
			case 0:
				// Insert specific code for handling 0 colon commands or call fnc
				if(strcmp(msg,"01")==0){
					// Insert message sends to all 5 ?
					// Update State machine to be receiving all ? or just grab from latest?
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
				char* comms_cmd = "";
				char* comms_val = "";
				one_colon_extraction(msg,&comms_cmd,&comms_val);
				if((strcmp(comms_cmd,"03")==0)||(strcmp(comms_cmd,"04")==0)){
					// forward reaction wheel command to motor msp
				}
				else if(strcmp(comms_cmd,"05")==0){
					if(strcmp(comms_val,"B0")==0){
						// Send Board Reset to B0
					}
					else if(strcmp(comms_val,"B1")==0){
						// Send Board Reset to B1
					}
					else if(strcmp(comms_val,"B2")==0){
						// Send Board Reset to B2
					}
					else if(strcmp(comms_val,"B3")==0){
						// Send Board Reset to B3
					}
					else {
						// error msg?
					}
				}
				else if(strcmp(comms_cmd,"06")==0){
					// send same msg to all 4 pis and to motor MSP
					// update own timestamp.
				}
				else {
					// error msg?
				}

				break;
			case 2:
				// Insert specific code for handling 2 colon commands or call fnc
				char* comms_cmd = "";
				char* comms_brd = "";
				char* comms_sns = "";
				two_colon_extraction(msg,&comms_cmd,&comms_brd,&comms_sns);

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
