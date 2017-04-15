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
