/*
 * command_interface.c
 *
 *  Created on: Apr 15, 2017
 *      Author: Lincster
 */
#include "command_interface.h"
#include <string.h>
#include "common.h"
#include "uart.h"

char response_buffer_b0[PI_HAT_SENSOR_CNT][MSG_LEN]={};
char response_buffer_b1[PI_HAT_SENSOR_CNT][MSG_LEN]={};
char response_buffer_b2[PI_HAT_SENSOR_CNT][MSG_LEN]={};
char response_buffer_b3[PI_HAT_SENSOR_CNT][MSG_LEN]={};
char response_buffer[4][PI_HAT_SENSOR_CNT][MSG_LEN]={};
char response_buffer_b4[DAQCS_SENSOR_CNT][MSG_LEN]={};

char response_status_b0[PI_HAT_SENSOR_CNT] = {{OLD}};
char response_status_b1[PI_HAT_SENSOR_CNT] = {{OLD}};
char response_status_b2[PI_HAT_SENSOR_CNT] = {{OLD}};
char response_status_b3[PI_HAT_SENSOR_CNT] = {{OLD}};
char response_status[4][PI_HAT_SENSOR_CNT] = {{{OLD}}};
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
// TODO: Auto Baud Rate
void read_response_val(int brd_num, char* sns, char** val){
	if(!((brd_num>=0)&&(brd_num<=4))){
		// error msg?
	}
	if((brd_num >= 0) && (brd_num <= 3)){
		if(strcmp(sns,"TD0")==0){
			response_status[brd_num][PI_TD0] = OLD;
			strcpy(*val,response_buffer[brd_num][PI_TD0]);
		}
		else if(strcmp(sns,"TB0")==0){
			response_status[brd_num][PI_TB0] = OLD;
			strcpy(*val,response_buffer[brd_num][PI_TB0]);
		}
		else if(strcmp(sns,"TB1")==0){
			response_status[brd_num][PI_TB1] = OLD;
			strcpy(*val,response_buffer[brd_num][PI_TB1]);
		}
		else if(strcmp(sns,"TE0")==0){
			response_status[brd_num][PI_TE0] = OLD;
			strcpy(*val,response_buffer[brd_num][PI_TE0]);
		}
		else if(strcmp(sns,"TE1")==0){
			response_status[brd_num][PI_TE1] = OLD;
			strcpy(*val,response_buffer[brd_num][PI_TE1]);
		}
		else if(strcmp(sns,"P0")==0){
			response_status[brd_num][PI_P0] = OLD;
			strcpy(*val,response_buffer[brd_num][PI_P0]);
		}
		else if(strcmp(sns,"P1")==0){
			response_status[brd_num][PI_P1] = OLD;
			strcpy(*val,response_buffer[brd_num][PI_P1]);
		}
		else if(strcmp(sns,"H")==0){
			response_status[brd_num][PI_H] = OLD;
			strcpy(*val,response_buffer[brd_num][PI_H]);
		}
		else if(strcmp(sns,"V")==0){
			response_status[brd_num][PI_V] = OLD;
			strcpy(*val,response_buffer[brd_num][PI_V]);
		}
		else if(strcmp(sns,"C")==0){
			response_status[brd_num][PI_C] = OLD;
			strcpy(*val,response_buffer[brd_num][PI_C]);
		}
		else {
			// error msg?
		}
	}
	else if(brd_num == 4){
		if(strcmp(sns,"TB0")==0){
			response_status_b4[DQ_TB0] = OLD;
			strcpy(*val,response_buffer_b4[DQ_TB0]);
		}
		else if(strcmp(sns,"P0")==0){
			response_status_b4[DQ_P0] = OLD;
			strcpy(*val,response_buffer_b4[DQ_P0]);
		}
		else if(strcmp(sns,"PB")==0){
			response_status_b4[DQ_PB] = OLD;
			strcpy(*val,response_buffer_b4[DQ_PB]);
		}
		else if(strcmp(sns,"V")==0){
			response_status_b4[DQ_V] = OLD;
			strcpy(*val,response_buffer_b4[DQ_V]);
		}
		else if(strcmp(sns,"C")==0){
			response_status_b4[DQ_C] = OLD;
			strcpy(*val,response_buffer_b4[DQ_C]);
		}
		else if(strcmp(sns,"XGY")==0){
			response_status_b4[DQ_XGY] = OLD;
			strcpy(*val,response_buffer_b4[DQ_XGY]);
		}
		else if(strcmp(sns,"XAC")==0){
			response_status_b4[DQ_XAC] = OLD;
			strcpy(*val,response_buffer_b4[DQ_XAC]);
		}
		else if(strcmp(sns,"YGY")==0){
			response_status_b4[DQ_YGY] = OLD;
			strcpy(*val,response_buffer_b4[DQ_YGY]);
		}
		else if(strcmp(sns,"YAC")==0){
			response_status_b4[DQ_YAC] = OLD;
			strcpy(*val,response_buffer_b4[DQ_YAC]);
		}
		else if(strcmp(sns,"ZGY")==0){
			response_status_b4[DQ_ZGY] = OLD;
			strcpy(*val,response_buffer_b4[DQ_ZGY]);
		}
		else if(strcmp(sns,"ZAC")==0){
			response_status_b4[DQ_ZAC] = OLD;
			strcpy(*val,response_buffer_b4[DQ_ZAC]);
		}
		else if(strcmp(sns,"MS")==0){
			response_status_b4[DQ_MS] = OLD;
			strcpy(*val,response_buffer_b4[DQ_MS]);
		}
		else if(strcmp(sns,"MC")==0){
			response_status_b4[DQ_MC] = OLD;
			strcpy(*val,response_buffer_b4[DQ_MC]);
		}
		else if(strcmp(sns,"MV")==0){
			response_status_b4[DQ_MV] = OLD;
			strcpy(*val,response_buffer_b4[DQ_MV]);
		}
		else if(strcmp(sns,"MD")==0){
			response_status_b4[DQ_MD] = OLD;
			strcpy(*val,response_buffer_b4[DQ_MD]);
		}
		else if(strcmp(sns,"ME")==0){
			response_status_b4[DQ_ME] = OLD;
			strcpy(*val,response_buffer_b4[DQ_ME]);
		}
		else {
			// error msg?
		}
	}
	else {
		// error msg?
	}
}
// TODO: initializing low for GPIO for cutdown and board resets.
void store_response_val(int brd_num,char* sns, char*val){
	if(!((brd_num>=0)&&(brd_num<=4))){
		// error msg?
	}
	if((brd_num >= 0) && (brd_num <= 3)){
		if(strcmp(sns,"TD0")==0){
			strcpy(response_buffer[brd_num][PI_TD0],val);
			response_status[brd_num][PI_TD0] = NEW;
		}
		else if(strcmp(sns,"TB0")==0){
			strcpy(response_buffer[brd_num][PI_TB0],val); // TODO: Null char needed?
			response_status[brd_num][PI_TB0] = NEW;
		}
		else if(strcmp(sns,"TB1")==0){
			strcpy(response_buffer[brd_num][PI_TB1],val);
			response_status[brd_num][PI_TB1] = NEW;
		}
		else if(strcmp(sns,"TE0")==0){
			strcpy(response_buffer[brd_num][PI_TE0],val);
			response_status[brd_num][PI_TE0] = NEW;
		}
		else if(strcmp(sns,"TE1")==0){
			strcpy(response_buffer[brd_num][PI_TE1],val);
			response_status[brd_num][PI_TE1] = NEW;
		}
		else if(strcmp(sns,"P0")==0){
			strcpy(response_buffer[brd_num][PI_P0],val);
			response_status[brd_num][PI_P0] = NEW;
		}
		else if(strcmp(sns,"P1")==0){
			strcpy(response_buffer[brd_num][PI_P1],val);
			response_status[brd_num][PI_P1] = NEW;
		}
		else if(strcmp(sns,"H")==0){
			strcpy(response_buffer[brd_num][PI_H],val);
			response_status[brd_num][PI_H] = NEW;
		}
		else if(strcmp(sns,"V")==0){
			strcpy(response_buffer[brd_num][PI_V],val);
			response_status[brd_num][PI_V] = NEW;
		}
		else if(strcmp(sns,"C")==0){
			strcpy(response_buffer[brd_num][PI_C],val);
			response_status[brd_num][PI_C] = NEW;
		}
		else {
			// error msg?
		}
	}
	else if(brd_num == 4){
		if(strcmp(sns,"TB0")==0){
			strcpy(response_buffer_b4[DQ_TB0],val);
			response_status_b4[DQ_TB0] = NEW;
		}
		else if(strcmp(sns,"P0")==0){
			strcpy(response_buffer_b4[DQ_P0],val);
			response_status_b4[DQ_P0] = NEW;
		}
		else if(strcmp(sns,"PB")==0){
			strcpy(response_buffer_b4[DQ_PB],val);
			response_status_b4[DQ_PB] = NEW;
		}
		else if(strcmp(sns,"V")==0){
			strcpy(response_buffer_b4[DQ_V],val);
			response_status_b4[DQ_V] = NEW;
		}
		else if(strcmp(sns,"C")==0){
			strcpy(response_buffer_b4[DQ_C],val);
			response_status_b4[DQ_C] = NEW;
		}
		else if(strcmp(sns,"XGY")==0){
			strcpy(response_buffer_b4[DQ_XGY],val);
			response_status_b4[DQ_XGY] = NEW;
		}
		else if(strcmp(sns,"XAC")==0){
			strcpy(response_buffer_b4[DQ_XAC],val);
			response_status_b4[DQ_XAC] = NEW;
		}
		else if(strcmp(sns,"YGY")==0){
			strcpy(response_buffer_b4[DQ_YGY],val);
			response_status_b4[DQ_YGY] = NEW;
		}
		else if(strcmp(sns,"YAC")==0){
			strcpy(response_buffer_b4[DQ_YAC],val);
			response_status_b4[DQ_YAC] = NEW;
		}
		else if(strcmp(sns,"ZGY")==0){
			strcpy(response_buffer_b4[DQ_ZGY],val);
			response_status_b4[DQ_ZGY] = NEW;
		}
		else if(strcmp(sns,"ZAC")==0){
			strcpy(response_buffer_b4[DQ_ZAC],val);
			response_status_b4[DQ_ZAC] = NEW;
		}
		else if(strcmp(sns,"MS")==0){
			strcpy(response_buffer_b4[DQ_MS],val);
			response_status_b4[DQ_MS] = NEW;
		}
		else if(strcmp(sns,"MC")==0){
			strcpy(response_buffer_b4[DQ_MC],val);
			response_status_b4[DQ_MC] = NEW;
		}
		else if(strcmp(sns,"MV")==0){
			strcpy(response_buffer_b4[DQ_MV],val);
			response_status_b4[DQ_MV] = NEW;
		}
		else if(strcmp(sns,"MD")==0){
			strcpy(response_buffer_b4[DQ_MD],val);
			response_status_b4[DQ_MD] = NEW;
		}
		else if(strcmp(sns,"ME")==0){
			strcpy(response_buffer_b4[DQ_ME],val);
			response_status_b4[DQ_ME] = NEW;
		}
		else {
			// error msg?
		}
	}
	else {
		// error msg?
	}
}
void parse_response(char* msg){
	char msg_orig[MSG_LEN];
	char* resp_brd = "";
	char* resp_sns = "";
	char* resp_val = "";
	strcpy(msg_orig,msg);
	rmv_start_end_chars(msg);
	int count = get_colon_count(msg);
	if (count == 2){
		// Insert specific code for handling 2 colon commands or call fnc
		two_colon_extract(msg,&resp_brd,&resp_sns,&resp_val);
		// TODO: LP future can do error checking for making sure valid msg
		if(strcmp(resp_brd,"B0")==0){
			store_response_val(0, resp_sns, resp_val);
		}
		else if(strcmp(resp_brd,"B1")==0){
			store_response_val(1, resp_sns, resp_val);
		}
		else if(strcmp(resp_brd,"B2")==0){
			store_response_val(2, resp_sns, resp_val);
		}
		else if(strcmp(resp_brd,"B3")==0){
			store_response_val(3, resp_sns, resp_val);
		}
		else if(strcmp(resp_brd,"B4")==0){
			store_response_val(4, resp_sns, resp_val);
		}
		else {
			// error msg?
		}
	}
	else {
		// error msg
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
					//TODO:
					// Respond back with latest of every sensor imaginable from reponse_buffer for each board
					// temp send to B0-B3 for testing.
					UART_write_msg(0,msg_orig);
					UART_write_msg(1,msg_orig);
					UART_write_msg(2,msg_orig);
					UART_write_msg(3,msg_orig);
				}
				else if(strcmp(msg,"FF")==0){
					// Trigger Cutdown TODO:
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
						// Trigger Board Reset to B0 TODO:
					}
					else if(strcmp(comms2_val,"B1")==0){
						// Trigger Board Reset to B1
					}
					else if(strcmp(comms2_val,"B2")==0){
						// Trigger Board Reset to B2
					}
					else if(strcmp(comms2_val,"B3")==0){
						// Trigger Board Reset to B3
					}
					else {
						// error msg?
					}
				}
				else if(strcmp(comms2_cmd,"06")==0){
					// forward same msg to all 4 pis
					UART_write_msg(0,msg_orig);
					UART_write_msg(1,msg_orig);
					UART_write_msg(2,msg_orig);
					UART_write_msg(3,msg_orig);
					// forward same msg to motor MSP
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
					UART_write_msg(0,msg_orig);
					// something for preparing to receive later/immediately?
				}
				else if(strcmp(comms3_brd,"B1")==0){
					// forward command to B1
					UART_write_msg(1,msg_orig);
					// something for preparing to receive later/immediately?
				}
				else if(strcmp(comms3_brd,"B2")==0){
					// forward command to B2
					UART_write_msg(2,msg_orig);
					// something for preparing to receive later/immediately?
				}
				else if(strcmp(comms3_brd,"B3")==0){
					// forward command to B3
					UART_write_msg(3,msg_orig);
					// something for preparing to receive later/immediately?
					// TODO: decide on adding logic to ensure a response for every 00 sent
				}
				else if(strcmp(comms3_brd,"B4")==0){
					// forward command to Motor MSP
					// something for preparing to receive later/immediately?
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
