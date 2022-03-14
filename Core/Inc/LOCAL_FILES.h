/*
 * LOCAL_FILES.h
 *
 *  Created on: Mar 10, 2022
 *      Author: User
 */

#ifndef INC_LOCAL_FILES_H_
#define INC_LOCAL_FILES_H_

#define INFOLEN 255
//++++++++++++++++++++Structs+++++++++++++++++++++++++

struct user_info {
	char ip[16];
	int zone;
	char contacts[INFOLEN];
} user_info;

struct Time_rx
{
	char day[3];
	char month[3];
	char year[5];
	char hours[3];
	char minuttes[3];
	char seconds[6];
	char errors[2];
	char sinc[2];
}gps;
#endif /* INC_LOCAL_FILES_H_ */
