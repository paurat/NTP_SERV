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

#endif /* INC_LOCAL_FILES_H_ */
