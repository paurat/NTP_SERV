/*
 * myapi.c
 *
 *  Created on: Mar 4, 2022
 *      Author: User
 */
#include "myapi.h"
#include "local_files.h"
#define JSON_SIZE 1608

int fs_read_custom(struct fs_file *file, char *buffer, int count){
	return 0;
}
int fs_open_custom(struct fs_file *file, const char *name){
	char generated_html[JSON_SIZE];
	memset(generated_html, 0, JSON_SIZE);
	u16_t offset = 0;

	if (!strcmp(name, "/info.json")) {
			offset = sprintf(generated_html,"[{\"IPaddress\":\"%d",\"Timezone\":\"%d",\"contacts\":\"%d",\"software_version\":\"0.000000001\",\"mac\":\"no\"}]",user_info.ip,user_info.zone,user_info.contacts);
	}
	else if (!strcmp(name, "/uptime.json")) {
			RTC_DateTypeDef dateStruct;
			RTC_TimeTypeDef timeStruct;

			//hrtc.Instance = RTC;

			// Read actual date and time
			HAL_RTC_GetTime(&hrtc, &timeStruct, FORMAT_BIN); // Read time first!
			HAL_RTC_GetDate(&hrtc, &dateStruct, FORMAT_BIN);
			int Hours=timeStruct.Hours;
			int Minutes=timeStruct.Minutes;
			int Seconds=timeStruct.Seconds;
			int Date=dateStruct.Date;
			int Month=dateStruct.Month;
			int Year=dateStruct.Year+2000;


		offset = sprintf(generated_html,"%02d:%02d:%02d %02d.%02d.%04d",Hours,Minutes,Seconds,Date,Month,Year);
	}

	if (offset>0){
		memset(file, 0, sizeof(struct fs_file));
		file->pextension = mem_malloc(offset);
	}

	if (file->pextension != NULL) {
		/* instead of doing memcpy, you would generate e.g. a JSON here */
		memcpy(file->pextension, generated_html, offset);
		file->data = (const char *)file->pextension;
		file->len = offset; /* don't send the trailing 0 */
		file->index = file->len;
		/* allow persisteng connections */
		file->flags = FS_FILE_FLAGS_HEADER_INCLUDED;
		return 1;
	}

	printf("Extension fail %s\n\r",name);
	return 0;
}
void fs_close_custom(struct fs_file *file){
	if (file && file->pextension) {
		mem_free(file->pextension);
		file->pextension = NULL;
	}
}

