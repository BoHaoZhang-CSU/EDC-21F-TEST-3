/*
 * fun.c
 *
 *  Created on: May 12, 2025
 *      Author: ZBH
 */


#include "Fun.h"

void parse_coding_data(char* coding_data,
                       float* ch,
                       float* value_1,
                       float* value_2,
                       float* value_3,
                       float* target

) {
    // 本地拷贝并确保终止
    char buffer[50];
    strncpy(buffer, coding_data, sizeof(buffer));
    buffer[sizeof(buffer) - 1] = '\0';

    // 按逗号和分号分割命令
    char* token = strtok(buffer, ",;");
    while (token != NULL) {
        // 去除前后空白
        while (isspace((unsigned char)*token)) token++;
        char* end = token + strlen(token) - 1;
        while (end > token && isspace((unsigned char)*end)) end--;
        *(end + 1) = '\0';

        // 提取命令和数值
        char cmd[8];
        float val = 0;
        if (sscanf(token, "%7[^ (](%f)", cmd, &val) == 2) {
            if (strcmp(cmd, "ch") == 0 ) {
                *ch = val;
            } else if (strcmp(cmd, "kp0") == 0) {
                *value_1 = val;
            } else if (strcmp(cmd, "ki0") == 0 ) {
                *value_2 = val;
            } else if (strcmp(cmd, "kd0") == 0) {
                *value_3 = val;
            }else if (strcmp(cmd, "kp1") == 0) {
                *target = val;
            }
        }
        token = strtok(NULL, ",;");
    }
}
