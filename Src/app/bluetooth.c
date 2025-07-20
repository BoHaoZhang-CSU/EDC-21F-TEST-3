#include "bluetooth.h"
#include "dma.h"
#include "usart.h"
extern UART_HandleTypeDef huart3;
PID_Parameters para_temp;
char received_data[110];
char send_data[50];
extern DMA_HandleTypeDef hdma_usart3_rx;
void parse_coding_data(char *coding_data,
                       PID_Parameters *parameters)
{
    // 本地拷贝并确保终止
    char buffer[110];
    strncpy(buffer, coding_data, sizeof(buffer));
    // 清空codeing_data
    memset(coding_data, 0, sizeof(buffer));
    buffer[sizeof(buffer) - 1] = '\0';

    // 按逗号和分号分割命令
    char *token = strtok(buffer, ",;");
    while (token != NULL)
    {
        // 去除前后空白
        while (isspace((unsigned char)*token))
            token++;
        char *end = token + strlen(token) - 1;
        while (end > token && isspace((unsigned char)*end))
            end--;
        *(end + 1) = '\0';

        // 提取命令和数值
        char cmd[9];
        float val = 0;
        if (sscanf(token, "%8[^ (](%f)", cmd, &val) == 2)
        {
            if (strcmp(cmd, "ch") == 0)
            {
                parameters->ch = val;
            }
            else if (strcmp(cmd, "target0") == 0)
            {
                parameters->target0 = val;
            }
            else if (strcmp(cmd, "target1") == 0)
            {
                parameters->target1 = val;
            }
            else if (strcmp(cmd, "target2") == 0)
            {
                parameters->target2 = val;
            }
            else if (strcmp(cmd, "kp0") == 0)
            {
                parameters->kp0 = val;
            }
            else if (strcmp(cmd, "ki0") == 0)
            {
                parameters->ki0 = val;
            }
            else if (strcmp(cmd, "kd0") == 0)
            {
                parameters->kd0 = val;
            }
            else if (strcmp(cmd, "kp1") == 0)
            {
                parameters->kp1 = val;
            }
            else if (strcmp(cmd, "ki1") == 0)
            {
                parameters->ki1 = val;
            }
            else if (strcmp(cmd, "kd1") == 0)
            {
                parameters->kd1 = val;
            }
            else if (strcmp(cmd, "kp2") == 0)
            {
                parameters->kp2 = val;
            }
            else if (strcmp(cmd, "ki2") == 0)
            {
                parameters->ki2 = val;
            }
            else if (strcmp(cmd, "kd2") == 0)
            {
                parameters->kd2 = val;
            }
        }
        token = strtok(NULL, ",;");
    }
}


// uart receive
float parameter_ch;
extern float Kp0, Ki0, Kd0;
extern float Kp1, Ki1, Kd1;
extern int target_speed_L, target_speed_R;

char rx_buffer_tmp[100];
uint8_t rx_buffer_tmp_cnt = 0;
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{

    if (huart == &huart3)
    {
        // 处理接收到的数据
        // 将接收到的数据存入缓冲区
        for (int i = 0; i < Size; i++)
        {
            rx_buffer_tmp[rx_buffer_tmp_cnt] = received_data[i];
            rx_buffer_tmp_cnt++;
        }
        if (rx_buffer_tmp[rx_buffer_tmp_cnt - 1] == ';') // 确保数据以分号结尾
        {
            rx_buffer_tmp[rx_buffer_tmp_cnt] = '\0'; // 在分号处写入终止符，形成合法 C 字符串
            HAL_UART_Transmit(&huart3, (uint8_t *)rx_buffer_tmp, strlen(rx_buffer_tmp), 100);
            HAL_UART_Transmit(&huart3, (uint8_t *)"|", 1, 100);
            parse_coding_data(rx_buffer_tmp, &para_temp);
            rx_buffer_tmp_cnt = 0; // 重置缓冲区索引，准备下一帧
        }

        // HAL_UART_Transmit(&huart3, (uint8_t *)received_data, strlen(received_data), 100);
        // HAL_UART_Transmit(&huart3, (uint8_t *)"|", 1, 100);

        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, (uint8_t *)received_data, sizeof(received_data));
        // 关闭dma传输过半中断
        __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
    }
}
