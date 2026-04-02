/*
 * 调试输出示例代码
 * 将此代码添加到 main.c 的 while(1) 循环中，即可通过USART4输出摇杆数据
 */

#include <stdio.h>
#include <string.h>

// 外部声明CRSF数据（已在crsf.c中定义）
extern CRSF_Data crsf_data;
extern UART_HandleTypeDef huart4;

// 调试输出缓冲区
char debug_buffer[128];

/**
 * @brief 发送摇杆数据到USART4（调试用）
 * 数据格式: Left_X,Left_Y,Right_X,Right_Y,S1,S2,A,B,C,D,E,F\r\n
 */
void Debug_SendJoystickData(void)
{
    // 格式化数据为CSV格式
    // 格式: Left_X,Left_Y,Right_X,Right_Y,S1,S2,A,B,C,D,E,F
    int len = snprintf(debug_buffer, sizeof(debug_buffer),
        "%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%d,%d,%d,%d,%d,%d\r\n",
        crsf_data.Left_X,
        crsf_data.Left_Y,
        crsf_data.Right_X,
        crsf_data.Right_Y,
        crsf_data.S1,
        crsf_data.S2,
        crsf_data.A,
        crsf_data.B,
        crsf_data.C,
        crsf_data.D,
        crsf_data.E,
        crsf_data.F
    );

    // 通过USART4发送数据
    if (len > 0 && len < sizeof(debug_buffer))
    {
        HAL_UART_Transmit(&huart4, (uint8_t*)debug_buffer, len, 10);
    }
}

/**
 * @brief 发送格式化的调试信息到USART4
 * @param format 格式化字符串（printf风格）
 * @param ... 可变参数
 */
void Debug_Printf(const char* format, ...)
{
    va_list args;
    int len;

    va_start(args, format);
    len = vsnprintf(debug_buffer, sizeof(debug_buffer), format, args);
    va_end(args);

    if (len > 0 && len < sizeof(debug_buffer))
    {
        HAL_UART_Transmit(&huart4, (uint8_t*)debug_buffer, len, 10);
    }
}


/* ===== 使用示例 ===== */

// 在 main.c 的 while(1) 循环中添加：
// 注意：需要包含头文件 #include <stdarg.h>

/*
while (1)
{
    // 原有的电机测试代码
    Motor_Test(MOTOR_1);
    Motor_Test(MOTOR_2);

    // 每50ms发送一次摇杆数据（约20Hz）
    static uint32_t last_send_time = 0;
    uint32_t current_time = HAL_GetTick();
    if (current_time - last_send_time >= 50)
    {
        last_send_time = current_time;
        Debug_SendJoystickData();
    }

    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
}
*/

// 或者如果你想发送更详细的信息：
/*
while (1)
{
    // 每100ms发送一次完整调试信息
    static uint32_t last_send_time = 0;
    uint32_t current_time = HAL_GetTick();
    if (current_time - last_send_time >= 100)
    {
        last_send_time = current_time;

        Debug_Printf("=== CRSF Data ===\r\n");
        Debug_Printf("Left:  X=%6.1f Y=%6.1f\r\n", crsf_data.Left_X, crsf_data.Left_Y);
        Debug_Printf("Right: X=%6.1f Y=%6.1f\r\n", crsf_data.Right_X, crsf_data.Right_Y);
        Debug_Printf("Slider: S1=%5.1f S2=%5.1f\r\n", crsf_data.S1, crsf_data.S2);
        Debug_Printf("Switches: A=%d B=%d C=%d D=%d E=%d F=%d\r\n",
                     crsf_data.A, crsf_data.B, crsf_data.C,
                     crsf_data.D, crsf_data.E, crsf_data.F);
        Debug_Printf("Link Quality: %d%%\r\n", crsf_data.uplink_Link_quality);
        Debug_Printf("=================\r\n\r\n");
    }
}
*/
