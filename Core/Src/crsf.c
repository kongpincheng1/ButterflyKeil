#include "crsf.h"
#include "usart.h"
#include <string.h>

extern DMA_HandleTypeDef hdma_usart1_rx;

uint8_t crsf_data_temp[36] = {0};

CRSF_Data crsf_data;

// CRSF协议使用的CRC-8/DVB-S2校验表
static const uint8_t crsf_crc8_table[256] = {
    0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
    0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06, 0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
    0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
    0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
    0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9, 0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
    0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
    0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
    0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F, 0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
    0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
    0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9, 0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
    0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F, 0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
    0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
    0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
    0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74, 0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
    0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
    0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0, 0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9
};

/**
 * @brief 计算CRSF CRC-8校验值
 * @param ptr 数据指针
 * @param len 数据长度
 * @return CRC校验值
 */
static uint8_t CRSF_CalcCRC(const uint8_t *ptr, uint8_t len)
{
    uint8_t crc = 0;
    while (len--)
    {
        crc = crsf_crc8_table[crc ^ *ptr++];
    }
    return crc;
}

float CRSF_FloatMap(float input_value, float input_min, float input_max, float output_min, float output_max)
{
    float output_value;
    if (input_value < input_min)
    {
        output_value = output_min;
    }
    else if (input_value > input_max)
    {
        output_value = output_max;
    }
    else
    {
        output_value = output_min + (input_value - input_min) * (output_max - output_min) / (input_max - input_min);
    }
    return output_value;
}

float CRSF_FloatMapWithMedian(float input_value, float input_min, float input_max, float median, float output_min, float output_max)
{
    float output_median = (output_max - output_min) / 2 + output_min;
    if (input_min >= input_max || output_min >= output_max || median <= input_min || median >= input_max)
    {
        return output_min;
    }

    if (input_value < median)
    {
        return CRSF_FloatMap(input_value, input_min, median, output_min, output_median);
    }
    else
    {
        return CRSF_FloatMap(input_value, median, input_max, output_median, output_max);
    }
}

void CRSF_Init(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, crsf_data_temp, MAX_FRAME_SIZE);
    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
}

void CRSF_UART_RxCallback(uint16_t Size)
{
    // 最小帧长度检查：地址(1) + 长度(1) + 类型(1) + CRC(1) = 4字节
    if (Size < 4)
    {
        memset(crsf_data_temp, 0, sizeof(crsf_data_temp));
        CRSF_Init();
        return;
    }

    // 检查地址（支持飞行控制器地址和广播地址）
    if (crsf_data_temp[0] != CRSF_ADDRESS_FLIGHT_CONTROLLER &&
        crsf_data_temp[0] != CRSF_ADDRESS_BROADCAST)
    {
        memset(crsf_data_temp, 0, sizeof(crsf_data_temp));
        CRSF_Init();
        return;
    }

    // 获取声明的数据长度（从帧类型到CRC的字节数）
    uint8_t declared_len = crsf_data_temp[1];

    // 验证数据长度在合理范围内（至少包含帧类型1字节 + CRC 1字节 = 2字节）
    // 且不超过最大帧大小
    if (declared_len < 2 || declared_len > (MAX_FRAME_SIZE - 2))
    {
        memset(crsf_data_temp, 0, sizeof(crsf_data_temp));
        CRSF_Init();
        return;
    }

    // 验证实际接收长度与声明长度一致（地址 + 长度字段 + 声明长度 = 2 + declared_len）
    if (Size != (uint16_t)(declared_len + 2))
    {
        memset(crsf_data_temp, 0, sizeof(crsf_data_temp));
        CRSF_Init();
        return;
    }

    // CRC校验：计算从帧类型到数据末尾的CRC（不包括CRC字节本身）
    // 数据长度字段declared_len包含帧类型 + 数据 + CRC，所以计算CRC的长度是declared_len - 1
    uint8_t calc_crc = CRSF_CalcCRC(&crsf_data_temp[2], declared_len - 1);
    uint8_t recv_crc = crsf_data_temp[declared_len + 1]; // CRC在地址(0) + 长度(1) + 数据(declared_len-1)之后

    if (calc_crc != recv_crc)
    {
        memset(crsf_data_temp, 0, sizeof(crsf_data_temp));
        CRSF_Init();
        return;
    }

    // 获取帧类型
    uint8_t frame_type = crsf_data_temp[2];

    // 根据帧类型解析数据
    switch (frame_type)
    {
    case CRSF_FRAMETYPE_RC_CHANNELS_PACKED:
        // 通道数据帧：需要至少22字节数据（16个通道 * 11位 = 176位 = 22字节）
        // declared_len = 1(帧类型) + 22(数据) + 1(CRC) = 24 = 0x18
        if (declared_len < CHANNELS_Frame_Length)
        {
            break;
        }
        crsf_data.channels[0] = ((uint16_t)crsf_data_temp[3] >> 0 | ((uint16_t)crsf_data_temp[4] << 8)) & 0x07FF;
        crsf_data.channels[1] = ((uint16_t)crsf_data_temp[4] >> 3 | ((uint16_t)crsf_data_temp[5] << 5)) & 0x07FF;
        crsf_data.channels[2] = ((uint16_t)crsf_data_temp[5] >> 6 | ((uint16_t)crsf_data_temp[6] << 2) | ((uint16_t)crsf_data_temp[7] << 10)) & 0x07FF;
        crsf_data.channels[3] = ((uint16_t)crsf_data_temp[7] >> 1 | ((uint16_t)crsf_data_temp[8] << 7)) & 0x07FF;
        crsf_data.channels[4] = ((uint16_t)crsf_data_temp[8] >> 4 | ((uint16_t)crsf_data_temp[9] << 4)) & 0x07FF;
        crsf_data.channels[5] = ((uint16_t)crsf_data_temp[9] >> 7 | ((uint16_t)crsf_data_temp[10] << 1) | ((uint16_t)crsf_data_temp[11] << 9)) & 0x07FF;
        crsf_data.channels[6] = ((uint16_t)crsf_data_temp[11] >> 2 | ((uint16_t)crsf_data_temp[12] << 6)) & 0x07FF;
        crsf_data.channels[7] = ((uint16_t)crsf_data_temp[12] >> 5 | ((uint16_t)crsf_data_temp[13] << 3)) & 0x07FF;
        crsf_data.channels[8] = ((uint16_t)crsf_data_temp[14] >> 0 | ((uint16_t)crsf_data_temp[15] << 8)) & 0x07FF;
        crsf_data.channels[9] = ((uint16_t)crsf_data_temp[15] >> 3 | ((uint16_t)crsf_data_temp[16] << 5)) & 0x07FF;
        crsf_data.channels[10] = ((uint16_t)crsf_data_temp[16] >> 6 | ((uint16_t)crsf_data_temp[17] << 2) | ((uint16_t)crsf_data_temp[18] << 10)) & 0x07FF;
        crsf_data.channels[11] = ((uint16_t)crsf_data_temp[18] >> 1 | ((uint16_t)crsf_data_temp[19] << 7)) & 0x07FF;
        crsf_data.channels[12] = ((uint16_t)crsf_data_temp[19] >> 4 | ((uint16_t)crsf_data_temp[20] << 4)) & 0x07FF;
        crsf_data.channels[13] = ((uint16_t)crsf_data_temp[20] >> 7 | ((uint16_t)crsf_data_temp[21] << 1) | ((uint16_t)crsf_data_temp[22] << 9)) & 0x07FF;
        crsf_data.channels[14] = ((uint16_t)crsf_data_temp[22] >> 2 | ((uint16_t)crsf_data_temp[23] << 6)) & 0x07FF;
        crsf_data.channels[15] = ((uint16_t)crsf_data_temp[23] >> 5 | ((uint16_t)crsf_data_temp[24] << 3)) & 0x07FF;

        crsf_data.Left_X = CRSF_FloatMapWithMedian(crsf_data.channels[3], 174, 1808, 992, -100, 100);
        crsf_data.Left_Y = CRSF_FloatMapWithMedian(crsf_data.channels[2], 174, 1811, 992, 0, 100);
        crsf_data.Right_X = CRSF_FloatMapWithMedian(crsf_data.channels[0], 174, 1811, 992, -100, 100);
        crsf_data.Right_Y = CRSF_FloatMapWithMedian(crsf_data.channels[1], 174, 1808, 992, -100, 100);
        crsf_data.S1 = CRSF_FloatMapWithMedian(crsf_data.channels[8], 191, 1792, 992, 0, 100);
        crsf_data.S2 = CRSF_FloatMapWithMedian(crsf_data.channels[9], 191, 1792, 992, 0, 100);
        
        // 遥控器按键映射关系
        // SA对应B (通道5) - 二档位：下=0，上=1
        // SD对应C (通道7) - 二档位：下=0，上=1
        // SB对应E (通道6) - 三档位：下=0，中=1，上=2
        // SC对应F (通道8) - 三档位：下=0，中=1，上=2
        crsf_data.A = crsf_data.channels[10] > 1000 ? 1 : 0;  // 通道11：左按键A
        crsf_data.B = crsf_data.channels[4] > 1500 ? 1 : 0;  // 通道5：拨杆SA -> B (二档位)
        crsf_data.C = (crsf_data.channels[6] > 800 && crsf_data.channels[6] < 1100) ? 1 : (crsf_data.channels[6] > 1500 ? 2 : 0);  // 通道7：拨杆SD -> C (三档位)
        crsf_data.D = crsf_data.channels[11] > 1000 ? 1 : 0;  // 通道12：右按键D
        crsf_data.E = (crsf_data.channels[5] > 800 && crsf_data.channels[5] < 1100) ? 1 : (crsf_data.channels[5] > 1500 ? 2 : 0);  // 通道6：拨杆SB -> E (三档位)
        crsf_data.F = crsf_data.channels[7] > 1500 ? 1 : 0;  // 通道8：拨杆SC -> F (二档位)
        break;

    case CRSF_FRAMETYPE_LINK_STATISTICS:
        // 链路统计帧：declared_len = 1(帧类型) + 10(数据) + 1(CRC) = 12 = 0x0C
        if (declared_len < LINK_Frame_Length)
        {
            break;
        }
        crsf_data.uplink_RSSI_1 = crsf_data_temp[3];
        crsf_data.uplink_RSSI_2 = crsf_data_temp[4];
        crsf_data.uplink_Link_quality = crsf_data_temp[5];
        crsf_data.uplink_SNR = crsf_data_temp[6];
        crsf_data.active_antenna = crsf_data_temp[7];
        crsf_data.rf_Mode = crsf_data_temp[8];
        crsf_data.uplink_TX_Power = crsf_data_temp[9];
        crsf_data.downlink_RSSI = crsf_data_temp[10];
        crsf_data.downlink_Link_quality = crsf_data_temp[11];
        crsf_data.downlink_SNR = crsf_data_temp[12];
        break;

    case CRSF_FRAMETYPE_HEARTBEAT:
        // 心跳帧：declared_len = 1(帧类型) + 2(数据) + 1(CRC) = 4
        // 心跳计数器是16位大端格式
        if (declared_len >= 4)
        {
            crsf_data.heartbeat_counter = ((uint16_t)crsf_data_temp[3] << 8) | crsf_data_temp[4];
        }
        break;

    default:
        // 未支持的帧类型，不做处理
        break;
    }

    memset(crsf_data_temp, 0, sizeof(crsf_data_temp));
    CRSF_Init();
}
