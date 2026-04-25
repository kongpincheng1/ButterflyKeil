#ifndef __CRSF_H__
#define __CRSF_H__

#include "main.h"

#define MAX_FRAME_SIZE 36

/*
https://github.com/crsf-wg/crsf/wiki/Packet-Types CRSF协议定义
 */

/*
通道1：右摇杆 x轴
通道2：右摇杆 y轴
通道3：左摇杆 y轴
通道4：左摇杆 x轴
通道5：拨杆F
通道6：拨杆B
通道7：拨杆C
通道8：拨杆F
通道9：左滑块
通道10：右滑块
通道11：左按键A
通道12：右按键B
通道13：
通道14：
通道15：
通道16：
 */

/*
帧格式定义 地址 + 数据长度 + 帧类型 +  数据 + CRC校验码
 */

#define CRSF_FRAMETYPE_GPS 0x02
#define CRSF_FRAMETYPE_BATTERY_SENSOR 0x08
#define CRSF_FRAMETYPE_LINK_STATISTICS 0x14
#define CRSF_FRAMETYPE_OPENTX_SYNC 0x10
#define CRSF_FRAMETYPE_RADIO_ID 0x3A
#define CRSF_FRAMETYPE_RC_CHANNELS_PACKED 0x16
#define CRSF_FRAMETYPE_ATTITUDE 0x1E
#define CRSF_FRAMETYPE_FLIGHT_MODE 0x21
#define CRSF_FRAMETYPE_DEVICE_PING 0x28
#define CRSF_FRAMETYPE_DEVICE_INFO 0x29
#define CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY 0x2B
#define CRSF_FRAMETYPE_PARAMETER_READ 0x2C
#define CRSF_FRAMETYPE_PARAMETER_WRITE 0x2D
#define CRSF_FRAMETYPE_COMMAND 0x32
#define CRSF_FRAMETYPE_MSP_REQ 0x7A
#define CRSF_FRAMETYPE_MSP_RESP 0x7B
#define CRSF_FRAMETYPE_MSP_WRITE 0x7C
#define CRSF_FRAMETYPE_HEARTBEAT 0x0B

#define CRSF_ADDRESS_BROADCAST 0x00
#define CRSF_ADDRESS_USB 0x10
#define CRSF_ADDRESS_TBS_CORE_PNP_PRO 0x80
#define CRSF_ADDRESS_CURRENT_SENSOR 0xC0
#define CRSF_ADDRESS_GPS 0xC2
#define CRSF_ADDRESS_TBS_BLACKBOX 0xC4
#define CRSF_ADDRESS_FLIGHT_CONTROLLER 0xC8
#define CRSF_ADDRESS_RACE_TAG 0xCC
#define CRSF_ADDRESS_RADIO_TRANSMITTER 0xEA
#define CRSF_ADDRESS_CRSF_RECEIVER 0xEC
#define CRSF_ADDRESS_CRSF_TRANSMITTER 0xEE

#define CHANNELS_Frame_Length 0x18
#define LINK_Frame_Length 0x0C

typedef struct
{
    uint16_t channels[16];
    float Left_X;
    float Left_Y;
    float Right_X;
    float Right_Y;
    float S1;
    float S2;
    uint8_t A;
    uint8_t B;
    uint8_t C;
    uint8_t D;
    uint8_t E;
    uint8_t F;

    uint8_t uplink_RSSI_1;
    uint8_t uplink_RSSI_2;
    uint8_t uplink_Link_quality;
    int8_t uplink_SNR;
    uint8_t active_antenna;
    uint8_t rf_Mode;
    uint8_t uplink_TX_Power;
    uint8_t downlink_RSSI;
    uint8_t downlink_Link_quality;
    int8_t downlink_SNR;

    uint16_t heartbeat_counter;

} CRSF_Data;

extern CRSF_Data crsf_data;

void CRSF_Init(void);
void CRSF_RestartRx(void);
void CRSF_UART_RxCallback(uint16_t Size);
uint32_t CRSF_GetLastRxTick(void);

#endif
