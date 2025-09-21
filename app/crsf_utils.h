#ifndef __CRSF_UTILS_H__
#define __CRSF_UTILS_H__

#include "cmsis_os.h"
#include "string.h"
#include "main.h"


#define __REV24(x) ( ((x) & 0xFF) << 16 | ((x) & 0xFF00) | (((x) >> 16) & 0xFF) )

typedef struct __crsf_boardcast_frame_t
{
    uint8_t addr;
    uint8_t length;
    uint8_t type;
    uint8_t payload[60];
    uint8_t crc;
} crsf_boardcast_frame_t;


typedef struct __attribute__((packed)) __crsf_channels_t
{
    // 打包状态下，一个通道实际占用11位
    uint16_t ch[16];
    uint8_t armStatus; // optional ExpressLRS 4.0
} crsf_channels_t;

typedef struct __attribute__((packed)) __crsf_battery_sensor_t
{
    signed voltage : 16;      // 25.2V sent as 0x00FC / 252
    signed current : 16;      // 18.9A sent as 0x00BD / 189
    unsigned int used_capacity : 24;     // 2199mAh used sent as 0x0897 / 2199
    uint8_t estimated_remaining_capacity; // 20% battery remaining sent as 0x14 / 20
} crsf_battery_sensor_t;




/** CRSF地址定义 */
typedef enum __CRSF_ADDRESS_t{
    CRSF_ADDRESS_BROADCAST                   = 0x00,  // 广播
    CRSF_ADDRESS_USB                         = 0x10,
    CRSF_ADDRESS_BLUETOOTH                   = 0x12,  // 蓝牙
    CRSF_ADDRESS_TBS_CORE_PNP_PRO            = 0x80,
    CRSF_ADDRESS_RESERVED1                   = 0x8A,  // 保留地址1
    CRSF_ADDRESS_CURRENT_SENSOR              = 0xC0,  // 电流传感器
    CRSF_ADDRESS_GPS                         = 0xC2,  // GPS
    CRSF_ADDRESS_TBS_BLACKBOX                = 0xC4,  // TBS黑匣子
    CRSF_ADDRESS_FLIGHT_CONTROLLER           = 0xC8,  // 飞行控制器
    CRSF_ADDRESS_RESERVED2                   = 0xCA,  // 保留地址2
    CRSF_ADDRESS_RACE_TAG                    = 0xCC,  // 比赛标签
    CRSF_ADDRESS_RADIO_TRANSMITTER           = 0xEA,  // 无线电发射器
    CRSF_ADDRESS_CRSF_RECEIVER               = 0xEC,  // Receiver hardware (TBS Nano RX / RadioMaster RP1)
    CRSF_ADDRESS_CRSF_TRANSMITTER            = 0xEE,  // Transmitter module, not handset
    CRSF_ADDRESS_ELRS_LUA                    = 0xEF,  // elrs lua
} CRSF_ADDRESS_t;

/** CRSF Type定义 */
typedef enum __CRSF_FRAMETYPE_t{
    CRSF_FRAMETYPE_GPS                       = 0x02,  // 回传
    CRSF_FRAMETYPE_VARIO                     = 0x07,
    CRSF_FRAMETYPE_BATTERY_SENSOR            = 0x08,  // 回传
    CRSF_FRAMETYPE_BARO_ALTITUDE             = 0x09,  // 回传
    CRSF_FRAMETYPE_HEARTBEAT                 = 0x0B,
    CRSF_FRAMETYPE_LINK_STATISTICS           = 0x14,
    CRSF_FRAMETYPE_RC_CHANNELS_PACKED        = 0x16,
    CRSF_FRAMETYPE_SUBSET_RC_CHANNELS_PACKED = 0x17,
    CRSF_FRAMETYPE_LINK_RX_ID                = 0x1C,   // 回传
    CRSF_FRAMETYPE_LINK_TX_ID                = 0x1D,   // 回传
    CRSF_FRAMETYPE_ATTITUDE                  = 0x1E,   // 回传
    CRSF_FRAMETYPE_FLIGHT_MODE               = 0x21,   // 回传
} CRSF_FRAMETYPE_t;

// 后面还有拓展类型待补充


/** 解析结构变量 */

extern crsf_channels_t rc_channels;

/** 消息包解析方法 */

// 解包FLIGHT_CONTROLLER
__STATIC_INLINE uint8_t crsf_unpack_flight_controller(const crsf_boardcast_frame_t * frame){
    if (frame->type == CRSF_FRAMETYPE_RC_CHANNELS_PACKED)
    {
        const unsigned numOfChannels = 16;
        const unsigned srcBits = 11;
        const unsigned inputChannelMask = (1 << srcBits) - 1;

        // code from BetaFlight rx/crsf.cpp / bitpacker_unpack
        uint8_t bitsMerged = 0;
        uint32_t readValue = 0;
        unsigned readByteIndex = 0;
        for (uint8_t n = 0; n < numOfChannels; n++)
        {
            while (bitsMerged < srcBits)
            {
                uint8_t readByte = frame->payload[readByteIndex++];
                readValue |= ((uint32_t) readByte) << bitsMerged;
                bitsMerged += 8;
            }
            rc_channels.ch[n] = (readValue & inputChannelMask);
            readValue >>= srcBits;
            bitsMerged -= srcBits;
        }

        return 1;
    }

    return 0;
};

#endif /* __CRSF_UTILS_H__ */
