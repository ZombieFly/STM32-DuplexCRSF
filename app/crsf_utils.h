#ifndef __CRSF_UTILS_H__
#define __CRSF_UTILS_H__

#include "cmsis_os.h"
#include "string.h"
#include "main.h"


#define __REV24(x) ( ((x) & 0xFF) << 16 | ((x) & 0xFF00) | (((x) >> 16) & 0xFF) )
#define __REV32(x) __REV(x)

// 用以支持24位类型，仅用于REV宏
#define int24_t int32_t

#define REV(value, type) __REV##type((int##type##_t)value)


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

typedef struct __attribute__((packed)) __crsf_gps_t
{
    int32_t latitude;   // Latitude in degrees * 10^7 ( 28.0805804N sent as 0x10BCC1AC / 280805804)
    int32_t longitude;
    uint16_t groundspeed; // km/h * 10 ( 88 km/h sent as 0x0370 / 880; 15m/s sent as 0x021C / 540)
    uint16_t heading; // degrees * 100 (90 degrees sent as 0x2328 / 9000)
    uint16_t altitude; // meters + 1000 (10m sent as 0x03F2 / 1010; -10m sent as 0x03DE / 990)
    uint16_t satellites; // number of satellites
} crsf_gps_t;




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

/** 发送消息包方法 */

// 发送函数指针
extern void (*send_crsf_packet)(const CRSF_FRAMETYPE_t frame_type, const uint8_t * payload, const uint8_t size_of_payload);

/**
 * @brief 发送电池传感器数据
 * @param voltage 电压，V，四舍五入保留一位小数
 * @param current 电流，A，四舍五入保留一位小数
 * @param used_capacity 已用容量，mAh，整数
 * @param estimated_remaining_capacity 估计剩余容量，%，整数
 */
__STATIC_INLINE void crsf_send_BatterySensor(const float voltage, const float current, const uint32_t used_capacity, const uint8_t estimated_remaining_capacity)
{
    crsf_battery_sensor_t battery_data = {0};

    battery_data.voltage = REV(((voltage + .05f) * 10.f), 16);
    battery_data.current = REV(((current + .05f) * 10.f), 16);
    battery_data.used_capacity = REV(used_capacity, 24);
    battery_data.estimated_remaining_capacity = estimated_remaining_capacity;

    send_crsf_packet(CRSF_FRAMETYPE_BATTERY_SENSOR, (uint8_t *)&battery_data, sizeof(battery_data));
}

/**
 * @brief 发送GPS数据
 * @param latitude 纬度，度，七位小数
 * @param longitude 经度，度，七位小数
 * @param altitude 海拔，米，增加1000米后取整
 * @param groundspeed 地面速度，千米每小时，乘以10取整
 * @param heading 方向，度，乘以100取整
 * @param satellites 卫星数量，整数
 */
__STATIC_INLINE void crsf_send_GPS(const float latitude, const float longitude, const uint16_t altitude, const float groundspeed, const float heading, const uint16_t satellites)
{
    crsf_gps_t gps_data = {.latitude = REV((latitude * 1e7f), 32),
                           .longitude = REV((longitude * 1e7f), 32),
                           .altitude = REV(altitude + 1000, 16),
                           .groundspeed = REV((groundspeed * 10), 16),
                           .heading = REV((heading * 100), 16),
                           .satellites = satellites};

    send_crsf_packet(CRSF_FRAMETYPE_GPS, (uint8_t *)&gps_data, sizeof(gps_data));
}


#endif /* __CRSF_UTILS_H__ */
