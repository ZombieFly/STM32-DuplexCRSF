#include "tele_task.h"


float get_vbat_voltage(void)
{
    uint16_t adcx = 0;
    // 乘数算子
    // 3.3V / 4096
    //(10K Ω + 100K Ω)  / 10K Ω = 11
    const float multiplier = (8.0586080586080586080586080586081e-4f * 11.0f);
    float voltage;

    adcx = (uint16_t)HAL_ADC_GetValue(&hadc1);
    
    voltage =  (float)adcx * multiplier;

    HAL_ADC_Start(&hadc1);

    return voltage;
}

void tele_task(void *argument)
{
    crsf_battery_sensor_t battery_data = {0};
    crsf_gps_t gps_data = {0};

    float voltage;

    // 模拟电池数据
    battery_data.current = REV(1145, 16);
    battery_data.estimated_remaining_capacity = 100;

    while (1)
    {
        voltage = get_vbat_voltage();
        battery_data.voltage = REV(((voltage + .05f) * 10.f), 16);
        battery_data.used_capacity = REV(((voltage + .00005f) * 100000.f), 24); // 10000倍电压，用以回传更精细电压数据

        send_crsf_packet(CRSF_FRAMETYPE_BATTERY_SENSOR, (uint8_t *)&battery_data, sizeof(battery_data));
        osDelay(500);

        // 模拟GPS数据
        gps_data.latitude = REV(280805804, 32);
        gps_data.longitude = REV(-80000000, 32);
        gps_data.altitude = REV(1000, 16);
        gps_data.groundspeed = REV(880, 16);
        gps_data.heading = REV(9000, 16);
        gps_data.satellites = 8;

        send_crsf_packet(CRSF_FRAMETYPE_GPS, (uint8_t *)&gps_data, sizeof(gps_data));
        osDelay(500);
    }
}
