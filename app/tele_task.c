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
    battery_data.current = __REV16(1145);
    battery_data.estimated_remaining_capacity = 100;

    while (1)
    {
        voltage = get_vbat_voltage();
        battery_data.voltage = __REV16((int16_t)((voltage + .05f) * 10.f));
        battery_data.used_capacity = __REV24((uint32_t)((voltage + .00005f) * 100000.f)); // 10000倍电压，用以回传更精细电压数据

        send_crsf_packet(CRSF_FRAMETYPE_BATTERY_SENSOR, (uint8_t *)&battery_data, sizeof(battery_data));
        osDelay(500);

        // 模拟GPS数据
        gps_data.latitude = __REV32(280805804);
        gps_data.longitude = __REV32(80000000);
        gps_data.altitude = __REV16(1000);
        gps_data.groundspeed = __REV16(880);
        gps_data.heading = __REV16(9000);
        gps_data.satellites = 8;

        send_crsf_packet(CRSF_FRAMETYPE_GPS, (uint8_t *)&gps_data, sizeof(gps_data));
        osDelay(500);
    }
}
