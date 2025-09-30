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
    float voltage;

    while (1)
    {
        voltage = get_vbat_voltage();

        crsf_send_BatterySensor(voltage, 1145, ((voltage + .00005f) * 100000.f), 99);
        osDelay(500);

        crsf_send_GPS(28.0805804f, -80.1234567f, 0, 88.8f, 90.9f, 8);
        osDelay(500);
    }
}
