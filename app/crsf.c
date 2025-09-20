#include "crsf.h"


uint8_t rx_buff[CRSF_MAX_RX_BUFFER_SIZE*2] = {0}; // 接收缓冲区
crsf_boardcast_frame_t crsf_frame = {0};

crsf_channels_t rc_channels;

void crsf_init(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&CRSF_UART, rx_buff, CRSF_MAX_RX_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(&CRSF_RX_DMA, DMA_IT_HT); // 禁止DMA半传输中断
}

__STATIC_INLINE uint8_t crsf_calculate_crc(const uint8_t * ptr, const uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i=0; i<len; i++)
        crc = crc8tab[crc ^ *ptr++];
    return crc;
}

__STATIC_INLINE uint8_t crsf_rx_msg_check(const uint8_t * msg, const uint8_t len)
{
    if (len < 4 || len > 64)
        return 0;
    if (msg[1] != len - 2)
        return 0;
    if (crsf_calculate_crc(msg + 2, len - 3) != msg[len - 1])
        return 0;
    return 1;
}

void crsf_rx_idle_callback(const uint16_t size)
{
    
    crsf_init();

    if(!crsf_rx_msg_check(rx_buff, size))
    {
        return;
    }

    crsf_frame.addr = rx_buff[0];
    crsf_frame.length = rx_buff[1];
    crsf_frame.type = rx_buff[2];
    memcpy(crsf_frame.payload, &rx_buff[3], crsf_frame.length - 2);
    crsf_frame.crc = rx_buff[size - 1];

    switch (crsf_frame.addr)
    {
    case CRSF_ADDRESS_FLIGHT_CONTROLLER:
        switch (crsf_frame.type)
        {
        case CRSF_FRAMETYPE_RC_CHANNELS_PACKED:
            // 16ch 遥控
            crsf_unpack_flight_controller(&crsf_frame);
            break;
        
        default:
            break;
        }
        break;
    
    default:
        break;
    }
    

}


uint8_t send_buffer[64] = {CRSF_ADDRESS_FLIGHT_CONTROLLER};

void send_crsf_packet(const CRSF_FRAMETYPE_t frame_type, const uint8_t * payload, const uint8_t size_of_payload)
{

    send_buffer[1] = size_of_payload + 2;
    send_buffer[2] = frame_type;
    memcpy(&send_buffer[3], payload, size_of_payload);
    send_buffer[size_of_payload + (4 - 1)] = crsf_calculate_crc(&send_buffer[2], size_of_payload + 1);
    HAL_UART_Transmit_DMA(&CRSF_UART, send_buffer, size_of_payload + 4);
}



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
        osDelay(1000);
    }
}
