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
