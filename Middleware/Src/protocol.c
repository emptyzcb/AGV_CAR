/**
  * @file    protocol.c
  * @brief   Protocol parser state machine
  */

#include "protocol.h"
#include "bsp_uart.h"

static Proto_Callback_t user_cb = NULL;

void Proto_Init(Proto_Parser_t *parser)
{
    parser->state        = PROTO_STATE_IDLE;
    parser->rx_index     = 0;
    parser->expected_len = 0;
    parser->payload_len  = 0;
}

void Proto_RegisterCallback(Proto_Callback_t cb)
{
    user_cb = cb;
}

static uint8_t calc_checksum(const uint8_t *buf, uint8_t len)
{
    uint8_t xor_val = 0;
    for (uint8_t i = 0; i < len; i++)
        xor_val ^= buf[i];
    return xor_val;
}

void Proto_Feed(Proto_Parser_t *parser, const uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++)
    {
        uint8_t byte = data[i];

        switch (parser->state)
        {
        case PROTO_STATE_IDLE:
            if (byte == PROTO_HEADER)
            {
                parser->rx_buf[0] = byte;
                parser->rx_index  = 1;
                parser->state     = PROTO_STATE_LEN;
            }
            break;

        case PROTO_STATE_LEN:
            if (byte < 4 || byte > PROTO_MAX_FRAME)
            {
                parser->state = PROTO_STATE_IDLE;
                break;
            }
            parser->rx_buf[1]    = byte;
            parser->expected_len = byte;
            parser->payload_len  = byte - 4;
            parser->rx_index     = 2;
            parser->state        = PROTO_STATE_CMD;
            break;

        case PROTO_STATE_CMD:
            parser->rx_buf[2] = byte;
            parser->rx_index  = 3;
            parser->state = (parser->payload_len > 0) ? PROTO_STATE_PAYLOAD : PROTO_STATE_CHECKSUM;
            break;

        case PROTO_STATE_PAYLOAD:
            parser->rx_buf[parser->rx_index++] = byte;
            if (parser->rx_index >= 3 + parser->payload_len)
                parser->state = PROTO_STATE_CHECKSUM;
            break;

        case PROTO_STATE_CHECKSUM:
        {
            parser->rx_buf[parser->rx_index] = byte;
            uint8_t calc = calc_checksum(parser->rx_buf, parser->expected_len - 1);
            if (calc == byte && user_cb)
            {
                Proto_Packet_t pkt;
                pkt.cmd         = parser->rx_buf[2];
                pkt.payload_len = parser->payload_len;
                for (uint8_t j = 0; j < parser->payload_len; j++)
                    pkt.payload[j] = parser->rx_buf[3 + j];
                user_cb(&pkt);
            }
            parser->state = PROTO_STATE_IDLE;
            break;
        }

        default:
            parser->state = PROTO_STATE_IDLE;
            break;
        }
    }
}

void Proto_SendPacket(uint8_t cmd, const uint8_t *payload, uint8_t len)
{
    uint8_t frame[PROTO_MAX_FRAME];
    uint8_t total = 4 + len;

    frame[0] = PROTO_HEADER;
    frame[1] = total;
    frame[2] = cmd;
    for (uint8_t i = 0; i < len; i++)
        frame[3 + i] = payload[i];
    frame[3 + len] = calc_checksum(frame, 3 + len);

    BSP_UART_Send(frame, total);
}
