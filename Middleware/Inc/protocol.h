/**
  * @file    protocol.h
  * @brief   Communication protocol parser (HEADER+LEN+CMD+PAYLOAD+CHECKSUM)
  */

#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

#define PROTO_HEADER      0xAA
#define PROTO_MAX_PAYLOAD 64
#define PROTO_MAX_FRAME   (PROTO_MAX_PAYLOAD + 4)

typedef enum {
    PROTO_CMD_SET_SPEED     = 0x01,
    PROTO_CMD_SET_MOTOR_RAW = 0x02,
    PROTO_CMD_STOP          = 0x03,
    PROTO_CMD_GET_STATUS    = 0x10,
    PROTO_CMD_SET_PID       = 0x20,
    PROTO_CMD_HEARTBEAT     = 0xF0,
} Proto_Cmd_t;

typedef enum {
    PROTO_STATE_IDLE = 0,
    PROTO_STATE_LEN,
    PROTO_STATE_CMD,
    PROTO_STATE_PAYLOAD,
    PROTO_STATE_CHECKSUM
} Proto_State_t;

typedef struct {
    uint8_t cmd;
    uint8_t payload[PROTO_MAX_PAYLOAD];
    uint8_t payload_len;
} Proto_Packet_t;

typedef struct {
    Proto_State_t state;
    uint8_t       rx_buf[PROTO_MAX_FRAME];
    uint8_t       rx_index;
    uint8_t       expected_len;
    uint8_t       payload_len;
} Proto_Parser_t;

typedef void (*Proto_Callback_t)(const Proto_Packet_t *packet);

void Proto_Init(Proto_Parser_t *parser);
void Proto_Feed(Proto_Parser_t *parser, const uint8_t *data, uint16_t len);
void Proto_RegisterCallback(Proto_Callback_t cb);
void Proto_SendPacket(uint8_t cmd, const uint8_t *payload, uint8_t len);

#endif /* PROTOCOL_H */
