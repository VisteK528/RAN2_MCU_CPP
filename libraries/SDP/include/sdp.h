#ifndef RUDP_SDP_H
#define RUDP_SDP_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include "usart.h"
#include <stdlib.h>
#include <string.h>

typedef uint16_t crc;
#define WIDTH  (8 * sizeof(crc))
#define CRC_BYTES 2

// Start and stop bytes are defined by an ASCII SOH (Start of Header) and EOT (End of transmission) values
#define START_BYTE 0x01
#define END_BYTE 0x04

#define DATA_BYTES 7
#define MAX_FRAME_BYTES 13

#define SINGLE_FRAME 0x00
#define FIRST_FRAME 0x01
#define CONSECUTIVE_FRAME 0x02

#define MAX_DATA_LENGTH 14336       // (bytes)


typedef struct{
    uint8_t pData[MAX_DATA_LENGTH];
    uint16_t dataLength;
    HAL_StatusTypeDef transmission_status;
} SDP_Message;

typedef struct {
    uint8_t pData[DATA_BYTES];
    uint16_t dataLength;
    HAL_StatusTypeDef transmission_status;
    uint8_t mode;
    uint16_t sequence_frames;
} SDP_Frame_Data;

uint8_t encode_n_bytes(const uint8_t* bytes, uint16_t start_index, uint8_t* encoded_bytes, uint8_t n_bytes);
uint8_t decode_n_bytes(uint8_t* bytes, uint16_t start_index, uint8_t* encoded_bytes, uint8_t n_bytes);
void crcFast(const uint8_t* message, crc* remainder, uint8_t n_bytes);

HAL_StatusTypeDef send_single_frame(const uint8_t* message, uint8_t nbytes);
HAL_StatusTypeDef send_first_frame(const uint8_t* data, uint16_t data_length, uint8_t nbytes);
HAL_StatusTypeDef send_consecutive_frame(const uint8_t* data, uint16_t sequence_number, uint8_t nbytes);
HAL_StatusTypeDef send_frame(const uint8_t* data, uint16_t sequence_number, uint16_t control_bytes, uint8_t nbytes);

HAL_StatusTypeDef receive_single_frame(uint8_t* message);
HAL_StatusTypeDef receive_frame(SDP_Frame_Data* frame_data, uint32_t timeout);

HAL_StatusTypeDef SDP_Transmit(uint8_t* pData, uint16_t nbytes);
HAL_StatusTypeDef SDP_Receive(SDP_Message* message, uint32_t timeout);


#ifdef __cplusplus
}
#endif

#endif //RUDP_SDP_H
