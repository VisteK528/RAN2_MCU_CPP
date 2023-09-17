#include "../include/sdp.h"

static uint16_t crcTable[256] = {
        0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7, 0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD,
        0xE1CE, 0xF1EF, 0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6, 0x9339, 0x8318, 0xB37B, 0xA35A,
        0xD3BD, 0xC39C, 0xF3FF, 0xE3DE, 0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485, 0xA56A, 0xB54B,
        0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D, 0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
        0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC, 0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861,
        0x2802, 0x3823, 0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B, 0x5AF5, 0x4AD4, 0x7AB7, 0x6A96,
        0x1A71, 0x0A50, 0x3A33, 0x2A12, 0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A, 0x6CA6, 0x7C87,
        0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41, 0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
        0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70, 0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A,
        0x9F59, 0x8F78, 0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F, 0x1080, 0x00A1, 0x30C2, 0x20E3,
        0x5004, 0x4025, 0x7046, 0x6067, 0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E, 0x02B1, 0x1290,
        0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256, 0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
        0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405, 0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E,
        0xC71D, 0xD73C, 0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634, 0xD94C, 0xC96D, 0xF90E, 0xE92F,
        0x99C8, 0x89E9, 0xB98A, 0xA9AB, 0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3, 0xCB7D, 0xDB5C,
        0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A, 0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
        0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9, 0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83,
        0x1CE0, 0x0CC1, 0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8, 0x6E17, 0x7E36, 0x4E55, 0x5E74,
        0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
};

static uint16_t generateControlBytes(uint16_t frame_id, uint16_t sequence_number, uint16_t DLC){
    uint16_t control_bytes = 0x00;

    control_bytes = control_bytes | (frame_id << 14);
    control_bytes = control_bytes | (sequence_number << 3);
    control_bytes = control_bytes | DLC;
    return control_bytes;
}

static void copy_uint8_t(const uint8_t* src, uint8_t* dst, uint16_t bytes, uint16_t start_index){
    for(uint16_t i = 0; i < bytes; i++){
        dst[i+start_index] = src[i];
    }
}


uint8_t encode_n_bytes(const uint8_t* bytes, uint16_t start_index, uint8_t * encoded_bytes, uint8_t n_bytes){
    crc remainder = 0;

    for(uint8_t i = 0; i < n_bytes; i++){
        encoded_bytes[i] = bytes[i+start_index];
    }

    crcFast(encoded_bytes, &remainder, n_bytes);
    encoded_bytes[n_bytes] = remainder >> 8;
    encoded_bytes[n_bytes+1] = remainder & 0xff;
    return 0;
}

uint8_t decode_n_bytes(uint8_t* bytes, uint16_t start_index, uint8_t * encoded_bytes, uint8_t n_bytes){
    crc remainder = 0;
    crcFast(encoded_bytes, &remainder, n_bytes+CRC_BYTES);

    if(remainder != 0){
        return 1;
    }

    for(uint8_t i = 0; i < n_bytes; i++){
        bytes[i+start_index] = encoded_bytes[i];
    }
    return 0;
}

void crcFast(const uint8_t* message, crc* remainder, uint8_t n_bytes)
{
    uint8_t data;
    /*
     * Divide the message by the polynomial, a byte at a time.
     */
    for (int byte = 0; byte < n_bytes; ++byte)
    {
        data = message[byte] ^ (*remainder >> (WIDTH - 8));
        *remainder = crcTable[data] ^ (*remainder << 8);
    }
}

HAL_StatusTypeDef send_frame(const uint8_t* data, uint16_t sequence_number, uint16_t control_bytes, uint8_t nbytes){
    HAL_StatusTypeDef status;

    const uint8_t encoded_data_length = nbytes+CRC_BYTES;
    const uint8_t data_array_length = 4+encoded_data_length;
    uint8_t* encoded_data = malloc(encoded_data_length*sizeof(uint8_t));
    uint8_t* data_array = malloc(data_array_length*sizeof(uint8_t));

    data_array[0] = START_BYTE;
    data_array[2] = control_bytes;
    data_array[1] = control_bytes >> 8;
    data_array[3+nbytes+CRC_BYTES] = END_BYTE;

    encode_n_bytes(data, sequence_number*DATA_BYTES, encoded_data, nbytes);


    for(uint8_t i = 0; i < nbytes + CRC_BYTES; i++){
        data_array[i+3] = encoded_data[i];
    }

    status = HAL_UART_Transmit(&huart2, data_array, data_array_length, 100);
    free(encoded_data);
    free(data_array);
    if(status == HAL_ERROR){
        return status;
    }
    return HAL_OK;
}

HAL_StatusTypeDef send_single_frame(const uint8_t* data, uint8_t nbytes){
    uint16_t control_bytes = 0;
    control_bytes = generateControlBytes(SINGLE_FRAME, 0, nbytes);

    return send_frame(data, 0, control_bytes, nbytes);
}

HAL_StatusTypeDef send_first_frame(const uint8_t* data, uint16_t data_length, uint8_t nbytes){
    uint16_t control_bytes = 0;
    control_bytes = generateControlBytes(FIRST_FRAME, data_length, nbytes);

    return send_frame(data, 0, control_bytes, nbytes);
}

HAL_StatusTypeDef send_consecutive_frame(const uint8_t* data, uint16_t sequence_number, uint8_t nbytes){
    uint16_t control_bytes = 0;
    control_bytes = generateControlBytes(CONSECUTIVE_FRAME, sequence_number, nbytes);

    return send_frame(data, sequence_number, control_bytes, nbytes);
}

HAL_StatusTypeDef SDP_Transmit(uint8_t* pData, uint16_t nbytes){
    if(nbytes == 0){
        return HAL_OK;
    }
    else if(nbytes > 0 && nbytes < 8){
        send_single_frame(pData, nbytes);
        return HAL_OK;
    }
    else if(nbytes > 7 && nbytes <= 14336){
        uint16_t sequence_number = 1;
        uint16_t frames = (uint16_t)ceilf((float)nbytes / (float)DATA_BYTES);
        uint16_t data_size = 0;
        send_first_frame(pData, frames, DATA_BYTES);
        frames--;
        nbytes -= DATA_BYTES;

        while(frames > 0){
            if(nbytes > 6){
                data_size = DATA_BYTES;
            }
            else{
                data_size = nbytes % DATA_BYTES;
            }

            send_consecutive_frame(pData, sequence_number, data_size);
            sequence_number++;
            frames--;
            nbytes -= DATA_BYTES;
        }
        return HAL_OK;
    }
    else{
        return HAL_ERROR;
    }
}

HAL_StatusTypeDef receive_frame(SDP_Frame_Data* frame_data, uint32_t timeout){
    volatile HAL_StatusTypeDef status;
    uint8_t frameStart = 0;

    uint8_t pStart[2];
    uint16_t nbytes = 2;
    crc remainder = 0;

    uint32_t time_start = HAL_GetTick();
    uint32_t time = HAL_GetTick();
    while(time < time_start + timeout){
        HAL_UART_Receive(&huart2, &frameStart, 1, 0);

        if(frameStart == START_BYTE){
            break;
        }

        time = HAL_GetTick();
    }

    if(frameStart != START_BYTE){
        return HAL_TIMEOUT;
    }

    HAL_UART_Receive(&huart2, pStart, nbytes, timeout);

    frame_data->mode = pStart[0] >> 6;
    uint8_t frame_data_bytes = pStart[1] & 0x07;
    frame_data->sequence_frames =  ((pStart[0] & 0x3F) << 5 ) | ((pStart[1] & 0xF8) >> 3);

    nbytes = frame_data_bytes + CRC_BYTES + 1;
    uint8_t* pFrame = malloc(nbytes * sizeof(uint8_t));
    uint8_t* pDataBuffer = malloc((nbytes - 1) * sizeof(uint8_t));
    HAL_UART_Receive(&huart2, pFrame, nbytes, timeout);
    copy_uint8_t(pFrame, pDataBuffer, nbytes-1, 0);
    crcFast(pDataBuffer, &remainder, nbytes-1);

    //check remainder

    copy_uint8_t(pDataBuffer, frame_data->pData, frame_data_bytes, 0);
    frame_data->dataLength = frame_data_bytes;
    free(pDataBuffer);
    return HAL_OK;
}

HAL_StatusTypeDef SDP_Receive(SDP_Message* message, uint32_t timeout){
    SDP_Frame_Data frame_data;
    HAL_StatusTypeDef status;
    uint16_t frames_received = 0;
    message->dataLength = 0;
    status = receive_frame(&frame_data, timeout);

    if(status == HAL_TIMEOUT){
        return HAL_TIMEOUT;
    }

    if(frame_data.mode == SINGLE_FRAME){
        message->dataLength = frame_data.dataLength;
        message->transmission_status = frame_data.transmission_status;
        copy_uint8_t(frame_data.pData, message->pData, frame_data.dataLength, 0);
    }
    else if(frame_data.mode == FIRST_FRAME){
        uint16_t framesLeft = frame_data.sequence_frames-1;
        message->dataLength += frame_data.dataLength;
        copy_uint8_t(frame_data.pData, message->pData, frame_data.dataLength, 0);
        frames_received++;

        while(framesLeft > 0){
            status = receive_frame(&frame_data, timeout);
            message->dataLength += frame_data.dataLength;
            if(status == HAL_ERROR){
                return status;
            }
            copy_uint8_t(frame_data.pData, message->pData, frame_data.dataLength, DATA_BYTES*frames_received);
            framesLeft--;
            frames_received++;
        }
        return HAL_OK;
    }
    else{
        return HAL_ERROR;
    }
    return HAL_OK;
}