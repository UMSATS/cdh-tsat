#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h> // For memcpy

#include "main.h"

extern UART_HandleTypeDef huart2;

#define AX25_HAL_OK 0
#define AX25_HAL_ERROR 1

typedef int AX25_HALStatusTypedef;

#define NUM_TESTS 3

#define MAX_CONSECUTIVE_SYMBOLS 5

#define UINT8_WIDTH 8

const unsigned char src_callsign[6] = "VE4NJR";
const unsigned char dest_callsign[6] = "NOCALL";

// TODO: Figure out these. -NJR
const uint8_t src_ssid = 7;
const uint8_t dest_ssid = 7;

#define CALLSIGN_LEN 6

#define TRANSFER_FRAME_HEADER_LEN 16
#define FLAG_LEN 1
#define FCS_LEN 2 // CRC Length

#define MIN_PACKET_SIZE ((2 * FLAG_LEN) + TRANSFER_FRAME_HEADER_LEN + FCS_LEN)

#define AX25_FLAG_SEQUENCE 0x7E
#define AX25_CONTROL_BITS 0x03 // TODO: Right? -NJR
#define AX25_PROTOCOL_ID 0xF0

#define DEST_CALL_POSITION 0
#define SRC_CALL_POSITION DEST_CALL_POSITION + 7
#define CONTROL_BITS_POSITION SRC_CALL_POSITION + 7
#define PROTOCALL_ID_POSITION CONTROL_BITS_POSITION + 1
#define INFO_FIELD_POSITION PROTOCALL_ID_POSITION + 1

/**
 * @brief Sets the n'th bit starting from the LSB
 * 
 */
AX25_HALStatusTypedef AX25_Bitwise_Append(uint8_t *dest, size_t bit_position, bool is_one)
{
    AX25_HALStatusTypedef operation_status = AX25_HAL_OK;

    if (!dest) {
        operation_status = AX25_HAL_ERROR;
        goto error;
    }

    if (bit_position >= 8) {
        operation_status = AX25_HAL_ERROR;
        goto error;
    }

    if (is_one) {
        *dest |= (1 << bit_position);
    } else {
        *dest &= ~(1 << bit_position);
    }

error:
    return operation_status;
}

AX25_HALStatusTypedef AX25_Add_Bits_To_Array(uint8_t dest[], size_t max_size, uint8_t to_add, size_t byte_at, size_t bit_at)
{
	AX25_HALStatusTypedef operation_status = AX25_HAL_OK;
	if (!dest || max_size < byte_at || bit_at > 8) {
		operation_status = AX25_HAL_ERROR;
		goto error;
	}

	for (size_t i = bit_at; i < 8; i++) {
		AX25_Bitwise_Append(dest + byte_at, i, (to_add & (1 << i - bit_at)));
	}

	for (size_t i = 0; i < bit_at; i++) {
		AX25_Bitwise_Append(dest + byte_at + 1, i, (to_add & (1 << (i + (8 - bit_at)))));
	}

error:
	return operation_status;
}

AX25_HALStatusTypedef AX25_Bitstuff_Array(uint8_t input_array[], size_t input_size, uint8_t output_array[], size_t output_size, size_t *output_result_len, size_t *output_result_extra_bits)
{
    AX25_HALStatusTypedef operation_status = AX25_HAL_OK;
    bool output_array_overrun = false;

    size_t current_output_byte = 0;
    size_t current_output_bit = 0;

    size_t num_consecutive_ones = 0;

    if (!input_array || !output_array || !output_result_len || !output_result_extra_bits) {
        operation_status = AX25_HAL_ERROR;
        goto error;
    }

    if (input_size == 0 || output_size == 0 || output_size <= input_size) {
        operation_status = AX25_HAL_ERROR;
        goto error;
    }

    // iterate over each bit in each byte of input_array. If we see MAX_CONSECUTIVE_SYMBOLS of 1's, 
    // we add a zero to the next bit. -NJR
    size_t current_input_byte = 0;
    size_t current_input_bit = 0;

    while (current_input_byte < input_size && !output_array_overrun) {
        if (num_consecutive_ones == MAX_CONSECUTIVE_SYMBOLS) {
            AX25_Bitwise_Append(output_array + current_output_byte, current_output_bit, false);

            num_consecutive_ones = 0;
            current_output_bit++;
        } else {
            AX25_Bitwise_Append(output_array + current_output_byte, current_output_bit, ((input_array[current_input_byte]) & (1 << current_input_bit)));

            if (input_array[current_input_byte] & (1 << current_input_bit)) {
                num_consecutive_ones++;
            } else {
            	num_consecutive_ones = 0;
            }

            current_input_bit++;
            current_output_bit++;

            if (current_input_bit >= UINT8_WIDTH) {
                current_input_bit = 0;
                current_input_byte++;
            }
        }

        if (current_output_bit >= UINT8_WIDTH) {
            current_output_bit = 0;
            current_output_byte++;
        }

        if (current_output_byte == output_size) {
            output_array_overrun = true;
        }
    }

    if (output_array_overrun) {
        operation_status = AX25_HAL_ERROR;
    } else {
        // Special case for if the array ends with 5 bits.
        if (num_consecutive_ones == MAX_CONSECUTIVE_SYMBOLS) {
            AX25_Bitwise_Append(output_array + current_output_byte, current_output_bit, false);

            current_output_bit ++;
            if (current_output_bit >= UINT8_WIDTH) {
                current_output_bit = 0;
                current_output_byte++;
            }
        }

        *output_result_len = current_output_byte;
        *output_result_extra_bits = current_output_bit;
    }

error:
    return operation_status;
}


// TODO: We can use the CRC Peripheral for this. -NJR
AX25_HALStatusTypedef AX25_Form_Packet(uint8_t scratch_space[], size_t scratch_space_max_len, uint8_t data_to_send[], size_t data_len, uint8_t out_array[], size_t out_array_max_len, size_t *out_len)
{
    AX25_HALStatusTypedef operation_status = AX25_HAL_OK;

    size_t scratch_space_used = MIN_PACKET_SIZE - (2 * FLAG_LEN);

    size_t final_len = 0;
    size_t extra_bits = 0;

    if (!scratch_space || !out_array || !data_to_send || !out_len) {
        operation_status = AX25_HAL_ERROR;
        goto error;
    }

    // TODO: THIS IS NOT VERY SAFE!
    // Out_array holds post-bitstuffed data. This will likely be longer than the scratch
    // space and will cause bitstuffing to fail (albeit gracefully). -NJR
    if (scratch_space_max_len < MIN_PACKET_SIZE || out_array_max_len < MIN_PACKET_SIZE) {
        operation_status = AX25_HAL_ERROR;
        goto error;
    }

    if (data_len > scratch_space_max_len - MIN_PACKET_SIZE) {
        operation_status = AX25_HAL_ERROR;
        goto error;
    }

    for (size_t i = 0; i < 6; i++) {
        // Callsigns are shifted left by one bit. the lowest bit position is always 0.
        scratch_space[i + DEST_CALL_POSITION] = (dest_callsign[i] << 1);
    }
    // The ssid is the lowest 4 bits (0 - 15) and is shifted left one.
    scratch_space[SRC_CALL_POSITION - 1] = (dest_ssid & 0x0F) << 1;

    for (size_t i = 0; i < 6; i++) {
        scratch_space[i + SRC_CALL_POSITION] = (src_callsign[i] << 1);
    }
	HAL_UART_Transmit(&huart2, scratch_space, INFO_FIELD_POSITION + data_len, HAL_MAX_DELAY);
    scratch_space[CONTROL_BITS_POSITION - 1] = (src_ssid & 0x0F) << 1;

	HAL_UART_Transmit(&huart2, scratch_space, INFO_FIELD_POSITION + data_len, HAL_MAX_DELAY);
    scratch_space[CONTROL_BITS_POSITION] = AX25_CONTROL_BITS;
    scratch_space[PROTOCALL_ID_POSITION] = AX25_PROTOCOL_ID;

    memcpy(scratch_space + INFO_FIELD_POSITION, data_to_send, data_len);


    // TODO: Use the CRC Module to figure out the CRC. _What_ CRC, I do not currently know. -NJR
    scratch_space[INFO_FIELD_POSITION + data_len + 0] = 0xFF;
    scratch_space[INFO_FIELD_POSITION + data_len + 1] = 0xFF;

    scratch_space_used += data_len;

    out_array[0] = AX25_FLAG_SEQUENCE;

    // TODO: Form bitstuffed packet here. -NJR
    // Plus one for the flag there.
    AX25_Bitstuff_Array(scratch_space, scratch_space_used, out_array + 1, out_array_max_len - 1, &final_len, &extra_bits);

    // Final flag sequence.
    AX25_Add_Bits_To_Array(out_array, out_array_max_len, AX25_FLAG_SEQUENCE, final_len, extra_bits);

    final_len += 1;

    // Just send the extra bits! nothing else we can do here.
    if (extra_bits > 0) {
    	*out_len = final_len + 1;
    } else {
    	*out_len = final_len;
    }

error:
    return operation_status;
}
