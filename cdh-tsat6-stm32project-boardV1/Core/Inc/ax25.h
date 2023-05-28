/*
 * FILENAME: ax25.h
 *
 * DESCRIPTION: Functions dealing with the Transmission / Reception of AX.25 Telemetry.
 *
 * AUTHORS:
 *  - Nikolaus J. Reichert <nikolaus.reichert@umsats.ca>
 *
 */

typedef enum {
	AX25_HAL_OK     	= HAL_OK,
	AX25_HAL_ERROR  	= HAL_ERROR,
	AX25_HAL_BUSY   	= HAL_BUSY,
	AX25_HAL_TIMEOUT 	= HAL_TIMEOUT
} AX25_StatusTypeDef;

/**
 * @brief Sets the nth bit in a given byte, starting from the LSB.
 * 
 * @param dest The byte to modify.
 * @param bit_position The bit to modify.
 * @param is_one whether the bit should be set or reset.
 * 
 * @returns AX25_HAL_OK on success
 * @returns AX25_HAL_ERROR if dest is NULL or 0 <= bit_position < 8.
 */
AX25_StatusTypeDef AX25_Bitwise_Append(uint8_t *dest, size_t bit_position, bool is_one);

/**
 * @brief 
 * 
 * @param input_array 
 * @param input_size 
 * @param output_array 
 * @param output_size 
 * @param output_result_len 
 * @param output_result_extra_bits 
 * 
 * @returns AX25_HAL_OK on success.
 * @returns AX25_HAL_ERROR if any array is NULL or zero size, or if output_array is not large enough to fit the final packet.
 */
AX25_StatusTypeDef AX25_Bitstuff_Array(uint8_t input_array[], size_t input_size, uint8_t output_array[], size_t output_size, size_t *output_result_len, size_t *output_result_extra_bits);

/**
 * @brief Attempts to form an AX.25 Packet.
 * 
 * @param scratch_space Space to place the pre-bitstuffed packet.
 * @param scratch_space_max_len The size of the scratch space
 * @param data_to_send The data to place in the packet's Information Field.
 * @param data_len The length of the above data.
 * @param out_array The final bitstuffed packet.
 * @param out_array_max_len The maximum size of the final bitstuffed packet.
 * 
 * @returns AX25_HAL_OK on success
 * @returns AX25_HAL_ERROR if any array is NULL or not large enough, or if the bitstuffing fails.
 */
AX25_StatusTypeDef AX25_Form_Packet(uint8_t scratch_space[], size_t scratch_space_max_len, uint8_t data_to_send[], size_t data_len, uint8_t out_array[], size_t out_array_max_len, size_t *out_len);
