/* Dhara - NAND flash management layer
 * Copyright (C) 2013 Daniel Beer <dlbeer@gmail.com>
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#ifndef DHARA_NAND_H_
#define DHARA_NAND_H_

#include <stdint.h>
#include <stddef.h>
#include "error.h"

/* Each page in a NAND device is indexed, starting at 0. It's required
 * that there be a power-of-two number of pages in a eraseblock, so you can
 * view a page number is being a concatenation (in binary) of a block
 * number and the number of a page within a block.
 */
typedef uint32_t dhara_page_t;

/* Blocks are also indexed, starting at 0. */
typedef uint32_t dhara_block_t;

/* Each NAND chip must be represented by one of these structures. It's
 * intended that this structure be embedded in a larger structure for
 * context.
 *
 * The functions declared below are not implemented -- they must be
 * provided and satisfy the documented conditions.
*/
struct dhara_nand {
	/* Base-2 logarithm of the page size. If your device supports
	 * partial programming, you may want to subdivide the actual
	 * pages into separate ECC-correctable regions and present those
	 * as pages.
	 */
	uint8_t		log2_page_size;

	/* Base-2 logarithm of the number of pages within an eraseblock */
	uint8_t		log2_ppb;

	/* Total number of eraseblocks */
	unsigned int	num_blocks;
};

/*
 * External declaration of the dhara_nand struct that is initialized in Drivers/Dhara/Src/nand.c
 *
 * Used in Drivers/Dhara/Src/Dhara_Wrappers.c and Drivers/Dhara/Src/nand.c
 *
 * It is located here to hide it from people so they do not use the values in my_nand rather they use
 * the defines located in the Dhara_Wrapper.h file.
*/
extern const struct dhara_nand my_nand;

/*
 * FUNCTION: dhara_nand_is_bad
 *
 * DESCRIPTION: Checks if given NAND block is bad and returns results
 *
 * NOTES:
 *  -NAND blocks have two locations where the bad block marker is stored, the first being the first byte in the block and
 *   the second being the first byte of the space area.
 *
 * RETURNS:
 *  0: If the given block is good.
 *  1: The given block is bad.
*/
int dhara_nand_is_bad(const struct dhara_nand *n, dhara_block_t b);

/*
 * FUNCTION: dhara_nand_mark_bad
 *
 * DESCRIPTION: Mark bad the given block (or attempt to).
 *
 * NOTES:
 *  -No return value is required, because there's nothing that can be done in response.
 *
*/
void dhara_nand_mark_bad(const struct dhara_nand *n, dhara_block_t b);

/*
 * FUNCTION: dhara_nand_erase
 *
 * DESCRIPTION: Erase the given block.
 *
 * NOTES:
 *  -This function should return 0 on success or -1 on failure.
 *  -The status reported by the chip should be checked. If an erase operation fails, return -1 and set err to E_BAD_BLOCK.
 *
 * RETURNS:
 *  -1: If it fails.
 *  0: If it succeeds.
 */
int dhara_nand_erase(const struct dhara_nand *n, dhara_block_t b,
		     dhara_error_t *err);

/*
 * FUNCTION: dhara_nand_prog
 *
 * DESCRIPTION: Program the given page.
 *
 * NOTES:
 *  -The data pointer is a pointer to an entire page ((1 << log2_page_size) bytes).
 *  -The operation status should be checked.
 *  -Pages will be programmed sequentially within a block, and will not be reprogrammed.
 *
 * RETURNS:
 *  -1: If the operation fail and set err to E_BAD_BLOCK.
 *  0: If it succeeds.
*/
int dhara_nand_prog(const struct dhara_nand *n, dhara_page_t p,
		    const uint8_t *data,
		    dhara_error_t *err);

/*
 * FUNCTION: dhara_nand_is_free
 *
 * DESCRIPTION: Determine whether a page is erased (unprogrammed)
 *
 * NOTES:
 *
 * RETURNS:
 *  0: If page is not free.
 *  1: If page is free.
*/
int dhara_nand_is_free(const struct dhara_nand *n, dhara_page_t p);

/*
 * FUNCTION: dhara_nand_read
 *
 * DESCRIPTION: Read a portion of a page.
 *
 * NOTES:
 *  -ECC must be handled by the NAND implementation.
 *  -If an uncorrectable ECC error occurs, return -1 and set err to E_ECC.
 *
 * RETURNS:
 *  -1: If it fails.
 *  0: If it succeeds.
*/
int dhara_nand_read(const struct dhara_nand *n, dhara_page_t p,
		    size_t offset, size_t length,
		    uint8_t *data,
		    dhara_error_t *err);

/*
 * FUNCTION: dhara_nand_copy
 *
 * DESCRIPTION: Read a page from one location and reprogram it in another location.
 *
 * NOTES:
 *  -This might be done using the chip's internal buffers, but it must use ECC.
 *
 * RETURNS:
 *  -1: If it fails.
 *  0: If it succeeds.
 */
int dhara_nand_copy(const struct dhara_nand *n,
		    dhara_page_t src, dhara_page_t dst,
		    dhara_error_t *err);

#endif
