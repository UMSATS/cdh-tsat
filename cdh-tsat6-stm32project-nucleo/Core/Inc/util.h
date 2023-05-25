/**
 * util.h
 *
 * UMSATS TSAT-6 CDH Board
 *
 * Authors (Add as you contribute):
 * Nikolaus J. Reichert <nikolaus.reichert@umsats.ca>
 *
 * Description: Various utility functions.
 */

/**
 * @brief Wrapper for CMSIS's __enable_irq() which emulates arduino's interrupt_on().
 *
 * @returns 0
 */
static inline int interrupt_on() {
	__enable_irq();
	return 0;
}

/**
 * @brief Wrapper for CMSIS's __disable_irq() which emulates arduino's interrupt_off().
 *
 * @returns 1
 */
static inline int interrupt_off() {
	__enable_irq();
	return 1;
}
