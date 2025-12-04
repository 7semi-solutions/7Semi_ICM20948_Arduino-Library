#ifndef ICM20948_PLATFORM_H
#define ICM20948_PLATFORM_H

// #include <stdint.h>
// #include <stddef.h>

/**
 * 7Semi comment style
 * - Platform glue for bus IO, delays, and timing
 * - Implement these for your system (I2C or SPI)
 */

typedef struct {
  /** 
     * - Bus read: read `len` bytes from `reg` into `buf`
     * - Return 0 on success, nonzero on error
     */
  int (*read)(uint8_t reg, uint8_t *buf, size_t len, void *user);
  /** 
     * - Bus write: write `len` bytes from `buf` to `reg`
     * - Return 0 on success, nonzero on error
     */
  int (*write)(uint8_t reg, const uint8_t *buf, size_t len, void *user);
  /** 
     * - Optional: SPI transfers may need register R/W bit mangling
     * - For I2C, leave as NULL
     */
  uint8_t (*spi_reg_rw_bits)(uint8_t reg, int is_write);
  /** 
     * - Millisecond delay
     */
  void (*delay_ms)(uint32_t ms);
  /** 
     * - User context pointer passed to read/write
     */
  void *user;
  /** 
     * - If using SPI: set chip select active (1) or inactive (0)
     * - For I2C: leave as NULL
     */
  void (*spi_cs)(int level, void *user);
  /** 
     * - Device address (I2C) or ignored for SPI if you wrap inside user
     */
  uint8_t i2c_addr;
} icm_plat_t;

#endif
