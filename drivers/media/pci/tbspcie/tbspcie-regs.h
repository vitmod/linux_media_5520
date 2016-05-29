/*
 * PCIe driver for TBS cards based on ECP3 FPGA
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 2 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 */


#define TBSECP3_GPIO_PIN(_bank, _pin)	(((3 - _bank) << 5) + _pin)


#define TBS_GPIO_BASE		0x0000
#define TBS_GPIO_DATA(__n)	(0xc - (__n) * 4)
#define TBS_GPIO_DATA_3		0x00	/* adapter 0 */
#define TBS_GPIO_DATA_2		0x04	/* adapter 1 */
#define TBS_GPIO_DATA_1		0x08	/* adapter 2 */
#define TBS_GPIO_DATA_0		0x0c	/* adapter 3 */

#define TBS_I2C_BASE(__n)	(0x7000 - ((__n) * 0x1000))
#define TBS_I2C_BASE_3		0x4000
#define TBS_I2C_BASE_2		0x5000
#define TBS_I2C_BASE_1		0x6000
#define TBS_I2C_BASE_0		0x7000
#define TBS_I2C_CTRL		0x00
#define TBS_I2C_DATA		0x04
#define TBS_I2C_SPEED		0x08

#define TBS_INT_I2C_FLAG_0	0x08
#define TBS_INT_I2C_FLAG_1	0x04
#define TBS_INT_I2C_FLAG_2	0x02
#define TBS_INT_I2C_FLAG_3	0x01
#define TBS_INT_I2C_FLAG(__n)	(1 << (3 - (__n)))

#define TBS_INT_BASE		0xc000
#define TBS_INT_STATUS		0x00
#define TBS_INT_ENABLE		0x04
#define TBS_INT_I2C_MASK(__n)	(0x14 - (__n) * 4)
//#define TBS__I2C_MASK_3		0x08
//#define TBS_I2C_MASK_2		0x0c
//#define TBS_I2C_MASK_1		0x10
//#define TBS_I2C_MASK_0		0x14
#define TBS_INT_DMA_FLAG(__n)	((__n > 3) ? (0x800 >> (__n - 4)) : (0x80 >> (__n)))

#define TBS_DMA_MASK(i)		(0x18 + (3 - (i)) * 0x04)
#define TBS_DMA_MASK_3		0x18
#define TBS_DMA_MASK_2		0x1C
#define TBS_DMA_MASK_1		0x20
#define TBS_DMA_MASK_0		0x24
#define TBS_DMA2_MASK(i)	(0x28 + (7 - (i)) * 0x04)
#define TBS_DMA2_MASK_3		0x28
#define TBS_DMA2_MASK_2		0x2C
#define TBS_DMA2_MASK_1		0x30
#define TBS_DMA2_MASK_0		0x34

#define TBS_DMA_BASE(__n)		(0xb000 - ((__n) * 0x1000))
//#define TBS_DMA_BASE_3		0x8000
//#define TBS_DMA_BASE_2		0x9000
//#define TBS_DMA_BASE_1		0xa000
//#define TBS_DMA_BASE_0		0xb000
#define TBS_DMA2_BASE(__n)		(0xb800 - ((__n)-4) * 0x1000)
//#define TBS_DMA2_BASE_3			0x8800
//#define TBS_DMA2_BASE_2			0x9800
//#define TBS_DMA2_BASE_1			0xa800
//#define TBS_DMA2_BASE_0			0xb800

#define TBS_DMA_START		0x00
#define TBS_DMA_STATUS		0x00
#define TBS_DMA_SIZE		0x04
#define TBS_DMA_ADDR_HIGH	0x08
#define TBS_DMA_ADDR_LOW	0x0c
#define TBS_DMA_CELL_SIZE	0x10

