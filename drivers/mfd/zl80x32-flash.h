/* SPDX-License-Identifier: GPL-2.0+ */

#ifndef __ZL80X32_FLASH_H
#define __ZL80X32_FLASH_H

#include "zl80x32.h"

struct zl80x32_heximage;

/**
 * struct zl80x32_heximage_info - Flash hex-image type info
 *
 * @name: Flash image name
 * @max_words: Maximal image size in 32-bit words
 * @flash_op: Operation specific to flash image type
 * @load_addr: Device memory address where should be the image loaded
 * @flash_page: Destination page where should be the image flashed
 * @backup_page: Destination page where should be @flash_page copied
 */
struct zl80x32_heximage_info {
	const char	*name;
	size_t		max_words;
	int		(*flash_op)(struct zl80x32_dev *zldev,
				    struct zl80x32_heximage *image);
	u32		load_addr;
	u32		flash_page;
	u32		backup_page;
};

/**
 * struct zl80x32_heximage - Flash hex-image structure
 * @info: Hex-image type info
 * @words: Buffer with data to be flashed
 * @nwords: Size of the buffer in 32-bit words
 */
struct zl80x32_heximage {
	const struct zl80x32_heximage_info	*info;
	u32					*words;
	u32					nwords;
};

#endif /* __ZL80X32_FLASH_H */
