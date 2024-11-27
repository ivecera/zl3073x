// SPDX-License-Identifier: GPL-2.0-or-later

#include "zl80x32.h"
#include "zl80x32-flash.h"

/**
 * enum zl80x32_heximage_id - Identifiers for possible flash image types
 */
enum zl80x32_heximage_id {
	ZL80X32_HEXIMAGE_INVALID = -1,
	ZL80X32_HEXIMAGE_UTIL = 0,
	ZL80X32_HEXIMAGE_FW1,
	ZL80X32_HEXIMAGE_FW2,
	ZL80X32_HEXIMAGE_FW3,
	ZL80X32_HEXIMAGE_CFG0,
	ZL80X32_HEXIMAGE_CFG1,
	ZL80X32_HEXIMAGE_CFG2,
	ZL80X32_HEXIMAGE_CFG3,
	ZL80X32_HEXIMAGE_CFG4,
	ZL80X32_HEXIMAGE_CFG5,
	ZL80X32_HEXIMAGE_CFG6,
	ZL80X32_NUM_HEXIMAGES,
};

/*
 * Array that specifies all possible flash image types
 */
static const struct zl80x32_heximage_info zl80x32_heximage_info[] = {
	[ZL80X32_HEXIMAGE_UTIL] = {
		.name		= "utility",
		.max_words	= 0x08c0,
		.load_addr	= 0x20000000,
	},
	[ZL80X32_HEXIMAGE_FW1] = {
		.name		= "firmware1",
		.max_words	= 0xd400,
		.load_addr	= 0x20002000,
		.flash_page	= 0x020,
	},
	[ZL80X32_HEXIMAGE_FW2] = {
		.name		= "firmware2",
		.max_words	= 0x0010,
		.load_addr	= 0x20000000,
		.flash_page	= 0x3e0,
		.backup_page	= 0x000,
	},
	[ZL80X32_HEXIMAGE_FW3] = {
		.name		= "firmware3",
		.max_words	= 0x0092,
		.load_addr	= 0x20000400,
		.flash_page	= 0x3e4,
		.backup_page	= 0x004,
	},
	[ZL80X32_HEXIMAGE_CFG0] = {
		.name		= "config0",
		.max_words	= 0x0400,
		.load_addr	= 0x20000000,
		.flash_page	= 0x3d0,
	},
	[ZL80X32_HEXIMAGE_CFG1] = {
		.name		= "config1",
		.max_words	= 0x0400,
		.load_addr	= 0x20000000,
		.flash_page	= 0x3c0,
	},
	[ZL80X32_HEXIMAGE_CFG2] = {
		.name		= "config2",
		.max_words	= 0x0400,
		.load_addr	= 0x20000000,
		.flash_page	= 0x3b0,
	},
	[ZL80X32_HEXIMAGE_CFG3] = {
		.name		= "config3",
		.max_words	= 0x0400,
		.load_addr	= 0x20000000,
		.flash_page	= 0x3a0,
	},
	[ZL80X32_HEXIMAGE_CFG4] = {
		.name		= "config4",
		.max_words	= 0x0400,
		.load_addr	= 0x20000000,
		.flash_page	= 0x390,
	},
	[ZL80X32_HEXIMAGE_CFG5] = {
		.name		= "config5",
		.max_words	= 0x0400,
		.load_addr	= 0x20000000,
		.flash_page	= 0x380,
	},
	[ZL80X32_HEXIMAGE_CFG6] = {
		.name		= "config6",
		.max_words	= 0x0400,
		.load_addr	= 0x20000000,
		.flash_page	= 0x370,
	},
};

/* Santity check */
static_assert(ZL80X32_NUM_HEXIMAGES == ARRAY_SIZE(zl80x32_heximage_info));

static enum zl80x32_heximage_id zl80x32_get_heximage_id(const char *name)
{
	size_t i;

	for (i = 0; i < ZL80X32_NUM_HEXIMAGES; i++)
		if (!strcasecmp(name, zl80x32_heximage_info[i].name))
			return i;

	return ZL80X32_HEXIMAGE_INVALID;
}

/**
 * zl80x32_heximage_alloc - Alloc structure to hold hex-image
 * @nwords: size of buffer in 32-bit words to store data
 *
 * Returns pointer to allocated structure in case of success or
 * error otherwise.
 */
static struct zl80x32_heximage *zl80x32_heximage_alloc(u32 nwords)
{
	struct zl80x32_heximage *image;

	image = kzalloc(sizeof(struct zl80x32_heximage), GFP_KERNEL);
	if (!image)
		return ERR_PTR(-ENOMEM);

	image->words = kcalloc(nwords, sizeof(u32), GFP_KERNEL);
	if (!image->words) {
		kfree(image);
		return ERR_PTR(-ENOMEM);
	}

	image->nwords = nwords;

	return image;
}

/**
 * zl80x32_heximage_free - Free allocated hex-image structure
 * @image: pointer to allocated structure
 */
static void zl80x32_heximage_free(struct zl80x32_heximage *image)
{
	if (image)
		kfree(image->words);

	kfree(image);
}

/**
 * zl80x32_heximage_readline - Read next line from hex-image
 * @dst: destination buffer
 * @dst_sz: destination buffer size
 * @src: source buffer
 * @src_sz: source buffer size
 *
 * Returns number of characters read in case of success or -EINVAL if
 * the line to be read is too long for destination buffer.
 */
static ssize_t zl80x32_heximage_readline(char *dst, size_t dst_sz,
					 const char *src, size_t src_sz)
{
	const char *ptr;
	size_t len;

	/* Skip any existing new-lines at the beginning */
	ptr = memchr_inv(src, '\n', src_sz);
	if (ptr) {
		src_sz -= ptr - src;
		src = ptr;
	}

	/* Now look for the next new-line in the source */
	ptr = memscan((void *)src, '\n', src_sz);
	len = ptr - src;

	/* Return if the source line is too long for destination */
	if (len >= dst_sz)
		return -EINVAL;

	/* Copy the line from source and append NUL char  */
	memcpy(dst, src, len);
	*(dst+len) = '\0';

	/* Return number of read chars */
	return len;
}

#define FLASH_ERR_PREFIX "FW update failed: "
#define FLASH_ERR_MSG(_zldev, _extack, _msg, ...) do {			\
	dev_err((_zldev)->dev, FLASH_ERR_PREFIX _msg "\n",		\
		## __VA_ARGS__);					\
	NL_SET_ERR_MSG_FMT_MOD((_extack), FLASH_ERR_PREFIX _msg,	\
			       ## __VA_ARGS__);				\
} while (0)

/**
 * zl80x32_heximage_load - Load next hex-image from source
 * @zldev: pointer to device structure
 * @src: source buffer pointer
 * @src_sz: size of source buffer
 * @extack: netlink extack pointer to report errors
 *
 * Loads hex-image from source and decreases 'src_sz' by number of
 * characters loaded from source.
 * Returns pointer to hex-image structure filled with loaded data or
 * error otherwise.
 */
static
struct zl80x32_heximage *zl80x32_heximage_load(struct zl80x32_dev *zldev,
					       const char *src,
					       size_t *src_sz,
					       struct netlink_ext_ack *extack)
{
	struct zl80x32_heximage *image = NULL;
	struct device *dev = zldev->dev;
	enum zl80x32_heximage_id id;
	u32 nwords, count;
	char line[32];
	ssize_t len;
	int rc;

	/* Fetch image name from input */
	len = zl80x32_heximage_readline(line, sizeof(line), src, *src_sz);
	if (len < 0)
		goto err_too_long;
	else if (!len)
		return ERR_PTR(-ENODATA);

	*src_sz -= len;
	src += len;

	dev_dbg(dev, "Hex-image '%s' found\n", line);

	id = zl80x32_get_heximage_id(line);
	if (id == ZL80X32_HEXIMAGE_INVALID) {
		FLASH_ERR_MSG(zldev, extack,
			      "FW parse error - unknown image type '%s'", line);
		return ERR_PTR(-EINVAL);
	}

	/* Fetch image size from input */
	len = zl80x32_heximage_readline(line, sizeof(line), src, *src_sz);
	if (len < 0)
		goto err_too_long;
	else if (!len) {
		FLASH_ERR_MSG(zldev, extack, "FW parse error - missing size");
		return ERR_PTR(-EINVAL);
	}

	rc = kstrtou32(line, 10, &nwords);
	if (rc) {
		FLASH_ERR_MSG(zldev, extack,
			      "FW parse error - invalid size: '%s'", line);
		return ERR_PTR(rc);
	}

	dev_dbg(dev, "Expected image size: %u 32bit words\n", nwords);

	/* Alloc hex-image */
	image = zl80x32_heximage_alloc(nwords);
	if (IS_ERR(image)) {
		FLASH_ERR_MSG(zldev, extack, "Failed to alloc memory");
		return ERR_PTR(-ENOMEM);
	}

	image->info = &zl80x32_heximage_info[id];

	/* Load image data */
	for (count = 0; count < nwords; count++) {
		len = zl80x32_heximage_readline(line, sizeof(line), src,
						*src_sz);
		if (len < 0) {
			goto err_too_long;
		} else if (!len) {
			FLASH_ERR_MSG(zldev, extack,
				      "FW parse error - missing data");
			goto err_common;
		}

		rc = kstrtou32(line, 16, &image->words[count]);
		if (rc) {
			FLASH_ERR_MSG(zldev, extack,
				      "FW parse error - invalid data: '%s'",
				      line);
			goto err_common;
		}
	}

	return image;

err_too_long:
	FLASH_ERR_MSG(zldev, extack, "FW parse error - line too long");
	rc = -EINVAL;

err_common:
	zl80x32_heximage_free(image);

	return ERR_PTR(rc);
}

/**
 * zl80x32_flash_update - Devlink flash update callback
 * @devlink: devlink structure pointer
 * @params: flashing parameters pointer
 * @extack: netlink ext ack pointer to report errors
 *
 * Returns 0 in case of success or negative value otherwise
 */
int zl80x32_flash_update(struct devlink *devlink,
			 struct devlink_flash_update_params *params,
			 struct netlink_ext_ack *extack)
{
	struct zl80x32_dev *zldev = devlink_priv(devlink);
	struct zl80x32_heximage *image;
	const char *ptr;
	size_t size;
	int rc;

	devlink_flash_update_status_notify(devlink, "Preparing to flash",
					   params->component, 0, 0);

	ptr = params->fw->data;
	size = params->fw->size;

	do {
		image = zl80x32_heximage_load(zldev, ptr, &size, extack);
		if (IS_ERR(image)) {
			rc = PTR_ERR(image);
			break;
		}

		zl80x32_heximage_free(image);
	} while (1);

	devlink_flash_update_status_notify(devlink, "Flashing done",
					   params->component, 0, 0);

	return 0;
}
