// SPDX-License-Identifier: GPL-2.0-or-later

#include <linux/module.h>
#include <linux/unaligned.h>
#include <net/devlink.h>
#include "zl80x32.h"

#define ZL80032_PAGE_SIZE	128
#define ZL80032_NUM_PAGES	16
#define ZL80032_PAGE_SEL	0x7F

static const struct regmap_range_cfg zl80x32_regmap_ranges[] = {
	{
		.range_min	= 0,
		.range_max	= ZL80032_NUM_PAGES * ZL80032_PAGE_SIZE,
		.selector_reg	= ZL80032_PAGE_SEL,
		.selector_mask	= GENMASK(3, 0),
		.selector_shift	= 0,
		.window_start	= 0,
		.window_len	= ZL80032_PAGE_SIZE,
	},
};

const struct regmap_config zl80x32_regmap_config = {
	.reg_bits		= 8,
	.val_bits		= 8,
	.max_register		= ZL80032_NUM_PAGES * ZL80032_PAGE_SIZE,
	.reg_format_endian	= REGMAP_ENDIAN_BIG,
	.val_format_endian	= REGMAP_ENDIAN_BIG,
	.ranges			= zl80x32_regmap_ranges,
	.num_ranges		= ARRAY_SIZE(zl80x32_regmap_ranges),
};

const struct regmap_config *zl80x32_get_regmap_config(void)
{
	return &zl80x32_regmap_config;
}
EXPORT_SYMBOL_GPL(zl80x32_get_regmap_config);

/**
 * zl80x32_reg_read - Read value from device register
 * @zldev: device structure pointer
 * @reg_info: pointer to register definition
 * @value: place to store value read from device register
 *
 * Returns 0 in case of success or negative value otherwise
 */
static inline int zl80x32_reg_read(struct zl80x32_dev *zldev,
				   const struct zl80x32_reg_info *reg_info,
				   unsigned int *value)
{
	u8 buf[4];
	int rc;

	BUG_ON(!mutex_is_locked(&zldev->lock));

	rc = regmap_bulk_read(zldev->regmap, reg_info->addr, buf,
			      reg_info->len);
	if (rc)
		return rc;

	switch (reg_info->len) {
	case 1:
		*value = buf[0];
		break;
	case 2:
		*value = get_unaligned_be16(buf);
		break;
	case 4:
		*value = get_unaligned_be32(buf);
		break;
	default:
		BUG();
	}

	return rc;
}

/**
 * zl80x32_reg_write - Write value to device register
 * @zldev: device structure pointer
 * @reg_info: pointer to register definition
 * @value: value to write to device register
 *
 * Returns 0 in case of success or negative value otherwise
 */
static inline int zl80x32_reg_write(struct zl80x32_dev *zldev,
				    const struct zl80x32_reg_info *reg_info,
				    unsigned int value)
{
	u8 buf[4];

	BUG_ON(!mutex_is_locked(&zldev->lock));

	switch (reg_info->len) {
	case 1:
		buf[0] = value;
		break;
	case 2:
		put_unaligned_be16(value, buf);
		break;
	case 4:
		put_unaligned_be32(value, buf);
		break;
	default:
		BUG();
	}

	return regmap_bulk_write(zldev->regmap, reg_info->addr, buf,
				 reg_info->len);
}

/**
 * zl80x32_reg_write - Update value in device register
 * @zldev: device structure pointer
 * @reg_info: pointer to register definition
 * @value: value used to update to device register
 * @mask: mask speciifying bits to be updated
 *
 * Returns 0 in case of success or negative value otherwise
 */
static inline int zl80x32_reg_update(struct zl80x32_dev *zldev,
				     const struct zl80x32_reg_info *reg_info,
				     unsigned int value, unsigned int mask)
{
	unsigned int tmp;
	int rc;

	BUG_ON(!mutex_is_locked(&zldev->lock));

	rc = zl80x32_reg_read(zldev, reg_info, &tmp);
	if (rc)
		return rc;

	tmp &= ~mask;
	tmp |= value & mask;

	return zl80x32_reg_write(zldev, reg_info, tmp);
}

static const struct devlink_ops zl80x32_devlink_ops = {
};

static void zl80x32_devlink_free(void *ptr)
{
	devlink_free(ptr);
}

struct zl80x32_dev *zl80x32_dev_alloc(struct device *dev)
{
	struct devlink *devlink;

	devlink = devlink_alloc(&zl80x32_devlink_ops,
				sizeof(struct zl80x32_dev), dev);
	if (!devlink)
		return NULL;

	if (devm_add_action_or_reset(dev, zl80x32_devlink_free, devlink))
		return NULL;

	return devlink_priv(devlink);
}
EXPORT_SYMBOL_GPL(zl80x32_dev_alloc);

int zl80x32_dev_init(struct zl80x32_dev *zldev)
{
	struct devlink *devlink;

	mutex_init(&zldev->lock);

	devlink = priv_to_devlink(zldev);
	devlink_register(devlink);

	return 0;
}
EXPORT_SYMBOL_GPL(zl80x32_dev_init);

void zl80x32_dev_exit(struct zl80x32_dev *zldev)
{
	mutex_destroy(&zldev->lock);
}
EXPORT_SYMBOL_GPL(zl80x32_dev_exit);

MODULE_AUTHOR("Ivan Vecera <ivecera@redhat.com>");
MODULE_AUTHOR("Petr Oros <poros@redhat.com>");
MODULE_DESCRIPTION("Microchip ZL80032 core driver");
MODULE_LICENSE("GPL");
