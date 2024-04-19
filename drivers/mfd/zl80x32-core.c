// SPDX-License-Identifier: GPL-2.0-or-later

#include <linux/module.h>
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
