// SPDX-License-Identifier: GPL-2.0-or-later

#include <linux/module.h>
#include "zl3073x.h"

/*
 * Regmap ranges
 */
#define ZL3073x_PAGE_SIZE	128
#define ZL3073x_NUM_PAGES	16
#define ZL3073x_PAGE_SEL	0x7F

static const struct regmap_range_cfg zl3073x_regmap_ranges[] = {
	{
		.range_min	= 0,
		.range_max	= ZL3073x_NUM_PAGES * ZL3073x_PAGE_SIZE,
		.selector_reg	= ZL3073x_PAGE_SEL,
		.selector_mask	= GENMASK(3, 0),
		.selector_shift	= 0,
		.window_start	= 0,
		.window_len	= ZL3073x_PAGE_SIZE,
	},
};

/*
 * Regmap config
 */
const struct regmap_config zl3073x_regmap_config = {
	.reg_bits		= 8,
	.val_bits		= 8,
	.max_register		= ZL3073x_NUM_PAGES * ZL3073x_PAGE_SIZE,
	.ranges			= zl3073x_regmap_ranges,
	.num_ranges		= ARRAY_SIZE(zl3073x_regmap_ranges),
};

/**
 * zl3073x_get_regmap_config - return pointer to regmap config
 *
 * Returns pointer to regmap config
 */
const struct regmap_config *zl3073x_get_regmap_config(void)
{
	return &zl3073x_regmap_config;
}
EXPORT_SYMBOL_GPL(zl3073x_get_regmap_config);

struct zl3073x_dev *zl3073x_dev_alloc(struct device *dev)
{
	struct zl3073x_dev *zldev;

	return devm_kzalloc(dev, sizeof(*zldev), GFP_KERNEL);
}
EXPORT_SYMBOL_GPL(zl3073x_dev_alloc);

int zl3073x_dev_init(struct zl3073x_dev *zldev)
{
	mutex_init(&zldev->lock);

	return 0;
}
EXPORT_SYMBOL_GPL(zl3073x_dev_init);

void zl3073x_dev_exit(struct zl3073x_dev *zldev)
{
	mutex_destroy(&zldev->lock);
}
EXPORT_SYMBOL_GPL(zl3073x_dev_exit);

MODULE_AUTHOR("Ivan Vecera <ivecera@redhat.com>");
MODULE_DESCRIPTION("Microchip ZL3073x core driver");
MODULE_LICENSE("GPL");
