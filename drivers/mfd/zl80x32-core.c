// SPDX-License-Identifier: GPL-2.0-or-later

#include <linux/bitfield.h>
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

/**
 * zl80x32_devlink_info_get - Devlink device info callback
 * @devlink: devlink structure pointer
 * @req: devlink request pointer to store information
 * @extack: netlink extack pointer to report errors
 *
 * Returns 0 in case of success or negative value otherwise
 */
static int zl80x32_devlink_info_get(struct devlink *devlink,
				    struct devlink_info_req *req,
				    struct netlink_ext_ack *extack)
{
	struct zl80x32_dev *zldev = devlink_priv(devlink);
	unsigned int id, revision, fw_ver, cfg_ver;
	char buf[16];
	int rc;

	mutex_lock(&zldev->lock);

	rc = zl80x32_reg_read(zldev, ZL80X32_REG(id), &id);
	if (rc)
		goto finish;

	snprintf(buf, sizeof(buf), "%X", id);
	rc = devlink_info_version_fixed_put(req,
					DEVLINK_INFO_VERSION_GENERIC_ASIC_ID,
					buf);
	if (rc)
		goto finish;

	rc = zl80x32_reg_read(zldev, ZL80X32_REG(revision), &revision);
	if (rc)
		goto finish;

	snprintf(buf, sizeof(buf), "%X", revision);
	rc = devlink_info_version_fixed_put(req,
					DEVLINK_INFO_VERSION_GENERIC_ASIC_REV,
					buf);
	if (rc)
		goto finish;

	rc = zl80x32_reg_read(zldev, ZL80X32_REG(fw_ver), &fw_ver);
	if (rc)
		goto finish;

	snprintf(buf, sizeof(buf), "%u", fw_ver);
	rc = devlink_info_version_fixed_put(req,
					    DEVLINK_INFO_VERSION_GENERIC_FW,
					    buf);
	if (rc)
		goto finish;

	rc = zl80x32_reg_read(zldev, ZL80X32_REG(custom_config_ver), &cfg_ver);
	if (rc)
		goto finish;

	/* No custom config version */
	if (!cfg_ver)
		goto finish;

	snprintf(buf, sizeof(buf), "%lu.%lu.%lu.%lu",
		 FIELD_GET(GENMASK(31, 24), cfg_ver),
		 FIELD_GET(GENMASK(23, 16), cfg_ver),
		 FIELD_GET(GENMASK(15, 8), cfg_ver),
		 FIELD_GET(GENMASK(7, 0), cfg_ver));

	rc = devlink_info_version_running_put(req, "cfg.custom_ver", buf);

finish:
	mutex_unlock(&zldev->lock);

	return rc;
}

static const struct devlink_ops zl80x32_devlink_ops = {
	.info_get = zl80x32_devlink_info_get,
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
	unsigned int id, revision, fw_ver, cfg_ver;
	struct devlink *devlink;

	mutex_init(&zldev->lock);

	mutex_lock(&zldev->lock);

	zl80x32_reg_read(zldev, ZL80X32_REG(id), &id);
	zl80x32_reg_read(zldev, ZL80X32_REG(revision), &revision);
	zl80x32_reg_read(zldev, ZL80X32_REG(fw_ver), &fw_ver);
	zl80x32_reg_read(zldev, ZL80X32_REG(custom_config_ver), &cfg_ver);

	mutex_unlock(&zldev->lock);

	dev_info(zldev->dev, "ChipID(%X), ChipRev(%X), FwVer(%u)\n",
		 id, revision, fw_ver);
	dev_info(zldev->dev, "Custom config version: %lu.%lu.%lu.%lu\n",
		 FIELD_GET(GENMASK(31, 24), cfg_ver),
		 FIELD_GET(GENMASK(23, 16), cfg_ver),
		 FIELD_GET(GENMASK(15, 8), cfg_ver),
		 FIELD_GET(GENMASK(7, 0), cfg_ver));

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
