/* SPDX-License-Identifier: GPL-2.0+ */

#ifndef __ZL80X32_CORE_H
#define __ZL80X32_CORE_H

#include <linux/mfd/zl80x32.h>
#include <net/devlink.h>

struct zl80x32_dev *zl80x32_dev_alloc(struct device *dev);
int zl80x32_dev_init(struct zl80x32_dev *zldev);
void zl80x32_dev_exit(struct zl80x32_dev *zldev);
const struct regmap_config *zl80x32_get_regmap_config(void);

/*
 * Misc functions
 */
int zl80x32_flash_update(struct devlink *devlink,
			 struct devlink_flash_update_params *params,
			 struct netlink_ext_ack *extack);

#endif /* __ZL80X32_CORE_H */
