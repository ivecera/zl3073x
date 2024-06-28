/* SPDX-License-Identifier: GPL-2.0+ */

#ifndef __LINUX_MFD_ZL80X32_H
#define __LINUX_MFD_ZL80X32_H

#include <linux/device.h>
#include <linux/regmap.h>

struct zl80x32_dev {
	struct device		*dev;
	struct regmap		*regmap;
	struct mutex		lock;
};

#define ZL80X32_REG_INFO			0x0000
#define ZL80X32_REG_ID				0x0001
#define ZL80X32_REG_I2C_DEVICE_ADDR		0x003e

#endif /* __LINUX_MFD_ZL80X32_H */
