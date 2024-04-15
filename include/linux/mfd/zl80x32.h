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

#endif /* __LINUX_MFD_ZL80X32_H */
