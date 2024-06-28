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

/**
 * struct zl80x32_reg_info - Device register definition
 *
 * @addr: device register address
 * @len: device register size in bytes
 */
struct zl80x32_reg_info {
	u16	addr;
	u16	len;
};

#define ZL80X32_REG_DEFINE(_name, _addr, _len)				\
	static const struct zl80x32_reg_info zl80x32_reg_##_name = {	\
		.addr	= ZL80X32_REG_##_addr,				\
		.len	= _len,						\
	}

#define ZL80X32_REG(name) (&zl80x32_reg_##name)

#define ZL80X32_REG_INFO			0x0000
ZL80X32_REG_DEFINE(info, INFO, 1);

#define ZL80X32_REG_ID				0x0001
ZL80X32_REG_DEFINE(id, ID, 2);

#define ZL80X32_REG_I2C_DEVICE_ADDR		0x003e
ZL80X32_REG_DEFINE(i2c_device_addr, I2C_DEVICE_ADDR, 1);

#endif /* __LINUX_MFD_ZL80X32_H */
