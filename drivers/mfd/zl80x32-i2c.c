// SPDX-License-Identifier: GPL-2.0-or-later

#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include "zl80x32.h"

static const struct i2c_device_id zl80x32_i2c_id[] = {
	{ "zl80032-i2c", },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(i2c, zl80x32_i2c_id);

static const struct of_device_id zl80x32_i2c_of_match[] = {
	{ .compatible = "microchip,zl80032-i2c" },
	{ /* sentinel */ },
};

static int zl80x32_i2c_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	const struct i2c_device_id *id;
	struct zl80x32_dev *zldev;
	int rc = 0;

	zldev = zl80x32_dev_alloc(dev);
	if (!zldev)
		return -ENOMEM;

	id = i2c_client_get_device_id(client);
	zldev->dev = dev;

	zldev->regmap = devm_regmap_init_i2c(client,
					     zl80x32_get_regmap_config());
	if (IS_ERR(zldev->regmap)) {
		rc = PTR_ERR(zldev->regmap);
		dev_err(dev, "Failed to allocate register map: %d\n", rc);
		return rc;
	}

	i2c_set_clientdata(client, zldev);

	return zl80x32_dev_init(zldev);
}

static void zl80x32_i2c_remove(struct i2c_client *client)
{
	struct zl80x32_dev *zldev;

	zldev = i2c_get_clientdata(client);
	zl80x32_dev_exit(zldev);
}

static struct i2c_driver zl80x32_i2c_driver = {
	.driver = {
		.name = "zl80x32",
		.of_match_table = of_match_ptr(zl80x32_i2c_of_match),
	},
	.probe = zl80x32_i2c_probe,
	.remove = zl80x32_i2c_remove,
	.id_table = zl80x32_i2c_id,
};

module_i2c_driver(zl80x32_i2c_driver);

MODULE_AUTHOR("Ivan Vecera <ivecera@redhat.com>");
MODULE_AUTHOR("Petr Oros <poros@redhat.com>");
MODULE_DESCRIPTION("Microchip ZL80032 core driver");
MODULE_LICENSE("GPL");
