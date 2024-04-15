// SPDX-License-Identifier: GPL-2.0-or-later

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/spi/spi.h>
#include "zl80x32.h"

static const struct spi_device_id zl80x32_spi_id[] = {
	{ "zl80032-spi", },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(spi, zl80x32_spi_id);

static const struct of_device_id zl80x32_spi_of_match[] = {
	{ .compatible = "microchip,zl80032-spi" },
	{ /* sentinel */ },
};

static int zl80x32_spi_probe(struct spi_device *spidev)
{
	struct device *dev = &spidev->dev;
	const struct spi_device_id *id;
	struct zl80x32_dev *zldev;
	int rc;

	zldev = zl80x32_dev_alloc(dev);
	if (!zldev)
		return -ENOMEM;

	id = spi_get_device_id(spidev);
	zldev->dev = dev;

	zldev->regmap = devm_regmap_init_spi(spidev,
					     zl80x32_get_regmap_config());
	if (IS_ERR(zldev->regmap)) {
		rc = PTR_ERR(zldev->regmap);
		dev_err(dev, "Failed to allocate register map: %d\n", rc);
		return rc;
	}

	spi_set_drvdata(spidev, zldev);

	return zl80x32_dev_init(zldev);
}

static void zl80x32_spi_remove(struct spi_device *spidev)
{
	struct zl80x32_dev *zldev;

	zldev = spi_get_drvdata(spidev);
	zl80x32_dev_exit(zldev);
}

static struct spi_driver zl80x32_spi_driver = {
	.driver = {
		.name = "zl80x32",
		.of_match_table = of_match_ptr(zl80x32_spi_of_match),
	},
	.probe = zl80x32_spi_probe,
	.remove = zl80x32_spi_remove,
	.id_table = zl80x32_spi_id,
};

module_spi_driver(zl80x32_spi_driver);

MODULE_AUTHOR("Ivan Vecera <ivecera@redhat.com>");
MODULE_AUTHOR("Petr Oros <poros@redhat.com>");
MODULE_DESCRIPTION("Microchip ZL80x32 SPI driver");
MODULE_LICENSE("GPL");
