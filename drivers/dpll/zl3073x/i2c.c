// SPDX-License-Identifier: GPL-2.0-only

#include <linux/dev_printk.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regmap.h>

#include "core.h"
#include "dpll.h"

static char *pin_states_monitored = NULL;
module_param(pin_states_monitored, charp, 0644);
MODULE_PARM_DESC(pin_states_monitored, "comma seperated allowed values: selectable,connected,disconnected");

static int
zl3073x_event_states_update(struct zl3073x_dev *zldpll)
{
	char *str, *token, *cur;
	size_t len;

	zldpll->events_state_mask = 0;

	if (pin_states_monitored == NULL) {
		zldpll->events_state_mask =
			( 1 << DPLL_PIN_STATE_SELECTABLE |
			  1 << DPLL_PIN_STATE_CONNECTED |
			  1 << DPLL_PIN_STATE_DISCONNECTED);
		return 0;
	}

	len = strlen(pin_states_monitored);
	if (len > 64) {
		pr_err("Invalid command line arguments: Too Large\n");
		return -E2BIG;
	}

	/* Copy str to other buffer */
	str = kstrdup(pin_states_monitored, GFP_KERNEL);
	if (!str) {
		pr_err("Failed while copying arguments\n");
		return -ENOMEM;
	}

	cur = str;
	if (cur[0] == '\0') {
		zldpll->events_state_mask = 0;
		kfree(str);
		return 0;
	}
	while ((token = strsep(&cur, ",")) != NULL) {
		if (strcmp(token, "selectable") == 0)
			zldpll->events_state_mask |=
				(1 << DPLL_PIN_STATE_SELECTABLE);
		else if (strcmp(token, "connected") == 0)
			zldpll->events_state_mask |=
				(1 << DPLL_PIN_STATE_CONNECTED);
		else if (strcmp(token, "disconnected") == 0)
			zldpll->events_state_mask |=
				(1 << DPLL_PIN_STATE_DISCONNECTED);
		else {
			pr_err("Error: Invalid input:%s\n", token);
			kfree(str);
			return -EINVAL;
		}
	}
	kfree(str);
	return 0;
}

static int zl3073x_i2c_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct zl3073x_dev *zldev;
	int rc;

	zldev = zl3073x_devm_alloc(dev);
	if (IS_ERR(zldev))
		return PTR_ERR(zldev);

	zldev->regmap = devm_regmap_init_i2c(client, &zl3073x_regmap_config);
	if (IS_ERR(zldev->regmap)) {
		dev_err_probe(dev, PTR_ERR(zldev->regmap),
			      "Failed to initialize regmap\n");
		return PTR_ERR(zldev->regmap);
	}

	rc = zl3073x_event_states_update(zldev);
	if (rc)
		return rc;

	/* Initialize device and use I2C address as dev ID */
	return zl3073x_dev_probe(zldev, i2c_get_match_data(client),
				 client->addr);
}

static const struct i2c_device_id zl3073x_i2c_id[] = {
	{
		.name = "zl30731",
		.driver_data = (kernel_ulong_t)&zl3073x_chip_info[ZL30731],
	},
	{
		.name = "zl30732",
		.driver_data = (kernel_ulong_t)&zl3073x_chip_info[ZL30732],
	},
	{
		.name = "zl30733",
		.driver_data = (kernel_ulong_t)&zl3073x_chip_info[ZL30733],
	},
	{
		.name = "zl30734",
		.driver_data = (kernel_ulong_t)&zl3073x_chip_info[ZL30734],
	},
	{
		.name = "zl30735",
		.driver_data = (kernel_ulong_t)&zl3073x_chip_info[ZL30735],
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(i2c, zl3073x_i2c_id);

static const struct of_device_id zl3073x_i2c_of_match[] = {
	{
		.compatible = "microchip,zl30731",
		.data = &zl3073x_chip_info[ZL30731],
	},
	{
		.compatible = "microchip,zl30732",
		.data = &zl3073x_chip_info[ZL30732],
	},
	{
		.compatible = "microchip,zl30733",
		.data = &zl3073x_chip_info[ZL30733],
	},
	{
		.compatible = "microchip,zl30734",
		.data = &zl3073x_chip_info[ZL30734],
	},
	{
		.compatible = "microchip,zl30735",
		.data = &zl3073x_chip_info[ZL30735],
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, zl3073x_i2c_of_match);

static struct i2c_driver zl3073x_i2c_driver = {
	.driver = {
		.name = "zl3073x-i2c",
		.of_match_table = zl3073x_i2c_of_match,
	},
	.probe = zl3073x_i2c_probe,
	.id_table = zl3073x_i2c_id,
};
module_i2c_driver(zl3073x_i2c_driver);

MODULE_AUTHOR("Ivan Vecera <ivecera@redhat.com>");
MODULE_DESCRIPTION("Microchip ZL3073x I2C driver");
MODULE_IMPORT_NS("ZL3073X");
MODULE_LICENSE("GPL");
