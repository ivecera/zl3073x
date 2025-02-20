#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/mfd/zl3073x.h>

struct i2c_client *client;

static struct zl3073x_platform_data platform_data = {
	.dpll_types = {
		DPLL_TYPE_PPS,
		DPLL_TYPE_EEC,
	},
	.input_pins = {
		ZL3037X_PIN_PROP("INPUT0P", DPLL_PIN_TYPE_SYNCE_ETH_PORT),
		ZL3037X_PIN_PROP("INPUT0N", DPLL_PIN_TYPE_EXT),
		ZL3037X_PIN_PROP("INPUT1P", DPLL_PIN_TYPE_EXT),
		ZL3037X_PIN_PROP("INPUT1N", DPLL_PIN_TYPE_EXT),
		ZL3037X_PIN_PROP("INPUT2P", DPLL_PIN_TYPE_EXT),
		ZL3037X_PIN_PROP("INPUT2N", DPLL_PIN_TYPE_EXT),
		ZL3037X_PIN_PROP("INPUT3P", DPLL_PIN_TYPE_EXT),
		ZL3037X_PIN_PROP("INPUT3N", DPLL_PIN_TYPE_EXT),
		ZL3037X_PIN_PROP("INPUT4P", DPLL_PIN_TYPE_EXT),
		ZL3037X_PIN_PROP("INPUT4N", DPLL_PIN_TYPE_EXT),
	},
	.output_pins = {
		ZL3037X_PIN_PROP("OUTPUT0P", DPLL_PIN_TYPE_INT_OSCILLATOR),
		ZL3037X_PIN_PROP("OUTPUT0N", DPLL_PIN_TYPE_GNSS),
		ZL3037X_PIN_PROP("OUTPUT1P", DPLL_PIN_TYPE_GNSS),
		ZL3037X_PIN_PROP("OUTPUT1N", DPLL_PIN_TYPE_GNSS),
		ZL3037X_PIN_PROP("OUTPUT2P", DPLL_PIN_TYPE_GNSS),
		ZL3037X_PIN_PROP("OUTPUT2N", DPLL_PIN_TYPE_GNSS),
		ZL3037X_PIN_PROP("OUTPUT3P", DPLL_PIN_TYPE_GNSS),
		ZL3037X_PIN_PROP("OUTPUT3N", DPLL_PIN_TYPE_GNSS),
		ZL3037X_PIN_PROP("OUTPUT4P", DPLL_PIN_TYPE_GNSS),
		ZL3037X_PIN_PROP("OUTPUT4N", DPLL_PIN_TYPE_GNSS),
		ZL3037X_PIN_PROP("OUTPUT5P", DPLL_PIN_TYPE_GNSS),
		ZL3037X_PIN_PROP("OUTPUT5N", DPLL_PIN_TYPE_GNSS),
		ZL3037X_PIN_PROP("OUTPUT6P", DPLL_PIN_TYPE_GNSS),
		ZL3037X_PIN_PROP("OUTPUT6N", DPLL_PIN_TYPE_GNSS),
		ZL3037X_PIN_PROP("OUTPUT7P", DPLL_PIN_TYPE_GNSS),
		ZL3037X_PIN_PROP("OUTPUT7N", DPLL_PIN_TYPE_GNSS),
		ZL3037X_PIN_PROP("OUTPUT8P", DPLL_PIN_TYPE_GNSS),
		ZL3037X_PIN_PROP("OUTPUT8N", DPLL_PIN_TYPE_GNSS),
		ZL3037X_PIN_PROP("OUTPUT9P", DPLL_PIN_TYPE_GNSS),
		ZL3037X_PIN_PROP("OUTPUT9N", DPLL_PIN_TYPE_GNSS),
	},
	.output_pairs = {
		ZL3073X_OUTPUT_PAIR_PROP(ZL3073X_SINGLE_ENDED_IN_PHASE,
					 ZL3073X_SYNCE),/* OUT0 */
		ZL3073X_OUTPUT_PAIR_PROP(ZL3073X_SINGLE_ENDED_IN_PHASE,
					 ZL3073X_SYNCE),/* OUT1 */
		ZL3073X_OUTPUT_PAIR_PROP(ZL3073X_SINGLE_ENDED_IN_PHASE,
					 ZL3073X_PTP),	/* OUT2 */
		ZL3073X_OUTPUT_PAIR_PROP(ZL3073X_SINGLE_ENDED_IN_PHASE,
					 ZL3073X_PTP),	/* OUT3 */
		ZL3073X_OUTPUT_PAIR_PROP(ZL3073X_SINGLE_ENDED_IN_PHASE,
					 ZL3073X_PTP),	/* OUT4 */
		ZL3073X_OUTPUT_PAIR_PROP(ZL3073X_SINGLE_ENDED_IN_PHASE,
					 ZL3073X_PTP),	/* OUT5 */
		ZL3073X_OUTPUT_PAIR_PROP(ZL3073X_SINGLE_ENDED_IN_PHASE,
					 ZL3073X_PTP),	/* OUT6 */
		ZL3073X_OUTPUT_PAIR_PROP(ZL3073X_SINGLE_ENDED_IN_PHASE,
					 ZL3073X_PTP),	/* OUT7 */
		ZL3073X_OUTPUT_PAIR_PROP(ZL3073X_SINGLE_ENDED_IN_PHASE,
					 ZL3073X_PTP),	/* OUT8 */
		ZL3073X_OUTPUT_PAIR_PROP(ZL3073X_SINGLE_ENDED_IN_PHASE,
					 ZL3073X_PTP),	/* OUT9 */
	},
};

/* Sofware Nodes and properties... this example corresponds
 * to device tree:
 *
 * ...
 *	i2c@70 {
 *		compatible = "microchip,zl3073x-i2c";
 *		reg = <0x70>;
 *		#address-cells = <1>;
 *		#size-cells = <0>;
 *		status = "okay";
 *
 *		phc@0 {
 *			reg = <0>;
 *		};
 *	};
 * ...
 *
 * It defines the zl3073x DPLL device on I2C bus with address 0x70 and
 * PHC device on the DPLL channel 0.
 */
static const struct property_entry zl3073x_i2c_properties[] = {
	PROPERTY_ENTRY_U32("reg", 0x70),
	{ },
};

static const struct software_node zl3073x_i2c_node = {
	.name = "zl3073x-i2c",
	.properties = zl3073x_i2c_properties,
};

static const struct property_entry zl3073x_phc_properties[] = {
	PROPERTY_ENTRY_U32("reg", 0x0),
	{ },
};

static const struct software_node zl3073x_phc_node = {
	.name = "phc",
	.parent = &zl3073x_i2c_node,
	.properties = zl3073x_phc_properties,
};

static const struct software_node *zl3073x_node_group[] = {
	&zl3073x_i2c_node,
	&zl3073x_phc_node,
	NULL,
};

static int __init eds2_zl3073x_init(void)
{
	struct i2c_board_info info = { };
	struct i2c_adapter *adap;
	struct device_node *np;
	int rc;

	/* 1. Find I2C adapter where the DPLL chip is connected to */
	/* This part is Microchip EDS2 specific on different platform
	 * and board you have to find the correct I2C adapter.
	 */
	pr_info("Finding I2C Adapter...\n");
	np = of_find_node_by_path("/soc/flexcom@e0044000/i2c@600");
	if (!np) {
		pr_err("Failed to find I2C adapter node\n");
		return -ENODEV;
	}

	adap = of_get_i2c_adapter_by_node(np);
	of_node_put(np);
	if (!adap) {
		pr_err("Device node is not I2C adapter\n");
		rc = -ENODEV;
		goto out;
	}

	/* 2. Register DPLL device node with optional PHC sub-node(s) */
	rc = software_node_register_node_group(zl3073x_node_group);
	if (rc) {
		pr_err("Failed to register node group\n");
		goto out;
	}

	/* 3. Prepare I2C board info for the DPLL chip
	 * - set i2c device address
	 * - associate i2c client device with our software node
	 * - assign optionally platform data
	 */
	strscpy(info.type, "zl3073x-i2c", sizeof(info.type));
	info.addr = 0x70;
	info.swnode = &zl3073x_i2c_node;
	info.platform_data = &platform_data;

	/* 4. Create i2c client device on the adapter */
	client = i2c_new_client_device(adap, &info);
	if (IS_ERR(client)) {
		pr_err("Failed to register DPLL device\n");
		rc = PTR_ERR(client);
	}

out:
	if (rc)
		software_node_unregister_node_group(zl3073x_node_group);

	if (adap)
		i2c_put_adapter(adap);

	return rc;
}

/* Module exit function */
static void __exit eds2_zl3073x_exit(void)
{
    pr_info("Unloading I2C Enumerator Module...\n");

    /* 1. Remove i2c client device */
    i2c_unregister_device(client);
    /* 2. Unregister device nodes */
    software_node_unregister_node_group(zl3073x_node_group);
}

/* Module metadata */
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ivan Vecera <ivecera@redhat.com");
MODULE_DESCRIPTION("Sample module to create zl3703x-i2c device");
MODULE_VERSION("1.0");

module_init(eds2_zl3073x_init);
module_exit(eds2_zl3073x_exit);
