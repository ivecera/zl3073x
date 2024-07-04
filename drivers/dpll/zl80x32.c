/* SPDX-License-Identifier: GPL-2.0+ */

#include <linux/mfd/zl80x32.h>
#include <linux/dpll.h>

static int
zl80x32_dpll_input_frequency_get(const struct dpll_pin *pin, void *pin_priv,
				 const struct dpll_device *dpll, void *dpll_priv,
				 u64 *frequency, struct netlink_ext_ack *extack)
{
        return 0;
}

static int
zl80x32_dpll_input_frequency_set(const struct dpll_pin *pin, void *pin_priv,
				 const struct dpll_device *dpll, void *dpll_priv,
				 u64 frequency, struct netlink_ext_ack *extack)
{
        return 0;
}

static int
zl80x32_dpll_input_state_get(const struct dpll_pin *pin, void *pin_priv,
			     const struct dpll_device *dpll, void *dpll_priv,
			     enum dpll_pin_state *state,
			     struct netlink_ext_ack *extack)
{
        return 0;
}

static int
zl80x32_dpll_input_state_set(const struct dpll_pin *pin, void *pin_priv,
			     const struct dpll_device *dpll, void *dpll_priv,
			     enum dpll_pin_state state,
			     struct netlink_ext_ack *extack)
{
        return 0;
}

static int
zl80x32_dpll_input_prio_get(const struct dpll_pin *pin, void *pin_priv,
			    const struct dpll_device *dpll, void *dpll_priv,
			    u32 *prio, struct netlink_ext_ack *extack)
{
	return 0;
}

static int
zl80x32_dpll_input_prio_set(const struct dpll_pin *pin, void *pin_priv,
			    const struct dpll_device *dpll, void *dpll_priv,
			    u32 prio, struct netlink_ext_ack *extack)
{
	return 0;
}

static int
zl80x32_dpll_output_frequency_get(const struct dpll_pin *pin, void *pin_priv,
				  const struct dpll_device *dpll, void *dpll_priv,
				  u64 *frequency, struct netlink_ext_ack *extack)
{
        return 0;
}

static int
zl80x32_dpll_output_frequency_set(const struct dpll_pin *pin, void *pin_priv,
				  const struct dpll_device *dpll, void *dpll_priv,
				  u64 frequency, struct netlink_ext_ack *extack)
{
        return 0;
}

static int
zl80x32_dpll_output_state_get(const struct dpll_pin *pin, void *pin_priv,
			      const struct dpll_device *dpll, void *dpll_priv,
			      enum dpll_pin_state *state,
			      struct netlink_ext_ack *extack)
{
        return 0;
}

static int
zl80x32_dpll_output_state_set(const struct dpll_pin *pin, void *pin_priv,
			      const struct dpll_device *dpll, void *dpll_priv,
			      enum dpll_pin_state state,
			      struct netlink_ext_ack *extack)
{
        return 0;
}

static int
zl80x32_dpll_lock_status_get(const struct dpll_device *dpll, void *dpll_priv,
                             enum dpll_lock_status *status,
                             enum dpll_lock_status_error *status_error,
                             struct netlink_ext_ack *extack)
{
	return 0;
}

static int
zl80x32_dpll_mode_get(const struct dpll_device *dpll, void *dpll_priv,
		      enum dpll_mode *mode, struct netlink_ext_ack *extack)
{
	return 0;
}

static const struct dpll_pin_ops zl80x32_dpll_input_ops = {
        .frequency_get = zl80x32_dpll_input_frequency_get,
        .frequency_set = zl80x32_dpll_input_frequency_set,
        .state_on_dpll_get = zl80x32_dpll_input_state_get,
        .state_on_dpll_set = zl80x32_dpll_input_state_set,
        .prio_get = zl80x32_dpll_input_prio_get,
        .prio_set = zl80x32_dpll_input_prio_set,
};

static const struct dpll_pin_ops zl80x32_dpll_output_ops = {
        .frequency_get = zl80x32_dpll_output_frequency_get,
        .frequency_set = zl80x32_dpll_output_frequency_set,
        .state_on_dpll_get = zl80x32_dpll_output_state_get,
        .state_on_dpll_set = zl80x32_dpll_output_state_set,
};

static const struct dpll_device_ops zl80x32_dpll_ops = {
        .lock_status_get = zl80x32_dpll_lock_status_get,
        .mode_get = zl80x32_dpll_mode_get,
};

void zl80x32_dpll_init(struct zl80x32_dev * dev)
{
	// register dpll device
	// init pins
}
EXPORT_SYMBOL_GPL(zl80x32_dpll_init);

MODULE_AUTHOR("Ivan Vecera <ivecera@redhat.com>");
MODULE_AUTHOR("Petr Oros <poros@redhat.com>");
MODULE_DESCRIPTION("Microchip ZL80x32 DPLL driver");
MODULE_LICENSE("GPL");
