// SPDX-License-Identifier: GPL-2.0-only

#include <linux/array_size.h>
#include <linux/bits.h>
#include <linux/bitfield.h>
#include <linux/bug.h>
#include <linux/container_of.h>
#include <linux/dev_printk.h>
#include <linux/dpll.h>
#include <linux/err.h>
#include <linux/kthread.h>
#include <linux/mfd/core.h>
#include <linux/mfd/zl3073x.h>
#include <linux/mfd/zl3073x-regs.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/netlink.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/sprintf.h>

#define ZL3073X_DPLL_REF_NONE		ZL3073X_NUM_INPUT_PINS
#define ZL3073X_DPLL_REF_IS_VALID(_ref)	((_ref) != ZL3073X_DPLL_REF_NONE)

/**
 * struct zl3073x_dpll_pin_info - DPLL pin info
 * @props: DPLL core pin properties
 * @package_label: pin package label
 */
struct zl3073x_dpll_pin_info {
	struct dpll_pin_properties	props;
	char				package_label[8];
};

/**
 * struct zl3073x_dpll_pin - DPLL pin
 * @dpll_pin: pointer to registered dpll_pin
 * @index: index in zl3073x_dpll.pins array
 * @prio: pin priority <0, 14>
 * @selectable: pin is selectable in automatic mode
 * @pin_state: last saved pin state
 */
struct zl3073x_dpll_pin {
	struct dpll_pin			*dpll_pin;
	u8				index;
	u8				prio;
	bool				selectable;
	enum dpll_pin_state		pin_state;
};

/**
 * struct zl3073x_dpll - ZL3073x DPLL sub-device structure
 * @dev: device pointer
 * @mfd: pointer to multi-function parent device
 * @id: DPLL index
 * @refsel_mode: reference selection mode
 * @forced_ref: selected reference in forced reference lock mode
 * @dpll_dev: pointer to registered DPLL device
 * @lock_status: last saved DPLL lock status
 * @pins: array of pins
 * @kworker: thread for periodic work
 * @work: periodic work
 */
struct zl3073x_dpll {
	struct device			*dev;
	struct zl3073x_dev		*mfd;
	int				id;
	u8				refsel_mode;
	u8				forced_ref;
	struct dpll_device		*dpll_dev;
	enum dpll_lock_status		lock_status;
	struct zl3073x_dpll_pin		pins[ZL3073X_NUM_PINS];

	struct kthread_worker		*kworker;
	struct kthread_delayed_work	work;
};

#define pin_to_dpll(_pin)						\
	container_of((_pin), struct zl3073x_dpll, pins[(_pin)->index])

#define pin_to_dev(_pin)						\
	pin_to_dpll(_pin)->mfd

/**
 * zl3073x_dpll_is_input_pin - check if the pin is input one
 * @pin: pin to check
 *
 * Return: true if pin is input, false if pin is output.
 */
static bool
zl3073x_dpll_is_input_pin(struct zl3073x_dpll_pin *pin)
{
	/* Output pins are stored in zl3073x_dpll.pins first and input
	 * pins follow.
	 */
	if (pin->index >= ZL3073X_NUM_OUTPUT_PINS)
		return true;

	return false;
}

/**
 * zl3073x_dpll_pin_index_get - get pin HW index
 * @pin: pin pointer
 *
 * Return: pin hardware index
 */
static u8
zl3073x_dpll_pin_index_get(struct zl3073x_dpll_pin *pin)
{
	if (zl3073x_dpll_is_input_pin(pin))
		return pin->index - ZL3073X_NUM_OUTPUT_PINS;

	return pin->index;
}

/**
 * zl3073x_dpll_is_n_pin - check if the pin is N-pin
 * @pin: pin to check
 *
 * Return: true if the pin is N-pin, false if it is P-pin
 */
static bool
zl3073x_dpll_is_n_pin(struct zl3073x_dpll_pin *pin)
{
	/* P-pins indices are even while N-pins are odd */
	return zl3073x_is_n_pin(zl3073x_dpll_pin_index_get(pin));
}

/**
 * zl3073x_dpll_is_p_pin - check if the pin is P-pin
 * @pin: pin to check
 *
 * Return: true if the pin is P-pin, false if it is N-pin
 */
static bool
zl3073x_dpll_is_p_pin(struct zl3073x_dpll_pin *pin)
{
	return zl3073x_is_p_pin(zl3073x_dpll_pin_index_get(pin));
}

/**
 * zl3073x_dpll_output_pin_output_get - get output index for given output pin
 * @pin: pointer to pin
 *
 * Return: output hardware index for the given output pin
 */
static u8
zl3073x_dpll_output_pin_output_get(struct zl3073x_dpll_pin *pin)
{
	WARN_ON(zl3073x_dpll_is_input_pin(pin));

	return zl3073x_dpll_pin_index_get(pin) / 2;
}

static int
zl3073x_dpll_pin_direction_get(const struct dpll_pin *dpll_pin, void *pin_priv,
			       const struct dpll_device *dpll, void *dpll_priv,
			       enum dpll_pin_direction *direction,
			       struct netlink_ext_ack *extack)
{
	struct zl3073x_dpll_pin *pin = pin_priv;

	if (zl3073x_dpll_is_input_pin(pin))
		*direction = DPLL_PIN_DIRECTION_INPUT;
	else
		*direction = DPLL_PIN_DIRECTION_OUTPUT;

	return 0;
}

/**
 * zl3073x_dpll_selected_ref_get - get currently selected reference
 * @zldpll: pointer to zl3073x_dpll
 * @ref: place to store selected reference
 *
 * Check for currently selected reference the DPLL should be locked to
 * and stores its index to given @ref.
 *
 * Return: 0 on success, <0 on error
 */
static int
zl3073x_dpll_selected_ref_get(struct zl3073x_dpll *zldpll, u8 *ref)
{
	struct zl3073x_dev *zldev = zldpll->mfd;
	u8 state, value;
	int rc;

	switch (zldpll->refsel_mode) {
	case ZL_DPLL_MODE_REFSEL_MODE_AUTO:
		/* For automatic mode read refsel_status register */
		rc = zl3073x_read_u8(zldev,
				     ZL_REG_DPLL_REFSEL_STATUS(zldpll->id),
				     &value);
		if (rc)
			return rc;

		/* Extract reference state */
		state = FIELD_GET(ZL_DPLL_REFSEL_STATUS_STATE, value);

		/* Return the reference only if the DPLL is locked to it */
		if (state == ZL_DPLL_REFSEL_STATUS_STATE_LOCK)
			*ref = FIELD_GET(ZL_DPLL_REFSEL_STATUS_REFSEL, value);
		else
			*ref = ZL3073X_DPLL_REF_NONE;
		break;
	case ZL_DPLL_MODE_REFSEL_MODE_REFLOCK:
		/* For manual mode return stored value */
		*ref = zldpll->forced_ref;
		break;
	default:
		/* For other modes like NCO, freerun... there is no input ref */
		*ref = ZL3073X_DPLL_REF_NONE;
		break;
	}

	return 0;
}

/**
 * zl3073x_dpll_connected_ref_get - get currently connected reference
 * @zldpll: pointer to zl3073x_dpll
 * @ref: place to store selected reference
 *
 * Looks for currently connected the DPLL is locked to and stores its index
 * to given @ref.
 *
 * Return: 0 on success, <0 on error
 */
static int
zl3073x_dpll_connected_ref_get(struct zl3073x_dpll *zldpll, u8 *ref)
{
	struct zl3073x_dev *zldev = zldpll->mfd;
	int rc;

	/* Get currently selected input reference */
	rc = zl3073x_dpll_selected_ref_get(zldpll, ref);
	if (rc)
		return rc;

	if (ZL3073X_DPLL_REF_IS_VALID(*ref)) {
		u8 ref_status;

		/* Read the reference monitor status */
		rc = zl3073x_read_u8(zldev, ZL_REG_REF_MON_STATUS(*ref),
				     &ref_status);
		if (rc)
			return rc;

		/* If the monitor indicates an error nothing is connected */
		if (ref_status != ZL_REF_MON_STATUS_OK)
			*ref = ZL3073X_DPLL_REF_NONE;
	}

	return 0;
}

/**
 * zl3073x_dpll_ref_prio_get - get priority for given input pin
 * @pin: pointer to pin
 * @prio: place to store priority
 *
 * Reads current priority for the given input pin and stores the value
 * to @prio.
 *
 * Return: 0 on success, <0 on error
 */
static int
zl3073x_dpll_ref_prio_get(struct zl3073x_dpll_pin *pin, u8 *prio)
{
	struct zl3073x_dpll *zldpll = pin_to_dpll(pin);
	struct zl3073x_dev *zldev = zldpll->mfd;
	u8 idx, ref_prio;
	int rc;

	/* Get prio index - (one prio value contains 4bits for P and N pins) */
	idx = zl3073x_dpll_pin_index_get(pin) / 2;

	guard(mutex)(&zldev->multiop_lock);

	/* Read DPLL configuration */
	rc = zl3073x_mb_op(zldev, ZL_REG_DPLL_MB_SEM, ZL_DPLL_MB_SEM_RD,
			   ZL_REG_DPLL_MB_MASK, BIT(zldpll->id));
	if (rc)
		return rc;

	/* Read reference priority */
	rc = zl3073x_read_u8(zldev, ZL_REG_DPLL_REF_PRIO(idx), &ref_prio);
	if (rc)
		return rc;

	/* Select nibble according pin type */
	if (zl3073x_dpll_is_p_pin(pin))
		*prio = FIELD_GET(ZL_DPLL_REF_PRIO_REF_P, ref_prio);
	else
		*prio = FIELD_GET(ZL_DPLL_REF_PRIO_REF_N, ref_prio);

	return rc;
}

static int
zl3073x_dpll_input_pin_state_on_dpll_get(const struct dpll_pin *dpll_pin,
					 void *pin_priv,
					 const struct dpll_device *dpll,
					 void *dpll_priv,
					 enum dpll_pin_state *state,
					 struct netlink_ext_ack *extack)
{
	struct zl3073x_dpll *zldpll = dpll_priv;
	struct zl3073x_dpll_pin *pin = pin_priv;
	struct zl3073x_dev *zldev = zldpll->mfd;
	u8 ref_id, ref_conn, ref_status;
	int rc;

	/* Get index of the pin */
	ref_id = zl3073x_dpll_pin_index_get(pin);

	/* Get currently connected reference */
	rc = zl3073x_dpll_connected_ref_get(zldpll, &ref_conn);
	if (rc)
		return rc;

	if (ref_id == ref_conn) {
		*state = DPLL_PIN_STATE_CONNECTED;
		return 0;
	}

	/* If the DPLL is running in automatic mode and the reference is
	 * selectable and its monitor does not report any error then report
	 * pin as selectable.
	 */
	if (zldpll->refsel_mode == ZL_DPLL_MODE_REFSEL_MODE_AUTO &&
	    pin->selectable) {
		/* Read reference monitor status */
		rc = zl3073x_read_u8(zldev, ZL_REG_REF_MON_STATUS(ref_id),
				     &ref_status);
		if (rc)
			return rc;

		/* If the monitor indicates errors report the reference
		 * as disconnected
		 */
		if (ref_status == ZL_REF_MON_STATUS_OK) {
			*state = DPLL_PIN_STATE_SELECTABLE;
			return 0;
		}
	}

	/* Otherwise report the pin as disconnected */
	*state = DPLL_PIN_STATE_DISCONNECTED;

	return 0;
}

static int
zl3073x_dpll_output_pin_state_on_dpll_get(const struct dpll_pin *dpll_pin,
					  void *pin_priv,
					  const struct dpll_device *dpll,
					  void *dpll_priv,
					  enum dpll_pin_state *state,
					  struct netlink_ext_ack *extack)
{
	/* If the output pin is registered then it is always connected */
	*state = DPLL_PIN_STATE_CONNECTED;

	return 0;
}

static int
zl3073x_dpll_lock_status_get(const struct dpll_device *dpll, void *dpll_priv,
			     enum dpll_lock_status *status,
			     enum dpll_lock_status_error *status_error,
			     struct netlink_ext_ack *extack)
{
	struct zl3073x_dpll *zldpll = dpll_priv;
	struct zl3073x_dev *zldev = zldpll->mfd;
	u8 mon_status;
	int rc;

	rc = zl3073x_read_u8(zldev, ZL_REG_DPLL_MON_STATUS(zldpll->id),
			     &mon_status);

	if (rc)
		return rc;

	if (FIELD_GET(ZL_DPLL_MON_STATUS_LOCK, mon_status)) {
		if (FIELD_GET(ZL_DPLL_MON_STATUS_HO_READY, mon_status))
			*status = DPLL_LOCK_STATUS_LOCKED_HO_ACQ;
		else
			*status = DPLL_LOCK_STATUS_LOCKED;
	} else if (FIELD_GET(ZL_DPLL_MON_STATUS_HO, mon_status)) {
		*status = DPLL_LOCK_STATUS_HOLDOVER;
	} else {
		*status = DPLL_LOCK_STATUS_UNLOCKED;
	}

	return rc;
}

static int
zl3073x_dpll_mode_get(const struct dpll_device *dpll, void *dpll_priv,
		      enum dpll_mode *mode, struct netlink_ext_ack *extack)
{
	struct zl3073x_dpll *zldpll = dpll_priv;

	switch (zldpll->refsel_mode) {
	case ZL_DPLL_MODE_REFSEL_MODE_FREERUN:
	case ZL_DPLL_MODE_REFSEL_MODE_HOLDOVER:
	case ZL_DPLL_MODE_REFSEL_MODE_NCO:
	case ZL_DPLL_MODE_REFSEL_MODE_REFLOCK:
		/* Use MANUAL for device FREERUN, HOLDOVER, NCO and
		 * REFLOCK modes
		 */
		*mode = DPLL_MODE_MANUAL;
		break;
	case ZL_DPLL_MODE_REFSEL_MODE_AUTO:
		/* Use AUTO for device AUTO mode */
		*mode = DPLL_MODE_AUTOMATIC;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct dpll_pin_ops zl3073x_dpll_input_pin_ops = {
	.direction_get = zl3073x_dpll_pin_direction_get,
	.state_on_dpll_get = zl3073x_dpll_input_pin_state_on_dpll_get,
};

static const struct dpll_pin_ops zl3073x_dpll_output_pin_ops = {
	.direction_get = zl3073x_dpll_pin_direction_get,
	.state_on_dpll_get = zl3073x_dpll_output_pin_state_on_dpll_get,
};

static const struct dpll_device_ops zl3073x_dpll_device_ops = {
	.lock_status_get = zl3073x_dpll_lock_status_get,
	.mode_get = zl3073x_dpll_mode_get,
};

/**
 * zl3073x_dpll_pin_info_package_label_set - generate package label for the pin
 * @pin: pointer to pin
 * @pin_info: pointer to pin info structure
 *
 * Generates package label string and stores it into pin info structure.
 *
 * Possible formats:
 * REF<n> - differential input reference
 * REF<n>P & REF<n>N - single-ended input reference (P or N pin)
 * OUT<n> - differential output
 * OUT<n>P & OUT<n>N - single-ended output (P or N pin)
 */
static void
zl3073x_dpll_pin_info_package_label_set(struct zl3073x_dpll_pin *pin,
					struct zl3073x_dpll_pin_info *pin_info)
{
	struct zl3073x_dev *zldev = pin_to_dev(pin);
	char suffix;
	u8 idx;

	suffix = zl3073x_dpll_is_p_pin(pin) ? 'P' : 'N';

	if (zl3073x_dpll_is_input_pin(pin)) {
		idx = zl3073x_dpll_pin_index_get(pin);

		if (zl3073x_input_is_diff(zldev, idx))
			/* For differential use REF<n> */
			snprintf(pin_info->package_label,
				 sizeof(pin_info->package_label),
				 "REF%u", idx / 2);
		else
			/* For single-ended use REF<n>P/N */
			snprintf(pin_info->package_label,
				 sizeof(pin_info->package_label),
				 "REF%u%c", idx / 2, suffix);
	} else {
		idx = zl3073x_dpll_output_pin_output_get(pin);

		switch (zl3073x_output_signal_format_get(zldev, idx)) {
		case ZL_OUTPUT_MODE_SIGNAL_FORMAT_LVDS:
		case ZL_OUTPUT_MODE_SIGNAL_FORMAT_DIFF:
		case ZL_OUTPUT_MODE_SIGNAL_FORMAT_LOWVCM:
			/* For differential use OUT<n> */
			snprintf(pin_info->package_label,
				 sizeof(pin_info->package_label), "OUT%u", idx);
			break;
		default:
			/* For single-ended use OUT<n>P/N */
			snprintf(pin_info->package_label,
				 sizeof(pin_info->package_label), "OUT%u%c",
				 idx, suffix);
			break;
		}
	}

	/* Set package_label pointer in DPLL core properties to generated
	 * string.
	 */
	pin_info->props.package_label = pin_info->package_label;
}

/**
 * zl3073x_dpll_pin_info_get - get pin info
 * @pin: pin whose info is returned
 *
 * The function allocates pin info structure, generates package label
 * string according pin type and its order number.
 *
 * Pointer that is returned by this function should be freed
 * using @zl3073x_dpll_pin_info_put().
 *
 * Returns:
 * * pointer to allocated pin info structure on success
 * * error pointer in case of error
 */
static struct zl3073x_dpll_pin_info *
zl3073x_dpll_pin_info_get(struct zl3073x_dpll_pin *pin)
{
	struct zl3073x_dpll_pin_info *pin_info;

	/* Allocate pin info structure */
	pin_info = kzalloc(sizeof(*pin_info), GFP_KERNEL);
	if (!pin_info)
		return ERR_PTR(-ENOMEM);

	/* Set pin type */
	if (zl3073x_dpll_is_input_pin(pin))
		pin_info->props.type = DPLL_PIN_TYPE_EXT;
	else
		pin_info->props.type = DPLL_PIN_TYPE_GNSS;

	pin_info->props.phase_range.min = S32_MIN;
	pin_info->props.phase_range.max = S32_MAX;

	/* Generate package label for the given pin */
	zl3073x_dpll_pin_info_package_label_set(pin, pin_info);

	return pin_info;
}

/**
 * zl3073x_dpll_pin_info_put - free pin info
 * @pin_info: pin info to free
 *
 * The function deallocates given pin info structure.
 */
static void
zl3073x_dpll_pin_info_put(struct zl3073x_dpll_pin_info *pin_info)
{
	/* Free the pin info structure itself */
	kfree(pin_info);
}

static int
zl3073x_dpll_pin_register(struct zl3073x_dpll_pin *pin)
{
	struct zl3073x_dpll *zldpll = pin_to_dpll(pin);
	struct zl3073x_dpll_pin_info *pin_info;
	const struct dpll_pin_ops *ops;
	int rc;

	/* Get pin info */
	pin_info = zl3073x_dpll_pin_info_get(pin);
	if (IS_ERR(pin_info))
		return PTR_ERR(pin_info);

	/* Create or get existing DPLL pin */
	pin->dpll_pin = dpll_pin_get(zldpll->mfd->clock_id, pin->index,
				     THIS_MODULE, &pin_info->props);
	if (IS_ERR(pin->dpll_pin)) {
		rc = PTR_ERR(pin->dpll_pin);
		goto err_pin_get;
	}

	if (zl3073x_dpll_is_input_pin(pin))
		ops = &zl3073x_dpll_input_pin_ops;
	else
		ops = &zl3073x_dpll_output_pin_ops;

	/* Register the pin */
	rc = dpll_pin_register(zldpll->dpll_dev, pin->dpll_pin, ops, pin);
	if (rc)
		goto err_register;

	/* Free pin info */
	zl3073x_dpll_pin_info_put(pin_info);

	return 0;

err_register:
	dpll_pin_put(pin->dpll_pin);
	pin->dpll_pin = NULL;
err_pin_get:
	zl3073x_dpll_pin_info_put(pin_info);

	return rc;
}

static void
zl3073x_dpll_pin_unregister(struct zl3073x_dpll_pin *pin)
{
	struct zl3073x_dpll *zldpll = pin_to_dpll(pin);
	const struct dpll_pin_ops *ops;

	if (IS_ERR_OR_NULL(pin->dpll_pin))
		return;

	if (zl3073x_dpll_is_input_pin(pin))
		ops = &zl3073x_dpll_input_pin_ops;
	else
		ops = &zl3073x_dpll_output_pin_ops;

	/* Unregister the pin */
	dpll_pin_unregister(zldpll->dpll_dev, pin->dpll_pin, ops, pin);

	dpll_pin_put(pin->dpll_pin);
	pin->dpll_pin = NULL;
}

static int
zl3073x_dpll_register_input_pin(struct zl3073x_dpll_pin *pin)
{
	struct zl3073x_dpll *zldpll = pin_to_dpll(pin);
	struct zl3073x_dev *zldev = zldpll->mfd;
	struct device *dev = zldpll->dev;
	u8 ref;
	int rc;

	/* Get index of the pin */
	ref = zl3073x_dpll_pin_index_get(pin);

	/* If the ref is differential then register only for the P-pin */
	if (zl3073x_input_is_diff(zldev, ref) && zl3073x_dpll_is_n_pin(pin)) {
		dev_dbg(dev, "INPUT%u is differential, skipping N-pin\n", ref);
		return 0;
	}

	/* If the ref is disabled then skip registration */
	if (!zl3073x_input_is_enabled(zldev, ref)) {
		dev_dbg(dev, "INPUT%u is disabled\n", ref);
		return 0;
	}

	rc = zl3073x_dpll_ref_prio_get(pin, &pin->prio);
	if (rc)
		return rc;

	if (pin->prio == ZL_DPLL_REF_PRIO_NONE) {
		/* Clamp priority to max value and make pin non-selectable */
		pin->prio = ZL_DPLL_REF_PRIO_MAX;
		pin->selectable = false;
	} else {
		/* Mark pin as selectable */
		pin->selectable = true;
	}

	/* Register the pin */
	return zl3073x_dpll_pin_register(pin);
}

static int
zl3073x_dpll_register_output_pin(struct zl3073x_dpll_pin *pin)
{
	struct zl3073x_dpll *zldpll = pin_to_dpll(pin);
	struct zl3073x_dev *zldev = zldpll->mfd;
	struct device *dev = zldpll->dev;
	u8 dpll, output, synth;

	/* Get output id for the pin and synth where it is connected to */
	output = zl3073x_dpll_output_pin_output_get(pin);
	synth = zl3073x_output_synth_get(zldev, output);

	/* Get DPLL channel the synth is associated with */
	dpll = zl3073x_synth_dpll_get(zldev, synth);

	/* If the output's synth is connected to different DPLL channel
	 * then skip registration.
	 */
	if (dpll != zldpll->id) {
		dev_dbg(dev, "OUTPUT%u is driven by different DPLL\n", output);
		return 0;
	}

	/* If the output is disabled then skip registration */
	if (!zl3073x_output_is_enabled(zldev, output)) {
		dev_dbg(dev, "OUTPUT%u is disabled\n", output);
		return 0;
	}

	/* Check ouput's signal format */
	switch (zldev->output[output].signal_format) {
	case ZL_OUTPUT_MODE_SIGNAL_FORMAT_DISABLED:
		/* Output is disabled, nothing to register */
		dev_dbg(dev, "OUTPUT%u is disabled by signal format\n", output);
		return 0;

	case ZL_OUTPUT_MODE_SIGNAL_FORMAT_LVDS:
	case ZL_OUTPUT_MODE_SIGNAL_FORMAT_DIFF:
	case ZL_OUTPUT_MODE_SIGNAL_FORMAT_LOWVCM:
		/* Output is differential, skip registration for N-pin */
		if (zl3073x_dpll_is_n_pin(pin)) {
			dev_dbg(dev,
				"OUTPUT%u is differential, skipping N-pin\n",
				output);
			return 0;
		}
		break;

	case ZL_OUTPUT_MODE_SIGNAL_FORMAT_2:
	case ZL_OUTPUT_MODE_SIGNAL_FORMAT_2_INV:
	case ZL_OUTPUT_MODE_SIGNAL_FORMAT_2_NDIV:
	case ZL_OUTPUT_MODE_SIGNAL_FORMAT_2_NDIV_INV:
		/* Output is two single ended outputs, continue with
		 * registration.
		 */
		break;

	case ZL_OUTPUT_MODE_SIGNAL_FORMAT_1P:
		/* Output is one single ended P-pin output */
		if (zl3073x_dpll_is_n_pin(pin)) {
			dev_dbg(dev, "OUTPUT%u is P-pin only, skipping N-pin\n",
				output);
			return 0;
		}
		break;
	case ZL_OUTPUT_MODE_SIGNAL_FORMAT_1N:
		/* Output is one single ended N-pin output */
		if (zl3073x_dpll_is_p_pin(pin)) {
			dev_dbg(dev, "OUTPUT%u is N-pin only, skipping P-pin\n",
				output);
			return 0;
		}
		break;
	default:
		dev_warn(dev, "Unknown output mode signal format: %u\n",
			 zldev->output[output].signal_format);
		return 0;
	}

	/* Register the pin */
	return zl3073x_dpll_pin_register(pin);
}

static int
zl3073x_dpll_register_pins(struct zl3073x_dpll *zldpll)
{
	int i, rc;

	for (i = 0; i < ZL3073X_NUM_PINS; i++) {
		struct zl3073x_dpll_pin *pin = &zldpll->pins[i];

		pin->index = i;

		if (zl3073x_dpll_is_input_pin(pin))
			rc = zl3073x_dpll_register_input_pin(pin);
		else
			rc = zl3073x_dpll_register_output_pin(pin);

		if (rc)
			goto err_register;
	}

	return 0;

err_register:
	while (i--)
		zl3073x_dpll_pin_unregister(&zldpll->pins[i]);

	return rc;
}

static void
zl3073x_dpll_unregister_pins(struct zl3073x_dpll *zldpll)
{
	int i;

	for (i = 0; i < ZL3073X_NUM_PINS; i++)
		zl3073x_dpll_pin_unregister(&zldpll->pins[i]);
}

static enum dpll_type
zl3073x_dpll_type_get(struct zl3073x_dpll *zldpll)
{
	const char *types[ZL3073X_MAX_CHANNELS];
	enum dpll_type type;
	int rc;

	/* Set default */
	type = DPLL_TYPE_PPS;

	/* Read dpll types property from firmware */
	rc = device_property_read_string_array(zldpll->mfd->dev, "dpll-types",
					       types, ARRAY_SIZE(types));

	/* It is not present or property does not exist, use default */
	if (rc <= zldpll->id)
		return type;

	if (!strcmp(types[zldpll->id], "pps"))
		type = DPLL_TYPE_PPS;
	else if (!strcmp(types[zldpll->id], "eec"))
		type = DPLL_TYPE_EEC;
	else
		dev_info(zldpll->mfd->dev,
			 "Unknown dpll type '%s', using default\n",
			 types[zldpll->id]);

	return type;
}

static int
zl3073x_dpll_register(struct zl3073x_dpll *zldpll)
{
	struct zl3073x_dev *zldev = zldpll->mfd;
	u8 dpll_mode_refsel;
	int rc;

	/* Read DPLL mode and forcibly selected reference */
	rc = zl3073x_read_u8(zldev, ZL_REG_DPLL_MODE_REFSEL(zldpll->id),
			     &dpll_mode_refsel);
	if (rc)
		return rc;

	/* Extract mode and selected input reference */
	zldpll->refsel_mode = FIELD_GET(ZL_DPLL_MODE_REFSEL_MODE,
					dpll_mode_refsel);
	zldpll->forced_ref = FIELD_GET(ZL_DPLL_MODE_REFSEL_REF,
				       dpll_mode_refsel);

	zldpll->dpll_dev = dpll_device_get(zldev->clock_id, zldpll->id,
					   THIS_MODULE);
	if (IS_ERR(zldpll->dpll_dev))
		return PTR_ERR(zldpll->dpll_dev);

	rc = dpll_device_register(zldpll->dpll_dev,
				  zl3073x_dpll_type_get(zldpll),
				  &zl3073x_dpll_device_ops, zldpll);
	if (rc) {
		dpll_device_put(zldpll->dpll_dev);
		zldpll->dpll_dev = NULL;
	}

	return rc;
}

static void
zl3073x_dpll_unregister(struct zl3073x_dpll *zldpll)
{
	if (IS_ERR_OR_NULL(zldpll->dpll_dev))
		return;

	dpll_device_unregister(zldpll->dpll_dev, &zl3073x_dpll_device_ops,
			       zldpll);
	dpll_device_put(zldpll->dpll_dev);
	zldpll->dpll_dev = NULL;
}

static int
zl3073x_dpll_init(struct zl3073x_dpll *zldpll)
{
	int rc;

	rc = zl3073x_dpll_register(zldpll);
	if (rc)
		return rc;

	rc = zl3073x_dpll_register_pins(zldpll);
	if (rc)
		zl3073x_dpll_unregister(zldpll);

	return rc;
}

static void
zl3073x_dpll_periodic_work(struct kthread_work *work)
{
	struct zl3073x_dpll *zldpll = container_of(work, struct zl3073x_dpll,
						   work.work);
	enum dpll_lock_status lock_status;
	struct device *dev = zldpll->dev;
	int i, rc;

	/* Get current lock status for the DPLL */
	rc = zl3073x_dpll_lock_status_get(zldpll->dpll_dev, zldpll,
					  &lock_status, NULL, NULL);
	if (rc) {
		dev_err(dev, "Failed to get DPLL%u lock status: %pe\n",
			zldpll->id, ERR_PTR(rc));
		goto out;
	}

	/* If lock status was changed then notify DPLL core */
	if (zldpll->lock_status != lock_status) {
		zldpll->lock_status = lock_status;
		dpll_device_change_ntf(zldpll->dpll_dev);
	}

	for (i = 0; i < ZL3073X_NUM_INPUT_PINS; i++) {
		struct zl3073x_dpll_pin *pin;
		enum dpll_pin_state state;
		u8 index;

		/* Input pins starts are stored after output pins */
		pin = &zldpll->pins[ZL3073X_NUM_OUTPUT_PINS + i];

		/* Skip non-registered pins */
		if (!pin->dpll_pin)
			continue;

		index = zl3073x_dpll_pin_index_get(pin);

		rc = zl3073x_dpll_input_pin_state_on_dpll_get(pin->dpll_pin,
							      pin,
							      zldpll->dpll_dev,
							      zldpll, &state,
							      NULL);
		if (rc) {
			dev_err(dev,
				"Failed to get INPUT%u on DPLL%u state: %pe\n",
				index, zldpll->id, ERR_PTR(rc));
			goto out;
		}

		if (state != pin->pin_state) {
			pin->pin_state = state;
			dpll_pin_change_ntf(pin->dpll_pin);
		}
	}

	/* Output pins change checks are not necessary because output states
	 * are constant.
	 */

out:
	/* Run twice a second */
	kthread_queue_delayed_work(zldpll->kworker, &zldpll->work,
				   msecs_to_jiffies(500));
}

static int
zl3073x_dpll_init_worker(struct zl3073x_dpll *zldpll)
{
	struct kthread_worker *kworker;

	kthread_init_delayed_work(&zldpll->work, zl3073x_dpll_periodic_work);
	kworker = kthread_run_worker(0, "zl3073x-%s", dev_name(zldpll->dev));
	if (IS_ERR(kworker))
		return PTR_ERR(kworker);

	zldpll->kworker = kworker;
	kthread_queue_delayed_work(zldpll->kworker, &zldpll->work, 0);

	return 0;
}

static int
zl3073x_dpll_init_fine_phase_adjust(struct zl3073x_dpll *zldpll)
{
	struct zl3073x_dev *zldev = zldpll->mfd;
	int rc;

	rc = zl3073x_write_u8(zldev, ZL_REG_SYNTH_PHASE_SHIFT_MASK, 0x1f);
	if (rc)
		return rc;

	rc = zl3073x_write_u8(zldev, ZL_REG_SYNTH_PHASE_SHIFT_INTVL, 0x01);
	if (rc)
		return rc;

	rc = zl3073x_write_u16(zldev, ZL_REG_SYNTH_PHASE_SHIFT_DATA, 0xffff);
	if (rc)
		return rc;

	rc = zl3073x_write_u8(zldev, ZL_REG_SYNTH_PHASE_SHIFT_CTRL, 0x01);
	if (rc)
		return rc;

	return rc;
}

static int
zl3073x_dpll_probe(struct platform_device *pdev)
{
	struct zl3073x_dpll *zldpll;
	struct zl3073x_pdata *pdata;
	int rc;

	zldpll = devm_kzalloc(&pdev->dev, sizeof(*zldpll), GFP_KERNEL);
	if (!zldpll)
		return -ENOMEM;

	zldpll->dev = &pdev->dev;
	zldpll->mfd = dev_get_drvdata(pdev->dev.parent);

	pdata = pdev->dev.platform_data;
	zldpll->id = pdata->channel;

	rc = zl3073x_dpll_init(zldpll);
	if (rc)
		return rc;

	rc = zl3073x_dpll_init_worker(zldpll);
	if (rc)
		goto err_init_worker;

	platform_set_drvdata(pdev, zldpll);

	/* Initial firmware fine phase correction */
	rc = zl3073x_dpll_init_fine_phase_adjust(zldpll);
	if (rc)
		goto err_init_phase_adjust;

	return rc;

err_init_phase_adjust:
	kthread_cancel_delayed_work_sync(&zldpll->work);
	kthread_destroy_worker(zldpll->kworker);
err_init_worker:
	zl3073x_dpll_unregister_pins(zldpll);
	zl3073x_dpll_unregister(zldpll);

	return rc;
}

static void
zl3073x_dpll_remove(struct platform_device *pdev)
{
	struct zl3073x_dpll *zldpll = platform_get_drvdata(pdev);

	/* Stop worker */
	kthread_cancel_delayed_work_sync(&zldpll->work);
	kthread_destroy_worker(zldpll->kworker);

	/* Unregister all pins and dpll */
	zl3073x_dpll_unregister_pins(zldpll);
	zl3073x_dpll_unregister(zldpll);
}

static const struct platform_device_id zl3073x_dpll_platform_id[] = {
	{ "zl3073x-dpll", },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(platform, zl3073x_dpll_platform_id);

static struct platform_driver zl3073x_dpll_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
	},
	.probe = zl3073x_dpll_probe,
	.remove	= zl3073x_dpll_remove,
	.id_table = zl3073x_dpll_platform_id,
};

module_platform_driver(zl3073x_dpll_driver);

MODULE_AUTHOR("Ivan Vecera <ivecera@redhat.com>");
MODULE_AUTHOR("Tariq Haddad <tariq.haddad@microchip.com>");
MODULE_DESCRIPTION("Microchip ZL3073x DPLL driver");
MODULE_IMPORT_NS("ZL3073X");
MODULE_LICENSE("GPL");
