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
#include <linux/math64.h>
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
 * @fwnode: pin firmware node
 */
struct zl3073x_dpll_pin_info {
	struct dpll_pin_properties	props;
	char				package_label[8];
	struct fwnode_handle		*fwnode;
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
 * zl3073x_dpll_input_ref_frequency_factorize - factorize given frequency
 * @freq: input frequency
 * @base: base frequency
 * @mult: multiplier
 *
 * Checks if the given frequency can be factorized using one of the
 * supported base frequencies. If so the base frequency and multiplier
 * are stored into appropriate parameters if they are not NULL.
 *
 * Return: 0 on success, -EINVAL if the frequency cannot be factorized
 */
static int
zl3073x_dpll_input_ref_frequency_factorize(u32 freq, u16 *base, u16 *mult)
{
	static const u16 base_freqs[] = {
		1, 2, 4, 5, 8, 10, 16, 20, 25, 32, 40, 50, 64, 80, 100, 125,
		128, 160, 200, 250, 256, 320, 400, 500, 625, 640, 800, 1000,
		1250, 1280, 1600, 2000, 2500, 3125, 3200, 4000, 5000, 6250,
		6400, 8000, 10000, 12500, 15625, 16000, 20000, 25000, 31250,
		32000, 40000, 50000, 62500,
	};
	u32 div;
	int i;

	for (i = 0; i < ARRAY_SIZE(base_freqs); i++) {
		div = freq / base_freqs[i];

		if (div <= U16_MAX && freq % base_freqs[i] == 0) {
			if (base)
				*base = base_freqs[i];
			if (mult)
				*mult = div;

			return 0;
		}
	}

	return -EINVAL;
}

/**
 * zl3073x_dpll_input_ref_frequency_get - get input reference frequency
 * @zldpll: pointer to zl3073x_dpll
 * @ref_id: reference id
 * @frequency: pointer to variable to store frequency
 *
 * Reads frequency of given input reference.
 *
 * Return: 0 on success, <0 on error
 */
static int
zl3073x_dpll_input_ref_frequency_get(struct zl3073x_dpll *zldpll, u8 ref_id,
				     u32 *frequency)
{
	struct zl3073x_dev *zldev = zldpll->mfd;
	u16 base, mult, num, denom;
	int rc;

	guard(mutex)(&zldev->multiop_lock);

	/* Read reference configuration */
	rc = zl3073x_mb_op(zldev, ZL_REG_REF_MB_SEM, ZL_REF_MB_SEM_RD,
			   ZL_REG_REF_MB_MASK, BIT(ref_id));
	if (rc)
		return rc;

	/* Read registers to compute resulting frequency */
	rc = zl3073x_read_u16(zldev, ZL_REG_REF_FREQ_BASE, &base);
	if (rc)
		return rc;
	rc = zl3073x_read_u16(zldev, ZL_REG_REF_FREQ_MULT, &mult);
	if (rc)
		return rc;
	rc = zl3073x_read_u16(zldev, ZL_REG_REF_RATIO_M, &num);
	if (rc)
		return rc;
	rc = zl3073x_read_u16(zldev, ZL_REG_REF_RATIO_N, &denom);
	if (rc)
		return rc;

	/* Sanity check that HW has not returned zero denominator */
	if (!denom) {
		dev_err(zldpll->dev,
			"Zero divisor for ref %u frequency got from device\n",
			ref_id);
		return -EINVAL;
	}

	/* Compute the frequency */
	*frequency = base * mult * num / denom;

	return rc;
}

static int
zl3073x_dpll_input_pin_frequency_get(const struct dpll_pin *dpll_pin,
				     void *pin_priv,
				     const struct dpll_device *dpll,
				     void *dpll_priv, u64 *frequency,
				     struct netlink_ext_ack *extack)
{
	struct zl3073x_dpll *zldpll = dpll_priv;
	struct zl3073x_dpll_pin *pin = pin_priv;
	u32 ref_freq;
	u8 ref_id;
	int rc;

	/* Get index of the pin */
	ref_id = zl3073x_dpll_pin_index_get(pin);

	/* Read and return ref frequency */
	rc = zl3073x_dpll_input_ref_frequency_get(zldpll, ref_id, &ref_freq);
	if (!rc)
		*frequency = ref_freq;

	return rc;
}

static int
zl3073x_dpll_input_pin_frequency_set(const struct dpll_pin *dpll_pin,
				     void *pin_priv,
				     const struct dpll_device *dpll,
				     void *dpll_priv, u64 frequency,
				     struct netlink_ext_ack *extack)
{
	struct zl3073x_dpll *zldpll = dpll_priv;
	struct zl3073x_dev *zldev = zldpll->mfd;
	struct zl3073x_dpll_pin *pin = pin_priv;
	u16 base, mult;
	u8 ref_id;
	int rc;

	/* Get base frequency and multiplier for the requested frequency */
	rc = zl3073x_dpll_input_ref_frequency_factorize(frequency, &base,
							&mult);
	if (rc)
		return rc;

	/* Get index of the pin */
	ref_id = zl3073x_dpll_pin_index_get(pin);

	guard(mutex)(&zldev->multiop_lock);

	/* Load reference configuration */
	rc = zl3073x_mb_op(zldev, ZL_REG_REF_MB_SEM, ZL_REF_MB_SEM_RD,
			   ZL_REG_REF_MB_MASK, BIT(ref_id));

	/* Update base frequency, multiplier, numerator & denominator */
	rc = zl3073x_write_u16(zldev, ZL_REG_REF_FREQ_BASE, base);
	if (rc)
		return rc;
	rc = zl3073x_write_u16(zldev, ZL_REG_REF_FREQ_MULT, mult);
	if (rc)
		return rc;
	rc = zl3073x_write_u16(zldev, ZL_REG_REF_RATIO_M, 1);
	if (rc)
		return rc;
	rc = zl3073x_write_u16(zldev, ZL_REG_REF_RATIO_N, 1);
	if (rc)
		return rc;

	/* Commit reference configuration */
	return zl3073x_mb_op(zldev, ZL_REG_REF_MB_SEM, ZL_REF_MB_SEM_WR,
			     ZL_REG_REF_MB_MASK, BIT(ref_id));
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
 * zl3073x_dpll_selected_ref_set - select reference in manual mode
 * @zldpll: pointer to zl3073x_dpll
 * @ref: place to store selected reference
 *
 * Selects the given reference for the DPLL channel it should be
 * locked to.
 *
 * Return: 0 on success, <0 on error
 */
static int
zl3073x_dpll_selected_ref_set(struct zl3073x_dpll *zldpll, u8 ref)
{
	struct zl3073x_dev *zldev = zldpll->mfd;
	u8 mode, mode_refsel;
	int rc;

	mode = zldpll->refsel_mode;

	switch (mode) {
	case ZL_DPLL_MODE_REFSEL_MODE_REFLOCK: /* Manual mode with ref selected */
		if (ref == ZL3073X_DPLL_REF_NONE) {
			switch (zldpll->lock_status) {
			case DPLL_LOCK_STATUS_LOCKED_HO_ACQ:
			case DPLL_LOCK_STATUS_HOLDOVER:
				/* Switch to forced holdover */
				mode = ZL_DPLL_MODE_REFSEL_MODE_HOLDOVER;
				break;
			default:
				/* Switch to freerun */
				mode = ZL_DPLL_MODE_REFSEL_MODE_FREERUN;
				break;
			}
			/* Keep selected reference */
			ref = zldpll->forced_ref;
		} else if (ref == zldpll->forced_ref) {
			/* No register update - same mode and same ref */
			return 0;
		}
		break;
	case ZL_DPLL_MODE_REFSEL_MODE_FREERUN: /* Manual mode without no ref */
	case ZL_DPLL_MODE_REFSEL_MODE_HOLDOVER:
		if (ref == ZL3073X_DPLL_REF_NONE)
			/* No register update - keep current mode */
			return 0;

		/* Switch to reflock mode and update ref selection */
		mode = ZL_DPLL_MODE_REFSEL_MODE_REFLOCK;
		break;
	default:
		/* For other modes like automatic or NCO ref cannot be selected
		 * manually
		 */
		return -EOPNOTSUPP;
	}

	/* Build mode_refsel value */
	mode_refsel = FIELD_PREP(ZL_DPLL_MODE_REFSEL_MODE, mode) |
		      FIELD_PREP(ZL_DPLL_MODE_REFSEL_REF, ref);

	/* Update dpll_mode_refsel register */
	rc = zl3073x_write_u8(zldev, ZL_REG_DPLL_MODE_REFSEL(zldpll->id),
			      mode_refsel);
	if (rc)
		return rc;

	/* Store new mode and forced reference */
	zldpll->refsel_mode = mode;
	zldpll->forced_ref = ref;

	return rc;
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
zl3073x_dpll_ref_prio_set(struct zl3073x_dpll_pin *pin, u8 prio)
{
	struct zl3073x_dpll *zldpll = pin_to_dpll(pin);
	struct zl3073x_dev *zldev = zldpll->mfd;
	u8 idx, ref_prio;
	int rc;

	/* Get prio index - (one prio value contains 4bits for P and N pins) */
	idx = zl3073x_dpll_pin_index_get(pin) / 2;

	guard(mutex)(&zldev->multiop_lock);

	/* Read DPLL configuration into mailbox */
	rc = zl3073x_mb_op(zldev, ZL_REG_DPLL_MB_SEM, ZL_DPLL_MB_SEM_RD,
			   ZL_REG_DPLL_MB_MASK, BIT(zldpll->id));
	if (rc)
		return rc;

	/* Read reference priority */
	rc = zl3073x_read_u8(zldev, ZL_REG_DPLL_REF_PRIO(idx), &ref_prio);
	if (rc)
		return rc;

	/* Update nibble according pin type */
	if (zl3073x_dpll_is_p_pin(pin)) {
		ref_prio &= ~ZL_DPLL_REF_PRIO_REF_P;
		ref_prio |= FIELD_PREP(ZL_DPLL_REF_PRIO_REF_P, prio);
	} else {
		ref_prio &= ~ZL_DPLL_REF_PRIO_REF_N;
		ref_prio |= FIELD_PREP(ZL_DPLL_REF_PRIO_REF_N, prio);
	}

	/* Update reference priority */
	rc = zl3073x_write_u8(zldev, ZL_REG_DPLL_REF_PRIO(idx), ref_prio);
	if (rc)
		return rc;

	/* Commit configuration */
	return zl3073x_mb_op(zldev, ZL_REG_DPLL_MB_SEM, ZL_DPLL_MB_SEM_WR,
			     ZL_REG_DPLL_MB_MASK, BIT(zldpll->id));
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
zl3073x_dpll_input_pin_state_on_dpll_set(const struct dpll_pin *dpll_pin,
					 void *pin_priv,
					 const struct dpll_device *dpll,
					 void *dpll_priv,
					 enum dpll_pin_state state,
					 struct netlink_ext_ack *extack)
{
	struct zl3073x_dpll *zldpll = dpll_priv;
	struct zl3073x_dpll_pin *pin = pin_priv;
	u8 new_ref;
	int rc;

	switch (zldpll->refsel_mode) {
	case ZL_DPLL_MODE_REFSEL_MODE_REFLOCK:
	case ZL_DPLL_MODE_REFSEL_MODE_FREERUN:
	case ZL_DPLL_MODE_REFSEL_MODE_HOLDOVER:
		if (state == DPLL_PIN_STATE_CONNECTED) {
			/* Choose the pin as new selected reference */
			new_ref = zl3073x_dpll_pin_index_get(pin);
		} else if (state == DPLL_PIN_STATE_DISCONNECTED) {
			/* No reference */
			new_ref = ZL3073X_DPLL_REF_NONE;
		} else {
			NL_SET_ERR_MSG_MOD(extack,
					   "Invalid pin state for manual mode");
			return -EINVAL;
		}

		rc = zl3073x_dpll_selected_ref_set(zldpll, new_ref);
		break;

	case ZL_DPLL_MODE_REFSEL_MODE_AUTO:
		if (state == DPLL_PIN_STATE_SELECTABLE) {
			if (pin->selectable)
				return 0; /* Pin is already selectable */

			/* Restore pin priority in HW */
			rc = zl3073x_dpll_ref_prio_set(pin, pin->prio);
			if (rc)
				return rc;

			/* Mark pin as selectable */
			pin->selectable = true;
		} else if (state == DPLL_PIN_STATE_DISCONNECTED) {
			if (!pin->selectable)
				return 0; /* Pin is already disconnected */

			/* Set pin priority to none in HW */
			rc = zl3073x_dpll_ref_prio_set(pin,
						       ZL_DPLL_REF_PRIO_NONE);
			if (rc)
				return rc;

			/* Mark pin as non-selectable */
			pin->selectable = false;
		} else {
			NL_SET_ERR_MSG(extack,
				       "Invalid pin state for automatic mode");
			return -EINVAL;
		}
		break;

	default:
		/* In other modes we cannot change input reference */
		NL_SET_ERR_MSG(extack,
			       "Pin state cannot be changed in current mode");
		rc = -EOPNOTSUPP;
		break;
	}

	return rc;
}

static int
zl3073x_dpll_input_pin_prio_get(const struct dpll_pin *dpll_pin, void *pin_priv,
				const struct dpll_device *dpll, void *dpll_priv,
				u32 *prio, struct netlink_ext_ack *extack)
{
	struct zl3073x_dpll_pin *pin = pin_priv;

	*prio = pin->prio;

	return 0;
}

static int
zl3073x_dpll_input_pin_prio_set(const struct dpll_pin *dpll_pin, void *pin_priv,
				const struct dpll_device *dpll, void *dpll_priv,
				u32 prio, struct netlink_ext_ack *extack)
{
	struct zl3073x_dpll_pin *pin = pin_priv;
	int rc;

	if (prio > ZL_DPLL_REF_PRIO_MAX)
		return -EINVAL;

	/* If the pin is selectable then update HW registers */
	if (pin->selectable) {
		rc = zl3073x_dpll_ref_prio_set(pin, prio);
		if (rc)
			return rc;
	}

	/* Save priority */
	pin->prio = prio;

	return 0;
}

static u8
zl3073x_dpll_pin_synth_get(struct zl3073x_dpll_pin *pin)
{
	u8 output = zl3073x_dpll_output_pin_output_get(pin);

	return zl3073x_output_synth_get(pin_to_dev(pin), output);
}

static int
zl3073x_dpll_output_pin_frequency_get(const struct dpll_pin *dpll_pin,
				      void *pin_priv,
				      const struct dpll_device *dpll,
				      void *dpll_priv, u64 *frequency,
				      struct netlink_ext_ack *extack)
{
	struct zl3073x_dpll *zldpll = dpll_priv;
	struct zl3073x_dev *zldev = zldpll->mfd;
	struct zl3073x_dpll_pin *pin = pin_priv;
	struct device *dev = zldpll->dev;
	u8 output, signal_format, synth;
	u32 output_div, synth_freq;
	int rc;

	output = zl3073x_dpll_output_pin_output_get(pin);
	synth = zl3073x_dpll_pin_synth_get(pin);
	synth_freq = zl3073x_synth_freq_get(zldev, synth);

	guard(mutex)(&zldev->multiop_lock);

	/* Read output configuration into mailbox */
	rc = zl3073x_mb_op(zldev, ZL_REG_OUTPUT_MB_SEM, ZL_OUTPUT_MB_SEM_RD,
			   ZL_REG_OUTPUT_MB_MASK, BIT(output));
	if (rc)
		return rc;

	rc = zl3073x_read_u32(zldev, ZL_REG_OUTPUT_DIV, &output_div);
	if (rc)
		return rc;

	/* Check output divisor for zero */
	if (!output_div) {
		dev_err(dev, "Zero divisor for output %u got from device\n",
			output);
		return -EINVAL;
	}

	/* Read used signal format for the given output */
	signal_format = zl3073x_output_signal_format_get(zldev, output);

	switch (signal_format) {
	case ZL_OUTPUT_MODE_SIGNAL_FORMAT_2_NDIV:
	case ZL_OUTPUT_MODE_SIGNAL_FORMAT_2_NDIV_INV:
		/* In case of divided format we have to distiguish between
		 * given output pin type.
		 */
		if (zl3073x_dpll_is_p_pin(pin)) {
			/* For P-pin the resulting frequency is computed as
			 * simple division of synth frequency and output
			 * divisor.
			 */
			*frequency = synth_freq / output_div;
		} else {
			/* For N-pin we have to divide additionally by
			 * divisor stored in esync_period output mailbox
			 * register that is used as N-pin divisor for these
			 * modes.
			 */
			u32 ndiv;

			rc = zl3073x_read_u32(zldev, ZL_REG_OUTPUT_ESYNC_PERIOD,
					      &ndiv);
			if (rc)
				return rc;

			/* Check N-pin divisor for zero */
			if (!ndiv) {
				dev_err(dev,
					"Zero N-pin divisor for output %u got from device\n",
					output);
				return -EINVAL;
			}

			/* Compute final divisor for N-pin */
			*frequency = synth_freq / output_div / ndiv;
		}
		break;
	default:
		/* In other modes the resulting frequency is computed as
		 * division of synth frequency and output divisor.
		 */
		*frequency = synth_freq / output_div;
		break;
	}

	return rc;
}

static int
zl3073x_dpll_output_pin_frequency_set(const struct dpll_pin *dpll_pin,
				      void *pin_priv,
				      const struct dpll_device *dpll,
				      void *dpll_priv, u64 frequency,
				      struct netlink_ext_ack *extack)
{
	struct zl3073x_dpll *zldpll = dpll_priv;
	struct zl3073x_dev *zldev = zldpll->mfd;
	struct zl3073x_dpll_pin *pin = pin_priv;
	struct device *dev = zldpll->dev;
	u32 output_n_freq, output_p_freq;
	u8 output, signal_format, synth;
	u32 cur_div, new_div, ndiv;
	u32 synth_freq;
	int rc;

	output = zl3073x_dpll_output_pin_output_get(pin);
	synth = zl3073x_dpll_pin_synth_get(pin);
	synth_freq = zl3073x_synth_freq_get(zldev, synth);

	/* Check the requested frequency divides synth frequency without
	 * remainder.
	 */
	if (synth_freq % (u32)frequency) {
		dev_err(dev, "The requested frequency must divide %u Hz\n",
			synth_freq);
		return -EINVAL;
	}
	new_div = synth_freq / (u32)frequency;

	/* Get used signal format for the given output */
	signal_format = zl3073x_output_signal_format_get(zldev, output);

	guard(mutex)(&zldev->multiop_lock);

	/* Load output configuration */
	rc = zl3073x_mb_op(zldev, ZL_REG_OUTPUT_MB_SEM, ZL_OUTPUT_MB_SEM_RD,
			   ZL_REG_OUTPUT_MB_MASK, BIT(output));
	if (rc)
		return rc;

	/* Check signal format */
	if (signal_format != ZL_OUTPUT_MODE_SIGNAL_FORMAT_2_NDIV &&
	    signal_format != ZL_OUTPUT_MODE_SIGNAL_FORMAT_2_NDIV_INV) {
		/* For non N-divided signal formats the frequency is computed
		 * as division of synth frequency and output divisor.
		 */
		rc = zl3073x_write_u32(zldev, ZL_REG_OUTPUT_DIV, new_div);
		if (rc)
			return rc;

		/* For 50/50 duty cycle the divisor is equal to width */
		rc = zl3073x_write_u32(zldev, ZL_REG_OUTPUT_WIDTH, new_div);
		if (rc)
			return rc;

		/* Commit output configuration */
		return zl3073x_mb_op(zldev,
				     ZL_REG_OUTPUT_MB_SEM, ZL_OUTPUT_MB_SEM_WR,
				     ZL_REG_OUTPUT_MB_MASK, BIT(output));
	}

	/* For N-divided signal format get current divisor */
	rc = zl3073x_read_u32(zldev, ZL_REG_OUTPUT_DIV, &cur_div);
	if (rc)
		return rc;

	/* Check output divisor for zero */
	if (!cur_div) {
		dev_err(dev, "Zero divisor for output %u got from device\n",
			output);
		return -EINVAL;
	}

	/* Get N-pin divisor (shares the same register with esync */
	rc = zl3073x_read_u32(zldev, ZL_REG_OUTPUT_ESYNC_PERIOD, &ndiv);
	if (rc)
		return rc;

	/* Check N-pin divisor for zero */
	if (!ndiv) {
		dev_err(dev,
			"Zero N-pin divisor for output %u got from device\n",
			output);
		return -EINVAL;
	}

	/* Compute current output frequency for P-pin */
	output_p_freq = synth_freq / cur_div;

	/* Compute current N-pin frequency */
	output_n_freq = output_p_freq / ndiv;

	if (zl3073x_dpll_is_p_pin(pin)) {
		/* We are going to change output frequency for P-pin but
		 * if the requested frequency is less than current N-pin
		 * frequency then indicate a failure as we are not able
		 * to compute N-pin divisor to keep its frequency unchanged.
		 */
		if (frequency <= output_n_freq)
			return -EINVAL;

		/* Update the output divisor */
		rc = zl3073x_write_u32(zldev, ZL_REG_OUTPUT_DIV, new_div);
		if (rc)
			return rc;

		/* For 50/50 duty cycle the divisor is equal to width */
		rc = zl3073x_write_u32(zldev, ZL_REG_OUTPUT_WIDTH, new_div);
		if (rc)
			return rc;

		/* Compute new divisor for N-pin */
		ndiv = (u32)frequency / output_n_freq;
	} else {
		/* We are going to change frequency of N-pin but if
		 * the requested freq is greater or equal than freq of P-pin
		 * in the output pair we cannot compute divisor for the N-pin.
		 * In this case indicate a failure.
		 */
		if (output_p_freq <= frequency)
			return -EINVAL;

		/* Compute new divisor for N-pin */
		ndiv = output_p_freq / (u32)frequency;
	}

	/* Update divisor for the N-pin */
	rc = zl3073x_write_u32(zldev, ZL_REG_OUTPUT_ESYNC_PERIOD, ndiv);
	if (rc)
		return rc;

	/* For 50/50 duty cycle the divisor is equal to width */
	rc = zl3073x_write_u32(zldev, ZL_REG_OUTPUT_ESYNC_WIDTH, ndiv);
	if (rc)
		return rc;

	/* Commit output configuration */
	return zl3073x_mb_op(zldev, ZL_REG_OUTPUT_MB_SEM, ZL_OUTPUT_MB_SEM_WR,
			     ZL_REG_OUTPUT_MB_MASK, BIT(output));

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
	.frequency_get = zl3073x_dpll_input_pin_frequency_get,
	.frequency_set = zl3073x_dpll_input_pin_frequency_set,
	.prio_get = zl3073x_dpll_input_pin_prio_get,
	.prio_set = zl3073x_dpll_input_pin_prio_set,
	.state_on_dpll_get = zl3073x_dpll_input_pin_state_on_dpll_get,
	.state_on_dpll_set = zl3073x_dpll_input_pin_state_on_dpll_set,
};

static const struct dpll_pin_ops zl3073x_dpll_output_pin_ops = {
	.direction_get = zl3073x_dpll_pin_direction_get,
	.frequency_get = zl3073x_dpll_output_pin_frequency_get,
	.frequency_set = zl3073x_dpll_output_pin_frequency_set,
	.state_on_dpll_get = zl3073x_dpll_output_pin_state_on_dpll_get,
};

static const struct dpll_device_ops zl3073x_dpll_device_ops = {
	.lock_status_get = zl3073x_dpll_lock_status_get,
	.mode_get = zl3073x_dpll_mode_get,
};

/**
 * zl3073x_dpll_pin_fwnode_get - get fwnode for given pin
 * @pin: pointer to pin structure
 *
 * The caller is responsible for calling fwnode_handle_put() on the returned
 * fwnode pointer.
 *
 * Return: firmware node for the given pin, NULL if it is missing
 */
static struct fwnode_handle *
zl3073x_dpll_pin_fwnode_get(struct zl3073x_dpll_pin *pin)
{
	struct zl3073x_dpll *zldpll = pin_to_dpll(pin);
	struct fwnode_handle *pins_node, *pin_node;
	const char *node_name;
	u8 idx;

	if (zl3073x_dpll_is_input_pin(pin))
		node_name = "input-pins";
	else
		node_name = "output-pins";

	/* Get node containing input or output pins */
	pins_node = device_get_named_child_node(zldpll->mfd->dev, node_name);
	if (!pins_node) {
		dev_dbg(zldpll->mfd->dev, "'%s' sub-node is missing\n",
			node_name);
		return NULL;
	}

	/* Get pin HW index */
	idx = zl3073x_dpll_pin_index_get(pin);

	/* Enumerate pin nodes and find the requested one */
	fwnode_for_each_child_node(pins_node, pin_node) {
		u32 reg;

		if (fwnode_property_read_u32(pin_node, "reg", &reg))
			continue;

		if (idx == reg)
			break;
	}

	/* Release pin parent node */
	fwnode_handle_put(pins_node);

	dev_dbg(zldpll->mfd->dev, "Firmware node for %s%u%c %sfound\n",
		zl3073x_dpll_is_input_pin(pin) ? "REF" : "OUT", idx / 2,
		zl3073x_dpll_is_p_pin(pin) ? 'P' : 'N',
		pin_node ? "" : "NOT ");

	return pin_node;
}

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
 * zl3073x_dpll_check_frequency - verify frequency for given pin
 * @pin: pointer to pin
 * @freq: frequency to check
 *
 * The function checks the given frequency is valid for the device. For input
 * pins it checks that the frequency can be factorized using supported base
 * frequencies. For output pins it checks that the frequency divides connected
 * synth frequency without remainder.
 *
 * Return: true if the frequency is valid, false if not.
 */
static bool
zl3073x_dpll_check_frequency(struct zl3073x_dpll_pin *pin, u64 freq)
{
	struct zl3073x_dpll *zldpll = pin_to_dpll(pin);

	if (freq > U32_MAX)
		goto invalid;

	if (zl3073x_dpll_is_input_pin(pin)) {
		u16 base, mult;
		int rc;

		/* Check if the frequency can be factorized */
		rc = zl3073x_dpll_input_ref_frequency_factorize(freq, &base,
								&mult);
		if (rc)
			goto invalid;
	} else {
		struct zl3073x_dev *zldev = pin_to_dev(pin);
		u32 synth_freq;
		u8 synth;

		/* Get output pin synthesizer */
		synth = zl3073x_dpll_pin_synth_get(pin);

		/* Get synth frequency */
		synth_freq = zl3073x_synth_freq_get(zldev, synth);

		/* Check the frequency divides synth frequency */
		if (synth_freq % (u32)freq)
			goto invalid;
	}

	return true;
invalid:
	dev_warn(zldpll->dev,
		 "Unsupported frequency %llu Hz in firmware node\n", freq);

	return false;
}

/**
 * zl3073x_dpll_pin_info_get - get pin info
 * @pin: pin whose info is returned
 *
 * The function looks for firmware node for the given pin if it is provided
 * by the system firmware (DT or ACPI), allocates pin info structure,
 * generates package label string according pin type and its order number
 * and optionally fetches board label and supported frequencies from
 * the firmware node if they exist.
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
	struct device *dev = pin_to_dpll(pin)->dev;
	struct zl3073x_dpll_pin_info *pin_info;
	struct dpll_pin_frequency *ranges;
	int i, j, num_freqs, rc;
	const char *pin_type;
	u64 *freqs;

	/* Allocate pin info structure */
	pin_info = kzalloc(sizeof(*pin_info), GFP_KERNEL);
	if (!pin_info)
		return ERR_PTR(-ENOMEM);

	/* Set default pin type and capabilities */
	if (zl3073x_dpll_is_input_pin(pin)) {
		pin_info->props.type = DPLL_PIN_TYPE_EXT;
		pin_info->props.capabilities =
			DPLL_PIN_CAPABILITIES_PRIORITY_CAN_CHANGE |
			DPLL_PIN_CAPABILITIES_STATE_CAN_CHANGE;
	} else {
		pin_info->props.type = DPLL_PIN_TYPE_GNSS;
	}

	pin_info->props.phase_range.min = S32_MIN;
	pin_info->props.phase_range.max = S32_MAX;

	/* Generate package label for the given pin */
	zl3073x_dpll_pin_info_package_label_set(pin, pin_info);

	/* Get firmware node for the given pin */
	pin_info->fwnode = zl3073x_dpll_pin_fwnode_get(pin);
	if (!pin_info->fwnode)
		/* Return if it does not exist */
		return pin_info;

	/* Look for label property and store the value as board label */
	fwnode_property_read_string(pin_info->fwnode, "label",
				    &pin_info->props.board_label);

	/* Look for pin type property and translate its value to DPLL
	 * pin type enum if it is present.
	 */
	if (!fwnode_property_read_string(pin_info->fwnode, "connection-type",
					 &pin_type)) {
		if (!strcmp(pin_type, "ext"))
			pin_info->props.type = DPLL_PIN_TYPE_EXT;
		else if (!strcmp(pin_type, "gnss"))
			pin_info->props.type = DPLL_PIN_TYPE_GNSS;
		else if (!strcmp(pin_type, "int"))
			pin_info->props.type = DPLL_PIN_TYPE_INT_OSCILLATOR;
		else if (!strcmp(pin_type, "synce"))
			pin_info->props.type = DPLL_PIN_TYPE_SYNCE_ETH_PORT;
		else
			dev_warn(dev, "Unknown or unsupported pin type '%s'\n",
				 pin_type);
	}

	/* Read supported frequencies property if it is specified */
	num_freqs = fwnode_property_count_u64(pin_info->fwnode,
					      "supported-frequencies-hz");
	if (num_freqs <= 0)
		/* Return if the property does not exist or number is 0 */
		return pin_info;

	/* The firmware node specifies list of supported frequencies while
	 * DPLL core pin properties requires list of frequency ranges.
	 * So read the frequency list into temporary array.
	 */
	freqs = kcalloc(num_freqs, sizeof(*freqs), GFP_KERNEL);
	if (!freqs) {
		rc = -ENOMEM;
		goto err_alloc_freqs;
	}

	/* Read frequencies list from firmware node */
	fwnode_property_read_u64_array(pin_info->fwnode,
				       "supported-frequencies-hz", freqs,
				       num_freqs);

	/* Allocate frequency ranges list and fill it */
	ranges = kcalloc(num_freqs, sizeof(*ranges), GFP_KERNEL);
	if (!ranges) {
		rc = -ENOMEM;
		goto err_alloc_ranges;
	}

	/* Convert list of frequencies to list of frequency ranges but
	 * filter-out frequencies that are not representable by device
	 */
	for (i = 0, j = 0; i < num_freqs; i++) {
		struct dpll_pin_frequency freq = DPLL_PIN_FREQUENCY(freqs[i]);

		if (zl3073x_dpll_check_frequency(pin, freqs[i])) {
			ranges[j] = freq;
			j++;
		}
	}

	/* Save number of freq ranges and pointer to them into pin properties */
	pin_info->props.freq_supported = ranges;
	pin_info->props.freq_supported_num = j;

	/* Free temporary array */
	kfree(freqs);

	return pin_info;

err_alloc_ranges:
	kfree(freqs);
err_alloc_freqs:
	fwnode_handle_put(pin_info->fwnode);
	kfree(pin_info);

	return ERR_PTR(rc);
}

/**
 * zl3073x_dpll_pin_info_put - free pin info
 * @pin_info: pin info to free
 *
 * The function deallocates given pin info structure and firmware node handle.
 */
static void
zl3073x_dpll_pin_info_put(struct zl3073x_dpll_pin_info *pin_info)
{
	/* Free supported frequency ranges list if it is present */
	kfree(pin_info->props.freq_supported);

	/* Put firmware handle if it is present */
	if (pin_info->fwnode)
		fwnode_handle_put(pin_info->fwnode);

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
