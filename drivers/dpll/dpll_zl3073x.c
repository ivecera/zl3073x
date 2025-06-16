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
#define ZL3073X_DPLL_PHASE_OFFSET_MASK	(~0ULL << 12)

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
 * @esync_control: embedded sync is controllable
 * @pin_state: last saved pin state
 * @phase_offset: last saved pin phase offset
 * @freq_offset: last saved fractional frequency offset
 */
struct zl3073x_dpll_pin {
	struct dpll_pin			*dpll_pin;
	u8				index;
	u8				prio;
	bool				selectable;
	bool				esync_control;
	enum dpll_pin_state		pin_state;
	u64				phase_offset;
	u32				freq_offset;
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

/*
 * Supported esync ranges for input and for output per output pair type
 */
static const struct dpll_pin_frequency esync_freq_ranges[] = {
	DPLL_PIN_FREQUENCY_RANGE(0, 1),
};

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
zl3073x_dpll_input_pin_esync_get(const struct dpll_pin *dpll_pin,
				 void *pin_priv,
				 const struct dpll_device *dpll,
				 void *dpll_priv,
				 struct dpll_pin_esync *esync,
				 struct netlink_ext_ack *extack)
{
	struct zl3073x_dpll *zldpll = dpll_priv;
	struct zl3073x_dev *zldev = zldpll->mfd;
	struct zl3073x_dpll_pin *pin = pin_priv;
	u8 sync_mode, ref_id, ref_sync_ctrl;
	u32 esync_div, ref_freq;
	int rc;

	/* Get index of the pin */
	ref_id = zl3073x_dpll_pin_index_get(pin);

	/* Get reference frequency */
	rc = zl3073x_dpll_input_ref_frequency_get(zldpll, ref_id, &ref_freq);
	if (rc)
		return rc;

	guard(mutex)(&zldev->multiop_lock);

	/* Read reference configuration into mailbox */
	rc = zl3073x_mb_op(zldev, ZL_REG_REF_MB_SEM, ZL_REF_MB_SEM_RD,
			   ZL_REG_REF_MB_MASK, BIT(ref_id));
	if (rc)
		return rc;

	/* Get ref sync mode */
	rc = zl3073x_read_u8(zldev, ZL_REG_REF_SYNC_CTRL, &ref_sync_ctrl);
	if (rc)
		return rc;

	/* Get esync divisor */
	rc = zl3073x_read_u32(zldev, ZL_REG_REF_ESYNC_DIV, &esync_div);
	if (rc)
		return rc;

	sync_mode = FIELD_GET(ZL_REF_SYNC_CTRL_MODE, ref_sync_ctrl);

	switch (sync_mode) {
	case ZL_REF_SYNC_CTRL_MODE_50_50_ESYNC_25_75:
		esync->freq = (esync_div == ZL_REF_ESYNC_DIV_1HZ) ? 1 : 0;
		esync->pulse = 25;
		break;
	default:
		esync->freq = 0;
		esync->pulse = 50;
		break;
	}

	/* If the pin supports esync control expose its range but only
	 * if the current reference frequency is > 1 Hz.
	 */
	if (pin->esync_control && ref_freq > 1) {
		esync->range = esync_freq_ranges;
		esync->range_num = ARRAY_SIZE(esync_freq_ranges);
	} else {
		esync->range = NULL;
		esync->range_num = 0;
	}

	return rc;
}

static int
zl3073x_dpll_input_pin_esync_set(const struct dpll_pin *dpll_pin,
				 void *pin_priv,
				 const struct dpll_device *dpll,
				 void *dpll_priv, u64 freq,
				 struct netlink_ext_ack *extack)
{
	struct zl3073x_dpll *zldpll = dpll_priv;
	struct zl3073x_dev *zldev = zldpll->mfd;
	struct zl3073x_dpll_pin *pin = pin_priv;
	u8 ref_id, ref_sync_ctrl, sync_mode;
	int rc;

	/* Get index of the pin */
	ref_id = zl3073x_dpll_pin_index_get(pin);

	guard(mutex)(&zldev->multiop_lock);

	/* Read reference configuration into mailbox */
	rc = zl3073x_mb_op(zldev, ZL_REG_REF_MB_SEM, ZL_REF_MB_SEM_RD,
			   ZL_REG_REF_MB_MASK, BIT(ref_id));
	if (rc)
		return rc;

	/* Get ref sync mode */
	rc = zl3073x_read_u8(zldev, ZL_REG_REF_SYNC_CTRL, &ref_sync_ctrl);
	if (rc)
		return rc;

	/* Use freq == 0 to disable esync */
	if (!freq)
		sync_mode = ZL_REF_SYNC_CTRL_MODE_REFSYNC_PAIR_OFF;
	else
		sync_mode = ZL_REF_SYNC_CTRL_MODE_50_50_ESYNC_25_75;

	ref_sync_ctrl &= ZL_REF_SYNC_CTRL_MODE;
	ref_sync_ctrl |= FIELD_PREP(ZL_REF_SYNC_CTRL_MODE, sync_mode);

	/* Update ref sync control register */
	rc = zl3073x_write_u8(zldev, ZL_REG_REF_SYNC_CTRL, ref_sync_ctrl);
	if (rc)
		return rc;

	if (freq) {
		/* 1 Hz is only supported frequnecy currently */
		rc = zl3073x_write_u32(zldev, ZL_REG_REF_ESYNC_DIV,
				       ZL_REF_ESYNC_DIV_1HZ);
		if (rc)
			return rc;
	}

	/* Commit reference configuration */
	return zl3073x_mb_op(zldev, ZL_REG_REF_MB_SEM, ZL_REF_MB_SEM_WR,
			     ZL_REG_REF_MB_MASK, BIT(ref_id));
}

static int
zl3073x_dpll_input_pin_ffo_get(const struct dpll_pin *dpll_pin, void *pin_priv,
			       const struct dpll_device *dpll, void *dpll_priv,
			       s64 *ffo, struct netlink_ext_ack *extack)
{
	struct zl3073x_dpll_pin *pin = pin_priv;

	/* Values are stored in units of 2^-32 signed */
	*ffo = sign_extend64(pin->freq_offset, 31);

	/* Convert to ppm -> ffo = (10^6 * value) / 2^32 */
	*ffo = mul_s64_u64_shr(*ffo, 1000000, 32);

	return 0;
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

static int
zl3073x_dpll_input_pin_phase_offset_get(const struct dpll_pin *dpll_pin,
					void *pin_priv,
					const struct dpll_device *dpll,
					void *dpll_priv, s64 *phase_offset,
					struct netlink_ext_ack *extack)
{
	struct zl3073x_dpll *zldpll = dpll_priv;
	struct zl3073x_dev *zldev = zldpll->mfd;
	struct zl3073x_dpll_pin *pin = pin_priv;
	u8 conn_ref, ref_id, ref_status;
	s64 ref_phase;
	int rc;

	/* Get index of the pin */
	ref_id = zl3073x_dpll_pin_index_get(pin);

	/* Perform sign extension for 48bit signed value */
	ref_phase = sign_extend64(pin->phase_offset, 47);

	/* Register units are 0.01 ps -> convert it to fs */
	ref_phase = ref_phase * 10;

	/* Get currently connected reference */
	rc = zl3073x_dpll_connected_ref_get(zldpll, &conn_ref);
	if (rc)
		return rc;

	/* Get this pin monitor status */
	rc = zl3073x_read_u8(zldev, ZL_REG_REF_MON_STATUS(ref_id), &ref_status);
	if (rc)
		return rc;

	/* The DPLL being locked to a higher freq than the current ref
	 * the phase offset is modded to the period of the signal
	 * the dpll is locked to.
	 */
	if (ZL3073X_DPLL_REF_IS_VALID(conn_ref) && conn_ref != ref_id &&
	    ref_status == ZL_REF_MON_STATUS_OK) {
		u32 conn_freq, ref_freq;

		/* Get frequency of connected ref */
		rc = zl3073x_dpll_input_ref_frequency_get(zldpll, conn_ref,
							  &conn_freq);
		if (rc)
			return rc;

		/* Get frequency of given ref */
		rc = zl3073x_dpll_input_ref_frequency_get(zldpll, ref_id,
							  &ref_freq);
		if (rc)
			return rc;

		if (conn_freq > ref_freq) {
			s64 conn_period_fs;
			int div_factor;

			conn_period_fs = div_s64(FSEC_PER_SEC, conn_freq);
			div_factor = div64_s64(ref_phase, conn_period_fs);
			ref_phase -= conn_period_fs * div_factor;
		}
	}

	*phase_offset = ref_phase;

	return rc;
}

static int
zl3073x_dpll_input_pin_phase_adjust_get(const struct dpll_pin *dpll_pin,
					void *pin_priv,
					const struct dpll_device *dpll,
					void *dpll_priv,
					s32 *phase_adjust,
					struct netlink_ext_ack *extack)
{
	struct zl3073x_dpll *zldpll = dpll_priv;
	struct zl3073x_dev *zldev = zldpll->mfd;
	struct zl3073x_dpll_pin *pin = pin_priv;
	s64 phase_comp;
	u8 ref_id;
	int rc;

	/* Get index of the pin */
	ref_id = zl3073x_dpll_pin_index_get(pin);

	guard(mutex)(&zldev->multiop_lock);

	/* Read reference configuration */
	rc = zl3073x_mb_op(zldev, ZL_REG_REF_MB_SEM, ZL_REF_MB_SEM_RD,
			   ZL_REG_REF_MB_MASK, BIT(ref_id));
	if (rc)
		return rc;

	/* Read current phase offset compensation */
	rc = zl3073x_read_u48(zldev, ZL_REG_REF_PHASE_OFFSET_COMP, &phase_comp);
	if (rc)
		return rc;

	/* Perform sign extension for 48bit signed value */
	phase_comp = sign_extend64(phase_comp, 47);

	/* Reverse two's complement negation applied during set and convert
	 * to 32bit signed int
	 */
	*phase_adjust = (s32) -phase_comp;

	return rc;
}

static int
zl3073x_dpll_input_pin_phase_adjust_set(const struct dpll_pin *dpll_pin,
					void *pin_priv,
					const struct dpll_device *dpll,
					void *dpll_priv,
					s32 phase_adjust,
					struct netlink_ext_ack *extack)
{
	struct zl3073x_dpll *zldpll = dpll_priv;
	struct zl3073x_dev *zldev = zldpll->mfd;
	struct zl3073x_dpll_pin *pin = pin_priv;
	s64 phase_comp;
	u8 ref_id;
	int rc;

	/* The value in the register is stored as two's complement negation
	 * of requested value.
	 */
	phase_comp = (s64) -phase_adjust;

	/* Get index of the pin */
	ref_id = zl3073x_dpll_pin_index_get(pin);

	guard(mutex)(&zldev->multiop_lock);

	/* Read reference configuration */
	rc = zl3073x_mb_op(zldev, ZL_REG_REF_MB_SEM, ZL_REF_MB_SEM_RD,
			   ZL_REG_REF_MB_MASK, BIT(ref_id));
	if (rc)
		return rc;

	/* Write the requested value into the compensation register */
	rc = zl3073x_write_u48(zldev, ZL_REG_REF_PHASE_OFFSET_COMP, phase_comp);
	if (rc)
		return rc;

	/* Commit reference configuration */
	return zl3073x_mb_op(zldev, ZL_REG_REF_MB_SEM, ZL_REF_MB_SEM_WR,
			     ZL_REG_REF_MB_MASK, BIT(ref_id));
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
zl3073x_dpll_output_pin_esync_get(const struct dpll_pin *dpll_pin,
				  void *pin_priv,
				  const struct dpll_device *dpll,
				  void *dpll_priv,
				  struct dpll_pin_esync *esync,
				  struct netlink_ext_ack *extack)
{
	struct zl3073x_dpll *zldpll = dpll_priv;
	struct zl3073x_dev *zldev = zldpll->mfd;
	struct zl3073x_dpll_pin *pin = pin_priv;
	struct device *dev = zldpll->dev;
	u32 esync_period, esync_width;
	u8 output, output_mode;
	u8 clock_type, synth;
	u32 output_div;
	u32 synth_freq;
	int rc;

	output = zl3073x_dpll_output_pin_output_get(pin);

	/* If N-division is enabled, esync is not supported. The register used
	 * for N-division is also used for the esync divider so both cannot
	 * be used.
	 */
	switch (zl3073x_output_signal_format_get(zldev, output)) {
	case ZL_OUTPUT_MODE_SIGNAL_FORMAT_2_NDIV:
	case ZL_OUTPUT_MODE_SIGNAL_FORMAT_2_NDIV_INV:
		return -EOPNOTSUPP;
	default:
		break;
	}

	guard(mutex)(&zldev->multiop_lock);

	/* Read output configuration into mailbox */
	rc = zl3073x_mb_op(zldev, ZL_REG_OUTPUT_MB_SEM, ZL_OUTPUT_MB_SEM_RD,
			   ZL_REG_OUTPUT_MB_MASK, BIT(output));
	if (rc)
		return rc;

	/* Read output mode */
	rc = zl3073x_read_u8(zldev, ZL_REG_OUTPUT_MODE, &output_mode);
	if (rc)
		return rc;

	/* Read output divisor */
	rc = zl3073x_read_u32(zldev, ZL_REG_OUTPUT_DIV, &output_div);
	if (rc)
		return rc;

	/* Check output divisor for zero */
	if (!output_div) {
		dev_err(dev, "Zero divisor for OUTPUT%u got from device\n",
			output);
		return -EINVAL;
	}

	/* Get synth attached to output pin */
	synth = zl3073x_dpll_pin_synth_get(pin);

	/* Get synth frequency */
	synth_freq = zl3073x_synth_freq_get(zldev, synth);

	clock_type = FIELD_GET(ZL_OUTPUT_MODE_CLOCK_TYPE, output_mode);
	if (clock_type != ZL_OUTPUT_MODE_CLOCK_TYPE_ESYNC) {
		/* No need to read esync data if it is not enabled */
		esync->freq = 0;
		esync->pulse = 50;

		goto finish;
	}

	/* Read esync period */
	rc = zl3073x_read_u32(zldev, ZL_REG_OUTPUT_ESYNC_PERIOD, &esync_period);
	if (rc)
		return rc;

	/* Check esync divisor for zero */
	if (!esync_period) {
		dev_err(dev, "Zero esync divisor for OUTPUT%u got from device\n",
			output);
		return -EINVAL;
	}

	/* Get esync pulse width in units of half synth cycles */
	rc = zl3073x_read_u32(zldev, ZL_REG_OUTPUT_ESYNC_WIDTH, &esync_width);
	if (rc)
		return rc;

	/* Compute esync frequency */
	esync->freq = synth_freq / output_div / esync_period;

	/* By comparing the esync_pulse_width to the half of the pulse width
	 * the esync pulse percentage can be determined.
	 * Note that half pulse width is in units of half synth cycles, which
	 * is why it reduces down to be mb.div.
	 */
	esync->pulse = (50 * esync_width) / output_div;

finish:
	/* Set supported esync ranges if the pin supports esync control and
	 * if the output frequency is > 1 Hz.
	 */
	if (pin->esync_control && (synth_freq / output_div) > 1) {
		esync->range = esync_freq_ranges;
		esync->range_num = ARRAY_SIZE(esync_freq_ranges);
	} else {
		esync->range = NULL;
		esync->range_num = 0;
	}

	return 0;
}

static int
zl3073x_dpll_output_pin_esync_set(const struct dpll_pin *dpll_pin,
				  void *pin_priv,
				  const struct dpll_device *dpll,
				  void *dpll_priv, u64 freq,
				  struct netlink_ext_ack *extack)
{
	struct zl3073x_dpll *zldpll = dpll_priv;
	struct zl3073x_dev *zldev = zldpll->mfd;
	struct zl3073x_dpll_pin *pin = pin_priv;
	u32 esync_period, esync_width, output_div;
	u8 clock_type, output, output_mode, synth;
	u32 synth_freq;
	int rc;

	output = zl3073x_dpll_output_pin_output_get(pin);

	/* If N-division is enabled, esync is not supported. The register used
	 * for N-division is also used for the esync divider so both cannot
	 * be used.
	 */
	switch (zl3073x_output_signal_format_get(zldev, output)) {
	case ZL_OUTPUT_MODE_SIGNAL_FORMAT_2_NDIV:
	case ZL_OUTPUT_MODE_SIGNAL_FORMAT_2_NDIV_INV:
		return -EOPNOTSUPP;
	default:
		break;
	}

	guard(mutex)(&zldev->multiop_lock);

	/* Read output configuration into mailbox */
	rc = zl3073x_mb_op(zldev, ZL_REG_OUTPUT_MB_SEM, ZL_OUTPUT_MB_SEM_RD,
			   ZL_REG_OUTPUT_MB_MASK, BIT(output));
	if (rc)
		return rc;

	/* Read output mode */
	rc = zl3073x_read_u8(zldev, ZL_REG_OUTPUT_MODE, &output_mode);
	if (rc)
		return rc;

	/* Select clock type */
	if (freq)
		clock_type = ZL_OUTPUT_MODE_CLOCK_TYPE_ESYNC;
	else
		clock_type = ZL_OUTPUT_MODE_CLOCK_TYPE_NORMAL;

	/* Update clock type in ref sync control */
	output_mode &= ~ZL_OUTPUT_MODE_CLOCK_TYPE;
	output_mode |= FIELD_PREP(ZL_OUTPUT_MODE_CLOCK_TYPE, clock_type);

	/* If esync is being disabled just write mailbox and finish */
	if (!freq)
		goto write_mailbox;

	/* Get synth attached to output pin */
	synth = zl3073x_dpll_pin_synth_get(pin);

	/* Get synth frequency */
	synth_freq = zl3073x_synth_freq_get(zldev, synth);

	rc = zl3073x_read_u32(zldev, ZL_REG_OUTPUT_DIV, &output_div);
	if (rc)
		return rc;

	/* Compute and update esync period */
	esync_period = synth_freq / (u32)freq / output_div;
	rc = zl3073x_write_u32(zldev, ZL_REG_OUTPUT_ESYNC_PERIOD, esync_period);
	if (rc)
		return rc;

	/* Half of the period in units of 1/2 synth cycle can be represented by
	 * the output_div. To get the supported esync pulse width of 25% of the
	 * period the output_div can just be divided by 2. Note that this
	 * assumes that output_div is even, otherwise some resolution will be
	 * lost.
	 */
	esync_width = output_div / 2;
	rc = zl3073x_write_u32(zldev, ZL_REG_OUTPUT_ESYNC_WIDTH, esync_width);
	if (rc)
		return rc;

write_mailbox:
	/* Commit output configuration */
	return zl3073x_mb_op(zldev, ZL_REG_OUTPUT_MB_SEM, ZL_OUTPUT_MB_SEM_WR,
			     ZL_REG_OUTPUT_MB_MASK, BIT(output));
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
zl3073x_dpll_output_pin_phase_adjust_get(const struct dpll_pin *dpll_pin,
					 void *pin_priv,
					 const struct dpll_device *dpll,
					 void *dpll_priv,
					 s32 *phase_adjust,
					 struct netlink_ext_ack *extack)
{
	struct zl3073x_dpll *zldpll = dpll_priv;
	struct zl3073x_dev *zldev = zldpll->mfd;
	struct zl3073x_dpll_pin *pin = pin_priv;
	u8 output, synth;
	u32 synth_freq;
	s32 phase_comp;
	int rc;

	output = zl3073x_dpll_output_pin_output_get(pin);
	synth = zl3073x_dpll_pin_synth_get(pin);
	synth_freq = zl3073x_synth_freq_get(zldev, synth);

	guard(mutex)(&zldev->multiop_lock);

	/* Read output configuration */
	rc = zl3073x_mb_op(zldev, ZL_REG_OUTPUT_MB_SEM, ZL_OUTPUT_MB_SEM_RD,
			   ZL_REG_OUTPUT_MB_MASK, BIT(output));
	if (rc)
		return rc;

	/* Read current output phase compensation */
	rc = zl3073x_read_u32(zldev, ZL_REG_OUTPUT_PHASE_COMP, &phase_comp);
	if (rc)
		return rc;

	/* Value in register is expressed in half synth clock cycles */
	phase_comp *= (int)div_u64(PSEC_PER_SEC, 2 * synth_freq);

	/* Reverse two's complement negation applied during 'set' */
	*phase_adjust = -phase_comp;

	return rc;
}

static int
zl3073x_dpll_output_pin_phase_adjust_set(const struct dpll_pin *dpll_pin,
					 void *pin_priv,
					 const struct dpll_device *dpll,
					 void *dpll_priv,
					 s32 phase_adjust,
					 struct netlink_ext_ack *extack)
{
	struct zl3073x_dpll *zldpll = dpll_priv;
	struct zl3073x_dev *zldev = zldpll->mfd;
	struct zl3073x_dpll_pin *pin = pin_priv;
	int half_synth_cycle;
	u8 output, synth;
	u32 synth_freq;
	int rc;

	/* Get attached synth */
	synth = zl3073x_dpll_pin_synth_get(pin);

	/* Get synth's frequency */
	synth_freq = zl3073x_synth_freq_get(zldev, synth);

	/* Value in register is expressed in half synth clock cycles so
	 * the given phase adjustment a multiple of half synth clock.
	 */
	half_synth_cycle = (int)div_u64(PSEC_PER_SEC, 2 * synth_freq);

	if ((phase_adjust % half_synth_cycle) != 0) {
		NL_SET_ERR_MSG_FMT(extack,
				   "Phase adjustment value has to be multiple of %d",
				   half_synth_cycle);
		return -EINVAL;
	}
	phase_adjust /= half_synth_cycle;

	/* The value in the register is stored as two's complement negation
	 * of requested value.
	 */
	phase_adjust = -phase_adjust;

	guard(mutex)(&zldev->multiop_lock);

	/* Read output configuration */
	output = zl3073x_dpll_output_pin_output_get(pin);
	rc = zl3073x_mb_op(zldev, ZL_REG_OUTPUT_MB_SEM, ZL_OUTPUT_MB_SEM_RD,
			   ZL_REG_OUTPUT_MB_MASK, BIT(output));
	if (rc)
		return rc;

	/* Write the requested value into the compensation register */
	rc = zl3073x_write_u32(zldev, ZL_REG_OUTPUT_PHASE_COMP, phase_adjust);
	if (rc)
		return rc;

	/* Update output configuration from mailbox */
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
	.esync_get = zl3073x_dpll_input_pin_esync_get,
	.esync_set = zl3073x_dpll_input_pin_esync_set,
	.ffo_get = zl3073x_dpll_input_pin_ffo_get,
	.frequency_get = zl3073x_dpll_input_pin_frequency_get,
	.frequency_set = zl3073x_dpll_input_pin_frequency_set,
	.phase_offset_get = zl3073x_dpll_input_pin_phase_offset_get,
	.phase_adjust_get = zl3073x_dpll_input_pin_phase_adjust_get,
	.phase_adjust_set = zl3073x_dpll_input_pin_phase_adjust_set,
	.prio_get = zl3073x_dpll_input_pin_prio_get,
	.prio_set = zl3073x_dpll_input_pin_prio_set,
	.state_on_dpll_get = zl3073x_dpll_input_pin_state_on_dpll_get,
	.state_on_dpll_set = zl3073x_dpll_input_pin_state_on_dpll_set,
};

static const struct dpll_pin_ops zl3073x_dpll_output_pin_ops = {
	.direction_get = zl3073x_dpll_pin_direction_get,
	.esync_get = zl3073x_dpll_output_pin_esync_get,
	.esync_set = zl3073x_dpll_output_pin_esync_set,
	.frequency_get = zl3073x_dpll_output_pin_frequency_get,
	.frequency_set = zl3073x_dpll_output_pin_frequency_set,
	.phase_adjust_get = zl3073x_dpll_output_pin_phase_adjust_get,
	.phase_adjust_set = zl3073x_dpll_output_pin_phase_adjust_set,
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

	/* Check if the pin supports embedded sync control */
	pin->esync_control = fwnode_property_read_bool(pin_info->fwnode,
						       "esync-control");

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

static int
zl3073x_dpll_freq_meas(struct zl3073x_dpll *zldpll, u32 *freq_offset)
{
	struct zl3073x_dev *zldev = zldpll->mfd;
	int i, rc;

	guard(mutex)(&zldev->multiop_lock);

	/* Per datasheet we have to wait for 'ref_freq_meas_ctrl' to be zero
	 * to ensure that the measured data are coherent.
	 */
	rc = zl3073x_poll_zero_u8(zldev, ZL_REG_REF_FREQ_MEAS_CTRL,
				  ZL_REF_FREQ_MEAS_CTRL);
	if (rc)
		return rc;

	/* Select all references for measurement */
	rc = zl3073x_write_u8(zldev, ZL_REG_REF_FREQ_MEAS_MASK_3_0,
			      GENMASK(7, 0)); /* REF0P..REF3N */
	if (rc)
		return rc;
	rc = zl3073x_write_u8(zldev, ZL_REG_REF_FREQ_MEAS_MASK_4,
			      GENMASK(1, 0)); /* REF4P..REF4N */
	if (rc)
		return rc;

	/* Request frequency offset measurement */
	rc = zl3073x_write_u8(zldev, ZL_REG_REF_FREQ_MEAS_CTRL,
			      ZL_REF_FREQ_MEAS_CTRL_REF_FREQ_OFF);
	if (rc)
		return rc;

	/* Wait for finish */
	rc = zl3073x_poll_zero_u8(zldev, ZL_REG_REF_FREQ_MEAS_CTRL,
				  ZL_REF_FREQ_MEAS_CTRL);
	if (rc)
		return rc;

	/* Read DPLL-to-REFx frequency offset measurements */
	for (i = 0; i < ZL3073X_NUM_INPUT_PINS; i++) {
		rc = zl3073x_read_u32(zldev, ZL_REG_REF_FREQ(i),
				      &freq_offset[i]);
		if (rc)
			break;
	}

	return rc;
}

static int
zl3073x_dpll_phase_meas(struct zl3073x_dpll *zldpll, u64 *ref_phase)
{
	struct zl3073x_dev *zldev = zldpll->mfd;
	u8 dpll_meas_ctrl;
	int i, rc;

	guard(mutex)(&zldev->multiop_lock);

	/* Per datasheet we have to wait for 'dpll_ref_phase_err_rqst_opl'
	 * to be zero to ensure that the measured data are coherent.
	 */
	rc = zl3073x_poll_zero_u8(zldev, ZL_REG_REF_PHASE_ERR_READ_RQST,
				  ZL_REF_PHASE_ERR_READ_RQST_RD);
	if (rc)
		return rc;

	/* Read DPLL measurement control register */
	rc = zl3073x_read_u8(zldev, ZL_REG_DPLL_MEAS_CTRL, &dpll_meas_ctrl);
	if (rc)
		return rc;

	/* Enable DPLL measurement block */
	dpll_meas_ctrl |= ZL_DPLL_MEAS_CTRL_EN;
	rc = zl3073x_write_u8(zldev, ZL_REG_DPLL_MEAS_CTRL, dpll_meas_ctrl);
	if (rc)
		return rc;

	/* Select this DPLL for measurement */
	rc = zl3073x_write_u8(zldev, ZL_REG_DPLL_MEAS_IDX, zldpll->id);
	if (rc)
		return rc;

	/* Request phase offset measurement */
	rc = zl3073x_write_u8(zldev, ZL_REG_REF_PHASE_ERR_READ_RQST,
			      ZL_REF_PHASE_ERR_READ_RQST_RD);
	if (rc)
		return rc;

	/* Wait for finish */
	rc = zl3073x_poll_zero_u8(zldev, ZL_REG_REF_PHASE_ERR_READ_RQST,
				  ZL_REF_PHASE_ERR_READ_RQST_RD);
	if (rc)
		return rc;

	/* Read and store DPLL-to-REFx phase measurements */
	for (i = 0; i < ZL3073X_NUM_INPUT_PINS; i++) {
		rc = zl3073x_read_u48(zldev, ZL_REG_REF_PHASE(i),
				      &ref_phase[i]);
		if (rc)
			break;
	}

	return rc;
}

static void
zl3073x_dpll_periodic_work(struct kthread_work *work)
{
	struct zl3073x_dpll *zldpll = container_of(work, struct zl3073x_dpll,
						   work.work);
	u64 ref_phase[ZL3073X_NUM_INPUT_PINS];
	u32 ref_freq_offset[ZL3073X_NUM_INPUT_PINS];
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

	if (zldpll->refsel_mode != ZL_DPLL_MODE_REFSEL_MODE_AUTO &&
	    zldpll->refsel_mode != ZL_DPLL_MODE_REFSEL_MODE_REFLOCK)
		goto out;

	/* Perform phase measurement for all refs to this DPLL channel */
	rc = zl3073x_dpll_phase_meas(zldpll, ref_phase);
	if (rc) {
		dev_err(dev, "Failed to perform phase measurements: %pe\n",
			ERR_PTR(rc));
		goto out;
	}

	/* Perform frequency offset measurement for all refs */
	rc = zl3073x_dpll_freq_meas(zldpll, ref_freq_offset);
	if (rc) {
		dev_err(dev,
			"Failed to perform frequency offset measurement: %pe\n",
			ERR_PTR(rc));
		goto out;
	}

	for (i = 0; i < ZL3073X_NUM_INPUT_PINS; i++) {
		struct zl3073x_dpll_pin *pin;
		enum dpll_pin_state state;
		bool pin_changed = false;
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
			dev_dbg(dev, "INPUT%u state changed: %u->%u\n", index,
				pin->pin_state, state);
			pin->pin_state = state;
			pin_changed = true;
		}

		if ((ref_phase[index] & ZL3073X_DPLL_PHASE_OFFSET_MASK) != 
		    ((pin->phase_offset) & ZL3073X_DPLL_PHASE_OFFSET_MASK)) {
			dev_err(dev,
				"INPUT%u phase offset changed: 0x%llx-> 0x%llx diff:%lld mask:0x%llx lhs:-x%llx rhs:0x%llx\n",
				index, sign_extend64(pin->phase_offset, 47),
				sign_extend64(ref_phase[index], 47),
				abs(sign_extend64(pin->phase_offset, 47) - 
					sign_extend64(ref_phase[index], 47)),
				ZL3073X_DPLL_PHASE_OFFSET_MASK,
				ref_phase[index] & ZL3073X_DPLL_PHASE_OFFSET_MASK,
				(pin->phase_offset) & ZL3073X_DPLL_PHASE_OFFSET_MASK);
			pin->phase_offset = ref_phase[index];
			pin_changed = true;
		} else {
			if (abs(sign_extend64(pin->phase_offset, 47) - 
						sign_extend64(ref_phase[index], 47)) > 2047) {
				dev_err(dev,
						"Debug: INPUT%u phase offset ignoring: 0x%llx->0x%llx diff:0x%llx mask:0x%llx\n",
						index, sign_extend64(pin->phase_offset, 47),
						sign_extend64(ref_phase[index], 47),
						abs(sign_extend64(pin->phase_offset, 47) - 
							sign_extend64(ref_phase[index], 47)),
						ZL3073X_DPLL_PHASE_OFFSET_MASK);
			} else {
				dev_err(dev,
						"INPUT%u phase offset ignoring: 0x%llx->0x%llx diff:0x%llx mask:0x%llx\n",
						index, sign_extend64(pin->phase_offset, 47),
						sign_extend64(ref_phase[index], 47),
						abs(sign_extend64(pin->phase_offset, 47) - 
							sign_extend64(ref_phase[index], 47)),
						ZL3073X_DPLL_PHASE_OFFSET_MASK);
			}
		}

		if (ref_freq_offset[index] != pin->freq_offset) {
			dev_err(dev, "INPUT%u freq offset changed: %u->%u\n",
				index, pin->freq_offset, ref_freq_offset[index]);
			pin->freq_offset = ref_freq_offset[index];
			pin_changed = true;
		}

		if (pin_changed)
			dpll_pin_change_ntf(pin->dpll_pin);
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
