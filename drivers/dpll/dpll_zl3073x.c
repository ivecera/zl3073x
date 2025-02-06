/* SPDX-License-Identifier: GPL-2.0+ */

#include <linux/bitfield.h>
#include <linux/dpll.h>
#include <linux/mfd/core.h>
#include <linux/mfd/zl3073x.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>

/*
 * Register Map Page 0, General
 */
ZL3073X_REG16_DEF(id,				0x001);

/*
 * Register Map Page 2, Status
 */
ZL3073X_REG8_IDX_DEF(ref_mon_status,		0x102, ZL3073X_NUM_IPINS, 1);
#define REF_MON_STATUS_LOS_FAIL			BIT(0)
#define REF_MON_STATUS_SCM_FAIL			BIT(1)
#define REF_MON_STATUS_CFM_FAIL			BIT(2)
#define REF_MON_STATUS_GST_FAIL			BIT(3)
#define REF_MON_STATUS_PFM_FAIL			BIT(4)
#define REF_MON_STATUS_ESYNC_FAIL		BIT(6)
#define REF_MON_STATUS_SPLIT_XO_FAIL		BIT(7)
#define REF_MON_STATUS_OK			0	/* all bits zeroed */

ZL3073X_REG8_IDX_DEF(dpll_mon_status,		0x110, ZL3073X_NUM_CHANNELS, 1);
#define DPLL_MON_STATUS_HO_READY		BIT(2)

ZL3073X_REG8_IDX_DEF(dpll_state_refsel_status,	0x130, ZL3073X_NUM_CHANNELS, 1);
#define DPLL_STATE_REFSEL_STATUS_REFSEL		GENMASK(3, 0)
#define DPLL_STATE_REFSEL_STATUS_STATE		GENMASK(6, 4)
#define DPLL_STATE_REFSEL_STATUS_STATE_FREERUN	0
#define DPLL_STATE_REFSEL_STATUS_STATE_HOLDOVER	1
#define DPLL_STATE_REFSEL_STATUS_STATE_FASTLOCK	2
#define DPLL_STATE_REFSEL_STATUS_STATE_ACQUIRING 3
#define DPLL_STATE_REFSEL_STATUS_STATE_LOCK	4

/*
 * Register Map Page 4, Ref
 */
ZL3073X_REG8_DEF(ref_phase_err_read_rqst,	0x20f);
#define REF_PHASE_ERR_READ_RQST_RD		BIT(0)

ZL3073X_REG48_IDX_DEF(ref_phase,		0x220, ZL3073X_NUM_IPINS, 6);

/*
 * Register Map Page 5, DPLL
 */
ZL3073X_REG8_IDX_DEF(dpll_mode_refsel,		0x284, ZL3073X_NUM_CHANNELS, 4);
#define DPLL_MODE_REFSEL_MODE			GENMASK(2, 0)
#define DPLL_MODE_REFSEL_MODE_FREERUN		0
#define DPLL_MODE_REFSEL_MODE_HOLDOVER		1
#define DPLL_MODE_REFSEL_MODE_REFLOCK		2
#define DPLL_MODE_REFSEL_MODE_AUTO		3
#define DPLL_MODE_REFSEL_MODE_NCO		4
#define DPLL_MODE_REFSEL_REF			GENMASK(7, 4)

ZL3073X_REG8_DEF(dpll_meas_ctrl,		0x2d0);
#define DPLL_MEAS_CTRL_EN			BIT(0)
#define DPLL_MEAS_CTRL_AVG_FACTOR		GENMASK(7, 4)

ZL3073X_REG8_DEF(dpll_meas_idx,			0x2d1);
#define DPLL_MEAS_IDX_IDX			GENMASK(2, 0)

/*
 * Register Map Page 9, Synth and Output
 */
ZL3073X_REG8_IDX_DEF(synth_ctrl,		0x480, ZL3073X_NUM_SYNTHS, 1);
#define SYNTH_CTRL_EN				BIT(0)
#define SYNTH_CTRL_SPREAD_SPECTRUM_EN		BIT(1)
#define SYNTH_CTRL_DPLL_SEL			GENMASK(6, 4)

ZL3073X_REG8_IDX_DEF(output_ctrl,		0x4a8, ZL3073X_NUM_OPAIRS, 1);
#define OUTPUT_CTRL_EN				BIT(0)
#define OUTPUT_CTRL_STOP			BIT(1)
#define OUTPUT_CTRL_STOP_HIGH			BIT(2)
#define OUTPUT_CTRL_STOP_HZ			BIT(3)
#define OUTPUT_CTRL_SYNTH_SEL			GENMASK(6, 4)

ZL3073X_REG8_DEF(synth_phase_shift_ctrl,	0x49e);
ZL3073X_REG8_DEF(synth_phase_shift_mask,	0x49f);
ZL3073X_REG8_DEF(synth_phase_shift_intvl,	0x4a0);
ZL3073X_REG16_DEF(synth_phase_shift_data,	0x4a1);

/*
 * Register Map Page 10, Ref Mailbox
 */
ZL3073X_REG16_DEF(ref_freq_base,		0x505);
ZL3073X_REG16_DEF(ref_freq_mult,		0x507);
ZL3073X_REG16_DEF(ref_ratio_m,			0x509);
ZL3073X_REG16_DEF(ref_ratio_n,			0x50b);
ZL3073X_REG48_DEF(ref_phase_offset_compensation,0x528);

/*
 * Register Map Page 12, DPLL Mailbox
 */
ZL3073X_REG8_IDX_DEF(dpll_ref_prio,		0x652, ZL3073X_NUM_IPINS/2, 1);
#define DPLL_REF_PRIO_REF_P			GENMASK(3, 0)
#define DPLL_REF_PRIO_REF_N			GENMASK(7, 4)
#define DPLL_REF_PRIO_INVALID			0xf

/*
 * Register Map Page 14, Output Mailbox
 */
ZL3073X_REG32_DEF(output_div,			0x70c);
ZL3073X_REG32_DEF(output_width,			0x710);
ZL3073X_REG32_DEF(output_esync_period,		0x714);
ZL3073X_REG32_DEF(output_esync_width,		0x718);

ZL3073X_REG32_DEF(output_phase_compensation,	0x720);

#define ZL3073X_REF_INVALID			ZL3073X_NUM_IPINS
#define ZL3073X_REF_IS_VALID(_ref)		((_ref) < ZL3073X_REF_INVALID)

/**
 * struct zl3073x_dpll_pin - DPLL pin
 * dpll_pin: pointer to registered dpll_pin
 * props: pin properties
 * index: index in zl3073x_dpll.pins array
 */
struct zl3073x_dpll_pin {
	struct dpll_pin			*dpll_pin;
	struct dpll_pin_properties	props;
	u8				index;
	enum dpll_pin_state		pin_state;
	s64				phase_offset;
};

/**
 * struct zl3073x_dpll - ZL3073x DPLL sub-device structure
 * @dev: device pointer
 * @mfd: pointer to multi-function parent device
 * @id: DPLL identifier (0 or 1)
 * @pins: array of pins
 * @kworker: thread of periodic work
 * @work: periodic work
 */
struct zl3073x_dpll {
	struct device			*dev;
	struct zl3073x_dev		*mfd;
	int				id;
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

/*
 * Supported input frequencies
 */
struct dpll_pin_frequency input_freq_ranges[] = {
	DPLL_PIN_FREQUENCY(1),
	DPLL_PIN_FREQUENCY(25),
	DPLL_PIN_FREQUENCY(100),
	DPLL_PIN_FREQUENCY(1000),
	DPLL_PIN_FREQUENCY(10000000),
	DPLL_PIN_FREQUENCY(25000000),
	DPLL_PIN_FREQUENCY(62500000),
	DPLL_PIN_FREQUENCY(78125000),
	DPLL_PIN_FREQUENCY(100000000),
};

/*
 * Supported output frequencies per output pair type
 */
struct dpll_pin_frequency output_freq_ranges_synce[] = {
	DPLL_PIN_FREQUENCY(156250000),
};
struct dpll_pin_frequency output_freq_ranges_ptp[] = {
	DPLL_PIN_FREQUENCY(1),
	DPLL_PIN_FREQUENCY(25),
	DPLL_PIN_FREQUENCY(100),
	DPLL_PIN_FREQUENCY(1000),
	DPLL_PIN_FREQUENCY(10000000),
	DPLL_PIN_FREQUENCY(25000000),
};
struct dpll_pin_frequency output_freq_ranges_25mhz[] = {
	DPLL_PIN_FREQUENCY(25000000),
};
struct dpll_pin_frequency output_freq_ranges_10mhz[] = {
	DPLL_PIN_FREQUENCY(10000000),
};
struct dpll_pin_frequency output_freq_ranges_1hz[] = {
	DPLL_PIN_FREQUENCY(1),
};

/**
 * zl3073x_dpll_pin_ref_id_get - get ref id for input pin
 * @pin_index: index of pin
 *
 * Returns ref id for given input pin
 */
static inline u8 zl3073x_dpll_pin_ref_id_get(u8 pin_index)
{
	WARN_ON(!ZL3073X_IS_INPUT_PIN(pin_index));

	return pin_index - ZL3073X_NUM_OPINS;
}

/**
 * zl3073x_dpll_pin_pair_get - get pair id for output pin
 * @pin_index: index of pin
 *
 * Returns output pair id for given output pin
 */
static inline u8 zl3073x_dpll_pin_pair_get(u8 pin_index)
{
	WARN_ON(ZL3073X_IS_INPUT_PIN(pin_index));

	return pin_index / 2;
}

static int
zl3073x_dpll_pin_direction_get(const struct dpll_pin *dpll_pin, void *pin_priv,
			       const struct dpll_device *dpll, void *dpll_priv,
			       enum dpll_pin_direction *direction,
			       struct netlink_ext_ack *extack)
{
	struct zl3073x_dpll_pin *pin = pin_priv;

	if (ZL3073X_IS_INPUT_PIN(pin->index))
		*direction = DPLL_PIN_DIRECTION_INPUT;
	else
		*direction = DPLL_PIN_DIRECTION_OUTPUT;

	return 0;
}

enum zl3073x_pin_input_frequency {
	ZL3073X_INPUT_FREQ_1HZ		= 1,
	ZL3073X_INPUT_FREQ_25HZ		= 25,
	ZL3073X_INPUT_FREQ_100HZ	= 100,
	ZL3073X_INPUT_FREQ_1KHZ		= 1000,
	ZL3073X_INPUT_FREQ_10MHZ	= 10000000,
	ZL3073X_INPUT_FREQ_25MHZ	= 25000000,
	ZL3073X_INPUT_FREQ_62p5MHZ	= 62500000,
	ZL3073X_INPUT_FREQ_78p125MHZ	= 78125000,
	ZL3073X_INPUT_FREQ_100MHZ	= 100000000,
};

static const struct zl3073x_pin_input_freq {
	u64	frequency;
	u16	base_freq;
	u16	multiplier;
	u16	numerator;
	u16	denominator;
} zl3073x_dpll_input_freqs[] = {
	{ ZL3073X_INPUT_FREQ_1HZ,	0x0001, 0x0001, 0x0001, 0x0001 },
	{ ZL3073X_INPUT_FREQ_25HZ,	0x0001, 0x0019, 0x0001, 0x0001 },
	{ ZL3073X_INPUT_FREQ_100HZ,	0x0001, 0x0064, 0x0001, 0x0001 },
	{ ZL3073X_INPUT_FREQ_1KHZ,	0x0001, 0x03E8, 0x0001, 0x0001 },
	{ ZL3073X_INPUT_FREQ_10MHZ,	0x2710, 0x03E8, 0x0001, 0x0001 },
	{ ZL3073X_INPUT_FREQ_25MHZ,	0x61A8, 0x03E8, 0x0001, 0x0001 },
	{ ZL3073X_INPUT_FREQ_62p5MHZ,	0x4E20, 0x0C35, 0x0001, 0x0001 },
	{ ZL3073X_INPUT_FREQ_78p125MHZ,	0x61A8, 0x0C35, 0x0001, 0x0001 },
	{ ZL3073X_INPUT_FREQ_100MHZ,	0x4E20, 0x1388, 0x0001, 0x0001 },
};

/**
 * zl3073x_dpll_input_ref_frequency_get - get input reference frequency
 * zldev: pointer to device structure
 * ref_id: reference id
 * frequency: pointer to variable to store frequency
 *
 * Context: zl3073x_dev.lock has to be held
 *
 * Reads frequency of given input reference.
 *
 * Returns 0 in case of success or negative value if error occured
 */
static int
zl3073x_dpll_input_ref_frequency_get(struct zl3073x_dev *zldev, u8 ref_id,
				     u64 *frequency)
{
	u16 numerator, denominator;
	u16 base_freq, multiplier;
	u64 cur_freq;
	int f, rc;

	/* Read reference configuration into mailbox */
	rc = zl3073x_mb_ref_read(zldev, ref_id);
	if (rc)
		return rc;

	/* Read base frequency */
	rc = zl3073x_read_ref_freq_base(zldev, &base_freq);
	if (rc)
		return rc;

	/* Read multiplier */
	rc = zl3073x_read_ref_freq_mult(zldev, &multiplier);
	if (rc)
		return rc;

	/* Write numerator */
	rc = zl3073x_read_ref_ratio_m(zldev, &numerator);
	if (rc)
		return rc;

	/* Write denominator */
	rc = zl3073x_read_ref_ratio_n(zldev, &denominator);
	if (rc)
		return rc;

	cur_freq = base_freq * multiplier * numerator / denominator;

	for (f = 0; f < ARRAY_SIZE(zl3073x_dpll_input_freqs); f++)
		if (cur_freq == zl3073x_dpll_input_freqs[f].frequency)
			break;

	if (f < ARRAY_SIZE(zl3073x_dpll_input_freqs))
		*frequency = zl3073x_dpll_input_freqs[f].frequency;
	else
		*frequency = 0;

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
	struct zl3073x_dev *zldev = zldpll->mfd;
	struct zl3073x_dpll_pin *pin = pin_priv;
	u8 ref_id;

	/* Take device lock */
	guard(zl3073x)(zldev);

	/* Get ref id for the pin */
	ref_id = zl3073x_dpll_pin_ref_id_get(pin->index);

	/* Read and return ref frequency */
	return zl3073x_dpll_input_ref_frequency_get(zldev, ref_id, frequency);
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
	const struct zl3073x_pin_input_freq *freq;
	u8 ref_id;
	int f, rc;

	/* Find parameters for requested frequency */
	for (f = 0; f < ARRAY_SIZE(zl3073x_dpll_input_freqs); f++)
		if (frequency == zl3073x_dpll_input_freqs[f].frequency)
			break;

	if (f == ARRAY_SIZE(zl3073x_dpll_input_freqs))
		return -EINVAL;

	freq = &zl3073x_dpll_input_freqs[f];

	/* Take device lock */
	guard(zl3073x)(zldev);

	/* Write base frequency */
	rc = zl3073x_write_ref_freq_base(zldev, freq->base_freq);
	if (rc)
		return rc;

	/* Write multiplier */
	rc = zl3073x_write_ref_freq_mult(zldev, freq->multiplier);
	if (rc)
		return rc;

	/* Write numerator */
	rc = zl3073x_write_ref_ratio_m(zldev, freq->numerator);
	if (rc)
		return rc;

	/* Write denominator */
	rc = zl3073x_write_ref_ratio_n(zldev, freq->denominator);
	if (rc)
		return rc;

	/* Get ref id for the pin */
	ref_id = zl3073x_dpll_pin_ref_id_get(pin->index);

	/* Update reference configuration from mailbox */
	rc = zl3073x_mb_ref_write(zldev, ref_id);
	if (rc)
		return rc;

	return rc;
}

static int
zl3073x_dpll_selected_ref_get(struct zl3073x_dpll *zldpll, u8 *ref)
{
	struct zl3073x_dev *zldev = zldpll->mfd;
	u8 refsel_status;
	int rc;

	rc = zl3073x_read_dpll_state_refsel_status(zldev, zldpll->id,
						   &refsel_status);
	if (rc)
		return rc;

	*ref = FIELD_GET(DPLL_STATE_REFSEL_STATUS_REFSEL, refsel_status);

	return rc;
}

static int
zl3073x_dpll_connected_ref_get(struct zl3073x_dpll *zldpll, u8 *ref)
{
	struct zl3073x_dev *zldev = zldpll->mfd;
	u8 dpll_mode_refsel, mode;
	int rc;

	rc = zl3073x_read_dpll_mode_refsel(zldev, zldpll->id,
					   &dpll_mode_refsel);
	if (rc)
		return rc;

	mode = FIELD_GET(DPLL_MODE_REFSEL_MODE, dpll_mode_refsel);

	if (mode == DPLL_MODE_REFSEL_MODE_AUTO) {
		rc = zl3073x_dpll_selected_ref_get(zldpll, ref);
		if (rc)
			return rc;
	} else if (mode == DPLL_MODE_REFSEL_MODE_REFLOCK) {
		*ref = FIELD_GET(DPLL_MODE_REFSEL_REF, dpll_mode_refsel);
	} else {
		*ref = ZL3073X_REF_INVALID;
	}

	if (ZL3073X_REF_IS_VALID(*ref)) {
		u8 ref_status;

		rc = zl3073x_read_ref_mon_status(zldev, *ref, &ref_status);
		if (rc)
			return rc;

		if (ref_status != REF_MON_STATUS_OK)
			*ref = ZL3073X_REF_INVALID;
	}

	return rc;
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
	u8 dpll_meas_ctrl, dpll_meas_idx;
	u8 conn_ref, ref_id, ref_status;
	s64 ref_phase;
	int rc;

	/* Take device lock */
	guard(zl3073x)(zldev);

	/* Get ref id for the pin */
	ref_id = zl3073x_dpll_pin_ref_id_get(pin->index);

	/* Wait for reading to be ready */
	rc = zl3073x_wait_clear_bits(zldev, ref_phase_err_read_rqst,
				     REF_PHASE_ERR_READ_RQST_RD);
	if (rc)
		return rc;

	/* Read measurement control register */
	rc = zl3073x_read_dpll_meas_ctrl(zldev, &dpll_meas_ctrl);
	if (rc)
		return rc;

	/* Enable measurement */
	dpll_meas_ctrl |= DPLL_MEAS_CTRL_EN;

	/* Update measurement control register with new values */
	rc = zl3073x_write_dpll_meas_ctrl(zldev, dpll_meas_ctrl);
	if (rc)
		return rc;

	/* Set measurement index to channel index */
	dpll_meas_idx = FIELD_PREP(DPLL_MEAS_IDX_IDX, zldpll->id);
	rc = zl3073x_write_dpll_meas_idx(zldev, dpll_meas_idx);
	if (rc)
		return rc;

	/* Request read of the current phase error measurements */
	rc = zl3073x_write_ref_phase_err_read_rqst(zldev,
						   REF_PHASE_ERR_READ_RQST_RD);
	if (rc)
		return rc;

	/* Wait for confirmation from the device */
	rc = zl3073x_wait_clear_bits(zldev, ref_phase_err_read_rqst,
				     REF_PHASE_ERR_READ_RQST_RD);
	if (rc)
		return rc;

	/* Read DPLL-to-REF phase measurement */
	rc = zl3073x_read_ref_phase(zldev, ref_id, &ref_phase);
	if (rc)
		return rc;

	/* Perform sign extension for 48bit signed value */
	if (ref_phase & BIT_ULL(47))
		ref_phase |= GENMASK_ULL(63, 48);

	/* Register units are 0.01 ps -> convert it to ps */
	ref_phase = div_s64(ref_phase, 100);

	/* The DPLL being locked to a higher freq than the current ref
	 * the phase offset is modded to the period of the signal
	 * the dpll is locked to.
         */
	rc = zl3073x_dpll_connected_ref_get(zldpll, &conn_ref);
	if (rc)
		return rc;

	rc = zl3073x_read_ref_mon_status(zldev, ref_id, &ref_status);
	if (rc)
		return rc;

	if (ref_status == REF_MON_STATUS_OK &&
	    ZL3073X_REF_IS_VALID(conn_ref) &&
	    ZL3073X_REF_IS_VALID(ref_id) &&
	    conn_ref == ref_id) {
		u64 conn_freq, ref_freq;

		/* Get frequency of connected ref */
		rc = zl3073x_dpll_input_ref_frequency_get(zldev, conn_ref,
							  &conn_freq);
		if (rc)
			return rc;

		/* Get frequency of given ref */
		rc = zl3073x_dpll_input_ref_frequency_get(zldev, ref_id,
							  &ref_freq);
		if (rc)
			return rc;

		if (conn_freq > ref_freq) {
			s64 conn_period_ps;
			int div_factor;

			conn_period_ps = (s64)div_u64(PSEC_PER_SEC, conn_freq);
			div_factor = div64_s64(ref_phase, conn_period_ps);
			ref_phase -= conn_period_ps * div_factor;
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

	guard(zl3073x)(zldev);

	/* Get ref id for the pin */
	ref_id = zl3073x_dpll_pin_ref_id_get(pin->index);

	/* Read reference configuration into mailbox */
	rc = zl3073x_mb_ref_read(zldev, ref_id);
	if (rc)
		return rc;

	/* Read current phase offset compensation */
	rc = zl3073x_read_ref_phase_offset_compensation(zldev, &phase_comp);
	if (rc)
		return rc;

	/* Perform sign extension for 48bit signed value */
	if (phase_comp & BIT_ULL(47))
		phase_comp |= GENMASK_ULL(63, 48);

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

	guard(zl3073x)(zldev);

	/* The value in the register is stored as two's complement negation
	 * of requested value.
	 */
	phase_comp = (s64) -phase_adjust;

	/* Write the requested value into the compensation register */
	rc = zl3073x_write_ref_phase_offset_compensation(zldev, phase_comp);
	if (rc)
		return rc;

	/* Get ref id for the pin */
	ref_id = zl3073x_dpll_pin_ref_id_get(pin->index);

	/* Update reference configuration from mailbox */
	rc = zl3073x_mb_ref_write(zldev, ref_id);
	if (rc)
		return rc;

	return rc;
}

static int
zl3073x_dpll_ref_prio_get(struct zl3073x_dpll_pin *pin, u32 *prio)
{
	struct zl3073x_dpll *zldpll = pin_to_dpll(pin);
	struct zl3073x_dev *zldev = zldpll->mfd;
	u8 ref_id, ref_prio;
	int rc;

	/* Read DPLL configuration into mailbox */
	rc = zl3073x_mb_dpll_read(zldev, zldpll->id);
	if (rc)
		return rc;

	/* Get ref id for the pin */
	ref_id = zl3073x_dpll_pin_ref_id_get(pin->index);

	/* Read ref prio nibble */
	rc = zl3073x_read_dpll_ref_prio(zldev, ref_id / 2, &ref_prio);
	if (rc)
		return rc;

	/* Select nibble according pin type */
	if (ZL3073X_IS_P_PIN(ref_id))
		*prio = FIELD_GET(DPLL_REF_PRIO_REF_P, ref_prio);
	else
		*prio = FIELD_GET(DPLL_REF_PRIO_REF_N, ref_prio);

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
	u8 dpll_mode_refsel, mode, ref_forced;
	u8 ref_id, ref_status;
	int rc;

	guard(zl3073x)(zldev);

	/* Get ref id for the pin */
	ref_id = zl3073x_dpll_pin_ref_id_get(pin->index);

	/* Read ref status */
	rc = zl3073x_read_ref_mon_status(zldev, ref_id, &ref_status);
	if (rc)
		return rc;

	if (ref_status != REF_MON_STATUS_OK) {
		*state = DPLL_PIN_STATE_DISCONNECTED;
		return 0;
	}

	rc = zl3073x_read_dpll_mode_refsel(zldev, zldpll->id,
					   &dpll_mode_refsel);
	if (rc)
		return rc;

	mode = FIELD_GET(DPLL_MODE_REFSEL_MODE, dpll_mode_refsel);
	ref_forced = FIELD_GET(DPLL_MODE_REFSEL_REF, dpll_mode_refsel);

	if (mode == DPLL_MODE_REFSEL_MODE_AUTO) {
		u8 ref_selected;
		u32 ref_prio;

		rc = zl3073x_dpll_selected_ref_get(zldpll, &ref_selected);
		if (rc)
			return rc;

		rc = zl3073x_dpll_ref_prio_get(pin, &ref_prio);
		if (rc)
			return rc;

		if (ref_id == ref_selected)
			*state = DPLL_PIN_STATE_CONNECTED;
		else if (ref_prio != DPLL_REF_PRIO_INVALID)
			*state = DPLL_PIN_STATE_SELECTABLE;
		else
			*state = DPLL_PIN_STATE_DISCONNECTED;
	}
	else if (ref_id == ref_forced) {
		*state = DPLL_PIN_STATE_CONNECTED;
	}
	else {
		*state = DPLL_PIN_STATE_DISCONNECTED;
	}

	return rc;
}

static int
zl3073x_dpll_input_pin_prio_get(const struct dpll_pin *dpll_pin, void *pin_priv,
				const struct dpll_device *dpll, void *dpll_priv,
				u32 *prio, struct netlink_ext_ack *extack)
{
	struct zl3073x_dpll *zldpll = dpll_priv;
	struct zl3073x_dev *zldev = zldpll->mfd;
	struct zl3073x_dpll_pin *pin = pin_priv;
	int rc;

	guard(zl3073x)(zldev);

	rc = zl3073x_dpll_ref_prio_get(pin, prio);
	if (rc)
		return rc;

	return rc;
}

static int
zl3073x_dpll_input_pin_prio_set(const struct dpll_pin *dpll_pin, void *pin_priv,
				const struct dpll_device *dpll, void *dpll_priv,
				u32 prio, struct netlink_ext_ack *extack)
{
	struct zl3073x_dpll *zldpll = dpll_priv;
	struct zl3073x_dev *zldev = zldpll->mfd;
	struct zl3073x_dpll_pin *pin = pin_priv;
	u8 ref_id, ref_prio;
	int rc;

	guard(zl3073x)(zldev);

	/* Read channel configuration into mailbox */
	rc = zl3073x_mb_dpll_read(zldev, zldpll->id);
	if (rc)
		return rc;

	/* Get ref id for the pin */
	ref_id = zl3073x_dpll_pin_ref_id_get(pin->index);

	/* Read the current priority to preserve the other nibble */
	rc = zl3073x_read_dpll_ref_prio(zldev, ref_id, &ref_prio);
	if (rc)
		return rc;

	/* Update the priority */
	if (ZL3073X_IS_P_PIN(ref_id)) {
		ref_prio &= ~DPLL_REF_PRIO_REF_P;
		ref_prio |= FIELD_PREP(DPLL_REF_PRIO_REF_P, prio);
	} else {
		ref_prio &= ~DPLL_REF_PRIO_REF_N;
		ref_prio |= FIELD_PREP(DPLL_REF_PRIO_REF_N, prio);
	}

	/* Write the updated priority value */
	rc = zl3073x_write_dpll_ref_prio(zldev, ref_id, ref_prio);
	if (rc)
		return rc;

	/* Update channel configuration from mailbox */
	rc = zl3073x_mb_dpll_write(zldev, zldpll->id);

	return rc;
}

static u8
zl3073x_dpll_pin_synth_get(struct zl3073x_dpll_pin *pin)
{
	u8 pair = zl3073x_dpll_pin_pair_get(pin->index);

	return zl3073x_output_synth_get(pin_to_dev(pin), pair);
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
	enum zl3073x_output_pair_type pin_type;
	u64 synth_freq;
	u32 output_div;
	u8 synth;
	int rc;

	guard(zl3073x)(zldev);

	synth = zl3073x_dpll_pin_synth_get(pin);
	synth_freq = zl3073x_synth_freq_get(zldev, synth);

	/* Read output configuration into mailbox */
	rc = zl3073x_mb_output_read(zldev, pin->index / 2);
	if (rc)
		return rc;

	/* Get divisor */
	rc = zl3073x_read_output_div(zldev, &output_div);
	if (rc)
		return rc;

	pin_type = zldev->pdata->output_pairs[pin->index  / 2].type;

	switch (pin_type) {
	case ZL3073X_SINGLE_ENDED_DIVIDED:
		if (ZL3073X_IS_P_PIN(pin->index)) {
			/* P-pin */
			*frequency = div_u64(synth_freq, output_div);
		} else {
			/* N-pin */
			u32 esync_period;
			u64 divisor;

			rc = zl3073x_read_output_esync_period(zldev,
							      &esync_period);
			if (rc)
				return rc;

			divisor = mul_u32_u32(output_div, esync_period);
			*frequency = div64_u64(synth_freq, divisor);
		}
		break;
	case ZL3073X_SINGLE_ENDED_IN_PHASE:
	case ZL3073X_DIFFERENTIAL:
		*frequency = div_u64(synth_freq, output_div);
		break;
	default:
		WARN(1, "Unknown output pin type: %u", pin_type);
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
	enum zl3073x_output_pair_type pair_type;
	u32 i, output_div, output_p_freq;
	u64 synth_freq;
	u8 synth;
	int rc;

	/* Do not allow to set frequency on internal oscilator pin type */
	if (pin->props.type == DPLL_PIN_TYPE_INT_OSCILLATOR)
		return -EINVAL;

	guard(zl3073x)(zldev);

	synth = zl3073x_dpll_pin_synth_get(pin);
	synth_freq = zl3073x_synth_freq_get(zldev, synth);

	for (i = 0; i < pin->props.freq_supported_num; i++)
		if (pin->props.freq_supported[i].min <= frequency &&
		    pin->props.freq_supported[i].max >= frequency)
			break;

	if (i == pin->props.freq_supported_num)
		return -EINVAL;

	/* Read output configuration into mailbox */
	rc = zl3073x_mb_output_read(zldev, pin->index / 2);
	if (rc)
		return rc;

	/* Get divisor */
	rc = zl3073x_read_output_div(zldev, &output_div);
	if (rc)
		return rc;

	/* Compute output P frequency */
	output_p_freq = (u32)div_u64(synth_freq, output_div);

	pair_type = zldev->pdata->output_pairs[pin->index  / 2].type;
	switch (pair_type) {
	case ZL3073X_SINGLE_ENDED_DIVIDED: {
		u32 esync_period, output_n_freq;

		/* Compute output N frequency */
		rc = zl3073x_read_output_esync_period(zldev, &esync_period);
		if (rc)
			return rc;
		output_n_freq = output_p_freq / esync_period;

		if (ZL3073X_IS_P_PIN(pin->index)) {
			/* P-pin */
			if (frequency <= output_n_freq)
				return -EINVAL;

			/* Update output divisor */
			output_div = (u32)div_u64(synth_freq, (u32)frequency);
			rc = zl3073x_write_output_div(zldev, output_div);
			if (rc)
				return rc;

			/* output width == output div */
			rc = zl3073x_write_output_width(zldev, output_div);
			if (rc)
				return rc;

			/* Compute new embedded sync period */
			esync_period = (u32)div_u64(frequency, output_n_freq);
		} else {
			/* N-pin */
			if (output_p_freq <= frequency)
				return -EINVAL;

			/* Compute new embedded sync period */
			esync_period = output_p_freq / (u32)frequency;
		}

		/* Update embedded sync period */
		rc = zl3073x_write_output_esync_period(zldev, esync_period);
		if (rc)
			return rc;

		/* Embedded sync width == embedded sync period */
		rc = zl3073x_write_output_esync_width(zldev, esync_period);
		if (rc)
			return rc;

		break;
	}
	case ZL3073X_SINGLE_ENDED_IN_PHASE:
	case ZL3073X_DIFFERENTIAL:
		/* Update output divisor */
		output_div = (u32)div_u64(synth_freq, frequency);
		rc = zl3073x_write_output_div(zldev, output_div);
		if (rc)
			return rc;

		/* output width == output div */
		rc = zl3073x_write_output_width(zldev, output_div);
		if (rc)
			return rc;

		break;
	default:
		WARN(1, "Unknown output pin pair type: %u", pair_type);
		break;
	}

	/* Update output configuration from mailbox */
	rc = zl3073x_mb_output_write(zldev, pin->index / 2);

	return rc;
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
	u64 synth_freq;
	s32 phase_comp;
	u8 synth;
	int rc;

	guard(zl3073x)(zldev);

	synth = zl3073x_dpll_pin_synth_get(pin);
	synth_freq = zl3073x_synth_freq_get(zldev, synth);

	/* Read output configuration into mailbox */
	rc = zl3073x_mb_output_read(zldev, pin->index / 2);
	if (rc)
		return rc;

	/* Read current output phase compensation */
	rc = zl3073x_read_output_phase_compensation(zldev, &phase_comp);
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
	u64 synth_freq;
	int phase_comp;
	u8 synth;
	int rc;

	guard(zl3073x)(zldev);

	/* Get attached synth */
	synth = zl3073x_dpll_pin_synth_get(pin);

	/* Get synth's frequency */
	synth_freq = zl3073x_synth_freq_get(zldev, synth);

	/* Value in register is expressed in half synth clock cycles so
	 * the given phase adjustment a multiple of half synth clock.
	 */
	half_synth_cycle = (int)div_u64(PSEC_PER_SEC, 2 * synth_freq);
	if ((phase_adjust % half_synth_cycle) != 0)
		return -EINVAL;
	phase_adjust /= half_synth_cycle;

	guard(zl3073x)(zldev);

	/* The value in the register is stored as two's complement negation
	 * of requested value.
	 */
	phase_comp = -phase_adjust;

	/* Write the requested value into the compensation register */
	rc = zl3073x_write_output_phase_compensation(zldev, phase_comp);
	if (rc)
		return rc;

	/* Update output configuration from mailbox */
	rc = zl3073x_mb_output_write(zldev, pin->index / 2);

	return rc;
}

static int
zl3073x_dpll_output_pin_state_on_dpll_get(const struct dpll_pin *dpll_pin,
					  void *pin_priv,
					  const struct dpll_device *dpll,
					  void *dpll_priv,
					  enum dpll_pin_state *state,
					  struct netlink_ext_ack *extack)
{
	struct zl3073x_dpll *zldpll = dpll_priv;
	struct zl3073x_dev *zldev = zldpll->mfd;
	struct zl3073x_dpll_pin *pin = pin_priv;
	u8 dpll_sel, synth, synth_ctrl;
	int rc;

	guard(zl3073x)(zldev);

	synth = zl3073x_dpll_pin_synth_get(pin);

	rc = zl3073x_read_synth_ctrl(zldev, synth, &synth_ctrl);
	if (rc)
		return rc;
	dpll_sel = FIELD_GET(SYNTH_CTRL_DPLL_SEL, synth_ctrl);

	if (dpll_sel == zldpll->id)
		*state = DPLL_PIN_STATE_CONNECTED;
	else
		*state = DPLL_PIN_STATE_DISCONNECTED;

	return rc;
}

static int
zl3073x_dpll_lock_status_get(const struct dpll_device *dpll, void *dpll_priv,
			     enum dpll_lock_status *status,
			     enum dpll_lock_status_error *status_error,
			     struct netlink_ext_ack *extack)
{
	struct zl3073x_dpll *zldpll = dpll_priv;
	struct zl3073x_dev *zldev = zldpll->mfd;
	u8 ho_ready, mon_status, refsel_status, state;
	int rc;

	guard(zl3073x)(zldev);

	rc = zl3073x_read_dpll_state_refsel_status(zldev, zldpll->id,
						   &refsel_status);
	if (rc)
		return rc;
	state = FIELD_GET(DPLL_STATE_REFSEL_STATUS_STATE, refsel_status);

	rc = zl3073x_read_dpll_mon_status(zldev, zldpll->id,
					  &mon_status);
	if (rc)
		return rc;
	ho_ready = FIELD_GET(DPLL_MON_STATUS_HO_READY, mon_status);

	switch (state) {
		case DPLL_STATE_REFSEL_STATUS_STATE_FREERUN:
		case DPLL_STATE_REFSEL_STATUS_STATE_FASTLOCK:
		case DPLL_STATE_REFSEL_STATUS_STATE_ACQUIRING:
			*status = DPLL_LOCK_STATUS_UNLOCKED;
			break;
		case DPLL_STATE_REFSEL_STATUS_STATE_HOLDOVER:
			*status = DPLL_LOCK_STATUS_HOLDOVER;
			break;
		case DPLL_STATE_REFSEL_STATUS_STATE_LOCK:
			if (ho_ready)
				*status = DPLL_LOCK_STATUS_LOCKED_HO_ACQ;
			else
				*status = DPLL_LOCK_STATUS_LOCKED;
			break;
	}

	return rc;
}

static int
zl3073x_dpll_mode_get(const struct dpll_device *dpll, void *dpll_priv,
		      enum dpll_mode *mode, struct netlink_ext_ack *extack)
{
	struct zl3073x_dpll *zldpll = dpll_priv;
	struct zl3073x_dev *zldev = zldpll->mfd;
	u8 hw_mode, mode_refsel;
	int rc;

	guard(zl3073x)(zldev);

	rc = zl3073x_read_dpll_mode_refsel(zldev, zldpll->id,
					   &mode_refsel);
	if (rc)
		return rc;
	hw_mode = FIELD_GET(DPLL_MODE_REFSEL_MODE, mode_refsel);

	switch (hw_mode) {
	case DPLL_MODE_REFSEL_MODE_HOLDOVER:
	case DPLL_MODE_REFSEL_MODE_REFLOCK:
		/* Use MANUAL for device HOLDOVER and REFLOCK modes */
		*mode = DPLL_MODE_MANUAL;
		break;
	case DPLL_MODE_REFSEL_MODE_NCO:
	case DPLL_MODE_REFSEL_MODE_AUTO:
		/* Use AUTO for device AUTO mode */
		*mode = DPLL_MODE_AUTOMATIC;
		break;
	case DPLL_MODE_REFSEL_MODE_FREERUN:
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct dpll_pin_ops zl3073x_dpll_input_pin_ops = {
	.direction_get = zl3073x_dpll_pin_direction_get,
	.frequency_get = zl3073x_dpll_input_pin_frequency_get,
	.frequency_set = zl3073x_dpll_input_pin_frequency_set,
	.phase_offset_get = zl3073x_dpll_input_pin_phase_offset_get,
	.phase_adjust_get = zl3073x_dpll_input_pin_phase_adjust_get,
	.phase_adjust_set = zl3073x_dpll_input_pin_phase_adjust_set,
	.prio_get = zl3073x_dpll_input_pin_prio_get,
	.prio_set = zl3073x_dpll_input_pin_prio_set,
	.state_on_dpll_get = zl3073x_dpll_input_pin_state_on_dpll_get,
};

static const struct dpll_pin_ops zl3073x_dpll_output_pin_ops = {
	.direction_get = zl3073x_dpll_pin_direction_get,
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

static void
zl3073x_dpll_fill_pin_properties(struct zl3073x_dpll_pin *pin)
{
	struct dpll_pin_properties *props = &pin->props;
	struct zl3073x_dev *zldev = pin_to_dev(pin);
	const struct zl3073x_pin_prop *pin_prop;
	struct dpll_pin_frequency *freq;
	unsigned long capabilities;
	u8 idx = pin->index;
	u32 num_freq;

	memset(props, 0, sizeof(*props));

	if (ZL3073X_IS_INPUT_PIN(idx)) {
		pin_prop = &zldev->pdata->input_pins[idx - ZL3073X_NUM_OPINS];
		capabilities = DPLL_PIN_CAPABILITIES_PRIORITY_CAN_CHANGE;
		num_freq = ARRAY_SIZE(input_freq_ranges);
		freq = input_freq_ranges;
	} else {
		u8 pair = zl3073x_dpll_pin_pair_get(pin->index);

		pin_prop = &zldev->pdata->output_pins[idx];
		capabilities = 0;

		switch (zldev->pdata->output_pairs[pair].freq_type) {
		case ZL3073X_SYNCE:
			num_freq = ARRAY_SIZE(output_freq_ranges_synce);
			freq = output_freq_ranges_synce;
			break;
		case ZL3073X_PTP:
			num_freq = ARRAY_SIZE(output_freq_ranges_ptp);
			freq = output_freq_ranges_ptp;
			break;
		case ZL3073X_25MHZ_FIXED:
			num_freq = ARRAY_SIZE(output_freq_ranges_25mhz);
			freq = output_freq_ranges_25mhz;
			break;
		case ZL3073X_10MHZ_FIXED_EPPS:
			num_freq = ARRAY_SIZE(output_freq_ranges_10mhz);
			freq = output_freq_ranges_10mhz;
			break;
		case ZL3073X_SYNCE_1HZ_FIXED:
		case ZL3073X_1HZ_FIXED:
			num_freq = ARRAY_SIZE(output_freq_ranges_1hz);
			freq = output_freq_ranges_1hz;
			break;
		}
	}

	pin->props.board_label = pin_prop->name;
	pin->props.capabilities = capabilities;
	pin->props.type = pin_prop->type;
	pin->props.freq_supported_num = num_freq;
	pin->props.freq_supported = freq;
	pin->props.phase_range.min = S32_MIN;
	pin->props.phase_range.max = S32_MAX;
}

static int
zl3073x_dpll_pin_register(struct zl3073x_dpll_pin *pin)
{
	struct zl3073x_dpll *zldpll = pin_to_dpll(pin);
	const struct dpll_pin_ops *ops;
	int rc;

	/* Fill pin properties */
	zl3073x_dpll_fill_pin_properties(pin);

	/* Create DPLL pin */
	pin->dpll_pin = dpll_pin_get(zldpll->mfd->clock_id, pin->index,
				     THIS_MODULE, &pin->props);
	if (IS_ERR(pin->dpll_pin))
		return PTR_ERR(pin->dpll_pin);

	if (ZL3073X_IS_INPUT_PIN(pin->index))
		ops = &zl3073x_dpll_input_pin_ops;
	else
		ops = &zl3073x_dpll_output_pin_ops;

	/* Register the pin */
	rc = dpll_pin_register(zldpll->dpll_dev, pin->dpll_pin, ops, pin);
	if (rc)
		goto err_register;

	return 0;

err_register:
	dpll_pin_put(pin->dpll_pin);
	pin->dpll_pin = NULL;

	return rc;
}

static void
zl3073x_dpll_pin_unregister(struct zl3073x_dpll_pin *pin)
{
	struct zl3073x_dpll *zldpll = pin_to_dpll(pin);
	const struct dpll_pin_ops *ops;

	if (IS_ERR_OR_NULL(pin->dpll_pin))
		return;

	if (ZL3073X_IS_INPUT_PIN(pin->index))
		ops = &zl3073x_dpll_input_pin_ops;
	else
		ops = &zl3073x_dpll_output_pin_ops;

	/* Unregister the pin */
	dpll_pin_unregister(zldpll->dpll_dev, pin->dpll_pin, ops, pin);

	dpll_pin_put(pin->dpll_pin);
	pin->dpll_pin = NULL;
}

static void
zl3073x_dpll_unregister_pins(struct zl3073x_dpll *zldpll)
{
	int i;

	for (i = 0; i < ZL3073X_NUM_PINS; i++)
		zl3073x_dpll_pin_unregister(&zldpll->pins[i]);
}

static int
zl3073x_dpll_register_pins(struct zl3073x_dpll *zldpll)
{
	int i, rc;

	for (i = 0; i < ZL3073X_NUM_PINS; i++) {
		zldpll->pins[i].index = i;
		rc = zl3073x_dpll_pin_register(&zldpll->pins[i]);
		if (rc)
			goto err_register;
	}

	return 0;

err_register:
	while (i--)
		zl3073x_dpll_pin_unregister(&zldpll->pins[i]);

	return rc;
}

static int
zl3073x_dpll_register(struct zl3073x_dpll *zldpll)
{
	struct zl3073x_dev *zldev = zldpll->mfd;
	int rc;

	zldpll->dpll_dev = dpll_device_get(zldev->clock_id, zldpll->id,
					   THIS_MODULE);
	if (IS_ERR(zldpll->dpll_dev))
		return PTR_ERR(zldpll->dpll_dev);

	rc = dpll_device_register(zldpll->dpll_dev,
				  zldev->pdata->dpll_types[zldpll->id],
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
	int i, rc;

	/* Get current lock status for i-th DPLL */
	rc = zl3073x_dpll_lock_status_get(zldpll->dpll_dev, zldpll,
					  &lock_status, NULL, NULL);
	if (rc) {
		dev_err_probe(zldpll->mfd->dev, rc,
			      "Failed to get DPLL lock status");
		goto out;
	}

	/* If lock status was changed then notify DPLL core */
	if (zldpll->lock_status != lock_status) {
		zldpll->lock_status = lock_status;
		dpll_device_change_ntf(zldpll->dpll_dev);
	}

	/* Output pins change checks are not necessary because output states
	 * are constant.
	 */
	for (i = 0; i < ZL3073X_NUM_IPINS; i++) {
		struct zl3073x_dpll_pin *pin;
		enum dpll_pin_state state;
		s64 phase_offset;
		bool pin_changed;

		/* Input pins starts are stored after output pins */
		pin = &zldpll->pins[ZL3073X_NUM_OPINS + i];

		rc = zl3073x_dpll_input_pin_state_on_dpll_get(pin->dpll_pin,
							      pin,
							      zldpll->dpll_dev,
							      zldpll, &state,
							      NULL);
		if (rc)
			goto out;

		rc = zl3073x_dpll_input_pin_phase_offset_get(pin->dpll_pin,
							     pin,
							     zldpll->dpll_dev,
							     zldpll,
							     &phase_offset,
							     NULL);
		if (rc)
			goto out;

		pin_changed =
			(state != pin->pin_state) ||
			(phase_offset != pin->phase_offset);

		pin->pin_state = state;
		pin->phase_offset = phase_offset;

		if (pin_changed)
			dpll_pin_change_ntf(pin->dpll_pin);
	}

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
	kworker = kthread_create_worker(0, "zl3073x-%s", dev_name(zldpll->dev));
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

	guard(zl3073x)(zldpll->mfd);

	rc = zl3073x_write_synth_phase_shift_mask(zldev, 0x1f);
	if (rc)
		return rc;

	rc = zl3073x_write_synth_phase_shift_intvl(zldev, 0x01);
	if (rc)
		return rc;

	rc = zl3073x_write_synth_phase_shift_data(zldev, 0xffff);
	if (rc)
		return rc;

	rc = zl3073x_write_synth_phase_shift_ctrl(zldev, 0x01);
	if (rc)
		return rc;

	return rc;
}

static int
zl3073x_dpll_probe(struct platform_device *pdev)
{
	struct zl3073x_dpll *zldpll;
	int rc;

	zldpll = devm_kzalloc(&pdev->dev, sizeof(*zldpll), GFP_KERNEL);
	if (!zldpll)
		return -ENOMEM;

	zldpll->dev = &pdev->dev;
	zldpll->mfd = dev_get_drvdata(pdev->dev.parent);
	zldpll->id = pdev->mfd_cell->id;

	rc = zl3073x_dpll_init(zldpll);
	if (rc)
		return rc;

	rc = zl3073x_dpll_init_worker(zldpll);
	if (rc)
		return rc;

	platform_set_drvdata(pdev, zldpll);

	/* Initial firmware fine phase correction */
	rc = zl3073x_dpll_init_fine_phase_adjust(zldpll);

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
		.name = "zl3073x-dpll",
	},
	.probe = zl3073x_dpll_probe,
	.remove	= zl3073x_dpll_remove,
	.id_table = zl3073x_dpll_platform_id,
};

module_platform_driver(zl3073x_dpll_driver);

MODULE_AUTHOR("Ivan Vecera <ivecera@redhat.com>");
MODULE_AUTHOR("Tariq Haddad <tariq.haddad@microchip.com>");
MODULE_DESCRIPTION("Microchip ZL3073x DPLL driver");
MODULE_LICENSE("GPL");
