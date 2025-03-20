/* SPDX-License-Identifier: GPL-2.0+ */

#include <linux/bitfield.h>
#include <linux/dpll.h>
#include <linux/mfd/core.h>
#include <linux/mfd/zl3073x.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>
#include <linux/property.h>

/*
 * Register Map Page 2, Status
 */
ZL3073X_REG8_IDX_DEF(ref_mon_status,		0x102,
						ZL3073X_NUM_INPUT_PINS, 1);
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

ZL3073X_REG8_IDX_DEF(dpll_refsel_status,	0x130, ZL3073X_NUM_CHANNELS, 1);
#define DPLL_REFSEL_STATUS_REFSEL		GENMASK(3, 0)
#define DPLL_REFSEL_STATUS_STATE		GENMASK(6, 4)
#define DPLL_REFSEL_STATUS_STATE_FREERUN	0
#define DPLL_REFSEL_STATUS_STATE_HOLDOVER	1
#define DPLL_REFSEL_STATUS_STATE_FASTLOCK	2
#define DPLL_REFSEL_STATUS_STATE_ACQUIRING	3
#define DPLL_REFSEL_STATUS_STATE_LOCK		4

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

/*
 * Register Map Page 9, Synth and Output
 */
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

/*
 * Register Map Page 12, DPLL Mailbox
 */
ZL3073X_REG8_IDX_DEF(dpll_ref_prio,		0x652, ZL3073X_NUM_INPUT_PINS/2, 1);
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
	char				package_label[8];
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

/**
 * zl3073x_dpll_is_input_pin - check if the pin is input one
 * @pin: pin to check
 *
 * Returns true if the pin is input or false if output one.
 */
static inline
bool zl3073x_dpll_is_input_pin(struct zl3073x_dpll_pin *pin)
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
 * Returns index of the pin from the HW point of view.
 */
static inline
u8 zl3073x_dpll_pin_index_get(struct zl3073x_dpll_pin *pin)
{
	if (zl3073x_dpll_is_input_pin(pin))
		return pin->index - ZL3073X_NUM_OUTPUT_PINS;

	return pin->index;
}

/**
 * zl3073x_dpll_is_n_pin - check if the pin is N-pin
 * @pin: pin to check
 *
 * Returns true if the pin is N-pin or false if output one.
 */
static inline
bool zl3073x_dpll_is_n_pin(struct zl3073x_dpll_pin *pin)
{
	/* P-pins indices are even while N-pins are odd */
	return zl3073x_is_n_pin(zl3073x_dpll_pin_index_get(pin));
}

/**
 * zl3073x_dpll_is_p_pin - check if the pin is P-pin
 * @pin: pin to check
 *
 * Returns true if the pin is P-pin or false if output one.
 */
static inline
bool zl3073x_dpll_is_p_pin(struct zl3073x_dpll_pin *pin)
{
	return zl3073x_is_p_pin(zl3073x_dpll_pin_index_get(pin));
}

/**
 * zl3073x_dpll_output_pin_output_get - get output index for given output pin
 * @pin: pointer to pin
 *
 * Returns output index for the given output pin
 */
static inline
u8 zl3073x_dpll_output_pin_output_get(struct zl3073x_dpll_pin *pin)
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
 * @base_freq: base frequency
 * @mult: multiplier
 *
 * Checks if the given frequency can be factorized using one of the
 * supported base frequencies. If so the base frequency and multiplier
 * are stored into appropriate parameters if they are not NULL and
 * returns 0. If the frequency cannot be factorized then the function
 * returns -EINVAL.
 */
static int
zl3073x_dpll_input_ref_frequency_factorize(u64 freq, u16 *base, u16 *mult)
{
	static const u16 base_freqs[] = {
		1, 2, 4, 5, 8, 10, 16, 20, 25, 32, 40, 50, 64, 80, 100, 125,
		128, 160, 200, 250, 256, 320, 400, 500, 625, 640, 800, 1000,
		1250, 1280, 1600, 2000, 2500, 3125, 3200, 4000, 5000, 6250,
		6400, 8000, 10000, 12500, 15625, 16000, 20000, 25000, 31250,
		32000, 40000, 50000, 62500,
	};
	u32 div, rem;
	int i;

	for (i = 0; i < ARRAY_SIZE(base_freqs); i++) {
		div = div_u64_rem(freq, base_freqs[i], &rem);
		if (!rem && div <= U16_MAX) {
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
	u16 base_freq, mult, num, denom;
	int rc;

	/* Read reference configuration into mailbox */
	rc = zl3073x_mb_ref_read(zldev, ref_id);
	if (rc)
		return rc;

	/* Read base frequency */
	rc = zl3073x_read_ref_freq_base(zldev, &base_freq);
	if (rc)
		return rc;

	/* Read multiplier */
	rc = zl3073x_read_ref_freq_mult(zldev, &mult);
	if (rc)
		return rc;

	/* Write numerator */
	rc = zl3073x_read_ref_ratio_m(zldev, &num);
	if (rc)
		return rc;

	/* Write denominator */
	rc = zl3073x_read_ref_ratio_n(zldev, &denom);
	if (rc)
		return rc;

	*frequency = mul_u64_u32_div(base_freq * mult, num, denom);

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

	/* Get index of the pin */
	ref_id = zl3073x_dpll_pin_index_get(pin);

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
	u16 base_freq, mult;
	u8 ref_id;
	int rc;

	rc = zl3073x_dpll_input_ref_frequency_factorize(frequency, &base_freq,
							&mult);
	if (rc)
		return -EINVAL;

	/* Take device lock */
	guard(zl3073x)(zldev);

	/* Write base frequency */
	rc = zl3073x_write_ref_freq_base(zldev, base_freq);
	if (rc)
		return rc;

	/* Write multiplier */
	rc = zl3073x_write_ref_freq_mult(zldev, mult);
	if (rc)
		return rc;

	/* Write numerator */
	rc = zl3073x_write_ref_ratio_m(zldev, 1);
	if (rc)
		return rc;

	/* Write denominator */
	rc = zl3073x_write_ref_ratio_n(zldev, 1);
	if (rc)
		return rc;

	/* Get index of the pin */
	ref_id = zl3073x_dpll_pin_index_get(pin);

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

	/* Get index of the pin */
	ref_id = zl3073x_dpll_pin_index_get(pin);

	/* Read ref prio nibble */
	rc = zl3073x_read_dpll_ref_prio(zldev, ref_id / 2, &ref_prio);
	if (rc)
		return rc;

	/* Select nibble according pin type */
	if (zl3073x_dpll_is_p_pin(pin))
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

	/* Get index of the pin */
	ref_id = zl3073x_dpll_pin_index_get(pin);

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
		u8 refsel_status, ref_selected;
		u32 ref_prio;

		rc = zl3073x_read_dpll_refsel_status(zldev, zldpll->id,
						     &refsel_status);
		if (rc)
			return rc;

		ref_selected = FIELD_GET(DPLL_REFSEL_STATUS_REFSEL,
					 refsel_status);

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

	/* Get index of the pin */
	ref_id = zl3073x_dpll_pin_index_get(pin);

	/* Read the current priority to preserve the other nibble */
	rc = zl3073x_read_dpll_ref_prio(zldev, ref_id / 2, &ref_prio);
	if (rc)
		return rc;

	/* Update the priority */
	if (zl3073x_dpll_is_p_pin(pin)) {
		ref_prio &= ~DPLL_REF_PRIO_REF_P;
		ref_prio |= FIELD_PREP(DPLL_REF_PRIO_REF_P, prio);
	} else {
		ref_prio &= ~DPLL_REF_PRIO_REF_N;
		ref_prio |= FIELD_PREP(DPLL_REF_PRIO_REF_N, prio);
	}

	/* Write the updated priority value */
	rc = zl3073x_write_dpll_ref_prio(zldev, ref_id / 2 , ref_prio);
	if (rc)
		return rc;

	/* Update channel configuration from mailbox */
	rc = zl3073x_mb_dpll_write(zldev, zldpll->id);

	return rc;
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
	u8 output, signal_format, synth;
	u64 synth_freq;
	u32 output_div;
	int rc;

	guard(zl3073x)(zldev);

	output = zl3073x_dpll_output_pin_output_get(pin);
	synth = zl3073x_dpll_pin_synth_get(pin);
	synth_freq = zl3073x_synth_freq_get(zldev, synth);

	/* Read output configuration into mailbox */
	rc = zl3073x_mb_output_read(zldev, output);
	if (rc)
		return rc;

	/* Get divisor */
	rc = zl3073x_read_output_div(zldev, &output_div);
	if (rc)
		return rc;

	/* Read used signal format for the given output */
	signal_format = zl3073x_output_signal_format_get(zldev, output);

	switch (signal_format) {
	case OUTPUT_MODE_SIGNAL_FORMAT_TWO_N_DIV:
	case OUTPUT_MODE_SIGNAL_FORMAT_TWO_N_DIV_INV:
		/* In case of divided format we have to distiguish between
		 * given output pin type.
		 */
		if (zl3073x_dpll_is_p_pin(pin)) {
			/* For P-pin the resulting frequency is computed as
			 * simple division of synth frequency and output
			 * divisor.
			 */
			*frequency = div_u64(synth_freq, output_div);
		} else {
			/* For N-pin we have to divide additionally by
			 * divisor stored in output_esync_period register
			 * that is used as N-pin divisor for these modes.
			 */
			u64 divisor;
			u32 period;

			rc = zl3073x_read_output_esync_period(zldev, &period);
			if (rc)
				return rc;

			/* Compute final divisor for N-pin */
			divisor = mul_u32_u32(output_div, period);
			*frequency = div64_u64(synth_freq, divisor);
		}
		break;
	default:
		/* In other modes the resulting frequency is computed as
		 * division of synth frequency and output divisor.
		 */
		*frequency = div_u64(synth_freq, output_div);
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
	u32 i, output_div, output_p_freq;
	u8 output, signal_format, synth;
	u64 synth_freq;
	int rc;

	/* Do not allow to set frequency on internal oscilator pin type */
	if (pin->props.type == DPLL_PIN_TYPE_INT_OSCILLATOR)
		return -EINVAL;

	guard(zl3073x)(zldev);

	output = zl3073x_dpll_output_pin_output_get(pin);
	synth = zl3073x_dpll_pin_synth_get(pin);
	synth_freq = zl3073x_synth_freq_get(zldev, synth);

	for (i = 0; i < pin->props.freq_supported_num; i++)
		if (pin->props.freq_supported[i].min <= frequency &&
		    pin->props.freq_supported[i].max >= frequency)
			break;

	if (i == pin->props.freq_supported_num)
		return -EINVAL;

	/* Read output configuration into mailbox */
	rc = zl3073x_mb_output_read(zldev, output);
	if (rc)
		return rc;

	/* Get divisor */
	rc = zl3073x_read_output_div(zldev, &output_div);
	if (rc)
		return rc;

	/* Compute current output frequency for P-pin */
	output_p_freq = (u32)div_u64(synth_freq, output_div);

	/* Read used signal format for the given output */
	signal_format = zl3073x_output_signal_format_get(zldev, output);

	switch (signal_format) {
	case OUTPUT_MODE_SIGNAL_FORMAT_TWO_N_DIV:
	case OUTPUT_MODE_SIGNAL_FORMAT_TWO_N_DIV_INV: {
		/* For N-pin divided formats we have to find the divisor
		 * common for both P & N pins but also the N-pin divisor
		 * to keep frequency of N-pin unchanged after change of
		 * the common divisor.
		 */
		u32 period, output_n_freq;

		/* Read N-pin divisor and compute current output frequency
		 * for N-pin
		 */
		rc = zl3073x_read_output_esync_period(zldev, &period);
		if (rc)
			return rc;
		output_n_freq = output_p_freq / period;

		if (zl3073x_dpll_is_p_pin(pin)) {
			/* We are going to change output frequency for P-pin
			 * but if the requested frequency is less than current
			 * N-pin frequency then indicate a failure as we are
			 * not able to compute N-pin divisor to keep its
			 * frequency unchanged.
			 */
			if (frequency <= output_n_freq)
				return -EINVAL;

			/* Compute new common output divisor value and update
			 * the register.
			 */
			output_div = (u32)div_u64(synth_freq, (u32)frequency);
			rc = zl3073x_write_output_div(zldev, output_div);
			if (rc)
				return rc;

			/* For 50/50 duty cycle the divisor is equal to width */
			rc = zl3073x_write_output_width(zldev, output_div);
			if (rc)
				return rc;

			/* Compute new divisor for N-pin */
			period = (u32)div_u64(frequency, output_n_freq);
		} else {
			/* We are going to change frequency of N-pin but if the
			 * requested freq is greater or equal than freq of
			 * P-pin in the output pair we cannot compute divisor
			 * for the N-pin. In this case indicate a failure.
			 */
			if (output_p_freq <= frequency)
				return -EINVAL;

			/* Compute new divisor for N-pin */
			period = output_p_freq / (u32)frequency;
		}

		/* Update divisor for the N-pin */
		rc = zl3073x_write_output_esync_period(zldev, period);
		if (rc)
			return rc;

		/* For 50/50 duty cycle the divisor is equal to width */
		rc = zl3073x_write_output_esync_width(zldev, period);
		if (rc)
			return rc;

		break;
	}
	default:
		/* In other modes the resulting frequency is computed as
		 * division of synth frequency and output divisor.
		 *
		 * So compute output divisor and update the register
		 */
		output_div = (u32)div_u64(synth_freq, frequency);
		rc = zl3073x_write_output_div(zldev, output_div);
		if (rc)
			return rc;

		/* For 50/50 duty cycle the divisor is equal to width */
		rc = zl3073x_write_output_width(zldev, output_div);
		if (rc)
			return rc;

		break;
	}

	/* Update output configuration from mailbox */
	rc = zl3073x_mb_output_write(zldev, output);

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
	u8 ho_ready, mon_status, refsel_status, state;
	int rc;

	guard(zl3073x)(zldev);

	rc = zl3073x_read_dpll_refsel_status(zldev, zldpll->id, &refsel_status);
	if (rc)
		return rc;
	state = FIELD_GET(DPLL_REFSEL_STATUS_STATE, refsel_status);

	rc = zl3073x_read_dpll_mon_status(zldev, zldpll->id,
					  &mon_status);
	if (rc)
		return rc;
	ho_ready = FIELD_GET(DPLL_MON_STATUS_HO_READY, mon_status);

	switch (state) {
		case DPLL_REFSEL_STATUS_STATE_FREERUN:
		case DPLL_REFSEL_STATUS_STATE_FASTLOCK:
		case DPLL_REFSEL_STATUS_STATE_ACQUIRING:
			*status = DPLL_LOCK_STATUS_UNLOCKED;
			break;
		case DPLL_REFSEL_STATUS_STATE_HOLDOVER:
			*status = DPLL_LOCK_STATUS_HOLDOVER;
			break;
		case DPLL_REFSEL_STATUS_STATE_LOCK:
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
	case DPLL_MODE_REFSEL_MODE_FREERUN:
	case DPLL_MODE_REFSEL_MODE_HOLDOVER:
	case DPLL_MODE_REFSEL_MODE_NCO:
	case DPLL_MODE_REFSEL_MODE_REFLOCK:
		/* Use MANUAL for device FREERUN, HOLDOVER, NCO and
		 * REFLOCK modes
		 */
		*mode = DPLL_MODE_MANUAL;
		break;
	case DPLL_MODE_REFSEL_MODE_AUTO:
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
 * pin: pointer to pin structure
 *
 * The caller is responsible for calling fwnode_handle_put() on the returned
 * fwnode pointer.
 *
 * Returns the firmware node for the given pin if it is present or
 * NULL if it is missing.
 */
static struct fwnode_handle *
zl3073x_dpll_pin_fwnode_get(struct zl3073x_dpll_pin *pin)
{
	struct zl3073x_dpll *zldpll = pin_to_dpll(pin);
	struct fwnode_handle *pins_node, *pin_node;
	const char *node_name;
	u8 idx;

	if (zl3073x_dpll_is_input_pin(pin)) {
		node_name = "input-pins";
	} else {
		node_name = "output-pins";
	}

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

	if (pin_node)
		dev_dbg(zldpll->mfd->dev, "fwnode for %s pin %u: %pfw\n",
			zl3073x_dpll_is_input_pin(pin) ? "input" : "output",
			idx, pin_node);

	return pin_node;
}

/**
 * zl3073x_dpll_fill_pin_properties_from_fw - fill properties from firmware node
 * @pin: Pin whose properties are filled
 *
 * Gets firmware node for the given pin, enumerate its properties and use their
 * values to initialize given pin properties.
 */
static void
zl3073x_dpll_fill_pin_properties_from_fw(struct zl3073x_dpll_pin *pin)
{
	struct dpll_pin_properties *props = &pin->props;
	struct zl3073x_dpll *zldpll = pin_to_dpll(pin);
	struct fwnode_handle *node;
	int len;

	/* Get firmware node for the given pin */
	node = zl3073x_dpll_pin_fwnode_get(pin);
	if (!node)
		return;

	/* Look for label property and store the value as board label */
	fwnode_property_read_string(node, "label", &props->board_label);

	/* Read supported frequencies property if they are specified */
	len = fwnode_property_count_u64(node, "freqs-hz");
	if (len > 0) {
		u64 *freqs;
		int i;

		freqs = kcalloc(len, sizeof(u64), GFP_KERNEL);
		if (!freqs)
			goto finish;

		fwnode_property_read_u64_array(node, "freqs-hz", freqs, len);

		props->freq_supported = devm_kcalloc(zldpll->mfd->dev, len,
						     sizeof(u64), GFP_KERNEL);
		if (!props->freq_supported) {
			kfree(freqs);
			goto finish;
		}
		props->freq_supported_num = len;

		for (i = 0; i < len; i++) {
			struct dpll_pin_frequency freq =
				DPLL_PIN_FREQUENCY(freqs[i]);

			props->freq_supported[i] = freq;
		}

		kfree(freqs);
	}

finish:
	/* Release firmware node */
	fwnode_handle_put(node);
}

static void
zl3073x_dpll_fill_pin_package_label(struct zl3073x_dpll_pin *pin)
{
	char suffix = zl3073x_dpll_is_p_pin(pin) ? 'P' : 'N';
	struct zl3073x_dev *zldev = pin_to_dpll(pin)->mfd;
	u8 idx;

	if (zl3073x_dpll_is_input_pin(pin)) {
		idx = zl3073x_dpll_pin_index_get(pin);
		if (zl3073x_input_is_diff(zldev, idx))
			snprintf(pin->package_label, sizeof(pin->package_label),
				 "REF%u", idx / 2);
		else
			snprintf(pin->package_label, sizeof(pin->package_label),
				 "REF%u%c", idx / 2, suffix);
	} else {
		idx = zl3073x_dpll_output_pin_output_get(pin);

		switch (zl3073x_output_signal_format_get(zldev, idx)) {
		case OUTPUT_MODE_SIGNAL_FORMAT_LVDS:
		case OUTPUT_MODE_SIGNAL_FORMAT_DIFFERENTIAL:
		case OUTPUT_MODE_SIGNAL_FORMAT_LOWVCM:
			/* Differential formats */
			snprintf(pin->package_label, sizeof(pin->package_label),
				 "OUT%u", idx);
			break;
		default:
			snprintf(pin->package_label, sizeof(pin->package_label),
				 "OUT%u%c", idx, suffix);
			break;
		}
	}

	pin->props.package_label = pin->package_label;
}

static void
zl3073x_dpll_fill_pin_properties(struct zl3073x_dpll_pin *pin)
{
	struct dpll_pin_properties *props = &pin->props;

	memset(props, 0, sizeof(*props));

	if (zl3073x_dpll_is_input_pin(pin)) {
		props->type = DPLL_PIN_TYPE_EXT;
		props->capabilities = DPLL_PIN_CAPABILITIES_PRIORITY_CAN_CHANGE;
	} else {
		props->type = DPLL_PIN_TYPE_GNSS;
	}

	zl3073x_dpll_fill_pin_package_label(pin);

	pin->props.phase_range.min = S32_MIN;
	pin->props.phase_range.max = S32_MAX;

	/* Fill properties from corresponding firmware node if it is present */
	zl3073x_dpll_fill_pin_properties_from_fw(pin);
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

	if (zl3073x_dpll_is_input_pin(pin))
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
	u8 ref;

	/* Get index of the pin */
	ref = zl3073x_dpll_pin_index_get(pin);

	/* If the ref is differential then register only for the P-pin */
	if (zl3073x_input_is_diff(zldev, ref) && zl3073x_dpll_is_n_pin(pin)) {
		dev_dbg(zldev->dev,
			"Input pin %u is differential, skipping N-pin\n",
			pin->index);
		return 0;
	}

	/* If the ref is disabled then skip registration */
	if (!zl3073x_input_is_enabled(zldev, ref)) {
		dev_dbg(zldev->dev, "Input pin %u is disabled\n", pin->index);
		return 0;
	}


	/* Register the pin */
	return zl3073x_dpll_pin_register(pin);
}

static int
zl3073x_dpll_register_output_pin(struct zl3073x_dpll_pin *pin)
{
	struct zl3073x_dpll *zldpll = pin_to_dpll(pin);
	struct zl3073x_dev *zldev = zldpll->mfd;
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
		dev_dbg(zldev->dev,
			"Output %u is associated with different channel\n",
			output);
		return 0;
	}

	/* If the output is disabled then skip registration */
	if (!zl3073x_output_is_enabled(zldev, output)) {
		dev_dbg(zldev->dev, "Output %u is disabled\n", output);
		return 0;
	}

	/* Check ouput's signal format */
	switch (zldev->output[output].signal_format) {
	case OUTPUT_MODE_SIGNAL_FORMAT_DISABLED:
		/* Output is disabled, nothing to register */
		dev_dbg(zldev->dev, "Output %u is disabled\n", output);
		return 0;

	case OUTPUT_MODE_SIGNAL_FORMAT_LVDS:
	case OUTPUT_MODE_SIGNAL_FORMAT_DIFFERENTIAL:
	case OUTPUT_MODE_SIGNAL_FORMAT_LOWVCM:
		/* Output is differential, skip registration for N-pin */
		if (zl3073x_dpll_is_n_pin(pin)) {
			dev_dbg(zldev->dev,
				"Output %u is differential, skipping N-pin\n",
				output);
			return 0;
		}
		break;

	case OUTPUT_MODE_SIGNAL_FORMAT_TWO:
	case OUTPUT_MODE_SIGNAL_FORMAT_TWO_INV:
	case OUTPUT_MODE_SIGNAL_FORMAT_TWO_N_DIV:
	case OUTPUT_MODE_SIGNAL_FORMAT_TWO_N_DIV_INV:
		/* Output is two single ended outputs, continue with
		 * registration.
		 */
		break;

	case OUTPUT_MODE_SIGNAL_FORMAT_ONE_P:
		/* Output is one single ended P-pin output */
		if (zl3073x_dpll_is_n_pin(pin)) {
			dev_dbg(zldev->dev,
				"Output %u is P-pin only, skipping N-pin\n",
				output);
			return 0;
		}
		break;
	case OUTPUT_MODE_SIGNAL_FORMAT_ONE_N:
		/* Output is one single ended N-pin output */
		if (zl3073x_dpll_is_p_pin(pin)) {
			dev_dbg(zldev->dev,
				"Output %u is N-pin only, skipping P-pin\n",
				output);
			return 0;
		}
		break;
	default:
		dev_warn(zldev->dev, "Unknown output mode signal format: %u\n",
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
	const char *types[ZL3073X_NUM_CHANNELS];
	enum dpll_type type;
	int rc;

	/* Set default */
	type = DPLL_TYPE_PPS;

	/* Read dpll types property from firmware */
	rc = device_property_read_string_array(zldpll->mfd->dev,
					       "microchip,dpll-types", types,
					       ARRAY_SIZE(types));
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
	int rc;

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
	struct zl3073x_dev *zldev = zldpll->mfd;
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
	for (i = 0; i < ZL3073X_NUM_INPUT_PINS; i++) {
		struct zl3073x_dpll_pin *pin;
		enum dpll_pin_state state;

		/* Input pins starts are stored after output pins */
		pin = &zldpll->pins[ZL3073X_NUM_OUTPUT_PINS + i];

		/* Skip non-registered pins */
		if (!pin->dpll_pin)
			continue;

		rc = zl3073x_dpll_input_pin_state_on_dpll_get(pin->dpll_pin,
							      pin,
							      zldpll->dpll_dev,
							      zldpll, &state,
							      NULL);
		if (rc)
			goto out;

		if (state != pin->pin_state) {
			dev_dbg(zldev->dev, "Pin %u state changed to %u\n",
				pin->index, state);
			pin->pin_state = state;
			dpll_pin_change_ntf(pin->dpll_pin);
		}
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
