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
 * Register Map Page 12, DPLL Mailbox
 */
ZL3073X_REG8_IDX_DEF(dpll_ref_prio,		0x652, ZL3073X_NUM_IPINS/2, 1);
#define DPLL_REF_PRIO_REF_P			GENMASK(3, 0)
#define DPLL_REF_PRIO_REF_N			GENMASK(7, 4)
#define DPLL_REF_PRIO_INVALID			0xf

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
		u8 refsel_status, ref_selected;
		u32 ref_prio;

		rc = zl3073x_read_dpll_state_refsel_status(zldev,
							   zldpll->id,
							   &refsel_status);
		if (rc)
			return rc;

		ref_selected = FIELD_GET(DPLL_STATE_REFSEL_STATUS_REFSEL,
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

static u8
zl3073x_dpll_pin_synth_get(struct zl3073x_dpll_pin *pin)
{
	u8 pair = zl3073x_dpll_pin_pair_get(pin->index);

	return zl3073x_output_synth_get(pin_to_dev(pin), pair);
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

static void
zl3073x_dpll_fill_pin_properties(struct zl3073x_dpll_pin *pin)
{
	struct dpll_pin_properties *props = &pin->props;
	struct zl3073x_dev *zldev = pin_to_dev(pin);
	const struct zl3073x_pin_prop *pin_prop;
	struct dpll_pin_frequency *freq;
	u8 idx = pin->index;
	u32 num_freq;

	memset(props, 0, sizeof(*props));

	if (ZL3073X_IS_INPUT_PIN(idx)) {
		pin_prop = &zldev->pdata->input_pins[idx - ZL3073X_NUM_OPINS];
		num_freq = ARRAY_SIZE(input_freq_ranges);
		freq = input_freq_ranges;
	} else {
		u8 pair = zl3073x_dpll_pin_pair_get(pin->index);

		pin_prop = &zldev->pdata->output_pins[idx];

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

		/* Input pins starts are stored after output pins */
		pin = &zldpll->pins[ZL3073X_NUM_OPINS + i];

		rc = zl3073x_dpll_input_pin_state_on_dpll_get(pin->dpll_pin,
							      pin,
							      zldpll->dpll_dev,
							      zldpll, &state,
							      NULL);
		if (rc)
			goto out;

		if (state != pin->pin_state) {
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
