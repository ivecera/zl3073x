// SPDX-License-Identifier: GPL-2.0

#include <linux/bitfield.h>
#include <linux/firmware.h>
#include <linux/mfd/zl3073x.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>

#include "ptp_private.h"

/*
 * Register Map Page 5, DPLL
 */
ZL3073X_REG8_IDX_DEF(dpll_mode_refsel,		0x0284, 2, 4);
#define DPLL_MODE_REFSEL_MODE			GENMASK(2, 0)
#define DPLL_MODE_REFSEL_MODE_NCO		4
#define DPLL_MODE_REFSEL_REF			GENMASK(7, 4)

ZL3073X_REG8_DEF(dpll_tie_ctrl,			0x2b0);
#define DPLL_TIE_CTRL_OP			GENMASK(2, 0)
#define DPLL_TIE_CTRL_OP_WRITE			4
ZL3073X_REG8_DEF(dpll_tie_ctrl_mask,		0x2b1);

ZL3073X_REG8_IDX_DEF(dpll_tod_ctrl,		0x2b8, 2, 1);
#define DPLL_TOD_CTRL_CMD			GENMASK(3, 0)
#define DPLL_TOD_CTRL_CMD_WRITE_NEXT_1HZ	0x1
#define DPLL_TOD_CTRL_CMD_WRITE_NEXT_SYNC	0x2
#define DPLL_TOD_CTRL_CMD_WRITE_NEXT_HALF_HZ	0x3
#define DPLL_TOD_CTRL_CMD_READ			0x8
#define DPLL_TOD_CTRL_CMD_READ_NEXT_1HZ		0x9
#define DPLL_TOD_CTRL_SEM			BIT(4)

/*
 * Register Map Page 6, DPLL Data
 */
ZL3073X_REG48_IDX_DEF(dpll_df_offset,		0x300, 2, 0x20);
ZL3073X_REG48_IDX_DEF(dpll_tie_data,		0x30c, 2, 0x20);
ZL3073X_REG48_IDX_DEF(dpll_tod_sec,		0x312, 2, 0x20);
ZL3073X_REG32_IDX_DEF(dpll_tod_ns,		0x318, 2, 0x20);

/*
 * Register Map Page 9, Synth and Output
 */
ZL3073X_REG8_DEF(synth_phase_shift_ctrl,	0x49e);
ZL3073X_REG8_DEF(synth_phase_shift_mask,	0x49f);
ZL3073X_REG8_DEF(synth_phase_shift_intvl,	0x4a0);
ZL3073X_REG16_DEF(synth_phase_shift_data,	0x4a1);

ZL3073X_REG8_IDX_DEF(output_ctrl,		0x4a8, 9, 1);
#define OUTPUT_CTRL_SYNTH_SEL			GENMASK(6, 4)
#define OUTPUT_CTRL_EN				BIT(0)
#define OUTPUT_CTRL_STOP			BIT(1)
#define OUTPUT_CTRL_STOP_HIGH			BIT(2)
#define OUTPUT_CTRL_STOP_HZ			BIT(3)

ZL3073X_REG8_DEF(output_phase_step_ctrl,	0x4b8);
#define OUTPUT_PHASE_STEP_CTRL_OP		GENMASK(1, 0)
#define OUTPUT_PHASE_STEP_CTRL_OP_WRITE		3
#define OUTPUT_PHASE_STEP_CTRL_TOD_STEP		BIT(3)
#define OUTPUT_PHASE_STEP_CTRL_DPLL		GENMASK(6, 4)
ZL3073X_REG8_DEF(output_phase_step_number,	0x4b9);
ZL3073X_REG16_DEF(output_phase_step_mask,	0x4ba);
ZL3073X_REG32_DEF(output_phase_step_data,	0x4bc);

/*
 * Register Map Page 14, Output Mailbox
 */
ZL3073X_REG8_DEF(output_mode,			0x705);
#define OUTPUT_MODE_SIGNAL_FORMAT		GENMASK(7, 4)
#define OUTPUT_MODE_SIGNAL_FORMAT_BOTH_DISABLED	0x0
#define OUTPUT_MODE_SIGNAL_FORMAT_BOTH_ENABLED	0x4
#define OUTPUT_MODE_SIGNAL_FORMAT_P_ENABLED	0x5
#define OUTPUT_MODE_SIGNAL_FORMAT_N_ENABLED	0x6
ZL3073X_REG32_DEF(output_div,			0x70c);
ZL3073X_REG32_DEF(output_width,			0x710);
ZL3073X_REG8_DEF(output_gpo_en,			0x724);

struct zl3073x_dpll {
	struct zl3073x_ptp	*zlptp;
	u8			index;

	struct ptp_clock_info	info;
	struct ptp_clock	*clock;
	struct ptp_pin_desc	pins[ZL3073X_NUM_OPINS];

	u16			perout_mask;
};

struct zl3073x_ptp {
	struct device		*dev;
	struct zl3073x_dev	*mfd;

	struct zl3073x_dpll	dpll[ZL3073X_NUM_CHANNELS];
};

static inline struct zl3073x_dpll *ptp_to_dpll(struct ptp_clock_info *ptp)
{
	return container_of(ptp, struct zl3073x_dpll, info);
}

static int zl3073x_gettime64(struct zl3073x_dpll *dpll, struct timespec64 *ts,
			     unsigned int cmd)
{
	struct zl3073x_dev *zldev = dpll->zlptp->mfd;
	u32 nsec;
	u64 sec;
	u8 ctrl;
	int rc;

	/* Check that the semaphore is clear */
	rc = zl3073x_wait_clear_bits(zldev, dpll_tod_ctrl, DPLL_TOD_CTRL_SEM,
				     dpll->index);
	if (rc)
		return rc;

	/* Issue the read command */
	ctrl = DPLL_TOD_CTRL_SEM | FIELD_PREP(DPLL_TOD_CTRL_CMD, cmd);
	zl3073x_write_dpll_tod_ctrl(zldev, dpll->index, ctrl);

	/* Check that the semaphore is clear */
	rc = zl3073x_wait_clear_bits(zldev, dpll_tod_ctrl, DPLL_TOD_CTRL_SEM,
				     dpll->index);
	if (rc)
		return rc;

	/* Read the second and nanoseconds */
	rc = zl3073x_read_dpll_tod_sec(zldev, dpll->index, &sec);
	if (rc)
		return rc;
	rc = zl3073x_read_dpll_tod_ns(zldev, dpll->index, &nsec);
	if (rc)
		return rc;

	set_normalized_timespec64(ts, sec, nsec);

	return 0;
}

static int zl3073x_ptp_gettime64(struct ptp_clock_info *ptp,
				 struct timespec64 *ts)
{
	struct zl3073x_dpll *dpll = ptp_to_dpll(ptp);

	/* Take device lock */
	guard(zl3073x)(dpll->zlptp->mfd);

	return zl3073x_gettime64(dpll, ts, DPLL_TOD_CTRL_CMD_READ);
}

static int zl3073x_settime64(struct zl3073x_dpll *dpll,
			     const struct timespec64 *ts, unsigned int cmd)
{
	struct zl3073x_dev *zldev = dpll->zlptp->mfd;
	u8 tod_ctrl;
	int rc;

	/* Check that the semaphore is clear */
	rc = zl3073x_wait_clear_bits(zldev, dpll_tod_ctrl, DPLL_TOD_CTRL_SEM,
				     dpll->index);
	if (rc)
		return rc;

	/* Write the value */
	rc = zl3073x_write_dpll_tod_sec(zldev, dpll->index, ts->tv_sec);
	if (rc)
		return rc;
	rc = zl3073x_write_dpll_tod_ns(zldev, dpll->index, ts->tv_nsec);
	if (rc)
		return rc;

	/* Issue the write command */
	tod_ctrl = DPLL_TOD_CTRL_SEM | FIELD_PREP(DPLL_TOD_CTRL_CMD, cmd);
	rc = zl3073x_write_dpll_tod_ctrl(zldev, dpll->index, tod_ctrl);

	return rc;
}

static int zl3073x_ptp_settime64(struct ptp_clock_info *ptp,
				 const struct timespec64 *ts)
{
	struct zl3073x_dpll *dpll = ptp_to_dpll(ptp);

	/* Take device lock */
	guard(zl3073x)(dpll->zlptp->mfd);

	return zl3073x_settime64(dpll, ts, DPLL_TOD_CTRL_CMD_WRITE_NEXT_1HZ);
}

static int zl3073x_ptp_getmaxphase(struct ptp_clock_info *ptp)
{
	/* Adjphase accepts phase inputs from -1s to 1s */
	return NSEC_PER_SEC;
}

static int zl3073x_ptp_adjphase(struct ptp_clock_info *ptp, s32 delta)
{
	struct zl3073x_dpll *dpll = ptp_to_dpll(ptp);
	struct zl3073x_dev *zldev = dpll->zlptp->mfd;
	s64 delta_sub_sec_in_tie_units;
	s32 delta_sub_sec_in_ns;
	int rc;

	/* Remove seconds and convert to 0.01ps units */
	delta_sub_sec_in_ns = delta % NSEC_PER_SEC;
	delta_sub_sec_in_tie_units =  delta_sub_sec_in_ns * 100000LL;

	/* Take device lock */
	guard(zl3073x)(zldev);

	/* Set the ctrl to look at the correct dpll */
	zl3073x_write_dpll_tie_ctrl_mask(zldev, BIT(dpll->index));

	/* Wait for access to the CTRL register */
	rc = zl3073x_wait_clear_bits(zldev, dpll_tie_ctrl, DPLL_TIE_CTRL_OP);
	if (rc)
		return rc;

	/* Writes data to the tie register */
	zl3073x_write_dpll_tie_data(zldev, dpll->index,
				    delta_sub_sec_in_tie_units);

	/* Request to write the TIE */
	zl3073x_write_dpll_tie_ctrl(zldev, DPLL_TIE_CTRL_OP_WRITE);

	/* Wait until the TIE operation is completed*/
	return zl3073x_wait_clear_bits(zldev, dpll_tie_ctrl, DPLL_TIE_CTRL_OP);
}

static int zl3073x_steptime(struct zl3073x_dpll *dpll, const s64 delta)
{
	struct zl3073x_dev *zldev = dpll->zlptp->mfd;
	s32 step_data;
	unsigned int phase_step_ctrl;
	u8 output_ctrl, synth;
	s64 freq;
	int rc;

	/* Wait for the previous command to finish */
	rc = zl3073x_wait_clear_bits(zldev, output_phase_step_ctrl,
				     OUTPUT_PHASE_STEP_CTRL_OP);
	if (rc)
		return rc;

	/* Set the number of steps to take, the value is 1 as we want to finish
	 * fast
	 */
	rc = zl3073x_write_output_phase_step_number(zldev, 1);
	if (rc)
		return rc;

	/* Get the synth that is connected to the output, it is OK to get the
	 * synth for only 1 output as it is expected that all the outputs that
	 * are used by 1PPS are connected to same synth.
	 */
	rc = zl3073x_read_output_ctrl(zldev, __ffs(dpll->perout_mask),
				      &output_ctrl);
	if (rc)
		return rc;
	synth = FIELD_GET(OUTPUT_CTRL_SYNTH_SEL, output_ctrl);

	/* Configure the step */
	freq = zl3073x_synth_freq_get(zldev, synth);
	step_data = (s32)div_s64(delta * freq, NSEC_PER_SEC);
	zl3073x_write_output_phase_step_data(zldev, step_data);

	/* Select which output should be adjusted */
	rc = zl3073x_write_output_phase_step_mask(zldev, dpll->perout_mask);
	if (rc)
		return rc;

	/* Start the phase adjustment on the output pin and also on the ToD */
	phase_step_ctrl = OUTPUT_PHASE_STEP_CTRL_TOD_STEP;
	phase_step_ctrl |= FIELD_PREP(OUTPUT_PHASE_STEP_CTRL_DPLL, dpll->index);
	phase_step_ctrl |= FIELD_PREP(OUTPUT_PHASE_STEP_CTRL_OP,
				      OUTPUT_PHASE_STEP_CTRL_OP_WRITE);

	rc = zl3073x_write_output_phase_step_ctrl(zldev, phase_step_ctrl);

	return rc;
}

static int zl3073x_ptp_wait_sec_rollover(struct zl3073x_dpll *dpll)
{
	struct timespec64 ts, init_ts = { };
	unsigned long timeout;
	int rc;

	/* Wait up to 1 second */
	timeout = jiffies + secs_to_jiffies(1);

	do {
		/* Check that the semaphore is clear */
		rc = zl3073x_wait_clear_bits(dpll->zlptp->mfd, dpll_tod_ctrl,
					     DPLL_TOD_CTRL_SEM, dpll->index);
		if (rc)
			return rc;

		/* Read the time */
		rc = zl3073x_gettime64(dpll, &ts,
				       DPLL_TOD_CTRL_CMD_READ_NEXT_1HZ);
		if (rc)
			return rc;

		/* Determine if the second has roll over */
		if (!init_ts.tv_sec)
			init_ts = ts;
		else if (init_ts.tv_sec < ts.tv_sec)
			break;

		msleep(10);
	} while (time_before(jiffies, timeout));

	return 0;
}

static int zl3073x_ptp_adjtime(struct ptp_clock_info *ptp, s64 delta)
{
	struct zl3073x_dpll *dpll = ptp_to_dpll(ptp);
	struct zl3073x_dev *zldev = dpll->zlptp->mfd;
	s64 delta_sub_sec_in_ns;
	struct timespec64 ts;
	s64 delta_sec_in_ns;
	s32 delta_sec_rem;
	s64 delta_sec;
	int rc;

	/* Split the offset to apply into seconds and nanoseconds */
	delta_sec = div_s64_rem(delta, NSEC_PER_SEC, &delta_sec_rem);
	delta_sec_in_ns = delta_sec * NSEC_PER_SEC;
	delta_sub_sec_in_ns = delta_sec_rem;

	/* Take device lock */
	guard(zl3073x)(zldev);

	if (delta >= NSEC_PER_SEC || delta <= -NSEC_PER_SEC) {
		/* wait for rollover */
		rc = zl3073x_ptp_wait_sec_rollover(dpll);
		if (rc)
			return rc;

		/* get the predicted TOD at the next internal 1PPS */
		rc = zl3073x_gettime64(dpll, &ts,
				       DPLL_TOD_CTRL_CMD_READ_NEXT_1HZ);
		if (rc)
			return rc;

		ts = timespec64_add(ts, ns_to_timespec64(delta_sec_in_ns));

		rc = zl3073x_settime64(dpll, &ts,
				       DPLL_TOD_CTRL_CMD_WRITE_NEXT_1HZ);
		if (rc)
			return rc;

		/* Wait for the semaphore bit to confirm correct settime
		 * application */
		rc = zl3073x_wait_clear_bits(zldev, dpll_tod_ctrl,
					     DPLL_TOD_CTRL_SEM, dpll->index);
		if (rc)
			return rc;
	}

	return zl3073x_steptime(dpll, delta_sub_sec_in_ns);
}

static int zl3073x_ptp_adjfine(struct ptp_clock_info *ptp, long scaled_ppm)
{
	struct zl3073x_dpll *dpll = ptp_to_dpll(ptp);
	struct zl3073x_dev *zldev = dpll->zlptp->mfd;
	s64 ref;

	/* Store the scaled_ppm into a s64 variable because on 32bit arch, the
	 * multiplication with ZL30373X_1PPM_FORMAT with overflow meaning that
	 * will not be able to adjust to lowest ns
	 */
	if (!scaled_ppm)
		return 0;

	/* Take device lock */
	guard(zl3073x)(zldev);

	/* The delta frequency offset is computed using this formula:
	 * df_offset = (-ppm / 10^6) * 2^48
	 */
	ref = mul_s64_u64_shr(-scaled_ppm, BIT_ULL(48) / 1000000, 16);

	return zl3073x_write_dpll_df_offset(zldev, dpll->index, ref);
}

static unsigned int zl3073x_ptp_disable_pin(unsigned int current_mode, u8 pin)
{
	switch (current_mode) {
	case OUTPUT_MODE_SIGNAL_FORMAT_P_ENABLED:
		if (ZL3073X_IS_P_PIN(pin))
			return OUTPUT_MODE_SIGNAL_FORMAT_BOTH_DISABLED;
		break;
	case OUTPUT_MODE_SIGNAL_FORMAT_N_ENABLED:
		if (ZL3073X_IS_N_PIN(pin))
			return OUTPUT_MODE_SIGNAL_FORMAT_BOTH_DISABLED;
		break;
	case OUTPUT_MODE_SIGNAL_FORMAT_BOTH_ENABLED:
		if (ZL3073X_IS_P_PIN(pin))
			return OUTPUT_MODE_SIGNAL_FORMAT_N_ENABLED;
		else
			return OUTPUT_MODE_SIGNAL_FORMAT_P_ENABLED;
	default:
		return OUTPUT_MODE_SIGNAL_FORMAT_BOTH_DISABLED;
	}

	return OUTPUT_MODE_SIGNAL_FORMAT_BOTH_DISABLED;
}

static int zl3073x_ptp_perout_disable(struct zl3073x_dpll *dpll,
				      struct ptp_perout_request *perout)
{
	struct zl3073x_dev *zldev = dpll->zlptp->mfd;
	u8 mode, output_mode;
	int rc;
	int pin;

	pin = ptp_find_pin(dpll->clock, PTP_PF_PEROUT, perout->index);
	if (pin == -1 || pin >= ZL3073X_NUM_OPINS)
		return -EINVAL;

	/* Read output configuration into mailbox */
	rc = zl3073x_mb_output_read(zldev, pin / 2);
	if (rc)
		return rc;

	/* Read current configuration */
	rc = zl3073x_read_output_mode(zldev, &output_mode);

	mode = FIELD_GET(OUTPUT_MODE_SIGNAL_FORMAT, output_mode);
	mode = zl3073x_ptp_disable_pin(mode, pin);
	output_mode &= ~OUTPUT_MODE_SIGNAL_FORMAT;
	output_mode |= FIELD_PREP(OUTPUT_MODE_SIGNAL_FORMAT, mode);

	/* Update the configuration */
	zl3073x_write_output_mode(zldev, output_mode);

	/* Update output configuration from mailbox */
	rc = zl3073x_mb_output_write(zldev, pin / 2);
	if (rc)
		return rc;

	dpll->perout_mask &= ~BIT(pin / 2);

	return rc;
}

static unsigned int zl3073x_ptp_enable_pin(unsigned int current_mode, u8 pin)
{
	switch (current_mode) {
	case OUTPUT_MODE_SIGNAL_FORMAT_P_ENABLED:
		if (ZL3073X_IS_N_PIN(pin))
			return OUTPUT_MODE_SIGNAL_FORMAT_BOTH_ENABLED;
		break;
	case OUTPUT_MODE_SIGNAL_FORMAT_N_ENABLED:
		if (ZL3073X_IS_P_PIN(pin))
			return OUTPUT_MODE_SIGNAL_FORMAT_BOTH_ENABLED;
		break;
	case OUTPUT_MODE_SIGNAL_FORMAT_BOTH_DISABLED:
		if (ZL3073X_IS_P_PIN(pin))
			return OUTPUT_MODE_SIGNAL_FORMAT_P_ENABLED;
		else
			return OUTPUT_MODE_SIGNAL_FORMAT_N_ENABLED;
	default:
		return OUTPUT_MODE_SIGNAL_FORMAT_BOTH_ENABLED;
	}

	return OUTPUT_MODE_SIGNAL_FORMAT_BOTH_ENABLED;
}

static int zl3073x_ptp_perout_enable(struct zl3073x_dpll *dpll,
				     struct ptp_perout_request *perout)
{
	struct zl3073x_dev *zldev = dpll->zlptp->mfd;
	u8 mode, output_mode, synth;
	int pin, rc;
	u32 width;
	u64 freq;

	/* Only accept a 1-PPS aligned to the second. */
	if (perout->start.nsec || perout->period.sec != 1 ||
	    perout->period.nsec)
		return -ERANGE;

	pin = ptp_find_pin(dpll->clock, PTP_PF_PEROUT, perout->index);
	if (pin == -1 || pin >= ZL3073X_NUM_OPINS)
		return -EINVAL;

	/* Read output configuration into mailbox */
	rc = zl3073x_mb_output_read(zldev, pin / 2);
	if (rc)
		return rc;

	/* Read configuration of output mode */
	rc = zl3073x_read_output_mode(zldev, &output_mode);
	if (rc)
		return rc;

	mode = FIELD_GET(OUTPUT_MODE_SIGNAL_FORMAT, output_mode);
	mode = zl3073x_ptp_enable_pin(mode, pin);
	output_mode &= OUTPUT_MODE_SIGNAL_FORMAT;
	output_mode |= FIELD_PREP(OUTPUT_MODE_SIGNAL_FORMAT, mode);

	/* Update the configuration */
	rc = zl3073x_write_output_mode(zldev, output_mode);
	if (rc)
		return rc;

	/* Make sure that the output is set as clock and not GPIO */
	rc = zl3073x_write_output_gpo_en(zldev, 0x0);
	if (rc)
		return rc;

	/* Get the synth that is connected to the output and set the same value
	 * in the ouput divider of the pin so it can get an 1PPS as this is the
	 * only value supported
	 */
	synth = zl3073x_output_synth_get(zldev, pin / 2);
	freq = zl3073x_synth_freq_get(zldev, synth);

	rc = zl3073x_write_output_div(zldev, freq);
	if (rc)
		return rc;

	if (perout->flags & PTP_PEROUT_DUTY_CYCLE) {
		if (perout->on.sec)
			return -EINVAL;

		/* The value that needs to be written in the register is
		 * calculated as following:
		 * width = perout->on.nsec / (NSEC_PER_SEC / freq) * 2
		 *       = (freq * perout->on.nsec) / (C / 2)
		 */
		width = mul_u64_u32_div(freq, perout->on.nsec, NSEC_PER_SEC/2);

		rc = zl3073x_write_output_width(zldev, width);
		if (rc)
			return rc;
	}

	/* Update output configuration from mailbox */
	rc = zl3073x_mb_output_write(zldev, pin / 2);
	if (rc)
		return rc;

	dpll->perout_mask |= BIT(pin / 2);

	return 0;
}

static int zl3073x_ptp_enable(struct ptp_clock_info *ptp,
			      struct ptp_clock_request *rq, int on)
{
	struct zl3073x_dpll *dpll = ptp_to_dpll(ptp);
	struct zl3073x_ptp *zlptp = dpll->zlptp;
	int rc;

	if (rq->type != PTP_CLK_REQ_PEROUT)
		return -EINVAL;

	guard(zl3073x)(zlptp->mfd);

	if (on)
		rc = zl3073x_ptp_perout_enable(dpll, &rq->perout);
	else
		rc = zl3073x_ptp_perout_disable(dpll, &rq->perout);

	return rc;
}

static int zl3073x_ptp_verify(struct ptp_clock_info *ptp, unsigned int pin,
			      enum ptp_pin_function func, unsigned int chan)
{
	switch (func) {
	case PTP_PF_NONE:
	case PTP_PF_PEROUT:
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static struct ptp_clock_info zl3073x_ptp_clock_info = {
	.owner		= THIS_MODULE,
	.name		= "zl3073x ptp",
	.max_adj	= 1000000000,
	.gettime64	= zl3073x_ptp_gettime64,
	.settime64	= zl3073x_ptp_settime64,
	.adjtime	= zl3073x_ptp_adjtime,
	.adjfine	= zl3073x_ptp_adjfine,
	.adjphase	= zl3073x_ptp_adjphase,
	.getmaxphase	= zl3073x_ptp_getmaxphase,
	.enable		= zl3073x_ptp_enable,
	.verify		= zl3073x_ptp_verify,
	.n_per_out	= ZL3073X_NUM_OPINS,
	.n_ext_ts	= ZL3073X_NUM_OPINS,
	.n_pins		= ZL3073X_NUM_OPINS,
};

static int zl3073x_ptp_init(struct zl3073x_ptp *zlptp, u8 index)
{
	struct zl3073x_dpll *dpll = &zlptp->dpll[index];

	for (int i = 0; i < ZL3073X_NUM_OPINS; i++) {
		struct ptp_pin_desc *p = &dpll->pins[i];

		snprintf(p->name, sizeof(p->name), "pin%d", i);
		p->index = i;
		p->func = PTP_PF_NONE;
		p->chan = 0;
	}

	dpll->index = index;
	dpll->zlptp = zlptp;
	dpll->info = zl3073x_ptp_clock_info;
	dpll->info.pin_config = dpll->pins;
	dpll->clock = ptp_clock_register(&dpll->info, zlptp->dev);
	if (IS_ERR(dpll->clock))
		return PTR_ERR(dpll->clock);

	return 0;
}

static int zl3073x_ptp_firmware_parse_line(struct zl3073x_ptp *zlptp,
					   const char *line)
{
#define ZL3073X_FW_WHITESPACES_SIZE	3
#define ZL3073X_FW_COMMAND_SIZE		1

	const char *ptr = line;
	char *endp;
	u32 delay;
	u16 addr;
	u8 val;

	switch (ptr[0]) {
	case 'X':
		/* The line looks like this:
		 * X , ADDR , VAL
		 * Where:
		 *  - X means that is a command that needs to be executed
		 *  - ADDR represents the addr and is always 2 bytes and the
		 *         value is in hex, for example 0x0232
		 *  - VAL represents the value that is written and is always 1
		 *        byte and the value is in hex, for example 0x12
		 */
		ptr += ZL3073X_FW_COMMAND_SIZE;
		ptr += ZL3073X_FW_WHITESPACES_SIZE;
		addr = simple_strtoul(ptr, &endp, 16);

		ptr = endp;
		ptr += ZL3073X_FW_WHITESPACES_SIZE;
		val = simple_strtoul(ptr, NULL, 16);

		return zl3073x_write_reg(zlptp->mfd, addr, 1, &val);
	case 'W':
		/* The line looks like this:
		 * W , DELAY
		 * Where:
		 *  - W means that is a wait command
		 *  - DELAY represents the delay in microseconds and the value
		 *    is in decimal
		 */
		ptr += ZL3073X_FW_COMMAND_SIZE;
		ptr += ZL3073X_FW_WHITESPACES_SIZE;
		delay = simple_strtoul(ptr, NULL, 10);

		fsleep(delay);
		break;
	default:
		break;
	}

	return 0;
}

static int zl3073x_ptp_firmware_load(struct zl3073x_ptp *zlptp)
{
#define ZL3073X_FW_FILENAME		"zl3073x.mfg"
	const struct firmware *fw;
	const char *ptr, *end;
	char buf[128];
	int rc;

	rc = request_firmware(&fw, ZL3073X_FW_FILENAME, zlptp->dev);
	if (rc)
		return rc;

	ptr = fw->data;
	end = ptr + fw->size;
	while (ptr < end) {
		/* Get next end of the line or end of buffer */
		char *eol = strnchrnul(ptr, end - ptr, '\n');
		size_t len = eol - ptr;

		/* Check line length */
		if (len >= sizeof(buf)) {
			dev_err(zlptp->dev, "Line in firmware is too long\n");
			return -E2BIG;
		}

		/* Copy line from buffer */
		memcpy(buf, ptr, len);
		buf[len] = '\0';

		/* Parse and process the line */
		rc = zl3073x_ptp_firmware_parse_line(zlptp, buf);
		if (rc) {
			dev_err_probe(zlptp->dev, rc,
				      "Failed to parse firmware line\n");
			break;
		}

		/* Move to next line */
		ptr = eol + 1;
	}

	release_firmware(fw);

	return rc;
}

static int zl3073x_ptp_init_fine_phase_adjust(struct zl3073x_ptp *zlptp)
{
	struct zl3073x_dev *zldev = zlptp->mfd;
	int rc;

	rc = zl3073x_write_synth_phase_shift_mask(zldev, 0x1f);
	if (rc)
		return rc;

	rc = zl3073x_write_synth_phase_shift_intvl(zldev, 0x01);
	if (rc)
		return rc;

	rc = zl3073x_write_synth_phase_shift_data(zldev, 0xffff);
	if (rc)
		return rc;

	return zl3073x_write_synth_phase_shift_ctrl(zldev, 0x01);
}

static bool zl3073x_ptp_dpll_nco_mode(struct zl3073x_ptp *zlptp, int dpll_index)
{
	u8 mode, mode_refsel;
	int rc;

	rc = zl3073x_read_dpll_mode_refsel(zlptp->mfd, dpll_index,
					   &mode_refsel);
	if (rc)
		return false;

	mode = FIELD_GET(DPLL_MODE_REFSEL_MODE, mode_refsel);

	return (mode == DPLL_MODE_REFSEL_MODE_NCO);
}

static int zl3073x_ptp_probe(struct platform_device *pdev)
{
	struct zl3073x_ptp *zlptp;
	size_t i;
	int rc;

	zlptp = devm_kzalloc(&pdev->dev, sizeof(*zlptp), GFP_KERNEL);
	if (!zlptp)
		return -ENOMEM;

	zlptp->dev = &pdev->dev;
	zlptp->mfd = dev_get_drvdata(pdev->dev.parent);

	guard(zl3073x)(zlptp->mfd);

	zl3073x_ptp_firmware_load(zlptp);

	for (i = 0; i < ZL3073X_NUM_CHANNELS; i++) {
		if (!zl3073x_ptp_dpll_nco_mode(zlptp, i))
			continue;

		rc = zl3073x_ptp_init(zlptp, i);
		if (rc)
			return rc;
	}

	platform_set_drvdata(pdev, zlptp);

	/* Initial firmware fine phase correction */
	zl3073x_ptp_init_fine_phase_adjust(zlptp);

	return 0;
}

static void zl3073x_ptp_remove(struct platform_device *pdev)
{
	struct zl3073x_ptp *zlptp = platform_get_drvdata(pdev);
	int i;

	for (i = 0; i < ZL3073X_NUM_CHANNELS; i++) {
		scoped_guard(zl3073x, zlptp->mfd)
			if (!zl3073x_ptp_dpll_nco_mode(zlptp, i))
				continue;

		if (!IS_ERR_OR_NULL(zlptp->dpll[i].clock))
			ptp_clock_unregister(zlptp->dpll[i].clock);
	}
}

static const struct platform_device_id zl3073x_ptp_platform_id[] = {
        { "zl3073x-phc", },
        { /* sentinel */ },
};
MODULE_DEVICE_TABLE(platform, zl3073x_ptp_platform_id);

static struct platform_driver zl3073x_ptp_driver = {
	.driver = {
		.name = "zl3073x-phc",
	},
	.probe = zl3073x_ptp_probe,
	.remove	= zl3073x_ptp_remove,
	.id_table = zl3073x_ptp_platform_id,
};

module_platform_driver(zl3073x_ptp_driver);

MODULE_DESCRIPTION("Driver for zl3073x clock devices");
MODULE_AUTHOR("Ivan Vecera <ivecera@redhat.com>");
MODULE_AUTHOR("Tariq Haddad <Tariq.Haddad@microchip.com>");
MODULE_LICENSE("GPL");
