// SPDX-License-Identifier: GPL-2.0-only

#include <linux/cleanup.h>
#include <linux/delay.h>
#include <linux/dev_printk.h>
#include <linux/int_log.h>
#include <linux/math.h>
#include <linux/ptp_clock_kernel.h>
#include <linux/string.h>
#include <linux/types.h>

#include "chan.h"
#include "core.h"

/**
 * zl3073x_chan_state_update - update DPLL channel status from HW
 * @zldev: pointer to zl3073x_dev structure
 * @index: DPLL channel index
 *
 * Return: 0 on success, <0 on error
 */
int zl3073x_chan_state_update(struct zl3073x_dev *zldev, u8 index)
{
	struct zl3073x_chan *chan = &zldev->chan[index];
	u64 val;
	int rc;

	rc = zl3073x_read_u8(zldev, ZL_REG_DPLL_MON_STATUS(index),
			     &chan->mon_status);
	if (rc)
		return rc;

	rc = zl3073x_read_u8(zldev, ZL_REG_DPLL_REFSEL_STATUS(index),
			     &chan->refsel_status);
	if (rc)
		return rc;

	/* Read df_offset only when locked to a reference. In NCO mode
	 * df_offset was captured at entry by nco_mode_set() - preserve it.
	 */
	if (!zl3073x_chan_is_locked(chan)) {
		if (!zl3073x_chan_mode_is_nco(chan))
			chan->df_offset = ZL_DPLL_DF_OFFSET_UNKNOWN;
		return 0;
	}

	rc = zl3073x_poll_zero_u8(zldev, ZL_REG_DPLL_DF_READ(index),
				  ZL_DPLL_DF_READ_SEM,
				  ZL_POLL_DF_READ_TIMEOUT_US);
	if (rc)
		return rc;

	rc = zl3073x_write_u8(zldev, ZL_REG_DPLL_DF_READ(index),
			      ZL_DPLL_DF_READ_SEM | ZL_DPLL_DF_READ_REF_OFST);
	if (rc)
		return rc;

	rc = zl3073x_poll_zero_u8(zldev, ZL_REG_DPLL_DF_READ(index),
				  ZL_DPLL_DF_READ_SEM,
				  ZL_POLL_DF_READ_TIMEOUT_US);
	if (rc)
		return rc;

	rc = zl3073x_read_u48(zldev, ZL_REG_DPLL_DF_OFFSET(index), &val);
	if (rc)
		return rc;

	chan->df_offset = sign_extend64(val, 47);

	return 0;
}

/**
 * zl3073x_chan_nco_mode_set - switch DPLL channel to NCO mode
 * @zldev: pointer to zl3073x_dev structure
 * @index: DPLL channel index
 *
 * Switches the channel to NCO mode and reads the df_offset
 * auto-captured by nco_auto_read directly from the register.
 * No DF_READ handshake is needed as nco_auto_read populates
 * the register before the mode switch completes.
 *
 * Return: 0 on success, <0 on error
 */
int zl3073x_chan_nco_mode_set(struct zl3073x_dev *zldev, u8 index)
{
	struct zl3073x_chan *chan = &zldev->chan[index];
	u8 prev_mode, df_read;
	u64 val;
	int rc;

	prev_mode = zl3073x_chan_mode_get(chan);

	/* nco_auto_read captures the tracking offset at NCO entry only
	 * from reflock, auto or holdover mode. From freerun the captured
	 * value is not meaningful.
	 */
	if (prev_mode == ZL_DPLL_MODE_REFSEL_MODE_FREERUN) {
		zl3073x_chan_mode_set(chan, ZL_DPLL_MODE_REFSEL_MODE_NCO);

		rc = zl3073x_write_u8(zldev, ZL_REG_DPLL_MODE_REFSEL(index),
				      chan->mode_refsel);
		if (rc) {
			zl3073x_chan_mode_set(chan, prev_mode);
			return rc;
		}

		chan->df_offset = ZL_DPLL_DF_OFFSET_UNKNOWN;
		return 0;
	}

	/* Configure df_read for nco_auto_read:
	 * ref_ofst=0 - reads offset relative to master clock (not input ref)
	 * cmd=CMD_ACC_I - accumulated I-part covering both locked and
	 *                 holdover entry.
	 *
	 * No semaphore is set - this only configures what the df_offset
	 * value represents after the mode switch; nco_auto_read performs
	 * the actual read automatically.
	 */
	df_read = FIELD_PREP(ZL_DPLL_DF_READ_REF_OFST, 0) |
		  FIELD_PREP(ZL_DPLL_DF_READ_CMD, ZL_DPLL_DF_READ_CMD_ACC_I);
	rc = zl3073x_write_u8(zldev, ZL_REG_DPLL_DF_READ(index), df_read);
	if (rc)
		return rc;

	/* Wait for df_read configuration to take effect before
	 * triggering nco_auto_read via mode switch. The worst-case
	 * internal register update time is 25 ms.
	 */
	fsleep(25000);

	zl3073x_chan_mode_set(chan, ZL_DPLL_MODE_REFSEL_MODE_NCO);
	rc = zl3073x_write_u8(zldev, ZL_REG_DPLL_MODE_REFSEL(index),
			      chan->mode_refsel);
	if (rc) {
		zl3073x_chan_mode_set(chan, prev_mode);
		return rc;
	}

	/* Wait for nco_auto_read to populate df_offset. The worst-case
	 * internal register update time is 25 ms.
	 */
	fsleep(25000);

	/* Read df_offset captured by nco_auto_read during mode switch.
	 * No DF_READ semaphore handshake needed. Mode switch already
	 * succeeded, so don't propagate a read failure back to userspace.
	 */
	rc = zl3073x_read_u48(zldev, ZL_REG_DPLL_DF_OFFSET(index), &val);
	if (rc) {
		dev_warn(zldev->dev,
			 "Failed to read DPLL%u df_offset: %pe\n",
			 index, ERR_PTR(rc));
		chan->df_offset = ZL_DPLL_DF_OFFSET_UNKNOWN;
	} else {
		chan->df_offset = sign_extend64(val, 47);
	}

	return 0;
}

/**
 * zl3073x_chan_state_fetch - fetch DPLL channel state from hardware
 * @zldev: pointer to zl3073x_dev structure
 * @index: DPLL channel index to fetch state for
 *
 * Reads the mode_refsel, status and reference priority registers for
 * the given DPLL channel and stores the values for later use.
 *
 * Return: 0 on success, <0 on error
 */
int zl3073x_chan_state_fetch(struct zl3073x_dev *zldev, u8 index)
{
	struct zl3073x_chan *chan = &zldev->chan[index];
	int rc, i;

	rc = zl3073x_read_u8(zldev, ZL_REG_DPLL_CTRL(index), &chan->ctrl);
	if (rc)
		return rc;

	rc = zl3073x_read_u8(zldev, ZL_REG_DPLL_MODE_REFSEL(index),
			     &chan->mode_refsel);
	if (rc)
		return rc;

	dev_dbg(zldev->dev, "DPLL%u mode: %u, ref: %u\n", index,
		zl3073x_chan_mode_get(chan), zl3073x_chan_ref_get(chan));

	rc = zl3073x_chan_state_update(zldev, index);
	if (rc)
		return rc;

	/* If firmware left the channel in NCO mode, mark df_offset as
	 * unknown - we cannot know whether the preconditions for a valid
	 * nco_auto_read capture were met.
	 */
	if (zl3073x_chan_mode_is_nco(chan))
		chan->df_offset = ZL_DPLL_DF_OFFSET_UNKNOWN;

	dev_dbg(zldev->dev,
		"DPLL%u lock_state: %u, ho: %u, sel_state: %u, sel_ref: %u\n",
		index, zl3073x_chan_lock_state_get(chan),
		zl3073x_chan_is_ho_ready(chan) ? 1 : 0,
		zl3073x_chan_refsel_state_get(chan),
		zl3073x_chan_refsel_ref_get(chan));

	guard(mutex)(&zldev->multiop_lock);

	/* Read DPLL configuration from mailbox */
	rc = zl3073x_mb_op(zldev, ZL_REG_DPLL_MB_SEM, ZL_DPLL_MB_SEM_RD,
			   ZL_REG_DPLL_MB_MASK, BIT(index));
	if (rc)
		return rc;

	/* Read reference priority registers */
	for (i = 0; i < ARRAY_SIZE(chan->ref_prio); i++) {
		rc = zl3073x_read_u8(zldev, ZL_REG_DPLL_REF_PRIO(i),
				     &chan->ref_prio[i]);
		if (rc)
			return rc;
	}

	rc = zl3073x_read_u8(zldev, ZL_REG_DPLL_BW_FIXED,
			     &chan->bw_fixed);
	if (rc)
		return rc;

	rc = zl3073x_read_u8(zldev, ZL_REG_DPLL_BW_VAR, &chan->bw_var);
	if (rc)
		return rc;

	rc = zl3073x_read_u16(zldev, ZL_REG_DPLL_PSL, &chan->psl);
	if (rc)
		return rc;

	return 0;
}

/**
 * zl3073x_chan_state_get - get current DPLL channel state
 * @zldev: pointer to zl3073x_dev structure
 * @index: DPLL channel index to get state for
 *
 * Return: pointer to given DPLL channel state
 */
const struct zl3073x_chan *zl3073x_chan_state_get(struct zl3073x_dev *zldev,
						  u8 index)
{
	return &zldev->chan[index];
}

/**
 * zl3073x_chan_tod_ready_wait - wait for ToD semaphore to clear
 * @zldev: pointer to zl3073x device
 * @ch: DPLL channel index
 *
 * Polls the ToD control register until the semaphore bit is cleared,
 * indicating the device has completed the previous ToD operation.
 *
 * Return: 0 on success, -EBUSY if semaphore not cleared, <0 on error
 */
int zl3073x_chan_tod_ready_wait(struct zl3073x_dev *zldev, u8 ch)
{
	unsigned int timeout;
	u8 tod_ctrl;
	int rc;

	rc = zl3073x_read_u8(zldev, ZL_REG_DPLL_TOD_CTRL(ch), &tod_ctrl);
	if (rc)
		return rc;

	switch (FIELD_GET(ZL_DPLL_TOD_CTRL_CMD, tod_ctrl)) {
	case 0:
		/* No operation pending (power-on/reset default) */
		return 0;
	case ZL_DPLL_TOD_CTRL_CMD_WR_NEXT_1HZ:
		timeout = ZL_POLL_TOD_WR_TIMEOUT_US;
		break;
	case ZL_DPLL_TOD_CTRL_CMD_RD_CURRENT:
	case ZL_DPLL_TOD_CTRL_CMD_RD_NEXT_1HZ:
		timeout = ZL_POLL_TOD_RD_TIMEOUT_US;
		break;
	default:
		/* FW never writes cmd bits; only the driver sets them */
		WARN_ON(1);
		return -EOPNOTSUPP;
	}

	rc = zl3073x_poll_zero_u8(zldev, ZL_REG_DPLL_TOD_CTRL(ch),
				  ZL_DPLL_TOD_CTRL_SEM, timeout);

	return rc == -ETIMEDOUT ? -EBUSY : rc;
}

/**
 * zl3073x_chan_tod_ctrl - issue ToD command
 * @zldev: pointer to zl3073x device
 * @ch: DPLL channel index
 * @cmd: ToD command to execute
 *
 * Writes the semaphore and command to dpll_tod_ctrl. The caller must
 * ensure the device is ready (semaphore clear) before calling and
 * must wait for completion if needed.
 *
 * Return: 0 on success, <0 on error
 */
static int zl3073x_chan_tod_ctrl(struct zl3073x_dev *zldev, u8 ch, u8 cmd)
{
	return zl3073x_write_u8(zldev, ZL_REG_DPLL_TOD_CTRL(ch),
				ZL_DPLL_TOD_CTRL_SEM | cmd);
}

/**
 * zl3073x_chan_tod_read - read ToD registers after issuing a command
 * @zldev: pointer to zl3073x device
 * @ch: DPLL channel index
 * @next_hz: if true, read predicted ToD at next 1 Hz; otherwise read current
 * @ts: timespec to store the result
 * @sts: optional system timestamp pair for cross-timestamping
 *
 * Context: Caller must serialize all zl3073x_chan_tod_* calls externally.
 * Return: 0 on success, <0 on error
 */
int zl3073x_chan_tod_read(struct zl3073x_dev *zldev, u8 ch,
			  bool next_hz, struct timespec64 *ts,
			  struct ptp_system_timestamp *sts)
{
	u32 nsec;
	u64 sec;
	u8 cmd;
	int rc;

	if (next_hz)
		cmd = ZL_DPLL_TOD_CTRL_CMD_RD_NEXT_1HZ;
	else
		cmd = ZL_DPLL_TOD_CTRL_CMD_RD_CURRENT;

	/* Wait for any previous ToD operation to complete */
	rc = zl3073x_chan_tod_ready_wait(zldev, ch);
	if (rc)
		return rc;

	ptp_read_system_prets(sts);
	rc = zl3073x_chan_tod_ctrl(zldev, ch, cmd);
	if (rc)
		return rc;

	rc = zl3073x_chan_tod_ready_wait(zldev, ch);
	if (rc)
		return rc;
	ptp_read_system_postts(sts);

	rc = zl3073x_read_u48(zldev, ZL_REG_DPLL_TOD_SEC(ch), &sec);
	if (rc)
		return rc;

	/* HW nanoseconds are always in [0, NSEC_PER_SEC) range */
	rc = zl3073x_read_u32(zldev, ZL_REG_DPLL_TOD_NS(ch), &nsec);
	if (rc)
		return rc;

	ts->tv_sec = sec;
	ts->tv_nsec = nsec;

	return 0;
}

/**
 * zl3073x_chan_tod_write - write ToD registers and trigger 1 Hz update
 * @zldev: pointer to zl3073x device
 * @ch: DPLL channel index
 * @ts: time to set
 *
 * Context: Caller must serialize all zl3073x_chan_tod_* calls externally.
 * Return: 0 on success, <0 on error
 */
int zl3073x_chan_tod_write(struct zl3073x_dev *zldev, u8 ch,
			   struct timespec64 ts)
{
	int rc;

	/* Wait for any previous ToD operation to complete */
	rc = zl3073x_chan_tod_ready_wait(zldev, ch);
	if (rc)
		return rc;

	rc = zl3073x_write_u48(zldev, ZL_REG_DPLL_TOD_SEC(ch), ts.tv_sec);
	if (rc)
		return rc;

	rc = zl3073x_write_u32(zldev, ZL_REG_DPLL_TOD_NS(ch), ts.tv_nsec);
	if (rc)
		return rc;

	return zl3073x_chan_tod_ctrl(zldev, ch,
				     ZL_DPLL_TOD_CTRL_CMD_WR_NEXT_1HZ);
}

/**
 * zl3073x_chan_tod_adjust - atomic ToD read-modify-write with rollover guard
 * @zldev: pointer to zl3073x device
 * @ch: DPLL channel index
 * @delta: time adjustment to apply
 *
 * Reads the next-Hz ToD and current ToD, then checks whether enough time
 * remains before the next 1 Hz rollover to safely complete the write.
 * Re-reads if the 1 Hz tick crossed between the two reads or if less
 * than 20 ms remains before the next rollover. Applies @delta and writes
 * the result back.
 *
 * Context: Caller must serialize all zl3073x_chan_tod_* calls externally.
 * Return: 0 on success, <0 on error
 */
#define ZL_TOD_MAX_RETRIES	20
int zl3073x_chan_tod_adjust(struct zl3073x_dev *zldev, u8 ch,
			    struct timespec64 delta)
{
	static const long threshold_ns = 20 * NSEC_PER_MSEC;
	struct timespec64 ts_next, ts_cur, diff;
	int rc, i;

	for (i = 0; i < ZL_TOD_MAX_RETRIES; i++) {
		rc = zl3073x_chan_tod_read(zldev, ch, true, &ts_next, NULL);
		if (rc)
			return rc;

		rc = zl3073x_chan_tod_read(zldev, ch, false, &ts_cur, NULL);
		if (rc)
			return rc;

		/* Ensure the 1 Hz tick did not cross between the two reads
		 * and that enough margin remains to complete the write.
		 */
		diff = timespec64_sub(ts_next, ts_cur);
		if (diff.tv_sec > 0 ||
		    (!diff.tv_sec && diff.tv_nsec >= threshold_ns))
			break;
	}
	if (i == ZL_TOD_MAX_RETRIES) {
		dev_warn(zldev->dev,
			 "DPLL%u ToD adjust failed to get stable margin\n",
			 ch);
		return -EBUSY;
	}

	/* Apply delta to the next-Hz ToD */
	ts_next = timespec64_add(ts_next, delta);
	if (!timespec64_valid(&ts_next))
		return -EINVAL;

	return zl3073x_chan_tod_write(zldev, ch, ts_next);
}

/**
 * zl3073x_chan_df_offset_set - write delta frequency offset to hardware
 * @zldev: pointer to zl3073x device
 * @ch: DPLL channel index
 * @offset: frequency offset in 2^-48 steps
 *
 * Context: Caller must hold the per-DPLL lock.
 * Return: 0 on success, <0 on error
 */
int zl3073x_chan_df_offset_set(struct zl3073x_dev *zldev, u8 ch, s64 offset)
{
	int rc;

	rc = zl3073x_write_u48(zldev, ZL_REG_DPLL_DF_OFFSET(ch), offset);
	if (!rc)
		zldev->chan[ch].df_offset = offset;

	return rc;
}

/**
 * zl3073x_chan_tie_write - adjust DPLL phase using TIE write
 * @zldev: pointer to zl3073x device
 * @ch: DPLL channel index
 * @delta_ns: phase adjustment in nanoseconds (must be within +-1s)
 *
 * Converts nanoseconds to TIE units (0.01 ps) and writes TIE data
 * to the specified channel.
 *
 * Return: 0 on success, <0 on error
 */
int zl3073x_chan_tie_write(struct zl3073x_dev *zldev, u8 ch, s64 delta_ns)
{
	s64 tie_data;
	int rc;

	if (WARN_ON(delta_ns <= -NSEC_PER_SEC || delta_ns >= NSEC_PER_SEC))
		return -ERANGE;

	guard(mutex)(&zldev->tie_lock);

	/* Wait for any previous TIE operation to complete */
	rc = zl3073x_poll_zero_u8(zldev, ZL_REG_DPLL_TIE_CTRL,
				  ZL_DPLL_TIE_CTRL_OP,
				  ZL_POLL_TIE_WR_TIMEOUT_US);
	if (rc)
		return rc;

	/* Convert ns to TIE units (0.01 ps = 10^-14 s) */
	tie_data = delta_ns * 100000LL;

	rc = zl3073x_write_u48(zldev, ZL_REG_DPLL_TIE_DATA(ch), tie_data);
	if (rc)
		return rc;

	rc = zl3073x_write_u8(zldev, ZL_REG_DPLL_TIE_CTRL_MASK, BIT(ch));
	if (rc)
		return rc;

	return zl3073x_write_u8(zldev, ZL_REG_DPLL_TIE_CTRL,
				ZL_DPLL_TIE_CTRL_OP_WR);
}

/**
 * zl3073x_chan_phase_step - execute one output phase step operation
 * @zldev: pointer to zl3073x device
 * @ch: DPLL channel index
 * @out_mask: bitmask of outputs to step
 * @step_cycles: phase step in synthesizer clock cycles
 * @tod_step: also step the ToD counter
 *
 * All masked outputs must use synthesizers of the same frequency since
 * the step value is in synthesizer clock cycles.
 *
 * Return: 0 on success, <0 on error
 */
int zl3073x_chan_phase_step(struct zl3073x_dev *zldev, u8 ch,
			    u16 out_mask, s32 step_cycles,
			    bool tod_step)
{
	u8 ctrl;
	int rc;

	guard(mutex)(&zldev->phase_step_lock);

	/* Wait for any previous phase step operation to complete */
	rc = zl3073x_poll_zero_u8(zldev, ZL_REG_OUTPUT_PHASE_STEP_CTRL,
				  ZL_OUTPUT_PHASE_STEP_CTRL_OP,
				  ZL_POLL_PHASE_STEP_TIMEOUT_US);
	if (rc)
		return rc;

	rc = zl3073x_write_u32(zldev, ZL_REG_OUTPUT_PHASE_STEP_DATA,
			       step_cycles);
	if (rc)
		return rc;

	rc = zl3073x_write_u16(zldev, ZL_REG_OUTPUT_PHASE_STEP_MASK, out_mask);
	if (rc)
		return rc;

	rc = zl3073x_write_u8(zldev, ZL_REG_OUTPUT_PHASE_STEP_NUMBER, 1);
	if (rc)
		return rc;

	ctrl = FIELD_PREP(ZL_OUTPUT_PHASE_STEP_CTRL_DPLL, ch) |
	       FIELD_PREP(ZL_OUTPUT_PHASE_STEP_CTRL_OP,
			  ZL_OUTPUT_PHASE_STEP_CTRL_OP_WRITE);
	if (tod_step)
		ctrl |= ZL_OUTPUT_PHASE_STEP_CTRL_TOD_STEP;

	return zl3073x_write_u8(zldev, ZL_REG_OUTPUT_PHASE_STEP_CTRL, ctrl);
}

/**
 * zl3073x_chan_state_set - commit DPLL channel state changes to hardware
 * @zldev: pointer to zl3073x_dev structure
 * @index: DPLL channel index to set state for
 * @chan: desired channel state
 *
 * Skips the HW write if the configuration is unchanged, and otherwise
 * writes only the changed registers to hardware. The mode_refsel register
 * is written directly, while the reference priority registers are written
 * via the DPLL mailbox interface.
 *
 * Return: 0 on success, <0 on HW error
 */
int zl3073x_chan_state_set(struct zl3073x_dev *zldev, u8 index,
			   const struct zl3073x_chan *chan)
{
	struct zl3073x_chan *dchan = &zldev->chan[index];
	int rc, i;

	/* Skip HW write if configuration hasn't changed */
	if (!memcmp(&dchan->cfg, &chan->cfg, sizeof(chan->cfg)))
		return 0;

	/* Direct register writes for ctrl and mode_refsel */
	if (dchan->ctrl != chan->ctrl) {
		rc = zl3073x_write_u8(zldev, ZL_REG_DPLL_CTRL(index),
				      chan->ctrl);
		if (rc)
			return rc;
		dchan->ctrl = chan->ctrl;
	}

	if (dchan->mode_refsel != chan->mode_refsel) {
		rc = zl3073x_write_u8(zldev, ZL_REG_DPLL_MODE_REFSEL(index),
				      chan->mode_refsel);
		if (rc)
			return rc;
		dchan->mode_refsel = chan->mode_refsel;
	}

	/* Check if any mailbox field changed */
	if (!memcmp(&dchan->cfg, &chan->cfg, sizeof(chan->cfg))) {
		dchan->cfg = chan->cfg;
		return 0;
	}

	guard(mutex)(&zldev->multiop_lock);

	/* Read DPLL configuration into mailbox */
	rc = zl3073x_mb_op(zldev, ZL_REG_DPLL_MB_SEM, ZL_DPLL_MB_SEM_RD,
			   ZL_REG_DPLL_MB_MASK, BIT(index));
	if (rc)
		return rc;

	/* Update changed ref_prio registers */
	for (i = 0; i < ARRAY_SIZE(chan->ref_prio); i++) {
		if (dchan->ref_prio[i] != chan->ref_prio[i]) {
			rc = zl3073x_write_u8(zldev,
					      ZL_REG_DPLL_REF_PRIO(i),
					      chan->ref_prio[i]);
			if (rc)
				return rc;
		}
	}

	/* Update changed bandwidth registers */
	if (dchan->bw_fixed != chan->bw_fixed) {
		rc = zl3073x_write_u8(zldev, ZL_REG_DPLL_BW_FIXED,
				      chan->bw_fixed);
		if (rc)
			return rc;
	}
	if (dchan->bw_var != chan->bw_var) {
		rc = zl3073x_write_u8(zldev, ZL_REG_DPLL_BW_VAR,
				      chan->bw_var);
		if (rc)
			return rc;
	}

	/* Update changed phase slope limit register */
	if (dchan->psl != chan->psl) {
		rc = zl3073x_write_u16(zldev, ZL_REG_DPLL_PSL, chan->psl);
		if (rc)
			return rc;
	}

	/* Commit DPLL configuration */
	rc = zl3073x_mb_op(zldev, ZL_REG_DPLL_MB_SEM, ZL_DPLL_MB_SEM_WR,
			   ZL_REG_DPLL_MB_MASK, BIT(index));
	if (rc)
		return rc;

	/* After successful write store new state */
	dchan->cfg = chan->cfg;

	return 0;
}

/*
 * Fractional power table: 10^(r/32) scaled by 10^6, for r = 0..31.
 * Used to compute bw_uhz = int_pow(10, q) * pow10_frac[r] / 10000
 * where bw_var = q * 32 + r.
 */
static const u32 zl3073x_pow10_frac[] = {
	 1000000, 1074608, 1154782, 1240938,
	 1333521, 1433013, 1539927, 1654817,
	 1778279, 1910953, 2053525, 2206734,
	 2371374, 2548297, 2738420, 2942727,
	 3162278, 3398208, 3651741, 3924190,
	 4216965, 4531584, 4869675, 5232991,
	 5623413, 6042964, 6493816, 6978306,
	 7498942, 8058422, 8659643, 9305720,
};

#define ZL3073X_BW_VAR_MIN_FREQ	100
#define ZL3073X_BW_VAR_MAX_FREQ	13335210
#define ZL3073X_BW_VAR_MAX_VAL	164
#define ZL3073X_BW_VAR_470HZ		0xC8
#define ZL3073X_BW_470HZ_FREQ		470000000

/**
 * zl3073x_chan_bw_var_to_uhz - convert bw_var register value to µHz
 * @bw_var: register value
 *
 * Formula: bw_hz = 10^(bw_var / 32) / 10^4
 * Decomposed as: bw_var = q * 32 + r, then
 * bw_uhz = int_pow(10, q) * pow10_frac[r] / 10000
 *
 * Return: bandwidth in microhertz
 */
static u32 zl3073x_chan_bw_var_to_uhz(u8 bw_var)
{
	u8 q, r;

	/* Original series uses 0xC8 as a special code for 470 Hz */
	if (bw_var == ZL3073X_BW_VAR_470HZ)
		return ZL3073X_BW_470HZ_FREQ;

	bw_var = min_t(u8, bw_var, ZL3073X_BW_VAR_MAX_VAL);
	q = bw_var / 32;
	r = bw_var % 32;

	return mul_u64_u32_div(int_pow(10, q), zl3073x_pow10_frac[r],
			       10000);
}

/**
 * zl3073x_chan_uhz_to_bw_var - convert µHz to bw_var register value
 * @uhz: bandwidth in microhertz
 *
 * Formula: bw_var = round(32 * log10(uhz / 100))
 * Using intlog10: bw_var = (intlog10(uhz) + 2^18) >> 19 - 64
 *
 * Return: bw_var register value, clamped to ZL3073X_BW_VAR_MAX_VAL
 */
static u8 zl3073x_chan_uhz_to_bw_var(u32 uhz)
{
	int bw_var;

	bw_var = ((intlog10(uhz) + (1 << 18)) >> 19) - 64;

	return (u8)clamp(bw_var, 0, ZL3073X_BW_VAR_MAX_VAL);
}

/* Fixed bandwidth presets in microhertz (µHz) */
static const u32 zl3073x_bw_fixed_uhz[] = {
	[ZL_DPLL_BW_FIXED_14HZ]  =  14000000,
	[ZL_DPLL_BW_FIXED_29HZ]  =  29000000,
	[ZL_DPLL_BW_FIXED_61HZ]  =  61000000,
	[ZL_DPLL_BW_FIXED_141HZ] = 141000000,
	[ZL_DPLL_BW_FIXED_403HZ] = 403000000,
	[ZL_DPLL_BW_FIXED_470HZ] = 470000000,
};

/**
 * zl3073x_chan_bandwidth_get - convert HW bandwidth to microhertz
 * @chan: pointer to channel state
 *
 * Return: current bandwidth in microhertz
 */
u32 zl3073x_chan_bandwidth_get(const struct zl3073x_chan *chan)
{
	u8 bw = FIELD_GET(ZL_DPLL_BW_FIXED_BW, chan->bw_fixed);

	if (bw == ZL_DPLL_BW_FIXED_VARIABLE)
		return zl3073x_chan_bw_var_to_uhz(chan->bw_var);

	if (bw < ARRAY_SIZE(zl3073x_bw_fixed_uhz))
		return zl3073x_bw_fixed_uhz[bw];

	return 0;
}

/**
 * zl3073x_chan_bandwidth_set - convert microhertz to HW bandwidth
 * @zldev: pointer to zl3073x_dev structure
 * @chan: pointer to channel state to modify
 * @uhz: desired bandwidth in microhertz
 *
 * Return: 0 on success, -EINVAL if value is out of range
 */
int zl3073x_chan_bandwidth_set(struct zl3073x_dev *zldev,
			       struct zl3073x_chan *chan, u32 uhz)
{
	int i;

	/* Original series lacks bw_fixed=5 (470 Hz preset) and uses special
	 * value of 0xC8 for bw_var instead.
	 */
	if (!(zldev->info->flags & ZL3073X_FLAG_BW_FIXED_470HZ) &&
	    uhz == ZL3073X_BW_470HZ_FREQ) {
		FIELD_MODIFY(ZL_DPLL_BW_FIXED_BW, &chan->bw_fixed,
			     ZL_DPLL_BW_FIXED_VARIABLE);
		chan->bw_var = ZL3073X_BW_VAR_470HZ;

		return 0;
	}

	/* Check fixed presets */
	for (i = 0; i < ARRAY_SIZE(zl3073x_bw_fixed_uhz); i++) {
		if (zl3073x_bw_fixed_uhz[i] == uhz) {
			FIELD_MODIFY(ZL_DPLL_BW_FIXED_BW, &chan->bw_fixed, i);
			return 0;
		}
	}

	/* Variable mode */
	if (uhz < ZL3073X_BW_VAR_MIN_FREQ || uhz > ZL3073X_BW_VAR_MAX_FREQ)
		return -EINVAL;

	FIELD_MODIFY(ZL_DPLL_BW_FIXED_BW, &chan->bw_fixed,
		     ZL_DPLL_BW_FIXED_VARIABLE);
	chan->bw_var = zl3073x_chan_uhz_to_bw_var(uhz);

	return 0;
}
