/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef _ZL3073X_CHAN_H
#define _ZL3073X_CHAN_H

#include <linux/bitfield.h>
#include <linux/stddef.h>
#include <linux/types.h>

#include "regs.h"

struct zl3073x_dev;

/**
 * struct zl3073x_chan - DPLL channel state
 * @psl: phase slope limit register value
 * @ctrl: DPLL control register value
 * @mode_refsel: mode and reference selection register value
 * @ref_prio: reference priority registers (4 bits per ref, P/N packed)
 * @bw_fixed: fixed bandwidth preset register value
 * @bw_var: variable bandwidth register value
 * @mon_status: monitor status register value
 * @refsel_status: reference selection status register value
 * @df_offset: frequency offset vs tracked reference in 2^-48 steps
 */
struct zl3073x_chan {
	struct_group(cfg,
		u16	psl;
		u8	ctrl;
		u8	mode_refsel;
		u8	ref_prio[ZL3073X_NUM_REFS / 2];
		u8	bw_fixed;
		u8	bw_var;
	);
	struct_group(stat,
		u8	mon_status;
		u8	refsel_status;
		s64	df_offset;
	);
};

int zl3073x_chan_state_fetch(struct zl3073x_dev *zldev, u8 index);
const struct zl3073x_chan *zl3073x_chan_state_get(struct zl3073x_dev *zldev,
						 u8 index);
int zl3073x_chan_state_set(struct zl3073x_dev *zldev, u8 index,
			   const struct zl3073x_chan *chan);

int zl3073x_chan_state_update(struct zl3073x_dev *zldev, u8 index);
int zl3073x_chan_nco_mode_set(struct zl3073x_dev *zldev, u8 index);

/**
 * zl3073x_chan_df_offset_get - get cached df_offset vs tracked reference
 * @chan: pointer to channel state
 *
 * Return: frequency offset in 2^-48 steps
 */
static inline s64
zl3073x_chan_df_offset_get(const struct zl3073x_chan *chan)
{
	return chan->df_offset;
}

/**
 * zl3073x_chan_mode_get - get DPLL channel operating mode
 * @chan: pointer to channel state
 *
 * Return: reference selection mode of the given DPLL channel
 */
static inline u8 zl3073x_chan_mode_get(const struct zl3073x_chan *chan)
{
	return FIELD_GET(ZL_DPLL_MODE_REFSEL_MODE, chan->mode_refsel);
}

/**
 * zl3073x_chan_ref_get - get manually selected reference
 * @chan: pointer to channel state
 *
 * Return: reference selected in forced reference lock mode
 */
static inline u8 zl3073x_chan_ref_get(const struct zl3073x_chan *chan)
{
	return FIELD_GET(ZL_DPLL_MODE_REFSEL_REF, chan->mode_refsel);
}

/**
 * zl3073x_chan_mode_set - set DPLL channel operating mode
 * @chan: pointer to channel state
 * @mode: mode to set
 */
static inline void zl3073x_chan_mode_set(struct zl3073x_chan *chan, u8 mode)
{
	FIELD_MODIFY(ZL_DPLL_MODE_REFSEL_MODE, &chan->mode_refsel, mode);
}

/**
 * zl3073x_chan_ref_set - set manually selected reference
 * @chan: pointer to channel state
 * @ref: reference to set
 */
static inline void zl3073x_chan_ref_set(struct zl3073x_chan *chan, u8 ref)
{
	FIELD_MODIFY(ZL_DPLL_MODE_REFSEL_REF, &chan->mode_refsel, ref);
}

/**
 * zl3073x_chan_tie_clear_get - get TIE clear state
 * @chan: pointer to channel state
 *
 * Return: true if TIE is cleared on reference switch, false otherwise
 */
static inline bool
zl3073x_chan_tie_clear_get(const struct zl3073x_chan *chan)
{
	return !!FIELD_GET(ZL_DPLL_CTRL_TIE_CLEAR, chan->ctrl);
}

/**
 * zl3073x_chan_tie_clear_set - set TIE clear state
 * @chan: pointer to channel state
 * @enable: true to enable, false to disable
 */
static inline void
zl3073x_chan_tie_clear_set(struct zl3073x_chan *chan, bool enable)
{
	FIELD_MODIFY(ZL_DPLL_CTRL_TIE_CLEAR, &chan->ctrl, enable ? 1 : 0);
}

/**
 * zl3073x_chan_ref_prio_get - get reference priority
 * @chan: pointer to channel state
 * @ref: input reference index
 *
 * Return: priority of the given reference <0, 15>
 */
static inline u8
zl3073x_chan_ref_prio_get(const struct zl3073x_chan *chan, u8 ref)
{
	u8 val = chan->ref_prio[ref / 2];

	if (!(ref & 1))
		return FIELD_GET(ZL_DPLL_REF_PRIO_REF_P, val);
	else
		return FIELD_GET(ZL_DPLL_REF_PRIO_REF_N, val);
}

/**
 * zl3073x_chan_ref_prio_set - set reference priority
 * @chan: pointer to channel state
 * @ref: input reference index
 * @prio: priority to set
 */
static inline void
zl3073x_chan_ref_prio_set(struct zl3073x_chan *chan, u8 ref, u8 prio)
{
	u8 *val = &chan->ref_prio[ref / 2];

	if (!(ref & 1))
		FIELD_MODIFY(ZL_DPLL_REF_PRIO_REF_P, val, prio);
	else
		FIELD_MODIFY(ZL_DPLL_REF_PRIO_REF_N, val, prio);
}

/**
 * zl3073x_chan_ref_is_selectable - check if reference is selectable
 * @chan: pointer to channel state
 * @ref: input reference index
 *
 * Return: true if the reference priority is not NONE, false otherwise
 */
static inline bool
zl3073x_chan_ref_is_selectable(const struct zl3073x_chan *chan, u8 ref)
{
	return zl3073x_chan_ref_prio_get(chan, ref) != ZL_DPLL_REF_PRIO_NONE;
}

/**
 * zl3073x_chan_lock_state_get - get DPLL channel lock state
 * @chan: pointer to channel state
 *
 * Return: lock state of the given DPLL channel
 */
static inline u8 zl3073x_chan_lock_state_get(const struct zl3073x_chan *chan)
{
	return FIELD_GET(ZL_DPLL_MON_STATUS_STATE, chan->mon_status);
}

/**
 * zl3073x_chan_is_locked - check if channel is locked to a reference
 * @chan: pointer to channel state
 *
 * Return: true if channel is locked, false otherwise
 */
static inline bool zl3073x_chan_is_locked(const struct zl3073x_chan *chan)
{
	u8 lock_state = zl3073x_chan_lock_state_get(chan);
	return lock_state == ZL_DPLL_MON_STATUS_STATE_LOCK;
}

/**
 * zl3073x_chan_mode_is_auto - check if channel is in automatic mode
 * @chan: pointer to channel state
 *
 * Return: true if channel is in automatic mode, false otherwise
 */
static inline bool zl3073x_chan_mode_is_auto(const struct zl3073x_chan *chan)
{
	return zl3073x_chan_mode_get(chan) == ZL_DPLL_MODE_REFSEL_MODE_AUTO;
}

/**
 * zl3073x_chan_mode_is_nco - check if channel is in NCO mode
 * @chan: pointer to channel state
 *
 * Return: true if channel is in NCO mode, false otherwise
 */
static inline bool zl3073x_chan_mode_is_nco(const struct zl3073x_chan *chan)
{
	return zl3073x_chan_mode_get(chan) == ZL_DPLL_MODE_REFSEL_MODE_NCO;
}

/**
 * zl3073x_chan_mode_is_reflock - check if channel is in reflock mode
 * @chan: pointer to channel state
 *
 * Return: true if channel is in reflock mode, false otherwise
 */
static inline bool zl3073x_chan_mode_is_reflock(const struct zl3073x_chan *chan)
{
	return zl3073x_chan_mode_get(chan) == ZL_DPLL_MODE_REFSEL_MODE_REFLOCK;
}

/**
 * zl3073x_chan_is_ho_ready - check if holdover is ready
 * @chan: pointer to channel state
 *
 * Return: true if holdover is ready, false otherwise
 */
static inline bool zl3073x_chan_is_ho_ready(const struct zl3073x_chan *chan)
{
	return !!FIELD_GET(ZL_DPLL_MON_STATUS_HO_READY, chan->mon_status);
}

/**
 * zl3073x_chan_refsel_state_get - get reference selection state
 * @chan: pointer to channel state
 *
 * Return: reference selection state of the given DPLL channel
 */
static inline u8 zl3073x_chan_refsel_state_get(const struct zl3073x_chan *chan)
{
	return FIELD_GET(ZL_DPLL_REFSEL_STATUS_STATE, chan->refsel_status);
}

/**
 * zl3073x_chan_refsel_ref_get - get currently selected reference in auto mode
 * @chan: pointer to channel state
 *
 * Return: reference selected by the DPLL in automatic mode
 */
static inline u8 zl3073x_chan_refsel_ref_get(const struct zl3073x_chan *chan)
{
	return FIELD_GET(ZL_DPLL_REFSEL_STATUS_REFSEL, chan->refsel_status);
}

/**
 * zl3073x_chan_psl_get - get phase slope limit
 * @chan: pointer to channel state
 *
 * Return: phase slope limit in ns/s, 0 means unlimited
 */
static inline u16 zl3073x_chan_psl_get(const struct zl3073x_chan *chan)
{
	return chan->psl;
}

/**
 * zl3073x_chan_psl_set - set phase slope limit
 * @chan: pointer to channel state
 * @psl: phase slope limit in ns/s, 0 means unlimited
 */
static inline void zl3073x_chan_psl_set(struct zl3073x_chan *chan, u16 psl)
{
	chan->psl = psl;
}

u32 zl3073x_chan_bandwidth_get(const struct zl3073x_chan *chan);
int zl3073x_chan_bandwidth_set(struct zl3073x_dev *zldev,
			       struct zl3073x_chan *chan, u32 uhz);

#endif /* _ZL3073X_CHAN_H */
