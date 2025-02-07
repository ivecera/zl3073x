/* SPDX-License-Identifier: GPL-2.0+ */

#ifndef __LINUX_MFD_ZL3073X_H
#define __LINUX_MFD_ZL3073X_H

#include <linux/device.h>
#include <linux/dpll.h>
#include <linux/regmap.h>

/*
 * Hardware limits for ZL3073x chip family
 */
#define ZL3073X_NUM_CHANNELS	2
#define ZL3073X_NUM_IPINS	10
#define ZL3073X_NUM_OPINS	20
#define ZL3073X_NUM_OPAIRS	(ZL3073X_NUM_OPINS / 2)
#define ZL3073X_NUM_PINS	(ZL3073X_NUM_IPINS + ZL3073X_NUM_OPINS)
#define ZL3073X_NUM_SYNTHS	5

/**
 * ZL3073X_IS_INPUT_PIN - check if the pin is input one
 * _pin: pin number to check
 *
 * Returns true if the pin number is input one
 */
#define ZL3073X_IS_INPUT_PIN(_pin)	((_pin) >= ZL3073X_NUM_OPINS)

/**
 * ZL3073X_IS_N_PIN - check if the pin is N-pin
 * _pin: pin number to check
 *
 * Returns true if the pin is N-pin
 */
#define ZL3073X_IS_N_PIN(_pin)	((_pin) & 1)

/**
 * ZL3073X_IS_P_PIN - check if the pin is P-pin
 * _pin: pin number to check
 *
 * Returns true if te pin is P-pin
 */
#define ZL3073X_IS_P_PIN(_pin)	(!ZL3073X_IS_N_PIN(_pin))

enum zl3073x_output_pair_type {
	ZL3073X_SINGLE_ENDED_IN_PHASE,	/* CMOS in phase		*/
	ZL3073X_SINGLE_ENDED_DIVIDED,	/* CMOS N divided		*/
	ZL3073X_DIFFERENTIAL,		/* programmable diff or LVDS	*/
};

enum zl3073x_output_pair_freq_type {
	ZL3073X_SYNCE,
	ZL3073X_SYNCE_1HZ_FIXED,
	ZL3073X_PTP,
	ZL3073X_25MHZ_FIXED,
	ZL3073X_10MHZ_FIXED_EPPS,
	ZL3073X_1HZ_FIXED,
};

/**
 * struct zl3073x_pin_prop - pin properties
 * name: pin name
 * type: pin type
 */
struct zl3073x_pin_prop {
	const char		*name;
	enum dpll_pin_type	type;
};

#define ZL3037X_PIN_PROP(_name, _type) {	\
	.name = _name,				\
	.type = _type,				\
}


/**
 * struct zl3073x_output_pair_prop - output pin pair properties
 */
struct zl3073x_output_pair_prop {
	enum zl3073x_output_pair_type		type;
	enum zl3073x_output_pair_freq_type	freq_type;
};

#define ZL3073X_OUTPUT_PAIR_PROP(_type, _freq_type) {	\
	.type = _type,					\
	.freq_type = _freq_type,			\
}

/**
 * struct zl3073x_platform_data - optional platform data
 * @dpll_types: type of particular DPLL
 * @input_pins: properties of input pins
 * @output_pins: properties of output pins
 * @output_pairs: types of output pin pairs
 */
struct zl3073x_platform_data {
	enum dpll_type			dpll_types[ZL3073X_NUM_CHANNELS];
	struct zl3073x_pin_prop		input_pins[ZL3073X_NUM_IPINS];
	struct zl3073x_pin_prop		output_pins[ZL3073X_NUM_OPINS];
	struct zl3073x_output_pair_prop	output_pairs[ZL3073X_NUM_OPAIRS];
	const char			*fw_init;
};

struct zl3073x_dev {
	struct device				*dev;
	struct regmap				*regmap;
	u64					clock_id;
	const struct zl3073x_platform_data	*pdata;
	struct mutex				lock;

	/* Synths' frequencies */
	u64					synth_freq[ZL3073X_NUM_SYNTHS];
};

/**
 * zl3073x_lock - Lock the device
 * @zldev: device structure pointer
 *
 * Caller has to take this lock when it needs to access device registers.
 */
static inline void zl3073x_lock(struct zl3073x_dev *zldev)
{
	mutex_lock(&zldev->lock);
}

/**
 * zl3073x_unlock - Unlock the device
 * @zldev: device structure pointer
 *
 * Caller unlocks the device when it does not need to access device
 * registers anymore.
 */
static inline void zl3073x_unlock(struct zl3073x_dev *zldev)
{
	mutex_unlock(&zldev->lock);
}

DEFINE_GUARD(zl3073x, struct zl3073x_dev *, zl3073x_lock(_T),
	     zl3073x_unlock(_T));

int zl3073x_read_reg(struct zl3073x_dev *zldev, unsigned int reg,
		     unsigned int len, void *value);

int zl3073x_write_reg(struct zl3073x_dev *zldev, unsigned int reg,
		      unsigned int len, const void *value);

/**
 * __ZL3073X_REG_DEF - Define a device register helpers
 * @_name: register name
 * @_addr: register address
 * @_len: size of register value in bytes
 * @_type: type of register value
 *
 * The macro defines helper functions for particular device register
 * to access it.
 *
 * Example:
 * __ZL3073X_REG_DEF(sample_reg, 0x1234, 4, u32)
 *
 * generates static inline functions:
 * int zl3073x_read_sample_reg(struct zl3073x_dev *dev, u32 *value);
 * int zl3073x_write_sample_reg(struct zl3073x_dev *dev, u32 value);
 *
 * Note that these functions have to be called with the device lock
 * taken.
 */
#define __ZL3073X_REG_DEF(_name, _addr, _len, _type)			\
typedef _type zl3073x_##_name##_t;					\
static inline								\
int zl3073x_read_##_name(struct zl3073x_dev *zldev, _type *value)	\
{									\
	return zl3073x_read_reg(zldev, _addr, _len, value);		\
}									\
static inline								\
int zl3073x_write_##_name(struct zl3073x_dev *zldev, _type value)	\
{									\
	return zl3073x_write_reg(zldev, _addr, _len, &value);		\
}

/**
 * __ZL3073X_REG_IDX_DEF - Define an indexed device register helpers
 * @_name: register name
 * @_addr: register address
 * @_len: size of register value in bytes
 * @_type: type of register value
 * @_num: number of register instances
 * @_stride: address stride between instances
 *
 * The macro defines helper functions for particular indexed device
 * register to access it.
 *
 * Example:
 * __ZL3073X_REG_IDX_DEF(sample_reg, 0x1234, 2, u16, 4, 0x10)
 *
 * generates static inline functions:
 * int zl3073x_read_sample_reg(struct zl3073x_dev *dev, unsigned int idx,
 *			       u32 *value);
 * int zl3073x_write_sample_reg(struct zl3073x_dev *dev, unsigned int idx,
 *				u32 value);
 *
 * Note that these functions have to be called with the device lock
 * taken.
 */
#define __ZL3073X_REG_IDX_DEF(_name, _addr, _len, _type, _num, _stride)	\
typedef _type zl3073x_##_name##_t;					\
static inline								\
int zl3073x_read_##_name(struct zl3073x_dev *zldev, unsigned int idx,	\
			 _type *value)					\
{									\
	WARN_ON(idx >= (_num));						\
	return zl3073x_read_reg(zldev, (_addr) + idx * (_stride), _len,	\
				value);					\
}									\
static inline								\
int zl3073x_write_##_name(struct zl3073x_dev *zldev, unsigned int idx,	\
			  _type value)					\
{									\
	WARN_ON(idx >= (_num));						\
	return zl3073x_write_reg(zldev, (_addr) + idx * (_stride),	\
				 _len, &value);				\
}

/*
 * Add register definition shortcuts for 8, 16, 32 and 48 bits
 */
#define ZL3073X_REG8_DEF(_name, _addr)	__ZL3073X_REG_DEF(_name, _addr, 1, u8)
#define ZL3073X_REG16_DEF(_name, _addr)	__ZL3073X_REG_DEF(_name, _addr, 2, u16)
#define ZL3073X_REG32_DEF(_name, _addr)	__ZL3073X_REG_DEF(_name, _addr, 4, u32)
#define ZL3073X_REG48_DEF(_name, _addr)	__ZL3073X_REG_DEF(_name, _addr, 6, u64)

/*
 * Add indexed register definition shortcuts for 8, 16, 32 and 48 bits
 */
#define ZL3073X_REG8_IDX_DEF(_name, _addr, _num, _stride)		\
	__ZL3073X_REG_IDX_DEF(_name, _addr, 1, u8, _num, _stride)

#define ZL3073X_REG16_IDX_DEF(_name, _addr, _num, _stride)		\
	__ZL3073X_REG_IDX_DEF(_name, _addr, 2, u16, _num, _stride)

#define ZL3073X_REG32_IDX_DEF(_name, _addr, _num, _stride)		\
	__ZL3073X_REG_IDX_DEF(_name, _addr, 4, u32, _num, _stride)

#define ZL3073X_REG48_IDX_DEF(_name, _addr, _num, _stride)		\
	__ZL3073X_REG_IDX_DEF(_name, _addr, 6, u64, _num, _stride)

/**
 * zl3073x_wait_clear_bits - wait for specific bits to be cleared
 * _zldev: pointer to device structure
 * _reg: register name
 * _bits: bits that should be cleared
 * _index: optional index for indexed register
 *
 * The macro waits up to @READ_TIMEOUT_US microseconds for @_bits in @_reg
 * to be cleared.
 *
 * Returns:
 * -ETIMEDOUT: if timeout occured
 *         <0: for other errors occured during communication
 *          0: success
 */
#define READ_SLEEP_US	10
#define READ_TIMEOUT_US	100000000
#define zl3073x_wait_clear_bits(_zldev, _reg, _bits, _index...)		\
	({								\
	 zl3073x_##_reg##_t __val;					\
	 int __rc;							\
	 if (read_poll_timeout_atomic(zl3073x_read_##_reg, __rc,	\
				      __rc || !((_bits) & __val),	\
				      READ_SLEEP_US, READ_TIMEOUT_US,   \
				      false, _zldev, ##_index, &__val))	\
		__rc = -ETIMEDOUT;					\
         __rc;								\
        })

/*
 * Mailbox operations
 */
int zl3073x_mb_dpll_read(struct zl3073x_dev *zldev, u8 index);
int zl3073x_mb_dpll_write(struct zl3073x_dev *zldev, u8 index);
int zl3073x_mb_output_read(struct zl3073x_dev *zldev, u8 index);
int zl3073x_mb_output_write(struct zl3073x_dev *zldev, u8 index);
int zl3073x_mb_ref_read(struct zl3073x_dev *zldev, u8 index);
int zl3073x_mb_ref_write(struct zl3073x_dev *zldev, u8 index);
int zl3073x_mb_synth_read(struct zl3073x_dev *zldev, u8 index);
int zl3073x_mb_synth_write(struct zl3073x_dev *zldev, u8 index);

/**
 * zl3073x_synth_freq_get - get synth current freq
 * @zldev: device structure pointer
 * @synth: synth order number
 *
 * Returns frequency of given synthetizer
 */
static inline
u64 zl3073x_synth_freq_get(struct zl3073x_dev *zldev, u8 synth)
{
	return zldev->synth_freq[synth];
}

#endif /* __LINUX_MFD_ZL3073X_H */
