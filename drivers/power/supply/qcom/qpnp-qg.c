// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2018-2021 The Linux Foundation. All rights reserved.
 */

#define pr_fmt(fmt)	"QG-K: %s: " fmt, __func__

#include <linux/debugfs.h>
#include <linux/alarmtimer.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/iio/iio.h>
#include <linux/kernel.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/timekeeping.h>
#include <linux/uaccess.h>
#include <linux/pmic-voter.h>
#include <linux/poll.h>
#include <linux/iio/consumer.h>
#include <dt-bindings/iio/qti_power_supply_iio.h>
#include <uapi/linux/qg.h>
#include <uapi/linux/qg-profile.h>
#include "fg-alg.h"
#include "qg-sdam.h"
#include "qg-core.h"
#include "qg-iio.h"
#include "qg-reg.h"
#include "qg-util.h"
#include "qg-soc.h"
#include "qg-battery-profile.h"
#include "qg-defs.h"
#include "battery-profile-loader.h"

static const struct qg_config config[] = {
	[PM2250]	= {QG_LITE, PM2250},
	[PM6150]	= {QG_PMIC5, PM6150},
	[PMI632]	= {QG_PMIC5, PMI632},
	[PM7250B]	= {QG_PMIC5, PM7250B},
};

static const char *qg_get_battery_type(struct qpnp_qg *chip);
static int qg_process_rt_fifo(struct qpnp_qg *chip);
static int qg_load_battery_profile(struct qpnp_qg *chip);

#if !defined(CONFIG_SOMC_CHARGER_EXTENSION)
static int qg_debug_mask;
#endif
#if defined(CONFIG_SOMC_CHARGER_EXTENSION)
static int qg_debug_mask = QG_DEBUG_SOMC | QG_DEBUG_PROFILE;
#endif

static int qg_esr_mod_count = 30;
static ssize_t esr_mod_count_show(struct device *dev, struct device_attribute
				     *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", qg_esr_mod_count);
}

static ssize_t esr_mod_count_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int val;

	if (kstrtos32(buf, 0, &val))
		return -EINVAL;

	qg_esr_mod_count = val;

	return count;
}
static DEVICE_ATTR_RW(esr_mod_count);

static int qg_esr_count = 3;
static ssize_t esr_count_show(struct device *dev, struct device_attribute
				 *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", qg_esr_count);
}

static ssize_t esr_count_store(struct device *dev, struct device_attribute
				  *attr, const char *buf, size_t count)
{
	int val;

	if (kstrtos32(buf, 0, &val))
		return -EINVAL;

	qg_esr_count = val;

	return count;
}
static DEVICE_ATTR_RW(esr_count);

static ssize_t battery_type_show(struct device *dev,
				struct device_attribute
				*attr, char *buf)
{
	struct qpnp_qg *chip = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%s\n",
			qg_get_battery_type(chip));
}
static DEVICE_ATTR_RO(battery_type);

static struct attribute *qg_attrs[] = {
	&dev_attr_esr_mod_count.attr,
	&dev_attr_esr_count.attr,
	&dev_attr_soc_interval_ms.attr,
	&dev_attr_soc_cold_interval_ms.attr,
	&dev_attr_maint_soc_update_ms.attr,
	&dev_attr_fvss_delta_soc_interval_ms.attr,
	&dev_attr_fvss_vbat_scaling.attr,
	&dev_attr_qg_ss_feature.attr,
	&dev_attr_battery_type.attr,
	NULL,
};
ATTRIBUTE_GROUPS(qg);

static bool is_battery_present(struct qpnp_qg *chip)
{
	bool present = true;
	u8 reg = 0;
	int rc;

	if (chip->qg_version == QG_LITE) {
		rc = qg_read(chip, chip->qg_base + QG_STATUS2_REG, &reg, 1);
		if (rc < 0)
			pr_err("Failed to read battery presence, rc=%d\n", rc);
		else
			present = !(reg & BATTERY_MISSING_BIT);
	} else {
		rc = qg_read(chip, chip->qg_base + QG_STATUS1_REG, &reg, 1);
		if (rc < 0)
			pr_err("Failed to read battery presence, rc=%d\n", rc);
		else
			present = !!(reg & BATTERY_PRESENT_BIT);
	}

	return present;
}

#define DEBUG_BATT_ID_LOW	6000
#define DEBUG_BATT_ID_HIGH	8500
static bool is_debug_batt_id(struct qpnp_qg *chip)
{
	if (is_between(DEBUG_BATT_ID_LOW, DEBUG_BATT_ID_HIGH,
					chip->batt_id_ohm))
		return true;

	return false;
}

static int qg_read_ocv(struct qpnp_qg *chip, u32 *ocv_uv, u32 *ocv_raw, u8 type)
{
	int rc, addr;
	u64 temp = 0;
	char ocv_name[20];

	switch (type) {
	case S3_GOOD_OCV:
		addr = QG_S3_GOOD_OCV_V_DATA0_REG;
		strlcpy(ocv_name, "S3_GOOD_OCV", 20);
		break;
	case S7_PON_OCV:
		addr = QG_S7_PON_OCV_V_DATA0_REG;
		strlcpy(ocv_name, "S7_PON_OCV", 20);
		break;
	case S3_LAST_OCV:
		addr = QG_LAST_S3_SLEEP_V_DATA0_REG;
		strlcpy(ocv_name, "S3_LAST_OCV", 20);
		break;
	case SDAM_PON_OCV:
		addr = QG_SDAM_PON_OCV_OFFSET;
		strlcpy(ocv_name, "SDAM_PON_OCV", 20);
		break;
	default:
		pr_err("Invalid OCV type %d\n", type);
		return -EINVAL;
	}

	if (type == SDAM_PON_OCV) {
		rc = qg_sdam_read(SDAM_PON_OCV_UV, ocv_raw);
		if (rc < 0) {
			pr_err("Failed to read SDAM PON OCV rc=%d\n", rc);
			return rc;
		}
	} else {
		rc = qg_read(chip, chip->qg_base + addr, (u8 *)ocv_raw, 2);
		if (rc < 0) {
			pr_err("Failed to read ocv, rc=%d\n", rc);
			return rc;
		}
	}

	temp = *ocv_raw;
	*ocv_uv = V_RAW_TO_UV(temp);

	pr_debug("%s: OCV_RAW=%x OCV=%duV\n", ocv_name, *ocv_raw, *ocv_uv);

	return rc;
}

#define DEFAULT_S3_FIFO_LENGTH		3
static int qg_update_fifo_length(struct qpnp_qg *chip, u8 length)
{
	int rc;
	u8 s3_entry_fifo_length = 0;

	if (!length || length > chip->max_fifo_length) {
		pr_err("Invalid FIFO length %d\n", length);
		return -EINVAL;
	}

	rc = qg_masked_write(chip, chip->qg_base + QG_S2_NORMAL_MEAS_CTL2_REG,
			FIFO_LENGTH_MASK, (length - 1) << FIFO_LENGTH_SHIFT);
	if (rc < 0)
		pr_err("Failed to write S2 FIFO length, rc=%d\n", rc);

	/* update the S3 FIFO length, when S2 length is updated */
	if (length > 3 && !chip->dt.qg_sleep_config)
		s3_entry_fifo_length = (chip->dt.s3_entry_fifo_length > 0) ?
			chip->dt.s3_entry_fifo_length : DEFAULT_S3_FIFO_LENGTH;
	else	/* Use S3 length as 1 for any S2 length <= 3 */
		s3_entry_fifo_length = 1;

	rc = qg_masked_write(chip,
			chip->qg_base + QG_S3_SLEEP_OCV_IBAT_CTL1_REG,
			SLEEP_IBAT_QUALIFIED_LENGTH_MASK,
			s3_entry_fifo_length - 1);
	if (rc < 0)
		pr_err("Failed to write S3-entry fifo-length, rc=%d\n",
						rc);

	return rc;
}

static int qg_master_hold(struct qpnp_qg *chip, bool hold)
{
	int rc;

	/* clear the master */
	rc = qg_masked_write(chip, chip->qg_base + QG_DATA_CTL1_REG,
					MASTER_HOLD_OR_CLR_BIT, 0);
	if (rc < 0)
		return rc;

	if (hold) {
		/* 0 -> 1, hold the master */
		rc = qg_masked_write(chip, chip->qg_base + QG_DATA_CTL1_REG,
					MASTER_HOLD_OR_CLR_BIT,
					MASTER_HOLD_OR_CLR_BIT);
		if (rc < 0)
			return rc;
	}

	qg_dbg(chip, QG_DEBUG_STATUS, "Master hold = %d\n", hold);

	return rc;
}

static void qg_notify_charger(struct qpnp_qg *chip)
{
	union power_supply_propval prop = {0, };
	int rc;

	if (!chip->batt_psy)
		return;

	if (!chip->profile_loaded)
		return;

	prop.intval = chip->bp.float_volt_uv;
	rc = power_supply_set_property(chip->batt_psy,
			POWER_SUPPLY_PROP_VOLTAGE_MAX, &prop);
	if (rc < 0) {
		pr_err("Failed to set voltage_max property on batt_psy, rc=%d\n",
			rc);
		return;
	}

	prop.intval = chip->bp.fastchg_curr_ma * 1000;
	rc = power_supply_set_property(chip->batt_psy,
			POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX, &prop);
	if (rc < 0) {
		pr_err("Failed to set constant_charge_current_max property on batt_psy, rc=%d\n",
			rc);
		return;
	}

	pr_debug("Notified charger on float voltage and FCC\n");

	rc = power_supply_get_property(chip->batt_psy,
			POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT, &prop);
	if (rc < 0) {
		pr_err("Failed to get charge term current, rc=%d\n", rc);
		return;
	}
	chip->chg_iterm_ma = prop.intval;
}

static bool is_batt_available(struct qpnp_qg *chip)
{
	if (chip->batt_psy)
		return true;

	chip->batt_psy = power_supply_get_by_name("battery");
	if (!chip->batt_psy)
		return false;

	/* batt_psy is initialized, set the fcc and fv */
	qg_notify_charger(chip);

	return true;
}

static int qg_store_soc_params(struct qpnp_qg *chip)
{
	int rc, batt_temp = 0, i;
	unsigned long rtc_sec = 0;
	u32 flash_ocv = 0;

	rc = get_rtc_time(&rtc_sec);
	if (rc < 0)
		pr_err("Failed to get RTC time, rc=%d\n", rc);
	else
		chip->sdam_data[SDAM_TIME_SEC] = rtc_sec;

	rc = qg_get_battery_temp(chip, &batt_temp);
	if (rc < 0)
		pr_err("Failed to get battery-temp, rc = %d\n", rc);
	else
		chip->sdam_data[SDAM_TEMP] = (u32)batt_temp;

	for (i = 0; i <= SDAM_TIME_SEC; i++) {
		rc |= qg_sdam_write(i, chip->sdam_data[i]);
		qg_dbg(chip, QG_DEBUG_STATUS, "SDAM write param %d value=%d\n",
					i, chip->sdam_data[i]);
	}

	/* store the SDAM OCV */
	flash_ocv = chip->sdam_data[SDAM_OCV_UV] / 20000;
	rc = qg_sdam_write(SDAM_FLASH_OCV, flash_ocv);
	if (rc < 0)
		pr_err("Failed to update flash-ocv rc=%d\n", rc);

	return rc;
}

#define MAX_FIFO_CNT_FOR_ESR			50
static int qg_config_s2_state(struct qpnp_qg *chip,
		enum s2_state requested_state, bool state_enable,
		bool process_fifo)
{
	int rc, acc_interval, acc_length;
	u8 fifo_length, reg = 0, state = S2_DEFAULT;

	if ((chip->s2_state_mask & requested_state) && state_enable)
		return 0; /* No change in state */

	if (!(chip->s2_state_mask & requested_state) && !state_enable)
		return 0; /* No change in state */

	if (state_enable)
		chip->s2_state_mask |= requested_state;
	else
		chip->s2_state_mask &= ~requested_state;

	/* define the priority of the states */
	if (chip->s2_state_mask & S2_FAST_CHARGING)
		state = S2_FAST_CHARGING;
	else if (chip->s2_state_mask & S2_LOW_VBAT)
		state = S2_LOW_VBAT;
	else if (chip->s2_state_mask & S2_SLEEP)
		state = S2_SLEEP;
	else
		state = S2_DEFAULT;

	if (state == chip->s2_state)
		return 0;

	switch (state) {
	case S2_FAST_CHARGING:
		fifo_length = chip->dt.fast_chg_s2_fifo_length;
		acc_interval = chip->dt.s2_acc_intvl_ms;
		acc_length = chip->dt.s2_acc_length;
		break;
	case S2_LOW_VBAT:
		fifo_length = chip->dt.s2_vbat_low_fifo_length;
		acc_interval = chip->dt.s2_acc_intvl_ms;
		acc_length = chip->dt.s2_acc_length;
		break;
	case S2_SLEEP:
		fifo_length = chip->dt.sleep_s2_fifo_length;
		acc_interval = chip->dt.sleep_s2_acc_intvl_ms;
		acc_length = chip->dt.sleep_s2_acc_length;
		break;
	case S2_DEFAULT:
		fifo_length = chip->dt.s2_fifo_length;
		acc_interval = chip->dt.s2_acc_intvl_ms;
		acc_length = chip->dt.s2_acc_length;
		break;
	default:
		pr_err("Invalid S2 state %d\n", state);
		return -EINVAL;
	}

	if (fifo_length)
		qg_esr_mod_count = MAX_FIFO_CNT_FOR_ESR / fifo_length;

	rc = qg_master_hold(chip, true);
	if (rc < 0) {
		pr_err("Failed to hold master, rc=%d\n", rc);
		return rc;
	}

	if (process_fifo) {
		rc = qg_process_rt_fifo(chip);
		if (rc < 0) {
			pr_err("Failed to process FIFO real-time, rc=%d\n", rc);
			goto done;
		}
	}

	rc = qg_update_fifo_length(chip, fifo_length);
	if (rc < 0) {
		pr_err("Failed to update S2 fifo-length, rc=%d\n", rc);
		goto done;
	}

	reg = acc_interval / 10;
	rc = qg_write(chip, chip->qg_base + QG_S2_NORMAL_MEAS_CTL3_REG,
					&reg, 1);
	if (rc < 0) {
		pr_err("Failed to update S2 acc intrvl, rc=%d\n", rc);
		goto done;
	}

	reg = ilog2(acc_length) - 1;
	rc = qg_masked_write(chip, chip->qg_base + QG_S2_NORMAL_MEAS_CTL2_REG,
					NUM_OF_ACCUM_MASK, reg);
	if (rc < 0) {
		pr_err("Failed to update S2 ACC length, rc=%d\n", rc);
		goto done;
	}

	chip->s2_state = state;

	qg_dbg(chip, QG_DEBUG_STATUS, "S2 New state=%x  fifo_length=%d interval=%d acc_length=%d\n",
				state, fifo_length, acc_interval, acc_length);

done:
	qg_master_hold(chip, false);
	/* FIFO restarted */
	chip->last_fifo_update_time = ktime_get_boottime();
	return rc;
}

static int qg_process_fifo(struct qpnp_qg *chip, u32 fifo_length)
{
	int rc = 0, i, j = 0, temp;
	u8 v_fifo[MAX_FIFO_LENGTH * 2], i_fifo[MAX_FIFO_LENGTH * 2];
	u32 sample_interval = 0, sample_count = 0, fifo_v = 0, fifo_i = 0;
	unsigned long rtc_sec = 0;
	bool qg_v_mode = (chip->qg_mode == QG_V_MODE);

	rc = get_rtc_time(&rtc_sec);
	if (rc < 0)
		pr_err("Failed to get RTC time, rc=%d\n", rc);

	chip->kdata.fifo_time = (u32)rtc_sec;

	if (!fifo_length) {
		pr_debug("No FIFO data\n");
		return 0;
	}

	qg_dbg(chip, QG_DEBUG_FIFO, "FIFO length=%d\n", fifo_length);

	rc = get_sample_interval(chip, &sample_interval);
	if (rc < 0) {
		pr_err("Failed to get FIFO sample interval, rc=%d\n", rc);
		return rc;
	}

	rc = get_sample_count(chip, &sample_count);
	if (rc < 0) {
		pr_err("Failed to get FIFO sample count, rc=%d\n", rc);
		return rc;
	}

	/*
	 * If there is pending data from suspend, append the new FIFO
	 * data to it. Only do this if we can accomadate 8 FIFOs
	 */
	if (chip->suspend_data &&
		(chip->kdata.fifo_length < (MAX_FIFO_LENGTH / 2))) {
		j = chip->kdata.fifo_length; /* append the data */
		chip->suspend_data = false;
		qg_dbg(chip, QG_DEBUG_FIFO,
			"Pending suspend-data FIFO length=%d\n", j);
	} else {
		/* clear any old pending data */
		chip->kdata.fifo_length = 0;
	}

	for (i = 0; i < fifo_length * 2; i = i + 2, j++) {
		rc = qg_read(chip, chip->qg_base + QG_V_FIFO0_DATA0_REG + i,
					&v_fifo[i], 2);
		if (rc < 0) {
			pr_err("Failed to read QG_V_FIFO, rc=%d\n", rc);
			return rc;
		}
		rc = qg_read(chip, chip->qg_base + QG_I_FIFO0_DATA0_REG + i,
					&i_fifo[i], 2);
		if (rc < 0) {
			pr_err("Failed to read QG_I_FIFO, rc=%d\n", rc);
			return rc;
		}

		fifo_v = v_fifo[i] | (v_fifo[i + 1] << 8);
		fifo_i = i_fifo[i] | (i_fifo[i + 1] << 8);

		if (fifo_v == FIFO_V_RESET_VAL ||
			(fifo_i == FIFO_I_RESET_VAL && !qg_v_mode)) {
			pr_err("Invalid FIFO data V_RAW=%x I_RAW=%x - FIFO rejected\n",
						fifo_v, fifo_i);
			return -EINVAL;
		}

		temp = sign_extend32(fifo_i, 15);

		chip->kdata.fifo[j].v = V_RAW_TO_UV(fifo_v);
		chip->kdata.fifo[j].i =
				qg_v_mode ? 0 : qg_iraw_to_ua(chip, temp);
		chip->kdata.fifo[j].interval = sample_interval;
		chip->kdata.fifo[j].count = sample_count;

		chip->last_fifo_v_uv = chip->kdata.fifo[j].v;
		chip->last_fifo_i_ua = chip->kdata.fifo[j].i;

		qg_dbg(chip, QG_DEBUG_FIFO, "FIFO %d raw_v=%d uV=%d raw_i=%d uA=%d interval=%d count=%d\n",
					j, fifo_v,
					chip->kdata.fifo[j].v,
					qg_v_mode ? 0 : fifo_i,
					(int)chip->kdata.fifo[j].i,
					chip->kdata.fifo[j].interval,
					chip->kdata.fifo[j].count);
	}

	chip->kdata.fifo_length += fifo_length;
	chip->kdata.seq_no = chip->seq_no++ % U32_MAX;

	return rc;
}

static int qg_process_accumulator(struct qpnp_qg *chip)
{
	int rc, sample_interval = 0;
	u8 count, index = chip->kdata.fifo_length;
	u64 acc_v = 0, acc_i = 0;
	s64 temp = 0;
	bool qg_v_mode = (chip->qg_mode == QG_V_MODE);

	rc = qg_read(chip, chip->qg_base + QG_ACCUM_CNT_RT_REG,
			&count, 1);
	if (rc < 0) {
		pr_err("Failed to read ACC count, rc=%d\n", rc);
		return rc;
	}

	if (!count || count < 10) { /* Ignore small accumulator data */
		pr_debug("No ACCUMULATOR data!\n");
		return 0;
	}

	rc = get_sample_interval(chip, &sample_interval);
	if (rc < 0) {
		pr_err("Failed to get ACC sample interval, rc=%d\n", rc);
		return 0;
	}

	rc = qg_read(chip, chip->qg_base + QG_V_ACCUM_DATA0_RT_REG,
			(u8 *)&acc_v, 3);
	if (rc < 0) {
		pr_err("Failed to read ACC RT V data, rc=%d\n", rc);
		return rc;
	}

	rc = qg_read(chip, chip->qg_base + QG_I_ACCUM_DATA0_RT_REG,
			(u8 *)&acc_i, 3);
	if (rc < 0) {
		pr_err("Failed to read ACC RT I data, rc=%d\n", rc);
		return rc;
	}

	temp = sign_extend64(acc_i, 23);

	chip->kdata.fifo[index].v = V_RAW_TO_UV(div_u64(acc_v, count));
	chip->kdata.fifo[index].i = qg_v_mode ?
				0 : qg_iraw_to_ua(chip, div_s64(temp, count));
	chip->kdata.fifo[index].interval = sample_interval;
	chip->kdata.fifo[index].count = count;
	chip->kdata.fifo_length++;
	if (chip->kdata.fifo_length == MAX_FIFO_LENGTH)
		chip->kdata.fifo_length = MAX_FIFO_LENGTH - 1;

	chip->last_fifo_v_uv = chip->kdata.fifo[index].v;
	chip->last_fifo_i_ua = chip->kdata.fifo[index].i;

	if (chip->kdata.fifo_length == 1)	/* Only accumulator data */
		chip->kdata.seq_no = chip->seq_no++ % U32_MAX;

	qg_dbg(chip, QG_DEBUG_FIFO, "ACC v_avg=%duV i_avg=%duA interval=%d count=%d\n",
			chip->kdata.fifo[index].v,
			(int)chip->kdata.fifo[index].i,
			chip->kdata.fifo[index].interval,
			chip->kdata.fifo[index].count);

	return rc;
}

static int qg_process_rt_fifo(struct qpnp_qg *chip)
{
	int rc;
	u32 fifo_length = 0;

	/* Get the real-time FIFO length */
	rc = get_fifo_length(chip, &fifo_length, true);
	if (rc < 0) {
		pr_err("Failed to read RT FIFO length, rc=%d\n", rc);
		return rc;
	}

	rc = qg_process_fifo(chip, fifo_length);
	if (rc < 0) {
		pr_err("Failed to process FIFO data, rc=%d\n", rc);
		return rc;
	}

	rc = qg_process_accumulator(chip);
	if (rc < 0) {
		pr_err("Failed to process ACC data, rc=%d\n", rc);
		return rc;
	}

	return rc;
}

#define MIN_FIFO_FULL_TIME_MS			12000
static int process_rt_fifo_data(struct qpnp_qg *chip, bool update_smb)
{
	int rc = 0;
	ktime_t now = ktime_get_boottime();
	s64 time_delta;

	/*
	 * Reject the FIFO read event if there are back-to-back requests
	 * This is done to gaurantee that there is always a minimum FIFO
	 * data to be processed, ignore this if vbat_low is set.
	 */
	time_delta = ktime_ms_delta(now, chip->last_user_update_time);

	qg_dbg(chip, QG_DEBUG_FIFO, "time_delta=%lld ms update_smb=%d\n",
				time_delta, update_smb);

	if (time_delta > MIN_FIFO_FULL_TIME_MS || update_smb) {
		rc = qg_master_hold(chip, true);
		if (rc < 0) {
			pr_err("Failed to hold master, rc=%d\n", rc);
			goto done;
		}

		rc = qg_process_rt_fifo(chip);
		if (rc < 0) {
			pr_err("Failed to process FIFO real-time, rc=%d\n", rc);
			goto done;
		}

		if (update_smb) {
			rc = qg_masked_write(chip, chip->qg_base +
				QG_MODE_CTL1_REG, PARALLEL_IBAT_SENSE_EN_BIT,
				chip->parallel_enabled ?
					PARALLEL_IBAT_SENSE_EN_BIT : 0);
			if (rc < 0) {
				pr_err("Failed to update SMB_EN, rc=%d\n", rc);
				goto done;
			}
			qg_dbg(chip, QG_DEBUG_STATUS, "Parallel SENSE %d\n",
						chip->parallel_enabled);
		}

		rc = qg_master_hold(chip, false);
		if (rc < 0) {
			pr_err("Failed to release master, rc=%d\n", rc);
			goto done;
		}
		/* FIFOs restarted */
		chip->last_fifo_update_time = ktime_get_boottime();

		/* signal the read thread */
		chip->data_ready = true;
		wake_up_interruptible(&chip->qg_wait_q);
		chip->last_user_update_time = now;

		/* vote to stay awake until userspace reads data */
		vote(chip->awake_votable, FIFO_RT_DONE_VOTER, true, 0);
	} else {
		qg_dbg(chip, QG_DEBUG_FIFO, "FIFO processing too early time_delta=%lld\n",
							time_delta);
	}
done:
	qg_master_hold(chip, false);
	return rc;
}

#define VBAT_LOW_HYST_UV		50000 /* 50mV */
static int qg_vbat_low_wa(struct qpnp_qg *chip)
{
	int rc, i, temp = 0;
	u32 vbat_low_uv = 0;

	if (chip->wa_flags & QG_VBAT_LOW_WA) {
		rc = qg_get_battery_temp(chip, &temp);
		if (rc < 0) {
			pr_err("Failed to read batt_temp rc=%d\n", rc);
			temp = 250;
		}

		vbat_low_uv = 1000 * ((temp < chip->dt.cold_temp_threshold) ?
					chip->dt.vbatt_low_cold_mv :
					chip->dt.vbatt_low_mv);

		for (i = 0; i < chip->kdata.fifo_length; i++) {
			if ((chip->kdata.fifo[i].v > (vbat_low_uv +
					VBAT_LOW_HYST_UV)) && chip->vbat_low) {
				chip->vbat_low = false;
				pr_info("Exit VBAT_LOW vbat_avg=%duV vbat_low=%duV\n",
					chip->kdata.fifo[i].v, vbat_low_uv);
				break;
			} else if ((chip->kdata.fifo[i].v < vbat_low_uv) &&
							!chip->vbat_low) {
				chip->vbat_low = true;
				pr_info("Enter VBAT_LOW vbat_avg=%duV vbat_low=%duV\n",
					chip->kdata.fifo[i].v, vbat_low_uv);
				break;
			}
		}
	}

	rc = qg_config_s2_state(chip, S2_LOW_VBAT,
			chip->vbat_low ? true : false, false);
	if (rc < 0)
		pr_err("Failed to configure for VBAT_LOW rc=%d\n", rc);

	return rc;
}

static int qg_vbat_thresholds_config(struct qpnp_qg *chip)
{
	int rc, temp = 0, vbat_mv;
	u8 reg;

	rc = qg_get_battery_temp(chip, &temp);
	if (rc < 0) {
		pr_err("Failed to read batt_temp rc=%d\n", rc);
		return rc;
	}

	vbat_mv = (temp < chip->dt.cold_temp_threshold) ?
			chip->dt.vbatt_empty_cold_mv :
			chip->dt.vbatt_empty_mv;

	rc = qg_read(chip, chip->qg_base + QG_VBAT_EMPTY_THRESHOLD_REG,
					&reg, 1);
	if (rc < 0) {
		pr_err("Failed to read vbat-empty, rc=%d\n", rc);
		return rc;
	}

	if (vbat_mv == (reg * 50))	/* No change */
		goto config_vbat_low;

	reg = vbat_mv / 50;
	rc = qg_write(chip, chip->qg_base + QG_VBAT_EMPTY_THRESHOLD_REG,
					&reg, 1);
	if (rc < 0) {
		pr_err("Failed to write vbat-empty, rc=%d\n", rc);
		return rc;
	}

	qg_dbg(chip, QG_DEBUG_STATUS,
		"VBAT EMPTY threshold updated to %dmV temp=%d\n",
						vbat_mv, temp);

config_vbat_low:
	if (chip->qg_version == QG_LITE)
		return 0;

	vbat_mv = (temp < chip->dt.cold_temp_threshold) ?
			chip->dt.vbatt_low_cold_mv :
			chip->dt.vbatt_low_mv;

	rc = qg_read(chip, chip->qg_base + QG_VBAT_LOW_THRESHOLD_REG,
					&reg, 1);
	if (rc < 0) {
		pr_err("Failed to read vbat-low, rc=%d\n", rc);
		return rc;
	}

	if (vbat_mv == (reg * 50))	/* No change */
		return 0;

	reg = vbat_mv / 50;
	rc = qg_write(chip, chip->qg_base + QG_VBAT_LOW_THRESHOLD_REG,
					&reg, 1);
	if (rc < 0) {
		pr_err("Failed to write vbat-low, rc=%d\n", rc);
		return rc;
	}

	qg_dbg(chip, QG_DEBUG_STATUS,
		"VBAT LOW threshold updated to %dmV temp=%d\n",
						vbat_mv, temp);

	return rc;
}

static int qg_fast_charge_config(struct qpnp_qg *chip)
{
	int rc = 0;

	if (!chip->dt.qg_fast_chg_cfg)
		return 0;

	rc = qg_config_s2_state(chip, S2_FAST_CHARGING,
			(chip->charge_status == POWER_SUPPLY_STATUS_CHARGING)
			? true : false, false);
	if (rc < 0)
		pr_err("Failed to exit S2_SLEEP rc=%d\n", rc);

	return rc;
}

static void qg_retrieve_esr_params(struct qpnp_qg *chip)
{
	u32 data = 0;
	int rc;

	rc = qg_sdam_read(SDAM_ESR_CHARGE_DELTA, &data);
	if (!rc && data) {
		chip->kdata.param[QG_ESR_CHARGE_DELTA].data = data;
		chip->kdata.param[QG_ESR_CHARGE_DELTA].valid = true;
		qg_dbg(chip, QG_DEBUG_ESR,
				"ESR_CHARGE_DELTA SDAM=%d\n", data);
	} else if (rc < 0) {
		pr_err("Failed to read ESR_CHARGE_DELTA rc=%d\n", rc);
	}

	rc = qg_sdam_read(SDAM_ESR_DISCHARGE_DELTA, &data);
	if (!rc && data) {
		chip->kdata.param[QG_ESR_DISCHARGE_DELTA].data = data;
		chip->kdata.param[QG_ESR_DISCHARGE_DELTA].valid = true;
		qg_dbg(chip, QG_DEBUG_ESR,
				"ESR_DISCHARGE_DELTA SDAM=%d\n", data);
	} else if (rc < 0) {
		pr_err("Failed to read ESR_DISCHARGE_DELTA rc=%d\n", rc);
	}

	rc = qg_sdam_read(SDAM_ESR_CHARGE_SF, &data);
	if (!rc && data) {
		data = CAP(QG_ESR_SF_MIN, QG_ESR_SF_MAX, data);
		chip->kdata.param[QG_ESR_CHARGE_SF].data = data;
		chip->kdata.param[QG_ESR_CHARGE_SF].valid = true;
		qg_dbg(chip, QG_DEBUG_ESR,
				"ESR_CHARGE_SF SDAM=%d\n", data);
	} else if (rc < 0) {
		pr_err("Failed to read ESR_CHARGE_SF rc=%d\n", rc);
	}

	rc = qg_sdam_read(SDAM_ESR_DISCHARGE_SF, &data);
	if (!rc && data) {
		data = CAP(QG_ESR_SF_MIN, QG_ESR_SF_MAX, data);
		chip->kdata.param[QG_ESR_DISCHARGE_SF].data = data;
		chip->kdata.param[QG_ESR_DISCHARGE_SF].valid = true;
		qg_dbg(chip, QG_DEBUG_ESR,
				"ESR_DISCHARGE_SF SDAM=%d\n", data);
	} else if (rc < 0) {
		pr_err("Failed to read ESR_DISCHARGE_SF rc=%d\n", rc);
	}
}

static void qg_store_esr_params(struct qpnp_qg *chip)
{
	unsigned int esr;

	if (chip->udata.param[QG_ESR_CHARGE_DELTA].valid) {
		esr = chip->udata.param[QG_ESR_CHARGE_DELTA].data;
		qg_sdam_write(SDAM_ESR_CHARGE_DELTA, esr);
		qg_dbg(chip, QG_DEBUG_ESR,
			"SDAM store ESR_CHARGE_DELTA=%d\n", esr);
	}

	if (chip->udata.param[QG_ESR_DISCHARGE_DELTA].valid) {
		esr = chip->udata.param[QG_ESR_DISCHARGE_DELTA].data;
		qg_sdam_write(SDAM_ESR_DISCHARGE_DELTA, esr);
		qg_dbg(chip, QG_DEBUG_ESR,
			"SDAM store ESR_DISCHARGE_DELTA=%d\n", esr);
	}

	if (chip->udata.param[QG_ESR_CHARGE_SF].valid) {
		esr = chip->udata.param[QG_ESR_CHARGE_SF].data;
		qg_sdam_write(SDAM_ESR_CHARGE_SF, esr);
		qg_dbg(chip, QG_DEBUG_ESR,
			"SDAM store ESR_CHARGE_SF=%d\n", esr);
	}

	if (chip->udata.param[QG_ESR_DISCHARGE_SF].valid) {
		esr = chip->udata.param[QG_ESR_DISCHARGE_SF].data;
		qg_sdam_write(SDAM_ESR_DISCHARGE_SF, esr);
		qg_dbg(chip, QG_DEBUG_ESR,
			"SDAM store ESR_DISCHARGE_SF=%d\n", esr);
	}
}

#define MAX_ESR_RETRY_COUNT		10
#define ESR_SD_PERCENT			10
static int qg_process_esr_data(struct qpnp_qg *chip)
{
	int i;
	int pre_i, post_i, pre_v, post_v, first_pre_i = 0;
	int diff_v, diff_i, esr_avg = 0, count = 0;

	for (i = 0; i < qg_esr_count; i++) {
		if (!chip->esr_data[i].valid)
			continue;

		pre_i = chip->esr_data[i].pre_esr_i;
		pre_v = chip->esr_data[i].pre_esr_v;
		post_i = chip->esr_data[i].post_esr_i;
		post_v = chip->esr_data[i].post_esr_v;

		/*
		 * Check if any of the pre/post readings have changed
		 * signs by comparing it with the first valid
		 * pre_i value.
		 */
		if (!first_pre_i)
			first_pre_i = pre_i;

		if ((first_pre_i < 0 && pre_i > 0) ||
			(first_pre_i > 0 && post_i < 0) ||
			(first_pre_i < 0 && post_i > 0)) {
			qg_dbg(chip, QG_DEBUG_ESR,
				"ESR-sign mismatch %d reject all data\n", i);
			esr_avg = count = 0;
			break;
		}

		/* calculate ESR */
		diff_v = abs(post_v - pre_v);
		diff_i = abs(post_i - pre_i);

		if (!diff_v || !diff_i ||
			(diff_i < chip->dt.esr_qual_i_ua) ||
			(diff_v < chip->dt.esr_qual_v_uv)) {
			qg_dbg(chip, QG_DEBUG_ESR,
				"ESR (%d) V/I %duA %duV fails qualification\n",
				i, diff_i, diff_v);
			chip->esr_data[i].valid = false;
			continue;
		}

		chip->esr_data[i].esr =
			DIV_ROUND_CLOSEST(diff_v * 1000, diff_i);
		qg_dbg(chip, QG_DEBUG_ESR,
			"ESR qualified: i=%d pre_i=%d pre_v=%d post_i=%d post_v=%d esr_diff_v=%d esr_diff_i=%d esr=%d\n",
			i, pre_i, pre_v, post_i, post_v,
			diff_v, diff_i, chip->esr_data[i].esr);

		esr_avg += chip->esr_data[i].esr;
		count++;
	}

	if (!count) {
		qg_dbg(chip, QG_DEBUG_ESR,
			"No ESR samples qualified, ESR not found\n");
		chip->esr_avg = 0;
		return 0;
	}

	esr_avg /= count;
	qg_dbg(chip, QG_DEBUG_ESR,
		"ESR all sample average=%d count=%d apply_SD=%d\n",
		esr_avg, count, (esr_avg * ESR_SD_PERCENT) / 100);

	/*
	 * Reject ESR samples which do not fall in
	 * 10% the standard-deviation
	 */
	count = 0;
	for (i = 0; i < qg_esr_count; i++) {
		if (!chip->esr_data[i].valid)
			continue;

		if ((abs(chip->esr_data[i].esr - esr_avg) <=
			(esr_avg * ESR_SD_PERCENT) / 100)) {
			/* valid ESR */
			chip->esr_avg += chip->esr_data[i].esr;
			count++;
			qg_dbg(chip, QG_DEBUG_ESR,
				"Valid ESR after SD (%d) %d mOhm\n",
				i, chip->esr_data[i].esr);
		} else {
			qg_dbg(chip, QG_DEBUG_ESR,
				"ESR (%d) %d falls-out of SD(%d)\n",
				i, chip->esr_data[i].esr, ESR_SD_PERCENT);
		}
	}

	if (count >= QG_MIN_ESR_COUNT) {
		chip->esr_avg /= count;
		qg_dbg(chip, QG_DEBUG_ESR, "Average estimated ESR %d mOhm\n",
					chip->esr_avg);
	} else {
		qg_dbg(chip, QG_DEBUG_ESR,
			"Not enough ESR samples, ESR not found\n");
		chip->esr_avg = 0;
	}

	return 0;
}

static int qg_esr_estimate(struct qpnp_qg *chip)
{
	int rc, i, ibat = 0, temp = 0;
	u8 esr_done_count, reg0 = 0, reg1 = 0;
	bool is_charging = false;

	if (chip->dt.esr_disable)
		return 0;

	/*
	 * Charge - enable ESR estimation if IBAT > MIN_IBAT.
	 * Discharge - enable ESR estimation only if enabled via DT.
	 */
	rc = qg_get_battery_current(chip, &ibat);
	if (rc < 0)
		return rc;
	if (chip->charge_status == POWER_SUPPLY_STATUS_CHARGING &&
				ibat > chip->dt.esr_min_ibat_ua) {
		qg_dbg(chip, QG_DEBUG_ESR,
			"Skip CHG ESR, Fails IBAT ibat(%d) min_ibat(%d)\n",
				ibat, chip->dt.esr_min_ibat_ua);
		return 0;
	}

	if (chip->charge_status != POWER_SUPPLY_STATUS_CHARGING &&
			!chip->dt.esr_discharge_enable)
		return 0;

	/* Ignore ESR if battery-temp is below a threshold */
	rc = qg_get_battery_temp(chip, &temp);
	if (rc < 0)
		return rc;
	if (temp < chip->dt.esr_low_temp_threshold) {
		qg_dbg(chip, QG_DEBUG_ESR,
			"Battery temperature(%d) below threshold(%d) for ESR\n",
				temp, chip->dt.esr_low_temp_threshold);
		return 0;
	}

	if (chip->batt_soc != INT_MIN && (chip->batt_soc <
					chip->dt.esr_disable_soc)) {
		qg_dbg(chip, QG_DEBUG_ESR,
			"Skip ESR, batt-soc below %d\n",
				chip->dt.esr_disable_soc);
		return 0;
	}

	qg_dbg(chip, QG_DEBUG_ESR, "FIFO done count=%d ESR mod count=%d\n",
			chip->fifo_done_count, qg_esr_mod_count);

	if ((chip->fifo_done_count % qg_esr_mod_count) != 0)
		return 0;

	if (qg_esr_count > QG_MAX_ESR_COUNT)
		qg_esr_count = QG_MAX_ESR_COUNT;

	if (qg_esr_count < QG_MIN_ESR_COUNT)
		qg_esr_count = QG_MIN_ESR_COUNT;

	/* clear all data */
	chip->esr_avg = 0;
	memset(&chip->esr_data, 0, sizeof(chip->esr_data));

	rc = qg_master_hold(chip, true);
	if (rc < 0) {
		pr_err("Failed to hold master, rc=%d\n", rc);
		goto done;
	}

	for (i = 0; i < qg_esr_count; i++) {
		/* Fire ESR measurement */
		rc = qg_masked_write(chip,
			chip->qg_base + QG_ESR_MEAS_TRIG_REG,
			HW_ESR_MEAS_START_BIT, HW_ESR_MEAS_START_BIT);
		if (rc < 0) {
			pr_err("Failed to start ESR rc=%d\n", rc);
			continue;
		}

		esr_done_count = reg0 = reg1 = 0;
		do {
			/* delay for ESR processing to complete */
			msleep(50);

			esr_done_count++;

			rc = qg_read(chip,
				chip->qg_base + QG_STATUS1_REG, &reg0, 1);
			if (rc < 0)
				continue;

			rc = qg_read(chip,
				chip->qg_base + QG_STATUS4_REG, &reg1, 1);
			if (rc < 0)
				continue;

			/* check ESR-done status */
			if (!(reg1 & ESR_MEAS_IN_PROGRESS_BIT) &&
					(reg0 & ESR_MEAS_DONE_BIT)) {
				qg_dbg(chip, QG_DEBUG_ESR,
					"ESR measurement done %d count %d\n",
						i, esr_done_count);
				break;
			}
		} while (esr_done_count < MAX_ESR_RETRY_COUNT);

		if (esr_done_count == MAX_ESR_RETRY_COUNT) {
			pr_err("Failed to get ESR done for %d iteration\n", i);
			continue;
		} else {
			/* found a valid ESR, read pre-post data */
			rc = qg_read_raw_data(chip, QG_PRE_ESR_V_DATA0_REG,
					&chip->esr_data[i].pre_esr_v);
			if (rc < 0)
				goto done;

			rc = qg_read_raw_data(chip, QG_PRE_ESR_I_DATA0_REG,
					&chip->esr_data[i].pre_esr_i);
			if (rc < 0)
				goto done;

			rc = qg_read_raw_data(chip, QG_POST_ESR_V_DATA0_REG,
					&chip->esr_data[i].post_esr_v);
			if (rc < 0)
				goto done;

			rc = qg_read_raw_data(chip, QG_POST_ESR_I_DATA0_REG,
					&chip->esr_data[i].post_esr_i);
			if (rc < 0)
				goto done;

			chip->esr_data[i].pre_esr_v =
				V_RAW_TO_UV(chip->esr_data[i].pre_esr_v);
			ibat = sign_extend32(chip->esr_data[i].pre_esr_i, 15);
			chip->esr_data[i].pre_esr_i = qg_iraw_to_ua(chip, ibat);
			chip->esr_data[i].post_esr_v =
				V_RAW_TO_UV(chip->esr_data[i].post_esr_v);
			ibat = sign_extend32(chip->esr_data[i].post_esr_i, 15);
			chip->esr_data[i].post_esr_i =
						qg_iraw_to_ua(chip, ibat);

			chip->esr_data[i].valid = true;

			if ((int)chip->esr_data[i].pre_esr_i < 0)
				is_charging = true;

			qg_dbg(chip, QG_DEBUG_ESR,
				"ESR values for %d iteration pre_v=%d pre_i=%d post_v=%d post_i=%d\n",
				i, chip->esr_data[i].pre_esr_v,
				(int)chip->esr_data[i].pre_esr_i,
				chip->esr_data[i].post_esr_v,
				(int)chip->esr_data[i].post_esr_i);
		}
		/* delay before the next ESR measurement */
		msleep(200);
	}

	rc = qg_process_esr_data(chip);
	if (rc < 0)
		pr_err("Failed to process ESR data rc=%d\n", rc);

	rc = qg_master_hold(chip, false);
	if (rc < 0) {
		pr_err("Failed to release master, rc=%d\n", rc);
		goto done;
	}
	/* FIFOs restarted */
	chip->last_fifo_update_time = ktime_get_boottime();

	if (chip->esr_avg) {
		chip->kdata.param[QG_ESR].data = chip->esr_avg;
		chip->kdata.param[QG_ESR].valid = true;
		qg_dbg(chip, QG_DEBUG_ESR, "ESR_SW=%d during %s\n",
			chip->esr_avg, is_charging ? "CHARGE" : "DISCHARGE");
		qg_retrieve_esr_params(chip);
		chip->esr_actual = chip->esr_avg;
	}

	return 0;
done:
	qg_master_hold(chip, false);
	return rc;
}

static void process_udata_work(struct work_struct *work)
{
	struct qpnp_qg *chip = container_of(work,
			struct qpnp_qg, udata_work);
	int rc;

	if (chip->udata.param[QG_CC_SOC].valid)
		chip->cc_soc = chip->udata.param[QG_CC_SOC].data;

	if (chip->udata.param[QG_BATT_SOC].valid)
		chip->batt_soc = chip->udata.param[QG_BATT_SOC].data;

	if (chip->udata.param[QG_FULL_SOC].valid)
		chip->full_soc = chip->udata.param[QG_FULL_SOC].data;

	if (chip->udata.param[QG_V_IBAT].valid)
		chip->qg_v_ibat = chip->udata.param[QG_V_IBAT].data;

	if (chip->udata.param[QG_SOC].valid ||
			chip->udata.param[QG_SYS_SOC].valid) {

		qg_dbg(chip, QG_DEBUG_SOC, "udata update: QG_SOC=%d QG_SYS_SOC=%d last_catchup_soc=%d\n",
				chip->udata.param[QG_SOC].valid ?
				chip->udata.param[QG_SOC].data : -EINVAL,
				chip->udata.param[QG_SYS_SOC].valid ?
				chip->udata.param[QG_SYS_SOC].data : -EINVAL,
				chip->catch_up_soc);

		if (chip->udata.param[QG_SYS_SOC].valid) {
			chip->sys_soc = chip->udata.param[QG_SYS_SOC].data;
			chip->catch_up_soc = qg_adjust_sys_soc(chip);
		} else {
			chip->catch_up_soc = chip->udata.param[QG_SOC].data;
		}

		qg_scale_soc(chip, chip->force_soc);
		chip->force_soc = false;

		/* update parameters to SDAM */
		chip->sdam_data[SDAM_SOC] = chip->msoc;
		chip->sdam_data[SDAM_OCV_UV] =
				chip->udata.param[QG_OCV_UV].data;
		chip->sdam_data[SDAM_RBAT_MOHM] =
				chip->udata.param[QG_RBAT_MOHM].data;
		chip->sdam_data[SDAM_VALID] = 1;

		rc = qg_store_soc_params(chip);
		if (rc < 0)
			pr_err("Failed to update SDAM params, rc=%d\n", rc);
	}

	if (chip->udata.param[QG_ESR].valid)
		chip->esr_last = chip->udata.param[QG_ESR].data;

	if (chip->esr_actual != -EINVAL && chip->udata.param[QG_ESR].valid) {
		chip->esr_nominal = chip->udata.param[QG_ESR].data;
		if (chip->qg_psy)
			power_supply_changed(chip->qg_psy);
	}

	if (!chip->dt.esr_disable)
		qg_store_esr_params(chip);

	qg_dbg(chip, QG_DEBUG_STATUS, "udata update: batt_soc=%d cc_soc=%d full_soc=%d qg_esr=%d\n",
		(chip->batt_soc != INT_MIN) ? chip->batt_soc : -EINVAL,
		(chip->cc_soc != INT_MIN) ? chip->cc_soc : -EINVAL,
		chip->full_soc, chip->esr_last);
	vote(chip->awake_votable, UDATA_READY_VOTER, false, 0);
}

#define MAX_FIFO_DELTA_PERCENT		10
static irqreturn_t qg_fifo_update_done_handler(int irq, void *data)
{
	ktime_t now = ktime_get_boottime();
	int rc, hw_delta_ms = 0, margin_ms = 0;
	u32 fifo_length = 0;
	s64 time_delta_ms = 0;
	struct qpnp_qg *chip = data;

	time_delta_ms = ktime_ms_delta(now, chip->last_fifo_update_time);
	chip->last_fifo_update_time = now;

	qg_dbg(chip, QG_DEBUG_IRQ, "IRQ triggered\n");
	mutex_lock(&chip->data_lock);

	rc = get_fifo_length(chip, &fifo_length, false);
	if (rc < 0) {
		pr_err("Failed to get FIFO length, rc=%d\n", rc);
		goto done;
	}

	rc = qg_process_fifo(chip, fifo_length);
	if (rc < 0) {
		pr_err("Failed to process QG FIFO, rc=%d\n", rc);
		goto done;
	}

	if (++chip->fifo_done_count == U32_MAX)
		chip->fifo_done_count = 0;

	rc = qg_vbat_thresholds_config(chip);
	if (rc < 0)
		pr_err("Failed to apply VBAT EMPTY config rc=%d\n", rc);

	rc = qg_fast_charge_config(chip);
	if (rc < 0)
		pr_err("Failed to apply fast-charge config rc=%d\n", rc);

	rc = qg_vbat_low_wa(chip);
	if (rc < 0) {
		pr_err("Failed to apply VBAT LOW WA, rc=%d\n", rc);
		goto done;
	}

	rc = qg_esr_estimate(chip);
	if (rc < 0) {
		pr_err("Failed to estimate ESR, rc=%d\n", rc);
		goto done;
	}

	rc = get_fifo_done_time(chip, false, &hw_delta_ms);
	if (rc < 0)
		hw_delta_ms = 0;
	else
		margin_ms = (hw_delta_ms * MAX_FIFO_DELTA_PERCENT) / 100;

	if (abs(hw_delta_ms - time_delta_ms) < margin_ms) {
		chip->kdata.param[QG_FIFO_TIME_DELTA].data = time_delta_ms;
		chip->kdata.param[QG_FIFO_TIME_DELTA].valid = true;
		qg_dbg(chip, QG_DEBUG_FIFO, "FIFO_done time_delta_ms=%lld\n",
							time_delta_ms);
	}

	/* signal the read thread */
	chip->data_ready = true;
	wake_up_interruptible(&chip->qg_wait_q);

	/* vote to stay awake until userspace reads data */
	vote(chip->awake_votable, FIFO_DONE_VOTER, true, 0);

done:
	mutex_unlock(&chip->data_lock);
	return IRQ_HANDLED;
}

static irqreturn_t qg_vbat_low_handler(int irq, void *data)
{
	int rc;
	struct qpnp_qg *chip = data;
	u8 status = 0;

	qg_dbg(chip, QG_DEBUG_IRQ, "IRQ triggered\n");
	mutex_lock(&chip->data_lock);

	rc = qg_read(chip, chip->qg_base + QG_INT_RT_STS_REG, &status, 1);
	if (rc < 0) {
		pr_err("Failed to read RT status, rc=%d\n", rc);
		goto done;
	}
	/* ignore VBAT low if battery is missing */
	if ((status & BATTERY_MISSING_INT_RT_STS_BIT) ||
			chip->battery_missing)
		goto done;

	chip->vbat_low = !!(status & VBAT_LOW_INT_RT_STS_BIT);

	qg_dbg(chip, QG_DEBUG_IRQ, "VBAT_LOW = %d\n", chip->vbat_low);
done:
	mutex_unlock(&chip->data_lock);
	return IRQ_HANDLED;
}

static irqreturn_t qg_vbat_empty_handler(int irq, void *data)
{
	struct qpnp_qg *chip = data;
	u32 ocv_uv = 0;
	int rc;
	u8 status = 0;

	qg_dbg(chip, QG_DEBUG_IRQ, "IRQ triggered\n");

	rc = qg_read(chip, chip->qg_base + QG_INT_RT_STS_REG, &status, 1);
	if (rc < 0)
		pr_err("Failed to read RT status rc=%d\n", rc);

	/* ignore VBAT empty if battery is missing */
	if ((status & BATTERY_MISSING_INT_RT_STS_BIT) ||
			chip->battery_missing)
		return IRQ_HANDLED;

	pr_warn("VBATT EMPTY SOC = 0\n");

	chip->catch_up_soc = 0;
	qg_scale_soc(chip, true);

	qg_sdam_read(SDAM_OCV_UV, &ocv_uv);
	chip->sdam_data[SDAM_SOC] = 0;
	chip->sdam_data[SDAM_OCV_UV] = ocv_uv;
	chip->sdam_data[SDAM_VALID] = 1;

	qg_store_soc_params(chip);

	if (chip->qg_psy)
		power_supply_changed(chip->qg_psy);

	return IRQ_HANDLED;
}

static irqreturn_t qg_good_ocv_handler(int irq, void *data)
{
	int rc;
	u8 status = 0;
	u32 ocv_uv = 0, ocv_raw = 0;
	struct qpnp_qg *chip = data;
	unsigned long rtc_sec = 0;

	qg_dbg(chip, QG_DEBUG_IRQ, "IRQ triggered\n");

	mutex_lock(&chip->data_lock);

	rc = qg_read(chip, chip->qg_base + QG_STATUS2_REG, &status, 1);
	if (rc < 0) {
		pr_err("Failed to read status2 register rc=%d\n", rc);
		goto done;
	}

	if (!(status & GOOD_OCV_BIT))
		goto done;

	rc = qg_read_ocv(chip, &ocv_uv, &ocv_raw, S3_GOOD_OCV);
	if (rc < 0) {
		pr_err("Failed to read good_ocv, rc=%d\n", rc);
		goto done;
	}

	get_rtc_time(&rtc_sec);
	chip->kdata.fifo_time = (u32)rtc_sec;
	chip->kdata.param[QG_GOOD_OCV_UV].data = ocv_uv;
	chip->kdata.param[QG_GOOD_OCV_UV].valid = true;

	vote(chip->awake_votable, GOOD_OCV_VOTER, true, 0);

	/* signal the readd thread */
	chip->data_ready = true;
	wake_up_interruptible(&chip->qg_wait_q);
done:
	mutex_unlock(&chip->data_lock);
	return IRQ_HANDLED;
}

static struct qg_irq_info qg_irqs[] = {
	[QG_BATT_MISSING_IRQ] = {
		.name		= "qg-batt-missing",
	},
	[QG_VBATT_LOW_IRQ] = {
		.name		= "qg-vbat-low",
		.handler	= qg_vbat_low_handler,
		.wake		= true,
	},
	[QG_VBATT_EMPTY_IRQ] = {
		.name		= "qg-vbat-empty",
		.handler	= qg_vbat_empty_handler,
		.wake		= true,
	},
	[QG_FIFO_UPDATE_DONE_IRQ] = {
		.name		= "qg-fifo-done",
		.handler	= qg_fifo_update_done_handler,
		.wake		= true,
	},
	[QG_GOOD_OCV_IRQ] = {
		.name		= "qg-good-ocv",
		.handler	= qg_good_ocv_handler,
		.wake		= true,
	},
	[QG_FSM_STAT_CHG_IRQ] = {
		.name		= "qg-fsm-state-chg",
	},
	[QG_EVENT_IRQ] = {
		.name		= "qg-event",
	},
};

static int qg_awake_cb(struct votable *votable, void *data, int awake,
			const char *client)
{
	struct qpnp_qg *chip = data;

	/* ignore if the QG device is not open */
	if (!chip->qg_device_open)
		return 0;

	if (awake)
		pm_stay_awake(chip->dev);
	else
		pm_relax(chip->dev);

	pr_debug("client: %s awake: %d\n", client, awake);
	return 0;
}

static int qg_fifo_irq_disable_cb(struct votable *votable, void *data,
				int disable, const char *client)
{
	if (disable) {
		if (qg_irqs[QG_FIFO_UPDATE_DONE_IRQ].wake)
			disable_irq_wake(
				qg_irqs[QG_FIFO_UPDATE_DONE_IRQ].irq);
		if (qg_irqs[QG_FIFO_UPDATE_DONE_IRQ].irq)
			disable_irq_nosync(
				qg_irqs[QG_FIFO_UPDATE_DONE_IRQ].irq);
	} else {
		if (qg_irqs[QG_FIFO_UPDATE_DONE_IRQ].irq)
			enable_irq(qg_irqs[QG_FIFO_UPDATE_DONE_IRQ].irq);
		if (qg_irqs[QG_FIFO_UPDATE_DONE_IRQ].wake)
			enable_irq_wake(
				qg_irqs[QG_FIFO_UPDATE_DONE_IRQ].irq);
	}

	return 0;
}

static int qg_vbatt_irq_disable_cb(struct votable *votable, void *data,
				int disable, const char *client)
{
	if (disable) {
		if (qg_irqs[QG_VBATT_LOW_IRQ].wake)
			disable_irq_wake(qg_irqs[QG_VBATT_LOW_IRQ].irq);
		if (qg_irqs[QG_VBATT_EMPTY_IRQ].wake)
			disable_irq_wake(qg_irqs[QG_VBATT_EMPTY_IRQ].irq);
		if (qg_irqs[QG_VBATT_LOW_IRQ].irq)
			disable_irq_nosync(qg_irqs[QG_VBATT_LOW_IRQ].irq);
		if (qg_irqs[QG_VBATT_EMPTY_IRQ].irq)
			disable_irq_nosync(qg_irqs[QG_VBATT_EMPTY_IRQ].irq);
	} else {
		if (qg_irqs[QG_VBATT_LOW_IRQ].irq)
			enable_irq(qg_irqs[QG_VBATT_LOW_IRQ].irq);
		if (qg_irqs[QG_VBATT_EMPTY_IRQ].irq)
			enable_irq(qg_irqs[QG_VBATT_EMPTY_IRQ].irq);
		if (qg_irqs[QG_VBATT_LOW_IRQ].wake)
			enable_irq_wake(qg_irqs[QG_VBATT_LOW_IRQ].irq);
		if (qg_irqs[QG_VBATT_EMPTY_IRQ].wake)
			enable_irq_wake(qg_irqs[QG_VBATT_EMPTY_IRQ].irq);
	}

	return 0;
}

static int qg_good_ocv_irq_disable_cb(struct votable *votable, void *data,
				int disable, const char *client)
{
	if (disable) {
		if (qg_irqs[QG_GOOD_OCV_IRQ].wake)
			disable_irq_wake(qg_irqs[QG_GOOD_OCV_IRQ].irq);
		if (qg_irqs[QG_GOOD_OCV_IRQ].irq)
			disable_irq_nosync(qg_irqs[QG_GOOD_OCV_IRQ].irq);
	} else {
		if (qg_irqs[QG_GOOD_OCV_IRQ].irq)
			enable_irq(qg_irqs[QG_GOOD_OCV_IRQ].irq);
		if (qg_irqs[QG_GOOD_OCV_IRQ].wake)
			enable_irq_wake(qg_irqs[QG_GOOD_OCV_IRQ].irq);
	}

	return 0;
}

/* ALG callback functions below */

static int qg_get_learned_capacity(void *data, int64_t *learned_cap_uah)
{
	struct qpnp_qg *chip = data;
	int16_t cc_mah;
	int rc;

	if (!chip)
		return -ENODEV;

	if (chip->battery_missing || !chip->profile_loaded)
		return -ENODEV;

	rc = qg_sdam_multibyte_read(QG_SDAM_LEARNED_CAPACITY_OFFSET,
					(u8 *)&cc_mah, 2);
	if (rc < 0) {
		pr_err("Error in reading learned_capacity, rc=%d\n", rc);
		return rc;
	}
	*learned_cap_uah = cc_mah * 1000;

	return 0;
}

static int qg_store_learned_capacity(void *data, int64_t learned_cap_uah)
{
	struct qpnp_qg *chip = data;
	int16_t cc_mah;
	int rc;

	if (!chip)
		return -ENODEV;

	if (chip->battery_missing || !learned_cap_uah)
		return -ENODEV;

#if !defined(CONFIG_SOMC_CHARGER_EXTENSION)
	cc_mah = div64_s64(learned_cap_uah, 1000);
#endif
#if defined(CONFIG_SOMC_CHARGER_EXTENSION)
	cc_mah = DIV_ROUND_CLOSEST(learned_cap_uah, 1000);
#endif
	rc = qg_sdam_multibyte_write(QG_SDAM_LEARNED_CAPACITY_OFFSET,
					 (u8 *)&cc_mah, 2);
	if (rc < 0) {
		pr_err("Error in writing learned_capacity, rc=%d\n", rc);
		return rc;
	}

#if !defined(CONFIG_SOMC_CHARGER_EXTENSION)
	qg_dbg(chip, QG_DEBUG_ALG_CL, "Stored learned capacity %llduah\n",
					learned_cap_uah);
#endif
#if defined(CONFIG_SOMC_CHARGER_EXTENSION)
	qg_dbg(chip, QG_DEBUG_SOMC, "Stored learned capacity %llduah\n",
					learned_cap_uah);
#endif
	return 0;
}

static int qg_get_batt_age_level(void *data, u32 *batt_age_level)
{
	struct qpnp_qg *chip = data;
	int rc;

	if (!chip)
		return -ENODEV;

	if (chip->battery_missing || is_debug_batt_id(chip))
		return -ENODEV;

	*batt_age_level = 0;
	rc = qg_sdam_read(SDAM_BATT_AGE_LEVEL, batt_age_level);
	if (rc < 0) {
		pr_err("Error in reading batt_age_level, rc=%d\n", rc);
		return rc;
	}

	return 0;
}

static int qg_store_batt_age_level(void *data, u32 batt_age_level)
{
	struct qpnp_qg *chip = data;
	int rc;

	if (!chip)
		return -ENODEV;

	if (chip->battery_missing)
		return -ENODEV;

	rc = qg_sdam_write(SDAM_BATT_AGE_LEVEL, batt_age_level);
	if (rc < 0) {
		pr_err("Error in writing batt_age_level, rc=%d\n", rc);
		return rc;
	}

	return 0;
}

#if defined(CONFIG_SOMC_CHARGER_EXTENSION)
static int qg_somc_get_monotonic_soc(void *data, int *msoc)
{
	struct qpnp_qg *chip = data;

	if (!chip)
		return -ENODEV;

	*msoc = chip->msoc;

	return 0;
}

#endif

static int qg_get_cc_soc(void *data, int *cc_soc)
{
	struct qpnp_qg *chip = data;

	if (!chip)
		return -ENODEV;

	if (is_debug_batt_id(chip) || chip->battery_missing) {
		*cc_soc = -EINVAL;
		return 0;
	}

	if (chip->cc_soc == INT_MIN)
		*cc_soc = -EINVAL;
	else
		*cc_soc = chip->cc_soc;

	return 0;
}

static int qg_restore_cycle_count(void *data, u16 *buf, int length)
{
	struct qpnp_qg *chip = data;
	int id, rc = 0;
	u8 tmp[2];

	if (!chip)
		return -ENODEV;

	if (chip->battery_missing || !chip->profile_loaded)
		return -ENODEV;

	if (!buf || length > BUCKET_COUNT)
		return -EINVAL;

	for (id = 0; id < length; id++) {
		rc = qg_sdam_multibyte_read(
				QG_SDAM_CYCLE_COUNT_OFFSET + (id * 2),
				(u8 *)tmp, 2);
		if (rc < 0) {
			pr_err("failed to read bucket %d rc=%d\n", id, rc);
			return rc;
		}
		*buf++ = tmp[0] | tmp[1] << 8;
	}

	return rc;
}

static int qg_store_cycle_count(void *data, u16 *buf, int id, int length)
{
	struct qpnp_qg *chip = data;
	int rc = 0;

	if (!chip)
		return -ENODEV;

	if (chip->battery_missing || !chip->profile_loaded)
		return -ENODEV;

	if (!buf || length > BUCKET_COUNT * 2 || id < 0 ||
		id > BUCKET_COUNT - 1 ||
		(((id * 2) + length) > BUCKET_COUNT * 2))
		return -EINVAL;

	rc = qg_sdam_multibyte_write(
			QG_SDAM_CYCLE_COUNT_OFFSET + (id * 2),
			(u8 *)buf, length);
	if (rc < 0)
		pr_err("failed to write bucket %d rc=%d\n", id, rc);

	return rc;
}

#define DEFAULT_BATT_TYPE	"Unknown Battery"
#define MISSING_BATT_TYPE	"Missing Battery"
#define DEBUG_BATT_TYPE		"Debug Board"
static const char *qg_get_battery_type(struct qpnp_qg *chip)
{
	if (chip->battery_missing)
		return MISSING_BATT_TYPE;

	if (is_debug_batt_id(chip))
		return DEBUG_BATT_TYPE;

	if (chip->bp.batt_type_str) {
		if (chip->profile_loaded)
			return chip->bp.batt_type_str;
	}

	return DEFAULT_BATT_TYPE;
}

#define DEBUG_BATT_SOC		67
#define BATT_MISSING_SOC	50
#define EMPTY_SOC		0
#define FULL_SOC		100
static int qg_get_battery_capacity(struct qpnp_qg *chip, int *soc)
{
	if (is_debug_batt_id(chip)) {
		*soc = DEBUG_BATT_SOC;
		return 0;
	}

	if (chip->battery_missing || !chip->profile_loaded) {
		*soc = BATT_MISSING_SOC;
		return 0;
	}

	if (chip->charge_full) {
		*soc = FULL_SOC;
		return 0;
	}

	mutex_lock(&chip->soc_lock);

	if (chip->dt.linearize_soc && chip->maint_soc > 0)
		*soc = chip->maint_soc;
	else
		*soc = chip->msoc;

	mutex_unlock(&chip->soc_lock);

	return 0;
}

static int qg_get_battery_capacity_real(struct qpnp_qg *chip, int *soc)
{
	mutex_lock(&chip->soc_lock);
	*soc = chip->msoc;
	mutex_unlock(&chip->soc_lock);

	return 0;
}

static int qg_get_charge_counter(struct qpnp_qg *chip, int *charge_counter)
{
	int rc, cc_soc = 0;
	int64_t temp = 0;

	if (is_debug_batt_id(chip) || chip->battery_missing) {
		*charge_counter = -EINVAL;
		return 0;
	}

	rc = qg_get_learned_capacity(chip, &temp);
	if (rc < 0 || !temp)
		rc = qg_get_nominal_capacity((int *)&temp, 250, true);

	if (rc < 0) {
		pr_err("Failed to get FCC for charge-counter rc=%d\n", rc);
		return rc;
	}

	cc_soc = CAP(0, 100, DIV_ROUND_CLOSEST(chip->cc_soc, 100));
	*charge_counter = div_s64(temp * cc_soc, 100);

	return 0;
}

static int qg_get_power(struct qpnp_qg *chip, int *val, bool average)
{
	int rc, v_min, v_ocv, rbatt = 0, esr = 0;
	s64 power;

	if (is_debug_batt_id(chip)) {
		*val = -EINVAL;
		return 0;
	}

	v_min = chip->dt.sys_min_volt_mv * 1000;

	rc = qg_sdam_read(SDAM_OCV_UV, &v_ocv);
	if (rc < 0) {
		pr_err("Failed to read OCV rc=%d\n", rc);
		return rc;
	}

	rc = qg_sdam_read(SDAM_RBAT_MOHM, &rbatt);
	if (rc < 0) {
		pr_err("Failed to read T_RBAT rc=%d\n", rc);
		return rc;
	}

	rbatt *= 1000;	/* uohms */
	esr = chip->esr_last * 1000;

	if (rbatt <= 0 || esr <= 0) {
		pr_debug("Invalid rbatt/esr rbatt=%d esr=%d\n", rbatt, esr);
		*val = -EINVAL;
		return 0;
	}

	power = (s64)v_min * (v_ocv - v_min);

	if (average)
		power = div_s64(power, rbatt);
	else
		power = div_s64(power, esr);

	*val = power;

	qg_dbg(chip, QG_DEBUG_STATUS, "v_min=%d v_ocv=%d rbatt=%d esr=%d power=%lld\n",
			v_min, v_ocv, rbatt, esr, power);

	return 0;
}

static int qg_get_ttf_param(void *data, enum ttf_param param, int *val)
{
	struct qpnp_qg *chip = data;
	int rc = 0;
	int64_t temp = 0;

	if (!chip)
		return -ENODEV;

	switch (param) {
	case TTF_TTE_VALID:
		*val = 1;
		if (chip->battery_missing || is_debug_batt_id(chip))
			*val = 0;
		break;
	case TTF_MSOC:
		rc = qg_get_battery_capacity(chip, val);
		break;
	case TTF_VBAT:
		rc = qg_get_battery_voltage(chip, val);
		break;
	case TTF_IBAT:
		rc = qg_get_battery_current(chip, val);
		break;
	case TTF_FCC:
		if (!chip->dt.cl_disable && chip->dt.cl_feedback_on)
			rc = qg_get_learned_capacity(chip, &temp);
		else
			rc = qg_get_nominal_capacity((int *)&temp, 250,
							true);
		if (!rc) {
			temp = div64_u64(temp, 1000);
			*val  = div64_u64(chip->full_soc * temp,
					QG_SOC_FULL);
		}
		break;
	case TTF_MODE:
		if (chip->ttf->step_chg_cfg_valid)
			*val = TTF_MODE_VBAT_STEP_CHG;
		else
			*val = TTF_MODE_NORMAL;
		break;
	case TTF_ITERM:
		if (chip->chg_iterm_ma == INT_MIN)
			*val = 0;
		else
			*val = chip->chg_iterm_ma;
		break;
	case TTF_RBATT:
		rc = qg_sdam_read(SDAM_RBAT_MOHM, val);
		if (!rc)
			*val *= 1000;
		break;
	case TTF_VFLOAT:
		*val = chip->bp.float_volt_uv;
		break;
	case TTF_CHG_TYPE:
		*val = chip->charge_type;
		break;
	case TTF_CHG_STATUS:
		*val = chip->charge_status;
		break;
	case TTF_CHG_DONE:
		*val = chip->charge_done;
		break;
	default:
		pr_err("Unsupported property %d\n", param);
		rc = -EINVAL;
		break;
	}

	return rc;
}

static int qg_ttf_awake_voter(void *data, bool val)
{
	struct qpnp_qg *chip = data;

	if (!chip)
		return -ENODEV;

	if (chip->battery_missing || !chip->profile_loaded)
		return -ENODEV;

	vote(chip->awake_votable, TTF_AWAKE_VOTER, val, 0);

	return 0;
}

#define MAX_QG_OK_RETRIES	20
static int qg_reset(struct qpnp_qg *chip)
{
	int rc = 0, count = 0, soc = 0;
	u32 ocv_uv = 0, ocv_raw = 0;
	u8 reg = 0;

	qg_dbg(chip, QG_DEBUG_STATUS, "QG RESET triggered\n");

	mutex_lock(&chip->data_lock);

	/* hold and release master to clear FIFO's */
	rc = qg_master_hold(chip, true);
	if (rc < 0) {
		pr_err("Failed to hold master, rc=%d\n", rc);
		goto done;
	}

	/* delay for the master-hold */
	msleep(20);

	rc = qg_master_hold(chip, false);
	if (rc < 0) {
		pr_err("Failed to release master, rc=%d\n", rc);
		goto done;
	}

	/* delay for master to settle */
	msleep(20);

	qg_get_battery_voltage(chip, &rc);
	qg_get_battery_capacity(chip, &soc);
	qg_dbg(chip, QG_DEBUG_STATUS, "VBAT=%duV SOC=%d\n", rc, soc);

	/* Trigger S7 */
	rc = qg_masked_write(chip, chip->qg_base + QG_STATE_TRIG_CMD_REG,
				S7_PON_OCV_START, S7_PON_OCV_START);
	if (rc < 0) {
		pr_err("Failed to trigger S7, rc=%d\n", rc);
		goto done;
	}

	/* poll for QG OK */
	do {
		rc = qg_read(chip, chip->qg_base + QG_STATUS1_REG, &reg, 1);
		if (rc < 0) {
			pr_err("Failed to read STATUS1_REG rc=%d\n", rc);
			goto done;
		}

		if (reg & QG_OK_BIT)
			break;

		msleep(200);
		count++;
	} while (count < MAX_QG_OK_RETRIES);

	if (count == MAX_QG_OK_RETRIES) {
		qg_dbg(chip, QG_DEBUG_STATUS, "QG_OK not set\n");
		goto done;
	}

	/* read S7 PON OCV */
	rc = qg_read_ocv(chip, &ocv_uv, &ocv_raw, S7_PON_OCV);
	if (rc < 0) {
		pr_err("Failed to read PON OCV rc=%d\n", rc);
		goto done;
	}

	qg_dbg(chip, QG_DEBUG_STATUS, "S7_OCV = %duV\n", ocv_uv);

	chip->kdata.param[QG_GOOD_OCV_UV].data = ocv_uv;
	chip->kdata.param[QG_GOOD_OCV_UV].valid = true;
	/* clear all the userspace data */
	chip->kdata.param[QG_CLEAR_LEARNT_DATA].data = 1;
	chip->kdata.param[QG_CLEAR_LEARNT_DATA].valid = true;

	vote(chip->awake_votable, GOOD_OCV_VOTER, true, 0);
	/* signal the read thread */
	chip->data_ready = true;
	chip->force_soc = true;
	wake_up_interruptible(&chip->qg_wait_q);

done:
	mutex_unlock(&chip->data_lock);
	return rc;
}

static int qg_setprop_batt_age_level(struct qpnp_qg *chip, int batt_age_level)
{
#if defined(CONFIG_SOMC_CHARGER_EXTENSION)
	int rc = 0;

	if (batt_age_level != chip->batt_age_level) {
		qg_dbg(chip, QG_DEBUG_SOMC, "Update batt age level from %d to %d\n",
					chip->batt_age_level, batt_age_level);
		chip->batt_age_level = batt_age_level;

		rc = qg_store_batt_age_level(chip, batt_age_level);
		if (rc < 0)
			pr_err("error in storing batt_age_level rc =%d\n", rc);

		if (chip->cl->active)
			cap_learning_abort(chip->cl);

		if (chip->qg_psy)
			power_supply_changed(chip->qg_psy);
	}
	return 0;
#endif
#if !defined(CONFIG_SOMC_CHARGER_EXTENSION)
	int rc = 0;
	u16 data = 0;

	if (!chip->dt.multi_profile_load)
		return 0;

	if (batt_age_level < 0) {
		pr_err("Invalid age-level %d\n", batt_age_level);
		return -EINVAL;
	}

	if (chip->batt_age_level == batt_age_level) {
		qg_dbg(chip, QG_DEBUG_PROFILE, "Same age-level %d\n",
						chip->batt_age_level);
		return 0;
	}

	chip->batt_age_level = batt_age_level;
	rc = qg_load_battery_profile(chip);
	if (rc < 0) {
		pr_err("failed to load profile\n");
	} else {
		rc = qg_store_batt_age_level(chip, batt_age_level);
		if (rc < 0)
			pr_err("error in storing batt_age_level rc =%d\n", rc);
	}
	/* Clear the learned capacity on loading a new profile */
	rc = qg_sdam_multibyte_write(QG_SDAM_LEARNED_CAPACITY_OFFSET,
						(u8 *)&data, 2);

	if (rc < 0)
		pr_err("Failed to clear SDAM learnt capacity rc=%d\n", rc);
	qg_dbg(chip, QG_DEBUG_PROFILE, "Profile with batt_age_level = %d loaded\n",
						chip->batt_age_level);

	return rc;
#endif
}

#if defined(CONFIG_SOMC_CHARGER_EXTENSION)
int qg_somc_get_aux_temp(struct qpnp_qg *chip, int *val)
{
	int rc = 0;

	if (chip->aux_temp_chan) {
		rc = iio_read_channel_processed(chip->aux_temp_chan, val);
		if (rc < 0)
			return -ENODATA;
	} else {
		return -ENODATA;
	}
	*val /= 100;
	return 0;
}

#define ECELSIUS_DEGREE (-2730)
static int qg_somc_get_real_temp(struct qpnp_qg *chip, int *val)
{
	int rc;
	int batt_temp;
	int aux_temp = ECELSIUS_DEGREE;
	int corrected_batt_temp = ECELSIUS_DEGREE;
	int corrected_aux_temp = ECELSIUS_DEGREE;

	if (chip->use_real_temp && (chip->real_temp_debug != -EINVAL)) {
		*val = chip->real_temp_debug;
		return 0;
	}

	rc = qg_get_battery_temp(chip, &batt_temp);
	if (rc < 0) {
		pr_err("failed to read batt_temp rc=%d\n", rc);
		return rc;
	}
	if (chip->use_real_temp) {
		corrected_batt_temp = batt_temp + chip->batt_temp_correctton;
	} else {
		*val = batt_temp;
		qg_dbg(chip, QG_DEBUG_STATUS,
			"Real Temp is not supported. So, batt_temp is used\n");
		return 0;
	}

	if (chip->real_temp_use_aux) {
		rc = qg_somc_get_aux_temp(chip, &aux_temp);
		if (rc) {
			pr_err("Couldn't get aux_temp rc = %d\n", rc);
		} else {
			corrected_aux_temp =
					aux_temp + chip->aux_temp_correctton;
		}
	}

	*val = max(corrected_batt_temp, corrected_aux_temp);
	qg_dbg(chip, QG_DEBUG_STATUS, "batt:%d aux:%d -> real battery temp:%d\n",
						batt_temp, aux_temp, *val);
	return 0;
}

static int qg_somc_set_real_temp_debug(struct qpnp_qg *chip, int val)
{
	int rc = 0;

	if (val < ECELSIUS_DEGREE) {
		chip->real_temp_debug = -EINVAL;
		rc = -EINVAL;
	} else {
		chip->real_temp_debug = val;
	}

	if (chip->batt_psy)
		power_supply_changed(chip->batt_psy);

	return 0;
}

static int qg_somc_get_batt_soc(struct qpnp_qg *chip, int *batt_soc)
{
	if (!chip)
		return -ENODEV;

	if (is_debug_batt_id(chip) || chip->battery_missing) {
		*batt_soc = -EINVAL;
		return 0;
	}

	*batt_soc = chip->batt_soc;

	return 0;
}
#endif

static int qg_iio_write_raw(struct iio_dev *indio_dev,
		struct iio_chan_spec const *chan, int val1,
		int val2, long mask)
{
	struct qpnp_qg *chip = iio_priv(indio_dev);
	int rc = 0;

	switch (chan->channel) {
	case PSY_IIO_CHARGE_FULL:
#if !defined(CONFIG_SOMC_CHARGER_EXTENSION)
		if (chip->dt.cl_disable) {
			pr_warn("Capacity learning disabled!\n");
			return 0;
		}
		if (chip->cl->active) {
			pr_warn("Capacity learning active!\n");
			return 0;
		}
		if (val1 <= 0 || val1 > chip->cl->nom_cap_uah) {
			pr_err("charge_full is out of bounds\n");
			return -EINVAL;
		}
		mutex_lock(&chip->cl->lock);
		rc = qg_store_learned_capacity(chip, val1);
		if (!rc)
			chip->cl->learned_cap_uah = val1;
		mutex_unlock(&chip->cl->lock);
#endif
#if defined(CONFIG_SOMC_CHARGER_EXTENSION)
		mutex_lock(&chip->cl->lock);
		chip->cl->learned_cap_uah = val1;
		cap_learning_somc_limit_learned_cap(chip->cl);
		rc = qg_store_learned_capacity(chip, chip->cl->learned_cap_uah);
		mutex_unlock(&chip->cl->lock);
		if (chip->cl->learned_cap_uah == chip->cl->nom_cap_uah)
			qg_reset(chip);
		if (chip->cl->active)
			cap_learning_abort(chip->cl);
#endif
		break;
	case PSY_IIO_SOH:
		chip->soh = val1;
		qg_dbg(chip, QG_DEBUG_STATUS, "SOH update: SOH=%d esr_actual=%d esr_nominal=%d\n",
				chip->soh, chip->esr_actual, chip->esr_nominal);
		if (chip->sp)
			soh_profile_update(chip->sp, chip->soh);
		break;
	case PSY_IIO_CLEAR_SOH:
		chip->first_profile_load = val1;
		break;
	case PSY_IIO_ESR_ACTUAL:
		chip->esr_actual = val1;
		break;
	case PSY_IIO_ESR_NOMINAL:
		chip->esr_nominal = val1;
		break;
	case PSY_IIO_FG_RESET:
		qg_reset(chip);
		break;
	case PSY_IIO_BATT_AGE_LEVEL:
#if !defined(CONFIG_SOMC_CHARGER_EXTENSION) /* RID011075 Soft charge 5.1 */
		rc = qg_setprop_batt_age_level(chip, val1);
#endif
		break;
#if defined(CONFIG_SOMC_CHARGER_EXTENSION)
	case PSY_IIO_REAL_TEMP:
		rc = qg_somc_set_real_temp_debug(chip, val1);
		if (rc < 0) {
			pr_err("Error in writing real_temp_debug, rc=%d\n", rc);
			return rc;
		}
		break;
	case PSY_IIO_BATT_AGING_LEVEL:
		qg_dbg(chip, QG_DEBUG_SOMC, "set batt_aging_level %d for SOMC\n",
                                    val1);
		rc = qg_setprop_batt_age_level(chip, val1);
		break;
#endif
	default:
		pr_debug("Unsupported QG IIO chan %d\n", chan->channel);
		rc = -EINVAL;
		break;
	}

	if (rc < 0)
		pr_err_ratelimited("Couldn't write IIO channel %d, rc = %d\n",
			chan->channel, rc);

	return rc;
}

static int qg_iio_read_raw(struct iio_dev *indio_dev,
		struct iio_chan_spec const *chan, int *val1,
		int *val2, long mask)
{
	struct qpnp_qg *chip = iio_priv(indio_dev);
	int64_t temp = 0;
	int rc = 0;

	*val1 = 0;

	switch (chan->channel) {
	case PSY_IIO_CAPACITY:
		rc = qg_get_battery_capacity(chip, val1);
		break;
	case PSY_IIO_CAPACITY_RAW:
		*val1 = chip->sys_soc;
		break;
	case PSY_IIO_REAL_CAPACITY:
		rc = qg_get_battery_capacity_real(chip, val1);
		break;
	case PSY_IIO_VOLTAGE_NOW:
		rc = qg_get_battery_voltage(chip, val1);
		break;
	case PSY_IIO_CURRENT_NOW:
		rc = qg_get_battery_current(chip, val1);
		break;
	case PSY_IIO_VOLTAGE_OCV:
		rc = qg_sdam_read(SDAM_OCV_UV, val1);
		break;
	case PSY_IIO_TEMP:
#if !defined(CONFIG_SOMC_CHARGER_EXTENSION)
		rc = qg_get_battery_temp(chip, val1);
#endif
#if defined(CONFIG_SOMC_CHARGER_EXTENSION)
		/* Get real_temp */
		if (chip->use_real_temp)
			rc = qg_somc_get_real_temp(chip, val1);
		else
			rc = qg_get_battery_temp(chip, val1);

		if (rc < 0) {
			pr_err("failed to get temp\n");
			*val1 = 250;
		}
#endif
		break;
	case PSY_IIO_RESISTANCE_ID:
		*val1 = chip->batt_id_ohm;
		break;
	case PSY_IIO_DEBUG_BATTERY:
		*val1 = is_debug_batt_id(chip);
		break;
	case PSY_IIO_RESISTANCE:
		rc = qg_sdam_read(SDAM_RBAT_MOHM, val1);
		if (!rc)
			*val1 *= 1000;
		break;
	case PSY_IIO_SOC_REPORTING_READY:
		*val1 = chip->soc_reporting_ready;
		break;
	case PSY_IIO_RESISTANCE_CAPACITIVE:
		*val1 = chip->dt.rbat_conn_mohm;
		break;
	case PSY_IIO_VOLTAGE_MIN:
		*val1 = chip->dt.vbatt_cutoff_mv * 1000;
		break;
	case PSY_IIO_VOLTAGE_MAX:
		*val1 = chip->bp.float_volt_uv;
		break;
	case PSY_IIO_BATT_FULL_CURRENT:
#if !defined(CONFIG_SOMC_CHARGER_EXTENSION)
		*val1 = chip->dt.iterm_ma * 1000;
#endif
#if defined(CONFIG_SOMC_CHARGER_EXTENSION)
		if (chip->step_ibatt_full)
			*val1 = chip->step_ibatt_full * 1000;
		else
			*val1 = chip->dt.iterm_ma * 1000;
#endif
		break;
	case PSY_IIO_BATT_PROFILE_VERSION:
		*val1 = chip->bp.qg_profile_version;
		break;
	case PSY_IIO_CHARGE_COUNTER:
		rc = qg_get_charge_counter(chip, val1);
		break;
	case PSY_IIO_CHARGE_FULL:
		if (!chip->dt.cl_disable && chip->dt.cl_feedback_on)
			rc = qg_get_learned_capacity(chip, &temp);
		else
			rc = qg_get_nominal_capacity((int *)&temp, 250, true);
		if (!rc)
			*val1 = (int)temp;
		break;
	case PSY_IIO_CHARGE_FULL_DESIGN:
#if !defined(CONFIG_SOMC_CHARGER_EXTENSION)
		rc = qg_get_nominal_capacity((int *)&temp, 250, true);
		if (!rc)
			*val1 = (int)temp;
#endif
#if defined(CONFIG_SOMC_CHARGER_EXTENSION)
		if (chip->initial_capacity >= 0) {
			*val1 = chip->initial_capacity;
		} else {
			rc = qg_get_nominal_capacity((int *)&temp, 250, true);
			if (!rc)
				*val1 = (int)temp;
		}
#endif
		break;
	case PSY_IIO_CYCLE_COUNT:
		rc = get_cycle_count(chip->counter, val1);
		break;
	case PSY_IIO_TIME_TO_FULL_AVG:
		rc = ttf_get_time_to_full(chip->ttf, val1);
		break;
	case PSY_IIO_TIME_TO_FULL_NOW:
		rc = ttf_get_time_to_full(chip->ttf, val1);
		break;
	case PSY_IIO_TIME_TO_EMPTY_AVG:
		rc = ttf_get_time_to_empty(chip->ttf, val1);
		break;
	case PSY_IIO_ESR_ACTUAL:
		*val1 = (chip->esr_actual == -EINVAL) ?  -EINVAL :
					(chip->esr_actual * 1000);
		break;
	case PSY_IIO_ESR_NOMINAL:
		*val1 = (chip->esr_nominal == -EINVAL) ?  -EINVAL :
					(chip->esr_nominal * 1000);
		break;
	case PSY_IIO_SOH:
		*val1 = chip->soh;
		break;
	case PSY_IIO_CLEAR_SOH:
		*val1 = chip->first_profile_load;
		break;
	case PSY_IIO_CC_SOC:
		rc = qg_get_cc_soc(chip, val1);
		break;
	case PSY_IIO_FG_RESET:
		*val1 = 0;
		break;
	case PSY_IIO_VOLTAGE_AVG:
		rc = qg_get_vbat_avg(chip, val1);
		break;
	case PSY_IIO_CURRENT_AVG:
		rc = qg_get_ibat_avg(chip, val1);
		break;
	case PSY_IIO_POWER_NOW:
		rc = qg_get_power(chip, val1, false);
		break;
	case PSY_IIO_POWER_AVG:
		rc = qg_get_power(chip, val1, true);
		break;
	case PSY_IIO_SCALE_MODE_EN:
		*val1 = chip->fvss_active;
		break;
	case PSY_IIO_BATT_AGE_LEVEL:
		*val1 = chip->batt_age_level;
		break;
	case PSY_IIO_FG_TYPE:
		*val1 = chip->qg_mode;
		break;
#if defined(CONFIG_SOMC_CHARGER_EXTENSION)
	case PSY_IIO_REAL_TEMP:
		rc = qg_somc_get_real_temp(chip, val1);
		break;
	case PSY_IIO_AUX_TEMP:
		rc = qg_somc_get_aux_temp(chip, val1);
		break;
	case PSY_IIO_CHARGE_FULL_RAW:
		*val1 = chip->cl->final_cap_uah;
		break;
	case PSY_IIO_LEARNING_COUNTER:
		*val1 = chip->cl->learning_counter;
		break;
	case PSY_IIO_LEARNING_TRIAL_COUNTER:
		*val1 = chip->cl->learning_trial_counter;
		break;
	case PSY_IIO_BATT_AGING_LEVEL:
		*val1 = chip->batt_age_level;
		break;
	case PSY_IIO_REAL_NOM_CAP:
		rc = qg_get_nominal_capacity((int *)&temp, 250, true);
		if (!rc)
			*val1 = (int)temp;
		break;
	case PSY_IIO_BATTERY_RAW_SOC:
		rc = qg_somc_get_batt_soc(chip, val1);
		break;
	case PSY_IIO_FULL_COUNTER:
		*val1 = chip->full_counter;
		break;
	case PSY_IIO_RECHARGE_COUNTER:
		*val1 = chip->recharge_counter;
		break;
	case PSY_IIO_STEP_PHASE:
		*val1 = chip->cur_step_phase;
		break;
	case PSY_IIO_STEP_JEITA_ZONE:
		*val1 = chip->cur_jeita_zone;
		break;
	case PSY_IIO_STEP_FCC:
		*val1 = chip->step_fcc;
		break;
	case PSY_IIO_STEP_FV:
		*val1 = chip->step_fv;
		break;
#endif
	default:
		pr_debug("Unsupported QG IIO chan %d\n", chan->channel);
		rc = -EINVAL;
		break;
	}

	if (rc < 0) {
		pr_err_ratelimited("Couldn't read IIO channel %d, rc = %d\n",
			chan->channel, rc);
		return rc;
	}

	return IIO_VAL_INT;
}

static int qg_iio_of_xlate(struct iio_dev *indio_dev,
				const struct of_phandle_args *iiospec)
{
	struct qpnp_qg *chip = iio_priv(indio_dev);
	struct iio_chan_spec *iio_chan = chip->iio_chan;
	int i;

	for (i = 0; i < ARRAY_SIZE(qg_iio_psy_channels);
					i++, iio_chan++)
		if (iio_chan->channel == iiospec->args[0])
			return i;

	return -EINVAL;
}

static const struct iio_info qg_iio_info = {
	.read_raw	= qg_iio_read_raw,
	.write_raw	= qg_iio_write_raw,
	.of_xlate	= qg_iio_of_xlate,
};

#define DEFAULT_CL_BEGIN_IBAT_UA	(-100000)
static bool qg_cl_ok_to_begin(void *data)
{
	struct qpnp_qg *chip = data;

	if (chip->last_fifo_i_ua < DEFAULT_CL_BEGIN_IBAT_UA)
		return true;

	return false;
}

#define DEFAULT_RECHARGE_SOC 95
static int qg_charge_full_update(struct qpnp_qg *chip)
{
	union power_supply_propval prop = {0, };
	int rc, recharge_soc, health, val;

	if (!chip->dt.hold_soc_while_full)
		goto out;

	rc = power_supply_get_property(chip->batt_psy,
			POWER_SUPPLY_PROP_HEALTH, &prop);
	if (rc < 0) {
		pr_err("Failed to get battery health, rc=%d\n", rc);
		goto out;
	}
	health = prop.intval;

	rc = qg_read_iio_chan(chip, RECHARGE_SOC, &val);
	if (rc < 0 || val < 0) {
		pr_debug("Failed to get recharge-soc\n");
		recharge_soc = DEFAULT_RECHARGE_SOC;
	} else {
		recharge_soc = val;
	}
	chip->recharge_soc = recharge_soc;

	qg_dbg(chip, QG_DEBUG_STATUS, "msoc=%d health=%d charge_full=%d charge_done=%d\n",
				chip->msoc, health, chip->charge_full,
				chip->charge_done);
	if (chip->charge_done && !chip->charge_full) {
#if !defined(CONFIG_SOMC_CHARGER_EXTENSION)
		if (chip->msoc >= 99 && health == POWER_SUPPLY_HEALTH_GOOD) {
#endif
#if defined(CONFIG_SOMC_CHARGER_EXTENSION)
		if (chip->msoc > 99 && health == POWER_SUPPLY_HEALTH_GOOD) {
#endif
			chip->charge_full = true;
#if defined(CONFIG_SOMC_CHARGER_EXTENSION)
			chip->full_counter++;
#endif
			qg_dbg(chip, QG_DEBUG_STATUS, "Setting charge_full (0->1) @ msoc=%d\n",
					chip->msoc);
		} else if (health != POWER_SUPPLY_HEALTH_GOOD) {
			/* terminated in JEITA */
			qg_dbg(chip, QG_DEBUG_STATUS, "Terminated charging @ msoc=%d\n",
					chip->msoc);
		}
	} else if ((!chip->charge_done || chip->msoc <= recharge_soc)
				&& chip->charge_full) {

		bool input_present = is_input_present(chip);

		/*
		 * force a recharge only if SOC <= recharge SOC and
		 * we have not started charging.
		 */
		if ((chip->wa_flags & QG_RECHARGE_SOC_WA) &&
			input_present && chip->msoc <= recharge_soc &&
			chip->charge_status != POWER_SUPPLY_STATUS_CHARGING) {
			/* Force recharge */
			rc = qg_write_iio_chan(chip, FORCE_RECHARGE, 0);
			if (rc < 0)
				pr_err("Failed to force recharge rc=%d\n", rc);
			else
				qg_dbg(chip, QG_DEBUG_STATUS, "Forced recharge\n");
		}


		if (chip->charge_done)
			return 0;	/* wait for recharge */

		/*
		 * If SOC has indeed dropped below recharge-SOC or
		 * the input is removed, if linearize-soc is set scale
		 * msoc from 100% for better UX.
		 */
		if (chip->msoc < recharge_soc || !input_present) {
			if (chip->dt.linearize_soc) {
				get_rtc_time(&chip->last_maint_soc_update_time);
				chip->maint_soc = FULL_SOC;
				qg_scale_soc(chip, false);
			}
			chip->charge_full = false;
#if defined(CONFIG_SOMC_CHARGER_EXTENSION)
			chip->recharge_counter++;
#endif
			qg_dbg(chip, QG_DEBUG_STATUS, "msoc=%d recharge_soc=%d charge_full (1->0)\n",
					chip->msoc, recharge_soc);
		} else {
			/* continue with charge_full state */
			qg_dbg(chip, QG_DEBUG_STATUS, "msoc=%d recharge_soc=%d charge_full=%d input_present=%d\n",
					chip->msoc, recharge_soc,
					chip->charge_full, input_present);
		}
	}
out:
	return 0;
}

static int qg_parallel_status_update(struct qpnp_qg *chip)
{
	int rc;
	bool parallel_enabled = is_parallel_enabled(chip);
	bool update_smb = false;

	if (parallel_enabled == chip->parallel_enabled)
		return 0;

	chip->parallel_enabled = parallel_enabled;
	qg_dbg(chip, QG_DEBUG_STATUS,
		"Parallel status changed Enabled=%d\n", parallel_enabled);

	mutex_lock(&chip->data_lock);
	/*
	 * dt.qg_ext_sense = Uses external rsense, if defined do not
	 *		     enable SMB sensing (for non-CP parallel charger).
	 * dt.cp_iin_sns = Uses CP IIN_SNS, enable SMB sensing (for CP charger).
	 */
	if (is_cp_available(chip))
		update_smb = chip->dt.use_cp_iin_sns ? true : false;
	else if (is_parallel_available(chip))
		update_smb = chip->dt.qg_ext_sense ? false : true;

	rc = process_rt_fifo_data(chip, update_smb);
	if (rc < 0)
		pr_err("Failed to process RT FIFO data, rc=%d\n", rc);

	mutex_unlock(&chip->data_lock);

	return 0;
}

static int qg_input_status_update(struct qpnp_qg *chip)
{
	bool usb_present = is_usb_present(chip);
	bool dc_present = is_dc_present(chip);

	if ((chip->usb_present != usb_present) ||
		(chip->dc_present != dc_present)) {
		qg_dbg(chip, QG_DEBUG_STATUS,
			"Input status changed usb_present=%d dc_present=%d\n",
						usb_present, dc_present);
		qg_scale_soc(chip, false);
	}

	chip->usb_present = usb_present;
	chip->dc_present = dc_present;

	return 0;
}

static int qg_handle_battery_removal(struct qpnp_qg *chip)
{
	int rc, length = QG_SDAM_MAX_OFFSET - QG_SDAM_VALID_OFFSET;
	u8 *data;

	/* clear SDAM */
	data = kcalloc(length, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	rc = qg_sdam_multibyte_write(QG_SDAM_VALID_OFFSET, data, length);
	if (rc < 0)
		pr_err("Failed to clear SDAM rc=%d\n", rc);

	return rc;
}

static int qg_handle_battery_insertion(struct qpnp_qg *chip)
{
	int rc, count = 0;
	u32 ocv_uv = 0, ocv_raw = 0;
	u8 reg = 0;

	do {
		rc = qg_read(chip, chip->qg_base + QG_STATUS1_REG, &reg, 1);
		if (rc < 0) {
			pr_err("Failed to read STATUS1_REG rc=%d\n", rc);
			return rc;
		}

		if (reg & QG_OK_BIT)
			break;

		msleep(200);
		count++;
	} while (count < MAX_QG_OK_RETRIES);

	if (count == MAX_QG_OK_RETRIES) {
		qg_dbg(chip, QG_DEBUG_STATUS, "QG_OK not set!\n");
		return 0;
	}

	/* read S7 PON OCV */
	rc = qg_read_ocv(chip, &ocv_uv, &ocv_raw, S7_PON_OCV);
	if (rc < 0) {
		pr_err("Failed to read PON OCV rc=%d\n", rc);
		return rc;
	}

	qg_dbg(chip, QG_DEBUG_STATUS,
		"S7_OCV on battery insertion = %duV\n", ocv_uv);

	chip->kdata.param[QG_GOOD_OCV_UV].data = ocv_uv;
	chip->kdata.param[QG_GOOD_OCV_UV].valid = true;
	/* clear all the userspace data */
	chip->kdata.param[QG_CLEAR_LEARNT_DATA].data = 1;
	chip->kdata.param[QG_CLEAR_LEARNT_DATA].valid = true;

	vote(chip->awake_votable, GOOD_OCV_VOTER, true, 0);
	/* signal the read thread */
	chip->data_ready = true;
	wake_up_interruptible(&chip->qg_wait_q);

	return 0;
}

static int qg_battery_status_update(struct qpnp_qg *chip)
{
	int rc;
	union power_supply_propval prop = {0, };

	if (!is_batt_available(chip))
		return 0;

	mutex_lock(&chip->data_lock);

	rc = power_supply_get_property(chip->batt_psy,
			POWER_SUPPLY_PROP_PRESENT, &prop);
	if (rc < 0) {
		pr_err("Failed to get battery-present, rc=%d\n", rc);
		goto done;
	}

	if (chip->battery_missing && prop.intval) {
		pr_warn("Battery inserted!\n");
		rc = qg_handle_battery_insertion(chip);
		if (rc < 0)
			pr_err("Failed in battery-insertion rc=%d\n", rc);
	} else if (!chip->battery_missing && !prop.intval) {
		pr_warn("Battery removed!\n");
		rc = qg_handle_battery_removal(chip);
		if (rc < 0)
			pr_err("Failed in battery-removal rc=%d\n", rc);
	}

	chip->battery_missing = !prop.intval;

done:
	mutex_unlock(&chip->data_lock);
	return rc;
}

static void qg_sleep_exit_work(struct work_struct *work)
{
	int rc;
	struct qpnp_qg *chip = container_of(work,
			struct qpnp_qg, qg_sleep_exit_work.work);

	vote(chip->awake_votable, SLEEP_EXIT_VOTER, true, 0);

	mutex_lock(&chip->data_lock);
	/*
	 * if this work is executing, the system has been active
	 * for a while. So, force back the S2 active configuration
	 */
	qg_dbg(chip, QG_DEBUG_STATUS, "sleep_exit_work: exit S2_SLEEP\n");
	rc = qg_config_s2_state(chip, S2_SLEEP, false, true);
	if (rc < 0)
		pr_err("Failed to exit S2_SLEEP rc=%d\n", rc);

	vote(chip->awake_votable, SLEEP_EXIT_DATA_VOTER, true, 0);
	/* signal the read thread */
	chip->data_ready = true;
	wake_up_interruptible(&chip->qg_wait_q);

	mutex_unlock(&chip->data_lock);

	vote(chip->awake_votable, SLEEP_EXIT_VOTER, false, 0);
}

#if defined(CONFIG_SOMC_CHARGER_EXTENSION)
static void qg_somc_jeita_step_wakelock(struct qpnp_qg *chip, bool en)
{
	if (en)
		__pm_stay_awake(chip->step_ws);
	else
		__pm_relax(chip->step_ws);

	qg_dbg(chip, QG_DEBUG_SOMC_STEP,
			"wake lock for JEITA/Step: %d\n", (int)en);
	chip->step_lock_en = en;
}

static void qg_somc_jeita_step_update(struct qpnp_qg *chip)
{
	bool input_present;

	if (!chip->step_en)
		return;

	input_present = is_input_present(chip);

	qg_dbg(chip, QG_DEBUG_SOMC_STEP, "step_lock_en=%d input_present=%d bsoc=%d\n",
			chip->step_lock_en, input_present, chip->batt_soc);

	if (!chip->step_lock_en && input_present) {
		mutex_lock(&chip->step_lock);
		qg_somc_jeita_step_wakelock(chip, true);
		qg_dbg(chip, QG_DEBUG_SOMC, "schedule JEITA/Step worker\n");
		schedule_delayed_work(&chip->somc_jeita_step_charge_work,
							msecs_to_jiffies(0));
		mutex_unlock(&chip->step_lock);
	}
}

#define CELL_IMPEDANCE_MOHM 30
static void qg_somc_get_current_step_value(struct qpnp_qg *chip,
					struct qg_dt_step_data *step_data,
					int *fv, int *fcc,
					int *health, int *iterm)
{
	chip->cur_jeita_zone = step_data->jeita_zone;
	chip->cur_step_phase = step_data->step_phase;
	chip->cur_check_current = step_data->check_current;
	chip->init_check_current = step_data->check_current;
	chip->cur_target_current = step_data->target_current;
	*fv = step_data->fv;
	*fcc = step_data->fcc;
	*health = step_data->health;
	*iterm = step_data->term_current;
}

static void qg_somc_jeita_step_charge_work(struct work_struct *work)
{
	struct qpnp_qg *chip = container_of(work, struct qpnp_qg,
					somc_jeita_step_charge_work.work);
	int rc, val;
	int lv = 0;
	s64 currentTime;
	int iterm_final = 0;
	int real_temp = 0, vbatt_mv = 0, ibat_ma = 0;
	bool batt_aging_level_changed = false;
	bool jeita_zone_charged = false;
	bool step_val_obtained = false;
	int charging_status;
	int msoc = 0, bsoc = 0;
	int current_now = 0, vbatt_uv = 0;
	int step_fv = 0, step_fcc = 0, step_health = 0, step_iterm = 0;
	int max_fv = 0;
	int cv_status;
	int i = 0, sel = -1;
	bool input_present;
	bool need_psy_changed = false;
	struct qg_dt_step_data step_data[STEP_DATA_MAX_CFG_NUM];

	static const char * const str_status[] = {"-", "Charging",
				"Discharging", "NotCharging", "FULL", "-"};

	/* read input present status */
	input_present = is_input_present(chip);

	/* Get battery charging status */
	charging_status = chip->charge_status;
	rc = qg_read_iio_chan(chip, CV_STATUS, &val);
	if (rc < 0) {
		pr_err("Failed to get cv_status, rc=%d\n", rc);
		cv_status = 0;
	} else {
		cv_status = val;
	}

	/* Get real_temp */
	if (chip->use_real_temp)
		rc = qg_somc_get_real_temp(chip, &real_temp);
	else
		rc = qg_get_battery_temp(chip, &real_temp);

	if (rc < 0) {
		pr_err("failed to get temp\n");
		real_temp = 250;
	}

	/* Get voltage */
	rc = qg_get_battery_voltage(chip, &vbatt_uv);
	if (rc < 0) {
		pr_err("failed to get battery voltage, rc=%d\n", rc);
		vbatt_uv = 0;
	}
	vbatt_mv = vbatt_uv / 1000;

	/* Get Ibat (Reverse sign, Positive value means charging) */
	rc = qg_get_battery_current(chip, &current_now);
	if (rc < 0) {
		pr_err("failed to get battery current, rc=%d\n", rc);
		current_now = 0;
	}
	ibat_ma = (-1) * current_now / 1000;

	/* Get Aging Level, Select step_data, Set stbl */
	lv = chip->batt_age_level;
	if (lv < 0 || lv >= AGING_LV_NUM)
		lv = 0;
	for (i = 0; i < STEP_DATA_MAX_CFG_NUM; i++)
		step_data[i] = chip->step_data[lv][i];

	/* Get socs */
	msoc = chip->msoc;
	bsoc = chip->batt_soc;

	/* calc avarage value of temp, current and voltage */
	for (i = STEP_INPUT_BUF_NUM - 1; i > 0; i--)
		chip->step_input_data[i] = chip->step_input_data[i - 1];
	chip->step_input_data[0].temp = real_temp;
	chip->step_input_data[0].current_now = current_now;
	chip->step_input_data[0].voltage_now = vbatt_uv;
	chip->step_input_data[0].stored_ktime_ms =
					ktime_to_ms(ktime_get_boottime());
	currentTime = chip->step_input_data[0].stored_ktime_ms;

	qg_dbg(chip, QG_DEBUG_SOMC_STEP, "%s, cv=%d, batt_aging_level=%d, bsoc=%d, msoc=%d, real_temp=%d, vbatt_mv=%d, ibat_ma=%d",
				str_status[charging_status], cv_status, lv,
				bsoc, msoc, real_temp, vbatt_mv, ibat_ma);

	if (chip->cvstep_jeita_is_running) {
		qg_dbg(chip, QG_DEBUG_SOMC_STEP, "somc_step_jeita_is_running=%d, cur_jeita_zone=%d, cur_step_phase=%d",
				chip->cvstep_jeita_is_running,
				chip->cur_jeita_zone, chip->cur_step_phase);

		/* Check jeita_zone change */
		for (i = 0; i < STEP_DATA_MAX_CFG_NUM; i++) {
			if (real_temp >= step_data[i].temp_range_l &&
			    real_temp < step_data[i].temp_range_h) {
				if (chip->cur_jeita_zone !=
					step_data[i].jeita_zone) {
					jeita_zone_charged = true;
					qg_dbg(chip, QG_DEBUG_SOMC, "jeita_zone_charged");
				}
				break;
			}
		}

		if (batt_aging_level_changed) {
			/* During charging - 1 : Update config when aging_lv is changed */
			for (i = 0; i < STEP_DATA_MAX_CFG_NUM; i++) {
				if (real_temp >= step_data[i].temp_range_l &&
				    real_temp < step_data[i].temp_range_h) {
					sel = i;
					if (vbatt_mv < step_data[i].fv)
						break;
				}
			}
			if (sel >= 0) {
				qg_somc_get_current_step_value(chip,
							&step_data[sel],
							&step_fv,
							&step_fcc,
							&step_health,
							&step_iterm);
				step_val_obtained = true;
				qg_dbg(chip, QG_DEBUG_SOMC, "AgingLv is changed during charging !! jeita_zone=%d(%d-%d), health=%d, step_phase=%d, fcc=%d, fv=%d",
					chip->cur_jeita_zone,
					step_data[sel].temp_range_l,
					step_data[sel].temp_range_h,
					step_health,
					chip->cur_step_phase,
					step_fcc, step_fv);
			}
		} else if (jeita_zone_charged) {
			/* During charging - 2 : Update config when jeita_zone is changed */
			for (i = 0; i < STEP_DATA_MAX_CFG_NUM; i++) {
				if (real_temp >= step_data[i].temp_range_l &&
				    real_temp < step_data[i].temp_range_h) {
					sel = i;
					if (vbatt_mv < step_data[i].fv)
						break;
				}
			}
			if (sel >= 0) {
				qg_somc_get_current_step_value(chip,
							&step_data[sel],
							&step_fv,
							&step_fcc,
							&step_health,
							&step_iterm);
				step_val_obtained = true;
				qg_dbg(chip, QG_DEBUG_SOMC, "JEITA_ZONE Changed!! jeita_zone=%d(%d-%d), health=%d, step_phase=%d, fcc=%d, fv=%d",
					chip->cur_jeita_zone,
					step_data[sel].temp_range_l,
					step_data[sel].temp_range_h,
					step_health,
					chip->cur_step_phase,
					step_fcc, step_fv);
			}
		} else if (chip->step_phase_is_changed) {
			/* During charging - 3 : Proceed next step */
			for (i = 0; i < STEP_DATA_MAX_CFG_NUM; i++) {
				if (real_temp >= step_data[i].temp_range_l &&
				    real_temp < step_data[i].temp_range_h &&
					chip->cur_step_phase ==
						step_data[i].step_phase) {

					qg_somc_get_current_step_value(chip,
							&step_data[i],
							&step_fv,
							&step_fcc,
							&step_health,
							&step_iterm);
					step_val_obtained = true;
					qg_dbg(chip, QG_DEBUG_SOMC, "Step Phase proceeded!! jeita_zone=%d(%d-%d), health=%d, step_phase=%d, fcc=%d, fv=%d",
						chip->cur_jeita_zone,
						step_data[i].temp_range_l,
						step_data[i].temp_range_h,
						step_health,
						chip->cur_step_phase,
						step_fcc, step_fv);
					break;
				}
			}
		}
	} else {
		for (i = 0; i < STEP_DATA_MAX_CFG_NUM; i++) {
			if (real_temp >= step_data[i].temp_range_l &&
			    real_temp < step_data[i].temp_range_h) {
				sel = i;
				if (vbatt_mv < step_data[i].fv)
					break;
			}
		}
		if (sel >= 0) {
			qg_somc_get_current_step_value(chip,
							&step_data[sel],
							&step_fv,
							&step_fcc,
							&step_health,
							&step_iterm);
			step_val_obtained = true;
			qg_dbg(chip, QG_DEBUG_SOMC, "Jeita/Step phase determined!! jeita_zone=%d(%d-%d), health=%d, step_phase=%d, fcc=%d, fv=%d",
				chip->cur_jeita_zone,
				step_data[sel].temp_range_l,
				step_data[sel].temp_range_h,
				step_health,
				chip->cur_step_phase,
				step_fcc, step_fv);
		}
	}
	if (batt_aging_level_changed || jeita_zone_charged ||
					chip->step_phase_is_changed)
		need_psy_changed = true;

	qg_dbg(chip, QG_DEBUG_SOMC_STEP, "batt_aging_level_changed=%d, jeita_zone_charged=%d, chip->step_phase_is_changed=%d, need_psy_changed=%d",
				batt_aging_level_changed, jeita_zone_charged,
				chip->step_phase_is_changed, need_psy_changed);

	/* request vote for FV and FCC and health condition to smb driver */
	if (step_val_obtained && step_fv != chip->step_fv) {
		qg_dbg(chip, QG_DEBUG_SOMC_STEP, "Set FV=%d from %d",
			step_fv, chip->step_fv);
		rc = qg_write_iio_chan(chip, JEITA_STEP_FV,
						(step_fv * 1000));
		if (rc < 0)
			pr_err("Error in step jeita fv set, rc=%d\n", rc);

		chip->step_fv = step_fv;
	}

	if (step_val_obtained && step_fcc != chip->step_fcc) {
		qg_dbg(chip, QG_DEBUG_SOMC_STEP, "Set FCC=%d from %d",
			step_fcc, chip->step_fcc);
		rc = qg_write_iio_chan(chip, JEITA_STEP_FCC,
							(step_fcc * 1000));
		if (rc < 0)
			pr_err("Error in step current set, rc=%d\n", rc);

		chip->step_fcc = step_fcc;
	}

	if (step_val_obtained && step_health != chip->step_health) {
		qg_dbg(chip, QG_DEBUG_SOMC_STEP, "Set JEITA_CONDITION=%d from %d",
			step_health, chip->step_health);
		rc = qg_write_iio_chan(chip, JEITA_CONDITION, step_health);
		if (rc < 0)
			pr_err("Error in step jeita cond set, rc=%d\n", rc);

		chip->step_health = step_health;
	}

	/* Check charging status and taper current to determine next phase */
	if (charging_status == POWER_SUPPLY_STATUS_CHARGING && !cv_status) {
		chip->cur_check_current = chip->init_check_current;
		chip->cvstep_jeita_is_running = 1;
		chip->step_phase_is_changed = false;
	} else if (charging_status == POWER_SUPPLY_STATUS_CHARGING &&
								cv_status) {
		qg_dbg(chip, QG_DEBUG_SOMC_STEP, "POWER_SUPPLY_CHARGE_TYPE_TAPER::: ibat_ma=%d, cur_check_current=%d, cur_target_current=%d",
			ibat_ma, chip->cur_check_current,
			chip->cur_target_current);
		if (!chip->cvstep_jeita_is_running) {
			if (ibat_ma < chip->cur_target_current) {
				chip->cur_step_phase++;
				chip->step_phase_is_changed = true;
				qg_dbg(chip, QG_DEBUG_SOMC_STEP, "Step charge started with TAPER but Step Phase should proceeded soon. cur_step_phase->%d, ibat_ma=%d",
					chip->cur_step_phase, ibat_ma);
			}
		} else if (ibat_ma < chip->cur_check_current) {
			if (chip->cur_check_current <
						chip->cur_target_current) {
				chip->cur_step_phase++;
				chip->step_phase_is_changed = true;
				qg_dbg(chip, QG_DEBUG_SOMC_STEP, "cur_step_phase->%d, ibat_ma=%d, cur_check_current=%d",
					chip->cur_step_phase,
					ibat_ma, chip->cur_check_current);
			} else {
				chip->cur_check_current -= 500;
				if (chip->cur_check_current <
						chip->cur_target_current) {
					chip->cur_check_current =
						chip->cur_target_current - 10;
				}
				chip->step_phase_is_changed = false;
				qg_dbg(chip, QG_DEBUG_SOMC_STEP, "cur_check_current->%d",
						chip->cur_check_current);
			}
		}
		chip->cvstep_jeita_is_running = 1;
	} else if (charging_status == POWER_SUPPLY_STATUS_FULL) {
		if (chip->cur_target_current == 0) {
			chip->cvstep_jeita_is_running = 0;
			chip->cur_step_phase = 0;
			chip->step_phase_is_changed = false;
			if (chip->cvstep_jeita_is_running)
				qg_dbg(chip, QG_DEBUG_SOMC, "Step Charge Terminated!! (Full)");
		} else {
			chip->cur_step_phase++;
			chip->step_phase_is_changed = true;
			chip->cvstep_jeita_is_running = 1;
			qg_dbg(chip, QG_DEBUG_SOMC, "Terminated during Step Charging!!");
		}
	} else { /*PAUSE, CHARGING_DISABLED, INVALID*/
		chip->cvstep_jeita_is_running = 0;
		chip->step_phase_is_changed = false;
		if (chip->cvstep_jeita_is_running)
			qg_dbg(chip, QG_DEBUG_SOMC, "Step Charge Terminated!! (Pause/Disabled/Invalid)");
	}

	/* Configure digital Iterm */
	if (step_val_obtained && step_iterm != chip->step_iterm) {
		qg_dbg(chip, QG_DEBUG_SOMC_STEP, "Set JEITA_STEP_ITERM=%d from %d",
			step_iterm, chip->step_iterm);
		rc = qg_write_iio_chan(chip, JEITA_STEP_ITERM, step_iterm);
		if (rc < 0)
			pr_err("Error in iterm set on batt_psy, rc=%d\n", rc);

		chip->step_iterm = step_iterm;
	}

	/* Configure Ibat full. hvdcp_opti reads this param via psy get */
	for (i = 0; i < STEP_DATA_MAX_CFG_NUM; i++) {
		if (step_data[i].jeita_zone == chip->cur_jeita_zone &&
					step_data[i].term_current > 0) {
			iterm_final = step_data[i].term_current;
			break;
		}
	}
	chip->step_ibatt_full = iterm_final + chip->ibat_full_term_diff;
	chip->dt.iterm_ma = chip->step_ibatt_full; /* Overwrite iterm_ma for qg-soc algorithm*/

	/* Configure Vbat full after search max FV on current aging_lv */
	for (i = 0; i < STEP_DATA_MAX_CFG_NUM; i++) {
		if (step_data[i].fv > max_fv)
			max_fv = step_data[i].fv;
	}
	if (max_fv > 0 && max_fv <= chip->product_max_fv)
		chip->bp.float_volt_uv = max_fv * 1000;

	/* Configure FCSS. If iterm = zero, then slow down FCSS */
	/* TBD */

	/*
	 * Enable ECC to reset Battery OV RT status
	 * when Vbatt falls below threshold with final step only
	 */
	/* TBD */

	/* Log Full/OV related params*/
	qg_dbg(chip, QG_DEBUG_SOMC_STEP, "iterm=%d, ibat_full=%d, vbat_full=%d",
		chip->step_iterm , chip->step_ibatt_full, max_fv);

	if (chip->qg_psy && need_psy_changed)
		power_supply_changed(chip->qg_psy);

	/* re-schedule work */
	if (input_present &&
		(chip->charge_status != POWER_SUPPLY_STATUS_FULL)) {
		schedule_delayed_work(&chip->somc_jeita_step_charge_work,
							msecs_to_jiffies(2000));
	} else {
		qg_dbg(chip, QG_DEBUG_SOMC,
			"exit step_work input_present:%d status:%d\n",
					input_present, chip->charge_status);
		memset(chip->step_input_data, 0, sizeof(chip->step_input_data));
		qg_somc_jeita_step_wakelock(chip, false);
	}
	mutex_unlock(&chip->step_lock);
}

static enum alarmtimer_restart qg_somc_psy_chg_alarm_timer(struct alarm *alarm,
								ktime_t time)
{
	struct qpnp_qg *chip = container_of(alarm, struct qpnp_qg,
						psy_chg_alarm_timer);

	chip->psy_chg_awake = true;
	pm_stay_awake(chip->dev);

	qg_dbg(chip, QG_DEBUG_SOMC, "PSY CHANGE timer expired\n");
	schedule_work(&chip->psy_chg_work);

	return ALARMTIMER_NORESTART;
}

#define PSY_CHG_ALARM_ONLINE_HOT_TIME_MS	(10 * 60 * 1000)
#define PSY_CHG_ALARM_ONLINE_OTHER_TIME_MS	(60 * 60 * 1000)
#define PSY_CHG_ALARM_OFFLINE_HOT_TIME_MS	(30 * 60 * 1000)
#define PSY_CHG_ALARM_OFFLINE_OTHER_TIME_MS	(24 * 60 * 60 * 1000)
#define PSY_CHG_ALARM_FIRST_TIME_MS		(10 * 60 * 1000)
#define PSY_CHG_HOT_THRESHOLD			(400)
#define PSY_CHG_WAKE_HOLD_TIME_MS		(20)
static void qg_somc_psy_chg_work(struct work_struct *work)
{
	struct qpnp_qg *chip = container_of(work, struct qpnp_qg, psy_chg_work);
	int rc;
	bool input_present;
	int temp, next_time = 0;

	if (chip->psy_chg_awake)
		vote(chip->awake_votable, PSY_CHG_VOTER, true, 0);

	/* read real temp */
	if (chip->use_real_temp)
		rc = qg_somc_get_real_temp(chip, &temp);
	else
		rc = qg_get_battery_temp(chip, &temp);

	if (rc < 0) {
		pr_err("failed to get temp\n");
		temp = 250;
	}

	/* read input present status */
	input_present = is_input_present(chip);

	if (input_present) {
		if (temp > PSY_CHG_HOT_THRESHOLD)
			next_time = PSY_CHG_ALARM_ONLINE_HOT_TIME_MS;
		else
			next_time = PSY_CHG_ALARM_ONLINE_OTHER_TIME_MS;
	} else {
		if (temp > PSY_CHG_HOT_THRESHOLD)
			next_time = PSY_CHG_ALARM_OFFLINE_HOT_TIME_MS;
		else
			next_time = PSY_CHG_ALARM_OFFLINE_OTHER_TIME_MS;
	}

	if (chip->batt_psy) {
		chip->psy_chg_counter++;
		power_supply_changed(chip->batt_psy);
		msleep(PSY_CHG_WAKE_HOLD_TIME_MS);
	}

	qg_dbg(chip, QG_DEBUG_SOMC, "Sent power_supply_changed. input_present:%d, temp=%d, next_time:%d\n",
					input_present, temp, next_time);
	if (next_time)
		alarm_start_relative(&chip->psy_chg_alarm_timer,
						ms_to_ktime(next_time));

	chip->psy_chg_awake = false;
	vote(chip->awake_votable, PSY_CHG_VOTER, false, 0);
}
#endif

static void qg_status_change_work(struct work_struct *work)
{
	struct qpnp_qg *chip = container_of(work,
			struct qpnp_qg, qg_status_change_work);
	union power_supply_propval prop = {0, };
	int rc = 0, batt_temp = 0, val;
	bool input_present = false;

	if (!is_batt_available(chip)) {
		pr_debug("batt-psy not available\n");
		goto out;
	}

	rc = qg_battery_status_update(chip);
	if (rc < 0)
		pr_err("Failed to process battery status update rc=%d\n", rc);

	rc = power_supply_get_property(chip->batt_psy,
			POWER_SUPPLY_PROP_CHARGE_TYPE, &prop);
	if (rc < 0)
		pr_err("Failed to get charge-type, rc=%d\n", rc);
	else
		chip->charge_type = prop.intval;

	rc = power_supply_get_property(chip->batt_psy,
			POWER_SUPPLY_PROP_STATUS, &prop);
	if (rc < 0)
		pr_err("Failed to get charger status, rc=%d\n", rc);
	else
		chip->charge_status = prop.intval;

	rc = qg_read_iio_chan(chip, CHARGE_DONE, &val);
	if (rc < 0)
		pr_err("Failed to get charge done status, rc=%d\n", rc);
	else
		chip->charge_done = val;

	qg_dbg(chip, QG_DEBUG_STATUS, "charge_status=%d charge_done=%d\n",
			chip->charge_status, chip->charge_done);

	rc = qg_parallel_status_update(chip);
	if (rc < 0)
		pr_err("Failed to update parallel-status, rc=%d\n", rc);

	rc = qg_input_status_update(chip);
	if (rc < 0)
		pr_err("Failed to update input status, rc=%d\n", rc);

	/* get input status */
	input_present = is_input_present(chip);

	cycle_count_update(chip->counter,
			DIV_ROUND_CLOSEST(chip->msoc * 255, 100),
			chip->charge_status, chip->charge_done,
			input_present);

	if (!chip->dt.cl_disable) {
		rc = qg_get_battery_temp(chip, &batt_temp);
		if (rc < 0) {
			pr_err("Failed to read BATT_TEMP at PON rc=%d\n", rc);
		} else if (chip->batt_soc >= 0) {
			cap_learning_update(chip->cl, batt_temp, chip->batt_soc,
				chip->charge_status, chip->charge_done,
				input_present, false);
		}
	}
	rc = qg_charge_full_update(chip);
	if (rc < 0)
		pr_err("Failed in charge_full_update, rc=%d\n", rc);

	ttf_update(chip->ttf, input_present);
#if defined(CONFIG_SOMC_CHARGER_EXTENSION)
	qg_somc_jeita_step_update(chip);
	chip->prev_charge_status = chip->charge_status;
#endif
out:
	pm_relax(chip->dev);
}

static int qg_notifier_cb(struct notifier_block *nb,
			unsigned long event, void *data)
{
	struct power_supply *psy = data;
	struct qpnp_qg *chip = container_of(nb, struct qpnp_qg, nb);

	if (event != PSY_EVENT_PROP_CHANGED)
		return NOTIFY_OK;

	if (work_pending(&chip->qg_status_change_work))
		return NOTIFY_OK;

	if ((strcmp(psy->desc->name, "battery") == 0)
		|| (strcmp(psy->desc->name, "parallel") == 0)
		|| (strcmp(psy->desc->name, "usb") == 0)
		|| (strcmp(psy->desc->name, "dc") == 0)
		|| (strcmp(psy->desc->name, "charge_pump_master") == 0)) {
		/*
		 * We cannot vote for awake votable here as that takes
		 * a mutex lock and this is executed in an atomic context.
		 */
		pm_stay_awake(chip->dev);
		schedule_work(&chip->qg_status_change_work);
	}

	return NOTIFY_OK;
}

static int qg_psy_get_property(struct power_supply *psy,
			       enum power_supply_property psp,
			       union power_supply_propval *pval)
{
	if (psp == POWER_SUPPLY_PROP_TYPE)
		pval->intval = POWER_SUPPLY_TYPE_MAINS;

	return 0;
}

static enum power_supply_property qg_psy_props[] = {
	POWER_SUPPLY_PROP_TYPE,
};

static const struct power_supply_desc qg_psy_desc = {
	.name = "bms",
	.type = POWER_SUPPLY_TYPE_MAINS,
	.properties = qg_psy_props,
	.num_properties = ARRAY_SIZE(qg_psy_props),
	.get_property = qg_psy_get_property,
};

static int qg_init_psy(struct qpnp_qg *chip)
{
	struct power_supply_config qg_psy_cfg = {};
	int rc;

	qg_psy_cfg.drv_data = chip;
	chip->qg_psy = devm_power_supply_register(chip->dev,
				&qg_psy_desc, &qg_psy_cfg);
	if (IS_ERR_OR_NULL(chip->qg_psy)) {
		pr_err("Failed to register qg_psy, rc = %d\n",
				PTR_ERR(chip->qg_psy));
		return -ENODEV;
	}

	chip->nb.notifier_call = qg_notifier_cb;
	rc = power_supply_reg_notifier(&chip->nb);
	if (rc < 0)
		pr_err("Failed to register psy notifier rc = %d\n", rc);

	return rc;
}

static int qg_init_iio_psy(struct qpnp_qg *chip,
				struct platform_device *pdev)
{
	struct iio_dev *indio_dev = chip->indio_dev;
	struct iio_chan_spec *chan;
	int qg_num_iio_channels = ARRAY_SIZE(qg_iio_psy_channels);
	int rc, i;

	chip->iio_chan = devm_kcalloc(chip->dev, qg_num_iio_channels,
				sizeof(*chip->iio_chan), GFP_KERNEL);
	if (!chip->iio_chan)
		return -ENOMEM;

	chip->int_iio_chans = devm_kcalloc(chip->dev,
				qg_num_iio_channels,
				sizeof(*chip->int_iio_chans),
				GFP_KERNEL);
	if (!chip->int_iio_chans)
		return -ENOMEM;

	chip->ext_iio_chans = devm_kcalloc(chip->dev,
				ARRAY_SIZE(qg_ext_iio_chan_name),
				sizeof(*chip->ext_iio_chans),
				GFP_KERNEL);
	if (!chip->ext_iio_chans)
		return -ENOMEM;

	indio_dev->info = &qg_iio_info;
	indio_dev->dev.parent = chip->dev;
	indio_dev->dev.of_node = chip->dev->of_node;
	indio_dev->name = "qpnp,qg";
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = chip->iio_chan;
	indio_dev->num_channels = qg_num_iio_channels;

	for (i = 0; i < qg_num_iio_channels; i++) {
		chip->int_iio_chans[i].indio_dev = indio_dev;
		chan = &chip->iio_chan[i];
		chip->int_iio_chans[i].channel = chan;
		chan->address = i;
		chan->channel = qg_iio_psy_channels[i].channel_num;
		chan->type = qg_iio_psy_channels[i].type;
		chan->datasheet_name =
			qg_iio_psy_channels[i].datasheet_name;
		chan->extend_name =
			qg_iio_psy_channels[i].datasheet_name;
		chan->info_mask_separate =
			qg_iio_psy_channels[i].info_mask;
	}

	rc = devm_iio_device_register(chip->dev, indio_dev);
	if (rc)
		pr_err("Failed to register QG IIO device, rc=%d\n", rc);

	return rc;
}

static ssize_t qg_device_read(struct file *file, char __user *buf, size_t count,
			  loff_t *ppos)
{
	int rc;
	struct qpnp_qg *chip = file->private_data;
	unsigned long data_size = sizeof(chip->kdata);

	if (count < data_size) {
		pr_err("Invalid datasize %lu, expected lesser then %zu\n",
							data_size, count);
		return -EINVAL;
	}

	/* non-blocking access, return */
	if (!chip->data_ready && (file->f_flags & O_NONBLOCK))
		return 0;

	/* blocking access wait on data_ready */
	if (!(file->f_flags & O_NONBLOCK)) {
		rc = wait_event_interruptible(chip->qg_wait_q,
					chip->data_ready);
		if (rc < 0) {
			pr_debug("Failed wait! rc=%d\n", rc);
			return rc;
		}
	}

	mutex_lock(&chip->data_lock);

	if (!chip->data_ready) {
		pr_debug("No Data, false wakeup\n");
		rc = -EFAULT;
		goto fail_read;
	}


	if (copy_to_user(buf, &chip->kdata, data_size)) {
		pr_err("Failed in copy_to_user\n");
		rc = -EFAULT;
		goto fail_read;
	}
	chip->data_ready = false;

	/* release all wake sources */
	vote(chip->awake_votable, GOOD_OCV_VOTER, false, 0);
	vote(chip->awake_votable, FIFO_DONE_VOTER, false, 0);
	vote(chip->awake_votable, FIFO_RT_DONE_VOTER, false, 0);
	vote(chip->awake_votable, SUSPEND_DATA_VOTER, false, 0);
	vote(chip->awake_votable, SLEEP_EXIT_DATA_VOTER, false, 0);

	qg_dbg(chip, QG_DEBUG_DEVICE,
		"QG device read complete Seq_no=%u Size=%ld\n",
				chip->kdata.seq_no, data_size);

	/* clear data */
	memset(&chip->kdata, 0, sizeof(chip->kdata));

	mutex_unlock(&chip->data_lock);

	return data_size;

fail_read:
	mutex_unlock(&chip->data_lock);
	return rc;
}

static ssize_t qg_device_write(struct file *file, const char __user *buf,
			size_t count, loff_t *ppos)
{
	int rc = -EINVAL;
	struct qpnp_qg *chip = file->private_data;
	unsigned long data_size = sizeof(chip->udata);

	mutex_lock(&chip->data_lock);
	if (count == 0) {
		pr_err("No data!\n");
		goto fail;
	}

	if (count != 0 && count < data_size) {
		pr_err("Invalid datasize %zu expected %lu\n", count, data_size);
		goto fail;
	}

	if (copy_from_user(&chip->udata, buf, data_size)) {
		pr_err("Failed in copy_from_user\n");
		rc = -EFAULT;
		goto fail;
	}

	rc = data_size;
	vote(chip->awake_votable, UDATA_READY_VOTER, true, 0);
	schedule_work(&chip->udata_work);
	qg_dbg(chip, QG_DEBUG_DEVICE, "QG write complete size=%d\n", rc);
fail:
	mutex_unlock(&chip->data_lock);
	return rc;
}

static unsigned int qg_device_poll(struct file *file, poll_table *wait)
{
	struct qpnp_qg *chip = file->private_data;
	unsigned int mask = 0;

	poll_wait(file, &chip->qg_wait_q, wait);

	if (chip->data_ready)
		mask = POLLIN | POLLRDNORM;

	return mask;
}

static int qg_device_open(struct inode *inode, struct file *file)
{
	struct qpnp_qg *chip = container_of(inode->i_cdev,
				struct qpnp_qg, qg_cdev);

	file->private_data = chip;
	chip->qg_device_open = true;
	qg_dbg(chip, QG_DEBUG_DEVICE, "QG device opened!\n");

	return 0;
}

static int qg_device_release(struct inode *inode, struct file *file)
{
	struct qpnp_qg *chip = container_of(inode->i_cdev,
				struct qpnp_qg, qg_cdev);

	file->private_data = chip;
	chip->qg_device_open = false;
	qg_dbg(chip, QG_DEBUG_DEVICE, "QG device closed!\n");

	return 0;
}

static const struct file_operations qg_fops = {
	.owner		= THIS_MODULE,
	.open		= qg_device_open,
	.release	= qg_device_release,
	.read		= qg_device_read,
	.write		= qg_device_write,
	.poll		= qg_device_poll,
};

static int qg_register_device(struct qpnp_qg *chip)
{
	int rc;

	rc = alloc_chrdev_region(&chip->dev_no, 0, 1, "qg");
	if (rc < 0) {
		pr_err("Failed to allocate chardev rc=%d\n", rc);
		return rc;
	}

	cdev_init(&chip->qg_cdev, &qg_fops);
	rc = cdev_add(&chip->qg_cdev, chip->dev_no, 1);
	if (rc < 0) {
		pr_err("Failed to cdev_add rc=%d\n", rc);
		goto unregister_chrdev;
	}

	chip->qg_class = class_create(THIS_MODULE, "qg");
	if (IS_ERR_OR_NULL(chip->qg_class)) {
		pr_err("Failed to create qg class\n");
		rc = -EINVAL;
		goto delete_cdev;
	}
	chip->qg_device = device_create(chip->qg_class, NULL, chip->dev_no,
					NULL, "qg");
	if (IS_ERR(chip->qg_device)) {
		pr_err("Failed to create qg_device\n");
		rc = -EINVAL;
		goto destroy_class;
	}

	qg_dbg(chip, QG_DEBUG_DEVICE, "'/dev/qg' successfully created\n");

	return 0;

destroy_class:
	class_destroy(chip->qg_class);
delete_cdev:
	cdev_del(&chip->qg_cdev);
unregister_chrdev:
	unregister_chrdev_region(chip->dev_no, 1);
	return rc;
}

#define BID_RPULL_OHM		100000
#define BID_VREF_MV		1875
static int get_batt_id_ohm(struct qpnp_qg *chip, u32 *batt_id_ohm)
{
	int rc, batt_id_mv;
	int64_t denom;

	/* Read battery-id */
	rc = iio_read_channel_processed(chip->batt_id_chan, &batt_id_mv);
	if (rc < 0) {
		pr_err("Failed to read BATT_ID over ADC, rc=%d\n", rc);
		return rc;
	}

	batt_id_mv = div_s64(batt_id_mv, 1000);
	if (batt_id_mv == 0) {
		pr_debug("batt_id_mv = 0 from ADC\n");
		return 0;
	}

	denom = div64_s64(BID_VREF_MV * 1000, batt_id_mv) - 1000;
	if (denom <= 0) {
		/* batt id connector might be open, return 0 kohms */
		return 0;
	}

	*batt_id_ohm = div64_u64(BID_RPULL_OHM * 1000 + denom / 2, denom);

	qg_dbg(chip, QG_DEBUG_PROFILE, "batt_id_mv=%d, batt_id_ohm=%d\n",
					batt_id_mv, *batt_id_ohm);

	return 0;
}

#if defined(CONFIG_SOMC_CHARGER_EXTENSION)
#define PROPERTY_NAME_SIZE 128
#endif
static int qg_load_battery_profile(struct qpnp_qg *chip)
{
	struct device_node *node = chip->dev->of_node;
	struct device_node *profile_node;
	int rc, tuple_len, len, i, avail_age_level = 0;
#if defined(CONFIG_SOMC_CHARGER_EXTENSION)
	int num, lv;
	int step_len;
	u32 step_buf[STEP_DATA_DT_MAX_NUM];
	char prop_name[PROPERTY_NAME_SIZE];

	memset(step_buf, 0, sizeof(step_buf));
#endif

	chip->batt_node = of_find_node_by_name(node, "qcom,battery-data");
	if (!chip->batt_node) {
		pr_err("Batterydata not available\n");
		return -ENXIO;
	}

	if (chip->dt.multi_profile_load) {
		if (chip->batt_age_level == -EINVAL) {
			rc = qg_get_batt_age_level(chip, &chip->batt_age_level);
			if (rc < 0) {
				pr_err("error in retrieving batt age level rc=%d\n",
									rc);
				return rc;
			}
		}
		profile_node = of_batterydata_get_best_aged_profile(
					chip->batt_node,
					chip->batt_id_ohm / 1000,
					chip->batt_age_level,
					&avail_age_level);
		if (chip->batt_age_level != avail_age_level) {
			qg_dbg(chip, QG_DEBUG_PROFILE, "Batt_age_level %d doesn't exist, using %d\n",
					chip->batt_age_level, avail_age_level);
			chip->batt_age_level = avail_age_level;
		}
	} else {
#if defined(CONFIG_SOMC_CHARGER_EXTENSION)
		if (chip->batt_age_level == -EINVAL) {
			rc = qg_get_batt_age_level(chip, &chip->batt_age_level);
			if (rc < 0) {
				pr_err("error in retrieving batt age level rc=%d\n",
									rc);
				chip->batt_age_level = 0;
			}
		}
		qg_dbg(chip, QG_DEBUG_SOMC, "Read batt_age_level from SDAM: lv=%d\n",
							chip->batt_age_level);
#endif
		profile_node = of_batterydata_get_best_profile(chip->batt_node,
				chip->batt_id_ohm / 1000, NULL);
	}

	if (IS_ERR(profile_node)) {
		rc = PTR_ERR(profile_node);
		pr_err("Failed to detect valid QG battery profile %d\n", rc);
		return rc;
	}

	rc = of_property_read_string(profile_node, "qcom,battery-type",
				&chip->bp.batt_type_str);
	if (rc < 0) {
		pr_err("Failed to detect battery type rc:%d\n", rc);
		return rc;
	}

	rc = qg_batterydata_init(profile_node);
	if (rc < 0) {
		pr_err("Failed to initialize battery-profile rc=%d\n", rc);
		return rc;
	}

	rc = of_property_read_u32(profile_node, "qcom,max-voltage-uv",
				&chip->bp.float_volt_uv);
	if (rc < 0) {
		pr_err("Failed to read battery float-voltage rc:%d\n", rc);
		chip->bp.float_volt_uv = -EINVAL;
	}

	rc = of_property_read_u32(profile_node, "qcom,fastchg-current-ma",
				&chip->bp.fastchg_curr_ma);
	if (rc < 0) {
		pr_err("Failed to read battery fastcharge current rc:%d\n", rc);
		chip->bp.fastchg_curr_ma = -EINVAL;
	}

	/*
	 * Update the max fcc values based on QG subtype including
	 * error margins.
	 */
	chip->bp.fastchg_curr_ma = min(chip->max_fcc_limit_ma,
					chip->bp.fastchg_curr_ma);

	rc = of_property_read_u32(profile_node, "qcom,qg-batt-profile-ver",
				&chip->bp.qg_profile_version);
	if (rc < 0) {
		pr_err("Failed to read QG profile version rc:%d\n", rc);
		chip->bp.qg_profile_version = -EINVAL;
	}
#if defined(CONFIG_SOMC_CHARGER_EXTENSION)
	rc = of_property_read_u32(profile_node, "somc,initial-capacity-uah",
				&chip->initial_capacity);
	if (rc < 0) {
		pr_err("battery initial capacity unavailable, rc:%d\n", rc);
		chip->initial_capacity = -EINVAL;
	}

	qg_notify_charger(chip);
#endif
	/*
	 * Currently step charging thresholds should be read only for Vbatt
	 * based and not for SOC based.
	 */
	if (!of_property_read_bool(profile_node, "qcom,soc-based-step-chg") &&
		of_find_property(profile_node, "qcom,step-chg-ranges", &len) &&
		chip->bp.float_volt_uv > 0 && chip->bp.fastchg_curr_ma > 0) {
		len /= sizeof(u32);
		tuple_len = len / (sizeof(struct range_data) / sizeof(u32));
		if (tuple_len <= 0 || tuple_len > MAX_STEP_CHG_ENTRIES)
			return -EINVAL;

		mutex_lock(&chip->ttf->lock);
		chip->ttf->step_chg_cfg =
			kcalloc(len, sizeof(*chip->ttf->step_chg_cfg),
				GFP_KERNEL);
		if (!chip->ttf->step_chg_cfg) {
			mutex_unlock(&chip->ttf->lock);
			return -ENOMEM;
		}

		chip->ttf->step_chg_data =
			kcalloc(tuple_len, sizeof(*chip->ttf->step_chg_data),
				GFP_KERNEL);
		if (!chip->ttf->step_chg_data) {
			kfree(chip->ttf->step_chg_cfg);
			mutex_unlock(&chip->ttf->lock);
			return -ENOMEM;
		}

		rc = qg_read_range_data_from_node(profile_node,
				"qcom,step-chg-ranges",
				chip->ttf->step_chg_cfg,
				chip->bp.float_volt_uv,
				chip->bp.fastchg_curr_ma * 1000);
		if (rc < 0) {
			pr_err("Error in reading qcom,step-chg-ranges from battery profile, rc=%d\n",
				rc);
			kfree(chip->ttf->step_chg_data);
			kfree(chip->ttf->step_chg_cfg);
			chip->ttf->step_chg_cfg = NULL;
			mutex_unlock(&chip->ttf->lock);
			return rc;
		}

		chip->ttf->step_chg_num_params = tuple_len;
		chip->ttf->step_chg_cfg_valid = true;
		mutex_unlock(&chip->ttf->lock);

		if (chip->ttf->step_chg_cfg_valid) {
			for (i = 0; i < tuple_len; i++)
				pr_debug("Vbatt_low: %d Vbatt_high: %d FCC: %d\n",
				chip->ttf->step_chg_cfg[i].low_threshold,
				chip->ttf->step_chg_cfg[i].high_threshold,
				chip->ttf->step_chg_cfg[i].value);
		}
	}

#if defined(CONFIG_SOMC_CHARGER_EXTENSION)
	mutex_lock(&chip->step_lock);
	chip->step_en = false;
	rc = of_property_count_elems_of_size(profile_node,
					"somc,step-cfg", sizeof(u32));

	for (lv = 0; lv < AGING_LV_NUM; lv++) {
		snprintf(prop_name, PROPERTY_NAME_SIZE, "somc,cvstep-cfg-lv%d", lv);
		rc = of_property_count_elems_of_size(profile_node, prop_name, sizeof(u32));
		if (rc < 0) {
			pr_err("Can't get size of %s %d\n", prop_name, rc);
			goto step_err;
		} else {
			step_len = rc;
			qg_dbg(chip, QG_DEBUG_SOMC_STEP,
				"size of %s: %d\n", prop_name, step_len);
		}
		if (step_len > STEP_DATA_DT_MAX_NUM || (step_len % STEP_DATA_RAW)) {
			pr_err("step config table size is invalid\n");
			goto step_err;
		} else {
			rc = of_property_read_u32_array(profile_node, prop_name,
								&step_buf[0], step_len);
			if (rc < 0) {
				pr_err("Can't get %s %d\n", prop_name, rc);
				goto step_err;
			}
			num = 0;
			for (i = 0; i < step_len; i += STEP_DATA_RAW) {
				chip->step_data[lv][num].temp_range_l =
							step_buf[i];
				chip->step_data[lv][num].temp_range_h =
							step_buf[i + 1];
				chip->step_data[lv][num].jeita_zone =
							step_buf[i + 2];
				chip->step_data[lv][num].step_phase =
							step_buf[i + 3];
				chip->step_data[lv][num].fv =
							step_buf[i + 4];
				chip->step_data[lv][num].fcc =
							step_buf[i + 5];
				chip->step_data[lv][num].target_current =
							step_buf[i + 6];
				chip->step_data[lv][num].check_current =
							step_buf[i + 7];
				chip->step_data[lv][num].term_current =
							step_buf[i + 8];
				chip->step_data[lv][num].health =
							step_buf[i + 9];
				num++;
			}

			/* Check Step Data */
			qg_dbg(chip, QG_DEBUG_SOMC, "Step/JEITA table for Aging Level %d\n", lv);
			qg_dbg(chip, QG_DEBUG_SOMC, "| No |  Temp Range   |Zone|Step|  FV  | FCC  |Target| Term |Hlth|\n");
			for (i = 0; i < num; i++)
				qg_dbg(chip, QG_DEBUG_SOMC,
					"| %2d | %5d - %5d | %2d | %2d | %4d | %4d | %4d | %4d | %2d |\n",
					i + 1,
					chip->step_data[lv][i].temp_range_l,
					chip->step_data[lv][i].temp_range_h,
					chip->step_data[lv][i].jeita_zone,
					chip->step_data[lv][i].step_phase,
					chip->step_data[lv][i].fv,
					chip->step_data[lv][i].fcc,
					chip->step_data[lv][i].target_current,
					chip->step_data[lv][i].term_current,
					chip->step_data[lv][i].health);

		}
	}
	chip->step_iterm = -1;
	chip->step_en = true;
step_err:
	mutex_unlock(&chip->step_lock);

#endif
	qg_dbg(chip, QG_DEBUG_PROFILE, "profile=%s FV=%duV FCC=%dma\n",
			chip->bp.batt_type_str, chip->bp.float_volt_uv,
			chip->bp.fastchg_curr_ma);

	return 0;
}

static int qg_setup_battery(struct qpnp_qg *chip)
{
	int rc;

	if (!is_battery_present(chip)) {
		qg_dbg(chip, QG_DEBUG_PROFILE, "Battery Missing!\n");
		chip->battery_missing = true;
		chip->profile_loaded = false;
		chip->soc_reporting_ready = true;
	} else {
		/* battery present */
		rc = get_batt_id_ohm(chip, &chip->batt_id_ohm);
		if (rc < 0) {
			pr_err("Failed to detect batt_id rc=%d\n", rc);
			chip->profile_loaded = false;
		} else {
			rc = qg_load_battery_profile(chip);
			if (rc < 0) {
				pr_err("Failed to load battery-profile rc=%d\n",
								rc);
				chip->profile_loaded = false;
				chip->soc_reporting_ready = true;
			} else {
				chip->profile_loaded = true;
			}
		}
	}

	qg_dbg(chip, QG_DEBUG_PROFILE, "battery_missing=%d batt_id_ohm=%d Ohm profile_loaded=%d profile=%s\n",
			chip->battery_missing, chip->batt_id_ohm,
			chip->profile_loaded, chip->bp.batt_type_str);

	return 0;
}

static struct ocv_all ocv[] = {
	[S7_PON_OCV] = { 0, 0, "S7_PON_OCV"},
	[S3_GOOD_OCV] = { 0, 0, "S3_GOOD_OCV"},
	[S3_LAST_OCV] = { 0, 0, "S3_LAST_OCV"},
	[SDAM_PON_OCV] = { 0, 0, "SDAM_PON_OCV"},
};

#define S7_ERROR_MARGIN_UV		20000
static int qg_determine_pon_soc(struct qpnp_qg *chip)
{
	int rc = 0, batt_temp = 0, i, shutdown_temp = 0;
	bool use_pon_ocv = true;
	unsigned long rtc_sec = 0;
	u32 ocv_uv = 0, soc = 0, pon_soc = 0, full_soc = 0, cutoff_soc = 0;
	u32 shutdown[SDAM_MAX] = {0}, soc_raw = 0;
	char ocv_type[20] = "NONE";

	if (!chip->profile_loaded) {
		qg_dbg(chip, QG_DEBUG_PON, "No Profile, skipping PON soc\n");
		return 0;
	}

	/* read all OCVs */
	for (i = S7_PON_OCV; i < PON_OCV_MAX; i++) {
		rc = qg_read_ocv(chip, &ocv[i].ocv_uv,
					&ocv[i].ocv_raw, i);
		if (rc < 0)
			pr_err("Failed to read %s OCV rc=%d\n",
					ocv[i].ocv_type, rc);
		else
			qg_dbg(chip, QG_DEBUG_PON, "%s OCV=%d\n",
					ocv[i].ocv_type, ocv[i].ocv_uv);
	}

	rc = qg_get_battery_temp(chip, &batt_temp);
	if (rc < 0) {
		pr_err("Failed to read BATT_TEMP at PON rc=%d\n", rc);
		goto done;
	}

	rc = get_rtc_time(&rtc_sec);
	if (rc < 0) {
		pr_err("Failed to read RTC time rc=%d\n", rc);
		goto use_pon_ocv;
	}

	rc = qg_sdam_read_all(shutdown);
	if (rc < 0) {
		pr_err("Failed to read shutdown params rc=%d\n", rc);
		goto use_pon_ocv;
	}
	shutdown_temp = sign_extend32(shutdown[SDAM_TEMP], 15);

	rc = lookup_soc_ocv(&pon_soc, ocv[S7_PON_OCV].ocv_uv, batt_temp, false);
	if (rc < 0) {
		pr_err("Failed to lookup S7_PON SOC rc=%d\n", rc);
		goto done;
	}

	qg_dbg(chip, QG_DEBUG_PON, "Shutdown: Valid=%d SOC=%d OCV=%duV time=%dsecs temp=%d, time_now=%ldsecs temp_now=%d S7_soc=%d\n",
			shutdown[SDAM_VALID],
			shutdown[SDAM_SOC],
			shutdown[SDAM_OCV_UV],
			shutdown[SDAM_TIME_SEC],
			shutdown_temp,
			rtc_sec, batt_temp,
			pon_soc);
	/*
	 * Use the shutdown SOC if
	 * 1. SDAM read is a success & SDAM data is valid
	 * 2. The device was powered off for < ignore_shutdown_time
	 * 2. Batt temp has not changed more than shutdown_temp_diff
	 */
	if (!shutdown[SDAM_VALID])
		goto use_pon_ocv;

	if (!is_between(0, chip->dt.ignore_shutdown_soc_secs,
			(rtc_sec - shutdown[SDAM_TIME_SEC])))
		goto use_pon_ocv;

	if (!is_between(0, chip->dt.shutdown_temp_diff,
			abs(shutdown_temp -  batt_temp)) &&
			(shutdown_temp < 0 || batt_temp < 0))
		goto use_pon_ocv;

	if ((chip->dt.shutdown_soc_threshold != -EINVAL) &&
			!is_between(0, chip->dt.shutdown_soc_threshold,
			abs(pon_soc - shutdown[SDAM_SOC])))
		goto use_pon_ocv;

	use_pon_ocv = false;
	ocv_uv = shutdown[SDAM_OCV_UV];
	soc = shutdown[SDAM_SOC];
	soc_raw = shutdown[SDAM_SOC] * 100;
	strlcpy(ocv_type, "SHUTDOWN_SOC", 20);
	qg_dbg(chip, QG_DEBUG_PON, "Using SHUTDOWN_SOC @ PON\n");

use_pon_ocv:
	if (use_pon_ocv) {
		if (chip->wa_flags & QG_PON_OCV_WA) {
			if (ocv[S3_LAST_OCV].ocv_raw == FIFO_V_RESET_VAL) {
				if (!ocv[SDAM_PON_OCV].ocv_uv) {
					strlcpy(ocv_type, "S7_PON_SOC", 20);
					ocv_uv = ocv[S7_PON_OCV].ocv_uv;
				} else if (ocv[SDAM_PON_OCV].ocv_uv <=
						ocv[S7_PON_OCV].ocv_uv) {
					strlcpy(ocv_type, "S7_PON_SOC", 20);
					ocv_uv = ocv[S7_PON_OCV].ocv_uv;
				} else if (!shutdown[SDAM_VALID] &&
					((ocv[SDAM_PON_OCV].ocv_uv -
						ocv[S7_PON_OCV].ocv_uv) >
						S7_ERROR_MARGIN_UV)) {
					strlcpy(ocv_type, "S7_PON_SOC", 20);
					ocv_uv = ocv[S7_PON_OCV].ocv_uv;
				} else {
					strlcpy(ocv_type, "SDAM_PON_SOC", 20);
					ocv_uv = ocv[SDAM_PON_OCV].ocv_uv;
				}
			} else {
				if (ocv[S3_LAST_OCV].ocv_uv >=
						ocv[S7_PON_OCV].ocv_uv) {
					strlcpy(ocv_type, "S3_LAST_SOC", 20);
					ocv_uv = ocv[S3_LAST_OCV].ocv_uv;
				} else {
					strlcpy(ocv_type, "S7_PON_SOC", 20);
					ocv_uv = ocv[S7_PON_OCV].ocv_uv;
				}
			}
		} else {
			/* Use S7 PON OCV */
			strlcpy(ocv_type, "S7_PON_SOC", 20);
			ocv_uv = ocv[S7_PON_OCV].ocv_uv;
		}

		ocv_uv = CAP(QG_MIN_OCV_UV, QG_MAX_OCV_UV, ocv_uv);
		rc = lookup_soc_ocv(&pon_soc, ocv_uv, batt_temp, false);
		if (rc < 0) {
			pr_err("Failed to lookup SOC@PON rc=%d\n", rc);
			goto done;
		}

		rc = lookup_soc_ocv(&full_soc, chip->bp.float_volt_uv,
							batt_temp, true);
		if (rc < 0) {
			pr_err("Failed to lookup FULL_SOC@PON rc=%d\n", rc);
			goto done;
		}
		full_soc = CAP(0, 99, full_soc);

		rc = lookup_soc_ocv(&cutoff_soc,
				chip->dt.vbatt_cutoff_mv * 1000,
				batt_temp, false);
		if (rc < 0) {
			pr_err("Failed to lookup CUTOFF_SOC@PON rc=%d\n", rc);
			goto done;
		}

		if ((full_soc > cutoff_soc) && (pon_soc > cutoff_soc)) {
			soc = DIV_ROUND_UP(((pon_soc - cutoff_soc) * 100),
						(full_soc - cutoff_soc));
			soc = CAP(0, 100, soc);
			soc_raw = soc * 100;
		} else {
			soc_raw = pon_soc * 100;
			soc = pon_soc;
		}

		qg_dbg(chip, QG_DEBUG_PON, "v_float=%d v_cutoff=%d FULL_SOC=%d CUTOFF_SOC=%d PON_SYS_SOC=%d pon_soc=%d\n",
			chip->bp.float_volt_uv, chip->dt.vbatt_cutoff_mv * 1000,
			full_soc, cutoff_soc, soc, pon_soc);
	}
done:
	if (rc < 0) {
		pr_err("Failed to get %s @ PON, rc=%d\n", ocv_type, rc);
		return rc;
	}

	if (chip->qg_mode == QG_V_I_MODE)
		chip->cc_soc = soc_raw;
	chip->sys_soc = soc_raw;
#if defined(CONFIG_SOMC_CHARGER_EXTENSION)
	chip->batt_soc = soc_raw;
#endif
	chip->last_adj_ssoc = chip->catch_up_soc = chip->msoc = soc;
	chip->kdata.param[QG_PON_OCV_UV].data = ocv_uv;
	chip->kdata.param[QG_PON_OCV_UV].valid = true;

	/* write back to SDAM */
	chip->sdam_data[SDAM_SOC] = soc;
	chip->sdam_data[SDAM_OCV_UV] = ocv_uv;
	chip->sdam_data[SDAM_VALID] = 1;

	rc = qg_write_monotonic_soc(chip, chip->msoc);
	if (rc < 0)
		pr_err("Failed to update MSOC register rc=%d\n", rc);

	rc = qg_store_soc_params(chip);
	if (rc < 0)
		pr_err("Failed to update sdam params rc=%d\n", rc);

	pr_info("using %s @ PON ocv_uv=%duV soc=%d\n",
			ocv_type, ocv_uv, chip->msoc);

	/* SOC reporting is now ready */
	chip->soc_reporting_ready = 1;

	return 0;
}

static int qg_set_wa_flags(struct qpnp_qg *chip)
{
	switch (chip->pmic_version) {
	case PMI632:
		chip->wa_flags |= QG_RECHARGE_SOC_WA;
		if (!chip->dt.use_s7_ocv)
			chip->wa_flags |= QG_PON_OCV_WA;
		break;
	case PM6150:
		chip->wa_flags |= QG_CLK_ADJUST_WA |
				QG_RECHARGE_SOC_WA;
		qg_esr_mod_count = 10;
		break;
	case PM7250B:
		qg_esr_mod_count = 10;
		break;
	case PM2250:
		chip->wa_flags |= QG_CLK_ADJUST_WA |
				QG_RECHARGE_SOC_WA |
				QG_VBAT_LOW_WA;
		break;
	default:
		pr_err("Unsupported PMIC version %d\n",
			chip->pmic_version);
		return -EINVAL;
	}

	qg_dbg(chip, QG_DEBUG_PON, "wa_flags = %x\n", chip->wa_flags);

	return 0;
}

#define SDAM_MAGIC_NUMBER		0x12345678
static int qg_sanitize_sdam(struct qpnp_qg *chip)
{
	int rc = 0;
	u32 data = 0;

	rc = qg_sdam_read(SDAM_MAGIC, &data);
	if (rc < 0) {
		pr_err("Failed to read SDAM rc=%d\n", rc);
		return rc;
	}

	if (data == SDAM_MAGIC_NUMBER) {
		qg_dbg(chip, QG_DEBUG_PON, "SDAM valid\n");
	} else if (data == 0) {
		rc = qg_sdam_write(SDAM_MAGIC, SDAM_MAGIC_NUMBER);
		if (!rc)
			qg_dbg(chip, QG_DEBUG_PON, "First boot. SDAM initilized\n");
		chip->first_profile_load = true;
	} else {
		/* SDAM has invalid value */
		rc = qg_sdam_clear();
		if (!rc) {
			pr_err("SDAM uninitialized, SDAM reset\n");
			rc = qg_sdam_write(SDAM_MAGIC, SDAM_MAGIC_NUMBER);
		}
		chip->first_profile_load = true;
	}

	if (rc < 0)
		pr_err("Failed in SDAM operation, rc=%d\n", rc);

	return rc;
}

#define ADC_CONV_DLY_512MS		0xA
#define IBAT_5A_FCC_MA			4800
#define IBAT_10A_FCC_MA			9600
static int qg_hw_init(struct qpnp_qg *chip)
{
	int rc, temp;
	u8 reg;

	/* read STATUS2 register to clear its last state */
	qg_read(chip, chip->qg_base + QG_STATUS2_REG, &reg, 1);

	/* read the QG perph_subtype */
	rc = qg_read(chip, chip->qg_base + PERPH_SUBTYPE_REG,
					&chip->qg_subtype, 1);
	if (rc < 0) {
		pr_err("Failed to read QG subtype rc=%d\n", rc);
		return rc;
	}

	if (chip->qg_subtype == QG_ADC_IBAT_5A)
		chip->max_fcc_limit_ma = IBAT_5A_FCC_MA;
	else
		chip->max_fcc_limit_ma = IBAT_10A_FCC_MA;

	if (chip->qg_version == QG_LITE) {
		rc = qg_read(chip, chip->qg_base + QG_MODE_CTL2_REG, &reg, 1);
		if (rc < 0) {
			pr_err("Failed to read QG mode rc=%d\n", rc);
			return rc;
		}
		chip->qg_mode = (reg & VI_MODE_BIT) ? QG_V_I_MODE : QG_V_MODE;
	} else {
		chip->qg_mode = QG_V_I_MODE;
	}

	if (chip->qg_mode == QG_V_MODE) {
		chip->dt.esr_disable = true;
		chip->dt.cl_disable = true;
		chip->dt.tcss_enable = false;
		chip->dt.bass_enable = false;
	}

	rc = qg_set_wa_flags(chip);
	if (rc < 0) {
		pr_err("Failed to update PMIC type flags, rc=%d\n", rc);
		return rc;
	}

	rc = qg_master_hold(chip, true);
	if (rc < 0) {
		pr_err("Failed to hold master, rc=%d\n", rc);
		goto done_fifo;
	}

	rc = qg_process_rt_fifo(chip);
	if (rc < 0) {
		pr_err("Failed to process FIFO real-time, rc=%d\n", rc);
		goto done_fifo;
	}

	/* update the changed S2 fifo DT parameters */
	if (chip->dt.s2_fifo_length > 0) {
		rc = qg_update_fifo_length(chip, chip->dt.s2_fifo_length);
		if (rc < 0)
			goto done_fifo;
	}

	if (chip->dt.s2_acc_length > 0) {
		reg = ilog2(chip->dt.s2_acc_length) - 1;
		rc = qg_masked_write(chip, chip->qg_base +
				QG_S2_NORMAL_MEAS_CTL2_REG,
				NUM_OF_ACCUM_MASK, reg);
		if (rc < 0) {
			pr_err("Failed to write S2 ACC length, rc=%d\n", rc);
			goto done_fifo;
		}
	}

	if (chip->dt.s2_acc_intvl_ms > 0) {
		reg = chip->dt.s2_acc_intvl_ms / 10;
		rc = qg_write(chip, chip->qg_base +
				QG_S2_NORMAL_MEAS_CTL3_REG,
				&reg, 1);
		if (rc < 0) {
			pr_err("Failed to write S2 ACC intrvl, rc=%d\n", rc);
			goto done_fifo;
		}
	}

	chip->s2_state = S2_DEFAULT;
	chip->s2_state_mask |= S2_DEFAULT;
	/* signal the read thread */
	chip->data_ready = true;
	wake_up_interruptible(&chip->qg_wait_q);

done_fifo:
	rc = qg_master_hold(chip, false);
	if (rc < 0) {
		pr_err("Failed to release master, rc=%d\n", rc);
		return rc;
	}
	chip->last_fifo_update_time = ktime_get_boottime();

	if (chip->dt.ocv_timer_expiry_min != -EINVAL &&
				chip->qg_version != QG_LITE) {
		if (chip->dt.ocv_timer_expiry_min < 2)
			chip->dt.ocv_timer_expiry_min = 2;
		else if (chip->dt.ocv_timer_expiry_min > 30)
			chip->dt.ocv_timer_expiry_min = 30;

		reg = (chip->dt.ocv_timer_expiry_min - 2) / 4;
		rc = qg_masked_write(chip,
			chip->qg_base + QG_S3_SLEEP_OCV_MEAS_CTL4_REG,
			SLEEP_IBAT_QUALIFIED_LENGTH_MASK, reg);
		if (rc < 0) {
			pr_err("Failed to write OCV timer, rc=%d\n", rc);
			return rc;
		}
	}

	if (chip->dt.ocv_tol_threshold_uv != -EINVAL) {
		if (chip->dt.ocv_tol_threshold_uv < 0)
			chip->dt.ocv_tol_threshold_uv = 0;
		else if (chip->dt.ocv_tol_threshold_uv > 12262)
			chip->dt.ocv_tol_threshold_uv = 12262;

		reg = chip->dt.ocv_tol_threshold_uv / 195;
		rc = qg_masked_write(chip,
			chip->qg_base + QG_S3_SLEEP_OCV_TREND_CTL2_REG,
			TREND_TOL_MASK, reg);
		if (rc < 0) {
			pr_err("Failed to write OCV tol-thresh, rc=%d\n", rc);
			return rc;
		}
	}

	if (chip->dt.s3_entry_fifo_length != -EINVAL) {
		if (chip->dt.s3_entry_fifo_length < 1)
			chip->dt.s3_entry_fifo_length = 1;
		else if (chip->dt.s3_entry_fifo_length >
					chip->max_fifo_length)
			chip->dt.s3_entry_fifo_length =
					chip->max_fifo_length;

		reg = chip->dt.s3_entry_fifo_length - 1;
		rc = qg_masked_write(chip,
			chip->qg_base + QG_S3_SLEEP_OCV_IBAT_CTL1_REG,
			SLEEP_IBAT_QUALIFIED_LENGTH_MASK, reg);
		if (rc < 0) {
			pr_err("Failed to write S3-entry fifo-length, rc=%d\n",
							rc);
			return rc;
		}
	}

	if (chip->dt.s3_entry_ibat_ua != -EINVAL) {
		if (chip->dt.s3_entry_ibat_ua < 0)
			chip->dt.s3_entry_ibat_ua = 0;
		else if (chip->dt.s3_entry_ibat_ua > 155550)
			chip->dt.s3_entry_ibat_ua = 155550;

		reg = chip->dt.s3_entry_ibat_ua / 610;
		rc = qg_write(chip, chip->qg_base +
				QG_S3_ENTRY_IBAT_THRESHOLD_REG,
				&reg, 1);
		if (rc < 0) {
			pr_err("Failed to write S3-entry ibat-uA, rc=%d\n", rc);
			return rc;
		}
	}

	if (chip->dt.s3_exit_ibat_ua != -EINVAL) {
		if (chip->dt.s3_exit_ibat_ua < 0)
			chip->dt.s3_exit_ibat_ua = 0;
		else if (chip->dt.s3_exit_ibat_ua > 155550)
			chip->dt.s3_exit_ibat_ua = 155550;

		rc = qg_read(chip, chip->qg_base +
				QG_S3_ENTRY_IBAT_THRESHOLD_REG,
				&reg, 1);
		if (rc < 0) {
			pr_err("Failed to read S3-entry ibat-uA, rc=%d\n", rc);
			return rc;
		}
		temp = reg * 610;
		if (chip->dt.s3_exit_ibat_ua < temp)
			chip->dt.s3_exit_ibat_ua = temp;
		else
			chip->dt.s3_exit_ibat_ua -= temp;

		reg = chip->dt.s3_exit_ibat_ua / 610;
		rc = qg_write(chip,
			chip->qg_base + QG_S3_EXIT_IBAT_THRESHOLD_REG,
			&reg, 1);
		if (rc < 0) {
			pr_err("Failed to write S3-entry ibat-uA, rc=%d\n", rc);
			return rc;
		}
	}

	/* vbat based configs */
	if (chip->dt.vbatt_low_mv < 0)
		chip->dt.vbatt_low_mv = 0;
	else if (chip->dt.vbatt_low_mv > 12750)
		chip->dt.vbatt_low_mv = 12750;

	if (chip->dt.vbatt_empty_mv < 0)
		chip->dt.vbatt_empty_mv = 0;
	else if (chip->dt.vbatt_empty_mv > 12750)
		chip->dt.vbatt_empty_mv = 12750;

	if (chip->dt.vbatt_empty_cold_mv < 0)
		chip->dt.vbatt_empty_cold_mv = 0;
	else if (chip->dt.vbatt_empty_cold_mv > 12750)
		chip->dt.vbatt_empty_cold_mv = 12750;

	rc = qg_vbat_thresholds_config(chip);
	if (rc < 0) {
		pr_err("Failed to configure VBAT empty/low rc=%d\n", rc);
		return rc;
	}

	if (chip->qg_version != QG_LITE) {
		/* disable S5 */
		rc = qg_masked_write(chip, chip->qg_base +
					QG_S5_OCV_VALIDATE_MEAS_CTL1_REG,
					ALLOW_S5_BIT, 0);
		if (rc < 0)
			pr_err("Failed to disable S5 rc=%d\n", rc);
	}

	/* change PON OCV time to 512ms */
	rc = qg_masked_write(chip, chip->qg_base +
				QG_S7_PON_OCV_MEAS_CTL1_REG,
				ADC_CONV_DLY_MASK,
				ADC_CONV_DLY_512MS);
	if (rc < 0)
		pr_err("Failed to reconfigure S7-delay rc=%d\n", rc);


	return 0;
}

static int qg_soh_batt_profile_init(struct qpnp_qg *chip)
{
#if defined(CONFIG_SOMC_CHARGER_EXTENSION)
	return 0;
#endif
#if !defined(CONFIG_SOMC_CHARGER_EXTENSION)
	int rc = 0;

	if (!chip->dt.multi_profile_load)
		return 0;

	if (is_debug_batt_id(chip) || chip->battery_missing)
		return 0;

	if (!chip->sp)
		chip->sp = devm_kzalloc(chip->dev, sizeof(*chip->sp),
					GFP_KERNEL);
	if (!chip->sp)
		return -ENOMEM;

	if (!chip->sp->initialized) {
		chip->sp->batt_id_kohms = chip->batt_id_ohm / 1000;
		chip->sp->bp_node = chip->batt_node;
		chip->sp->last_batt_age_level = chip->batt_age_level;
		chip->sp->bms_psy = chip->qg_psy;
		chip->sp->iio_chan_list = chip->int_iio_chans;
		rc = soh_profile_init(chip->dev, chip->sp);
		if (rc < 0)
			chip->sp = NULL;
		else
			qg_dbg(chip, QG_DEBUG_PROFILE, "SOH profile count: %d\n",
				chip->sp->profile_count);
	}

	return rc;
#endif
}

static int qg_post_init(struct qpnp_qg *chip)
{
	int rc = 0;

	/* disable all IRQs if profile is not loaded */
	if (!chip->profile_loaded) {
		vote(chip->vbatt_irq_disable_votable,
				PROFILE_IRQ_DISABLE, true, 0);
		vote(chip->fifo_irq_disable_votable,
				PROFILE_IRQ_DISABLE, true, 0);
		vote(chip->good_ocv_irq_disable_votable,
				PROFILE_IRQ_DISABLE, true, 0);
	}

	/* restore ESR data */
	if (!chip->dt.esr_disable)
		qg_retrieve_esr_params(chip);

	/*soh based multi profile init */
	rc = qg_soh_batt_profile_init(chip);
	if (rc < 0) {
		pr_err("Failed to initialize battery based on soh rc=%d\n",
								rc);
		return rc;
	}

	return 0;
}

static int qg_get_irq_index_byname(const char *irq_name)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(qg_irqs); i++) {
		if (strcmp(qg_irqs[i].name, irq_name) == 0)
			return i;
	}

	return -ENOENT;
}

static int qg_request_interrupt(struct qpnp_qg *chip,
		struct device_node *node, const char *irq_name)
{
	int rc, irq, irq_index;

	irq = of_irq_get_byname(node, irq_name);
	if (irq < 0) {
		pr_err("Failed to get irq %s byname\n", irq_name);
		return irq;
	}

	irq_index = qg_get_irq_index_byname(irq_name);
	if (irq_index < 0) {
		pr_err("%s is not a defined irq\n", irq_name);
		return irq_index;
	}

	if (!qg_irqs[irq_index].handler)
		return 0;

	rc = devm_request_threaded_irq(chip->dev, irq, NULL,
				qg_irqs[irq_index].handler,
				IRQF_ONESHOT, irq_name, chip);
	if (rc < 0) {
		pr_err("Failed to request irq %d\n", irq);
		return rc;
	}

	qg_irqs[irq_index].irq = irq;
	if (qg_irqs[irq_index].wake)
		enable_irq_wake(irq);

	qg_dbg(chip, QG_DEBUG_PON, "IRQ %s registered wakeable=%d\n",
			qg_irqs[irq_index].name, qg_irqs[irq_index].wake);

	return 0;
}

static int qg_request_irqs(struct qpnp_qg *chip)
{
	struct device_node *node = chip->dev->of_node;
	struct device_node *child;
	const char *name;
	struct property *prop;
	int rc = 0;

	for_each_available_child_of_node(node, child) {
		of_property_for_each_string(child, "interrupt-names",
					    prop, name) {
			rc = qg_request_interrupt(chip, child, name);
			if (rc < 0)
				return rc;
		}
	}


	return 0;
}

#define QG_TTF_ITERM_DELTA_MA		1
static int qg_alg_init(struct qpnp_qg *chip)
{
	struct cycle_counter *counter;
	struct cap_learning *cl;
	struct ttf *ttf;
	struct device_node *node = chip->dev->of_node;
	int rc;

	counter = devm_kzalloc(chip->dev, sizeof(*counter), GFP_KERNEL);
	if (!counter)
		return -ENOMEM;

	counter->restore_count = qg_restore_cycle_count;
	counter->store_count = qg_store_cycle_count;
	counter->data = chip;

	rc = cycle_count_init(counter);
	if (rc < 0) {
		dev_err(chip->dev, "Error in initializing cycle counter, rc:%d\n",
			rc);
		counter->data = NULL;
		return rc;
	}

	chip->counter = counter;

	ttf = devm_kzalloc(chip->dev, sizeof(*ttf), GFP_KERNEL);
	if (!ttf)
		return -ENOMEM;

	ttf->get_ttf_param = qg_get_ttf_param;
	ttf->awake_voter = qg_ttf_awake_voter;
	ttf->iterm_delta = QG_TTF_ITERM_DELTA_MA;
	ttf->data = chip;

	rc = ttf_tte_init(ttf);
	if (rc < 0) {
		dev_err(chip->dev, "Error in initializing ttf, rc:%d\n",
			rc);
		ttf->data = NULL;
		counter->data = NULL;
		return rc;
	}

	chip->ttf = ttf;

	chip->dt.cl_disable = of_property_read_bool(node,
					"qcom,cl-disable");

	/*Return if capacity learning is disabled*/
	if (chip->dt.cl_disable)
		return 0;

	cl = devm_kzalloc(chip->dev, sizeof(*cl), GFP_KERNEL);
	if (!cl)
		return -ENOMEM;

	cl->cc_soc_max = QG_SOC_FULL;
	cl->get_cc_soc = qg_get_cc_soc;
	cl->get_learned_capacity = qg_get_learned_capacity;
	cl->store_learned_capacity = qg_store_learned_capacity;
	cl->ok_to_begin = qg_cl_ok_to_begin;
#if defined(CONFIG_SOMC_CHARGER_EXTENSION) /* RID011075 Soft charge 5.1 */
	cl->get_monotonic_soc = qg_somc_get_monotonic_soc;
#endif
	cl->data = chip;

	rc = cap_learning_init(cl);
	if (rc < 0) {
		dev_err(chip->dev, "Error in initializing capacity learning, rc:%d\n",
			rc);
		counter->data = NULL;
		cl->data = NULL;
		return rc;
	}

	chip->cl = cl;
	return 0;
}

#ifdef CONFIG_DEBUG_FS
static void qg_create_debugfs(struct qpnp_qg *chip)
{
	struct dentry *entry;

	chip->dfs_root = debugfs_create_dir("qgauge", NULL);
	if (IS_ERR_OR_NULL(chip->dfs_root)) {
		pr_err("Failed to create debugfs directory rc=%ld\n",
				(long)chip->dfs_root);
		return;
	}

	entry = debugfs_create_u32("debug_mask", 0600, chip->dfs_root,
			&qg_debug_mask);
	if (IS_ERR_OR_NULL(entry)) {
		pr_err("Failed to create debug_mask rc=%ld\n", (long)entry);
		debugfs_remove_recursive(chip->dfs_root);
	}
}
#else
static void qg_create_debugfs(struct qpnp_qg *chip)
{
}
#endif

#define DEFAULT_S2_FIFO_LENGTH		5
#define DEFAULT_S2_VBAT_LOW_LENGTH	2
#define DEFAULT_S2_ACC_LENGTH		128
#define DEFAULT_S2_ACC_INTVL_MS		100
#define DEFAULT_SLEEP_S2_FIFO_LENGTH	8
#define DEFAULT_SLEEP_S2_ACC_LENGTH	256
#define DEFAULT_SLEEP_S2_ACC_INTVL_MS	200
#define DEFAULT_FAST_CHG_S2_FIFO_LENGTH	1
static int qg_parse_s2_dt(struct qpnp_qg *chip)
{
	int rc;
	struct device_node *node = chip->dev->of_node;
	u32 temp;

	/* S2 state params */
	rc = of_property_read_u32(node, "qcom,s2-fifo-length", &temp);
	if (rc < 0)
		chip->dt.s2_fifo_length = DEFAULT_S2_FIFO_LENGTH;
	else
		chip->dt.s2_fifo_length = temp;

	if (chip->dt.s2_fifo_length > chip->max_fifo_length) {
		pr_err("Invalid S2 fifo-length=%d max_length=%d\n",
					chip->dt.s2_fifo_length,
					chip->max_fifo_length);
		return -EINVAL;
	}

	rc = of_property_read_u32(node, "qcom,s2-vbat-low-fifo-length", &temp);
	if (rc < 0)
		chip->dt.s2_vbat_low_fifo_length = DEFAULT_S2_VBAT_LOW_LENGTH;
	else
		chip->dt.s2_vbat_low_fifo_length = temp;

	rc = of_property_read_u32(node, "qcom,s2-acc-length", &temp);
	if (rc < 0)
		chip->dt.s2_acc_length = DEFAULT_S2_ACC_LENGTH;
	else
		chip->dt.s2_acc_length = temp;

	rc = of_property_read_u32(node, "qcom,s2-acc-interval-ms", &temp);
	if (rc < 0)
		chip->dt.s2_acc_intvl_ms = DEFAULT_S2_ACC_INTVL_MS;
	else
		chip->dt.s2_acc_intvl_ms = temp;

	qg_dbg(chip, QG_DEBUG_PON, "DT: S2 FIFO length=%d low_vbat_length=%d acc_length=%d acc_interval=%d\n",
		chip->dt.s2_fifo_length, chip->dt.s2_vbat_low_fifo_length,
		chip->dt.s2_acc_length, chip->dt.s2_acc_intvl_ms);

	if (of_property_read_bool(node, "qcom,qg-sleep-config")) {

		chip->dt.qg_sleep_config = true;

		rc = of_property_read_u32(node,
				"qcom,sleep-s2-fifo-length", &temp);
		if (rc < 0)
			chip->dt.sleep_s2_fifo_length =
					DEFAULT_SLEEP_S2_FIFO_LENGTH;
		else
			chip->dt.sleep_s2_fifo_length = temp;

		if (chip->dt.s2_fifo_length > chip->max_fifo_length) {
			pr_err("Invalid S2 sleep-fifo-length=%d max_length=%d\n",
					chip->dt.sleep_s2_fifo_length,
					chip->max_fifo_length);
			return -EINVAL;
		}

		rc = of_property_read_u32(node,
				"qcom,sleep-s2-acc-length", &temp);
		if (rc < 0)
			chip->dt.sleep_s2_acc_length =
					DEFAULT_SLEEP_S2_ACC_LENGTH;
		else
			chip->dt.sleep_s2_acc_length = temp;

		rc = of_property_read_u32(node,
				"qcom,sleep-s2-acc-intvl-ms", &temp);
		if (rc < 0)
			chip->dt.sleep_s2_acc_intvl_ms =
					DEFAULT_SLEEP_S2_ACC_INTVL_MS;
		else
			chip->dt.sleep_s2_acc_intvl_ms = temp;
	}

	if (of_property_read_bool(node, "qcom,qg-fast-chg-config")) {

		chip->dt.qg_fast_chg_cfg = true;

		rc = of_property_read_u32(node,
				"qcom,fast-chg-s2-fifo-length", &temp);
		if (rc < 0)
			chip->dt.fast_chg_s2_fifo_length =
					DEFAULT_FAST_CHG_S2_FIFO_LENGTH;
		else
			chip->dt.fast_chg_s2_fifo_length = temp;

		if (chip->dt.fast_chg_s2_fifo_length > chip->max_fifo_length) {
			pr_err("Invalid S2 fast-fifo-length=%d max_length=%d\n",
					chip->dt.fast_chg_s2_fifo_length,
					chip->max_fifo_length);
			return -EINVAL;
		}
	}

	return 0;
}

#define DEFAULT_CL_MIN_START_SOC	10
#define DEFAULT_CL_MAX_START_SOC	40
#define DEFAULT_CL_MIN_TEMP_DECIDEGC	150
#define DEFAULT_CL_MAX_TEMP_DECIDEGC	500
#define DEFAULT_CL_MAX_INC_DECIPERC	10
#define DEFAULT_CL_MAX_DEC_DECIPERC	20
#define DEFAULT_CL_MIN_LIM_DECIPERC	500
#define DEFAULT_CL_MAX_LIM_DECIPERC	100
#define DEFAULT_CL_DELTA_BATT_SOC	10
#define DEFAULT_CL_WT_START_SOC		15
static int qg_parse_cl_dt(struct qpnp_qg *chip)
{
	int rc;
	struct device_node *node = chip->dev->of_node;
	u32 temp;

	if (chip->dt.cl_disable)
		return 0;

	chip->dt.cl_feedback_on = of_property_read_bool(node,
					"qcom,cl-feedback-on");

	rc = of_property_read_u32(node, "qcom,cl-min-start-soc", &temp);
	if (rc < 0)
		chip->cl->dt.min_start_soc = DEFAULT_CL_MIN_START_SOC;
	else
		chip->cl->dt.min_start_soc = temp;

	rc = of_property_read_u32(node, "qcom,cl-max-start-soc", &temp);
	if (rc < 0)
		chip->cl->dt.max_start_soc = DEFAULT_CL_MAX_START_SOC;
	else
		chip->cl->dt.max_start_soc = temp;

	rc = of_property_read_u32(node, "qcom,cl-min-temp", &temp);
	if (rc < 0)
		chip->cl->dt.min_temp = DEFAULT_CL_MIN_TEMP_DECIDEGC;
	else
		chip->cl->dt.min_temp = temp;

	rc = of_property_read_u32(node, "qcom,cl-max-temp", &temp);
	if (rc < 0)
		chip->cl->dt.max_temp = DEFAULT_CL_MAX_TEMP_DECIDEGC;
	else
		chip->cl->dt.max_temp = temp;

	rc = of_property_read_u32(node, "qcom,cl-max-increment", &temp);
	if (rc < 0)
		chip->cl->dt.max_cap_inc = DEFAULT_CL_MAX_INC_DECIPERC;
	else
		chip->cl->dt.max_cap_inc = temp;

	rc = of_property_read_u32(node, "qcom,cl-max-decrement", &temp);
	if (rc < 0)
		chip->cl->dt.max_cap_dec = DEFAULT_CL_MAX_DEC_DECIPERC;
	else
		chip->cl->dt.max_cap_dec = temp;

	rc = of_property_read_u32(node, "qcom,cl-min-limit", &temp);
	if (rc < 0)
		chip->cl->dt.min_cap_limit =
					DEFAULT_CL_MIN_LIM_DECIPERC;
	else
		chip->cl->dt.min_cap_limit = temp;

	rc = of_property_read_u32(node, "qcom,cl-max-limit", &temp);
	if (rc < 0)
		chip->cl->dt.max_cap_limit =
					DEFAULT_CL_MAX_LIM_DECIPERC;
	else
		chip->cl->dt.max_cap_limit = temp;

	chip->cl->dt.min_delta_batt_soc = DEFAULT_CL_DELTA_BATT_SOC;
	/* read from DT property and update, if value exists */
	of_property_read_u32(node, "qcom,cl-min-delta-batt-soc",
				&chip->cl->dt.min_delta_batt_soc);

	if (of_property_read_bool(node, "qcom,cl-wt-enable")) {
		chip->cl->dt.cl_wt_enable = true;
		chip->cl->dt.min_start_soc = DEFAULT_CL_WT_START_SOC;
		chip->cl->dt.max_start_soc = -EINVAL;
	}

	qg_dbg(chip, QG_DEBUG_PON, "DT: cl_min_start_soc=%d cl_max_start_soc=%d cl_min_temp=%d cl_max_temp=%d chip->cl->dt.cl_wt_enable=%d\n",
		chip->cl->dt.min_start_soc, chip->cl->dt.max_start_soc,
		chip->cl->dt.min_temp, chip->cl->dt.max_temp,
		chip->cl->dt.cl_wt_enable);

	return 0;
}

#define DEFAULT_VBATT_EMPTY_MV		3200
#define DEFAULT_VBATT_EMPTY_COLD_MV	3000
#define DEFAULT_VBATT_CUTOFF_MV		3400
#define DEFAULT_VBATT_LOW_MV		3500
#define DEFAULT_VBATT_LOW_COLD_MV	3800
#define DEFAULT_ITERM_MA		100
#define DEFAULT_DELTA_SOC		1
#define DEFAULT_SHUTDOWN_SOC_SECS	360
#define DEFAULT_COLD_TEMP_THRESHOLD	0
#define DEFAULT_SHUTDOWN_TEMP_DIFF	60	/* 6 degC */
#define DEFAULT_ESR_QUAL_CURRENT_UA	130000
#define DEFAULT_ESR_QUAL_VBAT_UV	7000
#define DEFAULT_ESR_DISABLE_SOC		1000
#define ESR_CHG_MIN_IBAT_UA		(-450000)
#define DEFAULT_SLEEP_TIME_SECS		1800 /* 30 mins */
#define DEFAULT_SYS_MIN_VOLT_MV		2800
#define DEFAULT_FVSS_VBAT_MV		3500
#define DEFAULT_TCSS_ENTRY_SOC		90
#define DEFAULT_ESR_LOW_TEMP_THRESHOLD	100 /* 10 deg */
#if defined(CONFIG_SOMC_CHARGER_EXTENSION)
#define DEFAULT_MAX_FV_MV		4480
#endif
static int qg_parse_dt(struct qpnp_qg *chip)
{
	int rc = 0;
	struct device_node *child, *node = chip->dev->of_node;
	u32 base, temp;
	u8 type;

	if (!node)  {
		pr_err("Failed to find device-tree node\n");
		return -ENXIO;
	}

	for_each_available_child_of_node(node, child) {
		rc = of_property_read_u32(child, "reg", &base);
		if (rc < 0) {
			pr_err("Failed to read base address, rc=%d\n", rc);
			return rc;
		}

		rc = qg_read(chip, base + PERPH_TYPE_REG, &type, 1);
		if (rc < 0) {
			pr_err("Failed to read type, rc=%d\n", rc);
			return rc;
		}

		switch (type) {
		case QG_TYPE:
			chip->qg_base = base;
			break;
		default:
			break;
		}
	}

	if (!chip->qg_base) {
		pr_err("QG device node missing\n");
		return -EINVAL;
	}

	rc = qg_parse_s2_dt(chip);
	if (rc < 0)
		pr_err("Failed to parse S2 DT params rc=%d\n", rc);

	rc = qg_parse_cl_dt(chip);
	if (rc < 0)
		pr_err("Failed to parse CL parameters rc=%d\n", rc);

	/* OCV params */
	rc = of_property_read_u32(node, "qcom,ocv-timer-expiry-min", &temp);
	if (rc < 0)
		chip->dt.ocv_timer_expiry_min = -EINVAL;
	else
		chip->dt.ocv_timer_expiry_min = temp;

	rc = of_property_read_u32(node, "qcom,ocv-tol-threshold-uv", &temp);
	if (rc < 0)
		chip->dt.ocv_tol_threshold_uv = -EINVAL;
	else
		chip->dt.ocv_tol_threshold_uv = temp;

	qg_dbg(chip, QG_DEBUG_PON, "DT: OCV timer_expiry =%dmin ocv_tol_threshold=%duV\n",
		chip->dt.ocv_timer_expiry_min, chip->dt.ocv_tol_threshold_uv);

	/* S3 sleep configuration */
	rc = of_property_read_u32(node, "qcom,s3-entry-fifo-length", &temp);
	if (rc < 0)
		chip->dt.s3_entry_fifo_length = -EINVAL;
	else
		chip->dt.s3_entry_fifo_length = temp;

	rc = of_property_read_u32(node, "qcom,s3-entry-ibat-ua", &temp);
	if (rc < 0)
		chip->dt.s3_entry_ibat_ua = -EINVAL;
	else
		chip->dt.s3_entry_ibat_ua = temp;

	rc = of_property_read_u32(node, "qcom,s3-exit-ibat-ua", &temp);
	if (rc < 0)
		chip->dt.s3_exit_ibat_ua = -EINVAL;
	else
		chip->dt.s3_exit_ibat_ua = temp;

	/* VBAT thresholds */
	rc = of_property_read_u32(node, "qcom,vbatt-empty-mv", &temp);
	if (rc < 0)
		chip->dt.vbatt_empty_mv = DEFAULT_VBATT_EMPTY_MV;
	else
		chip->dt.vbatt_empty_mv = temp;

	rc = of_property_read_u32(node, "qcom,vbatt-empty-cold-mv", &temp);
	if (rc < 0)
		chip->dt.vbatt_empty_cold_mv = DEFAULT_VBATT_EMPTY_COLD_MV;
	else
		chip->dt.vbatt_empty_cold_mv = temp;

	rc = of_property_read_u32(node, "qcom,cold-temp-threshold", &temp);
	if (rc < 0)
		chip->dt.cold_temp_threshold = DEFAULT_COLD_TEMP_THRESHOLD;
	else
		chip->dt.cold_temp_threshold = temp;

	rc = of_property_read_u32(node, "qcom,vbatt-low-mv", &temp);
	if (rc < 0)
		chip->dt.vbatt_low_mv = DEFAULT_VBATT_LOW_MV;
	else
		chip->dt.vbatt_low_mv = temp;

	rc = of_property_read_u32(node, "qcom,vbatt-low-cold-mv", &temp);
	if (rc < 0)
		chip->dt.vbatt_low_cold_mv = DEFAULT_VBATT_LOW_COLD_MV;
	else
		chip->dt.vbatt_low_cold_mv = temp;

	rc = of_property_read_u32(node, "qcom,vbatt-cutoff-mv", &temp);
	if (rc < 0)
		chip->dt.vbatt_cutoff_mv = DEFAULT_VBATT_CUTOFF_MV;
	else
		chip->dt.vbatt_cutoff_mv = temp;

	/* IBAT thresholds */
	rc = of_property_read_u32(node, "qcom,qg-iterm-ma", &temp);
	if (rc < 0)
		chip->dt.iterm_ma = DEFAULT_ITERM_MA;
	else
		chip->dt.iterm_ma = temp;

	rc = of_property_read_u32(node, "qcom,delta-soc", &temp);
	if (rc < 0)
		chip->dt.delta_soc = DEFAULT_DELTA_SOC;
	else
		chip->dt.delta_soc = temp;

	rc = of_property_read_u32(node, "qcom,ignore-shutdown-soc-secs", &temp);
	if (rc < 0)
		chip->dt.ignore_shutdown_soc_secs = DEFAULT_SHUTDOWN_SOC_SECS;
	else
		chip->dt.ignore_shutdown_soc_secs = temp;

	rc = of_property_read_u32(node, "qcom,shutdown-temp-diff", &temp);
	if (rc < 0)
		chip->dt.shutdown_temp_diff = DEFAULT_SHUTDOWN_TEMP_DIFF;
	else
		chip->dt.shutdown_temp_diff = temp;

	chip->dt.hold_soc_while_full = of_property_read_bool(node,
					"qcom,hold-soc-while-full");

	chip->dt.linearize_soc = of_property_read_bool(node,
					"qcom,linearize-soc");

	rc = of_property_read_u32(node, "qcom,rbat-conn-mohm", &temp);
	if (rc < 0)
		chip->dt.rbat_conn_mohm = 0;
	else
		chip->dt.rbat_conn_mohm = (int)temp;

	/* esr */
	chip->dt.esr_disable = of_property_read_bool(node,
					"qcom,esr-disable");

	chip->dt.esr_discharge_enable = of_property_read_bool(node,
					"qcom,esr-discharge-enable");

	rc = of_property_read_u32(node, "qcom,esr-qual-current-ua", &temp);
	if (rc < 0)
		chip->dt.esr_qual_i_ua = DEFAULT_ESR_QUAL_CURRENT_UA;
	else
		chip->dt.esr_qual_i_ua = temp;

	rc = of_property_read_u32(node, "qcom,esr-qual-vbatt-uv", &temp);
	if (rc < 0)
		chip->dt.esr_qual_v_uv = DEFAULT_ESR_QUAL_VBAT_UV;
	else
		chip->dt.esr_qual_v_uv = temp;

	rc = of_property_read_u32(node, "qcom,esr-disable-soc", &temp);
	if (rc < 0)
		chip->dt.esr_disable_soc = DEFAULT_ESR_DISABLE_SOC;
	else
		chip->dt.esr_disable_soc = temp * 100;

	rc = of_property_read_u32(node, "qcom,esr-chg-min-ibat-ua", &temp);
	if (rc < 0)
		chip->dt.esr_min_ibat_ua = ESR_CHG_MIN_IBAT_UA;
	else
		chip->dt.esr_min_ibat_ua = (int)temp;

	rc = of_property_read_u32(node, "qcom,esr-low-temp-threshold", &temp);
	if (rc < 0)
		chip->dt.esr_low_temp_threshold =
					DEFAULT_ESR_LOW_TEMP_THRESHOLD;
	else
		chip->dt.esr_low_temp_threshold = (int)temp;

	rc = of_property_read_u32(node, "qcom,shutdown_soc_threshold", &temp);
	if (rc < 0)
		chip->dt.shutdown_soc_threshold = -EINVAL;
	else
		chip->dt.shutdown_soc_threshold = temp;

	rc = of_property_read_u32(node, "qcom,qg-sys-min-voltage", &temp);
	if (rc < 0)
		chip->dt.sys_min_volt_mv = DEFAULT_SYS_MIN_VOLT_MV;
	else
		chip->dt.sys_min_volt_mv = temp;

	chip->dt.qg_ext_sense = of_property_read_bool(node, "qcom,qg-ext-sns");

	chip->dt.use_cp_iin_sns = of_property_read_bool(node,
							"qcom,use-cp-iin-sns");

	chip->dt.use_s7_ocv = of_property_read_bool(node, "qcom,qg-use-s7-ocv");

	rc = of_property_read_u32(node, "qcom,min-sleep-time-secs", &temp);
	if (rc < 0)
		chip->dt.min_sleep_time_secs = DEFAULT_SLEEP_TIME_SECS;
	else
		chip->dt.min_sleep_time_secs = temp;

	if (of_property_read_bool(node, "qcom,fvss-enable")) {

		chip->dt.fvss_enable = true;

		rc = of_property_read_u32(node,
				"qcom,fvss-vbatt-mv", &temp);
		if (rc < 0)
			chip->dt.fvss_vbat_mv = DEFAULT_FVSS_VBAT_MV;
		else
			chip->dt.fvss_vbat_mv = temp;
	}

	if (of_property_read_bool(node, "qcom,tcss-enable")) {

		chip->dt.tcss_enable = true;

		rc = of_property_read_u32(node,
				"qcom,tcss-entry-soc", &temp);
		if (rc < 0)
			chip->dt.tcss_entry_soc = DEFAULT_TCSS_ENTRY_SOC;
		else
			chip->dt.tcss_entry_soc = temp;
	}

	chip->dt.bass_enable = of_property_read_bool(node, "qcom,bass-enable");

	chip->dt.multi_profile_load = of_property_read_bool(node,
					"qcom,multi-profile-load");

#if defined(CONFIG_SOMC_CHARGER_EXTENSION)
	chip->use_real_temp = of_property_read_bool(node,
					"somc,jeita-step-use-real-temp");
	if (chip->use_real_temp) {
		rc = of_property_read_u32(node,
					  "somc,jeita-aux-temp-correction",
					  &temp);
		if (rc < 0)
			chip->aux_temp_correctton = 0;
		else
			chip->aux_temp_correctton = temp;

		chip->real_temp_use_aux = (rc < 0) ? false : true;

		rc = of_property_read_u32(node,
					  "somc,jeita-batt-temp-correction",
					  &temp);
		if (rc < 0)
			chip->batt_temp_correctton = 0;
		else
			chip->batt_temp_correctton = temp;
	}

	rc = of_property_read_u32(node, "somc,ibat-full-term-diff", &temp);
	if (rc < 0)
		chip->ibat_full_term_diff = 0;
	else
		chip->ibat_full_term_diff = temp;

	rc = of_property_read_u32(node, "somc,fv-max-mv", &temp);
	if (rc < 0)
		chip->product_max_fv = DEFAULT_MAX_FV_MV;
	else
		chip->product_max_fv = temp;
#endif

	qg_dbg(chip, QG_DEBUG_PON, "DT: vbatt_empty_mv=%dmV vbatt_low_mv=%dmV delta_soc=%d ext-sns=%d\n",
			chip->dt.vbatt_empty_mv, chip->dt.vbatt_low_mv,
			chip->dt.delta_soc, chip->dt.qg_ext_sense);

	return 0;
}

#if defined(CONFIG_SOMC_CHARGER_EXTENSION)
/*****************************
 * somc sysfs implementation *
 *****************************/
enum qg_somc_sysfs {
	ATTR_REG_QGAUGE_STATUS1 = 0,
	ATTR_REG_QGAUGE_STATUS2,
	ATTR_REG_QGAUGE_STATUS4,
	ATTR_REG_QGAUGE_INT_RT_STS,
	ATTR_CC_SOC,
	ATTR_BATT_SOC,
	ATTR_SYS_SOC,
	ATTR_CATCH_UP_SOC,
	ATTR_MAINT_SOC,
	ATTR_MSOC,
	ATTR_PON_SOC,
	ATTR_FULL_SOC,
	ATTR_RECHARGE_SOC,
	ATTR_CHARGE_FULL,
	ATTR_VCELL_MAX,
	ATTR_CL_ACTIVE,
	ATTR_CL_BSOC_DROP,
	ATTR_CL_CCSOC_DROP,
	ATTR_CL_HOLD_TIME,
	ATTR_CL_TOTAL_TIME,
	ATTR_NOM_CAP,
	ATTR_PSY_CHG_COUNTER,
};

static ssize_t qg_somc_param_show(struct device *dev,
				struct device_attribute *attr, char *buf);

static struct device_attribute qg_somc_attrs[] = {
	__ATTR(somc_reg_qgauge_status1, 0444, qg_somc_param_show, NULL),
	__ATTR(somc_reg_qgauge_status2, 0444, qg_somc_param_show, NULL),
	__ATTR(somc_reg_qgauge_status4, 0444, qg_somc_param_show, NULL),
	__ATTR(somc_reg_qgauge_int_rt_sts, 0444, qg_somc_param_show, NULL),
	__ATTR(somc_cc_soc, 0444, qg_somc_param_show, NULL),
	__ATTR(somc_batt_soc, 0444, qg_somc_param_show, NULL),
	__ATTR(somc_sys_soc, 0444, qg_somc_param_show, NULL),
	__ATTR(somc_catch_up_soc, 0444, qg_somc_param_show, NULL),
	__ATTR(somc_maint_soc, 0444, qg_somc_param_show, NULL),
	__ATTR(somc_msoc, 0444, qg_somc_param_show, NULL),
	__ATTR(somc_pon_soc, 0444, qg_somc_param_show, NULL),
	__ATTR(somc_full_soc, 0444, qg_somc_param_show, NULL),
	__ATTR(somc_recharge_soc, 0444, qg_somc_param_show, NULL),
	__ATTR(somc_charge_full, 0444, qg_somc_param_show, NULL),
	__ATTR(somc_vcell_max, 0444, qg_somc_param_show, NULL),
	__ATTR(somc_cl_active, 0444, qg_somc_param_show, NULL),
	__ATTR(somc_cl_bsoc_drop, 0444, qg_somc_param_show, NULL),
	__ATTR(somc_cl_ccsoc_drop, 0444, qg_somc_param_show, NULL),
	__ATTR(somc_cl_hold_time, 0444, qg_somc_param_show, NULL),
	__ATTR(somc_cl_total_time, 0444, qg_somc_param_show, NULL),
	__ATTR(somc_nom_cap, 0444, qg_somc_param_show, NULL),
	__ATTR(somc_psy_chg_counter, 0444, qg_somc_param_show, NULL),
};

static ssize_t qg_somc_param_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct qpnp_qg *chip = dev_get_drvdata(dev);
	ssize_t size = 0;
	const ptrdiff_t off = attr - qg_somc_attrs;
	int rc = 0;
	u8 reg;
	int val;

	switch (off) {
	case ATTR_REG_QGAUGE_STATUS1:
		rc = qg_read(chip, chip->qg_base + QG_STATUS1_REG, &reg, 1);
		if (rc < 0) {
			pr_err("Failed to read STATUS1_REG rc=%d\n", rc);
			return rc;
		}
		size = scnprintf(buf, PAGE_SIZE, "%02X\n", reg);
		break;
	case ATTR_REG_QGAUGE_STATUS2:
		rc = qg_read(chip, chip->qg_base + QG_STATUS2_REG, &reg, 1);
		if (rc < 0) {
			pr_err("Failed to read STATUS2_REG rc=%d\n", rc);
			return rc;
		}
		size = scnprintf(buf, PAGE_SIZE, "%02X\n", reg);
		break;
	case ATTR_REG_QGAUGE_STATUS4:
		rc = qg_read(chip, chip->qg_base + QG_STATUS4_REG, &reg, 1);
		if (rc < 0) {
			pr_err("Failed to read STATUS4_REG rc=%d\n", rc);
			return rc;
		}
		size = scnprintf(buf, PAGE_SIZE, "%02X\n", reg);
		break;
	case ATTR_REG_QGAUGE_INT_RT_STS:
		rc = qg_read(chip, chip->qg_base + QG_INT_RT_STS_REG, &reg, 1);
		if (rc < 0) {
			pr_err("Failed to read RT status, rc=%d\n", rc);
			return rc;
		}
		size = scnprintf(buf, PAGE_SIZE, "%02X\n", reg);
		break;
	case ATTR_CC_SOC:
		size = scnprintf(buf, PAGE_SIZE, "%s%d.%02d\n",
					chip->cc_soc < 0 ? "-" : "",
					chip->cc_soc / 100,
					abs(chip->cc_soc % 100));
		break;
	case ATTR_BATT_SOC:
		size = scnprintf(buf, PAGE_SIZE, "%s%d.%02d\n",
					chip->batt_soc < 0 ? "-" : "",
					chip->batt_soc / 100,
					abs(chip->batt_soc % 100));
		break;
	case ATTR_SYS_SOC:
		size = scnprintf(buf, PAGE_SIZE, "%s%d.%02d\n",
					chip->sys_soc < 0 ? "-" : "",
					chip->sys_soc / 100,
					abs(chip->sys_soc % 100));
		break;
	case ATTR_CATCH_UP_SOC:
		size = scnprintf(buf, PAGE_SIZE, "%d\n", chip->catch_up_soc);
		break;
	case ATTR_MAINT_SOC:
		size = scnprintf(buf, PAGE_SIZE, "%d\n", chip->maint_soc);
		break;
	case ATTR_MSOC:
		size = scnprintf(buf, PAGE_SIZE, "%d\n", chip->msoc);
		break;
	case ATTR_PON_SOC:
		size = scnprintf(buf, PAGE_SIZE, "%d\n", chip->pon_soc);
		break;
	case ATTR_FULL_SOC:
		size = scnprintf(buf, PAGE_SIZE, "%s%d.%02d\n",
					chip->full_soc < 0 ? "-" : "",
					chip->full_soc / 100,
					abs(chip->full_soc % 100));
		break;
	case ATTR_RECHARGE_SOC:
		size = scnprintf(buf, PAGE_SIZE, "%d\n", chip->recharge_soc);
		break;
	case ATTR_CHARGE_FULL:
		size = scnprintf(buf, PAGE_SIZE, "%d\n", chip->charge_full);
		break;
	case ATTR_VCELL_MAX:
		size = scnprintf(buf, PAGE_SIZE, "%d\n", chip->vcell_max_mv);
		break;
	case ATTR_CL_ACTIVE:
		size = scnprintf(buf, PAGE_SIZE, "%d\n", (int)chip->cl->active);
		break;
	case ATTR_CL_BSOC_DROP:
		val = chip->cl->batt_soc_cp_drop;
		if (val >= 0) {
			size = scnprintf(buf, PAGE_SIZE, "%d.%02d\n",
							val / 100, val % 100);
		} else {
			size = scnprintf(buf, PAGE_SIZE, "-%d.%02d\n",
					abs(val / 100), abs(val % 100));
		}
		break;
	case ATTR_CL_CCSOC_DROP:
		val = (int)chip->cl->cc_soc_drop;
		if (val >= 0) {
			size = scnprintf(buf, PAGE_SIZE, "%d.%02d\n",
							val / 100, val % 100);
		} else {
			size = scnprintf(buf, PAGE_SIZE, "-%d.%02d\n",
					abs(val / 100), abs(val % 100));
		}
		break;
	case ATTR_CL_HOLD_TIME:
		size = scnprintf(buf, PAGE_SIZE, "%d\n",
					(int)(chip->cl->hold_time / 1000));
		break;
	case ATTR_CL_TOTAL_TIME:
		size = scnprintf(buf, PAGE_SIZE, "%d\n",
					(int)(chip->cl->total_time / 1000));
		break;
	case ATTR_NOM_CAP:
		rc = qg_get_nominal_capacity(&val, 250, true);
		if (!rc)
			size = scnprintf(buf, PAGE_SIZE, "%d\n", val);
		break;
	case ATTR_PSY_CHG_COUNTER:
		size = scnprintf(buf, PAGE_SIZE, "%d\n", chip->psy_chg_counter);
		break;
	default:
		size = 0;
		break;
	}
	return size;
}

static int qg_somc_create_sysfs_entries(struct device *dev)
{
	int i;
	int rc = 0;

	for (i = 0; i < ARRAY_SIZE(qg_somc_attrs); i++) {
		rc = device_create_file(dev, &qg_somc_attrs[i]);
		if (rc < 0) {
			dev_err(dev, "device_create_file failed rc = %d\n", rc);
			goto revert;
		}
	}
	return 0;
revert:
	for (i = i - 1; i >= 0; i--)
		device_remove_file(dev, &qg_somc_attrs[i]);
	return rc;
}

static void qg_somc_remove_sysfs_entries(struct device *dev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(qg_somc_attrs); i++)
		device_remove_file(dev, &qg_somc_attrs[i]);
}

#endif

static int process_suspend(struct qpnp_qg *chip)
{
	u8 status = 0;
	int rc;
	u32 fifo_rt_length = 0, sleep_fifo_length = 0;

	/* skip if profile is not loaded */
	if (!chip->profile_loaded)
		return 0;

	cancel_delayed_work_sync(&chip->ttf->ttf_work);

	chip->suspend_data = false;

	/* read STATUS2 register to clear its last state */
	qg_read(chip, chip->qg_base + QG_STATUS2_REG, &status, 1);

	/* ignore any suspend processing if we are charging */
	if (chip->charge_status == POWER_SUPPLY_STATUS_CHARGING) {
		/* Reset the sleep config if we are charging */
		if (chip->dt.qg_sleep_config) {
			qg_dbg(chip, QG_DEBUG_STATUS, "Suspend: Charging - Exit S2_SLEEP\n");
			rc = qg_config_s2_state(chip, S2_SLEEP, false, true);
			if (rc < 0)
				pr_err("Failed to exit S2-sleep rc=%d\n", rc);
		}
		qg_dbg(chip, QG_DEBUG_PM, "Charging @ suspend - ignore processing\n");
		return 0;
	}

	rc = get_fifo_length(chip, &fifo_rt_length, true);
	if (rc < 0) {
		pr_err("Failed to read FIFO RT count, rc=%d\n", rc);
		return rc;
	}

	rc = qg_read(chip, chip->qg_base + QG_S3_SLEEP_OCV_IBAT_CTL1_REG,
			(u8 *)&sleep_fifo_length, 1);
	if (rc < 0) {
		pr_err("Failed to read sleep FIFO count, rc=%d\n", rc);
		return rc;
	}
	sleep_fifo_length &= SLEEP_IBAT_QUALIFIED_LENGTH_MASK;
	sleep_fifo_length++;

	if (chip->dt.qg_sleep_config) {
		qg_dbg(chip, QG_DEBUG_STATUS, "Suspend: Forcing S2_SLEEP\n");
		rc = qg_config_s2_state(chip, S2_SLEEP, true, true);
		if (rc < 0)
			pr_err("Failed to config S2_SLEEP rc=%d\n", rc);
		if (chip->kdata.fifo_length > 0)
			chip->suspend_data = true;
	} else if (fifo_rt_length >=
			(chip->dt.s2_fifo_length - sleep_fifo_length)) {
		/*
		 * If the real-time FIFO count is greater than
		 * the the #fifo to enter sleep, save the FIFO data
		 * and reset the fifo count. This is avoid a gauranteed wakeup
		 * due to fifo_done event as the curent FIFO length is already
		 * beyond the sleep length.
		 */
		rc = qg_master_hold(chip, true);
		if (rc < 0) {
			pr_err("Failed to hold master, rc=%d\n", rc);
			return rc;
		}

		rc = qg_process_rt_fifo(chip);
		if (rc < 0) {
			pr_err("Failed to process FIFO real-time, rc=%d\n", rc);
			qg_master_hold(chip, false);
			return rc;
		}

		rc = qg_master_hold(chip, false);
		if (rc < 0) {
			pr_err("Failed to release master, rc=%d\n", rc);
			return rc;
		}
		/* FIFOs restarted */
		chip->last_fifo_update_time = ktime_get_boottime();

		chip->suspend_data = true;
	}

	get_rtc_time(&chip->suspend_time);

	qg_dbg(chip, QG_DEBUG_PM, "FIFO rt_length=%d sleep_fifo_length=%d default_s2_count=%d suspend_data=%d time=%d\n",
			fifo_rt_length, sleep_fifo_length,
			chip->dt.s2_fifo_length, chip->suspend_data,
			chip->suspend_time);

	return rc;
}

#define QG_SLEEP_EXIT_TIME_MS		15000 /* 15 secs */
static int process_resume(struct qpnp_qg *chip)
{
	u8 status2 = 0, rt_status = 0;
	u32 ocv_uv = 0, ocv_raw = 0;
	int rc;
	unsigned long rtc_sec = 0, sleep_time_secs = 0;

	/* skip if profile is not loaded */
	if (!chip->profile_loaded)
		return 0;

	get_rtc_time(&rtc_sec);
	sleep_time_secs = rtc_sec - chip->suspend_time;

	if (chip->dt.qg_sleep_config)
		schedule_delayed_work(&chip->qg_sleep_exit_work,
				msecs_to_jiffies(QG_SLEEP_EXIT_TIME_MS));

	rc = qg_read(chip, chip->qg_base + QG_STATUS2_REG, &status2, 1);
	if (rc < 0) {
		pr_err("Failed to read status2 register, rc=%d\n", rc);
		return rc;
	}

	if (status2 & GOOD_OCV_BIT) {
		rc = qg_read_ocv(chip, &ocv_uv, &ocv_raw, S3_GOOD_OCV);
		if (rc < 0) {
			pr_err("Failed to read good_ocv, rc=%d\n", rc);
			return rc;
		}

		 /* Clear suspend data as there has been a GOOD OCV */
		memset(&chip->kdata, 0, sizeof(chip->kdata));
		chip->kdata.fifo_time = (u32)rtc_sec;
		chip->kdata.param[QG_GOOD_OCV_UV].data = ocv_uv;
		chip->kdata.param[QG_GOOD_OCV_UV].valid = true;
		chip->suspend_data = false;

		/* allow SOC jump if we have slept longer */
		if (sleep_time_secs >= chip->dt.min_sleep_time_secs)
			chip->force_soc = true;

		qg_dbg(chip, QG_DEBUG_PM, "GOOD OCV @ resume good_ocv=%d uV\n",
				ocv_uv);
	}

	rc = qg_read(chip, chip->qg_base + QG_INT_LATCHED_STS_REG,
						&rt_status, 1);
	if (rc < 0) {
		pr_err("Failed to read latched status register, rc=%d\n", rc);
		return rc;
	}
	rt_status &= FIFO_UPDATE_DONE_INT_LAT_STS_BIT;

	qg_dbg(chip, QG_DEBUG_PM, "FIFO_DONE_STS=%d suspend_data=%d good_ocv=%d sleep_time=%d secs\n",
				!!rt_status, chip->suspend_data,
				chip->kdata.param[QG_GOOD_OCV_UV].valid,
				sleep_time_secs);
	/*
	 * If this is not a wakeup from FIFO-done,
	 * process the data immediately if - we have data from
	 * suspend or there is a good OCV.
	 */
	if (!rt_status && (chip->suspend_data ||
			chip->kdata.param[QG_GOOD_OCV_UV].valid)) {
		vote(chip->awake_votable, SUSPEND_DATA_VOTER, true, 0);
		/* signal the read thread */
		chip->data_ready = true;
		wake_up_interruptible(&chip->qg_wait_q);
		chip->suspend_data = false;
	}

	schedule_delayed_work(&chip->ttf->ttf_work, 0);

	return rc;
}

static int qpnp_qg_suspend_noirq(struct device *dev)
{
	int rc;
	struct qpnp_qg *chip = dev_get_drvdata(dev);

	/* cancel any pending sleep_exit work */
	cancel_delayed_work_sync(&chip->qg_sleep_exit_work);

	mutex_lock(&chip->data_lock);

	rc = process_suspend(chip);
	if (rc < 0)
		pr_err("Failed to process QG suspend, rc=%d\n", rc);

	mutex_unlock(&chip->data_lock);

	return 0;
}

static int qpnp_qg_resume_noirq(struct device *dev)
{
	int rc;
	struct qpnp_qg *chip = dev_get_drvdata(dev);

	mutex_lock(&chip->data_lock);

	rc = process_resume(chip);
	if (rc < 0)
		pr_err("Failed to process QG resume, rc=%d\n", rc);

	mutex_unlock(&chip->data_lock);

	return 0;
}

static int qpnp_qg_suspend(struct device *dev)
{
	struct qpnp_qg *chip = dev_get_drvdata(dev);

	/* skip if profile is not loaded */
	if (!chip->profile_loaded)
		return 0;

	/* disable GOOD_OCV IRQ in sleep */
	vote(chip->good_ocv_irq_disable_votable,
			QG_INIT_STATE_IRQ_DISABLE, true, 0);

	return 0;
}

static int qpnp_qg_resume(struct device *dev)
{
	struct qpnp_qg *chip = dev_get_drvdata(dev);

	/* skip if profile is not loaded */
	if (!chip->profile_loaded)
		return 0;

	/* enable GOOD_OCV IRQ when active */
	vote(chip->good_ocv_irq_disable_votable,
			QG_INIT_STATE_IRQ_DISABLE, false, 0);

	return 0;
}

static const struct dev_pm_ops qpnp_qg_pm_ops = {
	.suspend_noirq	= qpnp_qg_suspend_noirq,
	.resume_noirq	= qpnp_qg_resume_noirq,
	.suspend	= qpnp_qg_suspend,
	.resume		= qpnp_qg_resume,
};

static int qpnp_qg_probe(struct platform_device *pdev)
{
	int rc = 0, soc = 0, nom_cap_uah;
	struct qpnp_qg *chip;
	struct iio_dev *indio_dev;
	struct qg_config *config;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*chip));
	if (!indio_dev)
		return -ENOMEM;

	chip = iio_priv(indio_dev);
	chip->indio_dev = indio_dev;

	chip->regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!chip->regmap) {
		pr_err("Parent regmap is unavailable\n");
		return -ENXIO;
	}
	if (!alarmtimer_get_rtcdev()) {
		pr_err("Failed to get soc alarm-timer, probe later\n");
		return -EPROBE_DEFER;
	}
	/* ADC for BID & THERM */
	chip->batt_id_chan = iio_channel_get(&pdev->dev, "batt-id");
	if (IS_ERR(chip->batt_id_chan)) {
		rc = PTR_ERR(chip->batt_id_chan);
		if (rc != -EPROBE_DEFER)
			pr_err("batt-id channel unavailable, rc=%d\n", rc);
		chip->batt_id_chan = NULL;
		return rc;
	}

	chip->batt_therm_chan = iio_channel_get(&pdev->dev, "batt-therm");
	if (IS_ERR(chip->batt_therm_chan)) {
		rc = PTR_ERR(chip->batt_therm_chan);
		if (rc != -EPROBE_DEFER)
			pr_err("batt-therm channel unavailable, rc=%d\n", rc);
		chip->batt_therm_chan = NULL;
		return rc;
	}
#if defined(CONFIG_SOMC_CHARGER_EXTENSION)
	chip->aux_temp_chan = iio_channel_get(&pdev->dev, "aux-temp");
	if (IS_ERR(chip->aux_temp_chan)) {
		rc = PTR_ERR(chip->aux_temp_chan);
		if (rc != -EPROBE_DEFER)
			pr_err("aux-temp channel unavailable, rc=%d\n", rc);
		chip->aux_temp_chan = NULL;
	}

#endif
	chip->dev = &pdev->dev;
	chip->debug_mask = &qg_debug_mask;
	platform_set_drvdata(pdev, chip);
	INIT_WORK(&chip->udata_work, process_udata_work);
	INIT_WORK(&chip->qg_status_change_work, qg_status_change_work);
	INIT_DELAYED_WORK(&chip->qg_sleep_exit_work, qg_sleep_exit_work);
#if defined(CONFIG_SOMC_CHARGER_EXTENSION)
	INIT_DELAYED_WORK(&chip->somc_jeita_step_charge_work,
					qg_somc_jeita_step_charge_work);
	INIT_WORK(&chip->psy_chg_work, qg_somc_psy_chg_work);
	if (alarmtimer_get_rtcdev()) {
		alarm_init(&chip->psy_chg_alarm_timer,
			ALARM_BOOTTIME, qg_somc_psy_chg_alarm_timer);
	} else {
		pr_err("Error in alarm_init, rc:%d\n", rc);
		return rc;
	}

	//wakeup_source_init(&chip->step_ws, "somc_jeita_step");
	chip->step_ws = wakeup_source_register(chip->dev, "somc_jeita_step");
#endif
	mutex_init(&chip->bus_lock);
	mutex_init(&chip->soc_lock);
	mutex_init(&chip->data_lock);
#if defined(CONFIG_SOMC_CHARGER_EXTENSION)
	mutex_init(&chip->step_lock);
#endif
	init_waitqueue_head(&chip->qg_wait_q);
	chip->maint_soc = -EINVAL;
	chip->batt_soc = INT_MIN;
	chip->cc_soc = INT_MIN;
	chip->sys_soc = INT_MIN;
	chip->full_soc = QG_SOC_FULL;
	chip->chg_iterm_ma = INT_MIN;
	chip->soh = -EINVAL;
	chip->esr_actual = -EINVAL;
	chip->esr_nominal = -EINVAL;
	chip->batt_age_level = -EINVAL;
#if defined(CONFIG_SOMC_CHARGER_EXTENSION)
	chip->real_temp_debug = -EINVAL;
#endif

	config = (struct qg_config *)of_device_get_match_data(
							&pdev->dev);
	if (!config) {
		pr_err("Failed to get QG config data\n");
		return -EINVAL;
	}

	chip->qg_version = config->qg_version;
	chip->pmic_version = config->pmic_version;

	qg_dbg(chip, QG_DEBUG_PON, "QG version:%d PMIC version:%d",
		chip->qg_version, chip->pmic_version);

	switch (chip->qg_version) {
	case QG_LITE:
		chip->max_fifo_length = 5;
		break;
	default:
		chip->max_fifo_length = 8;
		break;
	}

	qg_create_debugfs(chip);

	rc = qg_alg_init(chip);
	if (rc < 0) {
		pr_err("Error in alg_init, rc:%d\n", rc);
		return rc;
	}

	rc = qg_parse_dt(chip);
	if (rc < 0) {
		pr_err("Failed to parse DT, rc=%d\n", rc);
		return rc;
	}

	rc = qg_hw_init(chip);
	if (rc < 0) {
		pr_err("Failed to hw_init, rc=%d\n", rc);
		return rc;
	}

	rc = qg_sdam_init(chip->dev);
	if (rc < 0) {
		pr_err("Failed to initialize QG SDAM, rc=%d\n", rc);
		return rc;
	}

	rc = qg_setup_battery(chip);
	if (rc < 0) {
		pr_err("Failed to setup battery, rc=%d\n", rc);
		return rc;
	}

	rc = qg_register_device(chip);
	if (rc < 0) {
		pr_err("Failed to register QG char device, rc=%d\n", rc);
		return rc;
	}

	rc = qg_sanitize_sdam(chip);
	if (rc < 0) {
		pr_err("Failed to sanitize SDAM, rc=%d\n", rc);
		return rc;
	}

	rc = qg_soc_init(chip);
	if (rc < 0) {
		pr_err("Failed to initialize SOC scaling init rc=%d\n", rc);
		return rc;
	}

	if (chip->profile_loaded) {
		if (!chip->dt.cl_disable) {
			/*
			 * Use FCC @ 25 C and charge-profile for
			 * Nominal Capacity
			 */
			rc = qg_get_nominal_capacity(&nom_cap_uah, 250, true);
			if (!rc) {
				rc = cap_learning_post_profile_init(chip->cl,
						nom_cap_uah);
				if (rc < 0) {
					pr_err("Error in cap_learning_post_profile_init rc=%d\n",
						rc);
					return rc;
				}
			}
		}
		rc = restore_cycle_count(chip->counter);
		if (rc < 0) {
			pr_err("Error in restoring cycle_count, rc=%d\n", rc);
			return rc;
		}
		schedule_delayed_work(&chip->ttf->ttf_work, 10000);
	}

	rc = qg_determine_pon_soc(chip);
	if (rc < 0) {
		pr_err("Failed to determine initial state, rc=%d\n", rc);
		goto fail_device;
	}

	chip->awake_votable = create_votable("QG_WS", VOTE_SET_ANY,
					 qg_awake_cb, chip);
	if (IS_ERR(chip->awake_votable)) {
		rc = PTR_ERR(chip->awake_votable);
		chip->awake_votable = NULL;
		goto fail_device;
	}

	chip->vbatt_irq_disable_votable = create_votable("QG_VBATT_IRQ_DISABLE",
				VOTE_SET_ANY, qg_vbatt_irq_disable_cb, chip);
	if (IS_ERR(chip->vbatt_irq_disable_votable)) {
		rc = PTR_ERR(chip->vbatt_irq_disable_votable);
		chip->vbatt_irq_disable_votable = NULL;
		goto fail_device;
	}

	chip->fifo_irq_disable_votable = create_votable("QG_FIFO_IRQ_DISABLE",
				VOTE_SET_ANY, qg_fifo_irq_disable_cb, chip);
	if (IS_ERR(chip->fifo_irq_disable_votable)) {
		rc = PTR_ERR(chip->fifo_irq_disable_votable);
		chip->fifo_irq_disable_votable = NULL;
		goto fail_device;
	}

	chip->good_ocv_irq_disable_votable =
			create_votable("QG_GOOD_IRQ_DISABLE",
			VOTE_SET_ANY, qg_good_ocv_irq_disable_cb, chip);
	if (IS_ERR(chip->good_ocv_irq_disable_votable)) {
		rc = PTR_ERR(chip->good_ocv_irq_disable_votable);
		chip->good_ocv_irq_disable_votable = NULL;
		goto fail_device;
	}

	rc = qg_init_iio_psy(chip, pdev);
	if (rc < 0) {
		pr_err("Failed to initialize QG IIO PSY, rc=%d\n", rc);
		goto fail_votable;
	}

	rc = qg_init_psy(chip);
	if (rc < 0) {
		pr_err("Failed to initialize QG PSY, rc=%d\n", rc);
		goto fail_votable;
	}
#if defined(CONFIG_SOMC_CHARGER_EXTENSION)
	rc = qg_somc_create_sysfs_entries(chip->dev);
	if (rc < 0) {
		dev_err(chip->dev,
			"Error in creating qg_somc_sysfs entries, rc:%d\n", rc);
		goto fail_votable;
	}
	alarm_start_relative(&chip->psy_chg_alarm_timer,
				ms_to_ktime(PSY_CHG_ALARM_FIRST_TIME_MS));
#endif
	rc = qg_request_irqs(chip);
	if (rc < 0) {
		pr_err("Failed to register QG interrupts, rc=%d\n", rc);
		goto fail_votable;
	}

	rc = qg_post_init(chip);
	if (rc < 0) {
		pr_err("Failed in qg_post_init rc=%d\n", rc);
		goto fail_votable;
	}

	rc = sysfs_create_groups(&chip->dev->kobj, qg_groups);
	if (rc < 0) {
		pr_err("Failed to create sysfs files rc=%d\n", rc);
		goto fail_votable;
	}

	qg_get_battery_capacity(chip, &soc);

	pr_info("QG initialized! battery_profile=%s SOC=%d QG_subtype=%d QG_version=%s QG_mode=%s\n",
			qg_get_battery_type(chip), soc, chip->qg_subtype,
			(chip->qg_version == QG_LITE) ? "QG_LITE" : "QG_PMIC5",
			(chip->qg_mode == QG_V_I_MODE) ? "QG_V_I" : "QG_V");

	return rc;

fail_votable:
	destroy_votable(chip->awake_votable);
fail_device:
	device_destroy(chip->qg_class, chip->dev_no);
	cdev_del(&chip->qg_cdev);
	unregister_chrdev_region(chip->dev_no, 1);
	return rc;
}

static int qpnp_qg_remove(struct platform_device *pdev)
{
	struct qpnp_qg *chip = platform_get_drvdata(pdev);

	qg_batterydata_exit();
	qg_soc_exit(chip);

	cancel_delayed_work_sync(&chip->qg_sleep_exit_work);
	cancel_work_sync(&chip->udata_work);
	cancel_work_sync(&chip->qg_status_change_work);
#if defined(CONFIG_SOMC_CHARGER_EXTENSION)
	cancel_work_sync(&chip->psy_chg_work);
#endif
	sysfs_remove_groups(&chip->dev->kobj, qg_groups);
	debugfs_remove_recursive(chip->dfs_root);
	device_destroy(chip->qg_class, chip->dev_no);
	cdev_del(&chip->qg_cdev);
	unregister_chrdev_region(chip->dev_no, 1);
	mutex_destroy(&chip->bus_lock);
	mutex_destroy(&chip->data_lock);
	mutex_destroy(&chip->soc_lock);
	if (chip->awake_votable)
		destroy_votable(chip->awake_votable);
#if defined(CONFIG_SOMC_CHARGER_EXTENSION)
	wakeup_source_unregister(chip->step_ws);
	qg_somc_remove_sysfs_entries(chip->dev);
#endif
	return 0;
}

static void qpnp_qg_shutdown(struct platform_device *pdev)
{
	struct qpnp_qg *chip = platform_get_drvdata(pdev);
	bool input_present = is_input_present(chip);

	if (!input_present || !chip->profile_loaded)
		return;
	/*
	 * Charging status doesn't matter when the device shuts down and we
	 * have to treat this as charge done. Hence pass charge_done as true.
	 */
	cycle_count_update(chip->counter,
			DIV_ROUND_CLOSEST(chip->msoc * 255, 100),
			POWER_SUPPLY_STATUS_NOT_CHARGING,
			true, input_present);
}

static const struct of_device_id match_table[] = {
	{
		.compatible = "qcom,qpnp-qg-lite",
		.data = (void *)&config[PM2250],
	},
	{
		.compatible = "qcom,pm6150-qg",
		.data = (void *)&config[PM6150],
	},
	{
		.compatible = "qcom,pmi632-qg",
		.data = (void *)&config[PMI632],
	},
	{
		.compatible = "qcom,pm7250b-qg",
		.data = (void *)&config[PM7250B],
	},
	{
	},
};

static struct platform_driver qpnp_qg_driver = {
	.driver		= {
		.name		= "qcom,qpnp-qg",
		.of_match_table	= match_table,
		.pm		= &qpnp_qg_pm_ops,
	},
	.probe		= qpnp_qg_probe,
	.remove		= qpnp_qg_remove,
	.shutdown	= qpnp_qg_shutdown,
};
module_platform_driver(qpnp_qg_driver);

MODULE_DESCRIPTION("QPNP QG Driver");
MODULE_LICENSE("GPL v2");
