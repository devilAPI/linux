/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef __QCOM_SMGR_H__
#define __QCOM_SMGR_H__

#include <linux/iio/types.h>
#include <linux/types.h>

enum smgr_sensor_type {
	SNS_SMGR_SENSOR_TYPE_UNKNOWN,
	SNS_SMGR_SENSOR_TYPE_ACCEL,
	SNS_SMGR_SENSOR_TYPE_GYRO,
	SNS_SMGR_SENSOR_TYPE_MAG,
	SNS_SMGR_SENSOR_TYPE_PROX_LIGHT,
	SNS_SMGR_SENSOR_TYPE_PRESSURE,
	SNS_SMGR_SENSOR_TYPE_HALL_EFFECT,

	SNS_SMGR_SENSOR_TYPE_COUNT
};

enum smgr_data_type {
	SNS_SMGR_DATA_TYPE_PRIMARY,
	SNS_SMGR_DATA_TYPE_SECONDARY,

	SNS_SMGR_DATA_TYPE_COUNT
};

struct smgr_data_type_item
{
	const char *name;
	const char *vendor;
	u16 max_sample_rate;
	u16 cur_sample_rate;
};

struct smgr_sensor
{
	u8 id;
	enum smgr_sensor_type type;
	u8 data_type_count;
	struct smgr_data_type_item *data_types;

	struct iio_dev *iio_dev;
	/* newest sample of each data type */
	s32 last[SNS_SMGR_DATA_TYPE_COUNT][3];

	/* fired for every report; the scan it pushes */
	struct iio_trigger *trig;
	struct {
		s32 values[3];
		aligned_s64 timestamp;
	} scan;
	s64 scan_ts;

	/* reporting without an enabled buffer, for sysfs reads */
	bool always_on;
};

struct smgr_iio_priv
{
	struct smgr_sensor *sensor;
	int samp_freq_avail[3];		/* min, step, max for read_avail */
};

extern struct iio_buffer_setup_ops smgr_buffer_ops;

struct device;
struct iio_dev;
int smgr_iio_setup(struct device *dev, struct iio_dev *iio_dev,
		   struct smgr_sensor *sensor);
int smgr_sensor_set_always_on(struct iio_dev *iio_dev, bool on);

#endif /* __QCOM_SMGR_H__ */
