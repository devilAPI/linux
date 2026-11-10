// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm Sensor Manager gyroscope, magnetometer and proximity/light driver
 *
 * Both report three axes of primary data in Q16 fixed point: rad/s for
 * the gyroscope and gauss for the magnetometer (the IIO base units), so
 * the scale is 1/65536. Modelled on smgr_accel.
 */

#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>
#include <linux/iio/buffer.h>
#include <linux/iio/common/qcom_smgr.h>
#include <linux/iio/iio.h>

static int smgr_motion_read_raw(struct iio_dev *iio_dev,
				struct iio_chan_spec const *chan, int *val,
				int *val2, long mask)
{
	struct smgr_iio_priv *priv = iio_priv(iio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		/* newest report: proximity count, or lux in Q16 */
		if (chan->type == IIO_PROXIMITY)
			*val = priv->sensor->last[SNS_SMGR_DATA_TYPE_PRIMARY][1];
		else
			*val = priv->sensor->last[SNS_SMGR_DATA_TYPE_SECONDARY][0];
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = priv->sensor->data_types[0].cur_sample_rate;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		if (chan->type == IIO_PROXIMITY) {
			*val = 1;	/* raw count */
			return IIO_VAL_INT;
		}
		*val = 1;
		*val2 = 16;	/* Q16 */
		return IIO_VAL_FRACTIONAL_LOG2;
	}

	return -EINVAL;
}

static int smgr_motion_write_raw(struct iio_dev *iio_dev,
				 struct iio_chan_spec const *chan, int val,
				 int val2, long mask)
{
	struct smgr_iio_priv *priv = iio_priv(iio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		if (val < 1 || val > priv->sensor->data_types[0].max_sample_rate)
			return -EINVAL;
		priv->sensor->data_types[0].cur_sample_rate = val;
		if (iio_buffer_enabled(iio_dev))
			return iio_dev->setup_ops->postenable(iio_dev);
		return 0;
	}

	return -EINVAL;
}

static int smgr_motion_read_avail(struct iio_dev *iio_dev,
				  struct iio_chan_spec const *chan,
				  const int **vals, int *type, int *length,
				  long mask)
{
	struct smgr_iio_priv *priv = iio_priv(iio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		priv->samp_freq_avail[0] = 1;
		priv->samp_freq_avail[1] = 1;
		priv->samp_freq_avail[2] =
			priv->sensor->data_types[0].max_sample_rate;
		*type = IIO_VAL_INT;
		*vals = priv->samp_freq_avail;
		*length = 3;
		return IIO_AVAIL_RANGE;
	}

	return -EINVAL;
}

static const struct iio_info smgr_motion_iio_info = {
	.read_raw = smgr_motion_read_raw,
	.write_raw = smgr_motion_write_raw,
	.read_avail = smgr_motion_read_avail,
};

/* Same as smgr_accel; TODO: per device from the SSC or the device tree */
static const struct iio_mount_matrix smgr_motion_mount_matrix = {
	.rotation = {
		"0", "-1", "0",
		"-1", "0", "0",
		"0", "0", "1"
	}
};

static const struct iio_mount_matrix *
smgr_motion_get_mount_matrix(const struct iio_dev *iio_dev,
			     const struct iio_chan_spec *chan)
{
	return &smgr_motion_mount_matrix;
}

static const struct iio_chan_spec_ext_info smgr_motion_ext_info[] = {
	IIO_MOUNT_MATRIX(IIO_SHARED_BY_DIR, smgr_motion_get_mount_matrix),
	{}
};

#define SMGR_MOTION_CHANNEL(_type, _axis, _index) {			\
	.type = _type,							\
	.modified = true,						\
	.channel2 = IIO_MOD_##_axis,					\
	.scan_index = _index,						\
	.scan_type = {							\
		.sign = 's',						\
		.realbits = 32,						\
		.storagebits = 32,					\
		.endianness = IIO_LE,					\
	},								\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |		\
				    BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
	.info_mask_shared_by_type_available = BIT(IIO_CHAN_INFO_SAMP_FREQ), \
	.ext_info = smgr_motion_ext_info,				\
}

#define SMGR_MOTION_TIMESTAMP {						\
	.type = IIO_TIMESTAMP,						\
	.channel = -1,							\
	.scan_index = 3,						\
	.scan_type = {							\
		.sign = 'u',						\
		.realbits = 32,						\
		.storagebits = 64,					\
		.endianness = IIO_LE,					\
	},								\
}

static const struct iio_chan_spec smgr_gyro_channels[] = {
	SMGR_MOTION_CHANNEL(IIO_ANGL_VEL, X, 0),
	SMGR_MOTION_CHANNEL(IIO_ANGL_VEL, Y, 1),
	SMGR_MOTION_CHANNEL(IIO_ANGL_VEL, Z, 2),
	SMGR_MOTION_TIMESTAMP,
};

static const struct iio_chan_spec smgr_mag_channels[] = {
	SMGR_MOTION_CHANNEL(IIO_MAGN, X, 0),
	SMGR_MOTION_CHANNEL(IIO_MAGN, Y, 1),
	SMGR_MOTION_CHANNEL(IIO_MAGN, Z, 2),
	SMGR_MOTION_TIMESTAMP,
};

/* proximity count (higher = closer) and illuminance in lux, Q16 */
static const struct iio_chan_spec smgr_prox_light_channels[] = {
	{
		.type = IIO_PROXIMITY,
		.scan_index = 0,
		.scan_type = {
			.sign = 's',
			.realbits = 32,
			.storagebits = 32,
			.endianness = IIO_LE,
		},
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_SCALE),
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.info_mask_shared_by_all_available = BIT(IIO_CHAN_INFO_SAMP_FREQ),
	},
	{
		.type = IIO_LIGHT,
		.scan_index = 1,
		.scan_type = {
			.sign = 's',
			.realbits = 32,
			.storagebits = 32,
			.endianness = IIO_LE,
		},
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_SCALE),
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.info_mask_shared_by_all_available = BIT(IIO_CHAN_INFO_SAMP_FREQ),
	},
	{
		.type = IIO_TIMESTAMP,
		.channel = -1,
		.scan_index = 2,
		.scan_type = {
			.sign = 'u',
			.realbits = 32,
			.storagebits = 64,
			.endianness = IIO_LE,
		},
	},
};

struct smgr_motion_variant {
	const char *name;
	const struct iio_chan_spec *channels;
	unsigned int num_channels;
};

static const struct smgr_motion_variant smgr_gyro_variant = {
	.name = "qcom-smgr-gyro",
	.channels = smgr_gyro_channels,
	.num_channels = ARRAY_SIZE(smgr_gyro_channels),
};

static const struct smgr_motion_variant smgr_mag_variant = {
	.name = "qcom-smgr-mag",
	.channels = smgr_mag_channels,
	.num_channels = ARRAY_SIZE(smgr_mag_channels),
};

static const struct smgr_motion_variant smgr_prox_light_variant = {
	.name = "qcom-smgr-prox-light",
	.channels = smgr_prox_light_channels,
	.num_channels = ARRAY_SIZE(smgr_prox_light_channels),
};

static int smgr_motion_probe(struct platform_device *pdev)
{
	const struct smgr_motion_variant *variant =
		(const void *)platform_get_device_id(pdev)->driver_data;
	struct iio_dev *iio_dev;
	struct smgr_iio_priv *priv;
	int ret;

	iio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*priv));
	if (!iio_dev)
		return -ENOMEM;

	priv = iio_priv(iio_dev);
	priv->sensor = *(struct smgr_sensor **)pdev->dev.platform_data;
	priv->sensor->iio_dev = iio_dev;
	platform_set_drvdata(pdev, iio_dev);

	iio_dev->name = variant->name;
	iio_dev->info = &smgr_motion_iio_info;
	iio_dev->channels = variant->channels;
	iio_dev->num_channels = variant->num_channels;

	ret = smgr_iio_setup(&pdev->dev, iio_dev, priv->sensor);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "failed to set up buffer\n");

	ret = devm_iio_device_register(&pdev->dev, iio_dev);
	if (ret)
		return ret;

	if (variant == &smgr_prox_light_variant) {
		struct smgr_sensor *sensor = priv->sensor;
		unsigned int i;

		/* report continuously, slowly, for sysfs readers */
		for (i = 0; i < sensor->data_type_count; i++)
			sensor->data_types[i].cur_sample_rate =
				min_t(u16, 10, sensor->data_types[i].max_sample_rate);
		ret = smgr_sensor_set_always_on(iio_dev, true);
		if (ret)
			dev_warn(&pdev->dev, "cannot start reporting: %d\n", ret);
	}

	return 0;
}

static void smgr_motion_remove(struct platform_device *pdev)
{
	struct iio_dev *iio_dev = platform_get_drvdata(pdev);
	struct smgr_iio_priv *priv = iio_priv(iio_dev);

	/*
	 * No QMI request here: this also runs while the core is going away.
	 * Reports that still arrive are dropped (no IIO device).
	 */
	priv->sensor->always_on = false;
	priv->sensor->trig = NULL;
	priv->sensor->iio_dev = NULL;
}

static const struct platform_device_id smgr_motion_ids[] = {
	{ .name = "qcom-smgr-gyro", .driver_data = (kernel_ulong_t)&smgr_gyro_variant },
	{ .name = "qcom-smgr-mag", .driver_data = (kernel_ulong_t)&smgr_mag_variant },
	{ .name = "qcom-smgr-prox-light", .driver_data = (kernel_ulong_t)&smgr_prox_light_variant },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(platform, smgr_motion_ids);

static struct platform_driver smgr_motion_driver = {
	.probe = smgr_motion_probe,
	.remove = smgr_motion_remove,
	.driver	= {
		.name = "smgr_motion",
	},
	.id_table = smgr_motion_ids,
};
module_platform_driver(smgr_motion_driver);

MODULE_DESCRIPTION("Qualcomm Sensor Manager gyroscope, magnetometer and proximity/light driver");
MODULE_LICENSE("GPL");
