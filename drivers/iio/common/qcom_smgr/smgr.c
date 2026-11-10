// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm Sensor Manager driver
 *
 * Copyright (c) 2021, Yassine Oudjana <y.oudjana@protonmail.com>
 */

#include <linux/iio/buffer.h>
#include <linux/iio/common/qcom_smgr.h>
#include <linux/iio/iio.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/remoteproc/qcom_rproc.h>
#include <linux/soc/qcom/qmi.h>
#include <linux/types.h>
#include <linux/workqueue.h>
#include <net/sock.h>

#include "qmi/sns_smgr.h"

#define SMGR_REPORT_RATE_IN_HZ		0xf000

struct smgr {
	struct device *dev;

	struct qmi_handle sns_smgr_hdl;
	struct sockaddr_qrtr sns_smgr_info;
	struct work_struct sns_smgr_work;

	u8 sensor_count;
	struct smgr_sensor *sensors;
};

static const char *smgr_sensor_type_platform_names[] = {
	[SNS_SMGR_SENSOR_TYPE_ACCEL] = "qcom-smgr-accel",
	[SNS_SMGR_SENSOR_TYPE_GYRO] = "qcom-smgr-gyro",
	[SNS_SMGR_SENSOR_TYPE_MAG] = "qcom-smgr-mag",
	[SNS_SMGR_SENSOR_TYPE_PROX_LIGHT] = "qcom-smgr-prox-light",
	[SNS_SMGR_SENSOR_TYPE_PRESSURE] = "qcom-smgr-pressure",
	[SNS_SMGR_SENSOR_TYPE_HALL_EFFECT] = "qcom-smgr-hall-effect"
};

static void smgr_unregister_sensor(void *data)
{
	struct platform_device *pdev = data;

	platform_device_unregister(pdev);
}

static int smgr_register_sensor(struct smgr *smgr, struct smgr_sensor *sensor)
{
	struct platform_device *pdev;
	const char *name = smgr_sensor_type_platform_names[sensor->type];

	pdev = platform_device_register_data(smgr->dev, name, sensor->id,
					     &sensor, sizeof(sensor));
	if (IS_ERR(pdev)) {
		dev_err(smgr->dev, "Failed to register %s: %pe\n", name, pdev);
		return PTR_ERR(pdev);
	}

	return devm_add_action_or_reset(smgr->dev, smgr_unregister_sensor,
					pdev);
}

static int smgr_request_all_sensor_info(struct smgr *smgr,
					struct smgr_sensor **sensors)
{
	struct sns_smgr_all_sensor_info_resp resp = {};
	struct qmi_txn txn;
	u8 i;
	int ret;

	dev_dbg(smgr->dev, "Getting available sensors\n");

	ret = qmi_txn_init(&smgr->sns_smgr_hdl, &txn,
			   sns_smgr_all_sensor_info_resp_ei, &resp);
	if (ret < 0) {
		dev_err(smgr->dev, "Failed to initialize QMI TXN: %d\n", ret);
		return ret;
	}

	ret = qmi_send_request(&smgr->sns_smgr_hdl, &smgr->sns_smgr_info, &txn,
			       SNS_SMGR_ALL_SENSOR_INFO_MSG_ID,
			       SNS_SMGR_ALL_SENSOR_INFO_REQ_MAX_LEN, NULL,
			       NULL);
	if (ret) {
		dev_err(smgr->dev,
			"Failed to send available sensors request: %d\n", ret);
		qmi_txn_cancel(&txn);
		return ret;
	}

	ret = qmi_txn_wait(&txn, 5 * HZ);
	if (ret < 0)
		return ret;

	/* Check the response */
	if (resp.result) {
		dev_err(smgr->dev, "Available sensors request failed: 0x%x\n",
			resp.result);
		return -EREMOTEIO;
	}

	*sensors = devm_kzalloc(smgr->dev,
				sizeof(struct smgr_sensor) * resp.item_len,
				GFP_KERNEL);

	for (i = 0; i < resp.item_len; ++i) {
		(*sensors)[i].id = resp.items[i].id;
		(*sensors)[i].type =
			sns_smgr_sensor_type_from_str(resp.items[i].type);
	}

	return resp.item_len;
}

static int smgr_request_single_sensor_info(struct smgr *smgr,
					   struct smgr_sensor *sensor)
{
	struct sns_smgr_single_sensor_info_req req = {
		.sensor_id = sensor->id,
	};
	struct sns_smgr_single_sensor_info_resp resp = {};
	struct qmi_txn txn;
	u8 i;
	int ret;

	dev_vdbg(smgr->dev, "Getting single sensor info for ID 0x%02x\n",
		 sensor->id);

	ret = qmi_txn_init(&smgr->sns_smgr_hdl, &txn,
			   sns_smgr_single_sensor_info_resp_ei, &resp);
	if (ret < 0) {
		dev_err(smgr->dev, "Failed to initialize QMI transaction: %d\n",
			ret);
		return ret;
	}

	ret = qmi_send_request(&smgr->sns_smgr_hdl, &smgr->sns_smgr_info, &txn,
			       SNS_SMGR_SINGLE_SENSOR_INFO_MSG_ID,
			       SNS_SMGR_SINGLE_SENSOR_INFO_REQ_MAX_LEN,
			       sns_smgr_single_sensor_info_req_ei, &req);
	if (ret < 0) {
		dev_err(smgr->dev, "Failed to send sensor data request: %d\n",
			ret);
		qmi_txn_cancel(&txn);
		return ret;
	}

	ret = qmi_txn_wait(&txn, 5 * HZ);
	if (ret < 0)
		return ret;

	/* Check the response */
	if (resp.result) {
		dev_err(smgr->dev, "Single sensor info request failed: 0x%x\n",
			resp.result);
		return -EREMOTEIO;
	}

	sensor->data_type_count = resp.data_type_len;
	sensor->data_types = devm_kzalloc(smgr->dev,
					  sizeof(struct smgr_data_type_item) *
						  sensor->data_type_count,
					  GFP_KERNEL);
	if (!sensor->data_types)
		return -ENOMEM;

	for (i = 0; i < sensor->data_type_count; ++i) {
		sensor->data_types[i].name = devm_kstrdup_const(
			smgr->dev, resp.data_types[i].name, GFP_KERNEL);
		sensor->data_types[i].vendor = devm_kstrdup_const(
			smgr->dev, resp.data_types[i].vendor, GFP_KERNEL);

		sensor->data_types[i].max_sample_rate =
			resp.data_types[i].max_sample_rate;
	}

	return 0;
}

static int smgr_request_buffering(struct smgr *smgr, struct smgr_sensor *sensor,
				  bool enable)
{
	struct sns_smgr_buffering_req req = {
		/*
		 * Reuse sensor ID as a report ID to avoid having to keep track
		 * of a separate set of IDs
		 */
		.report_id = sensor->id,
		.notify_suspend_valid = false
	};
	struct sns_smgr_buffering_resp resp = {};
	struct qmi_txn txn;
	int ret;

	if (enable) {
		req.action = SNS_SMGR_BUFFERING_ACTION_ADD;
		/* TODO: Replace hardcoded values */
		req.item_len = 1;
		req.items[0].sensor_id = sensor->id;
		req.items[0].data_type = SNS_SMGR_DATA_TYPE_PRIMARY;
		req.items[0].decimation = 0x3;
		req.items[0].calibration = 0xf;
		req.report_rate = sensor->data_types[0].cur_sample_rate *
				  SMGR_REPORT_RATE_IN_HZ;
		req.items[0].sampling_rate =
			sensor->data_types[0].cur_sample_rate;

		dev_dbg(smgr->dev,
			"Requesting buffering for sensor 0x%02x, report rate: %d, sample rate: %d",
			req.items[0].sensor_id, req.report_rate,
			req.items[0].sampling_rate);
	} else
		req.action = SNS_SMGR_BUFFERING_ACTION_DELETE;

	ret = qmi_txn_init(&smgr->sns_smgr_hdl, &txn,
			   sns_smgr_buffering_resp_ei, &resp);
	if (ret < 0) {
		dev_err(smgr->dev, "Failed to initialize QMI TXN: %d\n", ret);
		return ret;
	}

	ret = qmi_send_request(&smgr->sns_smgr_hdl, &smgr->sns_smgr_info, &txn,
			       SNS_SMGR_BUFFERING_MSG_ID,
			       SNS_SMGR_BUFFERING_REQ_MAX_LEN,
			       sns_smgr_buffering_req_ei, &req);
	if (ret < 0) {
		dev_err(smgr->dev, "Failed to send buffering request: %d\n",
			ret);
		qmi_txn_cancel(&txn);
		return ret;
	}

	ret = qmi_txn_wait(&txn, 5 * HZ);
	if (ret < 0)
		return ret;

	/* Check the response */
	if (resp.result) {
		dev_err(smgr->dev, "Buffering request failed: 0x%x\n",
			resp.result);
		return -EREMOTEIO;
	}

	dev_dbg(smgr->dev, "Buffering response ack_nak %d\n", resp.ack_nak);

	return 0;
}

static void smgr_buffering_report_handler(struct qmi_handle *hdl,
					  struct sockaddr_qrtr *sq,
					  struct qmi_txn *txn, const void *data)
{
	struct smgr *smgr = container_of(hdl, struct smgr, sns_smgr_hdl);
	struct sns_smgr_buffering_report_ind *ind =
		(struct sns_smgr_buffering_report_ind *)data;
	struct smgr_sensor *sensor;
	u8 i;

	for (i = 0; i < smgr->sensor_count; ++i) {
		sensor = &smgr->sensors[i];

		if (sensor->id != ind->report_id)
			continue;

		// TODO: handle multiple samples
		iio_push_to_buffers_with_timestamp(sensor->iio_dev,
						   ind->samples[0].values,
						   ind->metadata.timestamp);

		break;
	}
}

static const struct qmi_msg_handler smgr_msg_handlers[] = {
	{
		.type = QMI_INDICATION,
		.msg_id = SNS_SMGR_BUFFERING_REPORT_MSG_ID,
		.ei = sns_smgr_buffering_report_ind_ei,
		.decoded_size = sizeof(struct sns_smgr_buffering_report_ind),
		.fn = smgr_buffering_report_handler,
	},
	{}
};

static int smgr_sensor_postenable(struct iio_dev *iio_dev)
{
	struct smgr *smgr = dev_get_drvdata(iio_dev->dev.parent->parent);
	struct smgr_iio_priv *priv = iio_priv(iio_dev);
	struct smgr_sensor *sensor = priv->sensor;

	return smgr_request_buffering(smgr, sensor, true);
}

static int smgr_sensor_postdisable(struct iio_dev *iio_dev)
{
	struct smgr *smgr = dev_get_drvdata(iio_dev->dev.parent->parent);
	struct smgr_iio_priv *priv = iio_priv(iio_dev);
	struct smgr_sensor *sensor = priv->sensor;

	return smgr_request_buffering(smgr, sensor, false);
}

struct iio_buffer_setup_ops smgr_buffer_ops = {
	.postenable = &smgr_sensor_postenable,
	.postdisable = &smgr_sensor_postdisable
};
EXPORT_SYMBOL_GPL(smgr_buffer_ops);

static int smgr_probe(struct platform_device *pdev)
{
	const struct sockaddr_qrtr *sq = dev_get_platdata(&pdev->dev);
	struct smgr *smgr;
	int i, j;
	int ret;

	if (!sq)
		return -EINVAL;

	smgr = devm_kzalloc(&pdev->dev, sizeof(*smgr), GFP_KERNEL);
	if (!smgr)
		return -ENOMEM;

	smgr->dev = &pdev->dev;
	smgr->sns_smgr_info = *sq;

	dev_set_drvdata(&pdev->dev, smgr);

	ret = qmi_handle_init(&smgr->sns_smgr_hdl,
			      SNS_SMGR_SINGLE_SENSOR_INFO_RESP_MAX_LEN, NULL,
			      smgr_msg_handlers);
	if (ret < 0) {
		dev_err(smgr->dev,
			"Failed to initialize sensor manager handle: %d\n",
			ret);
		return ret;
	}

	ret = smgr_request_all_sensor_info(smgr, &smgr->sensors);
	if (ret < 0) {
		dev_err(smgr->dev, "Failed to get available sensors: %pe\n",
			ERR_PTR(ret));
		return ret;
	}
	smgr->sensor_count = ret;

	/* Get primary and secondary sensors from each sensor ID */
	for (i = 0; i < smgr->sensor_count; i++) {
		ret = smgr_request_single_sensor_info(smgr, &smgr->sensors[i]);
		if (ret < 0) {
			dev_err(smgr->dev,
				"Failed to get sensors from ID 0x%02x: %pe\n",
				smgr->sensors[i].id, ERR_PTR(ret));
			return ret;
		}

		for (j = 0; j < smgr->sensors[i].data_type_count; j++) {
			/* Default to maximum sample rate */
			smgr->sensors[i].data_types->cur_sample_rate =
				smgr->sensors[i].data_types->max_sample_rate;

			dev_dbg(smgr->dev, "0x%02x,%d: %s %s\n",
				smgr->sensors[i].id, j,
				smgr->sensors[i].data_types[j].vendor,
				smgr->sensors[i].data_types[j].name);
		}

		smgr_register_sensor(smgr, &smgr->sensors[i]);
	}

	return 0;
}

static void smgr_remove(struct platform_device *pdev)
{
	struct smgr *smgr = dev_get_drvdata(&pdev->dev);

	qmi_handle_release(&smgr->sns_smgr_hdl);
}

static struct platform_driver smgr_driver = {
	.probe = smgr_probe,
	.remove = smgr_remove,
	.driver	= {
		.name = "qcom-smgr",
	},
};

/*
 * The Sensor Manager is a QMI service of the SSC (SLPI), which appears
 * once the remote processor runs and the sensor registry has been served
 * to it, and goes away when it stops. Look it up and create the device
 * for it then, instead of relying on a bus for QRTR services.
 */
static struct qmi_handle smgr_lookup;
static struct platform_device *smgr_pdev;
static struct sockaddr_qrtr smgr_sq;
static bool smgr_present;
static DEFINE_MUTEX(smgr_lock);

static void smgr_update_work_fn(struct work_struct *work)
{
	mutex_lock(&smgr_lock);
	if (smgr_present && !smgr_pdev) {
		smgr_pdev = platform_device_register_data(NULL, "qcom-smgr",
							  PLATFORM_DEVID_NONE,
							  &smgr_sq,
							  sizeof(smgr_sq));
		if (IS_ERR(smgr_pdev)) {
			pr_err("qcom-smgr: failed to register device: %pe\n",
			       smgr_pdev);
			smgr_pdev = NULL;
		}
	} else if (!smgr_present && smgr_pdev) {
		platform_device_unregister(smgr_pdev);
		smgr_pdev = NULL;
	}
	mutex_unlock(&smgr_lock);
}
static DECLARE_WORK(smgr_update_work, smgr_update_work_fn);

static int smgr_new_server(struct qmi_handle *qmi, struct qmi_service *svc)
{
	mutex_lock(&smgr_lock);
	smgr_sq.sq_family = AF_QIPCRTR;
	smgr_sq.sq_node = svc->node;
	smgr_sq.sq_port = svc->port;
	smgr_present = true;
	mutex_unlock(&smgr_lock);

	/* probing talks QMI and waits: not from the lookup handle's worker */
	schedule_work(&smgr_update_work);

	return 0;
}

static void smgr_del_server(struct qmi_handle *qmi, struct qmi_service *svc)
{
	mutex_lock(&smgr_lock);
	smgr_present = false;
	mutex_unlock(&smgr_lock);

	schedule_work(&smgr_update_work);
}

static const struct qmi_ops smgr_lookup_ops = {
	.new_server = smgr_new_server,
	.del_server = smgr_del_server,
};

static int __init smgr_init(void)
{
	int ret;

	ret = platform_driver_register(&smgr_driver);
	if (ret)
		return ret;

	ret = qmi_handle_init(&smgr_lookup, 0, &smgr_lookup_ops, NULL);
	if (ret)
		goto err_driver;

	ret = qmi_add_lookup(&smgr_lookup, SNS_SMGR_QMI_SVC_ID,
			     SNS_SMGR_QMI_SVC_V1, SNS_SMGR_QMI_INS_ID);
	if (ret)
		goto err_handle;

	return 0;

err_handle:
	qmi_handle_release(&smgr_lookup);
err_driver:
	platform_driver_unregister(&smgr_driver);
	return ret;
}
module_init(smgr_init);

static void __exit smgr_exit(void)
{
	qmi_handle_release(&smgr_lookup);
	cancel_work_sync(&smgr_update_work);
	if (smgr_pdev)
		platform_device_unregister(smgr_pdev);
	platform_driver_unregister(&smgr_driver);
}
module_exit(smgr_exit);

MODULE_AUTHOR("Yassine Oudjana <y.oudjana@protonmail.com>");
MODULE_DESCRIPTION("Qualcomm Sensor Manager driver");
MODULE_LICENSE("GPL");
