// SPDX-License-Identifier: GPL-2.0-only
/*
 * Torch mode of the Qualcomm PMI8994/PMI8996 flash LED module.
 *
 * The register sequence is the torch path of the downstream
 * qpnp-flash-led driver (msm-3.18): unlock, torch mode, per-LED current,
 * module enable, headroom sense, software strobe. Flash (strobe) mode is
 * not supported. Both LEDs of a dual module are driven together as one
 * LED class device.
 *
 * The module only takes a new torch current when the strobe is switched
 * on, so a brightness change while lit switches it off and on again.
 */

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/regmap.h>

#define FLASH_TYPE			0x04
#define FLASH_TYPE_VAL			0x18
#define FLASH_SUBTYPE			0x05
#define FLASH_SUBTYPE_DUAL		0x01
#define FLASH_SUBTYPE_SINGLE		0x02
#define FLASH_MAX_CURRENT		0x41
#define FLASH_LED_CURRENT(n)		(0x42 + (n))
#define FLASH_CURRENT_MASK		GENMASK(6, 0)
#define FLASH_MODULE_ENABLE_CTRL	0x46
#define FLASH_MODULE_ENABLE_MASK	GENMASK(7, 5)
#define FLASH_MODULE_ENABLE		0x80
#define FLASH_MODULE_DISABLE		0x60
#define FLASH_LED_STROBE_CTRL		0x47
#define FLASH_STROBE_LED(n)		(0x80 >> (n))
#define FLASH_STROBE_TYPE_HW		0x04
#define FLASH_HDRM_SNS_CTRL(n)		(0x5c + (n))
#define FLASH_HDRM_SNS_MASK		0x81
#define FLASH_HDRM_SNS_ENABLE		0x81
#define FLASH_HDRM_SNS_DISABLE		0x01
#define FLASH_SEC_ACCESS		0xd0
#define FLASH_SEC_UNLOCK		0xa5
#define FLASH_TORCH			0xe4
#define FLASH_TORCH_MASK		GENMASK(1, 0)
#define FLASH_TORCH_ENABLE		0x00
#define FLASH_TORCH_DISABLE		0x03

/* 16 torch steps of 12.5 mA per LED, 200 mA at the top */
#define TORCH_MAX_LEVEL			0x0f

struct pmi8994_torch {
	struct led_classdev cdev;
	struct regmap *regmap;
	struct device *dev;
	struct mutex lock;
	u32 base;
	unsigned int num_leds;
	u8 strobe_mask;
	bool on;
};

static int torch_update(struct pmi8994_torch *t, unsigned int reg, u8 mask, u8 val)
{
	return regmap_update_bits(t->regmap, t->base + reg, mask, val);
}

static int torch_set_mode(struct pmi8994_torch *t, u8 mode)
{
	int ret;

	ret = regmap_write(t->regmap, t->base + FLASH_SEC_ACCESS, FLASH_SEC_UNLOCK);
	if (ret)
		return ret;
	return torch_update(t, FLASH_TORCH, FLASH_TORCH_MASK, mode);
}

static int torch_off(struct pmi8994_torch *t)
{
	unsigned int i;
	int ret;

	ret = torch_update(t, FLASH_LED_STROBE_CTRL,
			   t->strobe_mask | FLASH_STROBE_TYPE_HW, 0);
	if (ret)
		return ret;

	/* ramp-down time, as downstream */
	usleep_range(2160, 2500);

	for (i = 0; i < t->num_leds; i++) {
		ret = torch_update(t, FLASH_HDRM_SNS_CTRL(i), FLASH_HDRM_SNS_MASK,
				   FLASH_HDRM_SNS_DISABLE);
		if (ret)
			return ret;
	}

	ret = torch_set_mode(t, FLASH_TORCH_DISABLE);
	if (ret)
		return ret;

	ret = torch_update(t, FLASH_MODULE_ENABLE_CTRL, FLASH_MODULE_ENABLE_MASK,
			   FLASH_MODULE_DISABLE);
	if (ret)
		return ret;

	t->on = false;
	return 0;
}

static int torch_on(struct pmi8994_torch *t, unsigned int level)
{
	unsigned int i;
	int ret;

	ret = torch_set_mode(t, FLASH_TORCH_ENABLE);
	if (ret)
		return ret;

	for (i = 0; i < t->num_leds; i++) {
		ret = torch_update(t, FLASH_LED_CURRENT(i), FLASH_CURRENT_MASK, level);
		if (ret)
			return ret;
	}

	ret = torch_update(t, FLASH_MAX_CURRENT, FLASH_CURRENT_MASK, TORCH_MAX_LEVEL);
	if (ret)
		return ret;

	ret = torch_update(t, FLASH_MODULE_ENABLE_CTRL, FLASH_MODULE_ENABLE_MASK,
			   FLASH_MODULE_ENABLE);
	if (ret)
		return ret;

	for (i = 0; i < t->num_leds; i++) {
		ret = torch_update(t, FLASH_HDRM_SNS_CTRL(i), FLASH_HDRM_SNS_MASK,
				   FLASH_HDRM_SNS_ENABLE);
		if (ret)
			return ret;
	}

	/* software strobe of the LEDs (strobe type HW bit cleared) */
	ret = torch_update(t, FLASH_LED_STROBE_CTRL,
			   t->strobe_mask | FLASH_STROBE_TYPE_HW, t->strobe_mask);
	if (ret)
		return ret;

	t->on = true;
	return 0;
}

static int pmi8994_torch_set(struct led_classdev *cdev, enum led_brightness b)
{
	struct pmi8994_torch *t = container_of(cdev, struct pmi8994_torch, cdev);
	int ret = 0;

	mutex_lock(&t->lock);
	if (t->on || !b)
		ret = torch_off(t);
	if (!ret && b)
		ret = torch_on(t, b - 1);
	mutex_unlock(&t->lock);

	if (ret)
		dev_err(t->dev, "torch %s failed: %d\n", b ? "on" : "off", ret);
	return ret;
}

static int pmi8994_torch_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct led_init_data init_data = {};
	struct fwnode_handle *child;
	struct pmi8994_torch *t;
	unsigned int type, subtype, i;
	int ret;

	t = devm_kzalloc(dev, sizeof(*t), GFP_KERNEL);
	if (!t)
		return -ENOMEM;

	t->dev = dev;
	t->regmap = dev_get_regmap(dev->parent, NULL);
	if (!t->regmap)
		return dev_err_probe(dev, -ENODEV, "no regmap\n");

	ret = device_property_read_u32(dev, "reg", &t->base);
	if (ret)
		return dev_err_probe(dev, ret, "no reg\n");

	ret = regmap_read(t->regmap, t->base + FLASH_TYPE, &type);
	if (!ret)
		ret = regmap_read(t->regmap, t->base + FLASH_SUBTYPE, &subtype);
	if (ret)
		return dev_err_probe(dev, ret, "cannot read the module type\n");
	if (type != FLASH_TYPE_VAL ||
	    (subtype != FLASH_SUBTYPE_DUAL && subtype != FLASH_SUBTYPE_SINGLE))
		return dev_err_probe(dev, -ENODEV,
				     "not a PMI8994 flash module (type %#x subtype %#x)\n",
				     type, subtype);

	t->num_leds = subtype == FLASH_SUBTYPE_DUAL ? 2 : 1;
	for (i = 0; i < t->num_leds; i++)
		t->strobe_mask |= FLASH_STROBE_LED(i);

	ret = devm_mutex_init(dev, &t->lock);
	if (ret)
		return ret;

	/* start from a known state */
	ret = torch_off(t);
	if (ret)
		return dev_err_probe(dev, ret, "cannot switch the torch off\n");

	child = device_get_next_child_node(dev, NULL);
	init_data.fwnode = child;
	init_data.devicename = "pmi8994";
	init_data.default_label = "white:torch";

	t->cdev.max_brightness = TORCH_MAX_LEVEL + 1;
	t->cdev.brightness_set_blocking = pmi8994_torch_set;
	t->cdev.flags = LED_CORE_SUSPENDRESUME;

	ret = devm_led_classdev_register_ext(dev, &t->cdev, &init_data);
	fwnode_handle_put(child);
	if (ret)
		return dev_err_probe(dev, ret, "cannot register the LED\n");

	platform_set_drvdata(pdev, t);
	return 0;
}

static void pmi8994_torch_shutdown(struct platform_device *pdev)
{
	struct pmi8994_torch *t = platform_get_drvdata(pdev);

	mutex_lock(&t->lock);
	torch_off(t);
	mutex_unlock(&t->lock);
}

static const struct of_device_id pmi8994_torch_of_match[] = {
	{ .compatible = "qcom,pmi8994-flash-led" },
	{ }
};
MODULE_DEVICE_TABLE(of, pmi8994_torch_of_match);

static struct platform_driver pmi8994_torch_driver = {
	.probe = pmi8994_torch_probe,
	.shutdown = pmi8994_torch_shutdown,
	.driver = {
		.name = "leds-qcom-pmi8994-torch",
		.of_match_table = pmi8994_torch_of_match,
	},
};
module_platform_driver(pmi8994_torch_driver);

MODULE_DESCRIPTION("Qualcomm PMI8994/PMI8996 flash LED module, torch mode");
MODULE_LICENSE("GPL");
