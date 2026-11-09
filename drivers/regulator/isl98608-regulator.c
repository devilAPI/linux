// SPDX-License-Identifier: GPL-2.0-only
/*
 * Renesas (Intersil) ISL98608 supply for an audio output stage, as one
 * switchable regulator.
 *
 * The chip is enabled with a GPIO and then configured over I2C. The
 * register values are the ones ZTE programs on the ZTE Axon 7 Hi-Fi path
 * (msm-3.18 isl98608.c, default "not 7 V" variant); the outputs are not
 * otherwise adjustable here.
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>

#define ISL98608_CTRL		0x05
#define ISL98608_CTRL_OFF	0x80

static const struct reg_sequence isl98608_on_seq[] = {
	{ 0x00, 0x00 }, { 0x01, 0x00 }, { 0x02, 0x00 }, { 0x03, 0x00 },
	{ 0x04, 0x00 }, { 0x05, 0x87 }, { 0x06, 0x0d }, { 0x07, 0x00 },
	{ 0x08, 0x00 }, { 0x09, 0x08 }, { 0x0a, 0xf0 }, { 0x0b, 0x0a },
	{ 0x0c, 0x05 }, { 0x0d, 0x34 },
};

struct isl98608 {
	struct regmap *regmap;
	struct gpio_desc *enable;
	bool on;
};

static int isl98608_enable(struct regulator_dev *rdev)
{
	struct isl98608 *isl = rdev_get_drvdata(rdev);
	int ret;

	gpiod_set_value_cansleep(isl->enable, 1);
	usleep_range(10000, 11000);

	ret = regmap_multi_reg_write(isl->regmap, isl98608_on_seq,
				     ARRAY_SIZE(isl98608_on_seq));
	if (ret) {
		gpiod_set_value_cansleep(isl->enable, 0);
		return ret;
	}
	isl->on = true;
	return 0;
}

static int isl98608_disable(struct regulator_dev *rdev)
{
	struct isl98608 *isl = rdev_get_drvdata(rdev);

	regmap_write(isl->regmap, ISL98608_CTRL, ISL98608_CTRL_OFF);
	gpiod_set_value_cansleep(isl->enable, 0);
	isl->on = false;
	return 0;
}

static int isl98608_is_enabled(struct regulator_dev *rdev)
{
	struct isl98608 *isl = rdev_get_drvdata(rdev);

	return isl->on;
}

static const struct regulator_ops isl98608_ops = {
	.enable		= isl98608_enable,
	.disable	= isl98608_disable,
	.is_enabled	= isl98608_is_enabled,
};

static const struct regulator_desc isl98608_desc = {
	.name		= "isl98608",
	.of_match	= NULL,
	.ops		= &isl98608_ops,
	.type		= REGULATOR_VOLTAGE,
	.owner		= THIS_MODULE,
	.enable_time	= 10000,
};

static const struct regmap_config isl98608_regmap = {
	.reg_bits	= 8,
	.val_bits	= 8,
	.max_register	= 0x0d,
	.cache_type	= REGCACHE_NONE,
};

static int isl98608_probe(struct i2c_client *i2c)
{
	struct device *dev = &i2c->dev;
	struct regulator_config config = {};
	struct regulator_dev *rdev;
	struct isl98608 *isl;

	isl = devm_kzalloc(dev, sizeof(*isl), GFP_KERNEL);
	if (!isl)
		return -ENOMEM;

	isl->regmap = devm_regmap_init_i2c(i2c, &isl98608_regmap);
	if (IS_ERR(isl->regmap))
		return PTR_ERR(isl->regmap);

	isl->enable = devm_gpiod_get(dev, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(isl->enable))
		return dev_err_probe(dev, PTR_ERR(isl->enable), "no enable gpio\n");

	config.dev = dev;
	config.of_node = dev->of_node;
	config.driver_data = isl;
	config.init_data = of_get_regulator_init_data(dev, dev->of_node,
						      &isl98608_desc);

	rdev = devm_regulator_register(dev, &isl98608_desc, &config);
	if (IS_ERR(rdev))
		return dev_err_probe(dev, PTR_ERR(rdev), "cannot register\n");
	return 0;
}

static const struct of_device_id isl98608_of_match[] = {
	{ .compatible = "isil,isl98608" },
	{ }
};
MODULE_DEVICE_TABLE(of, isl98608_of_match);

static struct i2c_driver isl98608_driver = {
	.driver = {
		.name = "isl98608",
		.of_match_table = isl98608_of_match,
	},
	.probe = isl98608_probe,
};
module_i2c_driver(isl98608_driver);

MODULE_DESCRIPTION("ISL98608 audio supply regulator");
MODULE_LICENSE("GPL");
