// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm PMI8994/PMI8996 SMBCHG OTG boost (USB VBUS output).
 *
 * Only the 5 V boost is handled here; charging is left to the hardware
 * defaults. Register sequence as in the downstream qpnp-smbcharger driver
 * (command-controlled OTG, no pin control): disable the boost's pulse
 * skipping (OTG_TRIM6, a secured register), let it settle, then set OTG_EN
 * in the BAT_IF CMD_CHG register; the reverse on disable.
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>

#define SMBCHG_SEC_ACCESS	0xd0
#define SMBCHG_SEC_ACCESS_VAL	0xa5

/* OTG peripheral (0x1100) */
#define OTG_RT_STS		0x10
#define OTG_TRIM6		0xf6
#define TR_ENB_SKIP_BIT		BIT(2)

/* BAT_IF peripheral (0x1200) */
#define BAT_IF_CMD_CHG		0x42
#define OTG_EN_BIT		BIT(0)

struct smbchg_otg {
	struct regmap *regmap;
	u16 otg_base;
	u16 bat_if_base;
};

static int smbchg_otg_sec_update(struct smbchg_otg *otg, u16 reg, u8 mask, u8 val)
{
	int ret;

	/* the unlock applies to the next write in the same peripheral */
	ret = regmap_write(otg->regmap, (reg & ~0xff) | SMBCHG_SEC_ACCESS,
			   SMBCHG_SEC_ACCESS_VAL);
	if (ret)
		return ret;

	return regmap_update_bits(otg->regmap, reg, mask, val);
}

static int smbchg_otg_enable(struct regulator_dev *rdev)
{
	struct smbchg_otg *otg = rdev_get_drvdata(rdev);
	int ret;

	ret = smbchg_otg_sec_update(otg, otg->otg_base + OTG_TRIM6,
				    TR_ENB_SKIP_BIT, TR_ENB_SKIP_BIT);
	if (ret)
		return ret;

	msleep(20);

	return regmap_update_bits(otg->regmap, otg->bat_if_base + BAT_IF_CMD_CHG,
				  OTG_EN_BIT, OTG_EN_BIT);
}

static int smbchg_otg_disable(struct regulator_dev *rdev)
{
	struct smbchg_otg *otg = rdev_get_drvdata(rdev);
	int ret;

	ret = regmap_update_bits(otg->regmap, otg->bat_if_base + BAT_IF_CMD_CHG,
				 OTG_EN_BIT, 0);
	if (ret)
		return ret;

	return smbchg_otg_sec_update(otg, otg->otg_base + OTG_TRIM6,
				     TR_ENB_SKIP_BIT, 0);
}

static int smbchg_otg_is_enabled(struct regulator_dev *rdev)
{
	struct smbchg_otg *otg = rdev_get_drvdata(rdev);
	unsigned int val;
	int ret;

	ret = regmap_read(otg->regmap, otg->bat_if_base + BAT_IF_CMD_CHG, &val);
	if (ret)
		return ret;

	return !!(val & OTG_EN_BIT);
}

static const struct regulator_ops smbchg_otg_ops = {
	.enable = smbchg_otg_enable,
	.disable = smbchg_otg_disable,
	.is_enabled = smbchg_otg_is_enabled,
};

static const struct regulator_desc smbchg_otg_desc = {
	.name = "smbchg-otg",
	.of_match = NULL,
	.ops = &smbchg_otg_ops,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
	.fixed_uV = 5000000,
	.n_voltages = 1,
	.enable_time = 20000,
};

static int smbchg_otg_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct regulator_config config = { };
	struct regulator_init_data *init_data;
	struct regulator_dev *rdev;
	struct smbchg_otg *otg;
	u32 base;
	int ret;

	otg = devm_kzalloc(dev, sizeof(*otg), GFP_KERNEL);
	if (!otg)
		return -ENOMEM;

	otg->regmap = dev_get_regmap(dev->parent, NULL);
	if (!otg->regmap)
		return dev_err_probe(dev, -ENOENT, "no regmap from the PMIC\n");

	ret = of_property_read_u32_index(dev->of_node, "reg", 0, &base);
	if (ret)
		return dev_err_probe(dev, ret, "no OTG base (reg 0)\n");
	otg->otg_base = base;

	ret = of_property_read_u32_index(dev->of_node, "reg", 1, &base);
	if (ret)
		return dev_err_probe(dev, ret, "no BAT_IF base (reg 1)\n");
	otg->bat_if_base = base;

	init_data = of_get_regulator_init_data(dev, dev->of_node, &smbchg_otg_desc);
	if (!init_data)
		return -ENOMEM;

	config.dev = dev;
	config.init_data = init_data;
	config.of_node = dev->of_node;
	config.driver_data = otg;
	config.regmap = otg->regmap;

	rdev = devm_regulator_register(dev, &smbchg_otg_desc, &config);
	if (IS_ERR(rdev))
		return dev_err_probe(dev, PTR_ERR(rdev), "failed to register\n");

	return 0;
}

static const struct of_device_id smbchg_otg_match[] = {
	{ .compatible = "qcom,pmi8994-smbchg-otg" },
	{ }
};
MODULE_DEVICE_TABLE(of, smbchg_otg_match);

static struct platform_driver smbchg_otg_driver = {
	.driver = {
		.name = "qcom-smbchg-otg-regulator",
		.of_match_table = smbchg_otg_match,
	},
	.probe = smbchg_otg_probe,
};
module_platform_driver(smbchg_otg_driver);

MODULE_DESCRIPTION("Qualcomm PMI8994/PMI8996 SMBCHG OTG boost regulator");
MODULE_LICENSE("GPL");
