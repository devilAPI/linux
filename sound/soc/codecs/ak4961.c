// SPDX-License-Identifier: GPL-2.0-only
/*
 * AKM AK4961 audio codec (ZTE Axon 7), mainline port.
 *
 * The AK4961 is a SLIMbus device (manufacturer 0x01dd, product 0x4961)
 * with two enumeration addresses, the interface device (IFD) and the
 * generic device (PGD). Like ZTE's msm-3.18 setup ("SLIMbus + SPI" mode,
 * CIF1 low) its registers are accessed over SPI, while the audio data and
 * the SLIMbus port arrays go over SLIMbus.
 *
 * SPI protocol (ak49xx-core.c): read = 0x01, addr[15:8], addr[7:0], then
 * the data bytes; write = 0x81, addr[15:8], addr[7:0], data.
 *
 * Stage 1: power-up, enumeration, register access, device code check and
 * the SLIMbus port array setup ZTE does at bring-up.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/slimbus.h>
#include <linux/spi/spi.h>

#define AK4961_DEVICE_CODE		0x00d5
#define AK4961_DEVICE_CODE_VAL		0x61
#define AK4961_MAX_REGISTER		0x07ff

/* interface device value elements: 0x800 + register */
#define AK4961_IFD_REG_OFFSET		0x800
#define AK4961_SLIM_PGD_PORT_ARRAY(n)	(n)

#define AK4961_MCLK_RATE		9600000

struct ak4961_priv {
	struct device *dev;
	struct slim_device *slim;
	struct slim_device *slim_ifd;
	struct spi_device *spi;
	struct regmap *regmap;
	struct gpio_desc *reset;
	struct gpio_desc *ldo;
	struct gpio_desc *cif;
	struct clk *mclk;
	bool up;
};

static int ak4961_spi_reg_read(void *context, unsigned int reg, unsigned int *val)
{
	struct spi_device *spi = context;
	u8 tx[3] = { 0x01, reg >> 8, reg & 0xff };
	u8 rx;
	int ret;

	ret = spi_write_then_read(spi, tx, sizeof(tx), &rx, 1);
	if (ret)
		return ret;
	*val = rx;
	return 0;
}

static int ak4961_spi_reg_write(void *context, unsigned int reg, unsigned int val)
{
	struct spi_device *spi = context;
	u8 tx[4] = { 0x81, reg >> 8, reg & 0xff, val };

	return spi_write(spi, tx, sizeof(tx));
}

static const struct regmap_config ak4961_regmap_config = {
	.name		= "ak4961",
	.reg_bits	= 16,
	.val_bits	= 8,
	.max_register	= AK4961_MAX_REGISTER,
	.reg_read	= ak4961_spi_reg_read,
	.reg_write	= ak4961_spi_reg_write,
	.cache_type	= REGCACHE_NONE,
};

static int ak4961_ifd_write(struct ak4961_priv *ak, u16 reg, u8 val)
{
	return slim_writeb(ak->slim_ifd, AK4961_IFD_REG_OFFSET + reg, val);
}

/* the PGD port arrays ZTE sets up at bring-up (ak49xx_bring_up) */
static int ak4961_slim_ports_setup(struct ak4961_priv *ak)
{
	static const u8 ports[] = { 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f };
	unsigned int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(ports); i++) {
		ret = ak4961_ifd_write(ak, AK4961_SLIM_PGD_PORT_ARRAY(i), ports[i]);
		if (ret)
			return ret;
	}
	return 0;
}

static int ak4961_power_on(struct ak4961_priv *ak)
{
	int ret;

	/* LDO on, then SPI control interface (CIF1 low), then reset (ZTE) */
	gpiod_set_value_cansleep(ak->ldo, 1);
	msleep(30);
	gpiod_set_value_cansleep(ak->cif, 0);
	usleep_range(1000, 1100);

	gpiod_set_value_cansleep(ak->reset, 1);
	usleep_range(1000, 1100);
	gpiod_set_value_cansleep(ak->reset, 0);
	msleep(20);

	ret = clk_set_rate(ak->mclk, AK4961_MCLK_RATE);
	if (ret)
		dev_warn(ak->dev, "cannot set MCLK to %u: %d\n", AK4961_MCLK_RATE, ret);
	return clk_prepare_enable(ak->mclk);
}

static int ak4961_slim_status(struct slim_device *sdev,
			      enum slim_device_status status)
{
	struct ak4961_priv *ak = dev_get_drvdata(&sdev->dev);
	struct device_node *ifd_np;
	unsigned int code;
	int ret;

	if (!ak)	/* the interface device */
		return 0;

	dev_info(ak->dev, "SLIMbus status %d\n", status);
	if (status != SLIM_DEVICE_STATUS_UP || ak->up)
		return 0;

	ifd_np = of_parse_phandle(ak->dev->of_node, "slim-ifc-dev", 0);
	if (!ifd_np)
		return dev_err_probe(ak->dev, -EINVAL, "no slim-ifc-dev\n");
	ak->slim_ifd = of_slim_get_device(sdev->ctrl, ifd_np);
	of_node_put(ifd_np);
	if (!ak->slim_ifd)
		return dev_err_probe(ak->dev, -EINVAL, "no interface device\n");

	ret = slim_get_logical_addr(ak->slim_ifd);
	if (ret)
		return dev_err_probe(ak->dev, ret, "interface device not enumerated\n");
	dev_info(ak->dev, "enumerated: PGD LA %#x, IFD LA %#x\n",
		 sdev->laddr, ak->slim_ifd->laddr);

	ret = regmap_read(ak->regmap, AK4961_DEVICE_CODE, &code);
	if (ret)
		return dev_err_probe(ak->dev, ret, "SPI read failed\n");
	if (code != AK4961_DEVICE_CODE_VAL)
		return dev_err_probe(ak->dev, -ENODEV,
				     "device code %#x, not an AK4961\n", code);
	dev_info(ak->dev, "AK4961 device code %#x over SPI\n", code);

	ret = ak4961_slim_ports_setup(ak);
	if (ret)
		return dev_err_probe(ak->dev, ret, "SLIMbus port setup failed\n");

	ak->up = true;
	return 0;
}

static int ak4961_slim_probe(struct slim_device *sdev)
{
	struct device *dev = &sdev->dev;
	struct device_node *spi_np;
	struct device *spi_dev;
	struct ak4961_priv *ak;

	/* the interface device node only needs to exist */
	if (!of_property_present(dev->of_node, "slim-ifc-dev"))
		return 0;

	ak = devm_kzalloc(dev, sizeof(*ak), GFP_KERNEL);
	if (!ak)
		return -ENOMEM;
	ak->dev = dev;
	ak->slim = sdev;

	spi_np = of_parse_phandle(dev->of_node, "akm,spi", 0);
	if (!spi_np)
		return dev_err_probe(dev, -EINVAL, "no akm,spi\n");
	spi_dev = bus_find_device_by_of_node(&spi_bus_type, spi_np);
	of_node_put(spi_np);
	if (!spi_dev)
		return dev_err_probe(dev, -EPROBE_DEFER, "SPI device not there yet\n");
	ak->spi = to_spi_device(spi_dev);

	ak->regmap = devm_regmap_init(dev, NULL, ak->spi, &ak4961_regmap_config);
	if (IS_ERR(ak->regmap)) {
		put_device(spi_dev);
		return PTR_ERR(ak->regmap);
	}

	ak->ldo = devm_gpiod_get(dev, "ldo", GPIOD_OUT_LOW);
	if (IS_ERR(ak->ldo))
		return dev_err_probe(dev, PTR_ERR(ak->ldo), "no ldo gpio\n");
	ak->cif = devm_gpiod_get(dev, "cif", GPIOD_OUT_LOW);
	if (IS_ERR(ak->cif))
		return dev_err_probe(dev, PTR_ERR(ak->cif), "no cif gpio\n");
	ak->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ak->reset))
		return dev_err_probe(dev, PTR_ERR(ak->reset), "no reset gpio\n");
	ak->mclk = devm_clk_get(dev, "mclk");
	if (IS_ERR(ak->mclk))
		return dev_err_probe(dev, PTR_ERR(ak->mclk), "no mclk\n");

	dev_set_drvdata(dev, ak);
	/* enumeration follows once the chip is powered: ak4961_slim_status() */
	return ak4961_power_on(ak);
}

static void ak4961_slim_remove(struct slim_device *sdev)
{
	struct ak4961_priv *ak = dev_get_drvdata(&sdev->dev);

	if (!ak)
		return;
	clk_disable_unprepare(ak->mclk);
	gpiod_set_value_cansleep(ak->reset, 1);
	gpiod_set_value_cansleep(ak->ldo, 0);
	put_device(&ak->spi->dev);
}

static const struct slim_device_id ak4961_slim_id[] = {
	{ 0x01dd, 0x4961, 0x2, 0x0 },	/* PGD */
	{ 0x01dd, 0x4961, 0x0, 0x0 },	/* IFD */
	{ }
};
MODULE_DEVICE_TABLE(slim, ak4961_slim_id);

static struct slim_driver ak4961_slim_driver = {
	.driver = {
		.name = "ak4961-slim",
	},
	.probe = ak4961_slim_probe,
	.remove = ak4961_slim_remove,
	.device_status = ak4961_slim_status,
	.id_table = ak4961_slim_id,
};
module_slim_driver(ak4961_slim_driver);

MODULE_DESCRIPTION("AKM AK4961 codec driver");
MODULE_LICENSE("GPL");
