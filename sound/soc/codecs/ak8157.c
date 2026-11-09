// SPDX-License-Identifier: GPL-2.0-only
/*
 * AKM AK8157 audio clock generator, as a codec on the same DAI link as the
 * DAC it clocks. It is the I2S clock provider: it makes MCLK, BCLK and
 * LRCLK for the DAC and the SoC from its own PLL.
 *
 * Register use from ZTE's msm-3.18 driver (ZTE Axon 7):
 *   0x00 output enables: 0xe7 all off, 0x00 all on (MCLK + I2S clocks)
 *   0x01 sample rate:    0x01 44.1 kHz, 0x02 48 kHz, 0x12 96 kHz,
 *                        0x22 192 kHz
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#define AK8157_OUTPUTS		0x00
#define AK8157_OUTPUTS_OFF	0xe7
#define AK8157_OUTPUTS_ALL	0x00
#define AK8157_OUTPUTS_STEP	0xe0
#define AK8157_RATE		0x01

struct ak8157_priv {
	struct regmap *regmap;
};

static int ak8157_rate_code(unsigned int rate)
{
	switch (rate) {
	case 44100:
		return 0x01;
	case 48000:
		return 0x02;
	case 96000:
		return 0x12;
	case 192000:
		return 0x22;
	default:
		return -EINVAL;
	}
}

static int ak8157_hw_params(struct snd_pcm_substream *substream,
			    struct snd_pcm_hw_params *params,
			    struct snd_soc_dai *dai)
{
	struct ak8157_priv *ak8157 = snd_soc_component_get_drvdata(dai->component);
	int code = ak8157_rate_code(params_rate(params));
	int ret;

	if (code < 0) {
		dev_err(dai->dev, "unsupported rate %u\n", params_rate(params));
		return code;
	}

	ret = regmap_write(ak8157->regmap, AK8157_RATE, code);
	if (ret)
		return ret;
	usleep_range(5000, 6000);

	/* ZTE switches the outputs on in two steps */
	ret = regmap_write(ak8157->regmap, AK8157_OUTPUTS, AK8157_OUTPUTS_STEP);
	if (!ret)
		ret = regmap_write(ak8157->regmap, AK8157_OUTPUTS, AK8157_OUTPUTS_ALL);
	return ret;
}

static void ak8157_shutdown(struct snd_pcm_substream *substream,
			    struct snd_soc_dai *dai)
{
	struct ak8157_priv *ak8157 = snd_soc_component_get_drvdata(dai->component);

	regmap_write(ak8157->regmap, AK8157_OUTPUTS, AK8157_OUTPUTS_OFF);
}

static int ak8157_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	return 0;
}

static const struct snd_soc_dai_ops ak8157_dai_ops = {
	.hw_params	= ak8157_hw_params,
	.shutdown	= ak8157_shutdown,
	.set_fmt	= ak8157_set_fmt,
};

static struct snd_soc_dai_driver ak8157_dai = {
	.name = "ak8157",
	.playback = {
		.stream_name	= "Playback",
		.channels_min	= 1,
		.channels_max	= 8,
		.rates		= SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000 |
				  SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_192000,
		.formats	= SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE |
				  SNDRV_PCM_FMTBIT_S32_LE,
	},
	.ops = &ak8157_dai_ops,
};

static const struct snd_soc_component_driver ak8157_component = {
	.endianness = 1,
};

static const struct regmap_config ak8157_regmap = {
	.reg_bits	= 8,
	.val_bits	= 8,
	.max_register	= AK8157_RATE,
	.cache_type	= REGCACHE_NONE,
};

static int ak8157_i2c_probe(struct i2c_client *i2c)
{
	struct device *dev = &i2c->dev;
	struct gpio_descs *resets;
	struct ak8157_priv *ak8157;
	unsigned int i;
	int ret;

	ak8157 = devm_kzalloc(dev, sizeof(*ak8157), GFP_KERNEL);
	if (!ak8157)
		return -ENOMEM;

	ak8157->regmap = devm_regmap_init_i2c(i2c, &ak8157_regmap);
	if (IS_ERR(ak8157->regmap))
		return PTR_ERR(ak8157->regmap);

	/* reset pulse on every reset line (ZTE's board has two) */
	resets = devm_gpiod_get_array_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(resets))
		return dev_err_probe(dev, PTR_ERR(resets), "bad reset gpios\n");
	if (resets) {
		usleep_range(1000, 1100);
		for (i = 0; i < resets->ndescs; i++)
			gpiod_set_value_cansleep(resets->desc[i], 0);
		usleep_range(1000, 1100);
	}

	ret = regmap_write(ak8157->regmap, AK8157_OUTPUTS, AK8157_OUTPUTS_OFF);
	if (!ret)
		ret = regmap_write(ak8157->regmap, AK8157_RATE, 0x00);
	if (ret)
		return dev_err_probe(dev, ret, "no answer from the AK8157\n");

	i2c_set_clientdata(i2c, ak8157);
	return devm_snd_soc_register_component(dev, &ak8157_component,
					       &ak8157_dai, 1);
}

static const struct of_device_id ak8157_of_match[] = {
	{ .compatible = "asahi-kasei,ak8157" },
	{ }
};
MODULE_DEVICE_TABLE(of, ak8157_of_match);

static const struct i2c_device_id ak8157_i2c_id[] = {
	{ "ak8157" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ak8157_i2c_id);

static struct i2c_driver ak8157_i2c_driver = {
	.driver = {
		.name = "ak8157",
		.of_match_table = ak8157_of_match,
	},
	.probe = ak8157_i2c_probe,
	.id_table = ak8157_i2c_id,
};
module_i2c_driver(ak8157_i2c_driver);

MODULE_DESCRIPTION("ASoC AK8157 audio clock generator driver");
MODULE_LICENSE("GPL");
