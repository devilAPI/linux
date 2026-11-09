// SPDX-License-Identifier: GPL-2.0-only
/*
 * AKM AK4490 stereo DAC (PCM mode, I2S, clock consumer).
 *
 * Written for the ZTE Axon 7 Hi-Fi path from the register usage of ZTE's
 * msm-3.18 driver: the DAC is fed by I2S whose clocks come from an
 * external generator, it runs in 32-bit I2S mode (DIF = 111), the
 * sampling speed is set with DFS/DFS2, and RSTN in CONTROL1 is the DAPM
 * power bit of the DAC.
 *
 * Supplies: "dvdd" (digital/LDO) and "avdd" (analog). PDN is the
 * power-down pin, asserted while the codec is off.
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/tlv.h>

#define AK4490_CONTROL1		0x00
#define AK4490_RSTN		BIT(0)
#define AK4490_DIF_MASK		GENMASK(3, 1)
#define AK4490_DIF_I2S_32	(7 << 1)
#define AK4490_CONTROL2		0x01
#define AK4490_SMUTE		BIT(0)
#define AK4490_DFS_MASK		GENMASK(4, 3)
#define AK4490_DFS_NORMAL	(0 << 3)
#define AK4490_DFS_DOUBLE	(1 << 3)
#define AK4490_DFS_QUAD		(2 << 3)
#define AK4490_CONTROL3		0x02
#define AK4490_LATT		0x03
#define AK4490_RATT		0x04
#define AK4490_CONTROL4		0x05
#define AK4490_DFS2		BIT(1)
#define AK4490_DSD1		0x06
#define AK4490_CONTROL5		0x07
#define AK4490_CONTROL6		0x08
#define AK4490_DSD2		0x09
#define AK4490_MAX_REG		AK4490_DSD2

static const char * const ak4490_supply_names[] = { "dvdd", "avdd" };

struct ak4490_priv {
	struct regmap *regmap;
	struct gpio_desc *pdn;
	struct gpio_desc *mute;
	struct regulator_bulk_data supplies[ARRAY_SIZE(ak4490_supply_names)];
};

static const struct reg_default ak4490_reg_defaults[] = {
	{ AK4490_CONTROL1, 0x04 },
	{ AK4490_CONTROL2, 0x22 },
	{ AK4490_CONTROL3, 0x00 },
	{ AK4490_LATT,     0xff },
	{ AK4490_RATT,     0xff },
	{ AK4490_CONTROL4, 0x00 },
	{ AK4490_DSD1,     0x00 },
	{ AK4490_CONTROL5, 0x00 },
	{ AK4490_CONTROL6, 0x00 },
	{ AK4490_DSD2,     0x00 },
};

/* 0.5 dB steps from -127 dB, 0x00 is mute */
static const DECLARE_TLV_DB_SCALE(ak4490_att_tlv, -12750, 50, 1);

static const char * const ak4490_dem_texts[] = { "44.1kHz", "Off", "48kHz", "32kHz" };
static SOC_ENUM_SINGLE_DECL(ak4490_dem_enum, AK4490_CONTROL2, 1, ak4490_dem_texts);

static const struct snd_kcontrol_new ak4490_controls[] = {
	SOC_DOUBLE_R_TLV("Digital Playback Volume", AK4490_LATT, AK4490_RATT,
			 0, 0xff, 0, ak4490_att_tlv),
	SOC_ENUM("De-emphasis", ak4490_dem_enum),
	SOC_SINGLE("Short Delay Filter Switch", AK4490_CONTROL2, 5, 1, 0),
	SOC_SINGLE("Slow Roll-off Filter Switch", AK4490_CONTROL3, 0, 1, 0),
	SOC_SINGLE("Super Slow Roll-off Filter Switch", AK4490_CONTROL4, 0, 1, 0),
};

static const struct snd_soc_dapm_widget ak4490_dapm_widgets[] = {
	SND_SOC_DAPM_DAC("DAC", "Playback", AK4490_CONTROL1, 0, 0),
	SND_SOC_DAPM_OUTPUT("AOUTL"),
	SND_SOC_DAPM_OUTPUT("AOUTR"),
};

static const struct snd_soc_dapm_route ak4490_dapm_routes[] = {
	{ "AOUTL", NULL, "DAC" },
	{ "AOUTR", NULL, "DAC" },
};

static int ak4490_hw_params(struct snd_pcm_substream *substream,
			    struct snd_pcm_hw_params *params,
			    struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	unsigned int rate = params_rate(params);
	unsigned int dfs, dfs2 = 0;

	if (rate <= 54000)
		dfs = AK4490_DFS_NORMAL;
	else if (rate <= 108000)
		dfs = AK4490_DFS_DOUBLE;
	else if (rate <= 216000)
		dfs = AK4490_DFS_QUAD;
	else
		return -EINVAL;

	snd_soc_component_update_bits(component, AK4490_CONTROL2,
				      AK4490_DFS_MASK, dfs);
	snd_soc_component_update_bits(component, AK4490_CONTROL4,
				      AK4490_DFS2, dfs2);
	/* 32-bit I2S framing for every sample width, as ZTE runs it */
	snd_soc_component_update_bits(component, AK4490_CONTROL1,
				      AK4490_DIF_MASK, AK4490_DIF_I2S_32);
	return 0;
}

static int ak4490_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	if ((fmt & SND_SOC_DAIFMT_FORMAT_MASK) != SND_SOC_DAIFMT_I2S)
		return -EINVAL;
	if ((fmt & SND_SOC_DAIFMT_CLOCK_PROVIDER_MASK) != SND_SOC_DAIFMT_CBC_CFC)
		return -EINVAL;
	return 0;
}

static int ak4490_mute_stream(struct snd_soc_dai *dai, int mute, int stream)
{
	struct snd_soc_component *component = dai->component;
	struct ak4490_priv *ak4490 = snd_soc_component_get_drvdata(component);

	if (mute) {
		/* external output switch first, then the DAC (ZTE: 50 ms) */
		if (ak4490->mute) {
			gpiod_set_value_cansleep(ak4490->mute, 1);
			msleep(50);
		}
		snd_soc_component_update_bits(component, AK4490_CONTROL2,
					      AK4490_SMUTE, AK4490_SMUTE);
	} else {
		snd_soc_component_update_bits(component, AK4490_CONTROL2,
					      AK4490_SMUTE, 0);
		if (ak4490->mute) {
			msleep(50);
			gpiod_set_value_cansleep(ak4490->mute, 0);
		}
	}
	return 0;
}

static const struct snd_soc_dai_ops ak4490_dai_ops = {
	.hw_params	= ak4490_hw_params,
	.set_fmt	= ak4490_set_fmt,
	.mute_stream	= ak4490_mute_stream,
	.no_capture_mute = 1,
};

#define AK4490_RATES	(SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 | \
			 SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 | \
			 SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_176400 | \
			 SNDRV_PCM_RATE_192000)
#define AK4490_FORMATS	(SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE | \
			 SNDRV_PCM_FMTBIT_S32_LE)

static struct snd_soc_dai_driver ak4490_dai = {
	.name = "ak4490",
	.playback = {
		.stream_name	= "Playback",
		.channels_min	= 1,
		.channels_max	= 2,
		.rates		= AK4490_RATES,
		.formats	= AK4490_FORMATS,
	},
	.ops = &ak4490_dai_ops,
};

static int ak4490_power_on(struct ak4490_priv *ak4490)
{
	int ret;

	gpiod_set_value_cansleep(ak4490->pdn, 1);
	ret = regulator_bulk_enable(ARRAY_SIZE(ak4490->supplies), ak4490->supplies);
	if (ret)
		return ret;
	/* supplies settle, then out of power-down (ZTE: 10 ms) */
	msleep(10);
	gpiod_set_value_cansleep(ak4490->pdn, 0);
	usleep_range(1000, 2000);

	regcache_cache_only(ak4490->regmap, false);
	ret = regcache_sync(ak4490->regmap);
	if (ret) {
		gpiod_set_value_cansleep(ak4490->pdn, 1);
		regulator_bulk_disable(ARRAY_SIZE(ak4490->supplies), ak4490->supplies);
		regcache_cache_only(ak4490->regmap, true);
	}
	return ret;
}

static void ak4490_power_off(struct ak4490_priv *ak4490)
{
	regcache_cache_only(ak4490->regmap, true);
	regcache_mark_dirty(ak4490->regmap);
	gpiod_set_value_cansleep(ak4490->pdn, 1);
	regulator_bulk_disable(ARRAY_SIZE(ak4490->supplies), ak4490->supplies);
}

static int ak4490_set_bias_level(struct snd_soc_component *component,
				 enum snd_soc_bias_level level)
{
	struct ak4490_priv *ak4490 = snd_soc_component_get_drvdata(component);
	struct snd_soc_dapm_context *dapm = snd_soc_component_to_dapm(component);

	switch (level) {
	case SND_SOC_BIAS_STANDBY:
		if (snd_soc_dapm_get_bias_level(dapm) == SND_SOC_BIAS_OFF)
			return ak4490_power_on(ak4490);
		break;
	case SND_SOC_BIAS_OFF:
		ak4490_power_off(ak4490);
		break;
	default:
		break;
	}
	return 0;
}

static const struct snd_soc_component_driver ak4490_component = {
	.controls		= ak4490_controls,
	.num_controls		= ARRAY_SIZE(ak4490_controls),
	.dapm_widgets		= ak4490_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(ak4490_dapm_widgets),
	.dapm_routes		= ak4490_dapm_routes,
	.num_dapm_routes	= ARRAY_SIZE(ak4490_dapm_routes),
	.set_bias_level		= ak4490_set_bias_level,
	.idle_bias_on		= 0,
	.endianness		= 1,
};

static bool ak4490_volatile(struct device *dev, unsigned int reg)
{
	return false;
}

static const struct regmap_config ak4490_regmap = {
	.reg_bits		= 8,
	.val_bits		= 8,
	.max_register		= AK4490_MAX_REG,
	.reg_defaults		= ak4490_reg_defaults,
	.num_reg_defaults	= ARRAY_SIZE(ak4490_reg_defaults),
	.volatile_reg		= ak4490_volatile,
	.cache_type		= REGCACHE_FLAT,
};

static int ak4490_i2c_probe(struct i2c_client *i2c)
{
	struct device *dev = &i2c->dev;
	struct ak4490_priv *ak4490;
	unsigned int i;
	int ret;

	ak4490 = devm_kzalloc(dev, sizeof(*ak4490), GFP_KERNEL);
	if (!ak4490)
		return -ENOMEM;

	ak4490->regmap = devm_regmap_init_i2c(i2c, &ak4490_regmap);
	if (IS_ERR(ak4490->regmap))
		return PTR_ERR(ak4490->regmap);

	for (i = 0; i < ARRAY_SIZE(ak4490->supplies); i++)
		ak4490->supplies[i].supply = ak4490_supply_names[i];
	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(ak4490->supplies),
				      ak4490->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "cannot get supplies\n");

	/* asserted = powered down */
	ak4490->pdn = devm_gpiod_get(dev, "pdn", GPIOD_OUT_HIGH);
	if (IS_ERR(ak4490->pdn))
		return dev_err_probe(dev, PTR_ERR(ak4490->pdn), "no pdn gpio\n");

	/* optional external output mute (asserted = muted) */
	ak4490->mute = devm_gpiod_get_optional(dev, "mute", GPIOD_OUT_HIGH);
	if (IS_ERR(ak4490->mute))
		return dev_err_probe(dev, PTR_ERR(ak4490->mute), "bad mute gpio\n");

	/* check the chip once, then leave it off until used */
	ret = ak4490_power_on(ak4490);
	if (ret)
		return dev_err_probe(dev, ret, "cannot power up\n");
	{
		unsigned int val;

		regcache_cache_bypass(ak4490->regmap, true);
		ret = regmap_read(ak4490->regmap, AK4490_CONTROL2, &val);
		regcache_cache_bypass(ak4490->regmap, false);
		if (!ret)
			dev_info(dev, "AK4490 found, CONTROL2 %#x\n", val);
	}
	ak4490_power_off(ak4490);
	if (ret)
		return dev_err_probe(dev, ret, "no answer from the AK4490\n");

	/* sound setting 2 (SC = 1), as ZTE sets it; written on power-up */
	regmap_write(ak4490->regmap, AK4490_CONTROL6, 0x01);

	i2c_set_clientdata(i2c, ak4490);
	return devm_snd_soc_register_component(dev, &ak4490_component,
					       &ak4490_dai, 1);
}

static const struct of_device_id ak4490_of_match[] = {
	{ .compatible = "asahi-kasei,ak4490" },
	{ }
};
MODULE_DEVICE_TABLE(of, ak4490_of_match);

static const struct i2c_device_id ak4490_i2c_id[] = {
	{ "ak4490" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ak4490_i2c_id);

static struct i2c_driver ak4490_i2c_driver = {
	.driver = {
		.name = "ak4490",
		.of_match_table = ak4490_of_match,
	},
	.probe = ak4490_i2c_probe,
	.id_table = ak4490_i2c_id,
};
module_i2c_driver(ak4490_i2c_driver);

MODULE_DESCRIPTION("ASoC AK4490 DAC driver");
MODULE_LICENSE("GPL");
