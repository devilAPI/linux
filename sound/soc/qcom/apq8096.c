// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2018, Linaro Limited

#include <linux/module.h>
#include <linux/platform_device.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <dt-bindings/sound/qcom,q6dsp-lpass-ports.h>
#include "common.h"
#include "qdsp6/q6afe.h"

#define SLIM_MAX_TX_PORTS 16
#define SLIM_MAX_RX_PORTS 16
#define WCD9335_DEFAULT_MCLK_RATE	9600000

static int apq8096_be_hw_params_fixup(struct snd_soc_pcm_runtime *rtd,
				      struct snd_pcm_hw_params *params)
{
	struct snd_interval *rate = hw_param_interval(params,
					SNDRV_PCM_HW_PARAM_RATE);
	struct snd_interval *channels = hw_param_interval(params,
					SNDRV_PCM_HW_PARAM_CHANNELS);

	rate->min = rate->max = 48000;
	channels->min = channels->max = 2;

	return 0;
}

static int msm_snd_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *codec_dai = snd_soc_rtd_to_codec(rtd, 0);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	u32 rx_ch[SLIM_MAX_RX_PORTS], tx_ch[SLIM_MAX_TX_PORTS];
	u32 rx_ch_cnt = 0, tx_ch_cnt = 0;
	int ret = 0;

	ret = snd_soc_dai_get_channel_map(codec_dai,
				&tx_ch_cnt, tx_ch, &rx_ch_cnt, rx_ch);
	if (ret != 0 && ret != -ENOTSUPP) {
		pr_err("failed to get codec chan map, err:%d\n", ret);
		goto end;
	} else if (ret == -ENOTSUPP) {
		return 0;
	}

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		ret = snd_soc_dai_set_channel_map(cpu_dai, 0, NULL,
						  rx_ch_cnt, rx_ch);
	else
		ret = snd_soc_dai_set_channel_map(cpu_dai, tx_ch_cnt, tx_ch,
						  0, NULL);
	if (ret != 0 && ret != -ENOTSUPP)
		pr_err("Failed to set cpu chan map, err:%d\n", ret);
	else if (ret == -ENOTSUPP)
		ret = 0;
end:
	return ret;
}

static const struct snd_soc_ops apq8096_ops = {
	.hw_params = msm_snd_hw_params,
};

static int apq8096_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_dai *codec_dai = snd_soc_rtd_to_codec(rtd, 0);

	/*
	 * Codec SLIMBUS configuration
	 * RX1, RX2, RX3, RX4, RX5, RX6, RX7, RX8, RX9, RX10, RX11, RX12, RX13
	 * TX1, TX2, TX3, TX4, TX5, TX6, TX7, TX8, TX9, TX10, TX11, TX12, TX13
	 * TX14, TX15, TX16
	 */
	unsigned int rx_ch[SLIM_MAX_RX_PORTS] = {144, 145, 146, 147, 148, 149,
					150, 151, 152, 153, 154, 155, 156};
	unsigned int tx_ch[SLIM_MAX_TX_PORTS] = {128, 129, 130, 131, 132, 133,
					    134, 135, 136, 137, 138, 139,
					    140, 141, 142, 143};

	snd_soc_dai_set_channel_map(codec_dai, ARRAY_SIZE(tx_ch),
					tx_ch, ARRAY_SIZE(rx_ch), rx_ch);

	snd_soc_dai_set_sysclk(codec_dai, 0, WCD9335_DEFAULT_MCLK_RATE,
				SNDRV_PCM_STREAM_PLAYBACK);

	return 0;
}

struct apq8096_card_data {
	/* MI2S ports whose bit and frame clocks come from the codec side */
	DECLARE_BITMAP(mi2s_clk_consumer, LPASS_MAX_PORT);
};

static bool apq8096_is_mi2s(int id)
{
	return id >= PRIMARY_MI2S_RX && id <= QUATERNARY_MI2S_TX;
}

/* IBIT (internal) or EBIT (external) bit clock of an MI2S port */
static int apq8096_mi2s_bit_clk(int id, bool external)
{
	return Q6AFE_LPASS_CLK_ID_PRI_MI2S_IBIT +
	       ((id - PRIMARY_MI2S_RX) / 2) * 2 + (external ? 1 : 0);
}

static bool apq8096_mi2s_consumer(struct snd_soc_pcm_runtime *rtd)
{
	struct apq8096_card_data *data = snd_soc_card_get_drvdata(rtd->card);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);

	return test_bit(cpu_dai->id, data->mi2s_clk_consumer);
}

static int apq8096_mi2s_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	unsigned int fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF;

	fmt |= apq8096_mi2s_consumer(rtd) ? SND_SOC_DAIFMT_BC_FC :
					    SND_SOC_DAIFMT_BP_FP;
	return snd_soc_dai_set_fmt(cpu_dai, fmt);
}

static int apq8096_mi2s_hw_params(struct snd_pcm_substream *substream,
				  struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	bool ext = apq8096_mi2s_consumer(rtd);

	/*
	 * Bit clock of 64 fs. For an external clock the rate is nominal:
	 * the DSP only gates it, but enables it only with a rate.
	 */
	return snd_soc_dai_set_sysclk(cpu_dai,
				      apq8096_mi2s_bit_clk(cpu_dai->id, ext),
				      params_rate(params) * 64,
				      SNDRV_PCM_STREAM_PLAYBACK);
}

static void apq8096_mi2s_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	bool ext = apq8096_mi2s_consumer(rtd);

	snd_soc_dai_set_sysclk(cpu_dai, apq8096_mi2s_bit_clk(cpu_dai->id, ext),
			       0, SNDRV_PCM_STREAM_PLAYBACK);
}

static const struct snd_soc_ops apq8096_mi2s_ops = {
	.startup = apq8096_mi2s_startup,
	.hw_params = apq8096_mi2s_hw_params,
	.shutdown = apq8096_mi2s_shutdown,
};

static void apq8096_add_be_ops(struct snd_soc_card *card)
{
	struct apq8096_card_data *data = snd_soc_card_get_drvdata(card);
	struct snd_soc_dai_link *link;
	struct device_node *np;
	int i = 0;

	/* the links are in the order of the available child nodes */
	for_each_available_child_of_node(card->dev->of_node, np) {
		if (i >= card->num_links)
			break;
		link = &card->dai_link[i++];

		if (link->no_pcm != 1)
			continue;

		link->be_hw_params_fixup = apq8096_be_hw_params_fixup;
		if (apq8096_is_mi2s(link->id)) {
			/* "qcom,mi2s-clock-consumer": the codec side drives BCLK/WS */
			if (of_property_read_bool(np, "qcom,mi2s-clock-consumer"))
				set_bit(link->id, data->mi2s_clk_consumer);
			link->ops = &apq8096_mi2s_ops;
		} else {
			link->init = apq8096_init;
			link->ops = &apq8096_ops;
		}
	}
}

static int apq8096_platform_probe(struct platform_device *pdev)
{
	struct apq8096_card_data *data;
	struct snd_soc_card *card;
	struct device *dev = &pdev->dev;
	int ret;

	card = devm_kzalloc(dev, sizeof(*card), GFP_KERNEL);
	if (!card)
		return -ENOMEM;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;
	snd_soc_card_set_drvdata(card, data);

	card->driver_name = "apq8096";
	card->dev = dev;
	card->owner = THIS_MODULE;
	dev_set_drvdata(dev, card);
	ret = qcom_snd_parse_of(card);
	if (ret)
		return ret;

	apq8096_add_be_ops(card);
	return devm_snd_soc_register_card(dev, card);
}

static const struct of_device_id msm_snd_apq8096_dt_match[] = {
	{.compatible = "qcom,apq8096-sndcard"},
	{}
};

MODULE_DEVICE_TABLE(of, msm_snd_apq8096_dt_match);

static struct platform_driver msm_snd_apq8096_driver = {
	.probe  = apq8096_platform_probe,
	.driver = {
		.name = "msm-snd-apq8096",
		.of_match_table = msm_snd_apq8096_dt_match,
	},
};
module_platform_driver(msm_snd_apq8096_driver);
MODULE_AUTHOR("Srinivas Kandagatla <srinivas.kandagatla@linaro.org");
MODULE_DESCRIPTION("APQ8096 ASoC Machine Driver");
MODULE_LICENSE("GPL");
