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
 * So far: power-up, enumeration, register access, the init state of ZTE's
 * driver (taken from a register dump of the LineageOS 4.4 port at idle),
 * and SLIMbus playback (AIF1, PGD ports 10/11) to the PORT2 I2S output
 * SDTO2, which feeds the two TFA9890 speaker amplifiers. ZTE's speaker
 * path: SLIM RX1/RX2 = SB1_PB, PORT2 sync domain = SLIMbus, SDTO2 source
 * = SBI1.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/slimbus.h>
#include <linux/spi/spi.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/tlv.h>

#define AK4961_DEVICE_CODE		0x00d5
#define AK4961_DEVICE_CODE_VAL		0x61
#define AK4961_MAX_REGISTER		0x07ff

/* interface device value elements: 0x800 + register */
#define AK4961_IFD_REG_OFFSET		0x800
#define AK4961_SLIM_PGD_PORT_ARRAY(n)	(n)

#define AK4961_MCLK_RATE		9600000

#define AK4961_FLOW_CONTROL_3		0x0002
#define AK4961_DSPRSTN			BIT(0)
#define AK4961_POWER_MANAGEMENT_1	0x0003
#define AK4961_PMPLL			BIT(0)
#define AK4961_POWER_MANAGEMENT_2	0x0004
#define AK4961_PMAIF			BIT(4)
#define AK4961_CLOCK_MODE_SELECT	0x001a
#define AK4961_JITTER_CLEANER_1		0x002c
#define AK4961_MSYNC7_BDV		0x005b
#define AK4961_MSYNC7_SDV		0x005c
#define AK4961_CDCMCLK_DIVIDER		0x0065
#define AK4961_SYNC_DOMAIN_SEL1		0x0068
#define AK4961_SYNC_DOMAIN_SEL4		0x006b
#define AK4961_SDTO2_SOURCE_SEL		0x0074
#define AK4961_SBO1_SOURCE_SEL		0x0077
#define AK4961_POWER_MANAGEMENT_3	0x0005
#define AK4961_CP1_READY		BIT(4)
#define AK4961_POWER_MANAGEMENT_4	0x0006
#define AK4961_POWER_MANAGEMENT_5	0x0007
#define AK4961_POWER_MANAGEMENT_6	0x0008
#define AK4961_MIC_AMP_1_LCH_GAIN	0x0010
#define AK4961_CHARGE_PUMP_1_SETTING_1	0x0016
#define AK4961_MIC_INPUT_SELECTOR_1	0x0034

/* SLIMbus TX: SLIM TX1.. are PGD ports 0..; SB1 capture uses TX7/TX8 (ZTE) */
#define AK4961_TX_PORT_SB1		6
#define AK4961_MAX_TX_CH		16

/* SLIMbus: RX ports of the PGD start at 10 (ZTE AK4961_RX_PORT_START_NUMBER) */
#define AK4961_RX_PORT_START		10
#define AK4961_MAX_RX_CH		6

enum {
	AK4961_AIF1_PB = 0,
	AK4961_AIF1_CAP,
	AK4961_NUM_DAIS,
};

/*
 * Registers that differ from their reset values in ZTE's driver once it
 * is initialised and idle (LineageOS 4.4 port, reg_data dump), without
 * the read-only status registers. PORT2/4 are I2S masters from PLL1
 * (MSYNC2/4: 0x21, BCLK 64 fs), PLL1 runs from the 9.6 MHz MCLK.
 */
static const struct reg_sequence ak4961_init_seq[] = {
	{ 0x000f, 0x03 }, { 0x0010, 0x01 }, { 0x0011, 0x01 }, { 0x0012, 0x01 },
	{ 0x0013, 0x01 }, { 0x0016, 0x03 }, { 0x001b, 0x02 }, { 0x001c, 0x02 },
	{ 0x0026, 0x02 }, { 0x0027, 0xa0 }, { 0x002a, 0x30 }, { 0x002d, 0x22 },
	{ 0x0031, 0x80 }, { 0x0032, 0x33 }, { 0x0034, 0x66 }, { 0x0035, 0x66 },
	{ 0x003a, 0x73 }, { 0x003e, 0x04 }, { 0x0040, 0x04 }, { 0x0042, 0x3f },
	{ 0x004b, 0x21 }, { 0x004c, 0x27 }, { 0x004d, 0x3f }, { 0x004e, 0x01 },
	{ 0x0051, 0x21 }, { 0x0052, 0x27 }, { 0x0053, 0x3f }, { 0x0054, 0x21 },
	{ 0x0055, 0x4f }, { 0x0056, 0x3f }, { 0x0057, 0x21 }, { 0x0058, 0x77 },
	{ 0x0059, 0x3f }, { 0x005a, 0x21 }, { 0x005b, 0x27 }, { 0x005c, 0x3f },
	{ 0x0064, 0x01 }, { 0x0066, 0x01 }, { 0x0068, 0x10 }, { 0x006a, 0x07 },
	{ 0x006c, 0x07 }, { 0x006d, 0x77 }, { 0x006e, 0x00 }, { 0x006f, 0x77 },
};

/* per rate: CDCMCLK divider, MSYNC7 BCLK/LRCLK dividers, clock mode (ZTE) */
static const struct {
	unsigned int rate;
	u8 mdv, bdv, sdv, cmf;
} ak4961_rates[] = {
	{   8000, 0x0e, 0xef, 0x3f, 0x40 },
	{  16000, 0x0e, 0x77, 0x3f, 0x24 },
	{  32000, 0x0e, 0x3b, 0x3f, 0x08 },
	{  48000, 0x09, 0x27, 0x3f, 0x0a },
	{  96000, 0x04, 0x13, 0x3f, 0x0e },
	{ 192000, 0x04, 0x09, 0x3f, 0xf2 },
};

struct ak4961_dai_data {
	struct slim_stream_runtime *sruntime;
	struct slim_stream_config sconfig;
	unsigned int ch[AK4961_MAX_TX_CH];
};

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

	unsigned int rx_ch[AK4961_MAX_RX_CH];
	unsigned int num_rx_ch;
	unsigned int tx_ch[AK4961_MAX_TX_CH];
	unsigned int num_tx_ch;
	struct ak4961_dai_data dai[AK4961_NUM_DAIS];
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

static int ak4961_pll_event(struct snd_soc_dapm_widget *w,
			    struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		snd_soc_component_update_bits(component, AK4961_POWER_MANAGEMENT_1,
					      AK4961_PMPLL, AK4961_PMPLL);
		usleep_range(2000, 2100);
		snd_soc_component_update_bits(component, AK4961_POWER_MANAGEMENT_2,
					      AK4961_PMAIF, AK4961_PMAIF);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		snd_soc_component_update_bits(component, AK4961_FLOW_CONTROL_3,
					      AK4961_DSPRSTN, 0);
		snd_soc_component_update_bits(component, AK4961_POWER_MANAGEMENT_2,
					      AK4961_PMAIF, 0);
		snd_soc_component_update_bits(component, AK4961_POWER_MANAGEMENT_1,
					      AK4961_PMPLL, 0);
		break;
	}
	return 0;
}

static int ak4961_cp1_event(struct snd_soc_dapm_widget *w,
			    struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		usleep_range(6500, 6600);
		snd_soc_component_update_bits(component, AK4961_POWER_MANAGEMENT_3,
					      AK4961_CP1_READY, AK4961_CP1_READY);
		usleep_range(300, 400);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		snd_soc_component_update_bits(component, AK4961_POWER_MANAGEMENT_3,
					      AK4961_CP1_READY, 0);
		break;
	}
	return 0;
}

static int ak4961_micbias_event(struct snd_soc_dapm_widget *w,
				struct snd_kcontrol *kcontrol, int event)
{
	if (event == SND_SOC_DAPM_POST_PMU)
		usleep_range(14000, 14100);	/* ZTE: 48 kHz wait time */
	return 0;
}

static const char * const ak4961_cp1_clock_text[] = {
	"500kHz", "250kHz", "125kHz", "62.5kHz",
};
static SOC_ENUM_SINGLE_DECL(ak4961_cp1_clock_enum, AK4961_CHARGE_PUMP_1_SETTING_1, 0,
			    ak4961_cp1_clock_text);

/* 0 to 30 dB in 3 dB steps */
static const DECLARE_TLV_DB_SCALE(ak4961_mic_gain_tlv, 0, 300, 0);

static const char * const ak4961_mic_input_text[] = {
	"AIN1", "AIN2", "AIN3", "AIN4", "AIN5", "AIN6", "ZERO",
};
static SOC_ENUM_SINGLE_DECL(ak4961_mic1l_input_enum, AK4961_MIC_INPUT_SELECTOR_1, 0,
			    ak4961_mic_input_text);
static const struct snd_kcontrol_new ak4961_mic1l_input_mux =
	SOC_DAPM_ENUM("Mic_1L input Selector", ak4961_mic1l_input_enum);
static SOC_ENUM_SINGLE_DECL(ak4961_mic1r_input_enum, AK4961_MIC_INPUT_SELECTOR_1, 4,
			    ak4961_mic_input_text);
static const struct snd_kcontrol_new ak4961_mic1r_input_mux =
	SOC_DAPM_ENUM("Mic_1R input Selector", ak4961_mic1r_input_enum);

static const char * const ak4961_sync_domain_text[] = {
	"n/a", "aif1", "aif2", "aif3", "aif4", "dsp24k", "dsp16k", "slimbus",
};
static SOC_ENUM_SINGLE_DECL(ak4961_port2_sync_enum, AK4961_SYNC_DOMAIN_SEL1, 0,
			    ak4961_sync_domain_text);

static const char * const ak4961_sink_source_text[] = {
	"ZERO", "SDTI1A", "SDTI1B", "SDTI1C", "SDTI1D", "SDTI2", "SDTI3", "SDTI4",
	"reserved", "ADC1", "ADC2", "SBI1", "SBI2", "SBI3", "DSPO1", "DSPO2",
	"DSPO3", "DSPO4", "DSPO5", "MIXAO", "MIXBO", "SRCAO", "SRCBO", "SRCCO",
	"SRCDO", "VADO",
};
static SOC_ENUM_SINGLE_DECL(ak4961_sdto2_source_enum, AK4961_SDTO2_SOURCE_SEL, 0,
			    ak4961_sink_source_text);
static SOC_ENUM_SINGLE_DECL(ak4961_sbo1l_source_enum, AK4961_SBO1_SOURCE_SEL, 0,
			    ak4961_sink_source_text);
static const struct snd_kcontrol_new ak4961_sbo1l_source_mux =
	SOC_DAPM_ENUM("SBO1L Source Selector", ak4961_sbo1l_source_enum);

static const struct snd_kcontrol_new ak4961_controls[] = {
	SOC_ENUM("PORT2 Sync Domain", ak4961_port2_sync_enum),
	SOC_ENUM("SDTO2 Source Selector", ak4961_sdto2_source_enum),
	SOC_ENUM("CP1 Clock Mode", ak4961_cp1_clock_enum),
	SOC_SINGLE_TLV("MIC Gain 1L", AK4961_MIC_AMP_1_LCH_GAIN, 0, 10, 0,
		       ak4961_mic_gain_tlv),
	SOC_SINGLE_TLV("MIC Gain 1R", AK4961_MIC_AMP_1_LCH_GAIN + 1, 0, 10, 0,
		       ak4961_mic_gain_tlv),
};

static const struct snd_soc_dapm_widget ak4961_dapm_widgets[] = {
	SND_SOC_DAPM_AIF_IN("SB1 PB", "AIF1 Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_SUPPLY("PLL CLK", SND_SOC_NOPM, 0, 0, ak4961_pll_event,
			    SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	/* PORT2 I2S output to the speaker amplifiers */
	SND_SOC_DAPM_OUTPUT("SDTO2"),

	/* microphone 1 (ZTE handset-mic path without the DSP) */
	SND_SOC_DAPM_INPUT("AIN1"),
	SND_SOC_DAPM_SUPPLY_S("Charge Pump1", 1, AK4961_POWER_MANAGEMENT_3, 0, 0,
			      ak4961_cp1_event,
			      SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_SUPPLY("PMMP1A", AK4961_POWER_MANAGEMENT_4, 0, 0,
			    ak4961_micbias_event, SND_SOC_DAPM_POST_PMU),
	SND_SOC_DAPM_PGA("PMAIN1", AK4961_POWER_MANAGEMENT_5, 0, 0, NULL, 0),
	SND_SOC_DAPM_MUX("Mic_1L input Selector", SND_SOC_NOPM, 0, 0,
			 &ak4961_mic1l_input_mux),
	SND_SOC_DAPM_ADC("ADC_1L", NULL, AK4961_POWER_MANAGEMENT_6, 0, 0),

	/* microphone 2 (AIN2, MPWR1B bias) on the right channel of ADC1 */
	SND_SOC_DAPM_INPUT("AIN2"),
	SND_SOC_DAPM_SUPPLY("PMMP1B", AK4961_POWER_MANAGEMENT_4, 1, 0,
			    ak4961_micbias_event, SND_SOC_DAPM_POST_PMU),
	SND_SOC_DAPM_PGA("PMAIN2", AK4961_POWER_MANAGEMENT_5, 1, 0, NULL, 0),
	SND_SOC_DAPM_MUX("Mic_1R input Selector", SND_SOC_NOPM, 0, 0,
			 &ak4961_mic1r_input_mux),
	SND_SOC_DAPM_ADC("ADC_1R", NULL, AK4961_POWER_MANAGEMENT_6, 1, 0),
	SND_SOC_DAPM_MUX("SBO1L Source Selector", SND_SOC_NOPM, 0, 0,
			 &ak4961_sbo1l_source_mux),
	SND_SOC_DAPM_AIF_OUT("SB1 CAP", "AIF1 Capture", 0, SND_SOC_NOPM, 0, 0),
};

static const struct snd_soc_dapm_route ak4961_dapm_routes[] = {
	{ "SB1 PB", NULL, "PLL CLK" },
	{ "SDTO2", NULL, "SB1 PB" },

	{ "PMMP1A", NULL, "Charge Pump1" },
	{ "PMAIN1", NULL, "AIN1" },
	/* ZTE: AIN1 PMMP Selector = MPWR1A */
	{ "PMAIN1", NULL, "PMMP1A" },
	{ "Mic_1L input Selector", "AIN1", "PMAIN1" },
	{ "ADC_1L", NULL, "Mic_1L input Selector" },
	{ "ADC_1L", NULL, "Charge Pump1" },
	{ "SBO1L Source Selector", "ADC1", "ADC_1L" },
	/* SBO1 is one stereo source: ADC1 gives ADC_1L on TX7, ADC_1R on TX8 */
	{ "PMMP1B", NULL, "Charge Pump1" },
	{ "PMAIN2", NULL, "AIN2" },
	/* ZTE: AIN2 PMMP Selector = MPWR1B */
	{ "PMAIN2", NULL, "PMMP1B" },
	{ "Mic_1R input Selector", "AIN2", "PMAIN2" },
	{ "ADC_1R", NULL, "Mic_1R input Selector" },
	{ "ADC_1R", NULL, "Charge Pump1" },
	{ "SBO1L Source Selector", "ADC1", "ADC_1R" },
	{ "SB1 CAP", NULL, "SBO1L Source Selector" },
	{ "SB1 CAP", NULL, "PLL CLK" },
};

static int ak4961_hw_params(struct snd_pcm_substream *substream,
			    struct snd_pcm_hw_params *params,
			    struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct ak4961_priv *ak = snd_soc_component_get_drvdata(component);
	struct ak4961_dai_data *d = &ak->dai[dai->id];
	struct slim_stream_config *cfg = &d->sconfig;
	unsigned int rate = params_rate(params);
	unsigned int chs = params_channels(params);
	unsigned int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(ak4961_rates); i++)
		if (ak4961_rates[i].rate == rate)
			break;
	if (i == ARRAY_SIZE(ak4961_rates))
		return -EINVAL;
	bool playback = substream->stream == SNDRV_PCM_STREAM_PLAYBACK;
	unsigned int *slots = playback ? ak->rx_ch : ak->tx_ch;
	unsigned int nslots = playback ? ak->num_rx_ch : ak->num_tx_ch;
	unsigned int port0 = playback ? AK4961_RX_PORT_START : AK4961_TX_PORT_SB1;

	if (chs > nslots || chs > 2)
		return -EINVAL;

	/*
	 * The SLIMbus may have been restarted since bring-up (the controller
	 * runtime-suspends), which clears the PGD port arrays of the
	 * interface device; ZTE writes them on every hw_params as well.
	 */
	ret = slim_get_logical_addr(ak->slim_ifd);
	if (!ret)
		ret = ak4961_slim_ports_setup(ak);
	if (ret) {
		dev_err(ak->dev, "SLIMbus port arrays: %d\n", ret);
		return ret;
	}

	snd_soc_component_write(component, AK4961_CDCMCLK_DIVIDER, ak4961_rates[i].mdv);
	snd_soc_component_write(component, AK4961_MSYNC7_BDV, ak4961_rates[i].bdv);
	snd_soc_component_write(component, AK4961_MSYNC7_SDV, ak4961_rates[i].sdv);
	snd_soc_component_write(component, AK4961_CLOCK_MODE_SELECT, ak4961_rates[i].cmf);
	snd_soc_component_write(component, AK4961_JITTER_CLEANER_1, ak4961_rates[i].cmf);
	/* SB1 (AIF1) in the SLIMbus sync domain */
	snd_soc_component_update_bits(component, AK4961_SYNC_DOMAIN_SEL4, 0x70, 0x70);

	kfree(cfg->chs);
	cfg->chs = kcalloc(chs, sizeof(*cfg->chs), GFP_KERNEL);
	if (!cfg->chs)
		return -ENOMEM;
	cfg->ch_count = chs;
	cfg->rate = rate;
	cfg->bps = params_width(params);
	cfg->direction = substream->stream;
	cfg->port_mask = 0;
	for (i = 0; i < chs; i++) {
		cfg->chs[i] = slots[i];
		d->ch[i] = slots[i];
		cfg->port_mask |= BIT(port0 + i);
	}

	if (!d->sruntime)
		d->sruntime = slim_stream_allocate(ak->slim,
						   playback ? "AK4961-RX1" : "AK4961-TX1");
	return d->sruntime ? 0 : -ENOMEM;
}

static int ak4961_trigger(struct snd_pcm_substream *substream, int cmd,
			  struct snd_soc_dai *dai)
{
	struct ak4961_priv *ak = snd_soc_component_get_drvdata(dai->component);
	struct ak4961_dai_data *d = &ak->dai[dai->id];
	int ret = 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		ret = slim_stream_prepare(d->sruntime, &d->sconfig);
		if (!ret)
			ret = slim_stream_enable(d->sruntime);
		dev_dbg(ak->dev, "%s start: %u ch (%u, %u) ports %#lx rate %u bps %u: %d\n",
			dai->name, d->sconfig.ch_count, d->sconfig.chs[0],
			d->sconfig.ch_count > 1 ? d->sconfig.chs[1] : 0,
			d->sconfig.port_mask, d->sconfig.rate, d->sconfig.bps, ret);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		slim_stream_disable(d->sruntime);
		slim_stream_unprepare(d->sruntime);
		break;
	}
	return ret;
}

static int ak4961_set_channel_map(struct snd_soc_dai *dai,
				  unsigned int tx_num, const unsigned int *tx_slot,
				  unsigned int rx_num, const unsigned int *rx_slot)
{
	struct ak4961_priv *ak = snd_soc_component_get_drvdata(dai->component);
	unsigned int i;

	if (rx_slot) {
		ak->num_rx_ch = min_t(unsigned int, rx_num, AK4961_MAX_RX_CH);
		for (i = 0; i < ak->num_rx_ch; i++)
			ak->rx_ch[i] = rx_slot[i];
	}
	if (tx_slot) {
		ak->num_tx_ch = min_t(unsigned int, tx_num, AK4961_MAX_TX_CH);
		for (i = 0; i < ak->num_tx_ch; i++)
			ak->tx_ch[i] = tx_slot[i];
	}
	return 0;
}

static int ak4961_get_channel_map(const struct snd_soc_dai *dai,
				  unsigned int *tx_num, unsigned int *tx_slot,
				  unsigned int *rx_num, unsigned int *rx_slot)
{
	struct ak4961_priv *ak = snd_soc_component_get_drvdata(dai->component);
	unsigned int i;

	/*
	 * Asked before hw_params: AIF1 playback is the stereo pair on PGD
	 * ports 10/11 (ZTE's speaker path sets SLIM RX1 and RX2 to SB1_PB),
	 * AIF1 capture is SLIM TX7/TX8 (ports 6/7; TX7 = ZTE's handset mic).
	 */
	if (dai->id == AK4961_AIF1_CAP) {
		if (!tx_slot || !tx_num)
			return -EINVAL;
		*tx_num = min_t(unsigned int, 2, ak->num_tx_ch);
		for (i = 0; i < *tx_num; i++)
			tx_slot[i] = ak->tx_ch[i];
		return 0;
	}
	if (!rx_slot || !rx_num)
		return -EINVAL;
	*rx_num = min_t(unsigned int, 2, ak->num_rx_ch);
	for (i = 0; i < *rx_num; i++)
		rx_slot[i] = ak->rx_ch[i];
	return 0;
}

static const struct snd_soc_dai_ops ak4961_dai_ops = {
	.hw_params = ak4961_hw_params,
	.trigger = ak4961_trigger,
	.set_channel_map = ak4961_set_channel_map,
	.get_channel_map = ak4961_get_channel_map,
};

static struct snd_soc_dai_driver ak4961_dais[] = {
	[AK4961_AIF1_PB] = {
		.name = "ak4961_rx1",
		.id = AK4961_AIF1_PB,
		.playback = {
			.stream_name = "AIF1 Playback",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				 SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				 SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_192000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE,
			.channels_min = 1,
			.channels_max = 2,
		},
		.ops = &ak4961_dai_ops,
	},
	[AK4961_AIF1_CAP] = {
		.name = "ak4961_tx1",
		.id = AK4961_AIF1_CAP,
		.capture = {
			.stream_name = "AIF1 Capture",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				 SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE,
			.channels_min = 1,
			.channels_max = 2,	/* TX7 = mic 1, TX8 unused for now */
		},
		.ops = &ak4961_dai_ops,
	},
};

static const struct snd_soc_component_driver ak4961_component = {
	.name = "ak4961",
	.controls = ak4961_controls,
	.num_controls = ARRAY_SIZE(ak4961_controls),
	.dapm_widgets = ak4961_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(ak4961_dapm_widgets),
	.dapm_routes = ak4961_dapm_routes,
	.num_dapm_routes = ARRAY_SIZE(ak4961_dapm_routes),
	.endianness = 1,
};

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

	ret = regmap_multi_reg_write(ak->regmap, ak4961_init_seq,
				     ARRAY_SIZE(ak4961_init_seq));
	if (ret)
		return dev_err_probe(ak->dev, ret, "init sequence failed\n");

	ak->up = true;
	return devm_snd_soc_register_component(ak->dev, &ak4961_component,
					       ak4961_dais, ARRAY_SIZE(ak4961_dais));
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
