#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/clk.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>
#include <sound/soc.h>

#include <mach/hardware.h>

#include "aml_dai.h"
#include "aml_pcm.h"
#include "aml_audio_hw.h"
#include <linux/of.h>

static aml_dai_info_t dai_info[3] = {{0}};
#define AML_DAI_DEBUG


#define ALSA_PRINT(fmt,args...)	printk(KERN_INFO "[aml-i2s-dai]" fmt,##args)
#ifdef DEBUG_ALSA_SOC_DAI_SPDIF
#define ALSA_DEBUG(fmt,args...) 	printk(KERN_INFO "[aml-i2s-dai]" fmt,##args)
#define ALSA_TRACE()     			printk("[aml-i2s-dai] enter func %s,line %d\n",__FUNCTION__,__LINE__);
#else
#define ALSA_DEBUG(fmt,args...) 
#define ALSA_TRACE()   
#endif

/*
the I2S hw  and IEC958 PCM output initation,958 initation here,
for the case that only use our ALSA driver for PCM s/pdif output.
*/
static void  aml_hw_i2s_init(struct snd_pcm_runtime *runtime)
{

		unsigned i2s_mode;
		switch(runtime->format){
		case SNDRV_PCM_FORMAT_S32_LE:
			i2s_mode = AIU_I2S_MODE_PCM32;
			break;
		case SNDRV_PCM_FORMAT_S24_LE:
			i2s_mode = AIU_I2S_MODE_PCM24;
			break;
		case SNDRV_PCM_FORMAT_S16_LE:
			i2s_mode = AIU_I2S_MODE_PCM16;
			break;
		}
		audio_set_i2s_mode(i2s_mode);
		audio_set_aiubuf(runtime->dma_addr, runtime->dma_bytes,runtime->channels);
		ALSA_PRINT("i2s dma %x,phy addr %x \n",(unsigned)runtime->dma_area,(unsigned)runtime->dma_addr);
		//memset((void*)runtime->dma_area,0,runtime->dma_bytes);
		ALSA_PRINT("I2S hw init,i2s mode %d\n",i2s_mode);

}
static int aml_dai_i2s_startup(struct snd_pcm_substream *substream,
					struct snd_soc_dai *dai)
{	  	
#ifdef AML_DAI_DEBUG
	printk("***Entered %s:%s\n", __FILE__,__func__);
#endif
	return 0;
}

static void aml_dai_i2s_shutdown(struct snd_pcm_substream *substream,
					struct snd_soc_dai *dai)
{
#ifdef AML_DAI_DEBUG
	printk("***Entered %s:%s\n", __FILE__,__func__);
#endif
}
static unsigned set_clock = 0;
static int aml_dai_i2s_prepare(struct snd_pcm_substream *substream,
					struct snd_soc_dai *dai)
{
#ifdef AML_DAI_DEBUG
	printk("***Entered %s:%s\n", __FILE__,__func__);
#endif
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct aml_runtime_data *prtd = runtime->private_data;
	unsigned sample_rate = AUDIO_CLK_FREQ_48;
	audio_stream_t *s = &prtd->s;	
	switch(runtime->rate){
		case 192000:
			sample_rate	=	AUDIO_CLK_FREQ_192;
			break;
		case 176400:
			sample_rate	=	AUDIO_CLK_FREQ_1764;
			break;
		case 96000:
			sample_rate	=	AUDIO_CLK_FREQ_96;
			break;
		case 88200:
			sample_rate	=	AUDIO_CLK_FREQ_882;
			break;
		case 48000:
			sample_rate	=	AUDIO_CLK_FREQ_48;
			break;
		case 44100:
			sample_rate	=	AUDIO_CLK_FREQ_441;
			break;
		case 32000:
			sample_rate	=	AUDIO_CLK_FREQ_32;
			break;
		case 8000:
			sample_rate	=	AUDIO_CLK_FREQ_8;
			break;
		case 11025:
			sample_rate	=	AUDIO_CLK_FREQ_11;
			break;
		case 16000:
			sample_rate	=	AUDIO_CLK_FREQ_16;
			break;
		case 22050:
			sample_rate	=	AUDIO_CLK_FREQ_22;
			break;
		case 12000:
			sample_rate	=	AUDIO_CLK_FREQ_12;
			break;
		case 24000:
			sample_rate	=	AUDIO_CLK_FREQ_22;
			break;
		default:
			sample_rate	=	AUDIO_CLK_FREQ_441;
			break;
	};
	if(substream->stream == SNDRV_PCM_STREAM_CAPTURE)
	{
		s->i2s_mode = dai_info[dai->id].i2s_mode;
		audio_in_i2s_set_buf(runtime->dma_addr, runtime->dma_bytes*2,0);
		memset((void*)runtime->dma_area,0,runtime->dma_bytes*2);
		{
			int * ppp = (int*)(runtime->dma_area+runtime->dma_bytes*2-8);
			ppp[0] = 0x78787878;
			ppp[1] = 0x78787878;
		}
		s->device_type = AML_AUDIO_I2SIN;		
	}
	else{
        printk(KERN_INFO "enterd %s,set_clock:%d,sample_rate=%d\n",__func__,set_clock,sample_rate);
        if(set_clock != sample_rate ){
            set_clock = sample_rate;
            audio_set_i2s_clk(sample_rate, AUDIO_CLK_256FS);
        }
		audio_util_set_dac_i2s_format(AUDIO_ALGOUT_DAC_FORMAT_DSP);	
		
		s->device_type = AML_AUDIO_I2SOUT;
		aml_hw_i2s_init(runtime);
	}
	return 0;
}
static int aml_dai_i2s_trigger(struct snd_pcm_substream *substream, int cmd,
				struct snd_soc_dai *dai)
{
	ALSA_DEBUG();
    printk("****aml_dai_i2s_trigger******\n");
	struct snd_pcm_runtime *rtd = substream->runtime;
	switch (cmd) {
		case SNDRV_PCM_TRIGGER_START:
		case SNDRV_PCM_TRIGGER_RESUME:
		case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
			// TODO
			if(substream->stream == SNDRV_PCM_STREAM_PLAYBACK){
				printk("aiu i2s playback enable\n\n");
				audio_out_i2s_enable(1);
			}else{
				audio_in_i2s_enable(1);
				int * ppp = (int*)(rtd->dma_area+rtd->dma_bytes*2-8);
				ppp[0] = 0x78787878;
				ppp[1] = 0x78787878;
			}
			break;
		case SNDRV_PCM_TRIGGER_STOP:
		case SNDRV_PCM_TRIGGER_SUSPEND:
		case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
			if(substream->stream == SNDRV_PCM_STREAM_PLAYBACK){
				printk("aiu i2s playback disable\n\n");
				audio_out_i2s_enable(0);
			}else{
				audio_in_i2s_enable(0);
			}
			break;
		default:
			return -EINVAL;
	}

	return 0;
}	

static int aml_dai_i2s_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params,
					struct snd_soc_dai *dai)
{
#ifdef AML_DAI_DEBUG
	printk("***Entered %s:%s\n", __FILE__,__func__);
#endif
	
		
	return 0;
}

static int aml_dai_set_i2s_fmt(struct snd_soc_dai *dai,
					unsigned int fmt)
{
#ifdef AML_DAI_DEBUG
	printk("***Entered %s:%s\n", __FILE__,__func__);
#endif
	if(fmt&SND_SOC_DAIFMT_CBS_CFS)//slave mode 
		dai_info[dai->id].i2s_mode = I2S_SLAVE_MODE;
	return 0;
}

static int aml_dai_set_i2s_sysclk(struct snd_soc_dai *dai,
					int clk_id, unsigned int freq, int dir)
{
#ifdef AML_DAI_DEBUG
	printk("***Entered %s:%s\n", __FILE__,__func__);
#endif
	return 0;
}

#ifdef CONFIG_PM
static int aml_dai_i2s_suspend(struct snd_soc_dai *dai)
{
		
  printk("***Entered %s:%s\n", __FILE__,__func__);
  return 0;
}

static int aml_dai_i2s_resume(struct snd_soc_dai *dai)
{
  printk("***Entered %s:%s\n", __FILE__,__func__);
  return 0;
}

#else /* CONFIG_PM */
#  define aml_dai_i2s_suspend	NULL
#  define aml_dai_i2s_resume	NULL
#endif /* CONFIG_PM */

#ifdef AML_DAI_PCM_SUPPORT

static int aml_dai_pcm_startup(struct snd_pcm_substream *substream,
					struct snd_soc_dai *dai)
{	  	
#ifdef AML_DAI_DEBUG
	printk("***Entered %s:%s\n", __FILE__,__func__);
#endif
	return 0;
}

static void aml_dai_pcm_shutdown(struct snd_pcm_substream *substream,
					struct snd_soc_dai *dai)
{
#ifdef AML_DAI_DEBUG
	printk("***Entered %s:%s\n", __FILE__,__func__);
#endif
}

static int aml_dai_pcm_prepare(struct snd_pcm_substream *substream,
					struct snd_soc_dai *dai)
{
#ifdef AML_DAI_DEBUG
	printk("***Entered %s:%s\n", __FILE__,__func__);
#endif
	return 0;
}

static int aml_dai_pcm_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params,
					struct snd_soc_dai *dai)
{
#ifdef AML_DAI_DEBUG
	printk("***Entered %s:%s\n", __FILE__,__func__);
#endif
	return 0;
}

static int aml_dai_set_pcm_fmt(struct snd_soc_dai *dai,
					unsigned int fmt)
{
#ifdef AML_DAI_DEBUG
	printk("***Entered %s:%s\n", __FILE__,__func__);
#endif
	if(fmt&SND_SOC_DAIFMT_CBS_CFS)
	snd_soc_dai_get_drvdata(dai);		
	return 0;
}

static int aml_dai_set_pcm_sysclk(struct snd_soc_dai *dai,
					int clk_id, unsigned int freq, int dir)
{
#ifdef AML_DAI_DEBUG
	printk("***Entered %s:%s\n", __FILE__,__func__);
#endif
	return 0;
}

#ifdef CONFIG_PM
static int aml_dai_pcm_suspend(struct snd_soc_dai *dai)
{
		
  printk("***Entered %s:%s\n", __FILE__,__func__);
  return 0;
}

static int aml_dai_pcm_resume(struct snd_soc_dai *dai)
{
  printk("***Entered %s:%s\n", __FILE__,__func__);
  return 0;
}

#else /* CONFIG_PM */
#  define aml_dai_i2s_suspend	NULL
#  define aml_dai_i2s_resume	NULL
#endif /* CONFIG_PM */

#endif

#define AML_DAI_I2S_RATES		(SNDRV_PCM_RATE_8000_96000)
#define AML_DAI_I2S_FORMATS		(SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE)

#ifdef AML_DAI_PCM_SUPPORT
#define AML_DAI_PCM_RATES		(SNDRV_PCM_RATE_8000)
#define AML_DAI_PCM_FORMATS		(SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE)
#endif

static struct snd_soc_dai_ops aml_dai_i2s_ops = {
	.startup	= aml_dai_i2s_startup,
	.shutdown	= aml_dai_i2s_shutdown,
	.prepare	= aml_dai_i2s_prepare,
	.trigger = aml_dai_i2s_trigger,
	.hw_params	= aml_dai_i2s_hw_params,
	.set_fmt	= aml_dai_set_i2s_fmt,
	.set_sysclk	= aml_dai_set_i2s_sysclk,
};

#ifdef AML_DAI_PCM_SUPPORT
static struct snd_soc_dai_ops aml_dai_pcm_ops = {
	.startup	= aml_dai_pcm_startup,
	.shutdown	= aml_dai_pcm_shutdown,
	.prepare	= aml_dai_pcm_prepare,
	.hw_params	= aml_dai_pcm_hw_params,
	.set_fmt	= aml_dai_set_pcm_fmt,
	.set_sysclk	= aml_dai_set_pcm_sysclk,
};
#endif

struct snd_soc_dai_driver aml_dai[] = {
	{	.name = "aml-i2s-dai",
		.id = 0,
		.suspend = aml_dai_i2s_suspend,
		.resume = aml_dai_i2s_resume,
		.playback = {
			.channels_min = 1,
			.channels_max = 8,
			.rates = AML_DAI_I2S_RATES,
			.formats = AML_DAI_I2S_FORMATS,},
		.capture = {
			.channels_min = 1,
			.channels_max = 8,
			.rates = AML_DAI_I2S_RATES,
			.formats = AML_DAI_I2S_FORMATS,},
		.ops = &aml_dai_i2s_ops,
	},
#ifdef AML_DAI_PCM_SUPPORT
	{	.name = "aml-dai1",
		.id = 1,
		.suspend = aml_dai_pcm_suspend,
		.resume = aml_dai_pcm_resume,
		.playback = {
			.channels_min = 1,
			.channels_max = 1,
			.rates = AML_DAI_PCM_RATES,
			.formats = AML_DAI_PCM_FORMATS,},
		.capture = {
			.channels_min = 1,
			.channels_max = 1,
			.rates = AML_DAI_PCM_RATES,
			.formats = AML_DAI_PCM_FORMATS,},
		.ops = &aml_dai_pcm_ops,
	},
#endif
};

EXPORT_SYMBOL_GPL(aml_dai);

static const struct snd_soc_component_driver aml_component= {
	.name		= "aml-i2s-dai",
};
static int aml_dai_probe(struct platform_device *pdev)
{
	printk(KERN_DEBUG "enter %s\n", __func__);
#if 0
	BUG_ON(pdev->id < 0);
	BUG_ON(pdev->id >= ARRAY_SIZE(aml_dai));
	return snd_soc_register_dai(&pdev->dev, &aml_dai[pdev->id]);
#else
	return snd_soc_register_component(&pdev->dev, &aml_component,
					 aml_dai, ARRAY_SIZE(aml_dai));
#endif
}

static int aml_dai_remove(struct platform_device *pdev)
{
	snd_soc_unregister_component(&pdev->dev);
	return 0;
}

#ifdef CONFIG_USE_OF
static const struct of_device_id amlogic_dai_dt_match[]={
	{	.compatible = "amlogic,aml-i2s-dai",
	},
	{},
};
#else
#define amlogic_dai_dt_match NULL
#endif

static struct platform_driver aml_dai_driver = {
	.driver = {
		.name = "aml-i2s-dai",
		.owner = THIS_MODULE,
		.of_match_table = amlogic_dai_dt_match,
	},

	.probe = aml_dai_probe,
	.remove = aml_dai_remove,
};

static int __init aml_dai_modinit(void)
{
	return platform_driver_register(&aml_dai_driver);
}
module_init(aml_dai_modinit);

static void __exit aml_dai_modexit(void)
{
	platform_driver_unregister(&aml_dai_driver);
}
module_exit(aml_dai_modexit);

/* Module information */
MODULE_AUTHOR("AMLogic, Inc.");
MODULE_DESCRIPTION("AML DAI driver for ALSA");
MODULE_LICENSE("GPL");
