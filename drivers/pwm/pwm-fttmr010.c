// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2019 Intel Corporation

#include <linux/clk.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>

#define TIMER_CR			0x30

#define TIMER5_ASPEED_COUNT		0x50
#define TIMER5_ASPEED_LOAD		0x54
#define TIMER5_ASPEED_MATCH1		0x58
#define TIMER5_ASPEED_MATCH2		0x5c
#define TIMER6_ASPEED_COUNT		0x60
#define TIMER6_ASPEED_LOAD		0x64
#define TIMER6_ASPEED_MATCH1		0x68
#define TIMER6_ASPEED_MATCH2		0x6c
#define TIMER7_ASPEED_COUNT		0x70
#define TIMER7_ASPEED_LOAD		0x74
#define TIMER7_ASPEED_MATCH1		0x78
#define TIMER7_ASPEED_MATCH2		0x7c
#define TIMER8_ASPEED_COUNT		0x80
#define TIMER8_ASPEED_LOAD		0x84
#define TIMER8_ASPEED_MATCH1		0x88
#define TIMER8_ASPEED_MATCH2		0x8c

#define TIMER_5_CR_ASPEED_ENABLE	BIT(16)
#define TIMER_5_CR_ASPEED_CLOCK		BIT(17)
#define TIMER_5_CR_ASPEED_INT		BIT(18)
#define TIMER_5_CR_ASPEED_PULSE_OUT	BIT(19)
#define TIMER_6_CR_ASPEED_ENABLE	BIT(20)
#define TIMER_6_CR_ASPEED_CLOCK		BIT(21)
#define TIMER_6_CR_ASPEED_INT		BIT(22)
#define TIMER_6_CR_ASPEED_PULSE_OUT	BIT(23)
#define TIMER_7_CR_ASPEED_ENABLE	BIT(24)
#define TIMER_7_CR_ASPEED_CLOCK		BIT(25)
#define TIMER_7_CR_ASPEED_INT		BIT(26)
#define TIMER_7_CR_ASPEED_PULSE_OUT	BIT(27)
#define TIMER_8_CR_ASPEED_ENABLE	BIT(28)
#define TIMER_8_CR_ASPEED_CLOCK		BIT(29)
#define TIMER_8_CR_ASPEED_INT		BIT(30)
#define TIMER_8_CR_ASPEED_PULSE_OUT	BIT(31)

/**
 * struct pwm_fttmr010_variant - variant data depends on SoC
 * @bits:		timer counter resolution
 * @chan_min:		lowest timer channel which has pwm pulse output
 * @chan_max:		highest timer channel which has pwm pulse output
 * @output_mask:	pwm pulse output mask which is defined in device tree
 */
struct pwm_fttmr010_variant {
	u8	bits;
	u8	chan_min;
	u8	chan_max;
	u8	output_mask;
};

/**
 * struct pwm_fttmr010_chan - private data of FTTMR010 PWM channel
 * @period_ns:	current period in nanoseconds programmed to the hardware
 * @duty_ns:	current duty time in nanoseconds programmed to the hardware
 */
struct pwm_fttmr010_chan {
	u32	period_ns;
	u32	duty_ns;
};

/**
 * struct pwm_fttmr010 - private data of FTTMR010 PWM
 * @chip:		generic PWM chip
 * @variant:		local copy of hardware variant data
 * @disabled_mask:	disabled status for all channels - one bit per channel
 * @base:		base address of mapped PWM registers
 * @clk:		clock used to drive the timers
 */
struct pwm_fttmr010 {
	struct pwm_chip			chip;
	struct pwm_fttmr010_variant	variant;
	u8				disabled_mask;
	void __iomem			*base;
	struct clk			*clk;
	u32				clk_tick_ns;
};

static inline
struct pwm_fttmr010 *to_pwm_fttmr010(struct pwm_chip *chip)
{
	return container_of(chip, struct pwm_fttmr010, chip);
}

static int pwm_fttmr010_request(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct pwm_fttmr010 *priv = to_pwm_fttmr010(chip);
	struct pwm_fttmr010_chan *chan;

	if (!(priv->variant.output_mask & BIT(pwm->hwpwm))) {
		dev_warn(chip->dev,
			 "tried to request PWM channel %d without output\n",
			 pwm->hwpwm);
		return -EINVAL;
	}

	chan = devm_kzalloc(chip->dev, sizeof(*chan), GFP_KERNEL);
	if (!chan)
		return -ENOMEM;

	pwm_set_chip_data(pwm, chan);

	return 0;
}

static void pwm_fttmr010_free(struct pwm_chip *chip, struct pwm_device *pwm)
{
	devm_kfree(chip->dev, pwm_get_chip_data(pwm));
	pwm_set_chip_data(pwm, NULL);
}

static int pwm_fttmr010_enable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct pwm_fttmr010 *priv = to_pwm_fttmr010(chip);
	u32 cr;

	cr = readl(priv->base + TIMER_CR);

	switch (pwm->hwpwm) {
	case 5:
		cr |= (TIMER_5_CR_ASPEED_ENABLE | TIMER_5_CR_ASPEED_PULSE_OUT);
		break;
	case 6:
		cr |= (TIMER_6_CR_ASPEED_ENABLE | TIMER_6_CR_ASPEED_PULSE_OUT);
		break;
	case 7:
		cr |= (TIMER_7_CR_ASPEED_ENABLE | TIMER_7_CR_ASPEED_PULSE_OUT);
		break;
	case 8:
		cr |= (TIMER_8_CR_ASPEED_ENABLE | TIMER_8_CR_ASPEED_PULSE_OUT);
		break;
	default:
		return -ERANGE;
	}

	writel(cr, priv->base + TIMER_CR);
	priv->disabled_mask &= ~BIT(pwm->hwpwm);

	return 0;
}

static void pwm_fttmr010_disable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct pwm_fttmr010 *priv = to_pwm_fttmr010(chip);
	u32 cr;

	cr = readl(priv->base + TIMER_CR);

	switch (pwm->hwpwm) {
	case 5:
		cr &= ~(TIMER_5_CR_ASPEED_ENABLE | TIMER_5_CR_ASPEED_PULSE_OUT);
		break;
	case 6:
		cr &= ~(TIMER_6_CR_ASPEED_ENABLE | TIMER_6_CR_ASPEED_PULSE_OUT);
		break;
	case 7:
		cr &= ~(TIMER_7_CR_ASPEED_ENABLE | TIMER_7_CR_ASPEED_PULSE_OUT);
		break;
	case 8:
		cr &= ~(TIMER_8_CR_ASPEED_ENABLE | TIMER_8_CR_ASPEED_PULSE_OUT);
		break;
	default:
		return;
	}

	writel(cr, priv->base + TIMER_CR);
	priv->disabled_mask |= BIT(pwm->hwpwm);
}

static int pwm_fttmr010_config(struct pwm_chip *chip, struct pwm_device *pwm,
			       int duty_ns, int period_ns)
{
	u32 tload, tmatch, creg_offset, lreg_offset, mreg_offset;
	struct pwm_fttmr010_chan *chan = pwm_get_chip_data(pwm);
	struct pwm_fttmr010 *priv = to_pwm_fttmr010(chip);

	/*
	 * We currently avoid using 64bit arithmetic by using the
	 * fact that anything faster than 1Hz is easily representable
	 * by 32bits.
	 */
	if (period_ns > NSEC_PER_SEC)
		return -ERANGE;

	/* No need to update */
	if (chan->period_ns == period_ns || chan->duty_ns == duty_ns)
		return 0;

	tload = period_ns / priv->clk_tick_ns;

	/* Period is too short */
	if (tload <= 1)
		return -ERANGE;

	tmatch = duty_ns / priv->clk_tick_ns;

	/* 0% duty is not available */
	if (!tmatch)
		++tmatch;

	tmatch = tload - tmatch;

	/* Decrement to get tick numbers, instead of tick counts */
	--tload;
	--tmatch;

	if (tload == 0 || tmatch == 0)
		return -ERANGE;

	dev_dbg(priv->chip.dev, "clk_tick_ns:%u, tload:%u, tmatch:%u\n",
		priv->clk_tick_ns, tload, tmatch);

	switch (pwm->hwpwm) {
	case 5:
		creg_offset = TIMER5_ASPEED_COUNT;
		lreg_offset = TIMER5_ASPEED_LOAD;
		mreg_offset = TIMER5_ASPEED_MATCH1;
		break;
	case 6:
		creg_offset = TIMER6_ASPEED_COUNT;
		lreg_offset = TIMER6_ASPEED_LOAD;
		mreg_offset = TIMER6_ASPEED_MATCH1;
		break;
	case 7:
		creg_offset = TIMER7_ASPEED_COUNT;
		lreg_offset = TIMER7_ASPEED_LOAD;
		mreg_offset = TIMER7_ASPEED_MATCH1;
		break;
	case 8:
		creg_offset = TIMER8_ASPEED_COUNT;
		lreg_offset = TIMER8_ASPEED_LOAD;
		mreg_offset = TIMER8_ASPEED_MATCH1;
		break;
	default:
		return -ERANGE;
	}

	writel(tload, priv->base + creg_offset);
	writel(tload, priv->base + lreg_offset);
	writel(tmatch, priv->base + mreg_offset);

	chan->period_ns = period_ns;
	chan->duty_ns = duty_ns;

	return 0;
}

static const struct pwm_ops pwm_fttmr010_ops = {
	.request	= pwm_fttmr010_request,
	.free		= pwm_fttmr010_free,
	.enable		= pwm_fttmr010_enable,
	.disable	= pwm_fttmr010_disable,
	.config		= pwm_fttmr010_config,
	.owner		= THIS_MODULE,
};

#ifdef CONFIG_OF
static const struct pwm_fttmr010_variant aspeed_variant = {
	.bits		= 32,
	.chan_min	= 5,
	.chan_max	= 8,
};

static const struct of_device_id pwm_fttmr010_matches[] = {
	{ .compatible = "aspeed,ast2400-timer", .data = &aspeed_variant },
	{ .compatible = "aspeed,ast2500-timer", .data = &aspeed_variant },
	{ },
};
MODULE_DEVICE_TABLE(of, pwm_fttmr010_matches);

static int pwm_fttmr010_parse_dt(struct pwm_fttmr010 *priv)
{
	struct device_node *np = priv->chip.dev->of_node;
	const struct of_device_id *match;
	struct property *prop;
	const __be32 *cur;
	u32 val;

	match = of_match_node(pwm_fttmr010_matches, np);
	if (!match)
		return -ENODEV;

	memcpy(&priv->variant, match->data, sizeof(priv->variant));

	of_property_for_each_u32(np, "fttmr010,pwm-outputs", prop, cur, val) {
		if (val < priv->variant.chan_min ||
		    val > priv->variant.chan_max) {
			dev_err(priv->chip.dev,
				"invalid channel index in fttmr010,pwm-outputs property\n");
			continue;
		}
		priv->variant.output_mask |= BIT(val);
	}

	return 0;
}
#else
static int pwm_fttmr010_parse_dt(struct pwm_fttmr010 *priv)
{
	return -ENODEV;
}
#endif

static int pwm_fttmr010_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct pwm_fttmr010 *priv;
	struct resource *res;
	ulong clk_rate;
	int ret;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->chip.dev = &pdev->dev;
	priv->chip.ops = &pwm_fttmr010_ops;
	priv->chip.base = -1;

	if (IS_ENABLED(CONFIG_OF) && pdev->dev.of_node) {
		ret = pwm_fttmr010_parse_dt(priv);
		if (ret)
			return ret;

		priv->chip.of_xlate = of_pwm_xlate_with_flags;
		priv->chip.of_pwm_n_cells = 3;
	} else {
		if (!pdev->dev.platform_data) {
			dev_err(&pdev->dev, "no platform data specified\n");
			return -EINVAL;
		}

		memcpy(&priv->variant, pdev->dev.platform_data,
		       sizeof(priv->variant));
	}

	priv->chip.npwm = priv->variant.chan_max + 1;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	priv->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	priv->clk = devm_clk_get(&pdev->dev, "PCLK");
	if (IS_ERR(priv->clk)) {
		dev_err(dev, "failed to get timer base clk\n");
		return PTR_ERR(priv->clk);
	}

	ret = clk_prepare_enable(priv->clk);
	if (ret < 0) {
		dev_err(dev, "failed to enable base clock\n");
		return ret;
	}

	clk_rate = clk_get_rate(priv->clk);
	priv->clk_tick_ns = NSEC_PER_SEC / clk_rate;

	platform_set_drvdata(pdev, priv);

	ret = pwmchip_add(&priv->chip);
	if (ret < 0) {
		dev_err(dev, "failed to register PWM chip\n");
		clk_disable_unprepare(priv->clk);
		return ret;
	}

	dev_dbg(dev, "clk at %lu\n", clk_rate);

	return 0;
}

static int pwm_fttmr010_remove(struct platform_device *pdev)
{
	struct pwm_fttmr010 *priv = platform_get_drvdata(pdev);

	pwmchip_remove(&priv->chip);
	clk_disable_unprepare(priv->clk);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int pwm_fttmr010_resume(struct device *dev)
{
	struct pwm_fttmr010 *priv = dev_get_drvdata(dev);
	struct pwm_chip *chip = &priv->chip;
	unsigned int i;

	for (i = priv->variant.chan_min; i < priv->variant.chan_max; i++) {
		struct pwm_device *pwm = &chip->pwms[i];
		struct pwm_fttmr010_chan *chan = pwm_get_chip_data(pwm);

		if (!chan)
			continue;

		if (chan->period_ns) {
			pwm_fttmr010_config(chip, pwm, chan->duty_ns,
					    chan->period_ns);
		}

		if (priv->disabled_mask & BIT(i))
			pwm_fttmr010_disable(chip, pwm);
		else
			pwm_fttmr010_enable(chip, pwm);
	}

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(pwm_fttmr010_pm_ops, NULL, pwm_fttmr010_resume);

static struct platform_driver pwm_fttmr010_driver = {
	.driver		= {
		.name	= "fttmr010-timer-pwm",
		.pm	= &pwm_fttmr010_pm_ops,
		.of_match_table = of_match_ptr(pwm_fttmr010_matches),
	},
	.probe		= pwm_fttmr010_probe,
	.remove		= pwm_fttmr010_remove,
};
module_platform_driver(pwm_fttmr010_driver);

MODULE_AUTHOR("Jae Hyun Yoo <jae.hyun.yoo@linux.intel.com>");
MODULE_DESCRIPTION("FTTMR010 PWM Driver for timer pulse outputs");
MODULE_LICENSE("GPL v2");
