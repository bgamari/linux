/*
 * GPIO Multiplexer driver
 *
 * Copyright (C) 2015 Ben Gamari
 * Author:	Ben Gamari <ben@smart-cactus.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * Multiplexed GPIO virtual gpio_chip
 *
 * This driver is useful for instances where a one-hot group of GPIO outputs is
 * desired. The given N GPIO selector lines and one master line, the driver can
 * expose 2^N one-hot outputs. One common use of this is multiplexing SPI
 * chip-select lines.
 *
 */

#include <linux/mutex.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/platform_device.h>

struct gpio_mux {
	struct gpio_chip gpio_chip;
	struct mutex lock;
	int active;    /* index of active pin or -1 if none */
	int master;    /* GPIO number of master pin */
	int n_selects; /* number of select pins */
	int selects[];  /* GPIO numbers of select pins */
};

static int gpio_mux_get_direction(struct gpio_chip *chip, unsigned offset)
{
	return 0;
}

/* input not supported */
static int gpio_mux_set_direction_input(struct gpio_chip *chip, unsigned offset)
{
	return -ENOSYS;
}

static int gpio_mux_set_direction_output(struct gpio_chip *chip, unsigned offset, int value)
{
	return 0;
}

static int gpio_mux_get_value(struct gpio_chip *chip, unsigned offset)
{
	struct gpio_mux *mux = (struct gpio_mux *) chip;
	return offset == mux->active;
}

void gpio_mux_set_value(struct gpio_chip *chip, unsigned offset, int value)
{
	struct device *dev = chip->dev;
	struct gpio_mux *mux = (struct gpio_mux *) chip;
	int i;
	mutex_lock(&mux->lock);
	dev_warn(dev, "setting pin %d from %d to %d\n",
					 offset, mux->active, value);
	if (value && offset != mux->active) {
		gpio_set_value(mux->master, 0);
		if (mux->active != -1) {
			dev_warn(dev, "pin %d set high while pin %d already high\n",
							 offset, mux->active);
		}

		for (i = 0; i < mux->n_selects; i++) {
			gpio_set_value(mux->master, 1 & (offset >> (mux->n_selects - i)));
		}
		mux->active = offset;
		gpio_set_value(mux->master, 1);
	} else {
		mux->active = -1;
		gpio_set_value(mux->master, 0);
	}
	mutex_unlock(&mux->lock);
}

static const struct of_device_id gpio_mux_match[];

static int gpio_mux_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	struct gpio_mux *mux;
	int ret, i, n_gpios;
	uint32_t tmp, n_selects;

	printk(KERN_ERR "gpio-mux: found device\n");
	ret = of_property_read_u32(node, "#gpio-cells", &tmp);
	if (ret) {
		dev_err(dev, "expected #gpio-cells\n");
		return ret;
	}

	n_selects = of_gpio_named_count(node, "selector-gpios");
	if (n_selects < 0)
		return n_selects;
	n_gpios = 1 << n_selects;

	mux = devm_kzalloc(dev, sizeof(struct gpio_mux) + sizeof(int) * n_gpios, GFP_KERNEL);
	if (!mux) {
		dev_err(dev, "Memory alloc failed\n");
		return -ENOMEM;
	}

	mutex_init(&mux->lock);

	mux->master = of_get_named_gpio(node, "master-gpio", 0);
	if (mux->master < 0) {
		dev_err(dev, "failed to get master GPIO\n");
		return -ENODEV;
	}

	ret = gpio_request(mux->master, "master");
	if (ret < 0) {
		dev_err(dev, "failed to request gpio %d: %d\n",
						mux->master, ret);
		kfree(name);
		goto out;
	}

	ret = gpio_direction_output(mux->master, 0);
	if (ret < 0) {
		dev_err(dev, "failed to set gpio %d to output: %d\n",
						mux->master, ret);
		gpio_free(mux->master);
		kfree(name);
		goto out;
	}

	for (i=0; i < n_selects; i++) {
		char* name;
		mux->selects[i] = of_get_named_gpio(node, "selector-gpios", i);
		if (mux->selects[i] < 0) {
			dev_err(dev, "failed to get bit %d selector GPIO\n", i);
			ret = -ENODEV;
			goto out;
		}

		name = kasprintf(GFP_KERNEL, "mux-select%d", i);
		if (!name) {
			ret = -ENOMEM;
			goto out;
		}

		ret = gpio_request(mux->selects[i], name);
		if (ret < 0) {
			dev_err(dev, "failed to request gpio %d: %d\n",
							mux->selects[i], ret);
			kfree(name);
			goto out;
		}

		ret = gpio_direction_output(mux->selects[i], 0);
		if (ret < 0) {
			dev_err(dev, "failed to set gpio %d to output: %d\n",
							mux->selects[i], ret);
			gpio_free(mux->selects[i]);
			kfree(name);
			goto out;
		}
	}

	mux->gpio_chip.dev = dev;
	mux->gpio_chip.label = "GPIO multiplexer";
	mux->gpio_chip.direction_output = gpio_mux_set_direction_output;
	mux->gpio_chip.direction_input = gpio_mux_set_direction_input;
	mux->gpio_chip.get_direction = gpio_mux_get_direction;
	mux->gpio_chip.get = gpio_mux_get_value;
	mux->gpio_chip.set = gpio_mux_set_value;
	mux->gpio_chip.base = -1;
	mux->gpio_chip.ngpio = n_gpios;

	ret = gpiochip_add(&mux->gpio_chip);
	if (ret) {
		dev_err(dev, "Failed to register ADC mux: %d\n", ret);
		goto out;
	}

	platform_set_drvdata(pdev, mux);

out:
	return ret;
}

static int gpio_mux_remove(struct platform_device *pdev)
{
	struct gpio_mux *mux = platform_get_drvdata(pdev);

	gpiochip_remove(&mux->gpio_chip);
	kfree(mux);
	return 0;
}

static const struct of_device_id gpio_mux_match[] = {
	{ .compatible = "gpio,multiplexed", },
	{ },
};
MODULE_DEVICE_TABLE(of, gpio_mux_match);

static struct platform_driver gpio_mux_driver = {
	.probe = gpio_mux_probe,
	.remove = gpio_mux_remove,
	.driver = {
		.name = "gpio_mux",
		.of_match_table = of_match_ptr(gpio_mux_match),
	},
};
module_platform_driver(gpio_mux_driver);

MODULE_AUTHOR("Ben Gamari <ben@smart-cactus.org>");
MODULE_DESCRIPTION("Virtual GPIO chip for multiplexed GPIO groups");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:gpio-mux");
