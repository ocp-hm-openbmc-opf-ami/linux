// SPDX-License-Identifier: GPL-2.0
/* Copyright (C) 2023 Intel Corporation.*/
#include <linux/module.h>

#include <linux/i3c/device.h>

#define I3C_DEBUG_MINORS 4

struct i3c_debug {
	struct i3c_device *i3c;
	struct device *dev;
	int id;
};

static struct class *i3c_debug_class;
static dev_t i3c_debug_devt;
static DEFINE_IDA(i3c_debug_ida);

static struct i3c_debug *i3c_debug_alloc(struct i3c_device *i3c)
{
	struct device *dev = i3cdev_to_dev(i3c);
	struct i3c_debug *priv;
	int id;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return ERR_PTR(-ENOMEM);

	id = ida_alloc(&i3c_debug_ida, GFP_KERNEL);
	if (id < 0) {
		dev_err(dev, "No minor number available!\n");
		return ERR_PTR(id);
	}

	priv->id = id;
	priv->i3c = i3c;

	return priv;
}

static int i3c_debug_init(struct i3c_driver *drv)
{
	int ret;

	ret = alloc_chrdev_region(&i3c_debug_devt, 0, I3C_DEBUG_MINORS, "i3c-debug");
	if (ret)
		goto out;

	i3c_debug_class = class_create(THIS_MODULE, "i3c-debug");
	if (IS_ERR(i3c_debug_class)) {
		ret = PTR_ERR(i3c_debug_class);
		goto out_unreg_chrdev;
	}

	i3c_driver_register(drv);

	return 0;

out_unreg_chrdev:
	unregister_chrdev_region(i3c_debug_devt, I3C_DEBUG_MINORS);
out:
	pr_err("i3c_debug: driver initialisation failed\n");
	return ret;
}

static void i3c_debug_free(struct i3c_driver *drv)
{
	i3c_driver_unregister(drv);
	class_destroy(i3c_debug_class);
	unregister_chrdev_region(i3c_debug_devt, I3C_DEBUG_MINORS);
}

static int i3c_debug_probe(struct i3c_device *i3cdev)
{
	struct device *dev = i3cdev_to_dev(i3cdev);
	struct i3c_debug *priv;

	priv = i3c_debug_alloc(i3cdev);
	if (IS_ERR(priv))
		return PTR_ERR(priv);

	priv->dev = device_create(i3c_debug_class, dev,
				  MKDEV(MAJOR(i3c_debug_devt), priv->id), NULL,
				  "i3c-debug-%d", priv->id);
	if (IS_ERR(priv->dev))
		return PTR_ERR(priv->dev);

	i3cdev_set_drvdata(i3cdev, priv);

	dev_info(dev, "MIPI Debug for I3C driver probing\n");

	return 0;
}

static void i3c_debug_remove(struct i3c_device *i3cdev)
{
	struct i3c_debug *priv = i3cdev_get_drvdata(i3cdev);

	device_destroy(i3c_debug_class, MKDEV(MAJOR(i3c_debug_devt), priv->id));
	ida_free(&i3c_debug_ida, priv->id);
}

static const struct i3c_device_id i3c_debug_ids[] = {
	/* Use manufacturer ID, PART ID for now */
	I3C_DEVICE(0x105, 0x128, 0x0),
	{},
};

static struct i3c_driver i3c_debug_drv = {
	.driver.name = "i3c-debug",
	.id_table = i3c_debug_ids,
	.probe = i3c_debug_probe,
	.remove = i3c_debug_remove,
};

module_driver(i3c_debug_drv, i3c_debug_init, i3c_debug_free);
MODULE_AUTHOR("Oleksandr Shulzhenko <oleksandr.shulzhenko.viktorovych@intel.com>");
MODULE_AUTHOR("Zbigniew Lukwinski <zbigniew.lukwinski@linux.intel.com>");
MODULE_AUTHOR("Sumanth Bhat <sumanth.bhat@intel.com>");
MODULE_DESCRIPTION("I3C Debug driver to support MIPI Debug for I3C specification");
MODULE_LICENSE("GPL");
