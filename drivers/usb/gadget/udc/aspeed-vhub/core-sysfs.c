#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/usb/gadget.h>
#include <linux/platform_device.h>

#include "vhub.h"

#define DRIVER_NAME "usb"
#define DEVICE_NAME "usb_vhub"

static struct class *vHub_class;

static DEFINE_MUTEX(sysfs_lock);

/* Read handler for power save mode */
static ssize_t power_save_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    ssize_t ret;
    struct ast_vhub *vhub = dev_get_drvdata(dev);

    mutex_lock(&sysfs_lock);

    ret = sysfs_emit(buf, "%d\n", !ami_vhub_upstream_status(vhub));

    mutex_unlock(&sysfs_lock);

    return ret;
}

/* Write handler for power save mode */
static ssize_t power_save_mode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct ast_vhub *vhub = dev_get_drvdata(dev);
    int mode;

    if (kstrtoint(buf, 10, &mode))
        return -EINVAL;

    if (mode < 0 || mode > 1) {
        pr_err("%s: Invalid command: %d\n", __func__, mode);
        return -EINVAL;
    }

    mutex_lock(&sysfs_lock);

    // Execute the corresponding operation based on mode
    if (mode) {
        if (ami_vhub_upstream_disconnect(vhub)) {
            pr_err("%s: Failed to enable power save mode on vHub\n", __func__);
            mutex_unlock(&sysfs_lock);
            return -EIO;
        }
    } else {
        if (ami_vhub_upstream_connect(vhub)) {
            pr_err("%s: Failed to disable power save mode on vHub\n", __func__);
            mutex_unlock(&sysfs_lock);
            return -EIO;
        }
    }

    mutex_unlock(&sysfs_lock);

    return count;
}
static DEVICE_ATTR_RW(power_save_mode);

/* Attribute group */
static struct attribute *vHub_attrs[] = {
    &dev_attr_power_save_mode.attr,
    NULL,
};

static const struct attribute_group vHub_group = {
    .attrs = vHub_attrs,
};

const struct attribute_group *vHub_groups[] = {
    &vHub_group,
    NULL,
};

int ami_core_sysfs_register(struct ast_vhub *vhub)
{
    vHub_class = class_create(DRIVER_NAME);
    if (IS_ERR(vHub_class)) {
        pr_err("%s: Failed to create class\n", __func__);
        return PTR_ERR(vHub_class);
    }

    // Associate the device with the vhub instance
    struct device *vHub_device = device_create_with_groups(vHub_class, &vhub->pdev->dev, 0, vhub, vHub_groups, DEVICE_NAME);
    if (IS_ERR(vHub_device)) {
        pr_err("%s: Failed to create device with groups\n", __func__);
        class_destroy(vHub_class);
        return PTR_ERR(vHub_device);
    }

    dev_set_drvdata(vHub_device, vhub);

    dev_info(&vhub->pdev->dev, "AMI USB vHub Sysfs Driver registered successfully\n");
    return 0;
}


void ami_core_sysfs_unregister(struct ast_vhub *vhub)
{
    device_destroy(vHub_class, 0);
    class_destroy(vHub_class);

    dev_info(&vhub->pdev->dev, "AMI USB vHub Sysfs Driver unregister successfully\n");
}
